// SPDX-License-Identifier: Apache-2.0
//! Testbench of `tx_path` (T1.4b): two unrelated clocks, frames in on the
//! system clock, a transmitter on the MII clock, timestamps checked
//! against the instant the SFD is put on the MII.
//!
//! Time base: one simulation tick is 1 ns. The system clock has a period
//! of 10 ticks (100 MHz). The MII clock is 40 ticks (25 MHz, 4:1) with
//! several phases, or 37 ticks (an unrelated ratio, whose phase against
//! the system clock drifts through every value). The NTP clock is a linear
//! function of the tick, presented to the unit as a register of the system
//! clock would (the value of the previous edge).
//!
//! The transmitter is a behavioural copy of `mii_tx` written for any clock
//! (`mii_tx` itself is fixed to `ClkTx`, whose simulated period is the
//! default clock's); `real_mii_tx_one_to_one` runs the real `MiiTx` and the
//! real `NtpClock` at a 1:1 ratio and checks that the copy behaves the same.
use std::cell::RefCell;
use std::rc::Rc;

use ntp_fpga::clock::NtpClock;
use ntp_fpga::clocks::ClkTx;
use ntp_fpga::mii_tx::MiiTx;
use ntp_fpga::stream::{FrameByte, Ts};
use ntp_fpga::tx_path::{TxPathG, TX_STAMP_BIAS, TX_STAMP_BIAS_NS};
use txhdl::comp::{chan, join2, now, signal, Clock, DefaultClock, In, Out, Running, Rx, Tx, Unit};
use txhdl::types::{Bit, U};

// ---------------------------------------------------------------- clocks

struct Sys;
impl Clock for Sys {
    const NAME: &'static str = "clk_sys_t";
    const PERIOD: u64 = 10;
}

struct Mii<const P: u64, const PH: u64>;
impl<const P: u64, const PH: u64> Clock for Mii<P, PH> {
    const NAME: &'static str = "clk_tx_t";
    const PERIOD: u64 = P;
    const PHASE: u64 = PH;
}

const TSYS: u64 = 10;

/// The MII lines of a transmitter: txd, tx_en, sfd, underrun.
type MiiOut<C> = (Out<U<4>, C>, Out<Bit, C>, Out<Bit, C>, Out<Bit, C>);
type MiiIn<'a, C> = (
    &'a In<U<4>, C>,
    &'a In<Bit, C>,
    &'a In<Bit, C>,
    &'a In<Bit, C>,
);

type Dut<C> = TxPathG<12, 4096, 13, Sys, C>;

// ------------------------------------------------------------ NTP time

const BASE: u128 = 0xE000_0000u128 << 64;

/// Accumulator (32.64) at tick `t` (1 tick = 1 ns).
fn acc(t: u64) -> u128 {
    BASE + ((t as u128) << 64) / 1_000_000_000
}

fn ntp(t: u64) -> u64 {
    (acc(t) >> 32) as u64
}

fn to_ns(units: i64) -> f64 {
    units as f64 * 1e9 / 4294967296.0
}

// ------------------------------------------------- behavioural mii_tx

const IDLE: u8 = 0;
const PRE: u8 = 1;
const LO: u8 = 2;
const HI: u8 = 3;
const DRAIN: u8 = 4;

/// Cycle-accurate copy of `mii_tx` (same states, same ready, same
/// registered outputs), on any clock.
struct Phy<C: Clock>(std::marker::PhantomData<C>);

impl<C: Clock> Phy<C> {
    async fn run(self, frames: Rx<FrameByte, C>, (txd, tx_en, sfd, underrun): MiiOut<C>) {
        let (mut st, mut cnt, mut len) = (IDLE, 23u32, 0u32);
        let (mut byte, mut blast, mut berr) = (0u8, false, false);
        // Registered outputs: (txd, en, sfd, und).
        let mut q = (0u8, false, false, false);
        loop {
            C::rising().await;
            let mut n = (q.0, q.1, false, false);
            let want_first = st == IDLE && cnt == 0;
            let want_next = st == HI && !blast && len < 1518;
            let take = want_first || want_next || st == DRAIN;
            let got = if take { frames.recv() } else { None };
            if (st == IDLE || st == DRAIN) && cnt > 0 {
                cnt -= 1;
            }
            match st {
                IDLE => {
                    if let (true, Some(b)) = (want_first, got) {
                        st = PRE;
                        cnt = 1;
                        len = 1;
                        byte = b.data.raw() as u8;
                        blast = b.last.to_bool();
                        berr = b.err.to_bool();
                        n.0 = 5;
                        n.1 = true;
                    }
                }
                PRE => {
                    if cnt < 16 {
                        n.0 = 5;
                        if cnt == 15 {
                            n.0 = 0xD;
                            n.2 = true;
                        }
                        cnt += 1;
                    } else {
                        st = LO;
                        n.0 = byte & 15;
                    }
                }
                LO => {
                    if blast && berr {
                        st = IDLE;
                        cnt = 23;
                        n.0 = 0;
                        n.1 = false;
                    } else {
                        st = HI;
                        n.0 = byte >> 4;
                    }
                }
                HI => {
                    if blast {
                        st = IDLE;
                        cnt = 23;
                        n.0 = 0;
                        n.1 = false;
                    } else if let (true, Some(b)) = (want_next, got) {
                        st = LO;
                        len += 1;
                        byte = b.data.raw() as u8;
                        blast = b.last.to_bool();
                        berr = b.err.to_bool();
                        n.0 = byte & 15;
                    } else {
                        st = DRAIN;
                        cnt = 23;
                        n.0 = 0;
                        n.1 = false;
                        n.3 = true;
                    }
                }
                _ => {
                    if let Some(b) = got {
                        if b.last.to_bool() {
                            st = IDLE;
                        }
                    }
                }
            }
            // Outputs show the registers as the previous edge left them,
            // as `mii_tx` does.
            txd.set(U::<4>::from(q.0));
            tx_en.set(Bit::from_bool(q.1));
            sfd.set(Bit::from_bool(q.2));
            underrun.set(Bit::from_bool(q.3));
            q = n;
        }
    }
}

// --------------------------------------------------------- harness

/// One input byte and the number of idle system cycles before it.
type Item = (FrameByte, usize);

fn frame(data: &[u8], err: bool) -> Vec<Item> {
    let n = data.len();
    data.iter()
        .enumerate()
        .map(|(i, &b)| (FrameByte::new(b, i + 1 == n, err && i + 1 == n), 0))
        .collect()
}

fn pattern(n: usize, seed: u8) -> Vec<u8> {
    (0..n)
        .map(|i| (i as u8).wrapping_mul(7).wrapping_add(seed))
        .collect()
}

#[derive(Default, Debug)]
struct Log {
    /// Tick at which each input item was put in the input channel.
    offered: Vec<u64>,
    /// Ticks at which the feeder had a byte but the channel was full.
    stalls: u64,
    stamps: Vec<(u64, u64)>,
    /// MII lines after each edge of the MII clock: (tick, txd, en, sfd, und).
    mii: Vec<(u64, u8, bool, bool, bool)>,
}

/// Feeds `items` on clock `S`.
async fn feeder<S: Clock>(items: Vec<Item>, tx: Tx<FrameByte, S>, log: Rc<RefCell<Log>>) {
    let mut i = 0;
    let mut wait = items.first().map_or(0, |x| x.1);
    loop {
        S::rising().await;
        if i < items.len() {
            if wait > 0 {
                wait -= 1;
            } else if tx.ready().to_bool() {
                tx.send(items[i].0);
                log.borrow_mut().offered.push(now());
                i += 1;
                wait = items.get(i).map_or(0, |x| x.1);
            } else {
                log.borrow_mut().stalls += 1;
            }
        }
    }
}

/// Takes stamps on clock `S` when `allow(tick)`.
async fn sink<S: Clock>(rx: Rx<Ts, S>, allow: fn(u64) -> bool, log: Rc<RefCell<Log>>) {
    loop {
        S::rising().await;
        if allow(now()) {
            if let Some(v) = rx.recv() {
                log.borrow_mut().stamps.push((now(), v.raw() as u64));
            }
        }
    }
}

fn always(_: u64) -> bool {
    true
}

fn observe<C: Clock>(log: &Rc<RefCell<Log>>, (txd, en, sfd, und): MiiIn<'_, C>, t: u64) {
    if C::rising_at(t) {
        log.borrow_mut().mii.push((
            t,
            txd.get().raw() as u8,
            en.get().to_bool(),
            sfd.get().to_bool(),
            und.get().to_bool(),
        ));
    }
}

/// Runs the unit with the behavioural transmitter for `ticks` ticks.
fn run<C: Clock>(items: &[Item], ticks: u64, allow: fn(u64) -> bool) -> Log {
    let log = Rc::new(RefCell::new(Log::default()));
    let (in_tx, in_rx) = chan::<FrameByte, Sys>();
    let (time_o, time_i) = signal::<U<96>, Sys>();
    let (sfd_o, sfd_i) = signal::<Bit, C>();
    let (und_o, und_i) = signal::<Bit, C>();
    let (st_tx, st_rx) = chan::<Ts, Sys>();
    let (tf_tx, tf_rx) = chan::<FrameByte, C>();
    let (txd_o, txd) = signal::<U<4>, C>();
    let (en_o, en) = signal::<Bit, C>();
    let (sfd_w, sfd_r) = (sfd_o, sfd_i.clone());
    let (und_w, und_r) = (und_o, und_i.clone());
    let mut dut = Dut::<C>::default();
    let phy = Phy::<C>(std::marker::PhantomData);
    // The transmitter runs first in each step, so the unit reads its
    // outputs as a flop reads another flop's output.
    let mut sim = Running::new(join2(
        join2(
            phy.run(tf_rx, (txd_o, en_o, sfd_w, und_w)),
            dut.run((in_rx, time_i, sfd_i, und_i), (st_tx, tf_tx)),
        ),
        join2(
            feeder::<Sys>(items.to_vec(), in_tx, log.clone()),
            sink::<Sys>(st_rx, allow, log.clone()),
        ),
    ));
    // Ticks are absolute (`now()` keeps counting across simulations).
    for _ in 0..ticks {
        let t = now();
        time_o.set(U::<96>::from(acc(t.saturating_sub(TSYS))));
        sim.step();
        observe::<C>(&log, (&txd, &en, &sfd_r, &und_r), t);
    }
    drop(sim);
    Rc::try_unwrap(log).unwrap().into_inner()
}

// ------------------------------------------------------------ decoding

/// Runs of consecutive `en` MII cycles: (first index, end index exclusive).
fn bursts(m: &[(u64, u8, bool, bool, bool)]) -> Vec<(usize, usize)> {
    let mut v = Vec::new();
    let mut i = 0;
    while i < m.len() {
        if m[i].2 {
            let s = i;
            while i < m.len() && m[i].2 {
                i += 1;
            }
            v.push((s, i));
        } else {
            i += 1;
        }
    }
    v
}

/// Bytes of a burst after preamble and SFD; asserts the preamble.
fn burst_bytes(m: &[(u64, u8, bool, bool, bool)], (s, e): (usize, usize)) -> Vec<u8> {
    let nib: Vec<u8> = m[s..e].iter().map(|c| c.1).collect();
    let mut head = vec![5u8; 15];
    head.push(0xD);
    assert_eq!(&nib[..16], &head[..], "preamble and SFD");
    assert!(m[s + 15].3, "sfd strobe with the 0xD nibble");
    nib[16..]
        .chunks(2)
        .map(|c| c[0] | (c.get(1).unwrap_or(&0) << 4))
        .collect()
}

/// Tick at which the SFD went onto the MII: the MII edge whose registers
/// put 0xD on txd. The lines are observed one edge after they changed
/// (outputs show the previous edge's registers).
fn sfd_ticks<C: Clock>(m: &[(u64, u8, bool, bool, bool)]) -> Vec<u64> {
    m.iter().filter(|c| c.3).map(|c| c.0 - C::PERIOD).collect()
}

/// The common checks: frames out = `want`, one burst each, no gap, no
/// underrun, one stamp each in order, stamp error within bounds (ns).
fn check<C: Clock>(log: &Log, want: &[Vec<u8>], err_lo: f64, err_hi: f64) -> Vec<f64> {
    let b = bursts(&log.mii);
    assert_eq!(b.len(), want.len(), "one burst per good frame");
    for (bb, w) in b.iter().zip(want) {
        assert_eq!(&burst_bytes(&log.mii, *bb), w, "frame comes out identical");
        assert_eq!(
            bb.1 - bb.0,
            16 + 2 * w.len(),
            "no gap: tx_en for exactly 16 + 2 len cycles"
        );
    }
    assert!(log.mii.iter().all(|c| !c.4), "no underrun");
    let sfds = sfd_ticks::<C>(&log.mii);
    assert_eq!(sfds.len(), want.len());
    assert_eq!(log.stamps.len(), want.len(), "one stamp per frame");
    let mut errs = Vec::new();
    for (k, (&e0, &(ts, st))) in sfds.iter().zip(&log.stamps).enumerate() {
        assert!(ts > e0, "stamp after its SFD");
        if k + 1 < sfds.len() && std::env::var("TXP_DEBUG").is_ok() {
            println!("k={k} sfd={e0} stamp_at={ts} next_sfd={}", sfds[k + 1]);
        }
        let e = to_ns(st as i64 - ntp(e0) as i64);
        assert!(
            e >= err_lo && e <= err_hi,
            "stamp {k}: error {e:.2} ns not in [{err_lo}, {err_hi}]"
        );
        errs.push(e);
    }
    errs
}

/// Bounds of the stamp error for a MII period of `tc` ns: the latency is
/// `tc + delta + 10` with `delta` in (0, 10] (no metastability here), the
/// bias assumes 40 + 20; NTP units round to 0.24 ns.
fn bounds(tc: u64) -> (f64, f64) {
    let shift = tc as f64 - 40.0;
    (shift - 10.0 - 0.5, shift + 0.5)
}

// ---------------------------------------------------------------- tests

fn sizes_identical<C: Clock>() {
    let want: Vec<Vec<u8>> = [64usize, 100, 1518]
        .iter()
        .map(|&n| pattern(n, n as u8))
        .collect();
    let mut items = Vec::new();
    for w in &want {
        items.extend(frame(w, false));
    }
    let log = run::<C>(&items, 160_000, always);
    let (lo, hi) = bounds(C::PERIOD);
    check::<C>(&log, &want, lo, hi);
}

#[test]
fn frames_of_64_100_1518_bytes_come_out_identical_4_to_1() {
    sizes_identical::<Mii<40, 0>>();
    sizes_identical::<Mii<40, 3>>();
    sizes_identical::<Mii<40, 5>>();
    sizes_identical::<Mii<40, 7>>();
}

#[test]
fn frames_of_64_100_1518_bytes_come_out_identical_other_ratio() {
    sizes_identical::<Mii<37, 2>>();
    sizes_identical::<Mii<23, 0>>();
}

#[test]
fn input_with_gaps_comes_out_gap_free() {
    let want: Vec<Vec<u8>> = (0..6).map(|k| pattern(64 + 37 * k, k as u8)).collect();
    let mut items = Vec::new();
    let mut r = 12345u32;
    for w in &want {
        for mut it in frame(w, false) {
            r = r.wrapping_mul(1103515245).wrapping_add(12345);
            it.1 = ((r >> 16) % 13) as usize; // 0..12 idle cycles before each byte
            items.push(it);
        }
    }
    let log = run::<Mii<40, 1>>(&items, 200_000, always);
    let (lo, hi) = bounds(40);
    check::<Mii<40, 1>>(&log, &want, lo, hi);
}

#[test]
fn aborted_frame_is_discarded_and_later_frames_are_fine() {
    let a = pattern(64, 1);
    let bad = pattern(80, 2);
    let c = pattern(70, 3);
    let bad2 = pattern(3, 9);
    let mut items = frame(&a, false);
    items.extend(frame(&bad, true));
    items.extend(frame(&c, false));
    items.extend(frame(&bad2, true));
    items.extend(frame(&a, false));
    let log = run::<Mii<40, 0>>(&items, 60_000, always);
    let (lo, hi) = bounds(40);
    check::<Mii<40, 0>>(&log, &[a.clone(), c, a], lo, hi);
}

#[test]
fn frames_back_to_back_keep_line_rate() {
    let want: Vec<Vec<u8>> = (0..5).map(|k| pattern(64, k as u8)).collect();
    let mut items = Vec::new();
    for w in &want {
        items.extend(frame(w, false));
    }
    let log = run::<Mii<40, 0>>(&items, 80_000, always);
    let (lo, hi) = bounds(40);
    check::<Mii<40, 0>>(&log, &want, lo, hi);
    let b = bursts(&log.mii);
    for w in b.windows(2) {
        assert_eq!(
            w[1].0 - w[0].1,
            24,
            "queued frames follow after the minimum gap (line rate)"
        );
    }
}

#[test]
fn buffer_full_back_pressures_the_system_side() {
    // Five maximum frames offered at 8 bits per 10 ns: far faster than
    // the wire, so the buffer (4096 bytes) fills and the input is held.
    let want: Vec<Vec<u8>> = (0..5).map(|k| pattern(1518, k as u8)).collect();
    let mut items = Vec::new();
    for w in &want {
        items.extend(frame(w, false));
    }
    let log = run::<Mii<40, 0>>(&items, 700_000, always);
    assert!(log.stalls > 1000, "the input was held back");
    // While held, never more than the memory: checked by the unit's own
    // check! at every edge; here, nothing was lost or overwritten.
    let (lo, hi) = bounds(40);
    check::<Mii<40, 0>>(&log, &want, lo, hi);
}

fn slow_sink(t: u64) -> bool {
    // Accept stamps only during one microsecond out of five.
    (t / 1000) % 5 == 4
}

#[test]
fn stalled_stamp_consumer_holds_the_transmitter_and_loses_nothing() {
    let want: Vec<Vec<u8>> = (0..6).map(|k| pattern(64, k as u8)).collect();
    let mut items = Vec::new();
    for w in &want {
        items.extend(frame(w, false));
    }
    let log = run::<Mii<40, 0>>(&items, 120_000, slow_sink);
    let (lo, hi) = bounds(40);
    check::<Mii<40, 0>>(&log, &want, lo, hi);
}

#[test]
fn stamp_error_within_bound_over_many_phases() {
    // 37 ns against 10 ns: the phase of the SFD against clk_sys takes
    // every value over the frames.
    let want: Vec<Vec<u8>> = (0..40)
        .map(|k| pattern(64 + (k * 13) % 50, k as u8))
        .collect();
    let mut items = Vec::new();
    for (k, w) in want.iter().enumerate() {
        let mut f = frame(w, false);
        f[0].1 = 200 + 7 * k; // idle between frames: varied start phase
        items.extend(f);
    }
    let log = run::<Mii<37, 0>>(&items, 600_000, always);
    let (lo, hi) = bounds(37);
    let errs = check::<Mii<37, 0>>(&log, &want, lo, hi);
    let min = errs.iter().cloned().fold(f64::MAX, f64::min);
    let max = errs.iter().cloned().fold(f64::MIN, f64::max);
    assert!(
        max - min > 7.0,
        "phases were explored: spread {min:.2}..{max:.2} ns"
    );
    // 4:1, exact 40 ns: error in (-10, 0] ns (bias 60 ns).
    let (lo, hi) = bounds(40);
    assert!(lo >= -10.5 && hi <= 0.5);
    println!("37 ns MII clock: stamp error {min:.2} .. {max:.2} ns");
}

#[test]
fn bias_constant_matches_its_definition() {
    assert_eq!(TX_STAMP_BIAS_NS, 60);
    assert_eq!(TX_STAMP_BIAS, 258); // 60e-9 * 2^32 = 257.7
}

#[test]
fn underrun_truncated_frame_gets_no_stamp() {
    // A 1600-byte frame is truncated by the transmitter at 1518 bytes
    // (underrun pulse): no stamp for it; the frames around it are fine.
    let a = pattern(64, 1);
    let long = pattern(1600, 2);
    let c = pattern(100, 3);
    let mut items = frame(&a, false);
    items.extend(frame(&long, false));
    items.extend(frame(&c, false));
    let log = run::<Mii<40, 0>>(&items, 200_000, always);
    let b = bursts(&log.mii);
    assert_eq!(b.len(), 3);
    assert_eq!(log.mii.iter().filter(|c| c.4).count(), 1, "one underrun");
    assert_eq!(burst_bytes(&log.mii, b[0]), a);
    assert_eq!(burst_bytes(&log.mii, b[1]), long[..1518].to_vec());
    assert_eq!(burst_bytes(&log.mii, b[2]), c);
    assert_eq!(log.stamps.len(), 2, "no stamp for the truncated frame");
    let sfds = sfd_ticks::<Mii<40, 0>>(&log.mii);
    let (lo, hi) = bounds(40);
    for (e0, (_, st)) in [sfds[0], sfds[2]].iter().zip(&log.stamps) {
        let e = to_ns(*st as i64 - ntp(*e0) as i64);
        assert!(e >= lo && e <= hi, "error {e}");
    }
}

/// Latency from the last byte offered on `frames` to the SFD on the MII,
/// for isolated frames (link idle), 64 and 1518 bytes, several phases.
fn latency<C: Clock>() -> (i64, i64) {
    let mut lat = Vec::new();
    for (len, n) in [(64usize, 20usize), (1518, 4)] {
        let mut items = Vec::new();
        let mut lasts = Vec::new();
        for k in 0..n {
            let mut f = frame(&pattern(len, k as u8), false);
            // sys cycles: the link is idle again before each
            f[0].1 = (16 + 2 * len) * 4 + 300 + 3 * k;
            lasts.push(items.len() + len - 1);
            items.extend(f);
        }
        let log = run::<C>(
            &items,
            n as u64 * (2 * (16 + 2 * len as u64) * 40 + 6000),
            always,
        );
        let sfds = sfd_ticks::<C>(&log.mii);
        assert_eq!(sfds.len(), n, "{len}: {} frames out", sfds.len());
        for (i, &l) in lasts.iter().enumerate() {
            lat.push(sfds[i] as i64 - log.offered[l] as i64);
        }
    }
    (*lat.iter().min().unwrap(), *lat.iter().max().unwrap())
}

#[test]
fn latency_of_an_isolated_frame() {
    // The bytes of a released frame wait for nothing (the whole frame is
    // buffered), so the latency does not depend on the length.
    for (name, (min, max)) in [
        ("40/0", latency::<Mii<40, 0>>()),
        ("40/3", latency::<Mii<40, 3>>()),
        ("40/5", latency::<Mii<40, 5>>()),
        ("40/9", latency::<Mii<40, 9>>()),
    ] {
        println!("MII {name}: last byte offered -> SFD on the MII: {min} .. {max} ns");
        // Offered = accepted - 10 ns; accepted -> SFD is (770, 810] ns.
        assert!(min > 780 && max <= 820, "{name}: {min} .. {max}");
    }
    let (min, max) = latency::<Mii<41, 0>>();
    println!("MII 41 ns: {min} .. {max} ns");
    assert!(min > 20 + 19 * 41 && max <= 20 + 20 * 41, "{min} .. {max}");
}

// ------------------------------------------------- real mii_tx, 1:1

#[test]
fn real_mii_tx_one_to_one() {
    // DefaultClock and ClkTx have the same period in the simulation:
    // this checks the unit against the real transmitter and the real
    // NTP clock, and the behavioural copy against the real transmitter.
    let want: Vec<Vec<u8>> = vec![
        pattern(64, 1),
        pattern(100, 2),
        pattern(1518, 3),
        pattern(60, 4),
    ];
    let mut items = Vec::new();
    for (k, w) in want.iter().enumerate() {
        let mut f = frame(w, false);
        f[0].1 = 50 * k;
        items.extend(f);
    }
    items.splice(64..64, frame(&pattern(70, 7), true)); // an aborted frame after the first

    let log = Rc::new(RefCell::new(Log::default()));
    let (in_tx, in_rx) = chan::<FrameByte, DefaultClock>();
    let (time_o, time_i) = signal::<U<96>, DefaultClock>();
    let time_r = time_i.clone();
    let (sfd_o, sfd_i) = signal::<Bit, ClkTx>();
    let (und_o, und_i) = signal::<Bit, ClkTx>();
    let (st_tx, st_rx) = chan::<Ts, DefaultClock>();
    let (tf_tx, tf_rx) = chan::<FrameByte, ClkTx>();
    let (txd_o, txd) = signal::<U<4>, ClkTx>();
    let (en_o, en) = signal::<Bit, ClkTx>();
    let (sfd_r, und_r) = (sfd_i.clone(), und_i.clone());
    let mut clk = NtpClock::default();
    let mut mii = MiiTx::default();
    let mut dut = ntp_fpga::tx_path::TxPathG::<12, 4096, 13, DefaultClock, ClkTx>::default();
    let mut sim = Running::new(join2(
        join2(
            clk.run((), time_o),
            mii.run(tf_rx, (txd_o, en_o, sfd_o, und_o)),
        ),
        join2(
            dut.run((in_rx, time_i, sfd_i, und_i), (st_tx, tf_tx)),
            join2(
                feeder::<DefaultClock>(items.clone(), in_tx, log.clone()),
                sink::<DefaultClock>(st_rx, always, log.clone()),
            ),
        ),
    ));
    let mut times = std::collections::HashMap::new();
    for _ in 0..40_000u64 {
        let t = now();
        sim.step();
        times.insert(t, time_r.get().raw());
        observe::<ClkTx>(&log, (&txd, &en, &sfd_r, &und_r), t);
    }
    drop(sim);
    let real = Rc::try_unwrap(log).unwrap().into_inner();

    // The same through the behavioural copy, clocked like ClkTx.
    type Same = Mii<2, 0>;
    let copy = run_default::<Same>(&items, 40_000);
    let (ra, rb) = (real.mii[0].0, copy.mii[0].0);
    let a: Vec<_> = real
        .mii
        .iter()
        .map(|c| (c.0 - ra, c.1, c.2, c.3, c.4))
        .collect();
    let b: Vec<_> = copy
        .mii
        .iter()
        .map(|c| (c.0 - rb, c.1, c.2, c.3, c.4))
        .collect();
    if let Some(i) = (0..a.len().min(b.len())).find(|&i| a[i] != b[i]) {
        panic!(
            "copy differs from mii_tx at {i}: real {:?} copy {:?}",
            &a[i.saturating_sub(3)..i + 3],
            &b[i.saturating_sub(3)..i + 3]
        );
    }
    assert_eq!(a.len(), b.len());

    let bb = bursts(&real.mii);
    assert_eq!(bb.len(), want.len());
    for (x, w) in bb.iter().zip(&want) {
        assert_eq!(&burst_bytes(&real.mii, *x), w);
    }
    assert_eq!(
        real.stamps.len(),
        want.len(),
        "one stamp per frame, none for the aborted one"
    );
    // Stamp error against the NTP clock's own time at the SFD edge. With
    // a 10 ns MII clock the latency is 30 ns shorter than the bias assumes.
    let sfds = sfd_ticks::<ClkTx>(&real.mii);
    for (k, (&e0, &(_, st))) in sfds.iter().zip(&real.stamps).enumerate() {
        // The clock's value after edge e0: what it reads at that instant.
        // The value the clock shows one edge later is the accumulator of
        // edge e0 itself (the unit reads a register: previous edge's value).
        let ref_t = (times[&(e0 + 2)] >> 32) as u64;
        let e = to_ns(st as i64 - ref_t as i64);
        println!("real 1:1: stamp {k} error {e:.2} ns");
        assert!((-41.0..=-29.0).contains(&e), "stamp {k}: {e}");
    }
}

/// `run` on the default clock (period 2) for the comparison with `mii_tx`.
fn run_default<C: Clock>(items: &[Item], ticks: u64) -> Log {
    let log = Rc::new(RefCell::new(Log::default()));
    let (in_tx, in_rx) = chan::<FrameByte, DefaultClock>();
    let (time_o, time_i) = signal::<U<96>, DefaultClock>();
    let (sfd_o, sfd_i) = signal::<Bit, C>();
    let (und_o, und_i) = signal::<Bit, C>();
    let (st_tx, st_rx) = chan::<Ts, DefaultClock>();
    let (tf_tx, tf_rx) = chan::<FrameByte, C>();
    let (txd_o, txd) = signal::<U<4>, C>();
    let (en_o, en) = signal::<Bit, C>();
    let (sfd_r, und_r) = (sfd_i.clone(), und_i.clone());
    let mut clk = NtpClock::default();
    let mut dut = TxPathG::<12, 4096, 13, DefaultClock, C>::default();
    let phy = Phy::<C>(std::marker::PhantomData);
    let mut sim = Running::new(join2(
        join2(
            clk.run((), time_o),
            phy.run(tf_rx, (txd_o, en_o, sfd_o, und_o)),
        ),
        join2(
            dut.run((in_rx, time_i, sfd_i, und_i), (st_tx, tf_tx)),
            join2(
                feeder::<DefaultClock>(items.to_vec(), in_tx, log.clone()),
                sink::<DefaultClock>(st_rx, always, log.clone()),
            ),
        ),
    ));
    for _ in 0..ticks {
        let t = now();
        sim.step();
        observe::<C>(&log, (&txd, &en, &sfd_r, &und_r), t);
    }
    drop(sim);
    Rc::try_unwrap(log).unwrap().into_inner()
}
