// SPDX-License-Identifier: Apache-2.0
//! Two-clock tests of the receive path crossing (T1.4a).
//!
//! The path is run with its two clocks as types of this file, so that the
//! ratio and the phase can be chosen: `clk_sys` is [`Sys`] (20 ticks =
//! 10 ns, one tick is 0.5 ns) and `clk_rx` one of the `Rx*` clocks (80
//! ticks = 40 ns at several phases, or 81 ticks = 40.5 ns so that the two
//! edges drift past each other and every phase is visited).
//!
//! `mii_rx` is fixed to `ClkRx` (period 2 ticks), so it cannot run at
//! these ratios: [`MiiModel`] stands in for it with its exact output
//! timing (module doc of `mii_rx.rs`): the SFD nibble is sampled at edge
//! `k`, `sfd` is seen high by the consumer at edge `k + 1`, byte `i` of
//! `n` is offered at edge `k + 4 + 2i` and the final one at `k + 2n + 1`.
//! A last test runs the real `MiiRx` on `ClkRx` with the path at a ratio
//! of one.
//!
//! `time` is driven by the test bench before every step with the clock
//! value of that tick, so the value the path reads at an edge is the time
//! of that edge, as in the design where `NtpClock` drives it. The
//! reference of a frame's stamp is the time of the edge at which the SFD
//! nibble is sampled.
use ntp_fpga::clocks::ClkRx;
use ntp_fpga::mii_rx::MiiRx;
use ntp_fpga::rx_path::{RxPath, RX_STAMP_BIAS, STAMP_DEPTH};
use ntp_fpga::stream::{FrameByte, Ts};
use std::cell::RefCell;
use std::rc::Rc;
use txhdl::comp::{chan, join2, now, signal, Clock, DefaultClock, Out, Running, Rx, Tx, Unit};
use txhdl::types::{Bit, U};

/// `clk_sys`: 10 ns.
struct Sys;
impl Clock for Sys {
    const NAME: &'static str = "sys";
    const PERIOD: u64 = 20;
}

macro_rules! rx_clock {
    ($name:ident, $period:expr, $phase:expr) => {
        struct $name;
        impl Clock for $name {
            const NAME: &'static str = stringify!($name);
            const PERIOD: u64 = $period;
            const PHASE: u64 = $phase;
        }
    };
}
rx_clock!(Rx80p0, 80, 0);
rx_clock!(Rx80p1, 80, 1);
rx_clock!(Rx80p5, 80, 5);
rx_clock!(Rx80p10, 80, 10);
rx_clock!(Rx80p15, 80, 15);
rx_clock!(Rx80p19, 80, 19);
rx_clock!(Rx81, 81, 7);
rx_clock!(Rx79, 79, 3);

/// Picoseconds per tick of the `Sys`/`Rx*` clocks.
const TICK_PS: u128 = 500;
/// NTP seconds at tick 0: a date in 2023, so the stamps have real high bits.
const EPOCH_S: u128 = 3_900_000_000;

/// The 32.64 accumulator at tick `t`.
fn acc_at(t: u64, tick_ps: u128) -> U<96> {
    let frac = ((t as u128 * tick_ps) << 64) / 1_000_000_000_000;
    U::<96>::from((EPOCH_S << 64) + frac)
}

/// 32.32 NTP time at tick `t`.
fn ntp_at(t: u64, tick_ps: u128) -> u64 {
    (acc_at(t, tick_ps).raw() >> 32) as u64
}

/// Signed difference of two 32.32 times, in nanoseconds.
fn diff_ns(a: u64, b: u64) -> f64 {
    (a.wrapping_sub(b) as i64) as f64 * 1e9 / 4_294_967_296.0
}

/// A frame offered by the model. `len == 0` is an SFD without a single
/// complete byte (an orphan SFD). `gap` is the `rx_dv`-low time after it,
/// in `clk_rx` cycles; a preamble of 15 nibbles precedes each SFD.
#[derive(Clone)]
struct TFrame {
    len: usize,
    err: bool,
    gap: u64,
}

/// Byte `i` of frame `id`: the first two bytes carry the id.
fn byte(id: usize, i: usize) -> u8 {
    match i {
        0 => id as u8,
        1 => (id >> 8) as u8,
        _ => (i * 7 + id * 13) as u8,
    }
}

/// One `clk_rx` cycle of `mii_rx` output.
#[derive(Clone, Copy, Default)]
struct Act {
    sfd: bool,
    byte: Option<(u8, bool, bool)>,
}

/// The per-cycle output of `mii_rx` for `frames`, and the SFD edge of each.
fn script(frames: &[TFrame]) -> (Vec<Act>, Vec<usize>) {
    let mut acts = vec![Act::default(); 8];
    let mut sfds = vec![];
    let mut t = 8usize;
    for (id, f) in frames.iter().enumerate() {
        let k = t + 15;
        let end = k + 2 * f.len + 1; // first cycle with rx_dv low
        let next = end + f.gap as usize;
        acts.resize(next + 8, Act::default());
        acts[k].sfd = true;
        sfds.push(k);
        for i in 0..f.len {
            let last = i + 1 == f.len;
            let at = if last { end } else { k + 4 + 2 * i };
            assert!(acts[at].byte.is_none());
            acts[at].byte = Some((byte(id, i), last, last && f.err));
        }
        t = next;
    }
    acts.resize(t + 40, Act::default());
    (acts, sfds)
}

/// `mii_rx` at clock `R`, from a script; logs the tick of each SFD edge.
struct MiiModel {
    acts: Vec<Act>,
    sfd_ticks: Rc<RefCell<Vec<u64>>>,
}

impl<R: Clock> Unit<(), (Tx<FrameByte, R>, Out<Bit, R>)> for MiiModel {
    async fn run(&mut self, _i: (), (raw, sfd): (Tx<FrameByte, R>, Out<Bit, R>)) {
        let mut c = 0usize;
        let mut sfd_r = false;
        loop {
            R::rising().await;
            let a = self.acts.get(c).copied().unwrap_or_default();
            // `mii_rx`'s `sfd` is a register set at the SFD edge k and
            // read by the consumer at edge k + 1. An `Out` is seen in the
            // same step by a process polled after this one (the path is
            // after the model in the join), so the register's value is
            // driven, as `mii_rx` does.
            sfd.set(Bit::from_bool(sfd_r));
            sfd_r = a.sfd;
            if a.sfd {
                self.sfd_ticks.borrow_mut().push(now());
            }
            if let Some((d, last, err)) = a.byte {
                assert!(raw.ready().to_bool(), "the path stalled the wire");
                raw.send(FrameByte::new(d, last, err));
            }
            c += 1;
        }
    }
}

/// What came out on `clk_sys`.
#[derive(Default)]
struct Seen {
    /// (tick, stamp)
    stamps: Vec<(u64, u64)>,
    /// (tick, byte)
    bytes: Vec<(u64, FrameByte)>,
}

/// Consumer at clock `S`, ready when `ready(cycle)`.
struct Sink<F: Fn(u64) -> bool> {
    ready: F,
    seen: Rc<RefCell<Seen>>,
}

impl<S: Clock, F: Fn(u64) -> bool> Unit<(Rx<FrameByte, S>, Rx<Ts, S>), ()> for Sink<F> {
    async fn run(&mut self, (frames, stamps): (Rx<FrameByte, S>, Rx<Ts, S>), _o: ()) {
        let mut c = 0u64;
        loop {
            S::rising().await;
            if (self.ready)(c) {
                if let Some(s) = stamps.recv() {
                    self.seen.borrow_mut().stamps.push((now(), s.raw() as u64));
                }
                if let Some(b) = frames.recv() {
                    self.seen.borrow_mut().bytes.push((now(), b));
                }
            }
            c += 1;
        }
    }
}

/// A delivered frame: its bytes, the tick of its first byte, `err`.
struct OutFrame {
    data: Vec<u8>,
    first_tick: u64,
    err: bool,
}

/// Run `frames` through `RxPath<R, S>`; returns the delivered frames, the
/// stamps (tick, value) and the SFD ticks.
fn run_path<R: Clock, S: Clock>(
    frames: &[TFrame],
    ready: impl Fn(u64) -> bool,
    tick_ps: u128,
) -> (Vec<OutFrame>, Vec<(u64, u64)>, Vec<u64>) {
    let (acts, _) = script(frames);
    let total_ticks = (acts.len() as u64 + 20) * R::PERIOD + 4000 * S::PERIOD;
    let sfd_ticks = Rc::new(RefCell::new(vec![]));
    let seen = Rc::new(RefCell::new(Seen::default()));
    let (raw_tx, raw_rx) = chan::<FrameByte, R>();
    let (sfd_o, sfd_i) = signal::<Bit, R>();
    let (time_o, time_i) = signal::<U<96>, S>();
    let (fr_tx, fr_rx) = chan::<FrameByte, S>();
    let (st_tx, st_rx) = chan::<Ts, S>();
    let mut model = MiiModel {
        acts,
        sfd_ticks: sfd_ticks.clone(),
    };
    let mut path = RxPath::<R, S>::default();
    let mut sink = Sink {
        ready,
        seen: seen.clone(),
    };
    {
        let mut sim = Running::new(join2(
            join2(
                <MiiModel as Unit<(), (Tx<FrameByte, R>, Out<Bit, R>)>>::run(
                    &mut model,
                    (),
                    (raw_tx, sfd_o),
                ),
                path.run((raw_rx, sfd_i, time_i), (fr_tx, st_tx)),
            ),
            <Sink<_> as Unit<(Rx<FrameByte, S>, Rx<Ts, S>), ()>>::run(
                &mut sink,
                (fr_rx, st_rx),
                (),
            ),
        ));
        // Simulated time runs on across the runs of one test: `now()`.
        for _ in 0..total_ticks {
            time_o.set(acc_at(now(), tick_ps));
            sim.step();
        }
    }
    let seen = seen.borrow();
    // Cut the byte stream into frames, checking the stream contract.
    let mut out = vec![];
    let mut cur: Vec<u8> = vec![];
    let mut first_tick = 0;
    for &(t, b) in &seen.bytes {
        if cur.is_empty() {
            first_tick = t;
        }
        cur.push(b.data.raw() as u8);
        let last = b.last.to_bool();
        let err = b.err.to_bool();
        assert!(!err || last, "err without last");
        if last {
            assert!(cur.len() <= 1518, "frame longer than 1518 bytes");
            out.push(OutFrame {
                data: std::mem::take(&mut cur),
                first_tick,
                err,
            });
        }
    }
    assert!(cur.is_empty(), "a frame without last: {} bytes", cur.len());
    let st = seen.stamps.clone();
    let sf = sfd_ticks.borrow().clone();
    (out, st, sf)
}

/// Checks pairing (n-th stamp to n-th frame, no later than its first byte)
/// and returns the stamp error of each delivered frame, in ns, against the
/// time of its SFD edge. A complete frame must equal what was sent; a cut
/// one is a prefix closed by a `last & err` byte.
fn check_frames(
    frames: &[TFrame],
    out: &[OutFrame],
    stamps: &[(u64, u64)],
    sfd_ticks: &[u64],
    tick_ps: u128,
) -> Vec<f64> {
    assert_eq!(out.len(), stamps.len(), "one stamp per frame");
    let mut errs = vec![];
    let mut prev_id: Option<usize> = None;
    for (f, &(st_tick, stamp)) in out.iter().zip(stamps) {
        assert!(
            st_tick <= f.first_tick,
            "stamp after the frame's first byte"
        );
        // Every test has fewer than 256 frames: the first byte is the id.
        let id = f.data[0] as usize;
        assert!(id < frames.len(), "unknown frame id {id}");
        if let Some(p) = prev_id {
            assert!(id > p, "frames out of order: {p} then {id}");
        }
        prev_id = Some(id);
        let sent: Vec<u8> = (0..frames[id].len).map(|i| byte(id, i)).collect();
        if f.data.len() == sent.len() && f.data == sent {
            assert_eq!(f.err, frames[id].err, "frame {id}: err flag");
        } else {
            assert!(f.err, "frame {id} altered without err");
            assert!(f.data.len() <= sent.len(), "frame {id} too long");
            assert_eq!(
                f.data[..f.data.len() - 1],
                sent[..f.data.len() - 1],
                "frame {id}: prefix"
            );
        }
        // The stamp estimates the time of the SFD edge.
        let e = diff_ns(stamp, ntp_at(sfd_ticks[id], tick_ps));
        errs.push(e);
    }
    errs
}

fn always(_: u64) -> bool {
    true
}

fn mix(n: usize, gap: u64) -> Vec<TFrame> {
    let lens = [60usize, 64, 1518, 64, 60, 100, 64];
    (0..n)
        .map(|i| TFrame {
            len: lens[i % lens.len()],
            err: i % 5 == 3,
            gap,
        })
        .collect()
}

/// Bound on the stamp error: half a `clk_sys` cycle for the asynchronous
/// crossing, plus the deviation of 2 `clk_rx` cycles from 80 ns when the
/// test's `clk_rx` is not 40 ns, plus rounding.
fn bound_ns(rx_period_ticks: u64) -> f64 {
    let rx_ns = rx_period_ticks as f64 * TICK_PS as f64 / 1000.0;
    5.0 + 2.0 * (rx_ns - 40.0).abs() + 0.5
}

fn stamp_case<R: Clock>(n: usize, gap: u64) -> (f64, f64) {
    let frames = mix(n, gap);
    let (out, st, sf) = run_path::<R, Sys>(&frames, always, TICK_PS);
    assert_eq!(out.len(), n, "every frame delivered");
    let errs = check_frames(&frames, &out, &st, &sf, TICK_PS);
    let b = bound_ns(R::PERIOD);
    let (lo, hi) = errs
        .iter()
        .fold((f64::MAX, f64::MIN), |(l, h), &e| (l.min(e), h.max(e)));
    assert!(
        lo >= -b && hi <= b,
        "{}: stamp error {lo:.2}..{hi:.2} ns beyond ±{b} ns",
        R::NAME
    );
    println!("{}: {n} frames, stamp error {lo:.2} .. {hi:.2} ns", R::NAME);
    (lo, hi)
}

#[test]
fn frames_and_stamps_at_ratio_4_all_phases() {
    stamp_case::<Rx80p0>(12, 30);
    stamp_case::<Rx80p1>(12, 30);
    stamp_case::<Rx80p5>(12, 30);
    stamp_case::<Rx80p10>(12, 30);
    stamp_case::<Rx80p15>(12, 30);
    stamp_case::<Rx80p19>(12, 30);
}

#[test]
fn stamps_sweep_every_phase_at_ratio_4_05() {
    // 40.5 ns against 10 ns: the phase moves by a tick per clk_rx cycle,
    // so 60 frames visit every relative phase many times.
    let (lo, hi) = stamp_case::<Rx81>(60, 37);
    assert!(
        hi - lo > 8.0,
        "the sweep should show the full ±1/2 cycle spread"
    );
}

#[test]
fn stamps_at_ratio_3_95() {
    stamp_case::<Rx79>(40, 41);
}

#[test]
fn back_to_back_at_minimum_gap() {
    // 96 bit times of IFG = 24 clk_rx cycles, then preamble and SFD.
    let frames: Vec<TFrame> = (0..40)
        .map(|i| TFrame {
            len: if i % 4 == 2 { 1518 } else { 64 },
            err: false,
            gap: 24,
        })
        .collect();
    let (out, st, sf) = run_path::<Rx81, Sys>(&frames, always, TICK_PS);
    assert_eq!(out.len(), frames.len(), "no frame lost at minimum gap");
    let errs = check_frames(&frames, &out, &st, &sf, TICK_PS);
    assert!(errs.iter().all(|e| e.abs() <= bound_ns(81)));
    let mins: Vec<TFrame> = (0..60)
        .map(|_| TFrame {
            len: 60,
            err: false,
            gap: 24,
        })
        .collect();
    let (out, st, sf) = run_path::<Rx80p5, Sys>(&mins, always, TICK_PS);
    assert_eq!(out.len(), 60);
    check_frames(&mins, &out, &st, &sf, TICK_PS);
}

#[test]
fn orphan_sfds_do_not_misalign_stamps() {
    // Every other frame is an SFD with no complete byte; some are runs of
    // several orphans, one at the minimum spacing `mii_rx` allows.
    let mut frames = vec![];
    for i in 0..30 {
        let len = match i % 6 {
            1 | 3 | 4 => 0,
            2 => 1,
            _ => 64,
        };
        frames.push(TFrame {
            len,
            err: false,
            gap: if len == 0 { 2 } else { 30 },
        });
    }
    let (out, st, sf) = run_path::<Rx81, Sys>(&frames, always, TICK_PS);
    let expect = frames.iter().filter(|f| f.len > 0).count();
    assert_eq!(out.len(), expect);
    let errs = check_frames(&frames, &out, &st, &sf, TICK_PS);
    assert!(errs.iter().all(|e| e.abs() <= bound_ns(81)), "{errs:?}");
}

#[test]
fn consumer_backpressure() {
    // The consumer takes one cycle in three, and pauses for 2000 cycles.
    let ready = |c: u64| c % 3 == 0 && !(3000..5000).contains(&c);
    let frames = mix(20, 30);
    let (out, st, sf) = run_path::<Rx81, Sys>(&frames, ready, TICK_PS);
    assert_eq!(out.len(), 20, "a slow consumer loses nothing that fits");
    let errs = check_frames(&frames, &out, &st, &sf, TICK_PS);
    assert!(errs.iter().all(|e| e.abs() <= bound_ns(81)));
}

#[test]
fn overflow_cuts_with_err_and_keeps_pairing() {
    // The consumer is stalled while three 1518-byte frames arrive: the
    // first fits, the second is cut with `last & err`, the third finds the
    // FIFO full at its first byte and is dropped whole. After the stall
    // everything flows again.
    let ready = |c: u64| c > 40_000;
    let mut frames: Vec<TFrame> = (0..3)
        .map(|_| TFrame {
            len: 1518,
            err: false,
            gap: 24,
        })
        .collect();
    // The stall (400 us) ends during this idle time (120 us from 370 us).
    frames[2].gap = 3000;
    frames.extend((0..4).map(|_| TFrame {
        len: 64,
        err: false,
        gap: 3000,
    }));
    let (out, st, sf) = run_path::<Rx81, Sys>(&frames, ready, TICK_PS);
    let errs = check_frames(&frames, &out, &st, &sf, TICK_PS);
    let ids: Vec<usize> = out.iter().map(|f| f.data[0] as usize).collect();
    assert_eq!(ids[0], 0);
    assert!(
        !out[0].err && out[0].data.len() == 1518,
        "the first frame fits whole"
    );
    assert_eq!(ids[1], 1);
    assert!(
        out[1].err && out[1].data.len() < 1518,
        "the second is cut with err"
    );
    assert_eq!(
        *out[1].data.last().unwrap(),
        0,
        "the terminator byte carries 0"
    );
    // 2048 words in the FIFO plus the few slots of the channels around it.
    let held = out[0].data.len() + out[1].data.len() - 1;
    assert!(
        (2048..2048 + 8).contains(&held),
        "the FIFO was full: {held}"
    );
    assert!(!ids.contains(&2), "the third is dropped whole");
    assert!(
        ids.ends_with(&[3, 4, 5, 6]),
        "frames after the stall flow: {ids:?}"
    );
    assert!(errs.iter().all(|e| e.abs() <= bound_ns(81)), "{errs:?}");
}

#[test]
fn stamp_credits_limit_queued_frames() {
    // Short frames while the consumer is stalled: STAMP_DEPTH of them wait
    // with their stamps, the next ones are dropped whole (no credit), and
    // the pairing holds once the consumer resumes.
    let ready = |c: u64| c > 30_000;
    let mut frames: Vec<TFrame> = (0..STAMP_DEPTH + 6)
        .map(|_| TFrame {
            len: 64,
            err: false,
            gap: 24,
        })
        .collect();
    // A long idle time (400 us) before the last three, which arrive after
    // the stall (300 us).
    frames.last_mut().unwrap().gap = 10_000;
    frames.extend((0..3).map(|_| TFrame {
        len: 64,
        err: false,
        gap: 100,
    }));
    let (out, st, sf) = run_path::<Rx81, Sys>(&frames, ready, TICK_PS);
    let errs = check_frames(&frames, &out, &st, &sf, TICK_PS);
    let ids: Vec<usize> = out.iter().map(|f| f.data[0] as usize).collect();
    let n = frames.len();
    // The first STAMP_DEPTH frames keep their stamps in the queue; one
    // more may get through, its stamp already handed to the stalled
    // channel (a slot freed); the rest of the burst is dropped whole.
    assert_eq!(ids[..STAMP_DEPTH], (0..STAMP_DEPTH).collect::<Vec<_>>()[..]);
    assert!(ids.ends_with(&[n - 3, n - 2, n - 1]), "{ids:?}");
    assert!(
        ids.len() <= STAMP_DEPTH + 1 + 3,
        "frames beyond the credits were kept: {ids:?}"
    );
    assert!(errs.iter().all(|e| e.abs() <= bound_ns(81)), "{errs:?}");
}

// ---------------------------------------------------------------------
// With the real `mii_rx`, on `ClkRx` and `DefaultClock` (ratio 1).
// ---------------------------------------------------------------------

/// One MII cycle.
#[derive(Clone, Copy, Default)]
struct Cy {
    d: u8,
    dv: bool,
    er: bool,
}

fn nibbles(pre: usize, payload: &[u8], extra_nib: bool, er: bool) -> Vec<Cy> {
    let mut v = vec![
        Cy {
            d: 5,
            dv: true,
            er: false
        };
        pre
    ];
    v.push(Cy {
        d: 0xd,
        dv: true,
        er: false,
    });
    for (i, b) in payload.iter().enumerate() {
        let e = er && i == 3;
        v.push(Cy {
            d: b & 15,
            dv: true,
            er: e,
        });
        v.push(Cy {
            d: b >> 4,
            dv: true,
            er: false,
        });
    }
    if extra_nib {
        v.push(Cy {
            d: 7,
            dv: true,
            er: false,
        });
    }
    v
}

#[test]
fn with_mii_rx_at_ratio_1() {
    // Wire: frame 0, an SFD with one nibble only (orphan), frame 2 with
    // rx_er, an SFD with nothing (orphan), frame 4.
    let mut wire: Vec<Cy> = vec![Cy::default(); 5];
    let mut sfd_at = vec![];
    let mut expect = vec![];
    for id in 0..5usize {
        let (len, extra, er) = match id {
            1 => (0, true, false),
            3 => (0, false, false),
            2 => (70, false, true),
            _ => (64, false, false),
        };
        let p: Vec<u8> = (0..len).map(|i| byte(id, i)).collect();
        sfd_at.push(wire.len() + 15);
        wire.extend(nibbles(15, &p, extra, er));
        wire.extend(vec![Cy::default(); 24]);
        if len > 0 {
            expect.push((id, p, er));
        }
    }
    wire.extend(vec![Cy::default(); 200]);

    let (d_o, d) = signal::<U<4>, ClkRx>();
    let (dv_o, dv) = signal::<Bit, ClkRx>();
    let (er_o, er) = signal::<Bit, ClkRx>();
    let (m_tx, m_rx) = chan::<FrameByte, ClkRx>();
    let (sfd_o, sfd_i) = signal::<Bit, ClkRx>();
    let (time_o, time_i) = signal::<U<96>, DefaultClock>();
    let (fr_tx, fr_rx) = chan::<FrameByte, DefaultClock>();
    let (st_tx, st_rx) = chan::<Ts, DefaultClock>();
    let mut mii = MiiRx::default();
    let mut path = RxPath::<ClkRx, DefaultClock>::default();
    let mut sim = Running::new(join2(
        mii.run((d, dv, er), (m_tx, sfd_o)),
        path.run((m_rx, sfd_i, time_i), (fr_tx, st_tx)),
    ));
    // One tick is 5 ns here: DefaultClock has a period of 2 ticks.
    let tick_ps = 5000u128;
    let mut bytes = vec![];
    let mut stamps = vec![];
    for (c, w) in wire.iter().enumerate() {
        d_o.set(U::<4>::from(w.d));
        dv_o.set(Bit::from_bool(w.dv));
        er_o.set(Bit::from_bool(w.er));
        for _ in 0..2 {
            let t = now();
            time_o.set(acc_at(t, tick_ps));
            sim.step();
        }
        if let Some(s) = st_rx.recv() {
            stamps.push((c, s.raw() as u64));
        }
        if let Some(b) = fr_rx.recv() {
            bytes.push((c, b));
        }
    }
    let mut frames: Vec<(usize, Vec<u8>, bool)> = vec![];
    let mut cur = vec![];
    let mut first = 0;
    for (c, b) in &bytes {
        if cur.is_empty() {
            first = *c;
        }
        cur.push(b.data.raw() as u8);
        assert!(!b.err.to_bool() || b.last.to_bool());
        if b.last.to_bool() {
            frames.push((first, std::mem::take(&mut cur), b.err.to_bool()));
        }
    }
    assert_eq!(frames.len(), expect.len(), "orphan SFDs emit no frame");
    assert_eq!(stamps.len(), frames.len(), "one stamp per frame");
    for ((f, s), (id, p, er)) in frames.iter().zip(&stamps).zip(&expect) {
        assert_eq!(&f.1, p, "frame {id} bytes, FCS included");
        assert_eq!(f.2, *er, "frame {id} err");
        assert!(s.0 <= f.0, "stamp no later than the first byte");
        // Ratio 1, both cycles 10 ns: the capture is 2 clk_rx cycles,
        // one cycle to the first clk_sys edge after the event (the edges
        // coincide and a flop sees the old value), then 3 more: 60 ns.
        let wire_t = (sfd_at[*id] as u64) * 2;
        let ideal = ntp_at(wire_t, tick_ps);
        let got_ns = diff_ns(s.1.wrapping_add(RX_STAMP_BIAS), ideal);
        assert!(
            (got_ns - 60.0).abs() < 0.5,
            "frame {id}: capture {got_ns:.2} ns after the SFD edge, expected 60"
        );
    }
}
