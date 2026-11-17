// SPDX-License-Identifier: Apache-2.0
//! System tests of the top level (task T1.11), cycle accurate.
//!
//! Two benches:
//!
//! * **Pins** ([`PinBench`]): the whole `ArtyTop` (real `mii_rx`, `mii_tx`,
//!   `phy_init`) driven on its MII pins: the test writes the nibbles of
//!   each frame (preamble, SFD, frame, FCS, as a PHY delivers them) on
//!   `rxd`/`rx_dv` and decodes `txd`/`tx_en`. `MiiRx`/`MiiTx` are fixed to
//!   the clock types `ClkRx`/`ClkTx`, whose simulation period is that of
//!   `clk_sys` (`docs/questions.md`), so on this bench the MII runs at the
//!   `clk_sys` rate (100 Mnibble/s): functionally the same design, 4 times
//!   faster on the wire, and the timestamps are only checked for
//!   plausibility here.
//! * **Clocks** ([`run_core`]): `ArtyCore` (everything between the MII byte
//!   streams) with MII clocks of the test's own (`clk_sys` = 2 ticks of
//!   5 ns; `clk_rx`/`clk_tx` = 8 ticks = 40 ns = 4 `clk_sys` periods, at odd
//!   phases, i.e. between `clk_sys` edges; and a variant at 9 and 7 ticks),
//!   with models of `mii_rx` and `mii_tx` that reproduce their port timing
//!   exactly (module headers of `mii_rx.rs`, `mii_tx.rs`). This bench
//!   measures the receive and transmit timestamps against the SFD instants.
//!
//! # Comparing with the golden vectors
//!
//! Received frames are the vector's `rx` frames (already padded to 60
//! bytes) plus their FCS, after 15 preamble nibbles and the SFD. A
//! transmitted frame is decoded (preamble and SFD removed), its FCS is
//! verified and removed, and it must equal the vector's `tx` frame followed
//! by zero padding up to 60 bytes.
//!
//! NTP replies carry the Receive (T2) and Transmit (T3) timestamps of the
//! simulated clock. Choosing the clock's start so that both equal the
//! vector's would need T2 and T3 exact to 2^-32 s, while the design's
//! timestamps are quantised to 10 ns steps of the clock and depend on the
//! phase of the MII clocks: instead, the 16 timestamp bytes of the
//! transmitted reply are copied into the expected frame, whose UDP checksum
//! is then recomputed, and the whole frame must be byte-exact; T2 and T3
//! are checked separately against the simulated SFD instants (pins bench:
//! plausibility, within a few microseconds and T3 > T2; clocks bench: the
//! measured error, printed, under the 100 ns design target).
use ntp_fpga::clocks::{ClkRx, ClkTx};
use ntp_fpga::config::Test;
use ntp_fpga::phy_init;
use ntp_fpga::stream::FrameByte;
use ntp_fpga::top::{ArtyCore, ArtyTop, FixedSync, FixedUnsync, Standalone, SyncSource};
use std::cell::RefCell;
use std::rc::Rc;
use txhdl::comp::{chan, join2, now, signal, Clock, In, Out, Reg, Running, Rx, Tx, Unit};
use txhdl::types::{Bit, U};

// ------------------------------------------------------------ vectors

fn from_hex(s: &str) -> Vec<u8> {
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
        .collect()
}

/// (is_rx, frame) lines of a vector file.
fn vector(name: &str) -> Vec<(bool, Vec<u8>)> {
    let p = format!(
        "{}/../tools/golden/vectors/{}",
        env!("CARGO_MANIFEST_DIR"),
        name
    );
    let text = std::fs::read_to_string(&p).unwrap_or_else(|e| panic!("{p}: {e}"));
    let mut v = vec![];
    for l in text.lines() {
        let l = l.trim();
        if l.is_empty() || l.starts_with('#') {
            continue;
        }
        let (d, h) = l.split_once(' ').unwrap();
        v.push((d == "rx", from_hex(h)));
    }
    v
}

/// The `rx` frames of a vector and, for each, the `tx` frames it must cause.
fn exchanges(name: &str) -> Vec<(Vec<u8>, Vec<Vec<u8>>)> {
    let mut out: Vec<(Vec<u8>, Vec<Vec<u8>>)> = vec![];
    for (rx, f) in vector(name) {
        if rx {
            out.push((f, vec![]));
        } else {
            out.last_mut().expect("tx before any rx").1.push(f);
        }
    }
    out
}

fn crc32(bytes: &[u8]) -> u32 {
    let mut c = 0xffff_ffffu32;
    for &b in bytes {
        for i in 0..8 {
            let bit = ((c ^ (b as u32 >> i)) & 1) != 0;
            c >>= 1;
            if bit {
                c ^= 0xedb8_8320;
            }
        }
    }
    !c
}

/// A frame as the PHY delivers it after the SFD: the frame and its FCS.
fn with_fcs(f: &[u8]) -> Vec<u8> {
    let mut v = f.to_vec();
    v.extend_from_slice(&crc32(f).to_le_bytes());
    v
}

fn be64(b: &[u8]) -> u64 {
    b.iter().fold(0u64, |a, &x| (a << 8) | x as u64)
}

/// RFC 1071 sum, not complemented.
fn sum16(acc: u32, b: &[u8]) -> u32 {
    let mut a = acc;
    for c in b.chunks(2) {
        a += ((c[0] as u32) << 8) | *c.get(1).unwrap_or(&0) as u32;
    }
    a
}

fn fold(mut a: u32) -> u16 {
    while a > 0xffff {
        a = (a & 0xffff) + (a >> 16);
    }
    a as u16
}

/// Offset of the UDP header of an NTP reply (source port 123), if `f` is one.
fn ntp_udp_offset(f: &[u8]) -> Option<usize> {
    let et = u16::from_be_bytes([f[12], f[13]]);
    if et == 0x0800 && f.len() >= 42 && f[23] == 17 && u16::from_be_bytes([f[34], f[35]]) == 123 {
        Some(34)
    } else if et == 0x86dd
        && f.len() >= 62
        && f[20] == 17
        && u16::from_be_bytes([f[54], f[55]]) == 123
    {
        Some(54)
    } else {
        None
    }
}

/// Recomputes the UDP checksum of an IPv4 or IPv6 frame in place.
fn fix_udp_csum(f: &mut [u8], u: usize) {
    let len = u16::from_be_bytes([f[u + 4], f[u + 5]]) as usize;
    f[u + 6] = 0;
    f[u + 7] = 0;
    let mut a = if u == 34 {
        let a = sum16(0, &f[26..34]);
        a + 17 + len as u32
    } else {
        let a = sum16(0, &f[22..54]);
        a + 17 + len as u32
    };
    a = sum16(a, &f[u..u + len]);
    let mut c = !fold(a);
    if c == 0 {
        c = 0xffff;
    }
    f[u + 6..u + 8].copy_from_slice(&c.to_be_bytes());
}

/// Checks a transmitted frame (FCS included) against the expected `tx`
/// frame of a vector; for an NTP reply, returns (T2, T3) as transmitted.
fn check_reply(got: &[u8], want: &[u8], what: &str) -> Option<(u64, u64)> {
    let n = got.len();
    assert!(
        n >= 64,
        "{what}: frame of {n} bytes, shorter than 64 with FCS"
    );
    let (body, fcs) = got.split_at(n - 4);
    assert_eq!(
        u32::from_le_bytes([fcs[0], fcs[1], fcs[2], fcs[3]]),
        crc32(body),
        "{what}: FCS"
    );
    assert_eq!(
        body.len(),
        want.len().max(60),
        "{what}: length (padding to 60 only)"
    );
    assert!(
        body[want.len()..].iter().all(|b| *b == 0),
        "{what}: padding is zero"
    );
    let mut want = want.to_vec();
    let ts = ntp_udp_offset(&want).map(|u| {
        let t = u + 8 + 32;
        want[t..t + 16].copy_from_slice(&body[t..t + 16]);
        fix_udp_csum(&mut want, u);
        (be64(&body[t..t + 8]), be64(&body[t + 8..t + 16]))
    });
    if body[..want.len()] != want[..] {
        let d = (0..want.len()).find(|&i| body[i] != want[i]).unwrap();
        panic!(
            "{what}: byte {d} differs\n got  {}\n want {}",
            hex(&body[..want.len()]),
            hex(&want)
        );
    }
    ts
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}

/// Signed difference of two 32.32 times, in nanoseconds.
fn diff_ns(a: u64, b: u64) -> f64 {
    (a.wrapping_sub(b) as i64) as f64 * 1e9 / 4_294_967_296.0
}

/// Start of the simulated clock (32.64 accumulator): just before the
/// golden Receive timestamp `ecc58cc0.40000000`, so the replies look like
/// the vectors'.
const CLOCK_START: u128 = 0xECC5_8CC0_3FFF_0000u128 << 32;
/// `DisciplinedClock` increment per `clk_sys` cycle.
const INCR: u128 = ntp_fpga::clock::INCR;

/// Starts the clock at [`CLOCK_START`] instead of zero, with its ghost
/// registers (formal monitors, checked in simulation) in the state of a
/// clock that has been running: last increment `INCR`, a step already
/// applied (so the from-reset lifetime bound does not apply).
fn start_clock(c: &mut ntp_fpga::clock::DisciplinedClock) {
    c.acc = Reg::new(U::<96>::from(CLOCK_START));
    c.g_valid = Reg::new(Bit::One);
    c.g_prev = Reg::new(U::<96>::from(CLOCK_START - INCR));
    c.g_inc = Reg::new(U::<96>::from(INCR));
    c.g_any = Reg::new(Bit::One);
}

/// 32.32 time of the design's clock at tick `t` (`clk_sys` = 2 ticks: the
/// accumulator read at the edge of tick `2 n` is `CLOCK_START + n INCR`),
/// linear in between.
fn ntp_at(t: u64) -> u64 {
    ((CLOCK_START + t as u128 * INCR / 2) >> 32) as u64
}

// ------------------------------------------------------------ pins bench

/// One transmitted frame on the pins: bytes after the SFD (FCS included)
/// and the tick at which the SFD's 0xD nibble was seen on `txd`.
struct TxFrame {
    bytes: Vec<u8>,
    sfd_tick: u64,
}

/// `ArtyTop<Test, S, phy_init::Sim>` driven on its MII pins, MII at the
/// `clk_sys` rate (one nibble per cycle).
struct PinBench {
    /// Per cycle: (rxd, rx_dv, rx_er).
    wire: Vec<(u8, bool, bool)>,
    /// Tick of the cycle that carries each frame's SFD nibble.
    rx_sfd_ticks: Vec<u64>,
}

impl PinBench {
    fn new() -> Self {
        PinBench {
            wire: vec![(0, false, false); 40],
            rx_sfd_ticks: vec![],
        }
    }

    /// Appends `gap` idle cycles, then a frame with preamble, SFD and FCS
    /// (`fcs_flip` corrupts one FCS bit).
    fn frame(&mut self, f: &[u8], gap: usize, fcs_flip: bool) {
        self.wire
            .extend(std::iter::repeat_n((0, false, false), gap));
        let mut w = with_fcs(f);
        if fcs_flip {
            let n = w.len();
            w[n - 1] ^= 0x10;
        }
        self.wire.extend(std::iter::repeat_n((5, true, false), 15));
        // Ticks are 2 per cycle: the cycle index is half the tick.
        self.rx_sfd_ticks.push(2 * self.wire.len() as u64);
        self.wire.push((0xd, true, false));
        for b in w {
            self.wire.push((b & 15, true, false));
            self.wire.push((b >> 4, true, false));
        }
    }

    /// Runs the wire then `tail` more cycles; returns the frames on `txd`.
    fn run<S: SyncSource>(&self, tail: usize) -> Vec<TxFrame> {
        let (rxd_o, rxd) = signal::<U<4>, ClkRx>();
        let (dv_o, dv) = signal::<Bit, ClkRx>();
        let (er_o, er) = signal::<Bit, ClkRx>();
        let (mdio_o_, mdio_i) = signal::<Bit, txhdl::comp::DefaultClock>();
        let (txd_o, txd) = signal::<U<4>, ClkTx>();
        let (en_o, en) = signal::<Bit, ClkTx>();
        let outs: Vec<(Out<Bit>, In<Bit>)> = (0..7)
            .map(|_| signal::<Bit, txhdl::comp::DefaultClock>())
            .collect();
        let mut outs = outs.into_iter();
        let mut o = || outs.next().unwrap().0;
        let mut top = ArtyTop::<Test, S, phy_init::Sim>::default();
        start_clock(&mut top.core.clock);
        mdio_o_.set(Bit::One); // no PHY: the pulled-up line
        let t0 = now();
        let mut sim = Running::new(top.run(
            (rxd, dv, er, mdio_i),
            (txd_o, en_o, o(), o(), o(), o(), o(), o(), o()),
        ));
        let mut frames = vec![];
        let mut nib: Vec<u8> = vec![];
        let mut sfd_at = 0u64;
        for t in 0..self.wire.len() + tail {
            let (d, v, e) = self.wire.get(t).copied().unwrap_or((0, false, false));
            rxd_o.set(U::<4>::from(d));
            dv_o.set(Bit::from_bool(v));
            er_o.set(Bit::from_bool(e));
            sim.cycle();
            if en.get().to_bool() {
                let n = txd.get().raw() as u8;
                if nib.len() == 15 {
                    // The edge that put this nibble on txd began the cycle.
                    sfd_at = now() - t0 - 2;
                }
                nib.push(n);
            } else if !nib.is_empty() {
                frames.push(decode(&nib, sfd_at));
                nib.clear();
            }
        }
        assert!(
            nib.is_empty(),
            "a frame still on the wire at the end of the run"
        );
        frames
    }
}

/// Bytes of a frame from its nibbles (preamble and SFD checked and removed).
fn decode(nib: &[u8], sfd_tick: u64) -> TxFrame {
    assert!(
        nib.len() >= 16 && nib.len().is_multiple_of(2),
        "odd or short frame on txd: {} nibbles",
        nib.len()
    );
    assert!(nib[..15].iter().all(|&n| n == 5), "preamble");
    assert_eq!(nib[15], 0xd, "SFD");
    let bytes = nib[16..].chunks(2).map(|c| c[0] | (c[1] << 4)).collect();
    TxFrame { bytes, sfd_tick }
}

/// Plays the vectors on the pins bench with `S`; every expected reply must
/// come out, in order, and nothing else.
fn play_pins<S: SyncSource>(files: &[&str]) {
    let mut b = PinBench::new();
    let mut want: Vec<(String, Vec<u8>, usize)> = vec![];
    let mut k = 0;
    for name in files {
        for (rx, txs) in exchanges(name) {
            b.frame(&rx, 1500, false);
            for t in txs {
                want.push((name.to_string(), t, k));
            }
            k += 1;
        }
    }
    let got = b.run::<S>(1500);
    assert_eq!(
        got.len(),
        want.len(),
        "number of transmitted frames ({files:?})"
    );
    for (i, (g, (name, w, rxi))) in got.iter().zip(&want).enumerate() {
        let what = format!("{name}, reply {i}");
        if let Some((t2, t3)) = check_reply(&g.bytes, w, &what) {
            let rx_sfd = ntp_at(b.rx_sfd_ticks[*rxi]);
            let tx_sfd = ntp_at(g.sfd_tick);
            let e2 = diff_ns(t2, rx_sfd);
            let e3 = diff_ns(t3, tx_sfd);
            println!("{what}: T2 - rx SFD = {e2:.1} ns, T3 - tx SFD = {e3:.1} ns (MII at the clk_sys rate)");
            assert!(e2.abs() < 1000.0, "{what}: T2 implausible ({e2} ns)");
            assert!(e3.abs() < 2000.0, "{what}: T3 implausible ({e3} ns)");
            assert!(diff_ns(t3, t2) > 0.0, "{what}: T3 after T2");
        }
    }
}

#[test]
fn arp_ping_ndp_end_to_end() {
    play_pins::<FixedSync>(&[
        "arp_request_reply.hex",
        "ping4.hex",
        "ping6.hex",
        "ndp_ns_na.hex",
    ]);
}

#[test]
fn ntp_vectors_end_to_end() {
    play_pins::<FixedSync>(&["ntp4_basic.hex", "ntp6_basic.hex", "ntp_version3.hex"]);
}

#[test]
fn ntp_unsynchronised_end_to_end() {
    play_pins::<FixedUnsync>(&["ntp_unsynchronised.hex"]);
}

/// The negative vectors: the valid first frame of each is answered, the
/// altered second one is not.
#[test]
fn negative_vectors_end_to_end() {
    play_pins::<FixedSync>(&[
        "neg_ipv4_checksum.hex",
        "neg_udp6_checksum.hex",
        "neg_udp_port.hex",
        "neg_ipv4_fragment.hex",
        "neg_ipv4_ttl0.hex",
        "neg_ndp_hoplimit.hex",
        "neg_ntp_mode4.hex",
        "neg_foreign_mac.hex",
    ]);
}

/// The board's sync source: a request gets LI 3, stratum 16, reference id 0.
#[test]
fn standalone_answers_unsynchronised() {
    let (rx, _) = exchanges("ntp4_basic.hex").remove(0);
    let mut b = PinBench::new();
    b.frame(&rx, 100, false);
    let got = b.run::<Standalone>(1500);
    assert_eq!(got.len(), 1);
    let f = &got[0].bytes;
    let ntp = 42;
    assert_eq!(f[ntp] >> 6, 3, "LI 3");
    assert_eq!(f[ntp] & 7, 4, "mode 4");
    assert_eq!(f[ntp + 1], 16, "stratum 16");
    assert_eq!(&f[ntp + 12..ntp + 16], &[0, 0, 0, 0], "reference id");
    assert_eq!(
        u32::from_le_bytes(f[f.len() - 4..].try_into().unwrap()),
        crc32(&f[..f.len() - 4])
    );
}

/// Ping, NTP and ARP back to back at the minimum inter-frame gap (96 bit
/// times = 24 nibbles): three replies, each complete and correct.
#[test]
fn mixed_traffic_back_to_back() {
    let ping = exchanges("ping4.hex").remove(0);
    let ntp = exchanges("ntp4_basic.hex").remove(0);
    let arp = exchanges("arp_request_reply.hex").remove(0);
    let mut b = PinBench::new();
    b.frame(&ping.0, 100, false);
    b.frame(&ntp.0, 24, false);
    b.frame(&arp.0, 24, false);
    let got = b.run::<FixedSync>(2500);
    assert_eq!(got.len(), 3, "three replies");
    let mut seen = [false; 3];
    for g in &got {
        let et = u16::from_be_bytes([g.bytes[12], g.bytes[13]]);
        let k = if et == 0x0806 {
            2
        } else if g.bytes[23] == 1 {
            0
        } else {
            1
        };
        let w = [&ping.1[0], &ntp.1[0], &arp.1[0]][k];
        check_reply(&g.bytes, w, &format!("mixed, reply of kind {k}"));
        assert!(!seen[k], "two replies of kind {k}");
        seen[k] = true;
    }
    // Back-to-back replies keep the minimum gap on the wire.
    for w in got.windows(2) {
        let end = w[0].sfd_tick + 2 * (w[0].bytes.len() as u64 * 2);
        assert!(
            w[1].sfd_tick >= end + 2 * (24 + 15),
            "inter-frame gap on txd"
        );
    }
}

/// A frame with a bad FCS gets no reply; the same frame with a good FCS
/// right after does.
#[test]
fn bad_fcs_is_dropped() {
    let (rx, txs) = exchanges("ntp4_basic.hex").remove(0);
    let mut b = PinBench::new();
    b.frame(&rx, 100, true);
    b.frame(&rx, 1500, false);
    let got = b.run::<FixedSync>(1500);
    assert_eq!(got.len(), 1, "only the good frame is answered");
    assert!(
        got[0].sfd_tick > b.rx_sfd_ticks[1],
        "the reply is the second frame's"
    );
    check_reply(&got[0].bytes, &txs[0], "after a bad FCS");
}

/// Traffic that is not for us, or not served: nothing is transmitted.
#[test]
fn unrelated_traffic_is_ignored() {
    let (ntp, _) = exchanges("ntp4_basic.hex").remove(0);
    let mut other_mac = ntp.clone();
    other_mac[5] = 0x77; // 02:00:00:00:00:77
    let mut other_ip = ntp.clone();
    other_ip[33] = 0x55; // 192.0.2.85 (the IPv4 checksum then fails as well)
    let mut other_et = ntp.clone();
    other_et[12] = 0x88; // EtherType 0x88b5 (local experimental)
    other_et[13] = 0xb5;
    let (arp_other, none) = exchanges("arp_request_reply.hex").remove(1);
    assert!(none.is_empty());
    let mut multicast = ntp.clone();
    multicast[0] = 0x01; // 01:00:00:00:00:01, IPv4 to a group MAC
    let mut b = PinBench::new();
    for f in [&other_mac, &other_ip, &other_et, &arp_other, &multicast] {
        b.frame(f, 200, false);
    }
    let got = b.run::<FixedSync>(1500);
    assert!(got.is_empty(), "{} frames transmitted", got.len());
}

// ------------------------------------------------------------ clocks bench

macro_rules! clock {
    ($name:ident, $period:expr, $phase:expr) => {
        struct $name;
        impl Clock for $name {
            const NAME: &'static str = stringify!($name);
            const PERIOD: u64 = $period;
            const PHASE: u64 = $phase;
        }
    };
}
// clk_sys is DefaultClock: 2 ticks (10 ns), rising at even ticks.
clock!(Rx40a, 8, 1);
clock!(Tx40a, 8, 3);
clock!(Rx40b, 8, 7);
clock!(Tx40b, 8, 5);
clock!(Rx40c, 8, 2);
clock!(Tx40c, 8, 6);
clock!(Rx40d, 8, 5);
clock!(Tx40d, 8, 0);
clock!(Rx40e, 8, 3);
clock!(Tx40e, 8, 7);
clock!(Rx45, 9, 3);
clock!(Tx35, 7, 1);

/// One `clk_rx` cycle of `mii_rx`'s outputs.
#[derive(Clone, Copy, Default)]
struct Act {
    sfd: bool,
    byte: Option<(u8, bool)>,
}

/// `mii_rx` at clock `R` (its documented output timing): for a frame of
/// `n` bytes whose SFD nibble is sampled at edge `k`, `sfd` is seen high
/// by the consumer at edge `k + 1`, byte `i < n - 1` is offered at edge
/// `k + 4 + 2 i` and the final one at `k + 2 n + 1`.
struct MiiRxModel {
    acts: Vec<Act>,
    /// Tick of each SFD edge `k`.
    sfd_ticks: Rc<RefCell<Vec<u64>>>,
}

impl<R: Clock> Unit<(), (Tx<FrameByte, R>, Out<Bit, R>)> for MiiRxModel {
    async fn run(&mut self, _i: (), (raw, sfd): (Tx<FrameByte, R>, Out<Bit, R>)) {
        let mut c = 0usize;
        let mut sfd_r = false;
        loop {
            R::rising().await;
            let a = self.acts.get(c).copied().unwrap_or_default();
            // The register value of the previous edge, seen by the path
            // polled after this model in the same step.
            sfd.set(Bit::from_bool(sfd_r));
            sfd_r = a.sfd;
            if a.sfd {
                self.sfd_ticks.borrow_mut().push(now());
            }
            if let Some((d, last)) = a.byte {
                assert!(raw.ready().to_bool(), "the receive path stalled the wire");
                raw.send(FrameByte::new(d, last, false));
            }
            c += 1;
        }
    }
}

/// `mii_tx` at clock `T`, the register behaviour of `mii_tx.rs` without
/// the nibbles: takes the first byte when idle with the gap over, holds it
/// through 15 preamble/SFD nibbles (the SFD strobe register rises at the
/// edge that puts 0xD on the wire, cnt 15), then takes one byte every two
/// cycles; 24 idle cycles after a frame. Logs the frames and the tick of
/// each SFD edge.
struct MiiTxModel {
    log: Rc<RefCell<Vec<TxLog>>>,
}

/// A frame `mii_tx` sent (FCS included) and the tick of its SFD edge.
type TxLog = (Vec<u8>, u64);

impl<T: Clock> Unit<Rx<FrameByte, T>, (Out<Bit, T>, Out<Bit, T>)> for MiiTxModel {
    async fn run(&mut self, frames: Rx<FrameByte, T>, (sfd, underrun): (Out<Bit, T>, Out<Bit, T>)) {
        // 0 idle, 1 preamble, 2 low nibble, 3 high nibble.
        let (mut st, mut cnt, mut o_sfd) = (0u8, 23u32, false);
        let mut last = false;
        let mut cur: Vec<u8> = vec![];
        let mut sfd_tick = 0u64;
        loop {
            T::rising().await;
            sfd.set(Bit::from_bool(o_sfd));
            underrun.set(Bit::Zero);
            o_sfd = false;
            match st {
                0 => {
                    if cnt > 0 {
                        cnt -= 1;
                    } else if let Some(b) = frames.recv() {
                        cur = vec![b.data.raw() as u8];
                        last = b.last.to_bool();
                        assert!(!b.err.to_bool(), "aborted frame sent to the MAC");
                        st = 1;
                        cnt = 1;
                    }
                }
                1 => {
                    if cnt == 15 {
                        o_sfd = true;
                        sfd_tick = now();
                    }
                    if cnt == 16 {
                        st = 2;
                    } else {
                        cnt += 1;
                    }
                }
                2 => st = 3,
                _ => {
                    if last {
                        self.log
                            .borrow_mut()
                            .push((std::mem::take(&mut cur), sfd_tick));
                        st = 0;
                        cnt = 23;
                    } else {
                        let b = frames
                            .recv()
                            .expect("mii_tx underrun: tx_path must release whole frames");
                        cur.push(b.data.raw() as u8);
                        last = b.last.to_bool();
                        assert!(!b.err.to_bool(), "aborted frame sent to the MAC");
                        st = 2;
                    }
                }
            }
        }
    }
}

/// `mii_rx` script for `frames` (wire bytes with FCS, idle cycles before
/// each, 15 preamble nibbles); returns it and the SFD edge of each frame.
fn rx_script(frames: &[(Vec<u8>, usize)]) -> (Vec<Act>, Vec<usize>) {
    let mut acts = vec![Act::default(); 8];
    let mut sfds = vec![];
    let mut t = 8usize;
    for (f, gap) in frames {
        let k = t + gap + 15;
        let n = f.len();
        let end = k + 2 * n + 1;
        acts.resize(end + 2, Act::default());
        acts[k].sfd = true;
        sfds.push(k);
        for (i, &d) in f.iter().enumerate() {
            let last = i + 1 == n;
            let at = if last { end } else { k + 4 + 2 * i };
            acts[at].byte = Some((d, last));
        }
        t = end;
    }
    (acts, sfds)
}

/// Runs `ArtyCore<R, T, Test, FixedSync>` with the MII models for `ticks`
/// ticks; returns the transmitted frames (FCS included) with the tick of
/// their SFD edge, and the tick of each received frame's SFD edge.
fn run_core<R: Clock, T: Clock>(
    frames: &[(Vec<u8>, usize)],
    ticks: u64,
) -> (Vec<(Vec<u8>, u64)>, Vec<u64>) {
    let wire: Vec<(Vec<u8>, usize)> = frames.iter().map(|(f, g)| (with_fcs(f), *g)).collect();
    let (acts, _) = rx_script(&wire);
    let log = Rc::new(RefCell::new(vec![]));
    let sfd_ticks = Rc::new(RefCell::new(vec![]));
    let (raw_tx, raw_rx) = chan::<FrameByte, R>();
    let (rsfd_o, rsfd) = signal::<Bit, R>();
    let (txf_tx, txf_rx) = chan::<FrameByte, T>();
    let (tsfd_o, tsfd) = signal::<Bit, T>();
    let (und_o, und) = signal::<Bit, T>();
    let mut rxm = MiiRxModel {
        acts,
        sfd_ticks: sfd_ticks.clone(),
    };
    let mut txm = MiiTxModel { log: log.clone() };
    let mut core = ArtyCore::<R, T, Test, FixedSync>::default();
    start_clock(&mut core.clock);
    // Simulated time runs on across the runs of one test thread: ticks are
    // made relative to this run's start, a clk_sys edge (even tick).
    let t0 = now();
    assert_eq!(t0 % 2, 0);
    {
        let mut sim = Running::new(join2(
            join2(
                <MiiRxModel as Unit<(), (Tx<FrameByte, R>, Out<Bit, R>)>>::run(
                    &mut rxm,
                    (),
                    (raw_tx, rsfd_o),
                ),
                <MiiTxModel as Unit<Rx<FrameByte, T>, (Out<Bit, T>, Out<Bit, T>)>>::run(
                    &mut txm,
                    txf_rx,
                    (tsfd_o, und_o),
                ),
            ),
            core.run((raw_rx, rsfd, tsfd, und), txf_tx),
        ));
        for _ in 0..ticks + ticks % 2 {
            sim.step();
        }
    }
    let tx = log
        .borrow()
        .iter()
        .map(|(f, t)| (f.clone(), t - t0))
        .collect();
    let rx = sfd_ticks.borrow().iter().map(|t| t - t0).collect();
    (tx, rx)
}

/// Statistics of the timestamp errors over several phases, in ns.
struct Errs {
    t2: Vec<f64>,
    t3: Vec<f64>,
}

/// NTP over IPv4 and IPv6 through the core at clocks `R`, `T`: replies
/// byte-exact (modulo timestamps), T2 and T3 against the SFD instants.
fn ntp_through_core<R: Clock, T: Clock>(errs: &mut Errs) {
    let v4 = exchanges("ntp4_basic.hex").remove(0);
    let v6 = exchanges("ntp6_basic.hex").remove(0);
    // Each request alone on the wire and in the server (gap 1500 clk_rx).
    let frames = vec![
        (v4.0.clone(), 20),
        (v6.0.clone(), 1500),
        (v4.0.clone(), 1500),
    ];
    let wants = [&v4.1[0], &v6.1[0], &v4.1[0]];
    let ticks = (8 + 3 * (1500 + 15 + 2 * 120 + 10)) as u64 * R::PERIOD + 4000;
    let (tx, rx_sfd) = run_core::<R, T>(&frames, ticks);
    assert_eq!(tx.len(), 3, "three replies");
    for (i, ((f, sfd_tick), w)) in tx.iter().zip(wants).enumerate() {
        let what = format!("{}/{} reply {i}", R::NAME, T::NAME);
        let (t2, t3) = check_reply(f, w, &what).expect("an NTP reply");
        // Reference instants: the clk_rx edge that samples the SFD nibble,
        // the clk_tx edge that puts the SFD's 0xD nibble on txd.
        let e2 = diff_ns(t2, ntp_at(rx_sfd[i]));
        let e3 = diff_ns(t3, ntp_at(*sfd_tick));
        println!("{what}: T2 - rx SFD = {e2:+.1} ns, T3 - tx SFD = {e3:+.1} ns");
        errs.t2.push(e2);
        errs.t3.push(e3);
    }
}

fn span(v: &[f64]) -> (f64, f64) {
    v.iter()
        .fold((f64::MAX, f64::MIN), |(a, b), &x| (a.min(x), b.max(x)))
}

/// The timestamp errors at 25 MHz MII clocks (4 clk_sys periods), five
/// phase sets (MII edges between or on `clk_sys` edges): |T2 error| <= 50 ns (rx_path's budget: 40 ns MII granularity
/// and +-10 ns crossing) and |T3 error| < 100 ns (the design target).
#[test]
fn t3_error_at_mii_granularity() {
    let mut e = Errs {
        t2: vec![],
        t3: vec![],
    };
    ntp_through_core::<Rx40a, Tx40a>(&mut e);
    ntp_through_core::<Rx40b, Tx40b>(&mut e);
    ntp_through_core::<Rx40c, Tx40c>(&mut e);
    ntp_through_core::<Rx40d, Tx40d>(&mut e);
    ntp_through_core::<Rx40e, Tx40e>(&mut e);
    let (a2, b2) = span(&e.t2);
    let (a3, b3) = span(&e.t3);
    println!("25 MHz MII: T2 error {a2:+.1}..{b2:+.1} ns, T3 error {a3:+.1}..{b3:+.1} ns");
    for x in &e.t2 {
        assert!(x.abs() <= 50.0, "T2 error {x} ns");
    }
    for x in &e.t3 {
        assert!(x.abs() < 100.0, "T3 error {x} ns");
    }
}

/// MII clocks off the nominal ratio (45 ns receive, 35 ns transmit): the
/// replies are still byte-exact; the timestamp errors are printed (the
/// stamp biases assume 40 ns).
#[test]
fn other_clock_ratio() {
    let mut e = Errs {
        t2: vec![],
        t3: vec![],
    };
    ntp_through_core::<Rx45, Tx35>(&mut e);
    let (a2, b2) = span(&e.t2);
    let (a3, b3) = span(&e.t3);
    println!("45/35 ns MII: T2 error {a2:+.1}..{b2:+.1} ns, T3 error {a3:+.1}..{b3:+.1} ns");
    for x in e.t2.iter().chain(&e.t3) {
        assert!(x.abs() < 200.0, "timestamp error {x} ns");
    }
}

// ------------------------------------------------------------ netlist

/// Both builds lower to one module tree; after the `rst` fix-up no module
/// declares a net with the name of one of its ports.
#[test]
fn lowers_to_verilog() {
    use ntp_fpga::top::{fix_rst_clash, ArtyBoard, ArtyBoardWithClient};
    for (name, v) in [
        ("board", fix_rst_clash(&ArtyBoard::verilog("arty_top"))),
        (
            "with client",
            fix_rst_clash(&ArtyBoardWithClient::verilog("arty_top")),
        ),
    ] {
        assert!(v.contains("module arty_top("), "{name}: top module");
        assert!(
            v.contains("module arty_top_txhdl_chan"),
            "{name}: registered channel module"
        );
        for m in [
            "mii_rx",
            "mii_tx",
            "phy",
            "core_server",
            "core_route",
            "core_arb4",
            "core_arb6",
            "core_tx_path",
        ] {
            assert!(
                v.contains(&format!("module arty_top_{m}(")),
                "{name}: module {m}"
            );
        }
        for module in v.split("endmodule").filter(|m| m.contains("module ")) {
            let (head, body) = module.split_at(module.find(");").unwrap());
            let head = &head[head.find("module ").unwrap()..];
            let head = &head[head.rfind('(').unwrap() + 1..];
            let ports: Vec<&str> = head
                .split(',')
                .filter_map(|p| {
                    p.split(|c: char| !(c.is_ascii_alphanumeric() || c == '_'))
                        .rfind(|w| !w.is_empty())
                })
                .collect();
            for line in body.lines() {
                let l = line.trim();
                let Some(rest) = l.strip_prefix("wire ").or_else(|| l.strip_prefix("reg ")) else {
                    continue;
                };
                let rest = rest.trim_start();
                let rest = if rest.starts_with('[') {
                    &rest[rest.find(']').unwrap() + 1..]
                } else {
                    rest
                };
                let id: String = rest
                    .trim_start()
                    .chars()
                    .take_while(|c| c.is_ascii_alphanumeric() || *c == '_')
                    .collect();
                assert!(
                    !ports.contains(&id.as_str()),
                    "{name}: net {id} redeclares a port"
                );
            }
        }
    }
    let with = ArtyBoardWithClient::verilog("arty_top");
    for m in ["filter", "selection", "fork", "servo", "state"] {
        assert!(
            with.contains(&format!("module arty_top_core_sync_{m}(")),
            "with client: {m}"
        );
    }
}

// ------------------------------------------------------------ arbiter

/// `IpArb4` with both sources pending at once, twice each, and a consumer
/// that stalls every third cycle: packets alternate, each request is
/// followed by exactly its own payload, nothing is lost or interleaved.
#[test]
fn ip_arb_alternates_whole_packets() {
    use ntp_fpga::ip_arb::IpArb4;
    use ntp_fpga::ipv4_tx::IpTxReq;
    let (r0_tx, r0_rx) = chan::<IpTxReq, txhdl::comp::DefaultClock>();
    let (p0_tx, p0_rx) = chan::<FrameByte, txhdl::comp::DefaultClock>();
    let (r1_tx, r1_rx) = chan::<IpTxReq, txhdl::comp::DefaultClock>();
    let (p1_tx, p1_rx) = chan::<FrameByte, txhdl::comp::DefaultClock>();
    let (q_tx, q_rx) = chan::<IpTxReq, txhdl::comp::DefaultClock>();
    let (y_tx, y_rx) = chan::<FrameByte, txhdl::comp::DefaultClock>();
    let mut arb = IpArb4::default();
    let mut sim = Running::new(arb.run((r0_rx, p0_rx, r1_rx, p1_rx), (q_tx, y_tx)));
    // Source s, packet k: request len = 3 + k, payload bytes 16 s + i.
    let mut todo = [vec![], vec![]];
    for (s, t) in todo.iter_mut().enumerate() {
        for k in 0..2u16 {
            t.push((
                IpTxReq {
                    len: U::from(3 + k),
                    proto: U::from(s as u8),
                    ..Default::default()
                },
                (0..3 + k)
                    .map(|i| (16 * s as u16 + i) as u8)
                    .collect::<Vec<u8>>(),
            ));
        }
    }
    let mut pos = [(0usize, None::<usize>), (0, None)]; // (packet, byte in flight)
    let mut out: Vec<(u8, Vec<u8>)> = vec![];
    let mut cur = 0;
    for t in 0..200 {
        for s in 0..2 {
            let (rt, pt) = if s == 0 {
                (&r0_tx, &p0_tx)
            } else {
                (&r1_tx, &p1_tx)
            };
            let (k, b) = pos[s];
            if k >= todo[s].len() {
                continue;
            }
            let (req, bytes) = &todo[s][k];
            match b {
                None => {
                    if rt.ready().to_bool() {
                        rt.send(*req);
                        pos[s].1 = Some(0);
                    }
                }
                Some(i) => {
                    if pt.ready().to_bool() {
                        let last = i + 1 == bytes.len();
                        pt.send(FrameByte::new(bytes[i], last, false));
                        pos[s] = if last {
                            (k + 1, None)
                        } else {
                            (k, Some(i + 1))
                        };
                    }
                }
            }
        }
        if t % 3 != 2 {
            if let Some(q) = q_rx.recv() {
                out.push((q.proto.raw() as u8, vec![]));
            }
            if let Some(b) = y_rx.recv() {
                // Bytes belong to the oldest packet whose last byte has
                // not come yet (its request was sent first).
                assert!(cur < out.len(), "payload before its request");
                out[cur].1.push(b.data.raw() as u8);
                if b.last.to_bool() {
                    cur += 1;
                }
            }
        }
        sim.cycle();
    }
    let sources: Vec<u8> = out.iter().map(|o| o.0).collect();
    assert_eq!(
        sources,
        vec![1, 0, 1, 0],
        "round robin, source 1 first after reset"
    );
    for (s, bytes) in &out {
        assert_eq!(bytes[0] / 16, *s, "payload of the granted source");
    }
    let want: Vec<Vec<u8>> = [(1, 0), (0, 0), (1, 1), (0, 1)]
        .iter()
        .map(|&(src, k)| todo[src][k].1.clone())
        .collect();
    assert_eq!(out.iter().map(|o| o.1.clone()).collect::<Vec<_>>(), want);
}
