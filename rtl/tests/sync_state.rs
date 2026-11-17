// SPDX-License-Identifier: Apache-2.0
//! Tests of `SyncState` (T4.6). Expected values are literals produced by
//! the Python reference `tools/golden/ntpclient/sync.py` (see
//! `tools/golden/ntpclient/tests/test_sync.py`).
use ntp_fpga::ntp_types::Selected;
use ntp_fpga::sync_state::{SyncState, SYNC_TIMEOUT_Q32};
use txhdl::comp::{chan, signal, tie, DefaultClock, In, Out, Running, Tx, Unit};
use txhdl::types::{Bit, U};

const REFS: [u32; 4] = [0xC000_0263, 0xB70A_6091, 0x1111_1111, 0x2222_2222];
const PREC: u8 = 0xE8; // -24
/// 5000 s, 32.32.
const NOW: u64 = 5000 << 32;
/// Cycles for a change to cross the three pipeline stages and settle.
const SETTLE: usize = 8;

struct Bench {
    sim: Running<std::pin::Pin<Box<dyn std::future::Future<Output = ()>>>>,
    sel: Tx<Selected>,
    time: Out<U<96>>,
    servo: Out<Bit>,
    prec: Out<U<8>>,
    synced: In<Bit>,
    stratum: In<U<8>>,
    ref_id: In<U<32>>,
    ref_ts: In<U<64>>,
    root_delay: In<U<32>>,
    root_disp: In<U<32>>,
    leap: In<U<2>>,
}

#[derive(Debug, PartialEq, Clone, Copy)]
struct Snap {
    synced: bool,
    stratum: u8,
    ref_id: u32,
    ref_ts: u64,
    root_delay: u32,
    root_disp: u32,
    leap: u8,
}

fn bench() -> Bench {
    let (sel, sel_r) = chan::<Selected, DefaultClock>();
    let (time, time_i) = signal::<U<96>, DefaultClock>();
    let (servo, servo_i) = signal::<Bit, DefaultClock>();
    let (prec, prec_i) = signal::<U<8>, DefaultClock>();
    let (synced_o, synced) = signal::<Bit, DefaultClock>();
    let (stratum_o, stratum) = signal::<U<8>, DefaultClock>();
    let (ref_id_o, ref_id) = signal::<U<32>, DefaultClock>();
    let (ref_ts_o, ref_ts) = signal::<U<64>, DefaultClock>();
    let (rd_o, root_delay) = signal::<U<32>, DefaultClock>();
    let (rs_o, root_disp) = signal::<U<32>, DefaultClock>();
    let (leap_o, leap) = signal::<U<2>, DefaultClock>();
    let d: &'static mut SyncState = Box::leak(Box::default());
    let fut: std::pin::Pin<Box<dyn std::future::Future<Output = ()>>> = Box::pin(d.run(
        (
            sel_r,
            time_i,
            servo_i,
            prec_i,
            tie(U::<32>::from(REFS[0])),
            tie(U::<32>::from(REFS[1])),
            tie(U::<32>::from(REFS[2])),
            tie(U::<32>::from(REFS[3])),
        ),
        (synced_o, stratum_o, ref_id_o, ref_ts_o, rd_o, rs_o, leap_o),
    ));
    let b = Bench {
        sim: Running::new(fut),
        sel,
        time,
        servo,
        prec,
        synced,
        stratum,
        ref_id,
        ref_ts,
        root_delay,
        root_disp,
        leap,
    };
    b.time.set(U::<96>::new((NOW as u128) << 32));
    b.servo.set(Bit::from_bool(true));
    b.prec.set(U::<8>::from(PREC));
    b
}

impl Bench {
    fn set_time(&mut self, t: u64) {
        self.time.set(U::<96>::new((t as u128) << 32));
    }
    fn run(&mut self, n: usize) {
        for _ in 0..n {
            self.sim.cycle();
        }
    }
    fn push(&mut self, s: Selected) {
        while !self.sel.ready().to_bool() {
            self.sim.cycle();
        }
        self.sel.send(s);
        self.sim.cycle();
    }
    fn settle(&mut self) -> Snap {
        self.run(SETTLE);
        self.snap()
    }
    fn snap(&self) -> Snap {
        Snap {
            synced: self.synced.get().to_bool(),
            stratum: self.stratum.get().raw() as u8,
            ref_id: self.ref_id.get().raw() as u32,
            ref_ts: self.ref_ts.get().raw() as u64,
            root_delay: self.root_delay.get().raw() as u32,
            root_disp: self.root_disp.get().raw() as u32,
            leap: self.leap.get().raw() as u8,
        }
    }
}

/// The reference selection of the sync.py tests: stratum 1, peer 0,
/// root delay 0x333, root disp 0x666, delay 0x1234567, disp 0x100000,
/// jitter 0x200000, t4 = NOW.
fn sel() -> Selected {
    Selected {
        valid: Bit::from_bool(true),
        peer: U::<2>::from(0u8),
        stratum: U::<8>::from(1u8),
        root_delay: U::<32>::from(0x333u32),
        root_disp: U::<32>::from(0x666u32),
        delay: U::<64>::from(0x0123_4567u64),
        disp: U::<64>::from(0x10_0000u64),
        jitter: U::<64>::from(0x20_0000u64),
        t4: U::<64>::from(NOW),
        ..Default::default()
    }
}

fn unsynced(s: Snap) -> bool {
    !s.synced && s.stratum == 16 && s.leap == 3
}

#[test]
fn reset_is_unsynchronised() {
    let mut b = bench();
    let s = b.settle();
    assert!(unsynced(s));
    assert_eq!((s.ref_id, s.ref_ts, s.root_delay, s.root_disp), (0, 0, 0, 0));
}

#[test]
fn synchronises_on_a_valid_selected() {
    let mut b = bench();
    let mut x = sel();
    x.peer = U::<2>::from(1u8);
    b.push(x);
    let s = b.settle();
    assert_eq!(
        s,
        Snap {
            synced: true,
            stratum: 2,
            ref_id: REFS[1],
            ref_ts: NOW,
            root_delay: 0x333 + 0x124, // ceil16(0x1234567) = 0x124
            root_disp: 1686,           // 0x666 + ceil16(0x300000) = 0x666 + 0x30
            leap: 0,
        }
    );
}

#[test]
fn peer_index_selects_the_ref_id() {
    for p in 0..4u8 {
        let mut b = bench();
        let mut x = sel();
        x.peer = U::<2>::from(p);
        b.push(x);
        assert_eq!(b.settle().ref_id, REFS[p as usize]);
    }
}

#[test]
fn invalid_and_out_of_range_strata_are_unsynchronised() {
    for (valid, stratum, sync, out_st) in [
        (false, 1u8, false, 16u8),
        (true, 0, false, 16),
        (true, 1, true, 2),
        (true, 14, true, 15),
        (true, 15, false, 16),
        (true, 16, false, 16),
        (true, 255, false, 16),
    ] {
        let mut b = bench();
        let mut x = sel();
        x.valid = Bit::from_bool(valid);
        x.stratum = U::<8>::from(stratum);
        b.push(x);
        let s = b.settle();
        assert_eq!(s.synced, sync, "valid {valid} stratum {stratum}");
        assert_eq!(s.stratum, out_st);
        assert_eq!(s.leap, if sync { 0 } else { 3 });
    }
}

#[test]
fn an_unusable_selected_clears_sync_and_keeps_the_held_values() {
    let mut b = bench();
    let mut x = sel();
    x.peer = U::<2>::from(1u8);
    b.push(x);
    let good = b.settle();
    assert!(good.synced);
    b.set_time(NOW + (10 << 32));
    let mut bad = sel();
    bad.valid = Bit::from_bool(false);
    bad.peer = U::<2>::from(2u8);
    bad.stratum = U::<8>::from(3u8);
    b.push(bad);
    let s = b.settle();
    assert!(unsynced(s));
    // ref_id, ref_ts and roots keep their last synchronised value.
    assert_eq!(
        (s.ref_id, s.ref_ts, s.root_delay, s.root_disp),
        (good.ref_id, good.ref_ts, good.root_delay, 1696) // +10 s of age: 1686 + 10
    );
    // A new usable Selected brings it back, with its own update time.
    b.push(sel());
    let s = b.settle();
    assert!(s.synced);
    assert_eq!((s.ref_id, s.ref_ts), (REFS[0], NOW + (10 << 32)));
}

#[test]
fn servo_not_synced_is_unsynchronised() {
    let mut b = bench();
    b.push(sel());
    assert!(b.settle().synced);
    b.servo.set(Bit::from_bool(false));
    let s = b.settle();
    assert!(unsynced(s));
    assert_eq!(s.ref_id, REFS[0]); // held
    b.servo.set(Bit::from_bool(true));
    assert!(b.settle().synced);
}

#[test]
fn timeout_without_update() {
    let mut b = bench();
    b.push(sel());
    assert!(b.settle().synced);
    // Exactly at the limit: still synchronised.
    b.set_time(NOW + SYNC_TIMEOUT_Q32);
    assert!(b.settle().synced);
    // One 2^-32 s past it: not.
    b.time.set(U::<96>::new(((NOW + SYNC_TIMEOUT_Q32 + 1) as u128) << 32));
    let s = b.settle();
    assert!(unsynced(s));
    assert_eq!(s.ref_ts, NOW);
    // A fresh Selected (t4 moved on) revives it.
    let mut x = sel();
    x.t4 = U::<64>::from(NOW + SYNC_TIMEOUT_Q32 + 1);
    b.push(x);
    let s = b.settle();
    assert!(s.synced);
    assert_eq!(s.ref_ts, NOW + SYNC_TIMEOUT_Q32 + 1);
}

#[test]
fn a_stalled_input_keeps_the_state_until_the_timeout() {
    let mut b = bench();
    b.push(sel());
    let first = b.settle();
    // 300 cycles with nothing on the channel: nothing changes.
    b.run(300);
    assert_eq!(b.snap(), first);
}

#[test]
fn dispersion_grows_with_time() {
    let mut b = bench();
    b.push(sel());
    assert_eq!(b.settle().root_disp, 1686);
    b.set_time(NOW + (100 << 32));
    assert_eq!(b.settle().root_disp, 1785);
    b.set_time(NOW + (1000 << 32));
    let s = b.settle();
    assert!(s.synced);
    assert_eq!(s.root_disp, 2670);
    // The root delay does not grow.
    assert_eq!(s.root_delay, 0x457);
    // Same value from an older sample (t4 1000 s before, time unchanged).
    let mut b = bench();
    let mut x = sel();
    x.t4 = U::<64>::from(NOW - (1000 << 32));
    b.push(x);
    assert_eq!(b.settle().root_disp, 2670);
}

#[test]
fn dispersion_with_a_large_age() {
    // t4 = 0, time = 100000 s: age 100000 s (the Selected is fresh: it is
    // popped at that time).
    let mut b = bench();
    b.set_time(100_000u64 << 32);
    let mut x = sel();
    x.t4 = U::<64>::from(0u64);
    x.root_delay = U::<32>::from(0u32);
    x.root_disp = U::<32>::from(0u32);
    x.disp = U::<64>::from(0u64);
    x.jitter = U::<64>::from(0u64);
    b.push(x);
    let s = b.settle();
    assert!(s.synced);
    assert_eq!(s.root_disp, 98305);
}

#[test]
fn dispersion_saturates() {
    // Sum of the terms: 0xFFFFFFF0 + disp 2^40 (= 2^24 units of 16.16).
    let mut b = bench();
    let mut x = sel();
    x.root_disp = U::<32>::from(0xFFFF_FFF0u32);
    x.disp = U::<64>::from(1u64 << 40);
    b.push(x);
    assert_eq!(b.settle().root_disp, 0xFFFF_FFFF);
    // The age term alone saturating: t4 = 0, 4e9 s old, base near the top.
    let mut b = bench();
    b.set_time(4_000_000_000u64 << 32);
    let mut x = sel();
    x.t4 = U::<64>::from(0u64);
    x.root_disp = U::<32>::from(0u32);
    x.disp = U::<64>::from(0u64);
    x.jitter = U::<64>::from(0u64);
    b.push(x);
    assert_eq!(b.settle().root_disp, 3_932_189_942);
    let mut b = bench();
    b.set_time(4_000_000_000u64 << 32);
    let mut x = sel();
    x.t4 = U::<64>::from(0u64);
    x.root_disp = U::<32>::from(1_000_000_000u32);
    x.disp = U::<64>::from(0u64);
    x.jitter = U::<64>::from(0u64);
    b.push(x);
    assert_eq!(b.settle().root_disp, 0xFFFF_FFFF);
    // Root delay saturates too: 0xFFFFFFFF + 1 s.
    let mut b = bench();
    let mut x = sel();
    x.root_delay = U::<32>::from(0xFFFF_FFFFu32);
    x.delay = U::<64>::from(1u64 << 32);
    b.push(x);
    assert_eq!(b.settle().root_delay, 0xFFFF_FFFF);
}

#[test]
fn jitter_floor_from_precision() {
    // disp = jitter = 0, age 0: the root dispersion is root_disp + ceil16(2^p).
    for (p, extra) in [(0xE8u8, 1u32), (0, 65536), (0xFF, 32768), (0xE0, 1), (0xDF, 1), (0x80, 1)] {
        let mut b = bench();
        b.prec.set(U::<8>::from(p));
        let mut x = sel();
        x.disp = U::<64>::from(0u64);
        x.jitter = U::<64>::from(0u64);
        x.root_disp = U::<32>::from(0u32);
        b.push(x);
        assert_eq!(b.settle().root_disp, extra, "precision {p:#x}");
    }
    // A jitter above the floor wins: 2^32 > 2^-24.
    let mut b = bench();
    let mut x = sel();
    x.disp = U::<64>::from(0u64);
    x.jitter = U::<64>::from(1u64 << 32);
    x.root_disp = U::<32>::from(0u32);
    b.push(x);
    assert_eq!(b.settle().root_disp, 65536);
}

#[test]
fn a_clock_stepped_back_counts_as_age_zero() {
    let mut b = bench();
    let mut x = sel();
    x.t4 = U::<64>::from(NOW + (50 << 32)); // t4 in the future of `time`
    b.push(x);
    let s = b.settle();
    assert!(s.synced);
    assert_eq!(s.root_disp, 1686);
}

#[test]
fn the_pipeline_depth() {
    let mut b = bench();
    b.push(sel());
    let mut n = 0;
    while !b.snap().synced {
        b.run(1);
        n += 1;
        assert!(n < 20);
    }
    // 2 edges of input channel latency + 3 pipeline edges after the pop.
    assert_eq!(n, 5);
}

#[test]
fn lowers_to_verilog() {
    let v = SyncState::verilog("sync_state");
    assert!(v.contains("module sync_state"));
}
