// SPDX-License-Identifier: Apache-2.0
//! Tests of the RFC 5905 clock filter (T4.3): replay of the Python model's
//! vectors (`tools/golden/ntpclient/vectors/filter.txt`), multi-peer
//! interleaving, back-to-back events, a stalled consumer, extreme offsets,
//! delay ties, the cycles per event against the documented bound, and the
//! Verilog lowering.
use ntp_fpga::clock_filter::{ClockFilter, MAXDISP, MAX_EVENT_CYCLES};
use ntp_fpga::ntp_types::{PeerStat, Sample};
use txhdl::comp::{chan, DefaultClock, Running, Unit};
use txhdl::types::U;

const SEC: u64 = 1 << 32;
const MS: u64 = SEC / 1000;
const T0: u64 = 0xEC00_0000 * SEC;

#[derive(Clone, Debug, PartialEq, Eq)]
struct Smp {
    peer: u8,
    offset: u64,
    delay: u64,
    disp: u64,
    t4: u64,
    stratum: u8,
    leap: u8,
    ref_id: u32,
    rd: u32,
    rdisp: u32,
}

impl Smp {
    fn new(peer: u8, offset: i64, delay: u64, disp: u64, t4: u64) -> Smp {
        Smp {
            peer,
            offset: offset as u64,
            delay,
            disp,
            t4,
            stratum: 2,
            leap: 0,
            ref_id: 0xC000_0201,
            rd: 0x123,
            rdisp: 0x456,
        }
    }
    fn to_sample(&self) -> Sample {
        Sample {
            peer: U::<2>::from(self.peer),
            offset: U::<64>::from(self.offset),
            delay: U::<64>::from(self.delay),
            disp: U::<64>::from(self.disp),
            t4: U::<64>::from(self.t4),
            stratum: U::<8>::from(self.stratum),
            leap: U::<2>::from(self.leap),
            ref_id: U::<32>::from(self.ref_id),
            root_delay: U::<32>::from(self.rd),
            root_disp: U::<32>::from(self.rdisp),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
enum Ev {
    S(Smp),
    M(u8),
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct Stat {
    peer: u8,
    valid: bool,
    offset: u64,
    delay: u64,
    disp: u64,
    jitter: u64,
    reach: u8,
    stratum: u8,
    leap: u8,
    ref_id: u32,
    rd: u32,
    rdisp: u32,
    t4: u64,
}

fn stat_of(p: &PeerStat) -> Stat {
    Stat {
        peer: p.peer.raw() as u8,
        valid: p.valid.to_bool(),
        offset: p.offset.raw() as u64,
        delay: p.delay.raw() as u64,
        disp: p.disp.raw() as u64,
        jitter: p.jitter.raw() as u64,
        reach: p.reach.raw() as u8,
        stratum: p.stratum.raw() as u8,
        leap: p.leap.raw() as u8,
        ref_id: p.ref_id.raw() as u32,
        rd: p.root_delay.raw() as u32,
        rdisp: p.root_disp.raw() as u32,
        t4: p.t4.raw() as u64,
    }
}

// ------------------------------------------------------------ vectors

fn num(s: &str) -> u64 {
    match s.strip_prefix("0x") {
        Some(h) => u64::from_str_radix(h, 16).unwrap(),
        None => s.parse().unwrap(),
    }
}

fn parse_vectors() -> Vec<(Ev, Stat)> {
    let p = format!(
        "{}/../tools/golden/ntpclient/vectors/filter.txt",
        env!("CARGO_MANIFEST_DIR")
    );
    let text = std::fs::read_to_string(&p).unwrap_or_else(|e| panic!("{p}: {e}"));
    let mut v = vec![];
    for l in text.lines() {
        let l = l.trim();
        if l.is_empty() || l.starts_with('#') {
            continue;
        }
        let (a, b) = l.split_once(" -> ").unwrap();
        let a: Vec<&str> = a.split_whitespace().collect();
        let b: Vec<u64> = b.split_whitespace().map(num).collect();
        let ev = match a[0] {
            "sample" => Ev::S(Smp {
                peer: num(a[1]) as u8,
                offset: num(a[2]),
                delay: num(a[3]),
                disp: num(a[4]),
                t4: num(a[5]),
                stratum: num(a[6]) as u8,
                leap: num(a[7]) as u8,
                ref_id: num(a[8]) as u32,
                rd: num(a[9]) as u32,
                rdisp: num(a[10]) as u32,
            }),
            "miss" => Ev::M(num(a[1]) as u8),
            k => panic!("unknown event {k}"),
        };
        let st = Stat {
            peer: b[0] as u8,
            valid: b[1] == 1,
            offset: b[2],
            delay: b[3],
            disp: b[4],
            jitter: b[5],
            reach: b[6] as u8,
            stratum: b[7] as u8,
            leap: b[8] as u8,
            ref_id: b[9] as u32,
            rd: b[10] as u32,
            rdisp: b[11] as u32,
            t4: b[12],
        };
        v.push((ev, st));
    }
    v
}

// ------------------------------------------------------------ harness

#[derive(Debug)]
struct Result {
    stat: Stat,
    /// Cycle `send` was called / cycle `recv` returned the PeerStat.
    sent: usize,
    got: usize,
}

/// Plays `evs` in order. `seq`: one event at a time (the next is sent when the
/// previous PeerStat is back); otherwise as fast as the channels allow, but a
/// sample is never sent behind a miss (or the reverse) that has not been
/// answered (the unit takes a sample before a miss when both wait, so the
/// order across the two channels is only defined for an idle unit). `take(t)`:
/// the consumer reads `stats` in cycle t.
fn run(evs: &[Ev], seq: bool, take: impl Fn(usize) -> bool) -> Vec<Result> {
    let (st_t, st_r) = chan::<Sample, DefaultClock>();
    let (mi_t, mi_r) = chan::<U<2>, DefaultClock>();
    let (ou_t, ou_r) = chan::<PeerStat, DefaultClock>();
    let mut dut = ClockFilter::default();
    let mut sim = Running::new(dut.run((st_r, mi_r), ou_t));
    let mut out: Vec<Result> = vec![];
    let mut sent_at: Vec<usize> = vec![];
    let mut next = 0usize;
    let mut t = 0usize;
    while out.len() < evs.len() {
        assert!(
            t < 3_000 * (evs.len() + 1),
            "timeout: {} of {}",
            out.len(),
            evs.len()
        );
        if next < evs.len() {
            let answered = out.len() == next;
            let kind_ok = next == 0
                || answered
                || std::mem::discriminant(&evs[next]) == std::mem::discriminant(&evs[next - 1]);
            if kind_ok && (!seq || answered) {
                match &evs[next] {
                    Ev::S(s) => {
                        if st_t.ready().to_bool() {
                            st_t.send(s.to_sample());
                            sent_at.push(t);
                            next += 1;
                        }
                    }
                    Ev::M(p) => {
                        if mi_t.ready().to_bool() {
                            mi_t.send(U::<2>::from(*p));
                            sent_at.push(t);
                            next += 1;
                        }
                    }
                }
            }
        }
        if take(t) {
            if let Some(p) = ou_r.recv() {
                let k = out.len();
                out.push(Result {
                    stat: stat_of(&p),
                    sent: sent_at[k],
                    got: t,
                });
            }
        }
        sim.cycle();
        t += 1;
    }
    out
}

fn check_all(evs: &[(Ev, Stat)], res: &[Result]) {
    assert_eq!(evs.len(), res.len());
    for (i, ((ev, want), r)) in evs.iter().zip(res).enumerate() {
        assert_eq!(&r.stat, want, "event {i}: {ev:?}");
    }
}

fn evs_of(v: &[(Ev, Stat)]) -> Vec<Ev> {
    v.iter().map(|(e, _)| e.clone()).collect()
}

// ------------------------------------------------------------ vector replay

#[test]
fn vectors_one_event_at_a_time_and_cycles() {
    let v = parse_vectors();
    assert!(v.len() >= 1000);
    let res = run(&evs_of(&v), true, |_| true);
    check_all(&v, &res);
    let lat: Vec<usize> = res.iter().map(|r| r.got - r.sent).collect();
    let max = *lat.iter().max().unwrap();
    let avg = lat.iter().sum::<usize>() as f64 / lat.len() as f64;
    println!("cycles per event: max {max}, mean {avg:.1} (bound {MAX_EVENT_CYCLES})");
    assert!(max as u64 <= MAX_EVENT_CYCLES);
}

#[test]
fn vectors_back_to_back() {
    // Events sent as fast as the channels take them (same-kind runs queue behind a
    // busy unit).
    let v = parse_vectors();
    let n = 400;
    let res = run(&evs_of(&v[..n]), false, |_| true);
    check_all(&v[..n], &res);
}

#[test]
fn vectors_with_a_stalled_consumer() {
    // The consumer reads one PeerStat in 700 cycles, in bursts: the unit waits in
    // its last state, the next events wait on their channels, nothing is lost.
    let v = parse_vectors();
    let n = 60;
    let res = run(&evs_of(&v[..n]), false, |t| {
        t % 700 < 2 || (t / 3000) % 2 == 1
    });
    check_all(&v[..n], &res);
    let res = run(&evs_of(&v[..n]), true, |t| t % 211 == 0);
    check_all(&v[..n], &res);
}

// ------------------------------------------------------------ hand-derived cases

fn tail(from_rank: u32) -> u64 {
    (from_rank..8).map(|r| MAXDISP >> (r + 1)).sum()
}

#[test]
fn single_sample() {
    let mut s = Smp::new(2, -5 * MS as i64, 7 * MS, 1 << 20, T0);
    s.stratum = 1;
    let r = run(&[Ev::S(s.clone())], true, |_| true);
    let want = Stat {
        peer: 2,
        valid: true,
        offset: s.offset,
        delay: 7 * MS,
        disp: (1 << 19) + tail(1),
        jitter: 0,
        reach: 1,
        stratum: 1,
        leap: 0,
        ref_id: s.ref_id,
        rd: s.rd,
        rdisp: s.rdisp,
        t4: T0,
    };
    assert_eq!(r[0].stat, want);
}

#[test]
fn miss_on_a_fresh_peer_and_reach_register() {
    let mut evs = vec![Ev::M(1), Ev::S(Smp::new(1, 0, 5 * MS, 0, T0))];
    for _ in 0..8 {
        evs.push(Ev::M(1));
    }
    let r = run(&evs, true, |_| true);
    assert!(!r[0].stat.valid && r[0].stat.reach == 0 && r[0].stat.offset == 0 && r[0].stat.t4 == 0);
    assert!(r[1].stat.valid && r[1].stat.reach == 1);
    for k in 1..8 {
        assert!(r[1 + k].stat.valid);
        assert_eq!(r[1 + k].stat.reach, (1u32 << k) as u8);
        assert_eq!(
            r[1 + k].stat.delay,
            5 * MS,
            "the result is kept over the misses"
        );
    }
    assert!(!r[9].stat.valid);
    assert_eq!(r[9].stat.reach, 0);
}

#[test]
fn delay_tie_takes_the_newest() {
    let evs = [
        Ev::S(Smp::new(0, MS as i64, 10 * MS, 0, T0)),
        Ev::S(Smp::new(0, 2 * MS as i64, 10 * MS, 0, T0 + SEC)),
        Ev::S(Smp::new(0, 3 * MS as i64, 10 * MS, 0, T0 + 2 * SEC)),
    ];
    let r = run(&evs, true, |_| true);
    assert_eq!(r[0].stat.offset, MS);
    assert_eq!(r[1].stat.offset, 2 * MS);
    assert_eq!(r[2].stat.offset, 3 * MS);
    // jitter at the third: rms of (1 ms - 3 ms, 2 ms - 3 ms) over 2
    let d = [2 * MS as u128, MS as u128];
    let q = (d[0] * d[0] + d[1] * d[1]) / 2;
    assert_eq!(r[2].stat.jitter as u128, isqrt(q));
}

/// Exact floor square root.
fn isqrt(q: u128) -> u128 {
    let mut s = (q as f64).sqrt() as u128;
    while s * s > q {
        s -= 1;
    }
    while (s + 1) * (s + 1) <= q {
        s += 1;
    }
    s
}

#[test]
fn extreme_offsets() {
    let hi = (1u64 << 63) - 1; // +2^31 s - 1 LSB
    let lo = 1u64 << 63; // -2^31 s
    let evs = [
        Ev::S(Smp::new(0, hi as i64, 5 * MS, 0, T0)),
        Ev::S(Smp::new(0, lo as i64, 3 * MS, 0, T0 + SEC)),
        Ev::S(Smp::new(1, 0, 3 * MS, 0, T0)),
        Ev::S(Smp::new(1, 0, 2 * MS, 0, T0 + SEC)),
        Ev::S(Smp::new(2, -1, 1 * MS, 0, T0)),
    ];
    let r = run(&evs, true, |_| true);
    assert_eq!(r[0].stat.offset, hi);
    assert_eq!(r[1].stat.offset, lo);
    // |lo - hi| = 2^64 - 1 saturates at MAXDISP, not wrapped to 1
    assert_eq!(r[1].stat.jitter, MAXDISP);
    assert_eq!(r[3].stat.offset, 0);
    assert_eq!(r[3].stat.jitter, 0);
    assert_eq!(r[4].stat.offset, u64::MAX);
}

#[test]
fn four_peers_interleaved_are_independent() {
    let mut evs = vec![];
    for k in 0..6u64 {
        for p in 0..4u8 {
            // each peer has its own offsets, delays decreasing so every sample is accepted
            evs.push(Ev::S(Smp::new(
                p,
                (p as i64 * 10 + k as i64) * MS as i64,
                (20 - k) * MS + p as u64 * 100,
                1000 * (p as u64 + 1),
                T0 + k * 16 * SEC + p as u64,
            )));
        }
    }
    let r = run(&evs, false, |_| true);
    // reference: each peer alone, one event at a time
    for p in 0..4u8 {
        let alone: Vec<Ev> = evs
            .iter()
            .filter(|e| matches!(e, Ev::S(s) if s.peer == p))
            .cloned()
            .collect();
        let ra = run(&alone, true, |_| true);
        let mine: Vec<&Result> = r.iter().filter(|x| x.stat.peer == p).collect();
        assert_eq!(mine.len(), ra.len());
        for (a, b) in mine.iter().zip(&ra) {
            assert_eq!(a.stat, b.stat);
        }
    }
}

#[test]
fn worst_case_cycles_are_within_the_bound() {
    // 8 usable samples, each newer one better (accepted at every update), offsets
    // spread over hundreds of seconds so every squared difference is saturated and
    // the serial multiplier runs its full 37 steps.
    let mut evs = vec![];
    for k in 0..12u64 {
        let off = (k as i64) * 1000 * SEC as i64 * if k % 2 == 0 { 1 } else { -1 };
        evs.push(Ev::S(Smp::new(
            3,
            off,
            (100 - k) * MS,
            MAXDISP / 4,
            T0 + k * 16 * SEC,
        )));
    }
    let r = run(&evs, true, |_| true);
    let max = r.iter().map(|x| x.got - x.sent).max().unwrap();
    println!("worst-case event: {max} cycles (bound {MAX_EVENT_CYCLES})");
    assert!(max as u64 <= MAX_EVENT_CYCLES);
    assert!(
        max > 450,
        "the scenario should exercise the long path, got {max}"
    );
    assert_eq!(r[11].stat.jitter, MAXDISP);
}

#[test]
fn lowers_to_verilog() {
    let v = ClockFilter::verilog("clock_filter");
    assert!(v.contains("module clock_filter"));
}
