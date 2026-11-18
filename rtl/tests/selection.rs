// SPDX-License-Identifier: Apache-2.0
//! Cycle-by-cycle tests of the clock selection unit (T4.4): replay of the
//! Python reference vectors (`tools/golden/ntpclient/vectors/selection.txt`),
//! back-to-back events, stalled consumer, cycle count per event, lowering.
use ntp_fpga::ntp_types::{PeerStat, Selected};
use ntp_fpga::selection::{event_cycles, Selection, MAX_EVENT_CYCLES};
use txhdl::comp::{chan, signal, tie, DefaultClock, Running, Unit};
use txhdl::types::{Bit, U};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
struct Stat {
    peer: u64,
    valid: u64,
    offset: u64,
    delay: u64,
    disp: u64,
    jitter: u64,
    reach: u64,
    stratum: u64,
    leap: u64,
    ref_id: u64,
    root_delay: u64,
    root_disp: u64,
    t4: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
struct Sel {
    valid: u64,
    peer: u64,
    offset: u64,
    jitter: u64,
    root_dist: u64,
    stratum: u64,
    leap: u64,
    ref_id: u64,
    root_delay: u64,
    root_disp: u64,
    delay: u64,
    disp: u64,
    t4: u64,
}

struct Event {
    now: u64,
    stat: Stat,
    want: Sel,
}

struct Scenario {
    name: String,
    /// Enabled upstream slots (the unit's `n_en`, start-up majority F4).
    n_en: u64,
    events: Vec<Event>,
}

fn load() -> Vec<Scenario> {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../tools/golden/ntpclient/vectors/selection.txt"
    );
    let text = std::fs::read_to_string(path).unwrap();
    let mut out: Vec<Scenario> = vec![];
    let mut pending: Option<(u64, Stat)> = None;
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let w: Vec<&str> = line.split_whitespace().collect();
        let v: Vec<u64> = w[1..]
            .iter()
            .map(|x| u64::from_str_radix(x, 16).unwrap_or(0))
            .collect();
        match w[0] {
            "scenario" => out.push(Scenario {
                name: w[1].to_string(),
                n_en: w[2].parse().unwrap(),
                events: vec![],
            }),
            "stat" => {
                let s = Stat {
                    peer: v[1],
                    valid: v[2],
                    offset: v[3],
                    delay: v[4],
                    disp: v[5],
                    jitter: v[6],
                    reach: v[7],
                    stratum: v[8],
                    leap: v[9],
                    ref_id: v[10],
                    root_delay: v[11],
                    root_disp: v[12],
                    t4: v[13],
                };
                pending = Some((v[0], s));
            }
            "sel" => {
                let want = Sel {
                    valid: v[0],
                    peer: v[1],
                    offset: v[2],
                    jitter: v[3],
                    root_dist: v[4],
                    stratum: v[5],
                    leap: v[6],
                    ref_id: v[7],
                    root_delay: v[8],
                    root_disp: v[9],
                    delay: v[10],
                    disp: v[11],
                    t4: v[12],
                };
                let (now, stat) = pending.take().unwrap();
                out.last_mut()
                    .unwrap()
                    .events
                    .push(Event { now, stat, want });
            }
            _ => panic!("bad vector line {line}"),
        }
    }
    out
}

fn to_hw(s: &Stat) -> PeerStat {
    PeerStat {
        peer: U::<2>::from(s.peer),
        valid: Bit::from_bool(s.valid != 0),
        offset: U::<64>::from(s.offset),
        delay: U::<64>::from(s.delay),
        disp: U::<64>::from(s.disp),
        jitter: U::<64>::from(s.jitter),
        reach: U::<8>::from(s.reach),
        stratum: U::<8>::from(s.stratum),
        leap: U::<2>::from(s.leap),
        ref_id: U::<32>::from(s.ref_id),
        root_delay: U::<32>::from(s.root_delay),
        root_disp: U::<32>::from(s.root_disp),
        t4: U::<64>::from(s.t4),
    }
}

fn from_hw(s: &Selected) -> Sel {
    Sel {
        valid: s.valid.to_bool() as u64,
        peer: s.peer.raw() as u64,
        offset: s.offset.raw() as u64,
        jitter: s.jitter.raw() as u64,
        root_dist: s.root_dist.raw() as u64,
        stratum: s.stratum.raw() as u64,
        leap: s.leap.raw() as u64,
        ref_id: s.ref_id.raw() as u64,
        root_delay: s.root_delay.raw() as u64,
        root_disp: s.root_disp.raw() as u64,
        delay: s.delay.raw() as u64,
        disp: s.disp.raw() as u64,
        t4: s.t4.raw() as u64,
    }
}

/// One output: the Selected, the cycle its PeerStat was offered (sent into
/// the input channel) and the cycle it was received from the output channel.
struct Got {
    sel: Sel,
    sent: usize,
    recv: usize,
}

/// Channel overhead seen by the testbench: one cycle through the input
/// channel, one through the output channel.
const CHAN: usize = 2;

/// Drives the events through one fresh unit and returns one `Got` per event.
/// One at a time (`back_to_back` false): event i is offered `gap(i)` cycles
/// after output i - 1 is received, and `now` is event i's `now` from then
/// on. Back to back: every event is offered as soon as the input channel has
/// room; all events must then share one `now`. `take(t)`: the consumer is
/// ready in cycle t.
fn drive(
    n_en: u64,
    events: &[(u64, Stat)],
    back_to_back: bool,
    gap: impl Fn(usize) -> usize,
    take: impl Fn(usize) -> bool,
) -> Vec<Got> {
    if back_to_back {
        assert!(
            events.iter().all(|e| e.0 == events[0].0),
            "back to back needs one `now`"
        );
    }
    let (now_o, now) = signal::<U<64>, DefaultClock>();
    let (st_tx, st_rx) = chan::<PeerStat, DefaultClock>();
    let (sel_tx, sel_rx) = chan::<Selected, DefaultClock>();
    let mut dut = Selection::default();
    let n_en = tie::<U<3>, DefaultClock>(U::<3>::from(n_en as u8));
    let mut sim = Running::new(dut.run((st_rx, now, n_en), sel_tx));
    let mut out: Vec<Got> = vec![];
    let mut sent_at: Vec<usize> = vec![];
    let mut wait_until = gap(0);
    let limit = events.len() * 20_000 + 2000;
    let mut t = 0usize;
    while out.len() < events.len() {
        assert!(t < limit, "no output (deadlock)");
        let next = sent_at.len();
        now_o.set(U::<64>::from(events[out.len().min(events.len() - 1)].0));
        let can_offer =
            next < events.len() && t >= wait_until && (back_to_back || next == out.len());
        if can_offer && st_tx.ready().to_bool() {
            st_tx.send(to_hw(&events[next].1));
            sent_at.push(t);
        }
        if take(t) {
            if let Some(s) = sel_rx.recv() {
                let i = out.len();
                out.push(Got {
                    sel: from_hw(&s),
                    sent: sent_at[i],
                    recv: t,
                });
                if !back_to_back {
                    wait_until = t + 1 + gap(out.len());
                }
            }
        }
        sim.cycle();
        t += 1;
    }
    // Nothing more comes out.
    for _ in 0..1000 {
        assert!(sel_rx.recv().is_none(), "an output without a PeerStat");
        sim.cycle();
    }
    out
}

fn events_of(s: &Scenario) -> Vec<(u64, Stat)> {
    s.events.iter().map(|e| (e.now, e.stat)).collect()
}

fn find<'a>(sc: &'a [Scenario], name: &str) -> &'a Scenario {
    sc.iter()
        .find(|s| s.name == name)
        .unwrap_or_else(|| panic!("scenario {name}"))
}

fn check(s: &Scenario, got: &[Got]) {
    assert_eq!(got.len(), s.events.len());
    for (i, e) in s.events.iter().enumerate() {
        assert_eq!(got[i].sel, e.want, "scenario {} event {}", s.name, i);
    }
}

fn always(_: usize) -> bool {
    true
}

#[test]
fn lowers_to_verilog() {
    let v = Selection::verilog("selection");
    assert!(v.contains("module selection"));
}

#[test]
fn reference_vectors() {
    let sc = load();
    assert!(sc.len() > 300);
    let mut n = 0;
    for s in &sc {
        let got = drive(s.n_en, &events_of(s), false, |_| 0, always);
        check(s, &got);
        for g in &got {
            let l = (g.recv - g.sent - CHAN) as u64;
            assert!(l <= MAX_EVENT_CYCLES, "{}: {} cycles", s.name, l);
        }
        n += got.len();
    }
    assert!(n > 1000);
}

/// The cycle count of an event depends only on the number of survivors, as
/// `event_cycles` says, and stays within `MAX_EVENT_CYCLES`.
#[test]
fn cycles_per_event() {
    let sc = load();
    // (scenario, survivors of the last event, 4 truechimers)
    let cases = [
        ("two_disagree", 0, false),
        ("leap3_only", 0, false),
        ("single", 1, false),
        ("two_agree", 2, false),
        ("four_one_false", 3, false),
        ("cluster_drop", 3, true),
        ("cluster_keep", 4, true),
        ("widest_spread4_keep", 4, true),
        // F4: the start-up majority is not met (2 truechimers of 4 enabled
        // slots): no selection, in the cycles of an event without one.
        ("startup_gate_fails", 0, false),
        ("startup_four_cluster", 3, true),
    ];
    let mut max = 0;
    for (name, n, tc4) in cases {
        let s = find(&sc, name);
        let got = drive(s.n_en, &events_of(s), false, |i| 3 * i, always);
        check(s, &got);
        let g = got.last().unwrap();
        let l = (g.recv - g.sent - CHAN) as u64;
        assert_eq!(l, event_cycles(n, tc4), "{name}");
        max = max.max(l);
    }
    assert_eq!(max, MAX_EVENT_CYCLES);
    assert_eq!(event_cycles(4, true), MAX_EVENT_CYCLES);
}

/// Events offered back to back (the next PeerStat waits in its channel while
/// the unit computes) and idle gaps of various lengths: the state (latest
/// PeerStat of each peer, previous system peer) is kept between events.
#[test]
fn back_to_back_and_gaps() {
    let sc = load();
    for name in [
        "interleave_same_now",
        "hysteresis_keep",
        "hysteresis_lost",
        "hysteresis_after_none",
        "four_one_false",
        "cluster_drop",
        "tie_identical",
        "three_one_false_first",
    ] {
        let s = match sc.iter().find(|s| s.name == name) {
            Some(s) => s,
            None => continue,
        };
        let evs = events_of(s);
        if evs.iter().all(|e| e.0 == evs[0].0) {
            let got = drive(s.n_en, &evs, true, |_| 0, always);
            check(s, &got);
            // Back to back: each event is taken right after the previous output.
            for w in got.windows(2) {
                assert!(w[1].recv > w[0].recv);
            }
        }
        let got = drive(s.n_en, &evs, false, |i| (i * 37) % 101, always);
        check(s, &got);
    }
    // Random scenarios (each event has its own `now`) with gaps.
    for s in sc.iter().filter(|s| s.name.starts_with("random_")).take(60) {
        let got = drive(s.n_en, &events_of(s), false, |i| (i * 13) % 7, always);
        check(s, &got);
    }
}

/// A stalled consumer: the Selected waits, nothing is lost or duplicated, and
/// no new PeerStat is taken meanwhile.
#[test]
fn stalled_consumer() {
    let sc = load();
    for name in ["four_one_false", "hysteresis_switch_far", "cluster_keep"] {
        let s = find(&sc, name);
        let evs = events_of(s);
        // Ready one cycle in 97, and never during a long first stall.
        let got = drive(s.n_en, &evs, true, |_| 0, |t| t > 3000 && t % 97 == 0);
        check(s, &got);
        let got = drive(s.n_en, &evs, false, |_| 0, |t| t % 5 == 4);
        check(s, &got);
    }
    // Explicitly: the output channel buffers two Selected; with it full and
    // the consumer stalled, the unit holds the third in its OUT state and the
    // fourth PeerStat stays in its channel (nothing overwritten).
    let s = find(&sc, "four_one_false");
    let (now_o, now) = signal::<U<64>, DefaultClock>();
    let (st_tx, st_rx) = chan::<PeerStat, DefaultClock>();
    let (sel_tx, sel_rx) = chan::<Selected, DefaultClock>();
    let mut dut = Selection::default();
    let state = dut.st;
    let n_en = tie::<U<3>, DefaultClock>(U::<3>::from(1u8));
    let mut sim = Running::new(dut.run((st_rx, now, n_en), sel_tx));
    now_o.set(U::<64>::from(s.events[0].now));
    let mut sent = 0;
    for _ in 0..5000 {
        if sent < 4 && st_tx.ready().to_bool() {
            st_tx.send(to_hw(&s.events[sent].stat));
            sent += 1;
        }
        sim.cycle();
    }
    assert_eq!(sent, 4);
    assert_eq!(state.get().raw(), 16, "the unit waits in OUT");
    let mut got = vec![];
    for _ in 0..5000 {
        if let Some(x) = sel_rx.recv() {
            got.push(from_hw(&x));
        }
        sim.cycle();
    }
    assert_eq!(got.len(), 4);
    for (i, g) in got.iter().enumerate() {
        assert_eq!(*g, s.events[i].want, "event {i}");
    }
    assert_eq!(state.get().raw(), 0, "idle again");
}

/// Extremes of the ranges, through the reference vectors: offsets of
/// ±2^31 s, the MAXDIST boundary, saturating distances, survivors 3 s apart.
#[test]
fn extremes() {
    let sc = load();
    let last = |name: &str| {
        let s = find(&sc, name);
        let got = drive(s.n_en, &events_of(s), false, |_| 0, always);
        check(s, &got);
        got.last().unwrap().sel
    };
    let maxdist = 3u64 << 31;
    let a = last("maxdist_minus_1");
    assert_eq!((a.valid, a.root_dist), (1, maxdist - 1));
    assert_eq!(last("maxdist_exact").valid, 0);
    let a = last("offset_max");
    assert_eq!((a.valid, a.offset), (1, i64::MAX as u64));
    let a = last("offset_min");
    assert_eq!((a.valid, a.offset), (1, i64::MIN as u64));
    assert_eq!(last("offset_extremes").valid, 0);
    assert_eq!(last("offset_near_max_pair").valid, 1);
    assert_eq!(last("offset_near_min_three").valid, 1);
    assert_eq!(last("huge_delay").valid, 0);
    assert_eq!(last("age_clamp").valid, 0);
    for name in ["widest_spread3", "widest_spread4", "widest_spread4_keep"] {
        assert_eq!(last(name).valid, 1, "{name}");
    }
}

/// A falseticker does not influence the result: the third event of
/// `three_one_false` adds a peer far away, and the Selected is the one of the
/// two agreeing peers alone (second event).
#[test]
fn falseticker_ignored() {
    let sc = load();
    let s = find(&sc, "three_one_false");
    let got = drive(s.n_en, &events_of(s), false, |_| 0, always);
    check(s, &got);
    assert_eq!(got[2].sel.valid, 1);
    assert_eq!(got[2].sel, got[1].sel);
}

/// Security finding F4: until the first valid selection a majority of the
/// enabled slots (`n_en / 2 + 1` truechimers) is required. One liar of two
/// enabled slots never produces a selection (no start-up step); two of four
/// are not enough, three are; once a selection has been made, one peer
/// alone is accepted again. One enabled slot: the single source decides
/// (residual risk).
#[test]
fn startup_majority() {
    let sc = load();
    let valid = |name: &str| -> Vec<u64> {
        let s = find(&sc, name);
        let got = drive(s.n_en, &events_of(s), false, |_| 0, always);
        check(s, &got);
        got.iter().map(|g| g.sel.valid).collect()
    };
    assert_eq!(valid("startup_one_enabled"), vec![1]);
    assert_eq!(valid("startup_two_one_lies"), vec![0, 0, 0]);
    assert_eq!(valid("startup_two_need_both"), vec![0, 1, 1, 1]);
    assert_eq!(valid("startup_three"), vec![0, 0, 1]);
    assert_eq!(valid("startup_four"), vec![0, 0, 1, 1, 1, 1, 1]);
    assert_eq!(valid("startup_four_falseticker"), vec![0, 0, 0, 1]);
    assert_eq!(valid("startup_seven"), vec![0, 0, 0, 1]);
    // The same events with one enabled slot: selected at once.
    let s = find(&sc, "startup_two_need_both");
    let got = drive(1, &events_of(s), false, |_| 0, always);
    assert_eq!(got[0].sel.valid, 1);
}
