// SPDX-License-Identifier: Apache-2.0
//! Tests of the servo (T4.5): bit exact against the Python model's vectors
//! (`tools/golden/ntpclient/vectors/servo.txt`), the timing of the ports
//! (cycles per event, step pulse, back-pressure), saturation and
//! anti-windup, `valid = 0` events, and a short closed loop with the real
//! `DisciplinedClock` (trim latency, exact step).
use ntp_fpga::clock::{DisciplinedClock, INCR, MAXTRIM as CLK_MAXTRIM};
use ntp_fpga::ntp_types::Selected;
use ntp_fpga::servo::{
    process_cycles, Servo, CQ, CS, FREQ, MAXTRIM, NSET, PANIC, SPIK, STEPT_Q, SYNC,
};
use std::future::Future;
use txhdl::comp::{chan, join2, signal, DefaultClock, In, Out, Running, Tx, Unit};
use txhdl::types::{Bit, U};

const Q32: u64 = 1 << 32;
const M96: u128 = (1u128 << 96) - 1;
/// Longest processing (poll 10) plus margin.
const SETTLE: usize = 70;

fn q32(s: f64) -> u64 {
    (s * Q32 as f64).round() as i64 as u64
}

fn sel(valid: bool, offset: u64, jitter: u64) -> Selected {
    Selected {
        valid: Bit::from_bool(valid),
        offset: U::<64>::from(offset),
        jitter: U::<64>::from(jitter),
        ..Default::default()
    }
}

fn signed32(w: u32) -> i64 {
    w as i32 as i64
}

// ------------------------------------------------------------ bench

/// Outputs of the unit in one cycle.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Obs {
    trim: u32,
    step: u128,
    step_en: bool,
    state: u8,
    synced: bool,
}

struct Bench<F: Future<Output = ()>> {
    tx: Tx<Selected>,
    poll: Out<U<8>>,
    time: Out<U<96>>,
    o_trim: In<U<32>>,
    o_step: In<U<96>>,
    o_se: In<Bit>,
    o_state: In<U<3>>,
    o_synced: In<Bit>,
    sim: Running<F>,
}

fn bench() -> Bench<impl Future<Output = ()>> {
    let (tx, rx) = chan::<Selected, DefaultClock>();
    let (poll, poll_i) = signal::<U<8>, DefaultClock>();
    let (time, time_i) = signal::<U<96>, DefaultClock>();
    let (t_o, o_trim) = signal::<U<32>, DefaultClock>();
    let (s_o, o_step) = signal::<U<96>, DefaultClock>();
    let (e_o, o_se) = signal::<Bit, DefaultClock>();
    let (st_o, o_state) = signal::<U<3>, DefaultClock>();
    let (sy_o, o_synced) = signal::<Bit, DefaultClock>();
    let u: &'static mut Servo = Box::leak(Box::default());
    let sim = Running::new(u.run((rx, poll_i, time_i), (t_o, s_o, e_o, st_o, sy_o)));
    poll.set(U::<8>::from(6u8));
    time.set(U::<96>::from(0u32));
    Bench {
        tx,
        poll,
        time,
        o_trim,
        o_step,
        o_se,
        o_state,
        o_synced,
        sim,
    }
}

impl<F: Future<Output = ()>> Bench<F> {
    /// One cycle; returns the outputs presented during it.
    fn cycle(&mut self) -> Obs {
        self.sim.cycle();
        Obs {
            trim: self.o_trim.get().raw() as u32,
            step: self.o_step.get().raw(),
            step_en: self.o_se.get().to_bool(),
            state: self.o_state.get().raw() as u8,
            synced: self.o_synced.get().to_bool(),
        }
    }

    fn set_now(&mut self, now: u64) {
        self.time.set(U::<96>::new((now as u128) << 32));
    }

    /// Pushes one event in the next cycle (time = `now` held) and runs
    /// `n` cycles; returns the outputs of every cycle (index 0 = the cycle
    /// in which the event is offered).
    fn event(&mut self, s: Selected, now: u64, poll: u8, n: usize) -> Vec<Obs> {
        self.set_now(now);
        self.poll.set(U::<8>::from(poll));
        assert!(self.tx.ready().to_bool(), "channel full");
        self.tx.send(s);
        (0..n).map(|_| self.cycle()).collect()
    }
}

// ------------------------------------------------------------ vectors

#[test]
fn vectors_bit_exact() {
    let p = format!(
        "{}/../tools/golden/ntpclient/vectors/servo.txt",
        env!("CARGO_MANIFEST_DIR")
    );
    let text = std::fs::read_to_string(&p).unwrap_or_else(|e| panic!("{p}: {e}"));
    let mut b: Option<Bench<_>> = None;
    let mut name = String::new();
    let (mut n_ev, mut n_at, mut n_step) = (0, 0, 0);
    for (ln, l) in text.lines().enumerate() {
        let f: Vec<&str> = l.split_whitespace().collect();
        if f.is_empty() || f[0].starts_with('#') {
            continue;
        }
        let hx = |s: &str| u128::from_str_radix(s, 16).unwrap();
        let ctx = format!("{name}, line {}: {l}", ln + 1);
        match f[0] {
            "reset" => {
                b = Some(bench());
                name = f[1].to_string();
            }
            "ev" => {
                let bb = b.as_mut().unwrap();
                let valid = f[1] == "1";
                let now = hx(f[4]) as u64;
                let poll: u8 = f[5].parse().unwrap();
                let obs = bb.event(
                    sel(valid, hx(f[2]) as u64, hx(f[3]) as u64),
                    now,
                    poll,
                    SETTLE,
                );
                let pulses = obs.iter().filter(|o| o.step_en).count();
                let want_se = f[8] == "1";
                assert_eq!(pulses, want_se as usize, "step_en pulses, {ctx}");
                let last = *obs.last().unwrap();
                let want = Obs {
                    trim: hx(f[6]) as u32,
                    step: hx(f[7]),
                    step_en: false,
                    state: f[9].parse().unwrap(),
                    synced: f[10] == "1",
                };
                assert_eq!(last, want, "{ctx}");
                // The state never takes the value 1 (FSET) nor > 5, and the
                // trim is always within the clamp.
                for o in &obs {
                    assert!(o.state != 1 && o.state <= 5, "{ctx}");
                    assert!(signed32(o.trim).abs() <= MAXTRIM as i64, "{ctx}");
                }
                n_ev += 1;
                n_step += pulses;
            }
            "at" => {
                let bb = b.as_mut().unwrap();
                bb.set_now(hx(f[1]) as u64);
                bb.cycle();
                let o = bb.cycle();
                assert_eq!((o.trim, o.synced), (hx(f[2]) as u32, f[3] == "1"), "{ctx}");
                n_at += 1;
            }
            _ => panic!("bad line {ctx}"),
        }
    }
    println!("{n_ev} events, {n_at} time checks, {n_step} steps");
    assert!(n_ev > 400 && n_at > 50 && n_step >= 3);
}

// ------------------------------------------------------------ timing

/// Slew magnitude of a first (integral-free) update: min(c >> 1, MAXTRIM).
fn first_slew(off: u64, poll: u8) -> i64 {
    let a = (off as i64).unsigned_abs() as u128;
    let c = ((a * CQ as u128) >> CS) >> poll;
    let sm = (c >> 1).min(MAXTRIM as u128) as i64;
    if (off as i64) < 0 {
        -sm
    } else {
        sm
    }
}

#[test]
fn cycles_per_event() {
    for poll in [0u8, 4, 5, 6, 7, 8, 9, 10, 200] {
        let pn = poll.clamp(4, 10);
        let mut b = bench();
        for _ in 0..3 {
            b.cycle();
        }
        let off = q32(0.050);
        let obs = b.event(sel(true, off, q32(1e-4)), 1000 * Q32, poll, 80);
        // P: the cycle the event is popped = the cycle before the state
        // shows FREQ.
        let s1 = obs.iter().position(|o| o.state == FREQ).unwrap();
        let p = s1 - 1;
        let t1 = obs.iter().position(|o| o.trim != 0).unwrap();
        assert_eq!((t1 - p) as u64, process_cycles(pn), "poll {poll}");
        assert_eq!(process_cycles(pn), 50 + pn as u64);
        assert!(process_cycles(pn) <= 60);
        assert_eq!(signed32(obs[t1].trim), first_slew(off, pn), "poll {poll}");
        assert!(obs[..t1].iter().all(|o| o.trim == 0));
        assert!(obs.iter().all(|o| !o.step_en));
        println!(
            "poll {poll}: pop at offer+{p}, trim after {} cycles",
            t1 - p
        );
    }
}

#[test]
fn back_to_back_events_are_paced() {
    let mut b = bench();
    b.set_now(1000 * Q32);
    b.poll.set(U::<8>::from(6u8));
    // Two events queued at once (the channel holds one; the second is sent
    // as soon as there is room).
    let mut pending = vec![sel(true, q32(-0.002), 0), sel(true, q32(0.030), 0)];
    let mut states = vec![];
    let mut trims = vec![];
    for t in 0..200 {
        if !pending.is_empty() && b.tx.ready().to_bool() {
            b.tx.send(pending.remove(0));
        }
        let o = b.cycle();
        states.push(o.state);
        trims.push((t, o.trim));
    }
    // The first result shows in cycle t; the second event is popped in that
    // same cycle (the unit is IDLE again), which drops the first slew from
    // t + 1 (trim = freq), and its result shows process_cycles later.
    let changes: Vec<usize> = trims
        .windows(2)
        .filter(|w| w[0].1 != w[1].1)
        .map(|w| w[1].0)
        .collect();
    assert_eq!(changes.len(), 3, "{changes:?}");
    assert_eq!(changes[1], changes[0] + 1);
    assert_eq!((changes[2] - changes[0]) as u64, process_cycles(6));
    assert_eq!(*states.last().unwrap(), FREQ);
}

#[test]
fn step_pulse_is_one_cycle() {
    let mut b = bench();
    b.cycle();
    let off = q32(-0.5);
    let obs = b.event(sel(true, off, 0), 77 * Q32, 6, 20);
    let pulses: Vec<usize> = (0..obs.len()).filter(|&i| obs[i].step_en).collect();
    assert_eq!(pulses.len(), 1);
    let i = pulses[0];
    // The step shows with the state change (P + 1).
    assert_eq!(obs[i].state, FREQ);
    assert_eq!(obs[i - 1].state, NSET);
    assert_eq!(obs[i].step, ((off as i64 as i128) << 32) as u128 & M96);
    assert_eq!(obs[i].trim, 0);
    // The step word is held afterwards, without the pulse.
    assert_eq!(obs.last().unwrap().step, obs[i].step);
    // A second big offset (now in FREQ) is never stepped: SPIK.
    let o2 = b.event(sel(true, q32(0.5), 0), 141 * Q32, 6, 20);
    assert!(o2.iter().all(|o| !o.step_en));
    assert_eq!(o2.last().unwrap().state, SPIK);
}

// ------------------------------------------------------------ behaviour

#[test]
fn valid_zero_changes_nothing() {
    let mut b = bench();
    b.cycle();
    for k in 0..5 {
        let obs = b.event(sel(false, q32(-3.0), 0), (k + 1) * 100 * Q32, 6, 10);
        assert!(obs
            .iter()
            .all(|o| !o.step_en && o.state == NSET && o.trim == 0));
    }
    // Then synchronise and lose the updates: the frequency holds, no step.
    let mut now = 1000 * Q32;
    for off in [0.004, 0.002, 0.001, 0.0005, 0.0002, 0.0001] {
        b.event(sel(true, q32(off), q32(1e-4)), now, 4, SETTLE);
        now += 16 * Q32;
    }
    b.set_now(now);
    b.cycle();
    let held = b.cycle();
    assert_eq!(held.state, SYNC);
    for k in 0..20 {
        let obs = b.event(sel(false, q32(10.0), 0), now + k * 50 * Q32, 4, 5);
        for o in obs {
            assert!(!o.step_en);
            assert_eq!(o.trim, held.trim, "holdover keeps the frequency");
            assert_eq!(o.state, SYNC);
        }
    }
    // Eight poll intervals without an accepted update: not synced any more.
    b.set_now(now + 20 * 50 * Q32);
    b.cycle();
    assert!(!b.cycle().synced);
}

#[test]
fn saturation_and_anti_windup() {
    // A clock running 800 ppm slow (beyond the 500 ppm range): the offset
    // grows at 300 ppm even at full trim. The trim stays clamped, never a
    // step, and the integrator does not run away.
    let mut b = bench();
    b.cycle();
    let mut now = 5000 * Q32;
    let mut x = 0.0f64; // upstream - local, seconds
    let mut last = Obs {
        trim: 0,
        step: 0,
        step_en: false,
        state: 0,
        synced: false,
    };
    for k in 0..200 {
        let obs = b.event(sel(true, q32(x), q32(1e-4)), now, 4, SETTLE);
        for o in &obs {
            assert!(!o.step_en, "event {k}");
            assert!(signed32(o.trim).abs() <= MAXTRIM as i64);
        }
        last = *obs.last().unwrap();
        // 16 s at (800 ppm - trim) of drift, trim held for the interval.
        let rate = 800e-6 - signed32(last.trim) as f64 * 1e8 / 2f64.powi(64);
        x = (x + rate * 16.0).min(0.1);
        now += 16 * Q32;
    }
    assert_eq!(signed32(last.trim), MAXTRIM as i64, "full positive trim");
    assert_eq!(MAXTRIM as u128, CLK_MAXTRIM);
    // After the slew expires, the trim is the integrator alone, still within
    // range.
    b.set_now(now + 17 * Q32);
    b.cycle();
    let o = b.cycle();
    assert!(signed32(o.trim).abs() <= MAXTRIM as i64);
}

#[test]
fn spike_is_ignored_once() {
    let mut b = bench();
    b.cycle();
    let mut now = 1000 * Q32;
    for off in [0.0004, 0.0002, 0.0001, 0.00005, 0.00002] {
        b.event(sel(true, q32(off), q32(1e-4)), now, 6, SETTLE);
        now += 64 * Q32;
    }
    let before = b.event(sel(true, q32(0.00001), q32(1e-4)), now, 6, SETTLE);
    let before = *before.last().unwrap();
    assert_eq!(before.state, SYNC);
    now += 64 * Q32;
    // The outputs just before the spike (the slew has expired by now).
    b.set_now(now);
    b.cycle();
    let before = b.cycle();
    // +2 ms spike: ignored, state SPIK, trim unchanged.
    let obs = b.event(sel(true, q32(0.002), q32(1e-4)), now, 6, SETTLE);
    // Offered in cycle 0, popped in cycle 1 (the channel register): the
    // state shows SPIK from cycle 2; the trim never moves.
    assert!(obs.iter().all(|o| o.trim == before.trim && !o.step_en));
    for o in &obs[2..] {
        assert_eq!(o.state, SPIK);
    }
    now += 64 * Q32;
    // Next normal update: SYNC again.
    let o = *b
        .event(sel(true, q32(-0.00001), q32(1e-4)), now, 6, SETTLE)
        .last()
        .unwrap();
    assert_eq!(o.state, SYNC);
    now += 64 * Q32;
    // Panic: never followed.
    let o = *b
        .event(sel(true, q32(-2000.0), 0), now, 6, SETTLE)
        .last()
        .unwrap();
    assert_eq!(o.state, PANIC);
    assert!(!o.synced);
}

#[test]
fn stept_boundary() {
    // |offset| == STEPT is slewed, STEPT + 1 is stepped (at start-up).
    for (off, stepped) in [
        (STEPT_Q, false),
        (STEPT_Q.wrapping_neg(), false),
        (STEPT_Q + 1, true),
        ((STEPT_Q + 1).wrapping_neg(), true),
    ] {
        let mut b = bench();
        b.cycle();
        let obs = b.event(sel(true, off, 0), 10 * Q32, 6, SETTLE);
        assert_eq!(obs.iter().any(|o| o.step_en), stepped, "{off:#x}");
    }
}

// ------------------------------------------------------------ with the clock

#[test]
fn drives_the_disciplined_clock() {
    let (tx, rx) = chan::<Selected, DefaultClock>();
    let (poll, poll_i) = signal::<U<8>, DefaultClock>();
    let (time_o, time_i) = signal::<U<96>, DefaultClock>();
    let (trim_o, trim_i) = signal::<U<32>, DefaultClock>();
    let (step_o, step_i) = signal::<U<96>, DefaultClock>();
    let (se_o, se_i) = signal::<Bit, DefaultClock>();
    let (st_o, st_i) = signal::<U<3>, DefaultClock>();
    let (sy_o, _sy_i) = signal::<Bit, DefaultClock>();
    let (stepped_o, stepped_i) = signal::<Bit, DefaultClock>();
    let s: &'static mut Servo = Box::leak(Box::default());
    let c: &'static mut DisciplinedClock = Box::leak(Box::default());
    // The servo's time input is the clock's output.
    let time_tap = time_i.clone();
    let mut sim = Running::new(join2(
        s.run((rx, poll_i, time_i), (trim_o, step_o, se_o, st_o, sy_o)),
        c.run(
            (trim_i.clone(), step_i.clone(), se_i.clone()),
            (time_o, stepped_o),
        ),
    ));
    poll.set(U::<8>::from(4u8));
    let mut times: Vec<u128> = vec![];
    let mut trims: Vec<u32> = vec![];
    let mut ses: Vec<bool> = vec![];
    let mut steppeds: Vec<bool> = vec![];
    let mut states: Vec<u8> = vec![];
    macro_rules! run {
        ($n:expr) => {
            for _ in 0..$n {
                sim.cycle();
                times.push(time_tap.get().raw());
                trims.push(trim_i.get().raw() as u32);
                ses.push(se_i.get().to_bool());
                steppeds.push(stepped_i.get().to_bool());
                states.push(st_i.get().raw() as u8);
            }
        };
    }
    run!(10);
    // Start-up: the upstream is 3 913 056 000.3 s (2024) ahead of a clock
    // started at 0 (1900): beyond the signed 32.32 range, the offset word
    // wraps, and the step is modulo 2^32 s like the accumulator.
    let t_true: u128 = (3_913_056_000u128 << 64) + (3u128 << 64) / 10;
    let now = times.last().unwrap();
    let off = ((t_true.wrapping_sub(*now) & M96) >> 32) as u64;
    tx.send(sel(true, off, q32(1e-4)));
    run!(20);
    let i_se: Vec<usize> = (0..ses.len()).filter(|&i| ses[i]).collect();
    let i_st: Vec<usize> = (0..steppeds.len()).filter(|&i| steppeds[i]).collect();
    assert_eq!(i_se.len(), 1);
    assert_eq!(
        i_st,
        vec![i_se[0] + 1],
        "the clock shows the step one cycle later"
    );
    let k = i_st[0];
    let step = ((off as u128) << 32) & M96;
    assert_eq!(times[k], (times[k - 1] + INCR + step) & M96, "exact step");
    // The clock now reads the true time within the measurement latency.
    let err = (times[k] as i128 - (t_true >> 32 << 32) as i128).abs();
    assert!(err < (1i128 << 64) / 1000, "within 1 ms after the step");
    assert_eq!(states[k], FREQ);
    // Each cycle advances by INCR exactly (trim 0 in NSET/after the step).
    for j in k + 1..times.len() {
        assert_eq!(times[j] - times[j - 1], INCR);
    }
    // A +10 ms update at poll 4: the slew reaches the clock with the
    // documented latency and is applied every cycle for 3 ms of simulation.
    let n0 = trims.len();
    let off2 = q32(0.010);
    poll.set(U::<8>::from(4u8));
    tx.send(sel(true, off2, q32(1e-4)));
    run!(300_000);
    let t1 = (n0..trims.len()).find(|&i| trims[i] != 0).unwrap();
    let want = first_slew(off2, 4);
    assert_eq!(signed32(trims[t1]), want);
    // The pop cycle P is the cycle the event is offered (n0) or the next one.
    assert!(t1 - n0 == process_cycles(4) as usize || t1 - n0 == process_cycles(4) as usize + 1);
    // Trim driven in cycle t1 moves `time` from cycle t1 + 1 on.
    assert_eq!(times[t1] - times[t1 - 1], INCR);
    for j in t1 + 1..times.len() {
        assert_eq!(
            times[j] as i128 - times[j - 1] as i128,
            INCR as i128 + want as i128,
            "cycle {j}"
        );
    }
    // The second update (FREQ) is slewed, never stepped.
    assert_eq!(ses.iter().filter(|&&s| s).count(), 1);
    assert_eq!(*states.last().unwrap(), FREQ);
    println!(
        "slew {} LSB = {:.3} ppm, applied from cycle {} (event offered at {})",
        want,
        want as f64 * 1e8 / 2f64.powi(64) * 1e6,
        t1 + 1,
        n0
    );
}

#[test]
fn lowers_to_verilog() {
    let v = Servo::verilog("servo");
    assert!(v.contains("module servo"), "{v}");
}
