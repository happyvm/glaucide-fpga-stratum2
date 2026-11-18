// SPDX-License-Identifier: Apache-2.0
//! Tests of the disciplinable clock (T2.1).
use ntp_fpga::clock::{
    ppb_to_trim, step_word, trim_word, DisciplinedClock, NtpClock, ACC_MOD, CLK_HZ, INCR, MAXTRIM,
};
use std::future::Future;
use txhdl::comp::{signal, DefaultClock, In, Out, Running, Unit};
use txhdl::types::{Bit, U};

const MASK: u128 = ACC_MOD - 1;
/// One second in accumulator LSB (2^-64 s).
const SEC: i128 = 1i128 << 64;

/// The unit with its input drivers and output probes.
struct Bench<F: Future<Output = ()>> {
    trim: Out<U<32>>,
    step: Out<U<96>>,
    step_en: Out<Bit>,
    time: In<U<96>>,
    stepped: In<Bit>,
    sim: Running<F>,
    /// Reference model: the accumulator and `stepped` of the next cycle.
    model: u128,
    model_stepped: bool,
}

fn bench() -> Bench<impl Future<Output = ()>> {
    let (trim, trim_i) = signal::<U<32>, DefaultClock>();
    let (step, step_i) = signal::<U<96>, DefaultClock>();
    let (step_en, sen_i) = signal::<Bit, DefaultClock>();
    let (time_o, time) = signal::<U<96>, DefaultClock>();
    let (st_o, stepped) = signal::<Bit, DefaultClock>();
    let c: &'static mut DisciplinedClock = Box::leak(Box::default());
    let sim = Running::new(c.run((trim_i, step_i, sen_i), (time_o, st_o)));
    trim.set(U::<32>::from(0u32));
    step.set(U::<96>::from(0u32));
    step_en.set(Bit::Zero);
    Bench {
        trim,
        step,
        step_en,
        time,
        stepped,
        sim,
        model: 0,
        model_stepped: false,
    }
}

impl<F: Future<Output = ()>> Bench<F> {
    /// Drives the inputs for one cycle and returns `(time, stepped)` as
    /// presented during that cycle: the inputs take effect at the edge that
    /// ends it, so they show in the value returned by the next call (one
    /// cycle of latency, the register). Every value is compared with the
    /// reference model.
    fn cycle(&mut self, trim: i32, step: Option<i128>) -> (u128, bool) {
        self.trim.set(U::<32>::from(trim_word(trim)));
        self.step
            .set(U::<96>::from(step.map(step_word).unwrap_or(0)));
        self.step_en.set(Bit::from_bool(step.is_some()));
        self.sim.cycle();
        let got = (self.time.get().raw(), self.stepped.get().to_bool());
        assert_eq!(
            got,
            (self.model, self.model_stepped),
            "unit against the model"
        );
        self.model = add(self.model, inc(trim) as i128 + step.unwrap_or(0));
        self.model_stepped = step.is_some();
        got
    }
}

/// Reference model: the increment one cycle applies for a trim.
fn inc(trim: i32) -> u128 {
    let t = (trim as i128).clamp(-(MAXTRIM as i128), MAXTRIM as i128);
    (INCR as i128 + t) as u128
}

fn add(a: u128, d: i128) -> u128 {
    (a as i128).wrapping_add(d) as u128 & MASK
}

/// Signed difference a - b on 96 bits.
fn diff(a: u128, b: u128) -> i128 {
    let d = a.wrapping_sub(b) & MASK;
    if d >> 95 == 1 {
        d as i128 - ACC_MOD as i128
    } else {
        d as i128
    }
}

#[test]
fn latency_is_one_cycle() {
    let mut b = bench();
    let (t0, _) = b.cycle(1000, None);
    assert_eq!(t0, 0);
    // The trim driven in the previous cycle shows now.
    let (t1, s1) = b.cycle(0, Some(SEC));
    assert_eq!(t1, INCR + 1000);
    assert!(!s1);
    // The step shows one cycle after it was driven, with the increment.
    let (t2, s2) = b.cycle(0, None);
    assert_eq!(t2, t1 + INCR + SEC as u128);
    assert!(s2, "stepped shows with the stepped time");
    let (t3, s3) = b.cycle(0, None);
    assert_eq!(t3, t2 + INCR);
    assert!(!s3);
}

#[test]
fn free_running_matches_ntp_clock() {
    let (drive, nt) = signal::<U<96>, DefaultClock>();
    let mut n = NtpClock::default();
    let mut ns = Running::new(n.run((), drive));
    let mut b = bench();
    for _ in 0..100_000 {
        // 1 ms at 100 MHz.
        ns.cycle();
        let (t, s) = b.cycle(0, None);
        assert_eq!(t, nt.get().raw());
        assert!(!s);
    }
}

/// Trim of `ppm` over `cycles`: the drift against the nominal rate is
/// exactly trim * cycles, and within one trim LSB per cycle of the ideal
/// rate.
fn drift(ppm: f64, cycles: u64) {
    let trim = ppb_to_trim(ppm * 1000.0) as i32;
    let mut b = bench();
    let (t0, _) = b.cycle(trim, None);
    for _ in 1..cycles {
        b.cycle(trim, None);
    }
    let (t1, _) = b.cycle(0, None);
    let elapsed = diff(t1, t0);
    let nominal = (INCR * cycles as u128) as i128;
    assert_eq!(elapsed - nominal, trim as i128 * cycles as i128);
    // Ideal drift: ppm * elapsed real time (cycles * 10 ns), in LSB.
    let ideal = ppm * 1e-6 * (cycles as f64 * 1e-8) * SEC as f64;
    let err = (elapsed - nominal) as f64 - ideal;
    assert!(
        err.abs() <= cycles as f64,
        "{ppm} ppm: drift error {err} LSB over {cycles} cycles"
    );
    // In NTP 32.32 units the drift error is below one LSB (233 ps).
    assert!(err.abs() / (1u64 << 32) as f64 <= 1.0);
}

#[test]
fn trim_plus_minus_100_ppm_over_10_ms() {
    drift(100.0, 1_000_000);
    drift(-100.0, 1_000_000);
}

#[test]
fn trim_near_500_ppm_and_tenth_ppb() {
    drift(499.999, 200_000);
    drift(-499.999, 200_000);
    drift(0.1, 200_000);
    drift(-0.1, 200_000);
}

#[test]
fn trim_beyond_500_ppm_is_clamped() {
    for &trim in &[
        MAXTRIM as i32 + 1,
        -(MAXTRIM as i32) - 1,
        ppb_to_trim(1_000_000.0) as i32,
        ppb_to_trim(-1_000_000.0) as i32,
        i32::MAX,
        i32::MIN,
        MAXTRIM as i32,
        -(MAXTRIM as i32),
    ] {
        let mut b = bench();
        let (t0, _) = b.cycle(trim, None);
        let mut t = t0;
        for _ in 0..1000 {
            let (n, _) = b.cycle(trim, None);
            assert_eq!(diff(n, t), inc(trim) as i128, "trim {trim}");
            t = n;
        }
        let d = diff(t, t0) - (INCR * 1000) as i128;
        assert_eq!(d.abs(), MAXTRIM as i128 * 1000, "trim {trim}");
    }
}

#[test]
fn steps() {
    let mut b = bench();
    for _ in 0..10 {
        b.cycle(0, None);
    }
    // Start one hour in so that -1 s does not cross zero.
    b.cycle(0, Some(3600 * SEC));
    let (mut t, _) = b.cycle(0, None);
    for &(s, trim) in &[
        (SEC, 0),
        (-SEC, 0),
        (1, 0),
        (-1, 0),
        (SEC / 2, 5000),
        (-SEC, -(MAXTRIM as i32) - 7),
    ] {
        // The step and a trim change in the same cycle.
        let (a, sa) = b.cycle(trim, Some(s));
        assert!(!sa);
        assert_eq!(a, add(t, INCR as i128));
        let (n, sn) = b.cycle(trim, None);
        assert_eq!(diff(n, a), inc(trim) as i128 + s, "step {s}");
        assert!(sn, "stepped pulses");
        let (m, sm) = b.cycle(0, None);
        assert_eq!(diff(m, n), inc(trim) as i128);
        assert!(!sm, "stepped pulses for one cycle only");
        t = m;
    }
    // Two consecutive steps: two pulses, each applied once.
    let (n0, _) = b.cycle(0, Some(SEC));
    let (n1, s1) = b.cycle(0, Some(SEC));
    let (n2, s2) = b.cycle(0, None);
    let (_, s3) = b.cycle(0, None);
    assert!(s1 && s2 && !s3);
    assert_eq!(n1, add(n0, INCR as i128 + SEC));
    assert_eq!(n2, add(n1, INCR as i128 + SEC));
    assert_eq!(n0, add(t, INCR as i128));
}

#[test]
fn monotonic_with_random_trims() {
    let mut b = bench();
    let mut x: u64 = 0x1234_5678_9abc_def0;
    let mut t = 0u128;
    let mut trim = 0i32;
    for i in 0..100_000 {
        x = x
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        // A new random trim in range every cycle (the model checks the
        // exact value; here, monotony).
        let r = (x >> 33) as i64 % (2 * MAXTRIM as i64 + 1);
        let (n, s) = b.cycle(trim, None);
        assert!(!s);
        if i > 0 {
            assert!(n > t, "time increases at cycle {i}");
        }
        trim = (r - MAXTRIM as i64) as i32;
        t = n;
    }
}

#[test]
fn era_wrap_keeps_all_bits() {
    let mut b = bench();
    // Step to 100 us before the 2^96 (2^32 s) wrap.
    let near = ACC_MOD - (INCR * 10_000);
    let (t0, _) = b.cycle(0, Some(near as i128 - INCR as i128));
    assert_eq!(t0, 0);
    let trim = 12345;
    let (mut t, _) = b.cycle(trim, None);
    assert_eq!(t, near);
    let mut wrapped = 0;
    for _ in 0..20_000 {
        let (n, _) = b.cycle(trim, None);
        // No bit lost: the 96-bit difference is the increment.
        assert_eq!(diff(n, t), inc(trim) as i128);
        if n < t {
            wrapped += 1;
        }
        t = n;
    }
    assert_eq!(wrapped, 1, "exactly one era wrap");
    assert_eq!(t, add(near, (inc(trim) * 20_000) as i128));
    // After the wrap the time starts again from (almost) zero.
    assert_eq!(t, inc(trim) * 20_000 - INCR * 10_000);
}

/// Roadmap acceptance of T2.1: ±500 ppm checked over 1 s of simulated
/// time (1e8 cycles) to within 1 ppm. Two times 1e8 cycles take about a
/// minute in release and much longer in debug, hence ignored by default:
/// `cargo test -p ntp_fpga --release --test clock_disc -- --ignored`.
#[test]
#[ignore]
fn trim_500_ppm_over_one_second() {
    for ppm in [500.0, -500.0] {
        let trim = ppb_to_trim(ppm * 1000.0) as i32;
        let mut b = bench();
        let (t0, _) = b.cycle(trim, None);
        for _ in 1..CLK_HZ {
            b.cycle(trim, None);
        }
        let (t1, _) = b.cycle(0, None);
        // Elapsed clock time against 1 s of cycles, in ppm.
        let rate = (diff(t1, t0) - SEC) as f64 / SEC as f64 * 1e6;
        assert!((rate - ppm).abs() <= 1.0, "{ppm} ppm measured as {rate}");
        // Clamped at MAXTRIM: 499.999996 ppm.
        assert!((rate.abs() - 500.0).abs() < 0.001, "{rate}");
    }
}
