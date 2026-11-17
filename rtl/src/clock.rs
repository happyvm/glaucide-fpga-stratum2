// SPDX-License-Identifier: Apache-2.0
//! NTP clocks: a 96-bit accumulator, 32.64 fixed point: 32.32 NTP time in
//! the top 64 bits (acc >> 32) plus 32 sub-LSB bits so a 10 ns tick
//! (100 MHz) has no rounding drift. `INCR` is 2^64 / 100e6.
//!
//! * [`NtpClock`]: free running, adds `INCR` every cycle (kept for the
//!   existing users and for the reference model of the tests).
//! * [`DisciplinedClock`] (task T2.1): the same accumulator, with a
//!   frequency trim (slew) and a phase step, the two handles of the servo
//!   (T4.5). The servo decides when to step (offset > 128 ms) and when to
//!   slew; this unit only applies what it is told, within bounds.
//!
//! # `DisciplinedClock` ports (`DefaultClock`, `clk_sys` 100 MHz)
//!
//! * `trim: In<U<32>>`: signed frequency adjustment, two's complement on
//!   32 bits, in units of **1 LSB of the accumulator per cycle**, i.e.
//!   2^-64 s per 10 ns cycle. It is added to `INCR` every cycle after a
//!   clamp to ±[`MAXTRIM`] (see below). The port is a `U<32>` read as two's
//!   complement rather than an `I<32>`: TxHDL's lowering has no operator on
//!   `I<N>` (no sign extension, no compare), so a signed port would have to
//!   be converted anyway; [`trim_word`] makes the word from an `i32`.
//! * `step: In<U<96>>`, `step_en: In<Bit>`: a phase correction. When
//!   `step_en` is high in a cycle, `step` (two's complement on 96 bits, in
//!   accumulator LSB, i.e. 2^-64 s; NTP 32.32 offset `o` is `o << 32`,
//!   sign extended) is added once to the accumulator, in the same cycle as
//!   the normal increment (no increment is lost). [`step_word`] makes the
//!   word from an `i128`.
//! * `time: Out<U<96>>`: the accumulator, same layout as `NtpClock`
//!   (consumers read `time >> 32`).
//! * `stepped: Out<Bit>`: high for one cycle, the first cycle where `time`
//!   shows a step (one per `step_en` cycle).
//!
//! Update, every cycle: `acc <= acc + INCR + sext(clamp(trim)) + (step_en
//! ? step : 0)`, modulo 2^96.
//!
//! ## Latency
//!
//! The inputs are not registered: `trim`, `step` and `step_en` are sampled
//! at the clock edge that ends the cycle where they are driven, and `time`
//! (the register) shows the result from the next cycle on. A step or trim
//! driven in cycle `n` is visible on `time` (and `stepped`) in cycle
//! `n + 1`: one cycle of latency, the register itself, and the step is not
//! delayed by a further cycle. A trim held for `k` cycles moves `time` by
//! `k * (INCR + trim)` LSB.
//!
//! ## Trim unit and clamp
//!
//! One trim LSB = 2^-64 s per cycle of 1e-8 s, a relative frequency of
//! 1e8 / 2^64 = 5.421e-12 (0.005421 ppb); the nominal `INCR` is
//! 184 467 440 737 LSB (its truncation of 0.0955 LSB is a fixed
//! -5.2e-13 bias, 0.0005 ppb).
//!
//! | rate | trim (LSB) |
//! |---|---|
//! | 1 LSB | 0.005421 ppb |
//! | 0.1 ppb | 18.4467 |
//! | 1 ppb | 184.467 |
//! | 1 ppm | 184 467.44 |
//! | 100 ppm | 18 446 744.07 |
//! | 500 ppm | 92 233 720.37 |
//! | 32-bit range (±2^31) | ±11 641 ppm |
//!
//! So a 0.1 ppb step is 18 LSB, with a 0.0054 ppb resolution, and ±500 ppm
//! fits with room in 32 bits. Conversion: `trim = round(ppb * 2^64 / 1e17)`,
//! see [`ppb_to_trim`].
//!
//! The trim is clamped inside the unit to `[-MAXTRIM, +MAXTRIM]` with
//! `MAXTRIM = floor(500e-6 * 2^64 / 1e8) = 2^64 / 2e11 = 92 233 720`
//! (499.999996 ppm): whatever the servo drives, the applied increment stays
//! in `[INCR - MAXTRIM, INCR + MAXTRIM]` = `[184 375 207 017,
//! 184 559 674 457]`, never zero or negative, so the trim alone can never
//! move the clock back or run it away.
//!
//! ## Wrap of the accumulator (NTP eras)
//!
//! The accumulator is modulo 2^96, that is 2^32 s: its wrap is the NTP era
//! rollover (RFC 5905 §6: timestamps are era modular), which the real time
//! reaches on 2036-02-07 once the servo has stepped the clock to UTC. This
//! is not an overflow to avoid: it is the NTP semantics. What the design
//! guarantees (proved, see below): without a step, the accumulator cannot
//! wrap before `2^96 / (INCR + MAXTRIM)` cycles after reset (4.29e9 s, 136
//! years at +500 ppm), so starting from reset (time 0) the wrap is only
//! reachable through a step. That lifetime bound is an assumption of the
//! design; the era wrap itself is handled by the consumers (era-modular
//! arithmetic on 32.32 timestamps).
//!
//! ## Formal properties (F4.1, `tools/formal/prove.sh disciplined_clock`)
//!
//! Ghost registers hold the previous accumulator (`g_prev`), the increment
//! and step applied by the last edge (`g_inc`, `g_step`), whether any step
//! was ever applied (`g_any`) and an upper bound of a step-free
//! accumulator (`g_hi`, `+ (INCR + MAXTRIM)` per cycle, saturating at
//! 2^96). Checked every cycle, proved by k-induction:
//!
//! * clamp: the clamped trim is in `[-MAXTRIM, MAXTRIM]`, equal to the
//!   input when the input is in range; the applied increment is in
//!   `[INCR - MAXTRIM, INCR + MAXTRIM]` (so >= 1);
//! * step exactly once: `time - prev == g_inc + (stepped ? g_step : 0)`
//!   (mod 2^96), `stepped` is high iff a step was applied by the last edge;
//! * monotony: without `stepped`, `time > prev` unless `prev` is within
//!   `INCR + MAXTRIM` of 2^96 (the era wrap: one addition cannot wrap
//!   otherwise); a decrease needs `stepped` with a negative step or the era
//!   wrap (cover points show both are reachable, the wrap after a step);
//! * lifetime: while no step was ever applied and `g_hi < 2^96`,
//!   `acc <= g_hi`, so no wrap occurs: from reset, the wrap needs a step or
//!   2^96 / (INCR + MAXTRIM) cycles.
use txhdl::comp::{mux, Clock, DefaultClock, In, Out, Reg, Unit};
use txhdl::types::{Bit, U};
use txhdl::{check, cover, lower, when, with, Trace};

pub const CLK_HZ: u128 = 100_000_000;
/// 2^64 / CLK_HZ: the per-cycle increment of the accumulator.
pub const INCR: u128 = (1u128 << 64) / CLK_HZ;
/// Clamp of the trim, in accumulator LSB per cycle: floor(500 ppm * INCR
/// exact) = 2^64 / (2000 * CLK_HZ) = 92 233 720.
pub const MAXTRIM: u128 = (1u128 << 64) / (2000 * CLK_HZ);
/// Largest increment one cycle can apply without a step.
pub const MAXINC: u128 = INCR + MAXTRIM;
/// Smallest increment one cycle can apply without a step.
pub const MININC: u128 = INCR - MAXTRIM;
/// 2^96, the modulus of the accumulator (2^32 s).
pub const ACC_MOD: u128 = 1u128 << 96;
/// Largest accumulator from which one step-free addition cannot wrap.
pub const WRAP_GUARD: u128 = ACC_MOD - 1 - MAXINC;
/// Cycles from reset after which a step-free accumulator may wrap:
/// ceil(2^96 / MAXINC), about 4.29e17 cycles = 4.29e9 s = 136 years.
pub const LIFETIME_CYCLES: u128 = ACC_MOD.div_ceil(MAXINC);

// Signed compares through a biased (offset binary) form: x ^ 2^31 orders
// two's complement words as unsigned numbers.
const SIGN32: u32 = 1 << 31;
const BIAS_HI: u32 = SIGN32 + MAXTRIM as u32;
const BIAS_LO: u32 = SIGN32 - MAXTRIM as u32;
const TRIM_HI: u32 = MAXTRIM as u32;
const TRIM_LO: u32 = (MAXTRIM as u32).wrapping_neg();

#[derive(Default, Trace)]
pub struct NtpClock {
    pub acc: Reg<U<96>>,
}

#[lower]
impl Unit for NtpClock {
    async fn run(&mut self, _inputs: (), time: Out<U<96>>) {
        loop {
            DefaultClock::rising().await;
            let on = Bit::One;
            when!(on => self { acc: self.acc + INCR });
            time.set(self.acc);
        }
    }
}

/// The disciplinable clock (T2.1), see the module documentation.
#[derive(Default, Trace)]
pub struct DisciplinedClock {
    /// The accumulator, 32.64.
    pub acc: Reg<U<96>>,
    /// A step was applied by the last edge.
    pub st: Reg<Bit>,
    // ---- ghosts (checks only) ----
    /// At least one edge since reset (`g_prev`, `g_inc` are meaningful).
    pub g_valid: Reg<Bit>,
    /// The accumulator before the last edge.
    pub g_prev: Reg<U<96>>,
    /// The increment (INCR + clamped trim) applied by the last edge.
    pub g_inc: Reg<U<96>>,
    /// The `step` input at the last edge.
    pub g_step: Reg<U<96>>,
    /// A step was applied since reset.
    pub g_any: Reg<Bit>,
    /// Upper bound of a step-free accumulator: MAXINC per cycle, saturating
    /// once at or past 2^96.
    pub g_hi: Reg<U<97>>,
}

#[lower]
impl Unit<(In<U<32>>, In<U<96>>, In<Bit>), (Out<U<96>>, Out<Bit>)> for DisciplinedClock {
    async fn run(
        &mut self,
        (trim, step, step_en): (In<U<32>>, In<U<96>>, In<Bit>),
        (time, stepped): (Out<U<96>>, Out<Bit>),
    ) {
        loop {
            DefaultClock::rising().await;
            let acc = self.acc.get();
            let t = trim.get();
            let s = step.get();
            let se = step_en.get();

            // Clamp of the signed trim to [-MAXTRIM, MAXTRIM].
            let tb = t ^ SIGN32;
            let over = Bit::from(tb > BIAS_HI);
            let under = Bit::from(tb < BIAS_LO);
            let tc = mux(
                over,
                U::<32>::from(TRIM_HI),
                mux(under, U::<32>::from(TRIM_LO), t),
            );
            let inc = tc.sext::<96>() + INCR;
            let sadd = mux(se, s, U::<96>::from(0u32));
            let next = acc + inc + sadd;

            // ---- properties ----
            let prev = self.g_prev.get();
            let gi = self.g_inc.get();
            let gs = self.g_step.get();
            let st = self.st.get();
            let valid = self.g_valid.get();
            let any = self.g_any.get();
            let hi = self.g_hi.get();
            let tcb = tc ^ SIGN32;
            check!(tcb >= BIAS_LO, "the applied trim is at least -MAXTRIM");
            check!(tcb <= BIAS_HI, "the applied trim is at most +MAXTRIM");
            if (!over & !under).to_bool() {
                check!(tc == t, "a trim in range is applied unchanged");
            }
            check!(inc >= MININC, "the increment is at least INCR - MAXTRIM");
            check!(inc <= MAXINC, "the increment is at most INCR + MAXTRIM");
            if valid.to_bool() {
                check!(
                    gi >= MININC,
                    "the last increment was at least INCR - MAXTRIM"
                );
                check!(
                    gi <= MAXINC,
                    "the last increment was at most INCR + MAXTRIM"
                );
                check!(
                    acc - prev == gi + mux(st, gs, U::<96>::from(0u32)),
                    "time moved by the increment, plus the step exactly once if stepped"
                );
                if (!st & Bit::from(prev <= WRAP_GUARD)).to_bool() {
                    check!(
                        acc > prev,
                        "without a step, time increases (outside the era wrap)"
                    );
                }
                cover!(
                    st & gs.bit(95) & Bit::from(acc < prev),
                    "a negative step moves time back"
                );
                cover!(
                    !st & Bit::from(acc < prev),
                    "the era wrap (reachable after a step)"
                );
            } else {
                check!(acc == 0u32, "the accumulator starts at zero");
                check!(!st, "no step before the first edge");
            }
            if (!any & Bit::from(hi < ACC_MOD)).to_bool() {
                check!(
                    acc.zext::<97>() <= hi,
                    "a step-free accumulator stays below its lifetime bound"
                );
                check!(!st, "no step seen while none was ever applied");
                if (!se & Bit::from(hi < ACC_MOD - MAXINC)).to_bool() {
                    check!(
                        next > acc,
                        "within the lifetime, a step-free cycle cannot wrap"
                    );
                }
            }
            if any.to_bool() {
                check!(valid, "a step implies an edge");
            }

            with!(self <= {
                Bit::One ? {
                    acc: next,
                    st: se,
                    g_valid: Bit::One,
                    g_prev: acc,
                    g_inc: inc,
                    g_step: s,
                    g_any: any | se,
                    g_hi: mux(Bit::from(hi < ACC_MOD), hi + MAXINC, hi),
                },
            });
            time.set(acc);
            stepped.set(st);
        }
    }
}

/// NTP 32.32 seconds from the accumulator.
pub fn ntp_time(acc: u128) -> u64 {
    (acc >> 32) as u64
}

/// The `trim` port word (two's complement) for a trim in LSB per cycle.
pub fn trim_word(lsb: i32) -> u32 {
    lsb as u32
}

/// Trim in LSB per cycle for a frequency offset in ppb, rounded:
/// `ppb * 2^64 / 1e17` (not clamped: the unit clamps).
pub fn ppb_to_trim(ppb: f64) -> i64 {
    (ppb * (1u128 << 64) as f64 / 1e17).round() as i64
}

/// The `step` port word (two's complement on 96 bits) for a step in
/// accumulator LSB (2^-64 s).
pub fn step_word(lsb: i128) -> u128 {
    (lsb as u128) & (ACC_MOD - 1)
}
