// SPDX-License-Identifier: Apache-2.0
//! Clock discipline (servo) of the NTP client, task T4.5, domain
//! `DefaultClock` (`clk_sys`, 100 MHz).
//!
//! [`Servo`] turns the combined offset of the system peer ([`Selected`],
//! from the selection T4.4) into the two handles of the
//! [`DisciplinedClock`](crate::clock::DisciplinedClock) (T2.1): a
//! frequency trim and a one-shot phase step. It is a PI loop inspired by
//! RFC 5905 §11.3 / A.5.5.6 `local_clock()` (states, STEPT, STEPOUT,
//! PANICT) and by Fakernet (half of the correction applied persistently,
//! half until the next update), simplified for hardware. The bit-exact
//! reference is `tools/golden/ntpclient/servo.py` (class `Servo`); the
//! closed-loop behaviour (settling, noise) is shown by the same file's
//! `simulate()` and its tests.
//!
//! # Ports (flat, in this order)
//!
//! | port | type | meaning |
//! |---|---|---|
//! | `sel` | `Rx<Selected>` | one event per selection run; fields used: `valid`, `offset` (signed 32.32), `jitter` (unsigned 32.32) |
//! | `poll_log2` | `In<U<8>>` | poll exponent from the configuration; clamped to 4..=10 when sampled |
//! | `time` | `In<U<96>>` | the clock accumulator (`DisciplinedClock.time`); the unit uses `now_s = time[95:48]` (32.32 time >> 16, units 2^-16 s, modulo 2^48) |
//! | `trim` | `Out<U<32>>` | to `DisciplinedClock.trim`: two's complement, 2^-64 s per cycle (0.005421 ppb per LSB), **always in [-MAXTRIM, +MAXTRIM]** (±500 ppm) |
//! | `step` | `Out<U<96>>` | to `DisciplinedClock.step`: the last commanded step (two's complement, 2^-64 s), held; 0 after reset |
//! | `step_en` | `Out<Bit>` | to `DisciplinedClock.step_en`: high for exactly one cycle per step |
//! | `state` | `Out<U<3>>` | discipline state, below (for diagnostics and the sync state T4.6) |
//! | `synced` | `Out<Bit>` | `state` is FREQ or SYNC **and** the last accepted update is less than 2^(pl+3) s old (8 poll intervals) |
//!
//! All outputs are registers or a 2:1 mux of registers (`trim`, `synced`):
//! no combinational path from an input to an output.
//!
//! # Units and constants
//!
//! * `offset` θ: signed 32.32 (2^-32 s). Positive = the upstream is ahead
//!   of the local clock = the local clock must speed up (positive trim).
//!   `a = |θ|` is computed on 64 bits (`|−2^63|` = 2^63, unsigned).
//! * `STEPT_Q` = round(0.128 · 2^32) = 549 755 814 (128 ms);
//!   `PANICT_Q` = 1000 · 2^32 (1000 s); `STEPOUT_S` = 900 · 2^16 (900 s in
//!   `now_s` units); `SPKMIN_Q` = 4 294 967 (1 ms); `SGATE` = 3.
//! * `CQ` = round(2^56 / 1e8) = 720 575 940, `CS` = 24: the trim that removes
//!   an offset `a` in one second is `r = (a · CQ) >> 24` (2^32/1e8 = 42.95
//!   trim LSB per 2^-32 s per second).
//! * Gains, as shifts: `KP` = 1 (proportional 1/2), `KI_FREQ` = 1
//!   (integral 1/2 in FREQ, the Fakernet half/half), `KI_SYNC` = 3 (integral
//!   1/8 in SYNC); `FREQ_N` = 4; `DMAX` = 2·MAXTRIM; `MAXTRIM` = 92 233 720
//!   (clock.rs).
//!
//! # Registers that define the behaviour
//!
//! `st` (state), `freq` (integrator, signed trim LSB), `tr` (trim while the
//! slew is active), `tl` (`now_s` of the last accepted update), `pl`
//! (clamped poll of the last accepted update), `muv` (an accepted update
//! exists since the last step/reset: the integral may use the elapsed
//! time), `slw` (SPIK: slewing a persistent large offset), `fc` (FREQ
//! counter, 0..=4), `stp`/`se` (step word, step pulse).
//! Reset: everything 0 (st = NSET, pl = 0 until the first accepted update).
//!
//! # Event decision (cycle P, the cycle the event is popped)
//!
//! In IDLE an event is popped as soon as `sel` has one (one per IDLE cycle;
//! the unit does not pop while busy, so the channel back-pressures).
//! Let `el = (now_s − tl) mod 2^48` (units 2^-16 s), `pn` = clamped
//! `poll_log2`, `big = a > STEPT_Q`, `huge = a > PANICT_Q`,
//! `spk = a > max(3·jitter, SPKMIN_Q)` (the gate; `jitter ≥ 2^31` never
//! gates). `valid = 0`: the event is dropped, **nothing changes**
//! (holdover). Otherwise, by state:
//!
//! | state | condition | action | next state |
//! |---|---|---|---|
//! | NSET (0) | `big` (any size, no panic check) | **step** by θ | FREQ, `fc` = 0 |
//! | NSET | `!big` | process (no integral) | FREQ, `fc` = 0 |
//! | FREQ (3) / SYNC (4) | `huge` | ignore | PANIC |
//! | FREQ / SYNC | `big` | ignore | SPIK |
//! | SYNC | `!big & spk` | ignore (spike) | SPIK |
//! | SYNC | `!big & !spk` | process, `KI_SYNC` | SYNC |
//! | FREQ | `!big` | process, `KI_FREQ`; `fc` = `spk` ? 0 : min(`fc`+1, 4) | SYNC if `fc` reaches 4, else FREQ |
//! | SPIK (2) / PANIC (5) | `huge` | ignore | PANIC |
//! | SPIK / PANIC | `big & (el ≥ STEPOUT_S \| slw)` | process, **no integral**; `slw` = 1 | SPIK |
//! | SPIK / PANIC | `big` otherwise | ignore | SPIK |
//! | SPIK / PANIC | `!big` | process, `KI_SYNC`; `slw` = 0 | SYNC |
//!
//! FSET (1) is RFC 5905's "frequency from file" state: there is no stored
//! frequency here, the state is never entered (`state` is never 1 and never
//! above 5; it changes only in a cycle P).
//!
//! **Step** (only from NSET, so at most once after reset, and at most once
//! per event): `stp = θ << 32` (θ sign-extended into the 32.64 accumulator
//! format, i.e. `{θ, 32'b0}`), `se` = 1 for the single cycle P+1; `tl =
//! (now_s + (θ >> 16 arithmetic)) mod 2^48` (the time after the step),
//! `pl = pn`, `tr = freq`, `muv` = 0. `freq` is untouched (it is 0 in NSET).
//! A step is never commanded in any other state: after start-up, an offset
//! above STEPT is first ignored (SPIK), then slewed at ≤ 500 ppm after
//! STEPOUT; above PANICT it is never followed (PANIC is reported through
//! `state` and `synced` = 0). RFC 5905 would step in both cases.
//!
//! **Ignore**: only `st` changes (and nothing else); the previous outputs
//! hold, the slew of the last processed update runs to its expiry.
//!
//! **Process** (sequential, see « Arithmetic »): in cycle P `tl = now_s`,
//! `pl = pn`, `muv` = 1, `tr = freq` (the old slew stops at once), and the
//! state registers are written as in the table. In cycle FIN the new `freq`
//! and `tr` are written.
//!
//! # Arithmetic of « process » (bit exact with `Servo._process`)
//!
//! With `sg` = sign of θ, `a` = |θ| (here `a ≤ PANICT_Q < 2^42`), and
//! `m = min(el >> pn, 2^16)` if the integral is enabled (`muv` and not the
//! SPIK slew case) else 0 (`el` and `pn` of cycle P; `m/2^16` is the time
//! since the previous accepted update as a fraction of the poll interval
//! T = 2^pn s, clamped to 1):
//!
//! ```text
//! r   = (a * CQ) >> 24                     trim that removes θ in 1 s
//! c   = r >> pn                            trim that removes θ in T
//! sm  = min(c >> KP, MAXTRIM)              slew magnitude
//! dm  = min((c * m) >> (16 + ki), DMAX)    integral step magnitude, ki = KI_FREQ or KI_SYNC
//! s   = sg ? -sm : sm ;  d = sg ? -dm : dm
//! f1  = freq + d
//! lim = MAXTRIM - sm                       (>= 0: room left beside the slew)
//! freq' = sg == 0 ? (f1 >  lim ? max(freq,  lim) : f1)       (anti-windup,
//!                 : (f1 < -lim ? min(freq, -lim) : f1)        signed compares)
//! tr'   = clamp(freq' + s)                 clamp = to [-MAXTRIM, MAXTRIM]
//! ```
//!
//! So `|freq| ≤ MAXTRIM` and `|tr| ≤ MAXTRIM` always (32-bit two's
//! complement, no overflow: |f1| ≤ 3·MAXTRIM < 2^29). Anti-windup: the
//! integrator moves in the direction of `d` only, and at most until
//! `freq + s` reaches the clamp (back-calculation); it never moves
//! against `d`, so an integrator already beyond `lim` is held, and the
//! output saturates through the slew only. (A plain "hold while the output
//! would saturate" rule dead-locks: the slew then carries a frequency error
//! for ever with a constant offset; the closed-loop test at +400 ppm shows
//! it.) The phase part `s` removes half of θ over one poll interval:
//! since it runs for T, θ/2 is slewed out
//! without any step whatever |θ| ≤ STEPT (saturating at 500 ppm for large
//! θ / small T).
//!
//! # Outputs between events (slew expiry)
//!
//! Every cycle: `sx <= (el ≥ 2^(16+pl))` and `x8 <= (el ≥ 2^(19+pl))`
//! (current `time`, registers `tl`, `pl`). `trim = sx ? freq : tr`;
//! `synced = (st == FREQ | st == SYNC) & !x8`. So the slew of an update
//! lasts one poll interval 2^pl s of local time (the trim falls back to
//! `freq` alone): if no update follows (holdover, `valid` = 0, spikes), the
//! clock keeps the learnt frequency and never steps. `sx`/`x8` lag `time`
//! by one cycle.
//!
//! # Timing (cycles from P, the pop cycle)
//!
//! * ignored / `valid = 0` event: `state` new in P+1; IDLE again in P+1.
//! * step: `step` and `state` new in P+1, `step_en` high in P+1 only, so the
//!   clock shows the step in P+2 (its own one-cycle latency); `trim` =
//!   `freq` from P+1. IDLE again in P+1.
//! * process: phases MUL1 (30 cycles, P+1..P+30), SHR (`pn` cycles), MUL2
//!   (17 cycles), FIN (1 cycle): `state` new in P+1, `trim` = old `freq` in
//!   P+1 .. P+48+pn, the new `trim` from cycle **P + 49 + pn** (53..59
//!   cycles), when the unit is IDLE again and may pop the next event. The
//!   clock applies a trim one cycle after it is driven (clock.rs).
//!
//! An event sent on the channel in cycle n (TxHDL `chan`) is popped in cycle
//! n+1 at the earliest (tests `cycles_per_event`, `back_to_back_events_are_paced`).
//! Bound: at most 59 cycles per event (poll 10), 1 cycle for an ignored one.
//!
//! # Closed-loop behaviour
//!
//! Python `simulate()`, poll 64 s unless noted, 100 µs Gaussian measurement
//! noise, 8 seeds (`tools/golden/ntpclient/tests/test_servo.py`):
//!
//! * 50 ms initial offset: removed without a step, |offset| < 1 ms for good
//!   after ≤ 640 s;
//! * 500 ms initial offset: one step at the first update, then < 1 ms;
//! * 50 ppm frequency error: learnt (±0.3 ppm), |offset| < 1 ms for good
//!   after ≤ 448 s, residual RMS ≤ 113 µs (74 µs at poll 1024 s, 73 µs at
//!   ±200..400 ppm);
//! * a ±2 ms single spike: ignored (SPIK), no trace in the offset;
//! * 10 240 s without a valid update: frequency held, no step, drift
//!   ≈ 2 ms (the residual frequency error), `synced` drops after 8 polls.
//!
//! # Deviations from RFC 5905 `local_clock()` (design choices)
//!
//! * One step at most, at start-up (NSET); never after (RFC: step after
//!   STEPOUT in SPIK/FREQ). PANICT is never followed after start-up (RFC:
//!   exit); it is reported (state PANIC, `synced` = 0).
//! * No FLL and no initial frequency measurement (RFC FREQ state waits
//!   STEPOUT and divides): FREQ here is a PI with a larger integral gain for
//!   4 consecutive non-spike updates; a fixed, fast loop (time constant of
//!   a few polls) instead of the RFC's poll-adaptive `tc` and jitter-based
//!   poll control (the poll is a configuration input).
//! * Spike gate in SYNC on the system jitter (`3·jitter`, floor 1 ms, the
//!   popcorn-spike idea of the clock filter) in addition to STEPT; a
//!   single outlier is ignored, the next update is always accepted.
//! * The phase correction is a rate applied for one poll interval (half of
//!   θ), not an exponential amortisation per second.
//!
//! Area:one 80-bit shift-and-add multiplier (`ma`, `mp`, a 32-bit
//! multiplier register `mb`) used twice, no divider, no barrel shifter (the
//! poll shifts are sequential or 7:1 muxes on 17 bits).
use crate::clock::MAXTRIM as MAXTRIM_U128;
use crate::ntp_types::Selected;
use txhdl::comp::{mux, Clock, DefaultClock, In, Out, Reg, Rx, Unit};
use txhdl::types::{Bit, U};
use txhdl::{lower, with, Trace};

/// States (RFC 5905 A.5.5.6 numbering, plus PANIC).
pub const NSET: u8 = 0;
pub const FSET: u8 = 1;
pub const SPIK: u8 = 2;
pub const FREQ: u8 = 3;
pub const SYNC: u8 = 4;
pub const PANIC: u8 = 5;

/// STEPT = 128 ms in 2^-32 s, rounded.
pub const STEPT_Q: u64 = 549_755_814;
/// PANICT = 1000 s in 2^-32 s.
pub const PANICT_Q: u64 = 1000 << 32;
/// STEPOUT = 900 s in `now_s` units (2^-16 s).
pub const STEPOUT_S: u64 = 900 << 16;
/// Spike gate floor, 1 ms in 2^-32 s.
pub const SPKMIN_Q: u64 = 4_294_967;
/// round(2^56 / 1e8): `(a * CQ) >> CS` is the trim removing `a` in 1 s.
pub const CQ: u32 = 720_575_940;
pub const CS: u32 = 24;
/// Proportional and integral gains, as right shifts.
pub const KP: u32 = 1;
pub const KI_FREQ: u32 = 1;
pub const KI_SYNC: u32 = 3;
/// Consecutive non-spike updates in FREQ before SYNC.
pub const FREQ_N: u8 = 4;
/// Trim clamp (clock.rs `MAXTRIM`), and the clamp of one integral step.
pub const MAXTRIM: u32 = MAXTRIM_U128 as u32;
pub const DMAX: u32 = 2 * MAXTRIM;
/// Cycles of the two multiplications.
pub const MUL1_CYCLES: u8 = 30;
pub const MUL2_CYCLES: u8 = 17;
/// Clamp of `poll_log2`.
pub const POLL_MIN: u8 = 4;
pub const POLL_MAX: u8 = 10;

/// Cycles from the pop of a processed event to the first cycle showing the
/// new trim (and the first cycle the unit can pop again).
pub const fn process_cycles(poll: u8) -> u64 {
    1 + MUL1_CYCLES as u64 + poll as u64 + MUL2_CYCLES as u64 + 1
}

// Phases of the sequencer.
const IDLE: u8 = 0;
const MUL1: u8 = 1;
const SHR: u8 = 2;
const MUL2: u8 = 3;
const FIN: u8 = 4;

const SIGN32: u32 = 1 << 31;
const BIAS_HI: u32 = SIGN32 + MAXTRIM;
const BIAS_LO: u32 = SIGN32 - MAXTRIM;
const TRIM_LO: u32 = MAXTRIM.wrapping_neg();

/// `min(x >> p, 2^16)` for `p` in 4..=10 (other values read as 10).
#[lower]
pub fn shr_poll(x: U<48>, p: U<4>) -> U<17> {
    let s = mux(
        Bit::from(p == 4u8),
        x >> 4u32,
        mux(
            Bit::from(p == 5u8),
            x >> 5u32,
            mux(
                Bit::from(p == 6u8),
                x >> 6u32,
                mux(
                    Bit::from(p == 7u8),
                    x >> 7u32,
                    mux(
                        Bit::from(p == 8u8),
                        x >> 8u32,
                        mux(Bit::from(p == 9u8), x >> 9u32, x >> 10u32),
                    ),
                ),
            ),
        ),
    );
    let sat = Bit::from(s >= 65536u32);
    mux(sat, U::<17>::from(65536u32), s.slice::<0, 17>())
}

/// `x >= 2^(16 + p)`: a poll interval has elapsed (`p` in 4..=10, other
/// values read as 10; `p` = 0 after reset reads as 2^16, never used then).
#[lower]
pub fn past_poll(x: U<48>, p: U<4>) -> Bit {
    mux(
        Bit::from(p == 0u8),
        Bit::from(x >= 65536u32),
        mux(
            Bit::from(p == 4u8),
            Bit::from(x >= 1_048_576u32),
            mux(
                Bit::from(p == 5u8),
                Bit::from(x >= 2_097_152u32),
                mux(
                    Bit::from(p == 6u8),
                    Bit::from(x >= 4_194_304u32),
                    mux(
                        Bit::from(p == 7u8),
                        Bit::from(x >= 8_388_608u32),
                        mux(
                            Bit::from(p == 8u8),
                            Bit::from(x >= 16_777_216u32),
                            mux(
                                Bit::from(p == 9u8),
                                Bit::from(x >= 33_554_432u32),
                                Bit::from(x >= 67_108_864u32),
                            ),
                        ),
                    ),
                ),
            ),
        ),
    )
}

/// `x >= 2^(19 + p)`: eight poll intervals have elapsed (same encoding).
#[lower]
pub fn past_8polls(x: U<48>, p: U<4>) -> Bit {
    mux(
        Bit::from(p == 0u8),
        Bit::from(x >= 524_288u32),
        mux(
            Bit::from(p == 4u8),
            Bit::from(x >= 8_388_608u32),
            mux(
                Bit::from(p == 5u8),
                Bit::from(x >= 16_777_216u32),
                mux(
                    Bit::from(p == 6u8),
                    Bit::from(x >= 33_554_432u32),
                    mux(
                        Bit::from(p == 7u8),
                        Bit::from(x >= 67_108_864u32),
                        mux(
                            Bit::from(p == 8u8),
                            Bit::from(x >= 134_217_728u32),
                            mux(
                                Bit::from(p == 9u8),
                                Bit::from(x >= 268_435_456u32),
                                Bit::from(x >= 536_870_912u32),
                            ),
                        ),
                    ),
                ),
            ),
        ),
    )
}

/// Clamp of a two's complement word to [-MAXTRIM, MAXTRIM].
#[lower]
pub fn clamp_trim(x: U<32>) -> U<32> {
    let b = x ^ SIGN32;
    mux(
        Bit::from(b > BIAS_HI),
        U::<32>::from(MAXTRIM),
        mux(Bit::from(b < BIAS_LO), U::<32>::from(TRIM_LO), x),
    )
}

#[derive(Default, Trace)]
pub struct Servo {
    /// Discipline state (NSET, SPIK, FREQ, SYNC, PANIC).
    pub st: Reg<U<3>>,
    /// Sequencer phase (IDLE, MUL1, SHR, MUL2, FIN) and its counter.
    pub ph: Reg<U<3>>,
    pub cnt: Reg<U<5>>,
    /// Integrator (signed trim LSB) and the trim with the slew.
    pub freq: Reg<U<32>>,
    pub tr: Reg<U<32>>,
    /// `now_s` of the last accepted update and its clamped poll.
    pub tl: Reg<U<48>>,
    pub pl: Reg<U<4>>,
    /// An accepted update since the last step / reset.
    pub muv: Reg<Bit>,
    /// SPIK: slewing a persistent large offset.
    pub slw: Reg<Bit>,
    /// Consecutive non-spike updates in FREQ.
    pub fc: Reg<U<3>>,
    /// Last step word and the step pulse.
    pub stp: Reg<U<96>>,
    pub se: Reg<Bit>,
    /// The update being processed: sign, integral gain (KI_SYNC), enable.
    pub sg: Reg<Bit>,
    pub ki3: Reg<Bit>,
    /// Multiplier: multiplicand (shifts left), multiplier (shifts right),
    /// accumulator; `mq` the integral time factor, `sm` the slew magnitude.
    pub ma: Reg<U<80>>,
    pub mb: Reg<U<32>>,
    pub mp: Reg<U<80>>,
    pub mq: Reg<U<17>>,
    pub sm: Reg<U<32>>,
    /// The slew has expired / the last update is stale (one cycle lag).
    pub sx: Reg<Bit>,
    pub x8: Reg<Bit>,
}

#[lower]
impl
    Unit<
        (Rx<Selected>, In<U<8>>, In<U<96>>),
        (Out<U<32>>, Out<U<96>>, Out<Bit>, Out<U<3>>, Out<Bit>),
    > for Servo
{
    async fn run(
        &mut self,
        (sel, poll_log2, time): (Rx<Selected>, In<U<8>>, In<U<96>>),
        (trim, step, step_en, state, synced): (
            Out<U<32>>,
            Out<U<96>>,
            Out<Bit>,
            Out<U<3>>,
            Out<Bit>,
        ),
    ) {
        loop {
            DefaultClock::rising().await;
            let st = self.st.get();
            let ph = self.ph.get();
            let cnt = self.cnt.get();
            let freq = self.freq.get();
            let tr = self.tr.get();
            let tl = self.tl.get();
            let pl = self.pl.get();
            let muv = self.muv.get();
            let slw = self.slw.get();
            let fc = self.fc.get();
            let ma = self.ma.get();
            let mb = self.mb.get();
            let mp = self.mp.get();
            let sg_r = self.sg.get();

            let now_s = time.get().slice::<48, 48>();
            let el = now_s - tl;

            // ------------------------------------------------ IDLE: decision
            let is_idle = Bit::from(ph == IDLE);
            let pop = is_idle & Bit::from(sel.peek().is_some());
            let _ = sel.recv_if(pop);
            let h = sel.head();
            let ev = pop & h.valid;
            let th = h.offset;
            let sg = th.bit(63);
            let a = mux(sg, -th, th);
            let big = Bit::from(a > STEPT_Q);
            let huge = Bit::from(a > PANICT_Q);
            let jt = h.jitter;
            let jbig = Bit::from(jt >= 2_147_483_648u64);
            let j1 = jt.slice::<0, 34>();
            let j3 = (j1 << 1u32) + j1;
            let spk = !jbig & Bit::from(a > j3.zext::<64>()) & Bit::from(a > SPKMIN_Q);
            let pin = poll_log2.get();
            let pn = mux(
                Bit::from(pin < POLL_MIN),
                U::<4>::from(POLL_MIN),
                mux(
                    Bit::from(pin > POLL_MAX),
                    U::<4>::from(POLL_MAX),
                    pin.slice::<0, 4>(),
                ),
            );
            let mq = shr_poll(el, pn);
            let el_so = Bit::from(el >= STEPOUT_S);

            let nset = Bit::from(st == NSET);
            let s_freq = Bit::from(st == FREQ);
            let s_sync = Bit::from(st == SYNC);
            let fs = s_freq | s_sync;
            let sp = !nset & !fs;
            let do_step = ev & nset & big;
            let p_nset = ev & nset & !big;
            let p_sync = ev & s_sync & !big & !spk;
            let p_freq = ev & s_freq & !big;
            let p_spkbig = ev & sp & big & !huge & (el_so | slw);
            let p_spksmall = ev & sp & !big;
            let proc = p_nset | p_sync | p_freq | p_spkbig | p_spksmall;
            let ki_freq = p_nset | p_freq;
            let ien = muv & !p_spkbig;
            let fc_inc = mux(
                Bit::from(fc >= FREQ_N),
                U::<3>::from(FREQ_N),
                fc + U::<3>::from(1u8),
            );
            let fc_n = mux(spk, U::<3>::from(0u8), fc_inc);
            let freq_done = Bit::from(fc_n >= FREQ_N);
            // Next state of a valid event (unused otherwise).
            let st_n = mux(
                nset,
                U::<3>::from(FREQ),
                mux(
                    huge,
                    U::<3>::from(PANIC),
                    mux(
                        big,
                        U::<3>::from(SPIK),
                        mux(
                            sp,
                            U::<3>::from(SYNC),
                            mux(
                                s_sync,
                                mux(spk, U::<3>::from(SPIK), U::<3>::from(SYNC)),
                                mux(freq_done, U::<3>::from(SYNC), U::<3>::from(FREQ)),
                            ),
                        ),
                    ),
                ),
            );
            let step_word = th.concat::<32, 96>(U::<32>::from(0u32));
            let tl_step = now_s + th.slice::<16, 48>();

            // ------------------------------------------------ MUL1 / MUL2
            let add = mux(mb.bit(0), ma, U::<80>::from(0u32));
            let mp_add = mp + add;
            let in_mul1 = Bit::from(ph == MUL1);
            let in_mul2 = Bit::from(ph == MUL2);
            let in_mul = in_mul1 | in_mul2;
            let end_mul1 = in_mul1 & Bit::from(cnt == MUL1_CYCLES - 1);
            let end_mul2 = in_mul2 & Bit::from(cnt == MUL2_CYCLES - 1);

            // ------------------------------------------------ SHR
            let in_shr = Bit::from(ph == SHR);
            let mp_sh = mp >> 1u32;
            let end_shr = in_shr & Bit::from((cnt + U::<5>::from(1u8)) == pl.zext::<5>());
            let sm_n = mux(
                Bit::from((mp_sh >> KP) > MAXTRIM),
                U::<32>::from(MAXTRIM),
                (mp_sh >> KP).slice::<0, 32>(),
            );

            // ------------------------------------------------ FIN
            let in_fin = Bit::from(ph == FIN);
            let dm_w = mux(self.ki3.get(), mp >> (16 + KI_SYNC), mp >> (16 + KI_FREQ));
            let dm = mux(
                Bit::from(dm_w > DMAX),
                U::<32>::from(DMAX),
                dm_w.slice::<0, 32>(),
            );
            let smv = self.sm.get();
            let s = mux(sg_r, -smv, smv);
            let d = mux(sg_r, -dm, dm);
            let f1 = freq + d;
            // Anti-windup: freq moves towards d at most up to lim = MAXTRIM -
            // sm (where freq + slew reaches the clamp), never backwards.
            // Signed compares through the biased form x ^ 2^31.
            let lim = U::<32>::from(MAXTRIM) - smv;
            let nlim = -lim;
            let f1b = f1 ^ SIGN32;
            let fb = freq ^ SIGN32;
            let limb = lim ^ SIGN32;
            let nlimb = nlim ^ SIGN32;
            let f_pos = mux(
                Bit::from(f1b > limb),
                mux(Bit::from(fb > limb), freq, lim),
                f1,
            );
            let f_neg = mux(
                Bit::from(f1b < nlimb),
                mux(Bit::from(fb < nlimb), freq, nlim),
                f1,
            );
            let fnew = mux(sg_r, f_neg, f_pos);
            let trn = clamp_trim(fnew + s);

            with!(self <= {
                se: Bit::Zero,
                sx: past_poll(el, pl),
                x8: past_8polls(el, pl),
                ev ? st: st_n,
                p_freq ? fc: fc_n,
                (p_nset | do_step) ? fc: U::<3>::from(0u8),
                p_spkbig ? slw: Bit::One,
                p_spksmall ? slw: Bit::Zero,
                do_step ? {
                    stp: step_word,
                    se: Bit::One,
                    tl: tl_step,
                    pl: pn,
                    tr: freq,
                    muv: Bit::Zero,
                },
                proc ? {
                    tl: now_s,
                    pl: pn,
                    muv: Bit::One,
                    tr: freq,
                    ph: U::<3>::from(MUL1),
                    cnt: U::<5>::from(0u8),
                    ma: a.zext::<80>(),
                    mb: U::<32>::from(CQ),
                    mp: U::<80>::from(0u32),
                    mq: mux(ien, mq, U::<17>::from(0u32)),
                    sg: sg,
                    ki3: !ki_freq,
                },
                in_mul ? {
                    mp: mp_add,
                    ma: ma << 1u32,
                    mb: mb >> 1u32,
                    cnt: cnt + U::<5>::from(1u8),
                },
                end_mul1 ? {
                    ph: U::<3>::from(SHR),
                    cnt: U::<5>::from(0u8),
                    mp: mp_add >> CS,
                },
                in_shr ? {
                    mp: mp_sh,
                    cnt: cnt + U::<5>::from(1u8),
                },
                end_shr ? {
                    ph: U::<3>::from(MUL2),
                    cnt: U::<5>::from(0u8),
                    ma: mp_sh,
                    mb: self.mq.get().zext::<32>(),
                    mp: U::<80>::from(0u32),
                    sm: sm_n,
                },
                end_mul2 ? ph: U::<3>::from(FIN),
                in_fin ? {
                    freq: fnew,
                    tr: trn,
                    ph: U::<3>::from(IDLE),
                },
            });
            trim.set(mux(self.sx.get(), freq, tr));
            step.set(self.stp.get());
            step_en.set(self.se.get());
            state.set(st);
            synced.set(fs & !self.x8.get());
        }
    }
}
