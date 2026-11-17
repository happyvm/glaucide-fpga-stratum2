// SPDX-License-Identifier: Apache-2.0
//! Entropy source of the NTP client (security finding F3,
//! `docs/security.md`), domain `DefaultClock` (`clk_sys`, 100 MHz).
//!
//! One unit, [`Entropy`]: a xoshiro256++ pseudo-random generator (Blackman
//! and Vigna, 2019; 256 bits of state, 64-bit output) advanced **every
//! cycle** and continuously **stirred** with board events. Its only user is
//! `ntp_client`, which samples `rnd` in the cycle it builds the Transmit
//! Timestamp of a request (the Origin nonce, see `ntp_client.rs`).
//!
//! # No true random number generator
//!
//! This design has **no TRNG**: no ring oscillator, no XADC noise, no
//! FPGA-specific primitive is used, and the output is a deterministic
//! function of (seed, the cycle count since reset, the values of `time` and
//! `ev` in every cycle). What an attacker does not know comes only from:
//!
//! 1. `seed`, a **per-device secret constant** (64 bits). The board build
//!    ties it to [`crate::top_client::ENTROPY_SEED`], a public default: **the
//!    user must set a per-device value** before building a bitstream for a
//!    public server, and keep it secret (it is in the bitstream: whoever
//!    reads the configuration flash learns it).
//! 2. The low bits of `time` (the clock accumulator), mixed in every cycle:
//!    bits 0..31 are below 2^-32 s and are never published (our server sends
//!    `time >> 32`); after the first servo update they depend on the whole
//!    trim history, i.e. on the measured network offsets.
//! 3. `ev`, the receive timestamp of the last upstream reply the client took
//!    (`Meta.rx_ts`, its T4), mixed in every cycle: the arrival time of each
//!    reply at 10 ns resolution, including the network jitter of the path
//!    and the phase between the PHY's receive clock and `clk_sys` (an
//!    off-path attacker does not see our T4). Each new value also changes
//!    the state at an unpredictable cycle.
//! 4. The cycle at which the client samples `rnd` (the generator advances
//!    every cycle: it is its own free-running counter), which moves with the
//!    lookup / resolution delays of each poll.
//!
//! Consequences. The output is **not cryptographic-strength**: xoshiro is a
//! linear generator with a non-linear output function; someone who sees
//! enough outputs and knows every stirring input could rebuild the state.
//! It is sufficient for its purpose because the outputs are never shown to
//! the attacker it defends against: an **off-path** attacker never sees our
//! requests (an on-path one reads the nonce on the wire anyway, and no nonce
//! helps against it). Bound on prediction: an off-path attacker who knows
//! the design and the public default seed but not the secret seed faces
//! 2^64 seeds; with the per-device seed secret, each request's 32 random
//! bits cost him 2^-32 per forged packet (see `ntp_client.rs`). With the
//! **default seed left in place**, only the unpublished inputs (2 to 4
//! above) remain: before the first upstream reply is received they are
//! roughly the request's cycle position (the ARP/ND resolution delay, about
//! 10 bits), so the first iburst requests after a reset are weak; each
//! accepted reply then adds the jitter of its T4.
//!
//! Future work (stronger source): a ring-oscillator TRNG (free-running
//! inverter loops sampled by `clk_sys`, with a health test, as in the
//! AIS 31 / SP 800-90B designs), or the XADC's noisy LSBs, conditioned by a
//! cryptographic hash or a block cipher (AES-CTR DRBG); and a seed written
//! per device at programming time (eFUSE or a reserved flash sector).
//!
//! # Ports (flat, in this order)
//!
//! | port | type | meaning |
//! |---|---|---|
//! | `seed` | `In<U<64>>` | per-device secret constant, sampled once, in the first cycle after reset |
//! | `time` | `In<U<96>>` | the clock accumulator (bits 0..63 are used) |
//! | `ev` | `In<U<64>>` | receive timestamp of the last upstream reply (`ntp_client`'s `rx_stamp`) |
//! | `rnd` | `Out<U<64>>` | a fresh 64-bit value in every cycle (registered output) |
//!
//! # Behaviour (bit-exact, model `entropy_model` of `rtl/tests/entropy.rs`)
//!
//! State `s0..s3` (64 bits each, reset 0), `init` (reset 0), output
//! pipeline `sum`, `s0d`, `out`. In every cycle, with `T` = `time` bits
//! 0..63 and `E` = `ev` of that cycle:
//!
//! * `init` = 0 (the first cycle after reset): `s0 := seed ^ K0`, `s1 :=
//!   seed ^ K1`, `s2 := K2`, `s3 := K3`, `init := 1` (the constants are the
//!   SplitMix64 constants: the state is never all zero).
//! * `init` = 1: one xoshiro256++ step, then the stirring:
//!   `t = s1 << 17; a2 = s2 ^ s0; a3 = s3 ^ s1; s1 := s1 ^ a2 ^ E;
//!   s0 := s0 ^ a3; s2 := a2 ^ t ^ T; s3 := rotl(a3, 45)`.
//! * Output (two pipeline stages, in every cycle, from the state of the
//!   cycle): `sum := s0 + s3`, `s0d := s0`, `out := rotl(sum, 23) + s0d`
//!   (all modulo 2^64), `rnd` = `out`. Without stirring (`T` = `E` = 0)
//!   this is exactly the xoshiro256++ sequence, two cycles late.
//!
//! Timing: the state update is two levels of XOR (one LUT6 per bit plus the
//! `init` multiplexer); each of the two 64-bit additions has its own
//! pipeline stage. Cost: 7 × 64 + 1 = 449 flip-flops (state, `init`, the
//! three output registers), two 64-bit adders.
use txhdl::comp::{mux, Clock, DefaultClock, In, Out, Reg, Unit};
use txhdl::types::{Bit, U};
use txhdl::{check, cover, lower, with, Trace};

/// Seeding constants (SplitMix64: the golden ratio and its two multipliers,
/// and the `xorshift*` multiplier): nonzero, so the state never starts at 0.
pub const K0: u64 = 0x9E37_79B9_7F4A_7C15;
pub const K1: u64 = 0xBF58_476D_1CE4_E5B9;
pub const K2: u64 = 0x94D0_49BB_1331_11EB;
pub const K3: u64 = 0x2545_F491_4F6C_DD1D;

/// `x` rotated left by `k` (0 < k < 64).
#[lower]
fn rotl23(x: U<64>) -> U<64> {
    (x << 23u32) | (x >> 41u32)
}
#[lower]
fn rotl45(x: U<64>) -> U<64> {
    (x << 45u32) | (x >> 19u32)
}

#[derive(Trace, Default)]
pub struct Entropy {
    pub s0: Reg<U<64>>,
    pub s1: Reg<U<64>>,
    pub s2: Reg<U<64>>,
    pub s3: Reg<U<64>>,
    /// The state has been seeded.
    pub init: Reg<Bit>,
    /// Output pipeline: `s0 + s3`, `s0` of the same cycle, the output.
    pub sum: Reg<U<64>>,
    pub s0d: Reg<U<64>>,
    pub out: Reg<U<64>>,
    // Monitors (only read by check!/cover!; pruned in synthesis): the previous
    // cycle (it exists, its state, its inputs), the `s0`/`s3` of two cycles
    // ago, and whether a non-zero stirring word was ever mixed in.
    pub m_v: Reg<Bit>,
    pub m_ini: Reg<Bit>,
    pub m_s0: Reg<U<64>>,
    pub m_s1: Reg<U<64>>,
    pub m_s2: Reg<U<64>>,
    pub m_s3: Reg<U<64>>,
    pub m_e: Reg<U<64>>,
    pub m_t: Reg<U<64>>,
    pub m_seed: Reg<U<64>>,
    pub m2_s0: Reg<U<64>>,
    pub m2_s3: Reg<U<64>>,
    pub m_stir: Reg<Bit>,
}

#[lower]
impl Unit<(In<U<64>>, In<U<96>>, In<U<64>>), Out<U<64>>> for Entropy {
    async fn run(&mut self, (seed, time, ev): (In<U<64>>, In<U<96>>, In<U<64>>), rnd: Out<U<64>>) {
        loop {
            DefaultClock::rising().await;
            rnd.set(self.out.get());
            let s0 = self.s0.get();
            let s1 = self.s1.get();
            let s2 = self.s2.get();
            let s3 = self.s3.get();
            let ini = self.init.get();
            let sd = seed.get();
            let tt = time.get().slice::<0, 64>();
            let e = ev.get();
            // xoshiro256++ state step, then the stirring words.
            let t = s1 << 17u32;
            let a2 = s2 ^ s0;
            let a3 = s3 ^ s1;
            let n0 = s0 ^ a3;
            let n1 = (s1 ^ a2) ^ e;
            let n2 = (a2 ^ t) ^ tt;
            let n3 = rotl45(a3);
            // ---- formal properties (orchestrator; security finding F3) ----
            // xoshiro256++ as published (Blackman and Vigna, `next()`), on the
            // state of the previous cycle: t = s1 << 17; s2 ^= s0; s3 ^= s1;
            // s1 ^= s2; s0 ^= s3; s2 ^= t; s3 = rotl(s3, 45).
            let m_v = self.m_v.get();
            let m_ini = self.m_ini.get();
            let p0 = self.m_s0.get();
            let p1 = self.m_s1.get();
            let p2 = self.m_s2.get();
            let p3 = self.m_s3.get();
            let x_t = p1 << 17u32;
            let x_s2 = p2 ^ p0;
            let x_s3 = p3 ^ p1;
            let x_s1 = p1 ^ x_s2;
            let x_s0 = p0 ^ x_s3;
            let x_s2b = x_s2 ^ x_t;
            let x_s3b = (x_s3 << 45u32) | (x_s3 >> 19u32);
            let zero = Bit::from(s0 == 0) & Bit::from(s1 == 0) & Bit::from(s2 == 0) & Bit::from(s3 == 0);
            let q0 = self.m2_s0.get();
            let q_sum = q0 + self.m2_s3.get();
            let x_out = ((q_sum << 23u32) | (q_sum >> 41u32)) + q0;
            check!(!ini | m_v, "the state is seeded in the first cycle after reset, not before");
            check!(!m_v | ini, "the state is seeded after the first cycle");
            check!(!(m_v & !m_ini) | (Bit::from(s0 == (self.m_seed.get() ^ 0x9E37_79B9_7F4A_7C15u64)) & Bit::from(s1 == (self.m_seed.get() ^ 0xBF58_476D_1CE4_E5B9u64)) & Bit::from(s2 == 0x94D0_49BB_1331_11EBu64) & Bit::from(s3 == 0x2545_F491_4F6C_DD1Du64)), "seeding: s0 = seed ^ K0, s1 = seed ^ K1, s2 = K2, s3 = K3 (never all zero)");
            check!(!(m_v & m_ini) | (Bit::from(s0 == x_s0) & Bit::from(s3 == x_s3b)), "one xoshiro256++ step per cycle: s0 and s3");
            check!(!(m_v & m_ini) | (Bit::from(s1 == (x_s1 ^ self.m_e.get())) & Bit::from(s2 == (x_s2b ^ self.m_t.get()))), "one xoshiro256++ step per cycle: s1 and s2, stirred by XOR with ev and time (the stirring cannot cancel the generator)");
            check!(!(ini & !self.m_stir.get()) | !zero, "without stirring the state is never all zero (xoshiro256++ is a bijection of the non-zero states, seeded non-zero)");
            check!(Bit::from(self.sum.get() == p0 + p3) & Bit::from(self.s0d.get() == p0), "output stage 1: s0 + s3 and s0 of the previous cycle");
            check!(Bit::from(self.out.get() == x_out), "rnd = rotl(s0 + s3, 23) + s0 of the state two cycles earlier (xoshiro256++ output)");
            cover!(ini & !zero & !Bit::from(self.out.get() == 0), "a non-zero output from a seeded state");
            cover!(ini & zero, "an adversarial stirring can cancel the state (the inputs are not adversarial: see the header)");
            cover!(ini & self.m_stir.get() & !zero, "a stirred state");

            with!(self <= {
                m_v: Bit::One,
                m_ini: ini,
                m_s0: s0,
                m_s1: s1,
                m_s2: s2,
                m_s3: s3,
                m_e: e,
                m_t: tt,
                m_seed: sd,
                m2_s0: p0,
                m2_s3: p3,
                m_stir: self.m_stir.get() | (ini & (!Bit::from(e == 0) | !Bit::from(tt == 0))),
                s0: mux(ini, n0, sd ^ U::<64>::from(K0)),
                s1: mux(ini, n1, sd ^ U::<64>::from(K1)),
                s2: mux(ini, n2, U::<64>::from(K2)),
                s3: mux(ini, n3, U::<64>::from(K3)),
                init: Bit::One,
                sum: s0 + s3,
                s0d: s0,
                out: rotl23(self.sum.get()) + self.s0d.get(),
            });
        }
    }
}
