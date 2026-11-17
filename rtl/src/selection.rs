// SPDX-License-Identifier: Apache-2.0
//! Clock selection, clustering and combining (task T4.4), RFC 5905 §11.2
//! and Appendix A.5.5, for the `PEERS` = 4 upstream servers (`ntp_types`). Domain
//! `DefaultClock` (`clk_sys`, 100 MHz).
//!
//! The specification is the Python model
//! `tools/golden/ntpclient/selection.py` (`Selection.process`): this unit is
//! bit-exact with it, checked on `tools/golden/ntpclient/vectors/selection.txt`
//! by `rtl/tests/selection.rs`.
//!
//! # Ports (flat, in this order)
//!
//! | port | type | meaning |
//! |---|---|---|
//! | `stats` | `Rx<PeerStat>` | one per update of a peer, from the clock filter (T4.3) |
//! | `now` | `In<U<64>>` | our NTP time, 32.32 (ageing of the samples) |
//! | `n_en` | `In<U<3>>` | number of enabled upstream slots (a configuration constant, `top.rs` `n_enabled`); start-up majority, step 3b |
//! | `selected` | `Tx<Selected>` | one per `PeerStat` taken, to the servo (T4.5) and the sync state (T4.6) |
//!
//! # Timing and flow
//!
//! * A `PeerStat` is taken only when the unit is idle (`st` = IDLE), in the
//!   first cycle it is present (**cycle T**). In cycle T it is stored as the
//!   latest state of peer `peer` (replacing the previous one; a `PeerStat`
//!   with `valid = 0` also replaces it) and `now` is sampled (`now_r`): the
//!   whole event uses that one value of `now`. `now` is not read otherwise.
//! * The unit then computes for a fixed number of cycles that depends only on
//!   the number of survivors `n` (below; an event stopped by the start-up
//!   majority of step 3b counts as `n` = 0), with no other input:
//!   [`event_cycles`] gives the cycle (counted from T = 0) in which
//!   `selected` is offered, at most [`MAX_EVENT_CYCLES`] = 545 cycles
//!   (5.45 µs at 100 MHz).
//! * `selected` is offered from that cycle until taken (the channel's
//!   `ready`); exactly one `Selected` per `PeerStat` taken, in order. The
//!   next `PeerStat` is taken at the earliest in the cycle after the
//!   `Selected` is sent (no overlap: a stalled consumer stalls `stats`).
//! * After reset the 4 peers are empty (all zero: `valid` = 0, ineligible),
//!   there is no previous system peer, and `ever` = 0 (no valid selection
//!   yet, step 3b).
//!
//! # Arithmetic (all exact integers; Q32 = 2^-32 s)
//!
//! Offsets are 64-bit two's complement (±2^31 s), everything else unsigned.
//! For each stored peer `i` (in the event, with `now_r`):
//!
//! 1. **Root distance** (A.5.5.2 `root_dist`), at most 67 bits wide inside:
//!    * `age = now_r − t4` if `now_r ≥ t4` (unsigned) else 0, then
//!      `age = min(age, 2^50 − 1)` ([`AGE_MAX`], 2^18 s);
//!    * `phi = ((age >> 16) × PHI_Q32) >> 16` (two floors; PHI_Q32 = 64425,
//!      15 ppm, from `ntp_types`);
//!    * `half = max(MINDISP_Q32, (root_delay << 16) + delay) >> 1`
//!      ([`MINDISP_Q32`] = 42949673 = 0.01 s rounded to nearest);
//!    * `rd = half + (root_disp << 16) + disp + jitter + phi + RHO_Q32`
//!      ([`RHO_Q32`] = 256 = 2^-24 s, our precision, as a constant);
//!    * stored as `min(rd, 2^33 − 1)` (33 bits: only eligible peers, whose
//!      `rd` < MAXDIST < 2^33, ever use it, so it is exact where used).
//! 2. **Eligibility** (A.5.5.3 `fit`): `valid`, `reach ≠ 0`,
//!    `1 ≤ stratum ≤ 15` (stratum 0 reads as MAXSTRAT, §7.3), `leap ≠ 3` and
//!    `rd < MAXDIST_Q32` ([`MAXDIST_Q32`] = 3·2^31 = 1.5 s). `m` = number of
//!    eligible peers (0..=4).
//! 3. **Intersection** (§11.2.1, A.5.5.1, Marzullo), over the eligible peers:
//!    the tuples (θ − rd, low), (θ, mid), (θ + rd, high) as 66-bit signed
//!    edges, totally ordered by (edge, type low < mid < high, peer): the
//!    intervals are closed. For `allow` = 0 and, if `m ≥ 3`, 1 (i.e.
//!    `2·allow < m`): `low` = the first tuple, in that order, where
//!    (#low − #high tuples up to it, inclusive) ≥ `m − allow`; `high` = the
//!    last tuple where (#high − #low from it to the end) ≥ `m − allow`;
//!    `found` = midpoints before `low` + midpoints after `high`. Success for
//!    the first `allow` with `found ≤ allow` and edge(high) > edge(low)
//!    strictly. **No success: no selection** (`valid = 0`): the intersection
//!    must be non-empty and contain a majority. The **truechimers** are the
//!    eligible peers with edge(low) ≤ θ ≤ edge(high); there is at least one
//!    on success, and any two are less than 2·MAXDIST = 3 s (< 2^34 Q32)
//!    apart (a majority of the intervals contains [low, high]).
//!
//!    **3b. Start-up majority** (security finding F4; RFC 5905's NSANE /
//!    MINCLOCK in spirit). While `ever` = 0 (no valid selection since
//!    reset), success also needs at least `n_en / 2 + 1` truechimers (floor):
//!    1 enabled slot: 1 (the single source decides: the **residual risk**,
//!    nothing else is possible; configure three or more upstreams); 2: both,
//!    i.e. their intersection; 3: 2; 4: 3 (`n_en` above 4 asks for more than
//!    the 4 peers can give: never). Otherwise there is no selection (as for
//!    a failed intersection: `valid = 0`, the previous system peer is
//!    cleared). `ever` := 1 with the first valid selection and stays 1 until
//!    reset: afterwards the usual rules (a majority of the *eligible*
//!    peers). Effect: the servo's start-up step (NSET, any size, `servo.rs`)
//!    is no longer decided by the first source to reach 4 samples. The
//!    hardware counts the truechimers for both allows (`n_t0`, `n_t1`, the
//!    survivor tests below) and registers the comparisons (`g0`, `g1`) in
//!    every cycle; FIN1 reads them (their inputs are final from the end of
//!    INTER): no cycle is added.
//!    Hardware: no sort; each of the 12 tuples is compared with the 12
//!    (one comparison per cycle) to get its rank and the counts above. The
//!    comparator is a two-stage pipeline (task T5.3): the edges of a pair
//!    are registered (`e_*`), then their comparison (`c_*`), and the counts
//!    are updated from the registered comparison, two cycles after the pair
//!    is issued; the decision pairs (`lk`, `hk`) go through the same
//!    pipeline.
//! 4. **Clustering** (§11.2.2) with NMIN = 3: only when the 4 peers are
//!    truechimers. `S_p = Σ_q (θ_p − θ_q)²` (exact, Q64, < 2^70);
//!    `qmax` = the largest `S_p` (strict, lowest index on a tie);
//!    `jmin` = the smallest peer jitter. If `S_qmax ≥ 3·jmin²` (the RMS
//!    selection jitter sqrt(S/(n−1)) is not below the smallest peer jitter)
//!    `qmax` is dropped, leaving 3 survivors (= NMIN: stop). Otherwise the 4
//!    survive.
//! 5. **System peer**: `best` = the survivor of smallest (stratum, rd), lowest
//!    index on a tie (the A.5.5.1 metric MAXDIST·stratum + rd). The previous
//!    system peer `o` is kept instead iff it is a survivor, has the stratum of
//!    `best` (A.5.5.1: no clock hop) and `rd_o ≤ 3·rd_best` (our guard, SGATE
//!    = 3). Otherwise `best` is chosen. The chosen peer `s` becomes the
//!    previous system peer; an event with no selection clears it.
//! 6. **Combine** (§11.2.3), over the survivors `i`:
//!    `W_i = 2^48 / rd_i` (floor; 2^15 < W_i < 2^24), `Δ_i = θ_i − θ_s`
//!    (|Δ_i| < 2^34), `N = Σ W_i·Δ_i` (signed, |N| < 2^60), `D = Σ W_i`
//!    (< 2^26), `J = Σ W_i·Δ_i²` (< 2^94);
//!    `offset = θ_s + sign(N)·⌊|N| / D⌋` (truncation toward zero, mod 2^64);
//!    `jitter = ⌊sqrt(⌊J / D⌋ + jitter_s²)⌋` (< 2^35: the system jitter is
//!    sqrt(selection jitter² + peer jitter²) with the weights 1/rd). With a
//!    single survivor the offset is exactly θ_s and the jitter jitter_s.
//!
//! # Output
//!
//! `Selected { valid: 1, peer: s, offset, jitter, root_dist: rd_s, stratum,
//! leap, ref_id, root_delay, root_disp, delay, disp, t4 }` (the last eight
//! copied from the stored `PeerStat` of `s`), or all fields zero when there
//! is no selection (`m = 0` or no majority).
//!
//! # Datapath
//!
//! One 34×34 multiplier with registered operands (`ma`, `mb`) and two
//! pipeline registers (`mp`, `mq`): issued in sub-phase `ph` = 0, read in
//! `ph` = 3. One restoring divider (one quotient bit per cycle: 24 bits for
//! `W_i`, 34 for the offset, 68 for the variance) and one digit-by-digit
//! integer square root (35 steps) sharing the shift register `dx`.
//!
//! # Deviations from RFC 5905 (decisions, see the report of T4.4)
//!
//! * The MAXDIST test is `rd < MAXDIST` (no `PHI·2^poll` allowance) and there
//!   is no loop test on the reference ID (`fit`); no NSANE/MINCLOCK counts
//!   except the start-up majority of step 3b.
//! * Clustering runs the §11.2.2 test with the RMS selection jitter; the
//!   A.5.5.1 pseudocode bugs (`qmax = q`, survivors gathered from all tuples)
//!   are not reproduced.
//! * Hysteresis adds the `rd_o ≤ 3·rd_best` guard to the "same stratum, no
//!   clock hop" rule.
//! * The combine statistic uses θ of the system peer, the system jitter adds
//!   the system peer's jitter (§11.2.3), all with the fixed-point rules above.
//!
//! # Facts for the formal properties (orchestrator)
//!
//! * Flow: a `PeerStat` is taken only in IDLE; between a take and the send
//!   of its `Selected` no other `PeerStat` is taken; one `Selected` per take,
//!   sent `event_cycles(n)` cycles after the take when `selected` is ready
//!   (bounded liveness: ≤ [`MAX_EVENT_CYCLES`] cycles without backpressure).
//! * `valid = 0` ⇒ every other field of `Selected` is 0, and the previous
//!   system peer is cleared (`prev_v` = 0).
//! * `valid = 1` ⇒ `peer` is eligible (`el`), a survivor (`sv`), its
//!   `root_dist` < MAXDIST, its `stratum` in 1..=15, its `leap` ≠ 3; the
//!   survivors are a subset of the eligible peers and number at least
//!   `m − allow` > m/2 before clustering (a majority of the eligible peers).
//! * `ever = 0` and `valid = 1` ⇒ the truechimers number at least
//!   `n_en / 2 + 1` (step 3b); `ever` never returns to 0.
//! * A falseticker (eligible, θ outside [low, high]) has `sv` = 0 and enters
//!   none of `nn`, `dd`, `jj`: it does not influence `offset` or `jitter`.
//!   Its own fields can change freely without changing the output, as long
//!   as it stays a falseticker.
//! * One survivor `s`: `offset` = θ_s and `jitter` = jitter_s exactly.
//! * Ranges: in INTER, `ki`, `ji` ≤ 11, `rk` ≤ 11, the six counts ≤ 4; in the
//!   combine, |N| < 2^60 (`nn`, two's complement), `dd` < 2^26, `jj` < 2^94;
//!   the divider remainder `dr` < divisor; `res_jit` < 2^35.
//!
//! # Integration
//!
//! Inputs and output on registered channels (stream.rs, netlist note). To be
//! added to `rtl/src/bin/gen_all.rs` / `tools/gen_verilog.sh` (`selection`)
//! by the orchestrator (not done here: files outside this task).
use crate::ntp_types::{PeerStat, Selected, PHI_Q32};
use txhdl::comp::{mux, Clock, DefaultClock, In, Reg, Rx, Tx, Unit};
use txhdl::types::{Bit, U};
use txhdl::{check, cover, lower, with, Trace};

/// 1.5 s, RFC 5905 MAXDIST, Q32.
pub const MAXDIST_Q32: u64 = 3 << 31u32;
/// 0.01 s, RFC 5905 MINDISP, Q32 (42949672.96 rounded to nearest).
pub const MINDISP_Q32: u64 = 42_949_673;
/// Our precision 2^-24 s, Q32, added to every root distance.
pub const RHO_Q32: u64 = 1 << 8u32;
/// Ageing clamp (2^18 s).
pub const AGE_MAX: u64 = (1 << 50u32) - 1;
/// Largest number of cycles from the take of a `PeerStat` (cycle 0) to the
/// first cycle `selected` is offered.
pub const MAX_EVENT_CYCLES: u64 = 545;

/// Cycle (counted from the take, cycle 0) in which `selected` is first
/// offered, for `n` survivors (0 when there is no selection); `n = 4` means
/// 4 truechimers, whether clustering then keeps 4 or 3.
pub const fn event_cycles(n: u64, truechimers4: bool) -> u64 {
    // 1 take + 16 distances + 144 intersection (+ 2 for the comparator
    // pipeline) + 4 decision (two pairs through the comparator pipeline).
    let front = 1 + 16 + 146 + 4;
    if n == 0 {
        return front;
    }
    // Clustering: 16 squares + jmin² (4 cycles each).
    let clu = if truechimers4 { 64 + 4 } else { 0 };
    // System peer 4 + 1, combine: 1 per peer, plus 24 + 16 per survivor,
    // jitter_s² 4, divisions 34 + 68, square root 35.
    front + clu + 5 + 4 + 40 * n + 4 + 34 + 68 + 35
}

const IDLE: u8 = 0;
const DIST: u8 = 1;
const INTER: u8 = 2;
const FIN0: u8 = 3;
const FIN1: u8 = 4;
const CLU: u8 = 5;
const CLU2: u8 = 6;
const SYS: u8 = 7;
const SYS2: u8 = 8;
const CMBI: u8 = 9;
const DIVW: u8 = 10;
const CMBM: u8 = 11;
const FPHI: u8 = 12;
const DIVN: u8 = 13;
const DIVJ: u8 = 14;
const SQRT: u8 = 15;
const OUT: u8 = 16;
const FINA: u8 = 17;
const FINB: u8 = 18;

#[lower]
fn pick64(i: U<2>, a: U<64>, b: U<64>, c: U<64>, d: U<64>) -> U<64> {
    mux(i == 0, a, mux(i == 1, b, mux(i == 2, c, d)))
}
#[lower]
fn pick33(i: U<2>, a: U<33>, b: U<33>, c: U<33>, d: U<33>) -> U<33> {
    mux(i == 0, a, mux(i == 1, b, mux(i == 2, c, d)))
}
#[lower]
fn pick32(i: U<2>, a: U<32>, b: U<32>, c: U<32>, d: U<32>) -> U<32> {
    mux(i == 0, a, mux(i == 1, b, mux(i == 2, c, d)))
}
#[lower]
fn pick8(i: U<2>, a: U<8>, b: U<8>, c: U<8>, d: U<8>) -> U<8> {
    mux(i == 0, a, mux(i == 1, b, mux(i == 2, c, d)))
}
#[lower]
fn pick2(i: U<2>, a: U<2>, b: U<2>, c: U<2>, d: U<2>) -> U<2> {
    mux(i == 0, a, mux(i == 1, b, mux(i == 2, c, d)))
}
#[lower]
fn pickb(i: U<2>, a: Bit, b: Bit, c: Bit, d: Bit) -> Bit {
    mux(i == 0, a, mux(i == 1, b, mux(i == 2, c, d)))
}
/// |a − b| of two signed 64-bit values known to be less than 2^34 apart.
#[lower]
fn absdiff34(a: U<64>, b: U<64>) -> U<34> {
    let d = a - b;
    let m = mux(d.bit(63), U::<64>::from(0u8) - d, d);
    m.slice::<0, 34>()
}

#[derive(Trace, Default)]
pub struct Selection {
    /// State (IDLE .. OUT).
    pub st: Reg<U<5>>,
    /// `now` sampled when the PeerStat was taken.
    pub now_r: Reg<U<64>>,
    // ---- latest PeerStat of each peer ----
    /// valid & reach != 0 & 1 <= stratum <= 15 & leap != 3.
    pub ok0: Reg<Bit>,
    pub ok1: Reg<Bit>,
    pub ok2: Reg<Bit>,
    pub ok3: Reg<Bit>,
    pub off0: Reg<U<64>>,
    pub off1: Reg<U<64>>,
    pub off2: Reg<U<64>>,
    pub off3: Reg<U<64>>,
    pub dly0: Reg<U<64>>,
    pub dly1: Reg<U<64>>,
    pub dly2: Reg<U<64>>,
    pub dly3: Reg<U<64>>,
    pub dsp0: Reg<U<64>>,
    pub dsp1: Reg<U<64>>,
    pub dsp2: Reg<U<64>>,
    pub dsp3: Reg<U<64>>,
    pub jit0: Reg<U<64>>,
    pub jit1: Reg<U<64>>,
    pub jit2: Reg<U<64>>,
    pub jit3: Reg<U<64>>,
    pub t40: Reg<U<64>>,
    pub t41: Reg<U<64>>,
    pub t42: Reg<U<64>>,
    pub t43: Reg<U<64>>,
    pub str0: Reg<U<8>>,
    pub str1: Reg<U<8>>,
    pub str2: Reg<U<8>>,
    pub str3: Reg<U<8>>,
    pub lp0: Reg<U<2>>,
    pub lp1: Reg<U<2>>,
    pub lp2: Reg<U<2>>,
    pub lp3: Reg<U<2>>,
    pub rid0: Reg<U<32>>,
    pub rid1: Reg<U<32>>,
    pub rid2: Reg<U<32>>,
    pub rid3: Reg<U<32>>,
    pub rdl0: Reg<U<32>>,
    pub rdl1: Reg<U<32>>,
    pub rdl2: Reg<U<32>>,
    pub rdl3: Reg<U<32>>,
    pub rds0: Reg<U<32>>,
    pub rds1: Reg<U<32>>,
    pub rds2: Reg<U<32>>,
    pub rds3: Reg<U<32>>,
    // ---- per event ----
    /// Root distance, min(rd, 2^33 - 1), and eligibility.
    pub rd0: Reg<U<33>>,
    pub rd1: Reg<U<33>>,
    pub rd2: Reg<U<33>>,
    pub rd3: Reg<U<33>>,
    pub el0: Reg<Bit>,
    pub el1: Reg<Bit>,
    pub el2: Reg<Bit>,
    pub el3: Reg<Bit>,
    /// Root distance of peer `ia` without its `phi` term (written every
    /// cycle, read in phase 3 of the distance step).
    pub rdx: Reg<U<67>>,
    /// Peer loop index (distances, system peer, combine).
    pub ia: Reg<U<2>>,
    /// Multiplier sub-phase: issue at 0, result at 3.
    pub ph: Reg<U<2>>,
    // ---- intersection ----
    /// Tuples k (outer) and j (inner): bits 3:2 type (0 low, 1 mid, 2 high), 1:0 peer.
    pub ki: Reg<U<4>>,
    pub ji: Reg<U<4>>,
    /// All 144 pairs have been issued to the comparator pipeline.
    pub idone: Reg<Bit>,
    /// Comparator pipeline, stage E: the pair (j, k) issued in the previous
    /// cycle and its two edges.
    pub e_v: Reg<Bit>,
    pub e_j: Reg<U<4>>,
    pub e_k: Reg<U<4>>,
    pub e_ej: Reg<U<66>>,
    pub e_ek: Reg<U<66>>,
    /// Stage C: the pair and its comparison (edge j < edge k signed; tuple j
    /// before tuple k in the total order; same tuple).
    pub c_v: Reg<Bit>,
    pub c_j: Reg<U<4>>,
    pub c_k: Reg<U<4>>,
    pub c_lt: Reg<Bit>,
    pub c_klt: Reg<Bit>,
    pub c_keq: Reg<Bit>,
    /// Counts over j so far, for tuple k: rank (tuples before k), lows/highs
    /// at or before k, highs/lows at or after k, mids before/after k.
    pub rk: Reg<U<4>>,
    pub a_lole: Reg<U<3>>,
    pub a_hile: Reg<U<3>>,
    pub a_hige: Reg<U<3>>,
    pub a_loge: Reg<U<3>>,
    pub a_mlt: Reg<U<3>>,
    pub a_mgt: Reg<U<3>>,
    /// Lower / upper endpoint for allow = 0 and 1: found, tuple, rank,
    /// midpoints passed.
    pub lv0: Reg<Bit>,
    pub lk0: Reg<U<4>>,
    pub lr0: Reg<U<4>>,
    pub lf0: Reg<U<3>>,
    pub lv1: Reg<Bit>,
    pub lk1: Reg<U<4>>,
    pub lr1: Reg<U<4>>,
    pub lf1: Reg<U<3>>,
    pub hv0: Reg<Bit>,
    pub hk0: Reg<U<4>>,
    pub hr0: Reg<U<4>>,
    pub hf0: Reg<U<3>>,
    pub hv1: Reg<Bit>,
    pub hk1: Reg<U<4>>,
    pub hr1: Reg<U<4>>,
    pub hf1: Reg<U<3>>,
    /// Rank of each peer's midpoint tuple.
    pub mr0: Reg<U<4>>,
    pub mr1: Reg<U<4>>,
    pub mr2: Reg<U<4>>,
    pub mr3: Reg<U<4>>,
    /// Allow = 0 succeeded (decided in FIN0).
    pub s0: Reg<Bit>,
    /// Start-up majority (F4): the truechimers of allow = 0 / allow = 1
    /// number at least `n_en / 2 + 1` (registered every cycle; read in
    /// FIN1, its inputs are constant from the end of INTER).
    pub g0: Reg<Bit>,
    pub g1: Reg<Bit>,
    /// A valid selection has been made since reset (F4).
    pub ever: Reg<Bit>,
    /// Survivors.
    pub sv0: Reg<Bit>,
    pub sv1: Reg<Bit>,
    pub sv2: Reg<Bit>,
    pub sv3: Reg<Bit>,
    // ---- clustering ----
    pub cp: Reg<U<2>>,
    pub cq: Reg<U<2>>,
    pub acc: Reg<U<70>>,
    pub maxs: Reg<U<70>>,
    pub qmax: Reg<U<2>>,
    pub jmin: Reg<U<34>>,
    // ---- system peer ----
    pub bv: Reg<Bit>,
    pub bi: Reg<U<2>>,
    pub bs: Reg<U<8>>,
    pub brd: Reg<U<33>>,
    pub sp: Reg<U<2>>,
    pub prev_v: Reg<Bit>,
    pub prev_p: Reg<U<2>>,
    // ---- combine ----
    pub ms: Reg<U<2>>,
    pub w: Reg<U<24>>,
    pub sq: Reg<U<68>>,
    pub nn: Reg<U<64>>,
    pub dd: Reg<U<27>>,
    pub jj: Reg<U<95>>,
    // ---- multiplier ----
    pub ma: Reg<U<34>>,
    pub mb: Reg<U<34>>,
    pub mp: Reg<U<68>>,
    pub mq: Reg<U<68>>,
    // ---- divider / square root ----
    pub dx: Reg<U<96>>,
    pub dr: Reg<U<34>>,
    pub dq: Reg<U<68>>,
    pub sr: Reg<U<39>>,
    pub dc: Reg<U<7>>,
    // ---- result ----
    pub ok: Reg<Bit>,
    /// Monitor (check!/cover! only): the decision left at least one survivor.
    pub m_ne: Reg<Bit>,
    pub res_off: Reg<U<64>>,
    pub res_jit: Reg<U<35>>,
}

#[lower]
impl Unit<(Rx<PeerStat>, In<U<64>>, In<U<3>>), Tx<Selected>> for Selection {
    async fn run(
        &mut self,
        (stats, now, n_en): (Rx<PeerStat>, In<U<64>>, In<U<3>>),
        selected: Tx<Selected>,
    ) {
        loop {
            DefaultClock::rising().await;
            let st = self.st.get();
            let ph = self.ph.get();
            let ia = self.ia.get();
            let ph3 = Bit::from(ph == 3);
            let s_idle = Bit::from(st == IDLE);
            let s_dist = Bit::from(st == DIST);
            let s_inter = Bit::from(st == INTER);
            let s_fin0 = Bit::from(st == FIN0);
            let s_fin1 = Bit::from(st == FIN1);
            let s_clu = Bit::from(st == CLU);
            let s_clu2 = Bit::from(st == CLU2);
            let s_sys = Bit::from(st == SYS);
            let s_sys2 = Bit::from(st == SYS2);
            let s_cmbi = Bit::from(st == CMBI);
            let s_divw = Bit::from(st == DIVW);
            let s_cmbm = Bit::from(st == CMBM);
            let s_fphi = Bit::from(st == FPHI);
            let s_divn = Bit::from(st == DIVN);
            let s_divj = Bit::from(st == DIVJ);
            let s_sqrt = Bit::from(st == SQRT);
            let s_out = Bit::from(st == OUT);
            let s_fina = Bit::from(st == FINA);
            let s_finb = Bit::from(st == FINB);

            let off0 = self.off0.get();
            let off1 = self.off1.get();
            let off2 = self.off2.get();
            let off3 = self.off3.get();
            let rd0 = self.rd0.get();
            let rd1 = self.rd1.get();
            let rd2 = self.rd2.get();
            let rd3 = self.rd3.get();
            let el0 = self.el0.get();
            let el1 = self.el1.get();
            let el2 = self.el2.get();
            let el3 = self.el3.get();
            let sv0 = self.sv0.get();
            let sv1 = self.sv1.get();
            let sv2 = self.sv2.get();
            let sv3 = self.sv3.get();
            let mq = self.mq.get();

            // ------------------------------------------------ take
            let have = Bit::from(stats.peek().is_some());
            let ps = stats.head();
            let take = s_idle & have;
            let _ = stats.recv_if(take);
            let pk = ps.peer;
            let ps_ok = ps.valid
                & !Bit::from(ps.reach == 0)
                & !Bit::from(ps.stratum == 0)
                & Bit::from(ps.stratum < 16)
                & !Bit::from(ps.leap == 3);
            let tk0 = take & Bit::from(pk == 0);
            let tk1 = take & Bit::from(pk == 1);
            let tk2 = take & Bit::from(pk == 2);
            let tk3 = take & Bit::from(pk == 3);

            // ------------------------------------------------ root distance of peer ia
            let now_r = self.now_r.get();
            let t4i = pick64(
                ia,
                self.t40.get(),
                self.t41.get(),
                self.t42.get(),
                self.t43.get(),
            );
            let age0 = mux(now_r >= t4i, now_r - t4i, U::<64>::from(0u8));
            let agec = mux(age0 > AGE_MAX, U::<64>::from(AGE_MAX), age0);
            let age16 = (agec >> 16u32).slice::<0, 34>();
            let rdl = pick32(
                ia,
                self.rdl0.get(),
                self.rdl1.get(),
                self.rdl2.get(),
                self.rdl3.get(),
            );
            let rds = pick32(
                ia,
                self.rds0.get(),
                self.rds1.get(),
                self.rds2.get(),
                self.rds3.get(),
            );
            let dly = pick64(
                ia,
                self.dly0.get(),
                self.dly1.get(),
                self.dly2.get(),
                self.dly3.get(),
            );
            let dsp = pick64(
                ia,
                self.dsp0.get(),
                self.dsp1.get(),
                self.dsp2.get(),
                self.dsp3.get(),
            );
            let jit = pick64(
                ia,
                self.jit0.get(),
                self.jit1.get(),
                self.jit2.get(),
                self.jit3.get(),
            );
            let oki = pickb(
                ia,
                self.ok0.get(),
                self.ok1.get(),
                self.ok2.get(),
                self.ok3.get(),
            );
            let dsum = (rdl.zext::<67>() << 16u32) + dly.zext::<67>();
            let half = mux(dsum < MINDISP_Q32, U::<67>::from(MINDISP_Q32), dsum) >> 1u32;
            let phi = mq >> 16u32;
            // Everything but `phi` is summed into the register `rdx` (task
            // T5.3; `ia` is constant over the 4 phases of a peer, `rdx` is
            // read in phase 3), as a tree; `phi` is added in phase 3. The
            // same integer as the chain of additions (no wrap in 67 bits).
            let rdx_n = half
                + (((rds.zext::<67>() << 16u32) + dsp.zext::<67>())
                    + (jit.zext::<67>() + U::<67>::from(RHO_Q32)));
            let rd67 = self.rdx.get() + phi.zext::<67>();
            let rd_sat = mux(
                rd67 > 0x1_ffff_ffffu64,
                U::<33>::from(0x1_ffff_ffffu64),
                rd67.slice::<0, 33>(),
            );
            let el_new = oki & Bit::from(rd67 < MAXDIST_Q32);
            let dist_w = s_dist & ph3;
            let d0 = dist_w & Bit::from(ia == 0);
            let d1 = dist_w & Bit::from(ia == 1);
            let d2 = dist_w & Bit::from(ia == 2);
            let d3 = dist_w & Bit::from(ia == 3);
            let dist_end = dist_w & Bit::from(ia == 3);

            // ------------------------------------------------ tuple comparator
            // Issue: the pair (ji, ki) during the intersection, the decision
            // pairs in FINA / FINB. Stage E registers the two edges, stage C
            // their comparison; the counts use stage C.
            let ki = self.ki.get();
            let ji = self.ji.get();
            let lk0 = self.lk0.get();
            let lk1 = self.lk1.get();
            let hk0 = self.hk0.get();
            let hk1 = self.hk1.get();
            let idone = self.idone.get();
            let issue_i = s_inter & !idone;
            let ie = issue_i | s_fina | s_finb;
            let je_i = mux(s_fina, lk0, mux(s_finb, lk1, ji));
            let ke_i = mux(s_fina, hk0, mux(s_finb, hk1, ki));
            let ijp = je_i.slice::<0, 2>();
            let ikp = ke_i.slice::<0, 2>();
            let ijt = je_i.slice::<2, 2>();
            let ikt = ke_i.slice::<2, 2>();
            let jth = pick64(ijp, off0, off1, off2, off3).sext::<66>();
            let kth = pick64(ikp, off0, off1, off2, off3).sext::<66>();
            let jrd = pick33(ijp, rd0, rd1, rd2, rd3).zext::<66>();
            let krd = pick33(ikp, rd0, rd1, rd2, rd3).zext::<66>();
            let ej = mux(ijt == 0, jth - jrd, mux(ijt == 2, jth + jrd, jth));
            let ek = mux(ikt == 0, kth - krd, mux(ikt == 2, kth + krd, kth));
            let i_last = Bit::from(ji == 11);
            // Stage C, from the registered edges.
            let e_ej = self.e_ej.get();
            let e_ek = self.e_ek.get();
            let e_j = self.e_j.get();
            let e_k = self.e_k.get();
            let n_lt = e_ej.lt_signed(e_ek);
            let n_klt = n_lt | (Bit::from(e_ej == e_ek) & Bit::from(e_j < e_k));
            let n_keq = Bit::from(e_j == e_k);
            // The pair whose comparison is used in this cycle.
            let c_v = self.c_v.get();
            let je = self.c_j.get();
            let ke = self.c_k.get();
            let jp = je.slice::<0, 2>();
            let kp = ke.slice::<0, 2>();
            let jt = je.slice::<2, 2>();
            let kt = ke.slice::<2, 2>();
            let elt = self.c_lt.get();
            let klt = self.c_klt.get();
            let keq = self.c_keq.get();

            // ------------------------------------------------ intersection counts
            let el_j = pickb(jp, el0, el1, el2, el3);
            let el_k = pickb(kp, el0, el1, el2, el3);
            let m = el0.zext::<3>() + el1.zext::<3>() + el2.zext::<3>() + el3.zext::<3>();
            let thr0 = m;
            let thr1 = m - 1;
            let jlo = el_j & Bit::from(jt == 0);
            let jmid = el_j & Bit::from(jt == 1);
            let jhi = el_j & Bit::from(jt == 2);
            let le = klt | keq;
            let ge = !klt;
            let gt = !klt & !keq;
            let n_rk = self.rk.get() + (el_j & klt).zext::<4>();
            let n_lole = self.a_lole.get() + (jlo & le).zext::<3>();
            let n_hile = self.a_hile.get() + (jhi & le).zext::<3>();
            let n_hige = self.a_hige.get() + (jhi & ge).zext::<3>();
            let n_loge = self.a_loge.get() + (jlo & ge).zext::<3>();
            let n_mlt = self.a_mlt.get() + (jmid & klt).zext::<3>();
            let n_mgt = self.a_mgt.get() + (jmid & gt).zext::<3>();
            let c_lo = n_lole - n_hile;
            let c_hi = n_hige - n_loge;
            let jlast = Bit::from(je == 11);
            let acc_v = s_inter & c_v;
            let kend = acc_v & jlast;
            let inter_end = kend & Bit::from(ke == 11);
            let k_lo = kend & el_k & Bit::from(kt == 0);
            let k_mid = kend & el_k & Bit::from(kt == 1);
            let k_hi = kend & el_k & Bit::from(kt == 2);
            let lv0 = self.lv0.get();
            let lv1 = self.lv1.get();
            let hv0 = self.hv0.get();
            let hv1 = self.hv1.get();
            let lr0 = self.lr0.get();
            let lr1 = self.lr1.get();
            let hr0 = self.hr0.get();
            let hr1 = self.hr1.get();
            let ul0 = k_lo & Bit::from(c_lo >= thr0) & (!lv0 | Bit::from(n_rk < lr0));
            let ul1 = k_lo & Bit::from(c_lo >= thr1) & (!lv1 | Bit::from(n_rk < lr1));
            let uh0 = k_hi & Bit::from(c_hi >= thr0) & (!hv0 | Bit::from(n_rk > hr0));
            let uh1 = k_hi & Bit::from(c_hi >= thr1) & (!hv1 | Bit::from(n_rk > hr1));
            let um0 = k_mid & Bit::from(kp == 0);
            let um1 = k_mid & Bit::from(kp == 1);
            let um2 = k_mid & Bit::from(kp == 2);
            let um3 = k_mid & Bit::from(kp == 3);

            // ------------------------------------------------ decision
            let f0 = self.lf0.get().zext::<4>() + self.hf0.get().zext::<4>();
            let f1 = self.lf1.get().zext::<4>() + self.hf1.get().zext::<4>();
            let ok_a0 = lv0 & hv0 & Bit::from(f0 == 0) & elt;
            let s0 = self.s0.get();
            let ok_a1 = Bit::from(m >= 3) & lv1 & hv1 & Bit::from(f1 <= 1) & elt;
            let success = s0 | ok_a1;
            let rlo = mux(s0, lr0, lr1);
            let rhi = mux(s0, hr0, hr1);
            let mr0 = self.mr0.get();
            let mr1 = self.mr1.get();
            let mr2 = self.mr2.get();
            let mr3 = self.mr3.get();
            let nsv0 = el0 & Bit::from(mr0 > rlo) & Bit::from(mr0 < rhi);
            let nsv1 = el1 & Bit::from(mr1 > rlo) & Bit::from(mr1 < rhi);
            let nsv2 = el2 & Bit::from(mr2 > rlo) & Bit::from(mr2 < rhi);
            let nsv3 = el3 & Bit::from(mr3 > rlo) & Bit::from(mr3 < rhi);
            let all4 = nsv0 & nsv1 & nsv2 & nsv3;
            // Start-up majority (F4): truechimer counts for both allows,
            // registered (g0, g1) and used in FIN1 until the first valid
            // selection (`ever`).
            let maj = (n_en.get() >> 1u32) + U::<3>::from(1u8);
            let t00 = el0 & Bit::from(mr0 > lr0) & Bit::from(mr0 < hr0);
            let t01 = el1 & Bit::from(mr1 > lr0) & Bit::from(mr1 < hr0);
            let t02 = el2 & Bit::from(mr2 > lr0) & Bit::from(mr2 < hr0);
            let t03 = el3 & Bit::from(mr3 > lr0) & Bit::from(mr3 < hr0);
            let t10 = el0 & Bit::from(mr0 > lr1) & Bit::from(mr0 < hr1);
            let t11 = el1 & Bit::from(mr1 > lr1) & Bit::from(mr1 < hr1);
            let t12 = el2 & Bit::from(mr2 > lr1) & Bit::from(mr2 < hr1);
            let t13 = el3 & Bit::from(mr3 > lr1) & Bit::from(mr3 < hr1);
            let n_t0 = (t00.zext::<3>() + t01.zext::<3>()) + (t02.zext::<3>() + t03.zext::<3>());
            let n_t1 = (t10.zext::<3>() + t11.zext::<3>()) + (t12.zext::<3>() + t13.zext::<3>());
            let gate = self.ever.get() | mux(s0, self.g0.get(), self.g1.get());
            let fin_fail = s_fin1 & !(success & gate);
            let fin_ok = s_fin1 & success & gate;

            // ------------------------------------------------ clustering
            let cp = self.cp.get();
            let cq = self.cq.get();
            let thp = pick64(cp, off0, off1, off2, off3);
            let thq = pick64(cq, off0, off1, off2, off3);
            let dpq = absdiff34(thp, thq);
            let clu_w = s_clu & ph3;
            let acc_n = self.acc.get() + mq.zext::<70>();
            let p_end = clu_w & Bit::from(cq == 3);
            let maxs = self.maxs.get();
            let new_max = p_end & Bit::from(acc_n > maxs);
            let jitp = pick64(
                cp,
                self.jit0.get(),
                self.jit1.get(),
                self.jit2.get(),
                self.jit3.get(),
            );
            let jitp34 = mux(
                jitp > 0x3_ffff_ffffu64,
                U::<34>::from(0x3_ffff_ffffu64),
                jitp.slice::<0, 34>(),
            );
            let jmin = self.jmin.get();
            let new_jmin = p_end & Bit::from(jitp34 < jmin);
            let clu_end = p_end & Bit::from(cp == 3);
            let thr3 = (mq.zext::<70>() << 1u32) + mq.zext::<70>();
            let drop = s_clu2 & ph3 & !Bit::from(maxs < thr3);
            let qmax = self.qmax.get();
            let dr0 = drop & Bit::from(qmax == 0);
            let dr1 = drop & Bit::from(qmax == 1);
            let dr2 = drop & Bit::from(qmax == 2);
            let dr3 = drop & Bit::from(qmax == 3);
            let clu2_end = s_clu2 & ph3;

            // ------------------------------------------------ system peer
            let str_i = pick8(
                ia,
                self.str0.get(),
                self.str1.get(),
                self.str2.get(),
                self.str3.get(),
            );
            let rd_i = pick33(ia, rd0, rd1, rd2, rd3);
            let sv_i = pickb(ia, sv0, sv1, sv2, sv3);
            let bv = self.bv.get();
            let bs = self.bs.get();
            let brd = self.brd.get();
            let better = s_sys
                & sv_i
                & (!bv | Bit::from(str_i < bs) | (Bit::from(str_i == bs) & Bit::from(rd_i < brd)));
            let sys_end = s_sys & Bit::from(ia == 3);
            let o = self.prev_p.get();
            let str_o = pick8(
                o,
                self.str0.get(),
                self.str1.get(),
                self.str2.get(),
                self.str3.get(),
            );
            let rd_o = pick33(o, rd0, rd1, rd2, rd3);
            let sv_o = pickb(o, sv0, sv1, sv2, sv3);
            let brd3 = (brd.zext::<35>() << 1u32) + brd.zext::<35>();
            let keep = self.prev_v.get()
                & sv_o
                & Bit::from(str_o == bs)
                & Bit::from(rd_o.zext::<35>() <= brd3);
            let sp_new = mux(keep, o, self.bi.get());

            // ------------------------------------------------ combine
            let sp = self.sp.get();
            let th_s = pick64(sp, off0, off1, off2, off3);
            let th_i = pick64(ia, off0, off1, off2, off3);
            let dneg = (th_i - th_s).bit(63);
            let dmag = absdiff34(th_i, th_s);
            let ms = self.ms.get();
            let w = self.w.get();
            let sq = self.sq.get();
            let cmb_skip = s_cmbi & !sv_i;
            let cmb_go = s_cmbi & sv_i;
            let cmb_w = s_cmbm & ph3;
            let cmb_sq = cmb_w & Bit::from(ms == 0);
            let cmb_n = cmb_w & Bit::from(ms == 1);
            let cmb_j1 = cmb_w & Bit::from(ms == 2);
            let cmb_j2 = cmb_w & Bit::from(ms == 3);
            let cmb_j = cmb_j1 | cmb_j2;
            let nn = self.nn.get();
            let jj = self.jj.get();
            let dd = self.dd.get();
            let mq64 = mq.zext::<64>();
            let nn_n = mux(dneg, nn - mq64, nn + mq64);
            let jj_n = mux(
                cmb_j2,
                jj + (mq.zext::<95>() << 34u32),
                jj + mq.zext::<95>(),
            );
            let next_i = cmb_skip | cmb_j2;
            let cmb_end = next_i & Bit::from(ia == 3);
            let jit_s = pick64(
                sp,
                self.jit0.get(),
                self.jit1.get(),
                self.jit2.get(),
                self.jit3.get(),
            );
            let jit_s34 = jit_s.slice::<0, 34>();

            // ------------------------------------------------ multiplier operands
            let wz = w.zext::<34>();
            let opa = mux(
                s_dist,
                age16,
                mux(
                    s_clu,
                    dpq,
                    mux(s_clu2, jmin, mux(s_fphi, jit_s34, mux(ms == 0, dmag, wz))),
                ),
            );
            let opb = mux(
                s_dist,
                U::<34>::from(PHI_Q32),
                mux(
                    s_clu,
                    dpq,
                    mux(
                        s_clu2,
                        jmin,
                        mux(
                            s_fphi,
                            jit_s34,
                            mux(
                                ms == 0,
                                dmag,
                                mux(
                                    ms == 1,
                                    dmag,
                                    mux(ms == 2, sq.slice::<0, 34>(), sq.slice::<34, 34>()),
                                ),
                            ),
                        ),
                    ),
                ),
            );
            let prod = self
                .ma
                .get()
                .zext::<68>()
                .mul::<68>(self.mb.get().zext::<68>());
            let mul_st = s_dist | s_clu | s_clu2 | s_cmbm | s_fphi;

            // ------------------------------------------------ divider and square root
            let dr = self.dr.get();
            let dq = self.dq.get();
            let dx = self.dx.get();
            let dc = self.dc.get();
            let dv = mux(s_divw, rd_i.zext::<34>(), dd.zext::<34>());
            let t35 = (dr.zext::<35>() << 1u32) | dx.bit(95).zext::<35>();
            let dge = Bit::from(t35 >= dv.zext::<35>());
            let dr_n = mux(dge, t35 - dv.zext::<35>(), t35).slice::<0, 34>();
            let dq_n = (dq << 1u32) | dge.zext::<68>();
            let dlast = Bit::from(dc == 1);
            let in_div = s_divw | s_divn | s_divj;
            let divw_end = s_divw & dlast;
            let divn_end = s_divn & dlast;
            let divj_end = s_divj & dlast;
            let sr = self.sr.get();
            let rem2 = (sr << 2u32) | dx.slice::<94, 2>().zext::<39>();
            let trial = (dq.zext::<39>() << 2u32) | U::<39>::from(1u8);
            let sge = Bit::from(rem2 >= trial);
            let sr_n = mux(sge, rem2 - trial, rem2);
            let root_n = (dq << 1u32) | sge.zext::<68>();
            let sqrt_end = s_sqrt & dlast;
            // Divider inits.
            let nneg = nn.bit(63);
            let nabs = mux(nneg, U::<64>::from(0u8) - nn, nn);
            let c64 = dq_n.zext::<64>();
            let off_res = th_s + mux(nneg, U::<64>::from(0u8) - c64, c64);
            let var = dq_n.zext::<70>() + sq.zext::<70>();
            let fphi_end = s_fphi & ph3;

            // ------------------------------------------------ output
            let ok = self.ok.get();
            let out_go = s_out & selected.ready();
            if out_go.to_bool() {
                selected.send(Selected {
                    valid: ok,
                    peer: mux(ok, sp, U::<2>::from(0u8)),
                    offset: mux(ok, self.res_off.get(), U::<64>::from(0u8)),
                    jitter: mux(ok, self.res_jit.get().zext::<64>(), U::<64>::from(0u8)),
                    root_dist: mux(
                        ok,
                        pick33(sp, rd0, rd1, rd2, rd3).zext::<64>(),
                        U::<64>::from(0u8),
                    ),
                    stratum: mux(
                        ok,
                        pick8(
                            sp,
                            self.str0.get(),
                            self.str1.get(),
                            self.str2.get(),
                            self.str3.get(),
                        ),
                        U::<8>::from(0u8),
                    ),
                    leap: mux(
                        ok,
                        pick2(
                            sp,
                            self.lp0.get(),
                            self.lp1.get(),
                            self.lp2.get(),
                            self.lp3.get(),
                        ),
                        U::<2>::from(0u8),
                    ),
                    ref_id: mux(
                        ok,
                        pick32(
                            sp,
                            self.rid0.get(),
                            self.rid1.get(),
                            self.rid2.get(),
                            self.rid3.get(),
                        ),
                        U::<32>::from(0u8),
                    ),
                    root_delay: mux(
                        ok,
                        pick32(
                            sp,
                            self.rdl0.get(),
                            self.rdl1.get(),
                            self.rdl2.get(),
                            self.rdl3.get(),
                        ),
                        U::<32>::from(0u8),
                    ),
                    root_disp: mux(
                        ok,
                        pick32(
                            sp,
                            self.rds0.get(),
                            self.rds1.get(),
                            self.rds2.get(),
                            self.rds3.get(),
                        ),
                        U::<32>::from(0u8),
                    ),
                    delay: mux(
                        ok,
                        pick64(
                            sp,
                            self.dly0.get(),
                            self.dly1.get(),
                            self.dly2.get(),
                            self.dly3.get(),
                        ),
                        U::<64>::from(0u8),
                    ),
                    disp: mux(
                        ok,
                        pick64(
                            sp,
                            self.dsp0.get(),
                            self.dsp1.get(),
                            self.dsp2.get(),
                            self.dsp3.get(),
                        ),
                        U::<64>::from(0u8),
                    ),
                    t4: mux(
                        ok,
                        pick64(
                            sp,
                            self.t40.get(),
                            self.t41.get(),
                            self.t42.get(),
                            self.t43.get(),
                        ),
                        U::<64>::from(0u8),
                    ),
                });
            }

            // ------------------------------------------------ formal properties
            // (orchestrator; "Facts for the formal properties" above). `ok_i` is set with
            // the PeerStat; `el_i` and `sv_i` are recomputed per event.
            let okp0 = self.ok0.get();
            let okp1 = self.ok1.get();
            let okp2 = self.ok2.get();
            let okp3 = self.ok3.get();
            let stp0 = self.str0.get();
            let stp1 = self.str1.get();
            let stp2 = self.str2.get();
            let stp3 = self.str3.get();
            let lpp0 = self.lp0.get();
            let lpp1 = self.lp1.get();
            let lpp2 = self.lp2.get();
            let lpp3 = self.lp3.get();
            let elp0 = el0;
            let svp0 = sv0;
            let rdp0 = rd0;
            let elp1 = el1;
            let svp1 = sv1;
            let rdp1 = rd1;
            let elp2 = el2;
            let svp2 = sv2;
            let rdp2 = rd2;
            let elp3 = el3;
            let svp3 = sv3;
            let rdp3 = rd3;
            let post = s_clu | s_clu2 | s_sys | s_sys2 | s_cmbi | s_divw | s_cmbm | s_fphi | s_divn | s_divj | s_sqrt | s_out;
            check!(!okp0 | (Bit::from(stp0 >= 1) & Bit::from(stp0 <= 15) & !Bit::from(lpp0 == 3)), "a usable peer 0 has stratum 1..=15 and leap != 3");
            check!(!okp1 | (Bit::from(stp1 >= 1) & Bit::from(stp1 <= 15) & !Bit::from(lpp1 == 3)), "a usable peer 1 has stratum 1..=15 and leap != 3");
            check!(!okp2 | (Bit::from(stp2 >= 1) & Bit::from(stp2 <= 15) & !Bit::from(lpp2 == 3)), "a usable peer 2 has stratum 1..=15 and leap != 3");
            check!(!okp3 | (Bit::from(stp3 >= 1) & Bit::from(stp3 <= 15) & !Bit::from(lpp3 == 3)), "a usable peer 3 has stratum 1..=15 and leap != 3");
            check!(!(elp0 & !s_dist) | (okp0 & Bit::from(rdp0 < MAXDIST_Q32)), "an eligible peer 0 is usable with root distance < MAXDIST");
            check!(!(elp1 & !s_dist) | (okp1 & Bit::from(rdp1 < MAXDIST_Q32)), "an eligible peer 1 is usable with root distance < MAXDIST");
            check!(!(elp2 & !s_dist) | (okp2 & Bit::from(rdp2 < MAXDIST_Q32)), "an eligible peer 2 is usable with root distance < MAXDIST");
            check!(!(elp3 & !s_dist) | (okp3 & Bit::from(rdp3 < MAXDIST_Q32)), "an eligible peer 3 is usable with root distance < MAXDIST");
            check!(!(post & !s_out) | self.ok.get(), "after a successful decision the selection is valid until OUT");
            check!(!(post & svp0) | elp0, "a survivor 0 is eligible");
            check!(!(post & svp1) | elp1, "a survivor 1 is eligible");
            check!(!(post & svp2) | elp2, "a survivor 2 is eligible");
            check!(!(post & svp3) | elp3, "a survivor 3 is eligible");
            check!(!self.ok.get() | self.ever.get(), "a valid selection implies ever");
            let post2 = s_cmbi | s_divw | s_cmbm | s_fphi | s_divn | s_divj | s_sqrt | s_out;
            check!(!(post2 & self.ok.get() & self.m_ne.get()) | pickb(sp, svp0, svp1, svp2, svp3), "the system peer is a survivor (given that the intersection left one)");
            check!(!(post2 & self.ok.get() & self.m_ne.get()) | pickb(sp, elp0, elp1, elp2, elp3), "the system peer is eligible (given that the intersection left a survivor)");
            check!(!(out_go & ok & self.m_ne.get()) | Bit::from(pick33(sp, rd0, rd1, rd2, rd3) < MAXDIST_Q32), "valid: root_dist < MAXDIST (given a survivor)");
            cover!(out_go & ok, "a valid selection is output");
            cover!(out_go & !ok, "no selection is output");

            with!(self <= {
                // Multiplier pipeline, every cycle.
                rdx: rdx_n,
                g0: Bit::from(n_t0 >= maj),
                g1: Bit::from(n_t1 >= maj),
                ma: opa,
                mb: opb,
                mp: prod,
                mq: self.mp.get(),
                mul_st ? ph: ph + 1,
                // Take.
                take ? { st: U::<5>::from(DIST), now_r: now.get(), ia: U::<2>::from(0u8), ph: U::<2>::from(0u8) },
                tk0 ? { ok0: ps_ok, off0: ps.offset, dly0: ps.delay, dsp0: ps.disp, jit0: ps.jitter, t40: ps.t4, str0: ps.stratum, lp0: ps.leap, rid0: ps.ref_id, rdl0: ps.root_delay, rds0: ps.root_disp },
                tk1 ? { ok1: ps_ok, off1: ps.offset, dly1: ps.delay, dsp1: ps.disp, jit1: ps.jitter, t41: ps.t4, str1: ps.stratum, lp1: ps.leap, rid1: ps.ref_id, rdl1: ps.root_delay, rds1: ps.root_disp },
                tk2 ? { ok2: ps_ok, off2: ps.offset, dly2: ps.delay, dsp2: ps.disp, jit2: ps.jitter, t42: ps.t4, str2: ps.stratum, lp2: ps.leap, rid2: ps.ref_id, rdl2: ps.root_delay, rds2: ps.root_disp },
                tk3 ? { ok3: ps_ok, off3: ps.offset, dly3: ps.delay, dsp3: ps.disp, jit3: ps.jitter, t43: ps.t4, str3: ps.stratum, lp3: ps.leap, rid3: ps.ref_id, rdl3: ps.root_delay, rds3: ps.root_disp },
                // Distances.
                d0 ? { rd0: rd_sat, el0: el_new },
                d1 ? { rd1: rd_sat, el1: el_new },
                d2 ? { rd2: rd_sat, el2: el_new },
                d3 ? { rd3: rd_sat, el3: el_new },
                dist_w ? ia: ia + 1,
                dist_end ? {
                    st: U::<5>::from(INTER),
                    ki: U::<4>::from(0u8),
                    ji: U::<4>::from(0u8),
                    rk: U::<4>::from(0u8),
                    a_lole: U::<3>::from(0u8),
                    a_hile: U::<3>::from(0u8),
                    a_hige: U::<3>::from(0u8),
                    a_loge: U::<3>::from(0u8),
                    a_mlt: U::<3>::from(0u8),
                    a_mgt: U::<3>::from(0u8),
                    idone: Bit::Zero,
                    lv0: Bit::Zero,
                    lv1: Bit::Zero,
                    hv0: Bit::Zero,
                    hv1: Bit::Zero,
                },
                // Comparator pipeline, every cycle.
                e_v: ie,
                e_j: je_i,
                e_k: ke_i,
                e_ej: ej,
                e_ek: ek,
                c_v: self.e_v.get(),
                c_j: e_j,
                c_k: e_k,
                c_lt: n_lt,
                c_klt: n_klt,
                c_keq: n_keq,
                // Intersection: one pair issued per cycle.
                issue_i ? ji: ji + 1,
                issue_i & i_last ? {
                    ji: U::<4>::from(0u8),
                    ki: ki + 1,
                },
                issue_i & i_last & Bit::from(ki == 11) ? idone: Bit::One,
                // Counts, from the pair compared two cycles after its issue.
                acc_v ? {
                    rk: n_rk,
                    a_lole: n_lole,
                    a_hile: n_hile,
                    a_hige: n_hige,
                    a_loge: n_loge,
                    a_mlt: n_mlt,
                    a_mgt: n_mgt,
                },
                kend ? {
                    rk: U::<4>::from(0u8),
                    a_lole: U::<3>::from(0u8),
                    a_hile: U::<3>::from(0u8),
                    a_hige: U::<3>::from(0u8),
                    a_loge: U::<3>::from(0u8),
                    a_mlt: U::<3>::from(0u8),
                    a_mgt: U::<3>::from(0u8),
                },
                ul0 ? { lv0: Bit::One, lk0: ke, lr0: n_rk, lf0: n_mlt },
                ul1 ? { lv1: Bit::One, lk1: ke, lr1: n_rk, lf1: n_mlt },
                uh0 ? { hv0: Bit::One, hk0: ke, hr0: n_rk, hf0: n_mgt },
                uh1 ? { hv1: Bit::One, hk1: ke, hr1: n_rk, hf1: n_mgt },
                um0 ? mr0: n_rk,
                um1 ? mr1: n_rk,
                um2 ? mr2: n_rk,
                um3 ? mr3: n_rk,
                inter_end ? st: U::<5>::from(FINA),
                // Decision: the pairs (lk0, hk0) and (lk1, hk1) are issued in
                // FINA and FINB; their comparisons are read in FIN0 and FIN1.
                s_fina ? st: U::<5>::from(FINB),
                s_finb ? st: U::<5>::from(FIN0),
                // Decision.
                s_fin0 ? { s0: ok_a0, st: U::<5>::from(FIN1) },
                fin_fail ? { ok: Bit::Zero, prev_v: Bit::Zero, st: U::<5>::from(OUT) },
                fin_ok ? {
                    ever: Bit::One,
                    ok: Bit::One,
                    m_ne: nsv0 | nsv1 | nsv2 | nsv3,
                    sv0: nsv0,
                    sv1: nsv1,
                    sv2: nsv2,
                    sv3: nsv3,
                    st: mux(all4, U::<5>::from(CLU), U::<5>::from(SYS)),
                    cp: U::<2>::from(0u8),
                    cq: U::<2>::from(0u8),
                    ph: U::<2>::from(0u8),
                    acc: U::<70>::from(0u8),
                    maxs: U::<70>::from(0u8),
                    qmax: U::<2>::from(0u8),
                    jmin: U::<34>::from(0x3_ffff_ffffu64),
                    ia: U::<2>::from(0u8),
                    bv: Bit::Zero,
                },
                // Clustering.
                clu_w ? { acc: acc_n, cq: cq + 1 },
                p_end ? { acc: U::<70>::from(0u8), cp: cp + 1 },
                new_max ? { maxs: acc_n, qmax: cp },
                new_jmin ? jmin: jitp34,
                clu_end ? st: U::<5>::from(CLU2),
                dr0 ? sv0: Bit::Zero,
                dr1 ? sv1: Bit::Zero,
                dr2 ? sv2: Bit::Zero,
                dr3 ? sv3: Bit::Zero,
                clu2_end ? st: U::<5>::from(SYS),
                // System peer.
                better ? { bv: Bit::One, bi: ia, bs: str_i, brd: rd_i },
                s_sys ? ia: ia + 1,
                sys_end ? st: U::<5>::from(SYS2),
                s_sys2 ? {
                    sp: sp_new,
                    prev_v: Bit::One,
                    prev_p: sp_new,
                    st: U::<5>::from(CMBI),
                    ia: U::<2>::from(0u8),
                    nn: U::<64>::from(0u8),
                    dd: U::<27>::from(0u8),
                    jj: U::<95>::from(0u8),
                },
                // Combine.
                cmb_go ? {
                    st: U::<5>::from(DIVW),
                    dr: U::<34>::from(0x100_0000u64),
                    dx: U::<96>::from(0u8),
                    dq: U::<68>::from(0u8),
                    dc: U::<7>::from(24u8),
                },
                in_div ? { dr: dr_n, dq: dq_n, dx: dx << 1u32, dc: dc - 1 },
                divw_end ? { w: dq_n.slice::<0, 24>(), st: U::<5>::from(CMBM), ms: U::<2>::from(0u8), ph: U::<2>::from(0u8) },
                cmb_sq ? sq: mq,
                cmb_n ? nn: nn_n,
                cmb_j ? jj: jj_n,
                cmb_w ? ms: ms + 1,
                cmb_j2 ? { dd: dd + w.zext::<27>(), st: U::<5>::from(CMBI) },
                next_i ? ia: ia + 1,
                cmb_end ? { st: U::<5>::from(FPHI), ph: U::<2>::from(0u8) },
                fphi_end ? {
                    sq: mq,
                    st: U::<5>::from(DIVN),
                    dr: (nabs >> 34u32).slice::<0, 34>(),
                    dx: nabs.zext::<96>() << 62u32,
                    dq: U::<68>::from(0u8),
                    dc: U::<7>::from(34u8),
                },
                divn_end ? {
                    res_off: off_res,
                    st: U::<5>::from(DIVJ),
                    dr: (jj >> 68u32).slice::<0, 34>(),
                    dx: jj.zext::<96>() << 28u32,
                    dq: U::<68>::from(0u8),
                    dc: U::<7>::from(68u8),
                },
                divj_end ? {
                    st: U::<5>::from(SQRT),
                    dx: var.zext::<96>() << 26u32,
                    sr: U::<39>::from(0u8),
                    dq: U::<68>::from(0u8),
                    dc: U::<7>::from(35u8),
                },
                s_sqrt ? { sr: sr_n, dq: root_n, dx: dx << 2u32, dc: dc - 1 },
                sqrt_end ? { res_jit: root_n.slice::<0, 35>(), st: U::<5>::from(OUT) },
                // Output.
                out_go ? st: U::<5>::from(IDLE),
            });
        }
    }
}
