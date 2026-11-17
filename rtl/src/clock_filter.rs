// SPDX-License-Identifier: Apache-2.0
//! RFC 5905 clock filter (task T4.3), domain `DefaultClock`: 4 peers x 8
//! stages, sequential (area over speed), bit-exact with the Python reference
//! `tools/golden/ntpclient/filter.py` (the executable form of this text) and
//! replayed against `tools/golden/ntpclient/vectors/filter.txt` by
//! `rtl/tests/clock_filter.rs`. Interface types: `ntp_types.rs` (`Sample` in,
//! `PeerStat` out; 32.32 times, offsets signed two's complement in `U<64>`).
//!
//! # Ports
//!
//! * `samples: Rx<Sample>`: one reply of a peer (`peer` = index 0..=3).
//! * `miss: Rx<U<2>>`: the poll of that peer got no answer.
//! * `stats: Tx<PeerStat>`: exactly one transaction per event taken, for the
//!   peer of that event, in the order the events were taken.
//!
//! No other port: no clock input (the ageing reference is the t4 of the
//! samples) and no flush (a clock step by the servo is not signalled; the
//! consumers compare `PeerStat.t4` with their clock).
//!
//! # Port timing
//!
//! The unit is a sequencer (`st`, 14 states). In `S_IDLE` it takes an event
//! (`recv_if`) in the first cycle the event is visible at the head of its
//! channel: `samples` first when both channels hold one (a miss waits), else
//! `miss`. The storage updates of the event (below, "Intake") land at the
//! end of that cycle. The event is processed without interruption; events
//! arriving meanwhile wait on their channels (the unit never takes one outside
//! `S_IDLE`). At the end the unit stays in `S_OUT` until `stats.ready()`, sends
//! the `PeerStat` in that cycle and returns to `S_IDLE` (a stalled consumer
//! stalls the unit, nothing is lost or reordered). A new event can be taken
//! the cycle after the `PeerStat` is sent.
//!
//! # Cycles per event
//!
//! From the cycle the event is visible in its channel to the cycle `stats`
//! carries the `PeerStat` (consumer always ready), at most
//! [`MAX_EVENT_CYCLES`] = 640 (the worst case measured through the channels is
//! 611, the vectors give a mean of 356 and a maximum of 600). Counting in
//! states: 1 (intake) + ageing pass (1 per age without a sample, 18 per age
//! with one: 1 + 16 multiplier steps + 1; plus 1 to finish: at most 146) +
//! ranks (rank 0: 1 + at most 8 scan + 1; each further rank with a candidate:
//! 1 + at most 8 + 1 + at most 37 multiplier steps + 1; each rank without
//! candidate: 1; plus 1 to finish: at most 10 + 7 x 48 + 1) + finish 1 +
//! (only when accepted with m >= 2) division 75 + 1 + square root 38, + accept
//! 1 + send 1 = 610, plus 2 for the channels. A `miss` on a peer without
//! samples takes about 22 cycles. At 100 MHz: below 6.4 us for events at most
//! every 16 s.
//!
//! # State per peer (storage)
//!
//! Distributed RAM / LUT registers, nothing is ever read in a different cycle
//! than it is needed (asynchronous reads):
//!
//! * the 8 stages as a ring of 8 slots `{offset, delay, disp, t4}` (4 memories
//!   32 x 64 bits, index `{peer, slot}`) and a head pointer `head` (3 bits).
//!   Age `j` (0 = newest) is at slot `(head + j) mod 8`. An event does
//!   `head = head - 1 (mod 8)`, so the new sample goes to age 0 and the oldest
//!   falls out; no data is moved;
//! * `reach` (8 bits), bit j = the poll of age j was answered; it is also the
//!   validity mask of the stages (bit j of `reach` = stage j holds a real
//!   sample), because a sample and a miss both shift the ring and `reach`;
//! * `tnow`: t4 of the last sample inserted (ageing reference, see below);
//! * `tlast` (t4 of the best sample of the last accepted update) and `hv`
//!   (an update was accepted: a result exists);
//! * metadata of the last sample inserted: stratum, leap, ref_id, root_delay,
//!   root_disp (`p_meta`, 106 bits);
//! * the stored result `{offset, delay, disp, jitter}` of the last accepted
//!   update (4 x 64 bits).
//!
//! # Behaviour (what `PeerStat` is, bit exact)
//!
//! **Intake.** `sample`: `reach = (reach << 1 | 1) & 0xFF`, `head -= 1`, the
//! fields offset/delay/disp/t4 are stored at age 0, `tnow = sample.t4`, the
//! metadata replaces the old one. `miss`: `reach = (reach << 1) & 0xFF`,
//! `head -= 1` (an empty stage is shifted in; RFC 5905: the poll process calls
//! `clock_filter(p, 0, 0, MAXDISP)`), `tnow`, metadata and result unchanged.
//! `Sample.t4` is stored as given: no check that it is newer than anything.
//!
//! **The run** (after every event, for the peer of the event, `now = tnow`):
//!
//! 1. Ageing. For each age j with `reach[j] = 1`: `d = (tnow - t4_j) mod 2^64`,
//!    set to 0 if `d >= 2^63` (a stage after `tnow`), `aged_j = disp_j +
//!    floor(PHI_Q32 * d / 2^32)` (PHI_Q32 = 64425, i.e. 15 ppm; `d` is a
//!    32.32 duration, the product is formed exactly in 80 bits, the shift
//!    floors, the sum is formed in 80 bits). The stored `disp_j` is never
//!    modified: the ageing is recomputed from the stored sample each time, it
//!    does not compound. The stage is *usable* iff `aged_j < MAXDISP`
//!    (MAXDISP = 16 s = 2^36; RFC: a sample with dispersion >= MAXDISP is
//!    invalid). Stages with `reach[j] = 0` are not usable.
//! 2. Sorting. The usable stages are ranked by increasing `delay` (unsigned
//!    64 bits): rank r is found by scanning the ages 0, 1, .., 7 and keeping
//!    the first stage that has a strictly smaller delay than the best so far,
//!    so **ties go to the newest stage** (smallest age). `m` = number of
//!    usable stages.
//! 3. Dispersion: `disp = sum_{r=0}^{7} term_r` with `term_r = aged_{rank r}
//!    >> (r + 1)` (logical shift, floor, per term) for `r < m` and `term_r =
//!    MAXDISP >> (r + 1)` for `r >= m`. So one sample alone gives
//!    `aged_0/2 + MAXDISP * (1/4 + .. + 1/256)` (about 8 s) and the
//!    dispersion only falls under 1 s after about 5 usable samples (this is
//!    the RFC rule: empty and expired entries count as MAXDISP). The sum is
//!    below 2^36 + 2^35 < 2^37 (every `aged < 2^36`, no overflow of 64 bits).
//! 4. Jitter. `m <= 1`: 0. Else, for every rank r in 1..m: the true
//!    difference of the two signed offsets `|offset_r - offset_0|` (computed
//!    without wrap: both values are signed 64-bit, the difference is below
//!    2^64) saturated at MAXDISP, squared exactly (< 2^72), summed
//!    (`jsum` < 7 x 2^72 < 2^75), then `jitter = isqrt(jsum // (m - 1))`:
//!    unsigned floor division by m-1 (restoring divider, 75 steps) then the
//!    floor integer square root (digit by digit, 38 steps; equal to
//!    `math.isqrt`). RFC 5905 §10: RMS of the differences to the best sample
//!    over the n-1 others. So `jitter <= MAXDISP` (a saturated difference
//!    reads as 16 s), the precision floor is not applied (selection does it).
//! 5. Acceptance (RFC 5905 A.5.2 `if (f[0].t - p->t <= 0) return`): the
//!    result is accepted iff `m >= 1` and (`hv = 0` or `t4_0` of the best
//!    sample is strictly newer than `tlast`): `newer(a, b) = a != b and
//!    ((a - b) mod 2^64) < 2^63` (era-modular compare of 32.32 times). If
//!    accepted: stored result = `(offset_0, delay_0, disp, jitter)`, `tlast =
//!    t4_0`, `hv = 1`. If not (no usable stage, or the best stage is the one
//!    already used or an older one): the stored result, `tlast` and `hv` are
//!    unchanged, all four fields together. A stage is thus used by at most one
//!    update and the reported `t4` never goes back (in the era-modular
//!    sense); a newer sample with a worse delay does not replace an older
//!    better one until the older one leaves the 8 stages or its aged
//!    dispersion reaches MAXDISP.
//!
//! **Output** (`PeerStat`, one per event, `peer` = peer of the event):
//! `offset, delay, disp, jitter, t4` = stored result and `tlast` (all 0 while
//! `hv = 0`); `reach` = the new `reach`; `stratum, leap, ref_id, root_delay,
//! root_disp` = metadata of the last sample *inserted* (all 0 before the first
//! one; a `miss` does not change them); `valid = hv and reach != 0 and
//! 1 <= stratum <= 15 and leap != 3`. A peer whose only samples were never
//! usable (e.g. `disp >= MAXDISP`) reports `valid = 0` and the zeros above,
//! with `reach` and the metadata set.
//!
//! # Ranges and assumptions
//!
//! * Registers: `rem`/`dnm` 5 bits, divider/radicand/root/product registers
//!   80 bits, everything else <= 64 bits (no register above 128 bits).
//! * The serial multiplier handles `a` (<= 2^64) x `b` (< 2^37) and the
//!   ageing `PHI_Q32` x `d`; it needs `bit_length(b)` cycles (b = PHI_Q32:
//!   exactly 16; b = a saturated difference: at most 37, at least 1).
//! * Offsets may be anywhere in the 64-bit two's complement range
//!   (about +-2^31 s); the comparison `offset_r >= offset_0` is signed (bias
//!   trick `x ^ 2^63`).
//! * `peer` is 2 bits: every index is a peer, there is no out-of-range case.
//! * After reset every memory is zero: `reach = 0`, `hv = 0`.
//!
//! # Invariants the formal properties can be written from
//!
//! * `st` is in 0..=13; `S_IDLE` iff no event is being processed; an event is
//!   taken only in `S_IDLE`, never `samples` and `miss` in the same cycle, a
//!   pending `samples` event is taken before a pending `miss` one;
//! * exactly one `stats` per event taken, with `peer` = the peer taken, and
//!   `stats` only in `S_OUT`; no memory of a peer other than `cp` changes
//!   after the intake cycle;
//! * `reach` shifts by one bit and its low bit is 1 iff the event was a
//!   sample; `valid = 1` implies `reach != 0`, `hv`, stratum 1..=15, leap != 3;
//! * counters stay in range: `jj <= 8`, `r <= 8`, `mcnt <= 8` and `mcnt` = the
//!   popcount of `pm` (ranked stages), `pm` is a subset of `um`; `it <= 74`;
//! * `jitter` of the stored result `<= MAXDISP`; `disp < 2^37`; `jitter = 0`
//!   when the update used `m = 1`;
//! * the accepted `offset`/`delay` are those of the minimum-delay usable stage
//!   (`delay_0 <= delay_r` for every usable stage r; equality only if the
//!   chosen one is the newer), and `t4` of an accepted result is strictly
//!   newer than the previous one (era-modular);
//! * a `miss` never changes offset/delay/disp/jitter/t4 unless the run
//!   accepts a result, which needs a stage newer than `tlast` (a miss adds no
//!   stage: it can only expire stages, so after a miss the result is
//!   unchanged except when ageing made the previous best leave the usable set
//!   and a newer stage becomes best).
//!
//! # Deviations from RFC 5905 (decided here, to confirm)
//!
//! * Dispersion of stored samples is aged on the fly (PHI x (tnow - t4_j),
//!   recomputed, floor) instead of being incremented in the register at each
//!   update; same value up to rounding, without compounding. On a `miss`
//!   there is no clock, so `tnow` is the t4 of the last sample inserted: the
//!   ageing between that sample and the miss is accounted for by the consumers
//!   from `PeerStat.t4` (RFC: the root distance adds PHI x (clock - t)).
//! * The jitter is over the usable stages only (m of them), dividing by m - 1;
//!   no precision floor; the popcorn spike suppressor (RFC SGATE) is not
//!   implemented; `disp`, `jitter`, `offset`, `delay` are updated together or
//!   not at all (ntpd refreshes dispersion and jitter on every update).
//! * `miss` shifts an empty stage and `reach` but does not touch the
//!   metadata; `reach == 0` or an old `t4` is how the consumers detect an
//!   unreachable peer (the RFC's `poll()` demobilisation is not here).
//!
//! # Formal
//!
//! The `check!`/`cover!` are written by the orchestrator (row "Filtre/sélection"
//! of `docs/formal.md`) from the invariants above.
use crate::ntp_types::{PeerStat, Sample, PHI_Q32};
use txhdl::comp::{mux, Clock, DefaultClock, Mem, Reg, Rx, Tx, Unit};
use txhdl::types::{Bit, U};
use txhdl::{lower, with, Trace};

/// Stages of the filter (RFC 5905 NSTAGE).
pub const NSTAGE: usize = 8;
/// RFC 5905 MAXDISP, 16 s in 32.32.
pub const MAXDISP: u64 = 16 << 32;
/// Upper bound of the cycles from an event being visible at the head of its
/// channel to the cycle `stats` is accepted (consumer always ready).
pub const MAX_EVENT_CYCLES: u64 = 640;

const S_IDLE: u8 = 0;
const S_AGE: u8 = 1;
const S_MUL: u8 = 2;
const S_AGEF: u8 = 3;
const S_RANK: u8 = 4;
const S_SCAN: u8 = 5;
const S_PICK: u8 = 6;
const S_JACC: u8 = 7;
const S_FIN: u8 = 8;
const S_DIV: u8 = 9;
const S_SQ0: u8 = 10;
const S_SQ: u8 = 11;
const S_ACC: u8 = 12;
const S_OUT: u8 = 13;

/// Memory index of the stage of age `j` of peer `p`: `{p, (head + j) mod 8}`.
#[lower]
fn sidx(p: U<2>, h: U<3>, j: U<3>) -> U<5> {
    (p.zext::<5>() << 3u32) | (h + j).zext::<5>()
}

/// `x >> (r + 1)` for `r` in 0..=7 (0 beyond).
#[lower]
fn shr_rank(x: U<64>, r: U<4>) -> U<64> {
    mux(
        r == 0,
        x >> 1u32,
        mux(
            r == 1,
            x >> 2u32,
            mux(
                r == 2,
                x >> 3u32,
                mux(
                    r == 3,
                    x >> 4u32,
                    mux(
                        r == 4,
                        x >> 5u32,
                        mux(
                            r == 5,
                            x >> 6u32,
                            mux(r == 6, x >> 7u32, mux(r == 7, x >> 8u32, U::<64>::from(0u8))),
                        ),
                    ),
                ),
            ),
        ),
    )
}

#[derive(Trace, Default)]
pub struct ClockFilter {
    /// State of the sequencer (`S_*`).
    pub st: Reg<U<4>>,
    /// Peer being processed.
    pub cp: Reg<U<2>>,
    /// Age counter (0..=8) of the ageing pass and of the scan.
    pub jj: Reg<U<4>>,
    /// Remaining reach bits (shifted right) during the ageing pass.
    pub rm: Reg<U<8>>,
    /// Usable stages, bit j = age j (built by shifting in at the top).
    pub um: Reg<U<8>>,
    /// Stages already ranked.
    pub pm: Reg<U<8>>,
    /// Rank being filled (0..=8).
    pub r: Reg<U<4>>,
    /// Candidates still to scan (shifted right), one-hot position, best one-hot.
    pub cm: Reg<U<8>>,
    pub oh: Reg<U<8>>,
    pub bm: Reg<U<8>>,
    /// Age of the best candidate found so far, its delay, found flag.
    pub bj: Reg<U<3>>,
    pub bdly: Reg<U<64>>,
    pub found: Reg<Bit>,
    /// Serial multiplier: multiplicand (shifted left), multiplier (shifted
    /// right), accumulator, and where to go on (0 = ageing, 1 = jitter).
    pub ma: Reg<U<80>>,
    pub mb: Reg<U<64>>,
    pub macc: Reg<U<80>>,
    pub mk: Reg<Bit>,
    /// Dispersion sum, sum of squared offset differences, ranks filled.
    pub dsum: Reg<U<64>>,
    pub jsum: Reg<U<80>>,
    pub mcnt: Reg<U<4>>,
    /// Best sample (rank 0): offset, delay, t4.
    pub off0: Reg<U<64>>,
    pub dly0: Reg<U<64>>,
    pub t40: Reg<U<64>>,
    /// Divider (dividend shifted left, quotient, remainder, divisor) and
    /// square root (radicand in `q`, root, bit) state, iteration counter.
    pub dv: Reg<U<80>>,
    pub q: Reg<U<80>>,
    pub rem: Reg<U<5>>,
    pub dnm: Reg<U<5>>,
    pub sres: Reg<U<80>>,
    pub sbit: Reg<U<80>>,
    pub it: Reg<U<7>>,
    // Per stage storage, index {peer, slot}.
    #[distributed]
    pub m_off: Mem<U<64>, 32, DefaultClock>,
    #[distributed]
    pub m_dly: Mem<U<64>, 32, DefaultClock>,
    #[distributed]
    pub m_dsp: Mem<U<64>, 32, DefaultClock>,
    #[distributed]
    pub m_t4: Mem<U<64>, 32, DefaultClock>,
    /// Aged dispersion of the stage of age j (scratch of the current run).
    #[distributed]
    pub m_eff: Mem<U<64>, 8, DefaultClock>,
    // Per peer state.
    #[distributed]
    pub p_head: Mem<U<3>, 4, DefaultClock>,
    #[distributed]
    pub p_reach: Mem<U<8>, 4, DefaultClock>,
    #[distributed]
    pub p_tnow: Mem<U<64>, 4, DefaultClock>,
    #[distributed]
    pub p_tlast: Mem<U<64>, 4, DefaultClock>,
    #[distributed]
    pub p_hv: Mem<U<1>, 4, DefaultClock>,
    /// `{stratum[105:98], leap[97:96], ref_id[95:64], root_delay[63:32], root_disp[31:0]}`.
    #[distributed]
    pub p_meta: Mem<U<106>, 4, DefaultClock>,
    #[distributed]
    pub p_roff: Mem<U<64>, 4, DefaultClock>,
    #[distributed]
    pub p_rdly: Mem<U<64>, 4, DefaultClock>,
    #[distributed]
    pub p_rdsp: Mem<U<64>, 4, DefaultClock>,
    #[distributed]
    pub p_rjit: Mem<U<64>, 4, DefaultClock>,
}

#[lower]
impl Unit<(Rx<Sample>, Rx<U<2>>), Tx<PeerStat>> for ClockFilter {
    async fn run(&mut self, (samples, miss): (Rx<Sample>, Rx<U<2>>), stats: Tx<PeerStat>) {
        loop {
            DefaultClock::rising().await;
            let st = self.st.get();
            let cp = self.cp.get();
            let jj = self.jj.get();
            let rm = self.rm.get();
            let um = self.um.get();
            let pm = self.pm.get();
            let r = self.r.get();
            let cm = self.cm.get();
            let oh = self.oh.get();
            let bm = self.bm.get();
            let bj = self.bj.get();
            let bdly = self.bdly.get();
            let found = self.found.get();
            let ma = self.ma.get();
            let mb = self.mb.get();
            let macc = self.macc.get();
            let mk = self.mk.get();
            let dsum = self.dsum.get();
            let jsum = self.jsum.get();
            let mcnt = self.mcnt.get();
            let off0 = self.off0.get();
            let dly0 = self.dly0.get();
            let t40 = self.t40.get();
            let dv = self.dv.get();
            let q = self.q.get();
            let rem = self.rem.get();
            let dnm = self.dnm.get();
            let sres = self.sres.get();
            let sbit = self.sbit.get();
            let it = self.it.get();

            let s_idle = Bit::from(st == S_IDLE);
            let s_age = Bit::from(st == S_AGE);
            let s_mul = Bit::from(st == S_MUL);
            let s_agef = Bit::from(st == S_AGEF);
            let s_rank = Bit::from(st == S_RANK);
            let s_scan = Bit::from(st == S_SCAN);
            let s_pick = Bit::from(st == S_PICK);
            let s_jacc = Bit::from(st == S_JACC);
            let s_fin = Bit::from(st == S_FIN);
            let s_div = Bit::from(st == S_DIV);
            let s_sq0 = Bit::from(st == S_SQ0);
            let s_sq = Bit::from(st == S_SQ);
            let s_acc = Bit::from(st == S_ACC);
            let s_out = Bit::from(st == S_OUT);

            let max80 = U::<80>::from(MAXDISP);
            let sign64 = U::<64>::from(1u64 << 63);

            // ------------------------------------------------ event intake
            let hs = Bit::from(samples.peek().is_some());
            let hm = Bit::from(miss.peek().is_some());
            let take_s = s_idle & hs;
            let take_m = s_idle & !hs & hm;
            let take = take_s | take_m;
            let _ = samples.recv_if(take_s);
            let _ = miss.recv_if(take_m);
            let sh = samples.head();
            let mh = miss.head();
            let ep = mux(take_s, sh.peer, mh);
            let hd_n = self.p_head.read(ep) - U::<3>::from(1u8);
            let rc_n = (self.p_reach.read(ep) << 1u32) | take_s.zext::<8>();
            let widx = (ep.zext::<5>() << 3u32) | hd_n.zext::<5>();
            let meta_n = (sh.stratum.zext::<106>() << 98u32)
                | (sh.leap.zext::<106>() << 96u32)
                | (sh.ref_id.zext::<106>() << 64u32)
                | (sh.root_delay.zext::<106>() << 32u32)
                | sh.root_disp.zext::<106>();

            // ------------------------------------------------ per-peer reads (peer cp)
            let head = self.p_head.read(cp);
            let tnow = self.p_tnow.read(cp);
            let tlast = self.p_tlast.read(cp);
            let hv = self.p_hv.read(cp).bit(0);
            let jj3 = jj.slice::<0, 3>();

            // ------------------------------------------------ S_AGE
            let age_done = Bit::from(jj == 8);
            let has_bit = rm.bit(0);
            let sx_j = sidx(cp, head, jj3);
            let t4_j = self.m_t4.read(sx_j);
            let d0 = tnow - t4_j;
            let d = mux(d0.bit(63), U::<64>::from(0u8), d0);
            let age_load = s_age & !age_done & has_bit;
            let age_skip = s_age & !age_done & !has_bit;

            // ------------------------------------------------ S_MUL
            let mbit = mb.bit(0);
            let macc_n = macc + mux(mbit, ma, U::<80>::from(0u8));
            let mb_n = mb >> 1u32;
            let mul_last = Bit::from(mb_n == 0);

            // ------------------------------------------------ S_AGEF
            let dsp_j = self.m_dsp.read(sx_j);
            let eff = dsp_j.zext::<80>() + (macc >> 32u32);
            let eff_ok = Bit::from(eff < max80);

            // ------------------------------------------------ S_RANK
            let cand = um & !pm;
            let r8 = Bit::from(r == 8);
            let rank_end = s_rank & r8;
            let rank_empty = s_rank & !r8 & Bit::from(cand == 0);
            let rank_scan = s_rank & !r8 & !Bit::from(cand == 0);

            // ------------------------------------------------ S_SCAN
            let dly_j = self.m_dly.read(sx_j);
            let better = cm.bit(0) & (!found | Bit::from(dly_j < bdly));
            let scan_last = Bit::from((cm >> 1u32) == 0);

            // ------------------------------------------------ S_PICK
            let sx_b = sidx(cp, head, bj);
            let off_b = self.m_off.read(sx_b);
            let t4_b = self.m_t4.read(sx_b);
            let eff_b = self.m_eff.read(bj);
            // True |off_b - off0| of two signed 64-bit values, saturated at MAXDISP.
            let ge_o = Bit::from((off_b ^ sign64) >= (off0 ^ sign64));
            let dd0 = mux(ge_o, off_b - off0, off0 - off_b);
            let dd = mux(Bit::from(dd0 >= U::<64>::from(MAXDISP)), U::<64>::from(MAXDISP), dd0);
            let pick0 = s_pick & Bit::from(r == 0);
            let pick_j = s_pick & !Bit::from(r == 0);

            // ------------------------------------------------ S_FIN
            let e_new = t40 - tlast;
            let newer = !Bit::from(e_new == 0) & !e_new.bit(63);
            let accept = !Bit::from(mcnt == 0) & (!hv | newer);
            let fin_jit = s_fin & accept & Bit::from(mcnt != 1);
            let fin_one = s_fin & accept & Bit::from(mcnt == 1);

            // ------------------------------------------------ S_DIV
            let rem_s = (rem << 1u32) | dv.bit(74).zext::<5>();
            let div_ge = Bit::from(rem_s >= dnm);
            let div_last = Bit::from(it == 74);

            // ------------------------------------------------ S_SQ
            let sq_t = sres + sbit;
            let sq_ge = Bit::from(q >= sq_t);
            let sq_last = Bit::from(it == 37);

            // ------------------------------------------------ S_OUT
            let meta = self.p_meta.read(cp);
            let stratum = meta.slice::<98, 8>();
            let leap = meta.slice::<96, 2>();
            let reach = self.p_reach.read(cp);
            let valid = hv
                & !Bit::from(reach == 0)
                & Bit::from(stratum >= 1)
                & Bit::from(stratum <= 15)
                & !Bit::from(leap == 3);
            let send_o = s_out & stats.ready();
            if send_o.to_bool() {
                stats.send(PeerStat {
                    peer: cp,
                    valid,
                    offset: self.p_roff.read(cp),
                    delay: self.p_rdly.read(cp),
                    disp: self.p_rdsp.read(cp),
                    jitter: self.p_rjit.read(cp),
                    reach,
                    stratum,
                    leap,
                    ref_id: meta.slice::<64, 32>(),
                    root_delay: meta.slice::<32, 32>(),
                    root_disp: meta.slice::<0, 32>(),
                    t4: tlast,
                });
            }

            with!(self <= {
                // Intake: shift the stage ring, reach, store the sample.
                take ? p_head.at(ep): hd_n,
                take ? p_reach.at(ep): rc_n,
                take_s ? m_off.at(widx): sh.offset,
                take_s ? m_dly.at(widx): sh.delay,
                take_s ? m_dsp.at(widx): sh.disp,
                take_s ? m_t4.at(widx): sh.t4,
                take_s ? p_tnow.at(ep): sh.t4,
                take_s ? p_meta.at(ep): meta_n,
                take ? {
                    st: U::<4>::from(S_AGE),
                    cp: ep,
                    jj: U::<4>::from(0u8),
                    rm: rc_n,
                    um: U::<8>::from(0u8),
                    pm: U::<8>::from(0u8),
                    r: U::<4>::from(0u8),
                    dsum: U::<64>::from(0u8),
                    jsum: U::<80>::from(0u8),
                    mcnt: U::<4>::from(0u8),
                },
                // Ageing pass.
                s_age & age_done ? st: U::<4>::from(S_RANK),
                age_load ? {
                    st: U::<4>::from(S_MUL),
                    mk: Bit::Zero,
                    ma: d.zext::<80>(),
                    mb: U::<64>::from(PHI_Q32),
                    macc: U::<80>::from(0u8),
                },
                age_skip ? {
                    um: um >> 1u32,
                    rm: rm >> 1u32,
                    jj: jj + U::<4>::from(1u8),
                },
                // Serial multiplier.
                s_mul ? {
                    macc: macc_n,
                    ma: ma << 1u32,
                    mb: mb_n,
                },
                s_mul & mul_last ? st: mux(mk, U::<4>::from(S_JACC), U::<4>::from(S_AGEF)),
                s_agef ? {
                    um: (um >> 1u32) | (eff_ok.zext::<8>() << 7u32),
                    rm: rm >> 1u32,
                    jj: jj + U::<4>::from(1u8),
                    st: U::<4>::from(S_AGE),
                },
                s_agef ? m_eff.at(jj3): eff.slice::<0, 64>(),
                // Ranks.
                rank_end ? st: U::<4>::from(S_FIN),
                rank_empty ? {
                    dsum: dsum + shr_rank(U::<64>::from(MAXDISP), r),
                    r: r + U::<4>::from(1u8),
                },
                rank_scan ? {
                    cm: cand,
                    oh: U::<8>::from(1u8),
                    jj: U::<4>::from(0u8),
                    found: Bit::Zero,
                    st: U::<4>::from(S_SCAN),
                },
                s_scan ? {
                    cm: cm >> 1u32,
                    oh: oh << 1u32,
                    jj: jj + U::<4>::from(1u8),
                },
                s_scan & better ? {
                    found: Bit::One,
                    bdly: dly_j,
                    bm: oh,
                    bj: jj3,
                },
                s_scan & scan_last ? st: U::<4>::from(S_PICK),
                s_pick ? {
                    pm: pm | bm,
                    dsum: dsum + shr_rank(eff_b, r),
                    mcnt: mcnt + U::<4>::from(1u8),
                    r: r + U::<4>::from(1u8),
                },
                pick0 ? {
                    off0: off_b,
                    dly0: bdly,
                    t40: t4_b,
                    st: U::<4>::from(S_RANK),
                },
                pick_j ? {
                    ma: dd.zext::<80>(),
                    mb: dd,
                    macc: U::<80>::from(0u8),
                    mk: Bit::One,
                    st: U::<4>::from(S_MUL),
                },
                s_jacc ? {
                    jsum: jsum + macc,
                    st: U::<4>::from(S_RANK),
                },
                // Finish: accept or not.
                s_fin ? st: U::<4>::from(S_OUT),
                fin_one ? {
                    sres: U::<80>::from(0u8),
                    st: U::<4>::from(S_ACC),
                },
                fin_jit ? {
                    dv: jsum,
                    dnm: (mcnt - U::<4>::from(1u8)).zext::<5>(),
                    rem: U::<5>::from(0u8),
                    q: U::<80>::from(0u8),
                    it: U::<7>::from(0u8),
                    st: U::<4>::from(S_DIV),
                },
                // Restoring division.
                s_div ? {
                    rem: mux(div_ge, rem_s - dnm, rem_s),
                    q: (q << 1u32) | div_ge.zext::<80>(),
                    dv: dv << 1u32,
                    it: it + U::<7>::from(1u8),
                },
                s_div & div_last ? st: U::<4>::from(S_SQ0),
                // Square root, digit by digit.
                s_sq0 ? {
                    sres: U::<80>::from(0u8),
                    sbit: U::<80>::from(1u128 << 74),
                    it: U::<7>::from(0u8),
                    st: U::<4>::from(S_SQ),
                },
                s_sq ? {
                    q: mux(sq_ge, q - sq_t, q),
                    sres: (sres >> 1u32) + mux(sq_ge, sbit, U::<80>::from(0u8)),
                    sbit: sbit >> 2u32,
                    it: it + U::<7>::from(1u8),
                },
                s_sq & sq_last ? st: U::<4>::from(S_ACC),
                // Store the accepted result.
                s_acc ? {
                    st: U::<4>::from(S_OUT),
                },
                s_acc ? p_roff.at(cp): off0,
                s_acc ? p_rdly.at(cp): dly0,
                s_acc ? p_rdsp.at(cp): dsum,
                s_acc ? p_rjit.at(cp): sres.slice::<0, 64>(),
                s_acc ? p_tlast.at(cp): t40,
                s_acc ? p_hv.at(cp): U::<1>::from(1u8),
                send_o ? st: U::<4>::from(S_IDLE),
            });
        }
    }
}
