// SPDX-License-Identifier: Apache-2.0
//! Synchronisation state of the server (task T4.6), domain `DefaultClock`
//! (`clk_sys`, 100 MHz). Turns the system peer chosen by the selection
//! (`Selected`, `ntp_types.rs`) and the servo's "converged" flag into the
//! wires `ntp_server.rs` announces: `synced`, `stratum`, `ref_id`, `ref_ts`,
//! `root_delay`, `root_disp` (and `leap`, the LI of the reply). RFC 5905
//! §7.3 and §11. The Python reference is `tools/golden/ntpclient/sync.py`
//! (the Rust tests use literals it produced).
//!
//! # Ports
//!
//! Inputs: `sel: Rx<Selected>`; wires `time: In<U<96>>` (NTP time =
//! `time >> 32`, 32.32 seconds, as the `DisciplinedClock` output),
//! `servo_synced: In<Bit>`, `precision: In<U<8>>` (signed log2 seconds,
//! two's complement, the `precision` of `config.rs`), `peer_ref0` ..
//! `peer_ref3: In<U<32>>` (OUR Reference ID if upstream slot `i` is the
//! system peer: `refid_ipv4(addr)` or `refid_ipv6(addr)` of
//! `ConfigSet::UPSTREAM[i]`, computed at synthesis; not the peer's own
//! `Selected.ref_id`, which is unused).
//! Outputs, all wires driven by registers every cycle: `synced: Out<Bit>`,
//! `stratum: Out<U<8>>`, `ref_id: Out<U<32>>`, `ref_ts: Out<U<64>>`,
//! `root_delay: Out<U<32>>`, `root_disp: Out<U<32>>`, `leap: Out<U<2>>`.
//!
//! # The last `Selected` (state `S`)
//!
//! `sel` is popped whenever it has an element (never back-pressured); a
//! stalled `sel` just leaves `S` unchanged. Let `usable = valid &
//! (1 <= stratum <= 14)` of the popped value (stratum 0 is not a source;
//! 15 would give 16 once incremented, see below). At the edge it is popped
//! (time `now_pop = time >> 32` of that edge):
//! * `v <= usable` (also when the value is not valid: that clears `v`);
//! * if `usable`: `peer, stratum, root_delay, root_disp, delay, disp,
//!   jitter, t4` are copied from it and `upd <= now_pop` (the time of the
//!   last update of our clock, our own clock, 32.32).
//!
//! Fields of an unusable value are NOT copied: `S` keeps the last usable
//! one (this is where `ref_ts` and the dispersion base are "kept" while
//! unsynchronised). Reset: `v = 0`, everything else 0.
//!
//! # Synchronised iff `ok`
//!
//! `ok = v & servo_synced & fresh`, with `fresh = (now < upd) |
//! (now - upd <= SYNC_TIMEOUT_S << 32)`, `now = time >> 32` (64-bit,
//! unsigned; `now < upd`, i.e. the clock was stepped back, counts as age 0).
//! [`SYNC_TIMEOUT_S`] = 1024 s (ntpd's NTP_MAXAGE order of magnitude). The
//! age is measured from `upd`, the time the `Selected` was popped, not from
//! `Selected.t4`. A non-usable `Selected` (invalid, stratum 0, 15, 16...)
//! clears `v` and so makes `ok` low at once. `servo_synced` is used live.
//!
//! # Outputs, with `ok` the value computed as above
//!
//! Let `age = max(0, now - S.t4)` (64-bit, 32.32 seconds).
//! * `synced = ok`; `leap = 0` if `ok` else 3. v1 does not propagate leap
//!   seconds (`Selected.leap` is not used: a usable peer has LI < 3 by the
//!   clock filter and we announce no pending leap second).
//! * `stratum = S.stratum + 1` (2..=15 by `usable`) if `ok`, else 16. The
//!   `+ 1` is fixed by the protocol (RFC 5905 §7.3: a server is one stratum
//!   above its source), not a setting: there is no `STRATUM_OFFSET` in
//!   `config.rs`.
//! * `ref_id = peer_ref[S.peer]` if `ok`, else the last value (see below).
//! * `ref_ts = S.upd & !0xFFFF_FFFF` (`S.upd` rounded DOWN to a whole second,
//!   the 32 fraction bits cleared) if `ok`, else the last value. Rounding hides
//!   the instant of each poll (and so the scheduler's phase) from whoever
//!   queries the server (security finding F10; RFC 5905 only asks for the
//!   time of the last update, a one-second resolution is within the intent
//!   of the field and what ntpd's reference time carries in practice).
//! * `root_delay = sat32(S.root_delay + sat32(ceil16(S.delay)))`.
//! * `root_disp = sat32(S.root_disp + sat32(ceil48(T)))` with
//!   `T = (S.disp + J) * 2^32 + PHI * age`, an exact integer in units of
//!   2^-64 s, where `J = max(S.jitter, 2^precision)` (the jitter floor of
//!   RFC 5905 §11.2.3, a power of two in 32.32: `1 << (32 + p)` for `p` in
//!   -32..=-1, `1 << 32` for `p >= 0`, `1` for `p < -32`) and
//!   `PHI = PHI_Q32` = 64425 (2^-32 s per second of age, 15 ppm, see
//!   ntp_types.rs; `age` is in 32.32 so `PHI * age` is in 2^-64 s).
//!   No sum wraps (`age < 2^64`, `PHI < 2^16`: `T < 2^98`).
//!   `ceil48(x) = (x + 2^48 - 1) >> 48` converts 2^-64 s to 16.16 rounding
//!   UP (a dispersion is never understated); `ceil16(x) = (x + 0xFFFF) >>
//!   16` does the same for 32.32; `sat32(x) = min(x, 0xFFFFFFFF)`.
//!   The dispersion therefore grows with `time` while no update arrives:
//!   1 s of age adds 64425 / 2^16 = 0.983 units of 16.16 (about 15 us),
//!   1000 s add about 983 units (15 ms).
//!   The `Selected.root_dist` is not used.
//!
//! # Unsynchronised
//!
//! `synced = 0`, `stratum = 16`, `leap = 3`; `ref_id`, `ref_ts`,
//! `root_delay`, `root_disp` HOLD their last value computed while `ok`
//! (0 after reset), as the golden model keeps the reference id,
//! timestamp and roots of its `CONFIG` in `ntp_unsynchronised.hex`
//! (`ntp_server.rs` then forces LI 3 and stratum 16 by itself).
//!
//! # Timing
//!
//! Registered pipeline, no handshake. A `Selected` is popped at edge `E`
//! (visible in `S` after `E`). Stage 1 (after `E+1`) computes from `S`,
//! `time`, `servo_synced`, `precision`, `peer_ref*`: `age`, `ok`, the
//! disp-base `S.disp + J`, `root_delay`, the muxed ref id, `stratum + 1`.
//! Stage 2 (after `E+2`) multiplies `PHI * age`, as two 32 x 16 partial
//! products (`p2_plo` = `PHI * age[31:0]`, `p2_phi` = `PHI * age[63:32]`;
//! two DSP slices deep instead of four, task T5.3). The output registers
//! (after `E+3`) hold the final sums. Every output of one cycle belongs to
//! the same pipeline snapshot (no mixed old/new values). So output(`E+3`)
//! is the function of `S` and of the inputs sampled at `E+1` (`time`,
//! `servo_synced`, `precision`, `peer_ref*`); the `age` is thus measured at
//! `E+1`, and for the steady state `out(c) = f(S(c-3), inputs(c-2))`.
//! `servo_synced` dropping, or `S` changing, reaches `synced` three edges
//! later.
use crate::ntp_types::{Selected, PHI_Q32};
use txhdl::comp::{mux, Clock, DefaultClock, In, Out, Reg, Rx, Unit};
use txhdl::types::{Bit, U};
use txhdl::{check, cover, lower, with, Trace};

/// A `Selected` older than this many seconds no longer keeps us synchronised.
pub const SYNC_TIMEOUT_S: u64 = 1024;
/// The timeout in 32.32.
pub const SYNC_TIMEOUT_Q32: u64 = SYNC_TIMEOUT_S << 32;
/// Highest usable stratum of a source (we announce one more, at most 15).
pub const MAX_SRC_STRATUM: u8 = 14;

/// Saturate to 32 bits.
#[lower]
pub fn sat32(x: U<96>) -> U<32> {
    mux(
        Bit::from(x > U::<96>::from(0xFFFF_FFFFu32)),
        U::<32>::from(0xFFFF_FFFFu32),
        x.slice::<0, 32>(),
    )
}

/// 32.32 to 16.16, rounding up (result as wide as the input).
#[lower]
pub fn ceil16(x: U<96>) -> U<96> {
    (x + U::<96>::from(0xFFFFu32)) >> 16u32
}

/// 2^-64 s to 16.16, rounding up (the result is below 2^80).
#[lower]
pub fn ceil48(x: U<128>) -> U<96> {
    ((x + U::<128>::from(0xFFFF_FFFF_FFFFu64)) >> 48u32).slice::<0, 96>()
}

/// `2^precision` in 32.32, for the signed log2 `pr` (see the module header).
#[lower]
pub fn prec_floor(pr: U<8>) -> U<64> {
    let sh = pr.slice::<0, 5>();
    let x0 = U::<64>::from(1u32);
    let x1 = mux(sh.bit(0), x0 << 1u32, x0);
    let x2 = mux(sh.bit(1), x1 << 2u32, x1);
    let x3 = mux(sh.bit(2), x2 << 4u32, x2);
    let x4 = mux(sh.bit(3), x3 << 8u32, x3);
    let x5 = mux(sh.bit(4), x4 << 16u32, x4);
    let nonneg = !pr.bit(7);
    let tiny = pr.bit(7) & Bit::from(pr < U::<8>::from(0xE0u8));
    mux(
        nonneg,
        U::<64>::from(1u64 << 32),
        mux(tiny, U::<64>::from(1u32), x5),
    )
}

/// `2^precision` in 32.32 written as a table (formal reference of
/// [`prec_floor`]): `pr` is a signed log2, -32..=-1 gives `1 << (32 + pr)`,
/// 0 and above `1 << 32`, below -32 `1`.
#[lower]
fn prec_table(pr: U<8>) -> U<64> {
    mux(pr < 128, U::<64>::from(4294967296u64), mux(pr == 255, U::<64>::from(2147483648u64), mux(pr == 254, U::<64>::from(1073741824u64), mux(pr == 253, U::<64>::from(536870912u64), mux(pr == 252, U::<64>::from(268435456u64), mux(pr == 251, U::<64>::from(134217728u64), mux(pr == 250, U::<64>::from(67108864u64), mux(pr == 249, U::<64>::from(33554432u64), mux(pr == 248, U::<64>::from(16777216u64), mux(pr == 247, U::<64>::from(8388608u64), mux(pr == 246, U::<64>::from(4194304u64), mux(pr == 245, U::<64>::from(2097152u64), mux(pr == 244, U::<64>::from(1048576u64), mux(pr == 243, U::<64>::from(524288u64), mux(pr == 242, U::<64>::from(262144u64), mux(pr == 241, U::<64>::from(131072u64), mux(pr == 240, U::<64>::from(65536u64), mux(pr == 239, U::<64>::from(32768u64), mux(pr == 238, U::<64>::from(16384u64), mux(pr == 237, U::<64>::from(8192u64), mux(pr == 236, U::<64>::from(4096u64), mux(pr == 235, U::<64>::from(2048u64), mux(pr == 234, U::<64>::from(1024u64), mux(pr == 233, U::<64>::from(512u64), mux(pr == 232, U::<64>::from(256u64), mux(pr == 231, U::<64>::from(128u64), mux(pr == 230, U::<64>::from(64u64), mux(pr == 229, U::<64>::from(32u64), mux(pr == 228, U::<64>::from(16u64), mux(pr == 227, U::<64>::from(8u64), mux(pr == 226, U::<64>::from(4u64), mux(pr == 225, U::<64>::from(2u64), mux(pr == 224, U::<64>::from(1u64), U::<64>::from(1u32))))))))))))))))))))))))))))))))))
}

/// Formal reference of the root dispersion (module header): `sat32(rds +
/// sat32(ceil48(T)))`, `T = dj * 2^32 + PHI * age` given as `hi * 2^32 + lo`.
#[lower]
fn rds_ref(rds: U<32>, hi: U<128>, lo: U<128>) -> U<32> {
    let t = (hi << 32u32) + lo;
    let up = mux(Bit::from(t.slice::<0, 48>() == 0), U::<128>::from(0u8), U::<128>::from(1u8));
    let c = (t >> 48u32) + up;
    let c32 = mux(Bit::from(c >= U::<128>::from(1u64 << 32)), U::<33>::from(0xFFFF_FFFFu32), c.slice::<0, 33>());
    let sum = rds.zext::<33>() + c32;
    mux(sum.bit(32), U::<32>::from(0xFFFF_FFFFu32), sum.slice::<0, 32>())
}

#[derive(Trace, Default)]
pub struct SyncState {
    // ---- S: the last usable Selected ----
    pub v: Reg<Bit>,
    pub peer: Reg<U<2>>,
    pub s_st: Reg<U<8>>,
    pub s_rdl: Reg<U<32>>,
    pub s_rds: Reg<U<32>>,
    pub delay: Reg<U<64>>,
    pub disp: Reg<U<64>>,
    pub jit: Reg<U<64>>,
    pub t4: Reg<U<64>>,
    pub upd: Reg<U<64>>,
    // ---- stage 1 ----
    pub p1_age: Reg<U<64>>,
    pub p1_ok: Reg<Bit>,
    pub p1_dj: Reg<U<65>>,
    pub p1_rdl: Reg<U<32>>,
    pub p1_rds: Reg<U<32>>,
    pub p1_st: Reg<U<8>>,
    pub p1_rid: Reg<U<32>>,
    pub p1_upd: Reg<U<64>>,
    // ---- stage 2 ----
    pub p2_plo: Reg<U<48>>,
    pub p2_phi: Reg<U<48>>,
    pub p2_ok: Reg<Bit>,
    pub p2_dj: Reg<U<65>>,
    pub p2_rdl: Reg<U<32>>,
    pub p2_rds: Reg<U<32>>,
    pub p2_st: Reg<U<8>>,
    pub p2_rid: Reg<U<32>>,
    pub p2_upd: Reg<U<64>>,
    // ---- output registers ----
    pub o_ok: Reg<Bit>,
    pub o_st: Reg<U<8>>,
    pub o_rid: Reg<U<32>>,
    pub o_rts: Reg<U<64>>,
    pub o_rdl: Reg<U<32>>,
    pub o_rds: Reg<U<32>>,
    // ---- monitors (only read by check!/cover!; pruned in synthesis) ----
    // The last usable Selected as it was popped, and whether one ever was.
    pub ms_v: Reg<Bit>,
    pub ms_ever: Reg<Bit>,
    pub ms_peer: Reg<U<2>>,
    pub ms_st: Reg<U<8>>,
    pub ms_rdl: Reg<U<32>>,
    pub ms_rds: Reg<U<32>>,
    pub ms_delay: Reg<U<64>>,
    pub ms_disp: Reg<U<64>>,
    pub ms_jit: Reg<U<64>>,
    pub ms_t4: Reg<U<64>>,
    pub ms_upd: Reg<U<64>>,
    // The outputs expected from it and from the wires, computed in one step
    // from the module header, then delayed as the pipeline (x1, x2, x3).
    pub x1_ok: Reg<Bit>,
    pub x1_sst: Reg<U<8>>,
    pub x1_pst: Reg<U<8>>,
    pub x1_srdl: Reg<U<32>>,
    pub x1_srds: Reg<U<32>>,
    pub x1_rid: Reg<U<32>>,
    pub x1_rts: Reg<U<64>>,
    pub x1_rdl: Reg<U<32>>,
    pub x1_rds: Reg<U<32>>,
    pub x2_ok: Reg<Bit>,
    pub x2_sst: Reg<U<8>>,
    pub x2_pst: Reg<U<8>>,
    pub x2_srdl: Reg<U<32>>,
    pub x2_srds: Reg<U<32>>,
    pub x2_rid: Reg<U<32>>,
    pub x2_rts: Reg<U<64>>,
    pub x2_rdl: Reg<U<32>>,
    pub x2_rds: Reg<U<32>>,
    pub x3_ok: Reg<Bit>,
    pub x3_sst: Reg<U<8>>,
    pub x3_pst: Reg<U<8>>,
    pub x3_srdl: Reg<U<32>>,
    pub x3_srds: Reg<U<32>>,
    pub x3_rid: Reg<U<32>>,
    pub x3_rts: Reg<U<64>>,
    pub x3_rdl: Reg<U<32>>,
    pub x3_rds: Reg<U<32>>,
}

#[lower]
impl
    Unit<
        (
            Rx<Selected>,
            In<U<96>>,
            In<Bit>,
            In<U<8>>,
            In<U<32>>,
            In<U<32>>,
            In<U<32>>,
            In<U<32>>,
        ),
        (
            Out<Bit>,
            Out<U<8>>,
            Out<U<32>>,
            Out<U<64>>,
            Out<U<32>>,
            Out<U<32>>,
            Out<U<2>>,
        ),
    > for SyncState
{
    async fn run(
        &mut self,
        (sel, time, servo_synced, precision, peer_ref0, peer_ref1, peer_ref2, peer_ref3): (
            Rx<Selected>,
            In<U<96>>,
            In<Bit>,
            In<U<8>>,
            In<U<32>>,
            In<U<32>>,
            In<U<32>>,
            In<U<32>>,
        ),
        (synced, stratum, ref_id, ref_ts, root_delay, root_disp, leap): (
            Out<Bit>,
            Out<U<8>>,
            Out<U<32>>,
            Out<U<64>>,
            Out<U<32>>,
            Out<U<32>>,
            Out<U<2>>,
        ),
    ) {
        loop {
            DefaultClock::rising().await;
            let now = time.get().slice::<32, 64>();

            // ---------------------------------------------- pop Selected
            let pop = Bit::from(sel.peek().is_some());
            let _ = sel.recv_if(pop);
            let s = sel.head();
            let usable = s.valid
                & Bit::from(s.stratum >= U::<8>::from(1u8))
                & Bit::from(s.stratum <= U::<8>::from(MAX_SRC_STRATUM));

            // ---------------------------------------------- stage 1
            let t4 = self.t4.get();
            let upd = self.upd.get();
            let age = mux(Bit::from(now >= t4), now - t4, U::<64>::from(0u64));
            let fresh = Bit::from(now < upd)
                | Bit::from(now - upd <= U::<64>::from(SYNC_TIMEOUT_Q32));
            let ok1 = self.v.get() & servo_synced.get() & fresh;
            let jit = self.jit.get();
            let floor = prec_floor(precision.get());
            let jmax = mux(Bit::from(jit > floor), jit, floor);
            let dj = self.disp.get().zext::<65>() + jmax.zext::<65>();
            let rdl = sat32(
                self.s_rdl.get().zext::<96>() + sat32(ceil16(self.delay.get().zext::<96>())).zext::<96>(),
            );
            let pe = self.peer.get();
            let rid = mux(
                pe.bit(1),
                mux(pe.bit(0), peer_ref3.get(), peer_ref2.get()),
                mux(pe.bit(0), peer_ref1.get(), peer_ref0.get()),
            );

            // ---------------------------------------------- stage 3
            // ceil48(T) with T = (dj + phi) * 2^32 + plo (phi, plo: the two
            // partial products of PHI * age): (T + 2^48 - 1) >> 48 =
            // (dj + phi + ((plo + 2^48 - 1) >> 32)) >> 16, exactly (the low
            // 32 bits of the shifted part only round within the floor).
            let lo_up = (self.p2_plo.get().zext::<52>() + U::<52>::from(0xFFFF_FFFF_FFFFu64)) >> 32u32;
            let hsum = self.p2_dj.get().zext::<68>()
                + (self.p2_phi.get().zext::<68>() + lo_up.zext::<68>());
            let c48 = (hsum >> 16u32).zext::<96>();
            let rds = sat32(self.p2_rds.get().zext::<96>() + sat32(c48).zext::<96>());
            let ok2 = self.p2_ok.get();

            let o_ok = self.o_ok.get();

            // ---- formal properties (orchestrator; docs/formal.md, row "État de synchro") ----
            // The expected outputs, restated from the module header on the
            // monitors (the Selected as popped) and the wires of this cycle.
            let m_usable = s.valid & !Bit::from(s.stratum == 0) & Bit::from(s.stratum < 15);
            let ms_v = self.ms_v.get();
            let ms_st = self.ms_st.get();
            let ms_upd = self.ms_upd.get();
            let ms_t4 = self.ms_t4.get();
            let m_stale = Bit::from(now >= ms_upd) & Bit::from(now - ms_upd > U::<64>::from(1024u64 << 32));
            let e_ok = ms_v & servo_synced.get() & !m_stale;
            let mp = self.ms_peer.get();
            let e_rid = mux(mp == 0, peer_ref0.get(), mux(mp == 1, peer_ref1.get(), mux(mp == 2, peer_ref2.get(), peer_ref3.get())));
            let e_rts = (ms_upd >> 32u32) << 32u32;
            // root delay: source root delay + delay in 16.16 rounded up, saturated.
            let md = self.ms_delay.get();
            let md_up = (md >> 16u32) + mux(Bit::from(md.slice::<0, 16>() == 0), U::<64>::from(0u8), U::<64>::from(1u8));
            let md32 = mux(Bit::from(md_up >= U::<64>::from(1u64 << 32)), U::<33>::from(0xFFFF_FFFFu32), md_up.slice::<0, 33>());
            let md_sum = self.ms_rdl.get().zext::<33>() + md32;
            let e_rdl = mux(md_sum.bit(32), U::<32>::from(0xFFFF_FFFFu32), md_sum.slice::<0, 32>());
            // root dispersion: source root dispersion + (disp + max(jitter, 2^precision)) + PHI * age.
            let m_floor = prec_table(precision.get());
            let m_j = mux(Bit::from(self.ms_jit.get() < m_floor), m_floor, self.ms_jit.get());
            let m_dj = self.ms_disp.get().zext::<128>() + m_j.zext::<128>();
            let m_age = mux(Bit::from(now < ms_t4), U::<64>::from(0u8), now - ms_t4);
            let e_rds = rds_ref(self.ms_rds.get(), m_dj, m_age.zext::<128>() * U::<128>::from(PHI_Q32));
            // The same from the pipeline registers (induction: each stage holds its snapshot).
            let f1_rds = rds_ref(self.p1_rds.get(), self.p1_dj.get().zext::<128>(), self.p1_age.get().zext::<128>() * U::<128>::from(PHI_Q32));
            let f2_rds = rds_ref(self.p2_rds.get(), self.p2_dj.get().zext::<128>() + self.p2_phi.get().zext::<128>(), self.p2_plo.get().zext::<128>());
            let o_stratum = mux(o_ok, self.o_st.get(), U::<8>::from(16u8));
            let o_leap = mux(o_ok, U::<2>::from(0u8), U::<2>::from(3u8));
            let x1_ok = self.x1_ok.get();
            let x2_ok = self.x2_ok.get();
            let x3_ok = self.x3_ok.get();
            let fmask = U::<64>::from(0xFFFF_FFFF_0000_0000u64);
            // S is the last usable Selected.
            check!(!(self.v.get() ^ ms_v), "v: the last Selected popped was usable (valid, stratum 1..=14)");
            check!(Bit::from(self.peer.get() == mp) & Bit::from(self.s_st.get() == ms_st) & Bit::from(self.s_rdl.get() == self.ms_rdl.get()) & Bit::from(self.s_rds.get() == self.ms_rds.get()), "S: peer, stratum, root delay and root dispersion of the last usable Selected");
            check!(Bit::from(self.delay.get() == md) & Bit::from(self.disp.get() == self.ms_disp.get()) & Bit::from(self.jit.get() == self.ms_jit.get()) & Bit::from(t4 == ms_t4) & Bit::from(upd == ms_upd), "S: delay, dispersion, jitter, t4 of the last usable Selected and the time it was popped");
            check!(!ms_v | (self.ms_ever.get() & Bit::from(ms_st >= 1) & Bit::from(ms_st <= 14)), "a usable source was popped and has stratum 1..=14");
            // Each pipeline stage holds the expected snapshot.
            check!(!(self.p1_ok.get() ^ x1_ok) & Bit::from(self.p1_st.get() == self.x1_pst.get()) & Bit::from(self.p1_rid.get() == self.x1_rid.get()) & Bit::from(self.p1_rdl.get() == self.x1_rdl.get()) & Bit::from((self.p1_upd.get() & fmask) == self.x1_rts.get()), "stage 1 holds ok, stratum + 1, our ref id, the root delay and the update time expected");
            check!(Bit::from(self.p1_rds.get() == self.x1_srds.get()) & Bit::from(f1_rds == self.x1_rds.get()), "stage 1 holds the terms of the root dispersion expected");
            check!(!(self.p2_ok.get() ^ x2_ok) & Bit::from(self.p2_st.get() == self.x2_pst.get()) & Bit::from(self.p2_rid.get() == self.x2_rid.get()) & Bit::from(self.p2_rdl.get() == self.x2_rdl.get()) & Bit::from((self.p2_upd.get() & fmask) == self.x2_rts.get()), "stage 2 holds ok, stratum + 1, our ref id, the root delay and the update time expected");
            check!(Bit::from(self.p2_rds.get() == self.x2_srds.get()) & Bit::from(f2_rds == self.x2_rds.get()), "stage 2 holds the terms of the root dispersion expected");
            check!(!x1_ok | (self.ms_ever.get() & Bit::from(self.x1_sst.get() >= 1) & Bit::from(self.x1_sst.get() <= 14) & Bit::from(self.x1_pst.get() == self.x1_sst.get() + U::<8>::from(1u8))), "stage 1 ok: a usable source of stratum 1..=14, to publish with one more");
            check!(!x2_ok | (self.ms_ever.get() & Bit::from(self.x2_sst.get() >= 1) & Bit::from(self.x2_sst.get() <= 14) & Bit::from(self.x2_pst.get() == self.x2_sst.get() + U::<8>::from(1u8))), "stage 2 ok: a usable source of stratum 1..=14, to publish with one more");
            check!(!x3_ok | (self.ms_ever.get() & Bit::from(self.x3_sst.get() >= 1) & Bit::from(self.x3_sst.get() <= 14) & Bit::from(self.x3_pst.get() == self.x3_sst.get() + U::<8>::from(1u8))), "stage 3 ok: a usable source of stratum 1..=14, to publish with one more");
            check!(Bit::from(self.x1_rdl.get() >= self.x1_srdl.get()) & Bit::from(self.x1_rds.get() >= self.x1_srds.get()), "stage 1: the roots expected are never below those of the source");
            check!(Bit::from(self.x2_rdl.get() >= self.x2_srdl.get()) & Bit::from(self.x2_rds.get() >= self.x2_srds.get()), "stage 2: the roots expected are never below those of the source");
            check!(Bit::from(self.x3_rdl.get() >= self.x3_srdl.get()) & Bit::from(self.x3_rds.get() >= self.x3_srds.get()), "stage 3: the roots expected are never below those of the source");
            // The outputs (RFC 5905 §7.3).
            check!(Bit::from(self.o_st.get() == self.x3_pst.get()), "the output stage holds the stratum expected");
            check!(!(o_ok ^ x3_ok), "synced iff, three cycles earlier, the last Selected was usable, the servo converged and the update was at most 1024 s old");
            check!(!o_ok | self.ms_ever.get(), "synced only after a usable Selected");
            check!(!o_ok | (Bit::from(o_stratum == self.x3_sst.get() + U::<8>::from(1u8)) & Bit::from(o_stratum >= 2) & Bit::from(o_stratum <= 15) & Bit::from(o_leap == 0)), "synced: stratum = stratum of the source + 1, in 2..=15, LI 0");
            check!(o_ok | (Bit::from(o_stratum == 16) & Bit::from(o_leap == 3)), "not synced: stratum 16 and LI 3");
            check!(!o_ok | Bit::from(self.o_rid.get() == self.x3_rid.get()), "synced: the reference id is ours for the peer of the source");
            check!(!o_ok | Bit::from(self.o_rts.get() == self.x3_rts.get()), "synced: the reference timestamp is the time of the last update, whole seconds");
            check!(Bit::from(self.o_rts.get().slice::<0, 32>() == 0), "the reference timestamp never carries a fraction (F10)");
            check!(!o_ok | Bit::from(self.o_rdl.get() == self.x3_rdl.get()), "synced: root delay = sat32(source root delay + ceil16(delay))");
            check!(!o_ok | Bit::from(self.o_rds.get() == self.x3_rds.get()), "synced: root dispersion = sat32(source root dispersion + ceil48((disp + max(jitter, 2^precision)) 2^32 + PHI age))");
            check!(!o_ok | (Bit::from(self.o_rdl.get() >= self.x3_srdl.get()) & Bit::from(self.o_rds.get() >= self.x3_srds.get())), "synced: root delay and root dispersion are never below those of the source");
            cover!(o_ok & Bit::from(o_stratum == 2), "synchronised to a stratum 1 source");
            cover!(o_ok & Bit::from(o_stratum == 15), "synchronised to a stratum 14 source");
            cover!(!o_ok & self.ms_ever.get() & ms_v, "a usable source but not synchronised");
            cover!(o_ok & Bit::from(self.o_rds.get() == 0xFFFF_FFFFu32), "the root dispersion saturates");

            with!(self <= {
                pop ? v: usable,
                pop & usable ? {
                    peer: s.peer,
                    s_st: s.stratum,
                    s_rdl: s.root_delay,
                    s_rds: s.root_disp,
                    delay: s.delay,
                    disp: s.disp,
                    jit: s.jitter,
                    t4: s.t4,
                    upd: now,
                },
                Bit::One ? {
                    p1_age: age,
                    p1_ok: ok1,
                    p1_dj: dj,
                    p1_rdl: rdl,
                    p1_rds: self.s_rds.get(),
                    p1_st: self.s_st.get() + U::<8>::from(1u8),
                    p1_rid: rid,
                    p1_upd: upd,
                    p2_plo: self.p1_age.get().slice::<0, 32>().zext::<48>() * U::<48>::from(PHI_Q32),
                    p2_phi: self.p1_age.get().slice::<32, 32>().zext::<48>() * U::<48>::from(PHI_Q32),
                    p2_ok: self.p1_ok.get(),
                    p2_dj: self.p1_dj.get(),
                    p2_rdl: self.p1_rdl.get(),
                    p2_rds: self.p1_rds.get(),
                    p2_st: self.p1_st.get(),
                    p2_rid: self.p1_rid.get(),
                    p2_upd: self.p1_upd.get(),
                    o_ok: ok2,
                    o_st: self.p2_st.get(),
                },
                ok2 ? {
                    o_rid: self.p2_rid.get(),
                    // F10: whole seconds only (clear the 32 fraction bits).
                    o_rts: self.p2_upd.get() & U::<64>::from(0xFFFF_FFFF_0000_0000u64),
                    o_rdl: self.p2_rdl.get(),
                    o_rds: rds,
                },
                // Monitors.
                pop ? ms_v: m_usable,
                pop & m_usable ? {
                    ms_ever: Bit::One,
                    ms_peer: s.peer,
                    ms_st: s.stratum,
                    ms_rdl: s.root_delay,
                    ms_rds: s.root_disp,
                    ms_delay: s.delay,
                    ms_disp: s.disp,
                    ms_jit: s.jitter,
                    ms_t4: s.t4,
                    ms_upd: time.get().slice::<32, 64>(),
                },
                Bit::One ? {
                    x1_ok: e_ok,
                    x1_sst: ms_st,
                    x1_pst: ms_st + U::<8>::from(1u8),
                    x1_srdl: self.ms_rdl.get(),
                    x1_srds: self.ms_rds.get(),
                    x1_rid: e_rid,
                    x1_rts: e_rts,
                    x1_rdl: e_rdl,
                    x1_rds: e_rds,
                    x2_ok: x1_ok,
                    x2_sst: self.x1_sst.get(),
                    x2_pst: self.x1_pst.get(),
                    x2_srdl: self.x1_srdl.get(),
                    x2_srds: self.x1_srds.get(),
                    x2_rid: self.x1_rid.get(),
                    x2_rts: self.x1_rts.get(),
                    x2_rdl: self.x1_rdl.get(),
                    x2_rds: self.x1_rds.get(),
                    x3_ok: x2_ok,
                    x3_sst: self.x2_sst.get(),
                    x3_pst: self.x2_pst.get(),
                    x3_srdl: self.x2_srdl.get(),
                    x3_srds: self.x2_srds.get(),
                    x3_rid: self.x2_rid.get(),
                    x3_rts: self.x2_rts.get(),
                    x3_rdl: self.x2_rdl.get(),
                    x3_rds: self.x2_rds.get(),
                },
            });

            synced.set(o_ok);
            stratum.set(o_stratum);
            ref_id.set(self.o_rid.get());
            ref_ts.set(self.o_rts.get());
            root_delay.set(self.o_rdl.get());
            root_disp.set(self.o_rds.get());
            leap.set(o_leap);
        }
    }
}
