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
//! * `stratum = S.stratum + 1` (2..=15 by `usable`) if `ok`, else 16.
//! * `ref_id = peer_ref[S.peer]` if `ok`, else the last value (see below).
//! * `ref_ts = S.upd` if `ok`, else the last value.
//! * `root_delay = sat32(S.root_delay + sat32(ceil16(S.delay)))`.
//! * `root_disp = sat32(S.root_disp + sat32(ceil16(S.disp + J + PHI *
//!   age)))` with `J = max(S.jitter, 2^precision)` (the jitter floor of RFC
//!   5905 §11.2.3, a power of two in 32.32: `1 << (32 + p)` for `p` in
//!   -32..=-1, `1 << 32` for `p >= 0`, `1` for `p < -32`) and
//!   `PHI = PHI_Q32` = 64425 (2^-32 s per second, 15 ppm, ntp_types.rs).
//!   `PHI * age` is exact integer arithmetic on the 64-bit `age`.
//!   The sums are exact in 32.32 (no wrap: `age < 2^64`, `PHI < 2^16`);
//!   `ceil16(x) = (x + 0xFFFF) >> 16` converts 32.32 to 16.16 rounding UP
//!   (a dispersion is never understated); `sat32(x) = min(x, 0xFFFFFFFF)`.
//!   The dispersion therefore grows with `time` while no update arrives
//!   (1 s of age adds 64425 / 2^16 = 0.983 units of 16.16, about 15 us;
//!   1000 s add about 983 units, 15 ms).
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
//! Stage 2 (after `E+2`) multiplies `PHI * age`. The output registers
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
use txhdl::{lower, with, Trace};

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
    pub p2_prod: Reg<U<96>>,
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
            let total = self.p2_dj.get().zext::<96>() + self.p2_prod.get();
            let rds = sat32(
                self.p2_rds.get().zext::<96>() + sat32(ceil16(total)).zext::<96>(),
            );
            let ok2 = self.p2_ok.get();

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
                    p2_prod: self.p1_age.get().zext::<96>() * U::<96>::from(PHI_Q32),
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
                    o_rts: self.p2_upd.get(),
                    o_rdl: self.p2_rdl.get(),
                    o_rds: rds,
                },
            });

            let o_ok = self.o_ok.get();
            synced.set(o_ok);
            stratum.set(mux(o_ok, self.o_st.get(), U::<8>::from(16u8)));
            ref_id.set(self.o_rid.get());
            ref_ts.set(self.o_rts.get());
            root_delay.set(self.o_rdl.get());
            root_disp.set(self.o_rds.get());
            leap.set(mux(o_ok, U::<2>::from(0u8), U::<2>::from(3u8)));
        }
    }
}
