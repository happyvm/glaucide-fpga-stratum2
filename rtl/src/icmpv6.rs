// SPDX-License-Identifier: Apache-2.0
//! ICMPv6 responder: echo reply and neighbor advertisement (task T3.3),
//! domain `DefaultClock` (`clk_sys`, 100 MHz).
//!
//! Ports (flat): `meta` (`Rx<Ip6Meta>`) and `payload` (`Rx<FrameByte>`) are
//! the ICMPv6 outputs of `ipv6_rx` (`icmp_meta`, `icmp`: the ICMPv6 message,
//! IPv6 header and Ethernet padding removed); config wires `local_ll`
//! (`In<U<128>>`), `local_gua` (`In<U<128>>`), `gua_valid` (`In<Bit>`),
//! `local_mac` (`In<U<48>>`), sampled while the unit works on a message (they
//! must be stable during a frame); `req` (`Tx<Ip6TxReq>`) and `out_payload`
//! (`Tx<FrameByte>`) feed `ipv6_tx` (`req`, `payload`), whose output goes to
//! a `tx_arb` source. Addresses are `U<128>`, first wire byte in the most
//! significant byte.
//!
//! # Behaviour (rules K1-K2 and M1-M7 of `tools/golden/responder.py`)
//!
//! The message is read one byte per cycle; `meta` is popped with byte 0. The
//! unit verifies the ICMPv6 checksum of **every** message it acts on: running
//! sum seeded with `pseudo_v6(meta.src, meta.dst, 58, meta.len)` (taken at
//! byte 0), then the message words; the verdict (folded sum == 0xFFFF) is
//! known on the last byte. On byte 1 (type, code known) the unit decides:
//!
//! * type 128, code 0, `meta.len >= 8`, `meta.multicast` clear, message
//!   continues after byte 1: echo to answer (below);
//! * type 135, code 0, `meta.hop == 255` (rule M1), `meta.len >= 24`, message
//!   continues: neighbor solicitation to examine (below);
//! * type 136 (NA), code 0, `meta.hop == 255`, `meta.len >= 24`, message
//!   continues: neighbor advertisement to consider for learning (T3.6, below);
//! * anything else (other types, RA/Redirect, code != 0, hop limit of an
//!   NS != 255, too short, echo to the solicited-node group): **nothing is
//!   sent**, no `req`, no payload byte; the message is swallowed up to its
//!   `last`.
//!
//! # Echo (K1, K2): cut-through
//!
//! One `Ip6TxReq` is sent in the cycle byte 1 is taken (if `req` has room, the
//! input waits otherwise): `dst_mac = meta.eth_src`, `dst_ip = meta.src`,
//! `src_ip = meta.dst` (the address that was targeted: a unicast address of
//! ours, because `ipv6_rx` never forwards echo to another multicast group),
//! `next = 58`, `len = meta.len`, `hop = 64`. The reply is byte for byte the
//! request except type 129 (byte 0) and the checksum (bytes 2-3). The first
//! four bytes are held until byte 3 is in; the reply starts four cycles after
//! the request and from byte 4 on a byte goes out in the cycle it is taken.
//!
//! The pseudo-header sum is unchanged (src and dst swap, length and next
//! header stay); only the type word changes, 0x8000 -> 0x8100, i.e. by
//! +0x0100. RFC 1624 eqn 3: `HC' = ~(~HC + ~m + m')` with `m = 0x8000`,
//! `m' = 0x8100`; `~m + m' = 0x7FFF + 0x8100 = 0x100FF == 0x0100 (mod
//! 0xFFFF)`, so `HC' = ~fold(~HC + 0x0100)`. `csum_fold` returns the unique
//! representative in `1..=0xFFFF` (the sum is never 0 because `0x0100` is
//! added), so `HC'` is in `0..=0xFFFE` and equals exactly what the Python
//! model computes from scratch (`~fold(sum)`, never 0xFFFF either): no 0x0000
//! / 0xFFFF ambiguity remains for a valid request. For an invalid request
//! the reply is aborted anyway.
//!
//! **Late `err`**: the request checksum is verified as the bytes stream
//! through, but the verdict only comes with the last byte, when the reply is
//! already out: if it is bad, the final byte of the reply is sent with
//! `last & err`; the input's own `err` on `last` is copied (OR). `err` is only
//! set with `last`. If the input ends before byte 4 (truncated), the four
//! first bytes are sent with `last & err` on the fourth.
//!
//! # Neighbor solicitation (M1-M6): nothing is sent before the verdict
//!
//! The 16 target bytes (message bytes 8..23) and the options are captured as
//! they stream. Options (from byte 24) are parsed as the model does: type,
//! length in units of 8 bytes (0 is malformed), the last option must end
//! exactly with the message (no overrun, no stray byte); the source
//! link-layer option (type 1) is recorded (at most one, length 1, else it is
//! "bad"). On the last byte the NS is accepted only if: checksum good,
//! message well formed, input `err` clear. Then, **all from registers, a few
//! cycles later** (states CK0, CK1..; CK0 acts on the registered verdict of
//! the last byte and registers the address comparisons used in CK1, task
//! T5.3; a rejected NS returns to GATHER from CK0), it is answered only if:
//! the target is
//! `local_ll`, or `local_gua` with `gua_valid`, and is not multicast (M2); the
//! destination is the target or the solicited-node address of the target
//! (`ff02::1:ffXX:XXXX` with the target's low 24 bits, M3); and:
//!
//! * source not `::` (M6): no SLLA problem (duplicate or length != 1 drops the
//!   NS); NA to `dst_ip = meta.src`, `dst_mac` = the SLLA option value if
//!   present (the model uses it), else `meta.eth_src`; flags S=1 O=1 R=0;
//! * source `::` (DAD, M4/M5): destination must be the solicited-node
//!   address (destination == target drops it) and no SLLA option at all;
//!   NA to `ff02::1`, `dst_mac = 33:33:00:00:00:01`, flags S=0 O=1.
//!
//! Then one `Ip6TxReq` (`src_ip` = target, `next` 58, `len` 32, `hop` 255) is
//! sent, followed by the 32-byte NA from fields: type 136, code 0, checksum,
//! flags (`0x60000000` or `0x20000000`), target, option 2 / length 1 /
//! `local_mac`. The checksum is computed over the fields before the first
//! byte: `pseudo_v6(target, dst, 58, 32)`, plus words type/flags/option and
//! the MAC words (two registered adder stages), complemented. The request
//! and the 32 bytes are always sent together (`req` only when the payload
//! follows); `err` is never set on an NA. The NA starts about 7 cycles after
//! the NS's last byte is taken; the input is not read while the NA is built
//! and sent (the `ipv6_rx` channels buffer it).
//!
//! The seed `pseudo_v6` is a single-cycle path (19 chained 32-bit additions)
//! from the registered `meta` to `acc`: a timing watch point.
//!
//! # Neighbor discovery as an initiator (task T3.6)
//!
//! Extra ports: `resolve` (`Rx<U<128>>`, input, last): an IPv6 address to
//! resolve; `learn` (`Tx<NdLearn>`, output, last): `NdLearn { ip, mac }`, a
//! neighbor learnt from the wire, for `NbrCache`. Learning never produces an
//! output frame by itself. Rules in `tools/golden/responder.py` (`solicit`,
//! `learn`, rules N1-N5, as amended by the finding F5: solicited-only).
//!
//! ## Solicitation (N1-N3)
//!
//! `resolve` is taken only when the unit is idle at a message boundary (state
//! GATHER, byte 0 not yet taken); while a message is being read, an echo reply
//! or an NA is being sent, it waits in its channel (replies first). A pending
//! `resolve` also holds back the start of the next incoming message (its meta
//! and bytes stay in the `ipv6_rx` channels), so it cannot starve. Two cycles
//! after it is taken (state RS0 registers the address comparisons, task
//! T5.3; state RS1 decides, from registers):
//!
//! * target equal to `local_ll`, or to `local_gua` with `gua_valid`: dropped,
//!   nothing is sent (N3);
//! * else one NS is sent: `Ip6TxReq { dst_mac = 33:33:ff:T[2]:T[1]:T[0]` (low
//!   24 bits of the target), `dst_ip = ff02::1:ffT[2]T[1]T[0]`, `src_ip` =
//!   `local_ll` if the target is in `fe80::/10`, else `local_gua` if
//!   `gua_valid`, else `local_ll` (N2), `next` 58, `len` 32, `hop` 255 }
//!   followed by the 32 bytes: type 135, code 0, checksum, 4 zero bytes
//!   (reserved), target, option 1 (SLLA) / length 1 / `local_mac`. Same
//!   output path and request-then-32-bytes contract as the NA; `err` never set.
//!
//! Checksum (same two registered adder stages as the NA): CK2 sums the
//! pseudo-header `pseudo_v6(src_ip, dst_ip, 58, 32)` (the NS reuses the
//! `msrc`/`mdst` registers: addition commutes, so the order of the two
//! addresses is irrelevant); CK3 adds `pseudo_v6(target, mac << 80, 0,
//! 0x8700_0101)`, i.e. the target words, the three MAC words and the words
//! `0x8700` (type, code), `0x0000` (reserved), `0x0000`, `0x0101` (option
//! type, length); the checksum word itself is zero. The checksum is the
//! complement of the fold, in `0..=0xFFFE`. The NS starts about 7 cycles after
//! `resolve` is taken. Retransmission, timeouts and duplicate suppression are
//! the caller's job (no state is kept per target). The unit does keep the
//! address of its outstanding solicitation, `resolving`, for the learning
//! rule below: `rs_go` sets `resolving` := target and `rs_wait`; the pair is
//! cleared when the answer is learned and, as a safety, after
//! `T::TIMEOUT` cycles ([`ARP_TIMEOUT_CYCLES`] = 2^26 on the board, the same
//! style as `arp.rs`; the timer restarts at each solicitation). A solicitation
//! of another target replaces `resolving` (the client has one next hop, a
//! single resolution at a time).
//!
//! ## Learning (N4): state CK1, solicited-only (F5, F6)
//!
//! * NA: type 136, code 0, `hop == 255`, `len >= 24`, checksum good, input
//!   `err` clear, options well formed (as for the NS: length 0 or overrun
//!   rejects the message), target not multicast (first byte != 0xFF), at most
//!   one Target LLA option (type 2) and then of length 1, **and it answers our
//!   outstanding solicitation**: `rs_wait` is set, target == `resolving`, the
//!   S (solicited) flag is set (bit 0x40 of byte 4), and the IPv6 source
//!   address equals the target (RFC 4861 section 7.1.2: a solicited NA is sent
//!   from the target). Then `learn{ ip = target, mac = TLLA if present, else
//!   meta.eth_src }` and `rs_wait` is cleared (one learn per solicitation).
//!   Otherwise nothing is learnt: an unsolicited NA, an NA with S = 0, an NA
//!   for another target, with a spoofed source, with hop limit != 255, or one
//!   that arrives after the timeout. No frame is sent. Whatever destination
//!   `ipv6_rx` lets through is accepted (the unit does not look at it). The O
//!   and R flags are not examined (S = 1 and a matching target is what a
//!   neighbor that answers our NS sends). Other options, including a Source
//!   LLA, are skipped.
//! * NS: **never learned from** (an NS creates no entry in our design, RFC
//!   4861 section 7.2.3 would create a STALE one; we keep no STALE state). An
//!   NS that the unit answers (rules M1-M6) is answered directly to the
//!   source link-layer option if present (else the Ethernet source), exactly
//!   as before.
//!
//! F6: with solicited-only learning a host of the link cannot flood the 4
//! entries to evict the gateway; only the reply to our own solicitation
//! inserts an entry (the residual risk is an attacker that answers before the
//! genuine neighbor).
//!
//! # Integration
//!
//! Inputs and outputs must be registered channels or FIFOs (never port to
//! port). The `req` of a message is sent before its first `out_payload` byte,
//! one `req` is followed by exactly one frame on `out_payload`. The input
//! meta is not needed before byte 0 (do not make its availability depend on
//! this unit's payload progress).
use crate::arp::ArpTiming;
use crate::csum::{csum_add, csum_fold, csum_ok, pseudo_v6};
use crate::ipv6_rx::Ip6Meta;
use crate::ipv6_tx::Ip6TxReq;
use crate::stream::FrameByte;
use txhdl::comp::{mux, Clock, DefaultClock, In, Reg, Rx, Tx, Unit};
use txhdl::types::{Bit, U};
use std::marker::PhantomData;
use txhdl::{assume, check, cover, lower, with, Trace};
use txhdl::{Transaction as TransactionDerive, Value as ValueDerive};

/// A neighbor learnt from the wire (output `learn`): IPv6 address and the
/// link-layer address it is reachable at.
#[derive(TransactionDerive, ValueDerive, Clone, Copy, Default, Debug)]
pub struct NdLearn {
    pub ip: U<128>,
    pub mac: U<48>,
}

pub const NH_ICMPV6: u8 = 58;
pub const TYPE_ECHO_REQUEST: u8 = 128;
pub const TYPE_ECHO_REPLY: u8 = 129;
pub const TYPE_NS: u8 = 135;
pub const TYPE_NA: u8 = 136;
pub const ECHO_HOP: u8 = 64;
pub const NDP_HOP: u8 = 255;

const GATHER: u8 = 0;
const OUT: u8 = 1;
const PASS: u8 = 2;
const DROP: u8 = 3;
const NSI: u8 = 4;
const CK1: u8 = 5;
const CK2: u8 = 6;
const CK3: u8 = 7;
const REQ: u8 = 8;
const NAO: u8 = 9;
const RS1: u8 = 10;
const CK0: u8 = 11;
const RS0: u8 = 12;

/// The ICMPv6 unit of the board (pending-solicitation timeout 2^26 cycles).
pub type Icmpv6 = Icmpv6With<crate::arp::Board>;

#[derive(Trace, Default)]
pub struct Icmpv6With<T: ArpTiming> {
    pub st: Reg<U<4>>,
    /// Bytes of the message taken so far.
    pub pos: Reg<U<16>>,
    /// Running sum (not folded) and high byte in progress.
    pub acc: Reg<U<32>>,
    pub hi: Reg<U<8>>,
    pub mlen: Reg<U<16>>,
    pub msrc: Reg<U<128>>,
    pub mdst: Reg<U<128>>,
    pub esrc: Reg<U<48>>,
    pub mmc: Reg<Bit>,
    pub h255: Reg<Bit>,
    /// Echo: first four reply bytes, shifted as they go out.
    pub oreg: Reg<U<32>>,
    /// Echo: bytes of `oreg` sent; NA: bytes sent (0..=31).
    pub oc: Reg<U<5>>,
    pub abort: Reg<Bit>,
    /// NS: target address, shifted out during the NA.
    pub tgt: Reg<U<128>>,
    /// NS option parser: 0 expects a type, 1 a length, 2 is in the body.
    pub ost: Reg<U<2>>,
    pub ot: Reg<U<8>>,
    pub orem: Reg<U<12>>,
    pub bad: Reg<Bit>,
    pub sseen: Reg<Bit>,
    pub sbad: Reg<Bit>,
    /// SLLA option value; then `local_mac` for the NA.
    pub smac: Reg<U<48>>,
    /// NA flag S.
    pub fl: Reg<Bit>,
    pub ck: Reg<U<16>>,
    /// The NDP message in progress is a neighbor advertisement (else a NS).
    pub isna: Reg<Bit>,
    /// The frame being built is a solicitation we generate (else an NA).
    pub gsol: Reg<Bit>,
    /// Address comparisons of the NS check and of the solicitation,
    /// registered every cycle (read in CK1 / RS1, one cycle after CK0 / RS0,
    /// when their inputs have been stable for a cycle).
    pub own_r: Reg<Bit>,
    pub to_tgt_r: Reg<Bit>,
    pub m3_r: Reg<Bit>,
    pub unspec_r: Reg<Bit>,
    pub notmc_r: Reg<Bit>,
    /// The NS/NA just read passed its checks (written with its last byte,
    /// read in CK0).
    pub gd: Reg<Bit>,
    /// `mlen >= 8`, `mlen >= 24` (written with `mlen`).
    pub ml8: Reg<Bit>,
    pub ml24: Reg<Bit>,
    /// Solicitation outstanding (F5): its target, and a timer of the same
    /// style as `arp.rs`.
    pub rs_wait: Reg<Bit>,
    pub resolving: Reg<U<128>>,
    pub rs_tmo: Reg<U<32>>,
    /// Flag S of the NA being read (byte 4, bit 0x40).
    pub na_s: Reg<Bit>,
    /// Registered (CK0 -> CK1): target == `resolving` while waiting; source == target.
    pub rsv_r: Reg<Bit>,
    pub src_tgt_r: Reg<Bit>,
    // Monitors (check!/cover! only): an echo reply outstanding, a generated NA/NS outstanding.
    pub m_out: Reg<Bit>,
    pub m_gen: Reg<Bit>,
    pub m_ll: Reg<U<128>>,
    pub m_gua: Reg<U<128>>,
    pub m_gv: Reg<Bit>,
    pub m_mac: Reg<U<48>>,
    pub m_lv: Reg<Bit>,
    // Previous-cycle copies (check!/cover! only) for the one-cycle registered comparisons.
    pub m_rwp: Reg<Bit>,
    pub m_tgt: Reg<U<128>>,
    pub m_res: Reg<U<128>>,
    pub m_tgt_s: Reg<Bit>,
    _t: PhantomData<T>,
}

#[lower]
impl<T: ArpTiming>
    Unit<
        (
            Rx<Ip6Meta>,
            Rx<FrameByte>,
            In<U<128>>,
            In<U<128>>,
            In<Bit>,
            In<U<48>>,
            Rx<U<128>>,
        ),
        (Tx<Ip6TxReq>, Tx<FrameByte>, Tx<NdLearn>),
    > for Icmpv6With<T>
{
    async fn run(
        &mut self,
        (meta, payload, local_ll, local_gua, gua_valid, local_mac, resolve): (
            Rx<Ip6Meta>,
            Rx<FrameByte>,
            In<U<128>>,
            In<U<128>>,
            In<Bit>,
            In<U<48>>,
            Rx<U<128>>,
        ),
        (req, out_payload, learn): (Tx<Ip6TxReq>, Tx<FrameByte>, Tx<NdLearn>),
    ) {
        loop {
            DefaultClock::rising().await;
            let st = self.st.get();
            let pos = self.pos.get();
            let acc = self.acc.get();
            let hi = self.hi.get();
            let mlen = self.mlen.get();
            let msrc = self.msrc.get();
            let mdst = self.mdst.get();
            let esrc = self.esrc.get();
            let oreg = self.oreg.get();
            let oc = self.oc.get();
            let abort = self.abort.get();
            let tgt = self.tgt.get();
            let ost = self.ost.get();
            let ot = self.ot.get();
            let orem = self.orem.get();
            let bad = self.bad.get();
            let sseen = self.sseen.get();
            let sbad = self.sbad.get();
            let smac = self.smac.get();
            let fl = self.fl.get();
            let ck = self.ck.get();
            let isna = self.isna.get();
            let gsol = self.gsol.get();
            let is_g = Bit::from(st == GATHER);
            let is_out = Bit::from(st == OUT);
            let is_pass = Bit::from(st == PASS);
            let is_drop = Bit::from(st == DROP);
            let is_nsi = Bit::from(st == NSI);
            let is_ck1 = Bit::from(st == CK1);
            let is_ck2 = Bit::from(st == CK2);
            let is_ck3 = Bit::from(st == CK3);
            let is_req = Bit::from(st == REQ);
            let is_nao = Bit::from(st == NAO);
            let is_rs1 = Bit::from(st == RS1);
            let is_ck0 = Bit::from(st == CK0);
            let is_rs0 = Bit::from(st == RS0);
            // In GATHER `pos` is at most 3 (byte 3 leaves GATHER, every way
            // back sets `pos` to 0): its low 2 bits decode the position
            // (shorter than a 16-bit compare, task T5.3).
            let pg = pos.slice::<0, 2>();
            let p0 = is_g & Bit::from(pg == 0);
            let p1 = is_g & Bit::from(pg == 1);
            let p2 = is_g & Bit::from(pg == 2);
            let p3 = is_g & Bit::from(pg == 3);

            let have = Bit::from(payload.peek().is_some());
            let b = payload.head();
            let last = b.last;
            let m = meta.head();
            let mv = Bit::from(meta.peek().is_some());

            // Decision on byte 1.
            let echo_acc = Bit::from(hi == U::<8>::from(TYPE_ECHO_REQUEST))
                & Bit::from(b.data == U::<8>::from(0u8))
                & !last
                & self.ml8.get()
                & !self.mmc.get();
            let ns_acc = Bit::from(hi == U::<8>::from(TYPE_NS))
                & Bit::from(b.data == U::<8>::from(0u8))
                & !last
                & self.ml24.get()
                & self.h255.get();
            let echo_go = p1 & echo_acc;
            let na_acc = Bit::from(hi == U::<8>::from(TYPE_NA))
                & Bit::from(b.data == U::<8>::from(0u8))
                & !last
                & self.ml24.get()
                & self.h255.get();
            let ns_go = p1 & (ns_acc | na_acc);
            // A pending resolve request starts a solicitation when idle, at the
            // start of a message (before byte 0 is taken).
            let rv = Bit::from(resolve.peek().is_some());
            let rs_take = p0 & rv;
            let _ = resolve.recv_if(rs_take);
            let rt = resolve.head();
            let can = is_g | is_pass | is_drop | is_nsi;
            let take = have
                & can
                & (!p0 | (mv & !rv))
                & (!echo_go | req.ready())
                & (!is_pass | out_payload.ready());
            let _ = payload.recv_if(take);
            let _ = meta.recv_if(take & p0);

            // Running sum, seeded with the pseudo-header at byte 0.
            let odd = pos.bit(0);
            let lo = b.data.zext::<16>();
            let word = mux(odd, (hi.zext::<16>() << 8u32) | lo, lo << 8u32);
            let add = odd | last;
            let next = mux(add, csum_add(acc, word), acc);
            let sum_ok = csum_ok(next);
            let seed = pseudo_v6(
                m.src,
                m.dst,
                U::<8>::from(NH_ICMPV6),
                m.len.zext::<32>(),
            );

            // Echo: new checksum, RFC 1624 eqn 3 with m 0x8000 -> 0x8100.
            let ck_old = (hi.zext::<16>() << 8u32) | lo;
            let ck_inv = ck_old ^ U::<16>::from(0xFFFFu16);
            let ck_new = csum_fold(csum_add(ck_inv.zext::<32>(), U::<16>::from(0x0100u16)))
                ^ U::<16>::from(0xFFFFu16);

            // NS option parser.
            let in_tgt = Bit::from(pos >= 8) & Bit::from(pos < 24);
            let in_opt = Bit::from(pos >= 24);
            let o_t = is_nsi & in_opt & Bit::from(ost == 0);
            let o_l = is_nsi & in_opt & Bit::from(ost == 1);
            let o_b = is_nsi & in_opt & Bit::from(ost == 2);
            let zero_len = o_l & Bit::from(b.data == U::<8>::from(0u8));
            // Link-layer option of interest: source (1) in a NS, target (2) in a NA.
            let want_t = mux(isna, U::<8>::from(2u8), U::<8>::from(1u8));
            let slla_hdr = o_l & Bit::from(ot == want_t);
            let sbad_n = sbad
                | (slla_hdr & (sseen | Bit::from(b.data != U::<8>::from(1u8))));
            let clean = Bit::from(pos == 23) | (o_b & Bit::from(orem == 1));
            let bad_n = bad | zero_len | (last & !clean);
            let good = is_nsi & last & sum_ok & !bad_n & !b.err;
            let ns_end = take & is_nsi & last;

            // NA preparation, all from registers. The comparisons are
            // registered (`*_r`) and read one cycle later (CK1, RS1).
            let own_n = Bit::from(tgt == local_ll.get())
                | (gua_valid.get() & Bit::from(tgt == local_gua.get()));
            let notmc_n = Bit::from(tgt.slice::<120, 8>() != U::<8>::from(0xFFu8));
            let sn_tgt = U::<128>::from(0xff02_0000_0000_0000_0000_0001_ff00_0000u128)
                | (tgt & U::<128>::from(0x00FF_FFFFu128));
            let to_tgt_n = Bit::from(mdst == tgt);
            let m3_n = to_tgt_n | Bit::from(mdst == sn_tgt);
            // F5: the NA answers our outstanding solicitation.
            let rsv_n = self.rs_wait.get() & Bit::from(tgt == self.resolving.get());
            let src_tgt_n = Bit::from(msrc == tgt);
            let unspec_n = Bit::from(msrc == U::<128>::from(0u128));
            let own = self.own_r.get();
            let notmc = self.notmc_r.get();
            let to_tgt = self.to_tgt_r.get();
            let m3 = self.m3_r.get();
            let unspec = self.unspec_r.get();
            let ok_dad = !to_tgt & !sseen;
            let ok_ns = own & notmc & m3 & mux(unspec, ok_dad, !sbad);
            let all_nodes = U::<128>::from(0xff02_0000_0000_0000_0000_0000_0000_0001u128);
            let na_dst_mac = mux(
                unspec,
                U::<48>::from(0x3333_0000_0001u64),
                mux(sseen, smac, esrc),
            );
            let flag_hi = mux(fl, U::<16>::from(0x6000u16), U::<16>::from(0x2000u16));
            let len_k = ((U::<16>::from(0x8800u16) + flag_hi).zext::<32>() << 16u32)
                | U::<32>::from(0x0201u32);
            let req_go = is_req & req.ready();
            // Solicitation we generate: source address, solicited-node group.
            let ll_tgt = Bit::from(tgt.slice::<118, 10>() == U::<10>::from(0x3FAu16));
            let gen_src = mux(ll_tgt | !gua_valid.get(), local_ll.get(), local_gua.get());
            let gen_mac = U::<48>::from(0x3333_ff00_0000u64)
                | (tgt & U::<128>::from(0x00FF_FFFFu128)).slice::<0, 48>();
            let len_ns = U::<32>::from(0x8700_0101u32);
            let len_sel = mux(gsol, len_ns, len_k);
            // Learning (state CK1): only an NA that answers our solicitation
            // (F5): target == resolving, S = 1, source == target.
            let need_na = notmc & !sbad & self.rsv_r.get() & self.na_s.get() & self.src_tgt_r.get();
            let lrn_go = is_ck1 & isna & need_na & learn.ready();
            if lrn_go.to_bool() {
                learn.send(NdLearn {
                    ip: tgt,
                    mac: mux(sseen, smac, esrc),
                });
            }
            let ns_ok = is_ck1 & !isna & ok_ns;
            let ns_adv = ns_ok;
            let na_done = is_ck1 & isna & (!need_na | learn.ready());
            let na_fail = is_ck1 & !isna & !ok_ns;
            let na_end = na_fail | na_done;
            let rs_own = is_rs1 & own;
            let rs_go = is_rs1 & !own;
            // One request port: echo (with byte 1) or NA (state REQ).
            if ((take & echo_go) | req_go).to_bool() {
                req.send(Ip6TxReq {
                    dst_mac: esrc,
                    dst_ip: msrc,
                    src_ip: mux(is_req & !gsol, tgt, mdst),
                    next: U::<8>::from(NH_ICMPV6),
                    len: mux(is_req, U::<16>::from(32u16), mlen),
                    hop: mux(is_req, U::<8>::from(NDP_HOP), U::<8>::from(ECHO_HOP)),
                });
            }

            // NA bytes.
            let nao_top = tgt.slice::<120, 8>();
            let mac_top = smac.slice::<40, 8>();
            let flag_b = mux(fl, U::<8>::from(0x60u8), U::<8>::from(0x20u8));
            let na_b = mux(
                Bit::from(oc >= 26),
                mac_top,
                mux(
                    Bit::from(oc == 25),
                    U::<8>::from(1u8),
                    mux(
                        Bit::from(oc == 24),
                        mux(gsol, U::<8>::from(1u8), U::<8>::from(2u8)),
                        mux(
                            Bit::from(oc >= 8),
                            nao_top,
                            mux(
                                Bit::from(oc == 4),
                                mux(gsol, U::<8>::from(0u8), flag_b),
                                mux(
                                    Bit::from(oc == 3),
                                    ck.slice::<0, 8>(),
                                    mux(
                                        Bit::from(oc == 2),
                                        ck.slice::<8, 8>(),
                                        mux(
                                            Bit::from(oc == 0),
                                            mux(gsol, U::<8>::from(TYPE_NS), U::<8>::from(TYPE_NA)),
                                            U::<8>::from(0u8),
                                        ),
                                    ),
                                ),
                            ),
                        ),
                    ),
                ),
            );
            let send_n = is_nao & out_payload.ready();
            let n_end = send_n & Bit::from(oc == 31);

            // Echo output: first four bytes, then pass-through.
            let o_last = is_out & Bit::from(oc == 3) & abort;
            let send_o = is_out & out_payload.ready();
            let send_p = take & is_pass;
            let data = mux(
                is_out,
                oreg.slice::<24, 8>(),
                mux(is_nao, na_b, b.data),
            );
            let last_o = o_last | (is_pass & last) | n_end;
            let err_o = o_last | (is_pass & (b.err | (last & !sum_ok)));
            if (send_o | send_p | send_n).to_bool() {
                out_payload.send(FrameByte {
                    data,
                    last: last_o,
                    err: err_o,
                });
            }

            // ---- formal properties (orchestrator; docs/formal.md, rows "ICMPv6 / NDP" and "ICMP echo") ----
            assume!(!have | !b.err | last, "payload: err only together with last (stream contract 2)");
            assume!(!self.m_lv.get() | (Bit::from(local_ll.get() == self.m_ll.get()) & Bit::from(local_gua.get() == self.m_gua.get()) & (gua_valid.get() == self.m_gv.get()) & Bit::from(local_mac.get() == self.m_mac.get())), "local_ll, local_gua, gua_valid and local_mac are constant (configuration wires)");
            let m_out = self.m_out.get();
            let m_gen = self.m_gen.get();
            let e_req = take & echo_go;
            let sent_any = send_o | send_p | send_n;
            // Echo (K1, K2).
            check!(!(is_ck0 | is_ck1 | is_ck2 | is_ck3 | is_req | is_nao | is_rs0 | is_rs1) | Bit::from(pos == 0), "the message position is 0 outside the reading states");
            check!(!is_g | Bit::from(pos <= 3), "the gather phase lasts 4 bytes");
            check!(!(is_nsi | is_ck0 | is_ck1) | (self.h255.get() & self.ml24.get()), "reading or deciding an NS/NA: hop limit 255 and 24 bytes or more were checked on byte 1 (M1)");
            check!(!is_out | Bit::from(oc <= 3), "the four held echo bytes are counted 0..3");
            check!(!m_gen | is_nao, "a generated frame is outstanding only while it is output");
            check!(!is_nao | m_gen, "NAO only with a generated frame outstanding");
            check!(!m_out | is_g | is_out | is_pass, "an echo reply is outstanding only in the echo states");
            check!(!(m_out & is_g) | Bit::from(pg == 2) | Bit::from(pg == 3), "gathering with an echo outstanding: bytes 2 or 3 are next");
            check!(!(is_g & Bit::from(pos >= 2)) | m_out, "past byte 1 of a message in the gather phase, the echo was accepted");
            check!(!e_req | !(m_out | m_gen), "no echo request while a reply is outstanding");
            check!(!e_req | (Bit::from(hi == 128u8) & Bit::from(b.data == 0u8) & self.ml8.get() & !self.mmc.get()), "only an echo request with code 0, 8 bytes or more, not to a multicast group, is answered");
            check!(!(send_o & Bit::from(oc == 0)) | Bit::from(data == 0x81u8), "echo reply type is 129");
            check!(!(send_o & Bit::from(oc == 1)) | Bit::from(data == 0u8), "echo reply code is 0");
            check!(!(is_out & Bit::from(oc == 0)) | (Bit::from(oreg.slice::<24, 8>() == 0x81u8) & Bit::from(oreg.slice::<16, 8>() == 0u8)), "held reply bytes: type 129, code 0");
            check!(!(is_out & Bit::from(oc == 1)) | Bit::from(oreg.slice::<24, 8>() == 0u8), "held reply bytes: code 0");
            // Sequencing: a request is followed by exactly one frame, no byte outside one.
            check!(!sent_any | m_out | m_gen, "a payload byte is only sent for a request already sent");
            check!(!(is_out | is_pass) | m_out, "echo states only with a reply outstanding");
            check!(!is_nao | m_gen, "NA/NS output only with a generated frame outstanding");
            check!(!req_go | !(m_out | m_gen), "no generated request while a frame is outstanding");
            check!(!is_nao | Bit::from(oc <= 31), "a generated frame has 32 bytes");
            check!(!(send_n & Bit::from(oc == 31)) | last_o, "last on the 32nd byte of a generated frame");
            check!(!(send_n & last_o) | Bit::from(oc == 31), "last only on the 32nd byte of a generated frame");
            check!(!(sent_any & err_o) | last_o, "err only together with last (stream contract 2)");
            check!(!send_n | !err_o, "a generated frame never carries err");
            // Neighbor advertisement (M1-M6): only for an NS that passed the checks, to our address.
            check!(!is_ck1 | self.gd.get(), "CK1 only after a good verdict (checksum, well formed, no err)");
            check!(!(is_ck1 & !isna & ok_ns) | (self.h255.get() & self.ml24.get() & own & notmc), "an NS is answered only with hop limit 255, 24 bytes or more, target ours and not multicast (M1, M2)");
            check!(!((is_ck2 | is_ck3 | is_req) & !gsol) | (own_n & notmc_n), "while an NA is being built the target stays one of our addresses, not multicast");
            check!(!(req_go & !gsol) | (own_n & notmc_n), "an NA is built only for one of our addresses, not multicast");
            // Learning (F5/F6): only an NA that answers our outstanding solicitation.
            check!(!lrn_go | (isna & self.h255.get() & self.ml24.get() & self.gd.get() & !sbad & notmc), "learn only from a well formed NA with hop limit 255 (N4)");
            // `rsv_r` and `src_tgt_r` are registered one cycle before CK1 uses them: the target and
            // the address being resolved cannot change in between, and the wait flag may have
            // dropped (timeout) in that very cycle, so the learn is allowed one cycle after it.
            check!(Bit::from(self.rsv_r.get() == (self.m_rwp.get() & Bit::from(self.m_tgt.get() == self.m_res.get()))), "rsv_r is the previous cycle's comparison");
            check!(!is_ck1 | (Bit::from(tgt == self.m_tgt.get()) & Bit::from(self.resolving.get() == self.m_res.get())), "the target and the address being resolved are stable in CK1");
            check!(Bit::from(self.src_tgt_r.get() == Bit::from(self.m_tgt_s.get())), "src_tgt_r is the previous cycle's comparison");
            check!(!lrn_go | (self.na_s.get() & Bit::from(msrc == tgt) & Bit::from(tgt == self.resolving.get()) & (self.rs_wait.get() | self.m_rwp.get())), "learn only a solicited NA for the target being resolved, sent from the target, while waiting or in the cycle after the wait expired (F5)");
            check!(!rs_go | !own, "never solicit one of our own addresses (N3)");
            cover!(e_req, "an echo is answered");
            cover!(req_go & !gsol, "an NA is built");
            cover!(req_go & gsol, "an NS is generated");
            cover!(lrn_go, "a neighbor is learned");
            cover!(is_ck1 & isna & !need_na, "an unsolicited NA is not learned");
            cover!(send_n & last_o, "a generated frame is completed");

            let to_out = take & ((p2 & last) | p3);
            let end_in = take
                & last
                & !is_nsi
                & !(is_g & (Bit::from(pos == 2) | Bit::from(pos == 3)));
            let end_out = send_o & Bit::from(oc == 3) & abort;
            let end_all = end_in | end_out;
            with!(self <= {
                m_rwp: self.rs_wait.get(),
                m_tgt: tgt,
                m_res: self.resolving.get(),
                m_tgt_s: Bit::from(msrc == tgt),
                m_ll: local_ll.get(),
                m_gua: local_gua.get(),
                m_gv: gua_valid.get(),
                m_mac: local_mac.get(),
                m_lv: Bit::One,
                e_req ? m_out: Bit::One,
                sent_any & last_o ? m_out: Bit::Zero,
                req_go ? m_gen: Bit::One,
                n_end ? m_gen: Bit::Zero,
                take ? {
                    pos: pos + U::<16>::from(1u16),
                    hi: b.data,
                },
                take & p1 ? isna: na_acc,
                // Solicitation to send: latch the target, check it next cycle.
                own_r: own_n,
                notmc_r: notmc_n,
                to_tgt_r: to_tgt_n,
                m3_r: m3_n,
                rsv_r: rsv_n,
                src_tgt_r: src_tgt_n,
                take & is_nsi & Bit::from(pos == 4) ? na_s: b.data.bit(6),
                // Pending-solicitation timer (F5): counts while waiting.
                self.rs_wait.get() ? rs_tmo: self.rs_tmo.get() + U::<32>::from(1u32),
                self.rs_wait.get() & Bit::from(self.rs_tmo.get() == T::TIMEOUT - 1) ? rs_wait: Bit::Zero,
                lrn_go ? rs_wait: Bit::Zero,
                unspec_r: unspec_n,
                rs_take ? {
                    tgt: rt,
                    st: U::<4>::from(RS0),
                },
                is_rs0 ? st: U::<4>::from(RS1),
                is_ck0 ? st: mux(self.gd.get(), U::<4>::from(CK1), U::<4>::from(GATHER)),
                rs_own ? st: U::<4>::from(GATHER),
                rs_go ? {
                    st: U::<4>::from(CK2),
                    gsol: Bit::One,
                    fl: Bit::Zero,
                    mdst: gen_src,
                    msrc: sn_tgt,
                    esrc: gen_mac,
                    smac: local_mac.get(),
                    rs_wait: Bit::One,
                    resolving: tgt,
                    rs_tmo: U::<32>::from(0u32),
                },
                take & (is_g | is_pass | is_nsi) ? acc: next,
                take & p0 ? {
                    acc: seed,
                    esrc: m.eth_src,
                    msrc: m.src,
                    mdst: m.dst,
                    mlen: m.len,
                    ml8: Bit::from(m.len >= 8),
                    ml24: Bit::from(m.len >= 24),
                    mmc: m.multicast,
                    h255: Bit::from(m.hop == U::<8>::from(NDP_HOP)),
                },
                take & p1 & !echo_acc & !ns_acc & !na_acc & !last ? st: U::<4>::from(DROP),
                take & ns_go ? st: U::<4>::from(NSI),
                to_out ? {
                    st: U::<4>::from(OUT),
                    oreg: U::<32>::from(0x8100_0000u32) | ck_new.zext::<32>(),
                    oc: U::<5>::from(0u8),
                    abort: last,
                },
                send_o ? {
                    oreg: oreg << 8u32,
                    oc: oc + U::<5>::from(1u8),
                },
                send_o & Bit::from(oc == 3) ? st: U::<4>::from(PASS),
                // NS option parser and target capture.
                take & is_nsi & in_tgt ? tgt: (tgt << 8u32) | b.data.zext::<128>(),
                take & o_t ? {
                    ot: b.data,
                    ost: U::<2>::from(1u8),
                },
                take & o_l ? {
                    ost: U::<2>::from(2u8),
                    orem: (b.data.zext::<12>() << 3u32) - U::<12>::from(2u16),
                    sseen: sseen | slla_hdr,
                    sbad: sbad_n,
                },
                take & o_b ? {
                    orem: orem - U::<12>::from(1u16),
                    ost: mux(Bit::from(orem == 1), U::<2>::from(0u8), U::<2>::from(2u8)),
                },
                take & o_b & Bit::from(ot == want_t) ? smac: (smac << 8u32) | b.data.zext::<48>(),
                take & is_nsi ? bad: bad_n,
                // The verdict of the last byte is registered (`gd`) and acted on
                // in CK0 (task T5.3: the checksum is not on the path of `st`).
                ns_end ? {
                    st: U::<4>::from(CK0),
                    gd: good,
                    pos: U::<16>::from(0u16),
                    ost: U::<2>::from(0u8),
                    bad: Bit::Zero,
                },
                is_ck0 & !self.gd.get() ? {
                    sseen: Bit::Zero,
                    sbad: Bit::Zero,
                },
                // NA: checks, checksum stages, request, bytes.
                ns_adv ? {
                    st: U::<4>::from(CK2),
                    msrc: mux(unspec, all_nodes, msrc),
                    esrc: na_dst_mac,
                    smac: local_mac.get(),
                    fl: !unspec,
                },
                na_end ? {
                    st: U::<4>::from(GATHER),
                    sseen: Bit::Zero,
                    sbad: Bit::Zero,
                },
                is_ck2 ? {
                    acc: pseudo_v6(mux(gsol, mdst, tgt), msrc, U::<8>::from(NH_ICMPV6), U::<32>::from(32u32)),
                    st: U::<4>::from(CK3),
                },
                is_ck3 ? {
                    acc: csum_add(acc, U::<16>::from(0u16))
                        + pseudo_v6(
                            tgt,
                            smac.zext::<128>() << 80u32,
                            U::<8>::from(0u8),
                            len_sel,
                        ),
                    st: U::<4>::from(REQ),
                },
                req_go ? {
                    ck: csum_fold(acc) ^ U::<16>::from(0xFFFFu16),
                    st: U::<4>::from(NAO),
                    oc: U::<5>::from(0u8),
                },
                send_n ? oc: oc + U::<5>::from(1u8),
                send_n & Bit::from(oc >= 8) & Bit::from(oc < 24) ? tgt: tgt << 8u32,
                send_n & Bit::from(oc >= 26) ? smac: smac << 8u32,
                n_end ? {
                    st: U::<4>::from(GATHER),
                    gsol: Bit::Zero,
                    oc: U::<5>::from(0u8),
                    sseen: Bit::Zero,
                    sbad: Bit::Zero,
                },
                end_all ? {
                    st: U::<4>::from(GATHER),
                    pos: U::<16>::from(0u16),
                    oc: U::<5>::from(0u8),
                    abort: Bit::Zero,
                },
            });
        }
    }
}
