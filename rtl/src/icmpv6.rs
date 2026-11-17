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
//! * anything else (other types, RA/NA/Redirect, code != 0, hop limit of an
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
//! cycles later** (states CK1..), it is answered only if: the target is
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
//! follows); `err` is never set on an NA. The NA starts about 6 cycles after
//! the NS's last byte is taken; the input is not read while the NA is built
//! and sent (the `ipv6_rx` channels buffer it).
//!
//! The seed `pseudo_v6` is a single-cycle path (19 chained 32-bit additions)
//! from the registered `meta` to `acc`: a timing watch point.
//!
//! # Integration
//!
//! Inputs and outputs must be registered channels or FIFOs (never port to
//! port). The `req` of a message is sent before its first `out_payload` byte,
//! one `req` is followed by exactly one frame on `out_payload`. The input
//! meta is not needed before byte 0 (do not make its availability depend on
//! this unit's payload progress).
use crate::csum::{csum_add, csum_fold, pseudo_v6};
use crate::ipv6_rx::Ip6Meta;
use crate::ipv6_tx::Ip6TxReq;
use crate::stream::FrameByte;
use txhdl::comp::{mux, Clock, DefaultClock, In, Reg, Rx, Tx, Unit};
use txhdl::types::{Bit, U};
use txhdl::{lower, with, Trace};
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

#[derive(Trace, Default)]
pub struct Icmpv6 {
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
}

#[lower]
impl
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
    > for Icmpv6
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
            let p0 = is_g & Bit::from(pos == 0);
            let p1 = is_g & Bit::from(pos == 1);
            let p2 = is_g & Bit::from(pos == 2);
            let p3 = is_g & Bit::from(pos == 3);

            let have = Bit::from(payload.peek().is_some());
            let b = payload.head();
            let last = b.last;
            let m = meta.head();
            let mv = Bit::from(meta.peek().is_some());

            // Decision on byte 1.
            let echo_acc = Bit::from(hi == U::<8>::from(TYPE_ECHO_REQUEST))
                & Bit::from(b.data == U::<8>::from(0u8))
                & !last
                & Bit::from(mlen >= 8)
                & !self.mmc.get();
            let ns_acc = Bit::from(hi == U::<8>::from(TYPE_NS))
                & Bit::from(b.data == U::<8>::from(0u8))
                & !last
                & Bit::from(mlen >= 24)
                & self.h255.get();
            let echo_go = p1 & echo_acc;
            let na_acc = Bit::from(hi == U::<8>::from(TYPE_NA))
                & Bit::from(b.data == U::<8>::from(0u8))
                & !last
                & Bit::from(mlen >= 24)
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
            let sum_ok = Bit::from(csum_fold(next) == U::<16>::from(0xFFFFu16));
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

            // NA preparation, all from registers.
            let own = Bit::from(tgt == local_ll.get())
                | (gua_valid.get() & Bit::from(tgt == local_gua.get()));
            let notmc = Bit::from(tgt.slice::<120, 8>() != U::<8>::from(0xFFu8));
            let sn_tgt = U::<128>::from(0xff02_0000_0000_0000_0000_0001_ff00_0000u128)
                | (tgt & U::<128>::from(0x00FF_FFFFu128));
            let to_tgt = Bit::from(mdst == tgt);
            let m3 = to_tgt | Bit::from(mdst == sn_tgt);
            let unspec = Bit::from(msrc == U::<128>::from(0u128));
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
            // Learning (state CK1): NA target / NS source with their MAC.
            let need_ns = ok_ns & !unspec & sseen;
            let need_na = notmc & !sbad;
            let lrn_want = is_ck1 & mux(isna, need_na, need_ns);
            let lrn_go = lrn_want & learn.ready();
            if lrn_go.to_bool() {
                learn.send(NdLearn {
                    ip: mux(isna, tgt, msrc),
                    mac: mux(isna, mux(sseen, smac, esrc), smac),
                });
            }
            let ns_ok = is_ck1 & !isna & ok_ns;
            let ns_adv = ns_ok & (!need_ns | learn.ready());
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

            let to_out = take & ((p2 & last) | p3);
            let end_in = take
                & last
                & !is_nsi
                & !(is_g & (Bit::from(pos == 2) | Bit::from(pos == 3)));
            let end_out = send_o & Bit::from(oc == 3) & abort;
            let rst = end_in | end_out;
            with!(self <= {
                take ? {
                    pos: pos + U::<16>::from(1u16),
                    hi: b.data,
                },
                take & p1 ? isna: na_acc,
                // Solicitation to send: latch the target, check it next cycle.
                rs_take ? {
                    tgt: rt,
                    st: U::<4>::from(RS1),
                },
                rs_own ? st: U::<4>::from(GATHER),
                rs_go ? {
                    st: U::<4>::from(CK2),
                    gsol: Bit::One,
                    fl: Bit::Zero,
                    mdst: gen_src,
                    msrc: sn_tgt,
                    esrc: gen_mac,
                    smac: local_mac.get(),
                },
                take & (is_g | is_pass | is_nsi) ? acc: next,
                take & p0 ? {
                    acc: seed,
                    esrc: m.eth_src,
                    msrc: m.src,
                    mdst: m.dst,
                    mlen: m.len,
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
                ns_end ? {
                    st: mux(good, U::<4>::from(CK1), U::<4>::from(GATHER)),
                    pos: U::<16>::from(0u16),
                    ost: U::<2>::from(0u8),
                    bad: Bit::Zero,
                },
                ns_end & !good ? {
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
                rst ? {
                    st: U::<4>::from(GATHER),
                    pos: U::<16>::from(0u16),
                    oc: U::<5>::from(0u8),
                    abort: Bit::Zero,
                },
            });
        }
    }
}
