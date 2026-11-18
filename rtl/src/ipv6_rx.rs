// SPDX-License-Identifier: Apache-2.0
//! IPv6 receive (task T3.1), domain `DefaultClock` (`clk_sys`, 100 MHz).
//! Mirror of `ipv4_rx`.
//!
//! Ports (flat): `meta` (`Rx<EthMeta>`) and `payload` (`Rx<FrameByte>`) are
//! the IPv6 meta and payload channels of `eth_rx`; config wires `local_ll`
//! (`In<U<128>>`, link-local address, derived from the MAC by the
//! configuration), `local_gua` (`In<U<128>>`, global address), `gua_valid`
//! (`In<Bit>`), `sn_low` (`In<U<24>>`, low 24 bits of our unicast
//! addresses, the same wire as `eth_rx`'s); outputs, in this order:
//! `icmp_meta` (`Tx<Ip6Meta>`), `icmp` (`Tx<FrameByte>`), `udp_meta`
//! (`Tx<Ip6Meta>`), `udp` (`Tx<FrameByte>`). Addresses are `U<128>` with the
//! first wire byte in the most significant byte.
//!
//! # Behaviour (rules G1-G8 of `tools/golden/responder.py`)
//!
//! The payload of one Ethernet frame (IPv6 header, data, Ethernet padding)
//! is read one byte per cycle; the 40 header bytes are parsed on the fly and
//! the verdict is taken on byte 40, **before any payload byte is
//! forwarded**. The packet is forwarded only if all of these hold:
//!
//! * E1: the Ethernet source is not group addressed (`EthMeta.src` bit 40);
//! * G1: version 6 (high nibble of byte 0); the payload length is not 0 (a
//!   packet without data is dropped: the stream contract has no empty
//!   frame); the frame has a byte after the header (not `last` on byte 40);
//!   the payload length against the frame length is treated below;
//! * G2: next header 58 (ICMPv6) or 17 (UDP). Every other value, **all
//!   extension headers included** (0, 43, 44, 50, 51, 60, ...), is dropped:
//!   extension headers are never walked;
//! * G3: hop limit not 0 (the NDP rule "hop limit = 255" is the business of
//!   the ICMPv6 unit: `hop` is passed in the meta);
//! * G4: the source is not multicast (first byte != `0xFF`). The unspecified
//!   source `::` is **accepted** here exactly as the model does (the model
//!   only rejects it later, in ICMPv6/NDP, T3.3, which needs it for DAD
//!   neighbor solicitations; `::` to a unicast address passes this layer and
//!   the upper layers decide);
//! * G5: destination is `local_ll`, or `local_gua` when `gua_valid`, **and**
//!   the Ethernet destination is unicast (`EthMeta.multicast` clear: `eth_rx`
//!   already matched our MAC, so this means our MAC);
//! * G6: or the destination is the solicited-node multicast address of our
//!   unicast addresses, `ff02::1:ffXX:XXXX` with `XX:XXXX` = `sn_low`, **and**
//!   the Ethernet destination is the matching `33:33:ff:XX:XX:XX` (a
//!   solicited-node packet on a unicast MAC is dropped);
//! * G7: any other destination is dropped, **`ff02::1` (all-nodes) and every
//!   other multicast included**: the model has no rule that accepts them
//!   (the Router Advertisement path of T3.5 will need ff02::1 for next header
//!   58: a one-line change of the destination class, to be made with the
//!   model);
//! * G8: UDP (next header 17) only to our unicast addresses (G5), never to
//!   the solicited-node group; ICMPv6 to either;
//! * G9 (UDP only; RFC 4291 sections 2.5.2/2.5.3/2.5.5, RFC 6890; security
//!   finding F8): a UDP (NTP) datagram is dropped when its source is the
//!   unspecified address `::`, the loopback `::1`, an IPv4-mapped address
//!   (`::ffff:0:0/96`) or one of our own addresses (`local_ll`, or
//!   `local_gua` when `gua_valid`); multicast sources are G4's. The rule
//!   is checked here rather than in `udp6_rx` because only this unit has
//!   the local addresses; `udp6_rx` is unchanged. It does NOT apply to
//!   ICMPv6: a DAD Neighbor Solicitation legitimately comes from `::` and
//!   is forwarded to `icmpv6`, which applies its own rules. The verdict on
//!   the source is registered every cycle (`usrc_ok_r`; the source is
//!   complete from byte 25 on, the verdict is read on byte 40).
//!
//! The solicited-node group is matched with the single `sn_low` wire for
//! both addresses (the model derives one per address, they coincide when
//! the global address uses the same interface identifier, which is the
//! SLAAC/EUI-64 case this design supports).
//!
//! A dropped packet emits nothing on any channel. The configuration wires
//! are sampled in the cycle of byte 40 (they must be stable during a frame).
//!
//! # Outputs
//!
//! For a forwarded packet, on the channels of its next header: one
//! [`Ip6Meta`] sent in the cycle byte 40 is taken (so before the first
//! payload byte), then the IPv6 payload (40-byte header stripped, extension
//! headers do not exist here) as `FrameByte`, truncated to the payload
//! length field (Ethernet padding removed), `last` on its final byte, one
//! byte per cycle at best. The meta and the payload are on separate
//! channels: a consumer must not make popping the meta depend on payload
//! progress. `len` of the meta is the payload length field and equals the
//! number of payload bytes that follow; `eth_src` is the Ethernet source,
//! `hop` the received hop limit, `multicast` is set when the destination is
//! the solicited-node group.
//!
//! The meta of the frame is popped from `meta` with its first payload byte
//! (that byte is not taken before the meta is visible); one meta is popped
//! for every frame, dropped or not.
//!
//! # `err` and frames the length cannot be checked on
//!
//! Same as `ipv4_rx`:
//!
//! * the input `last` carries `err` (bad FCS): the output `last` carries
//!   `err`, **also when the packet ended before the padding did**: the final
//!   byte of the packet is held back when it is not the frame's last byte,
//!   the padding is swallowed, and the held byte is sent with `last` when the
//!   input `last` arrives (at most the padding's duration, 46 bytes);
//! * payload length longer than the frame (the model drops the packet; the
//!   frame's end is not known at byte 40): the meta and the bytes have been
//!   forwarded, the last input byte is sent with `last & err` (abort) and
//!   consumers discard it. With an input `last & err`, this is the only
//!   case where a forwarded payload ends with `err`.
//!
//! # Integration
//!
//! `meta` and `payload` come from `eth_rx` through registered channels
//! (never port to port); each output must feed a registered channel or FIFO.
//! One input byte is taken exactly when the output it feeds has room. The
//! ICMPv6 and UDP units read `icmp_meta`/`icmp`, `udp_meta`/`udp` and answer
//! through `ipv6_tx` with `Ip6TxReq.src_ip` = `Ip6Meta.dst`.
use crate::eth_rx::EthMeta;
use crate::stream::FrameByte;
use txhdl::comp::{mux, Clock, DefaultClock, In, Reg, Rx, Tx, Unit};
use txhdl::types::{Bit, U};
use txhdl::{assume, check, cover, lower, with, Trace, Transaction as TransactionDerive, Value as ValueDerive};

/// Next header values.
pub const NH_ICMPV6: u8 = 58;
pub const NH_UDP: u8 = 17;

/// Parsed IPv6 header of a forwarded packet, one per packet, before its
/// first payload byte.
#[derive(TransactionDerive, ValueDerive, Clone, Copy, Default, Debug)]
pub struct Ip6Meta {
    pub src: U<128>,
    pub dst: U<128>,
    /// Next header (58 or 17).
    pub next: U<8>,
    /// Payload length in bytes, IPv6 header excluded.
    pub len: U<16>,
    pub hop: U<8>,
    /// Source MAC of the frame (the reply goes there; no NDP table).
    pub eth_src: U<48>,
    /// The destination is a multicast address (solicited-node group).
    pub multicast: Bit,
    /// Receive timestamp of the frame (copied from `EthMeta.ts`).
    pub rx_ts: U<64>,
}

#[derive(Trace, Default)]
pub struct Ipv6Rx {
    /// Header bytes taken so far, 0..=40 (40 = in the payload / dropped part).
    pub pos: Reg<U<6>>,
    /// All header rules checked so far hold.
    pub ok: Reg<Bit>,
    pub plen: Reg<U<16>>,
    pub nxt: Reg<U<8>>,
    pub hop: Reg<U<8>>,
    pub src: Reg<U<128>>,
    /// G9 verdict on `src` (valid UDP source), registered every cycle.
    pub usrc_ok_r: Reg<Bit>,
    pub dst: Reg<U<128>>,
    pub esrc: Reg<U<48>>,
    /// Receive timestamp of the frame in progress (from `EthMeta.ts`).
    pub ets: Reg<U<64>>,
    pub edst: Reg<U<48>>,
    /// Payload bytes left, the current one included (while forwarding).
    pub rem: Reg<U<16>>,
    /// The packet is being forwarded to ICMP / UDP.
    pub f_icmp: Reg<Bit>,
    pub f_udp: Reg<Bit>,
    /// The final packet byte is held, the padding is being swallowed.
    pub sw: Reg<Bit>,
    pub held: Reg<U<8>>,
    // Monitors (check!/cover! only): the raw first header byte, the payload bytes sent for
    // the packet in progress, and the previous cycle's configuration wires.
    pub m_b0: Reg<U<8>>,
    pub m_s8: Reg<U<8>>,
    pub m_cnt: Reg<U<16>>,
    pub m_ll: Reg<U<128>>,
    pub m_gua: Reg<U<128>>,
    pub m_gv: Reg<Bit>,
    pub m_sn: Reg<U<24>>,
    pub m_lv: Reg<Bit>,
}

#[lower]
impl
    Unit<
        (
            Rx<EthMeta>,
            Rx<FrameByte>,
            In<U<128>>,
            In<U<128>>,
            In<Bit>,
            In<U<24>>,
        ),
        (Tx<Ip6Meta>, Tx<FrameByte>, Tx<Ip6Meta>, Tx<FrameByte>),
    > for Ipv6Rx
{
    async fn run(
        &mut self,
        (meta, payload, local_ll, local_gua, gua_valid, sn_low): (
            Rx<EthMeta>,
            Rx<FrameByte>,
            In<U<128>>,
            In<U<128>>,
            In<Bit>,
            In<U<24>>,
        ),
        (icmp_m, icmp_p, udp_m, udp_p): (Tx<Ip6Meta>, Tx<FrameByte>, Tx<Ip6Meta>, Tx<FrameByte>),
    ) {
        loop {
            DefaultClock::rising().await;
            let have = payload.peek().is_some();
            let b = payload.head();
            let last = b.last;
            let pos = self.pos.get();
            let ok = self.ok.get();
            let plen = self.plen.get();
            let rem = self.rem.get();
            let f_icmp = self.f_icmp.get();
            let f_udp = self.f_udp.get();
            let sw = self.sw.get();
            let in_h = Bit::from(pos < 40);
            let p0 = Bit::from(pos == 0);
            let p4 = Bit::from(pos == 4);
            let p5 = Bit::from(pos == 5);
            let p6 = Bit::from(pos == 6);
            let p7 = Bit::from(pos == 7);
            let p8 = Bit::from(pos == 8);
            let p39 = Bit::from(pos == 39);
            let in_src = Bit::from(pos >= 8) & Bit::from(pos < 24);
            let in_dst = Bit::from(pos >= 24) & Bit::from(pos < 40);

            // First byte: the meta of the frame must be there.
            let m = meta.head();
            let mv = Bit::from(meta.peek().is_some());

            // Header rules, byte by byte.
            let c0 = Bit::from(b.data.slice::<4, 4>() == U::<4>::from(6u8)) & !m.src.bit(40);
            let c7 = Bit::from(b.data != U::<8>::from(0u8));
            let c8 = Bit::from(b.data != U::<8>::from(0xFFu8));
            let ok_n = (p0 & c0) | (!p0 & ok & (!p7 | c7) & (!p8 | c8));

            // Verdict on byte 40 (index 39).
            let src = self.src.get();
            let dst_fin = (self.dst.get() << 8u32) | b.data.zext::<128>();
            let edst = self.edst.get();
            let mac_uni = !edst.bit(40);
            let own_ll = Bit::from(dst_fin == local_ll.get());
            let own_gua = gua_valid.get() & Bit::from(dst_fin == local_gua.get());
            let own = (own_ll | own_gua) & mac_uni;
            let sn_addr = U::<128>::from(0xff02_0000_0000_0000_0000_0001_ff00_0000u128)
                | sn_low.get().zext::<128>();
            let sn_mac = U::<48>::from(0x3333_ff00_0000u64) | sn_low.get().zext::<48>();
            let sn = Bit::from(dst_fin == sn_addr) & Bit::from(edst == sn_mac);
            let nxt = self.nxt.get();
            let to_icmp = Bit::from(nxt == U::<8>::from(NH_ICMPV6));
            let to_udp = Bit::from(nxt == U::<8>::from(NH_UDP));
            // G9: valid UDP source (registered from `src`).
            let src_top = src >> 32u32;
            let src_low = src.slice::<0, 32>();
            let usrc_n = !(Bit::from(src_top == U::<128>::from(0u128)) & Bit::from(src_low <= U::<32>::from(1u32)))
                & !Bit::from(src_top == U::<128>::from(0xFFFFu128))
                & !Bit::from(src == local_ll.get())
                & !(gua_valid.get() & Bit::from(src == local_gua.get()));
            let accept = ok_n
                & Bit::from(plen != U::<16>::from(0u16))
                & !last
                & ((to_icmp & (own | sn)) | (to_udp & own & self.usrc_ok_r.get()));
            let dec_icmp = p39 & accept & to_icmp;
            let dec_udp = p39 & accept & to_udp;
            let dec_fwd = dec_icmp | dec_udp;

            // Payload phase.
            let fwd = (f_icmp | f_udp) & !sw;
            let is_end = Bit::from(rem == U::<16>::from(1u16));
            let hold = fwd & is_end & !last;
            let send_pay = fwd & !hold;
            let send_held = sw & last;
            let meta_room = (dec_icmp & icmp_m.ready()) | (dec_udp & udp_m.ready());
            let pay_room = (f_icmp & icmp_p.ready()) | (f_udp & udp_p.ready());

            let take = Bit::from(have)
                & (!p0 | mv)
                & (!dec_fwd | meta_room)
                & (!(send_pay | send_held) | pay_room);
            let _ = payload.recv_if(take);
            let _ = meta.recv_if(take & p0);

            let ip_meta = Ip6Meta {
                src,
                dst: dst_fin,
                next: nxt,
                len: plen,
                hop: self.hop.get(),
                eth_src: self.esrc.get(),
                multicast: sn,
                rx_ts: self.ets.get(),
            };
            let m_icmp = take & dec_icmp;
            let m_udp = take & dec_udp;
            if m_icmp.to_bool() {
                icmp_m.send(ip_meta);
            }
            if m_udp.to_bool() {
                udp_m.send(ip_meta);
            }
            // The byte sent: the current one, or the held one at the frame end.
            let o_data = mux(send_held, self.held.get(), b.data);
            let o_last = send_held | is_end | last;
            let o_err = b.err | (send_pay & last & !is_end);
            let ob = FrameByte {
                data: o_data,
                last: o_last,
                err: o_err,
            };
            let p_icmp = take & (send_pay | send_held) & f_icmp;
            let p_udp = take & (send_pay | send_held) & f_udp;
            if p_icmp.to_bool() {
                icmp_p.send(ob);
            }
            if p_udp.to_bool() {
                udp_p.send(ob);
            }

            // ---- formal properties (orchestrator; docs/formal.md, rows "IPv6" and "Flux") ----
            assume!(!have | !b.err | last, "payload: err only together with last (stream contract 2)");
            assume!(!self.m_lv.get() | (Bit::from(local_ll.get() == self.m_ll.get()) & Bit::from(local_gua.get() == self.m_gua.get()) & (gua_valid.get() == self.m_gv.get()) & Bit::from(sn_low.get() == self.m_sn.get())), "local_ll, local_gua, gua_valid and sn_low are constant (configuration wires)");
            let m_any = m_icmp | m_udp;
            let m_dec = take & dec_fwd;
            let local_sn = sn_low.get().zext::<128>();
            let p_any = p_icmp | p_udp;
            let m_cnt = self.m_cnt.get();
            let fwd_any = f_icmp | f_udp;
            let own_uni = Bit::from(dst_fin == local_ll.get()) | (gua_valid.get() & Bit::from(dst_fin == local_gua.get()));
            let src_top96 = src.slice::<32, 96>();
            // The verdict restated from raw bytes and wires (independent of ok, accept, usrc_ok_r).
            check!(!m_any | p39, "a packet is forwarded on byte 40 of the header only");
            check!(!m_any | (Bit::from(self.m_b0.get().slice::<4, 4>() == 6u8) & !self.esrc.get().bit(40)), "version 6, Ethernet source not group addressed (G1, E1)");
            check!(!m_any | Bit::from(plen != 0u16), "payload length not 0 (G1)");
            check!(!m_any | !last, "a packet has a byte after the header");
            check!(!m_any | !Bit::from(self.hop.get() == 0u8), "hop limit not 0 (G3)");
            check!(!m_any | !Bit::from(src.slice::<120, 8>() == 0xFFu8), "source not multicast (G4)");
            check!(!m_icmp | Bit::from(nxt == 58u8), "ICMPv6 meta only for next header 58 (G2)");
            check!(!m_udp | Bit::from(nxt == 17u8), "UDP meta only for next header 17 (G2)");
            check!(!(m_icmp & m_udp), "a packet goes to one protocol");
            check!(!(f_icmp & f_udp), "forwarding to one protocol at a time");
            check!(!(m_any & fwd_any), "a new packet is not decided while one is being forwarded");
            check!(!m_udp | (own_uni & !self.edst.get().bit(40)), "UDP only to our unicast addresses on our unicast MAC (G5, G8)");
            check!(!m_icmp | ((own_uni & !self.edst.get().bit(40)) | (Bit::from(dst_fin == (U::<128>::from(0xff02_0000_0000_0000_0000_0001_ff00_0000u128) | local_sn)) & Bit::from(self.edst.get() == (U::<48>::from(0x3333_ff00_0000u64) | sn_low.get().zext::<48>())))), "ICMPv6 to our unicast addresses or to our solicited-node group with the matching MAC (G5, G6)");
            check!(!m_udp | (!Bit::from(src == 0u8) & !Bit::from(src == 1u8) & !Bit::from(src_top96 == 0xFFFFu16) & !Bit::from(src == local_ll.get()) & !(gua_valid.get() & Bit::from(src == local_gua.get()))), "UDP source is not ::, ::1, IPv4-mapped or one of our own addresses (G9)");
            check!(!fwd_any | Bit::from(pos == 40), "forwarding only past the 40-byte header");
            // Invariants tying `ok` to the raw bytes (the induction needs them).
            check!(!(ok & Bit::from(pos >= 1)) | (Bit::from(self.m_b0.get().slice::<4, 4>() == 6u8) & !self.esrc.get().bit(40)), "ok: byte 0 and the group bit were good");
            check!(!(ok & Bit::from(pos >= 8)) | !Bit::from(self.hop.get() == 0u8), "ok: hop limit was not 0");
            check!(!(ok & Bit::from(pos >= 9)) | !Bit::from(self.m_s8.get() == 0xFFu8), "ok: the first source byte was not 0xFF");
            check!(!Bit::from(pos == 9) | Bit::from(src.slice::<0, 8>() == self.m_s8.get()), "source shift register: the first source byte is at byte 0 after 1 bytes");
            check!(!Bit::from(pos == 10) | Bit::from(src.slice::<8, 8>() == self.m_s8.get()), "source shift register: the first source byte is at byte 1 after 2 bytes");
            check!(!Bit::from(pos == 11) | Bit::from(src.slice::<16, 8>() == self.m_s8.get()), "source shift register: the first source byte is at byte 2 after 3 bytes");
            check!(!Bit::from(pos == 12) | Bit::from(src.slice::<24, 8>() == self.m_s8.get()), "source shift register: the first source byte is at byte 3 after 4 bytes");
            check!(!Bit::from(pos == 13) | Bit::from(src.slice::<32, 8>() == self.m_s8.get()), "source shift register: the first source byte is at byte 4 after 5 bytes");
            check!(!Bit::from(pos == 14) | Bit::from(src.slice::<40, 8>() == self.m_s8.get()), "source shift register: the first source byte is at byte 5 after 6 bytes");
            check!(!Bit::from(pos == 15) | Bit::from(src.slice::<48, 8>() == self.m_s8.get()), "source shift register: the first source byte is at byte 6 after 7 bytes");
            check!(!Bit::from(pos == 16) | Bit::from(src.slice::<56, 8>() == self.m_s8.get()), "source shift register: the first source byte is at byte 7 after 8 bytes");
            check!(!Bit::from(pos == 17) | Bit::from(src.slice::<64, 8>() == self.m_s8.get()), "source shift register: the first source byte is at byte 8 after 9 bytes");
            check!(!Bit::from(pos == 18) | Bit::from(src.slice::<72, 8>() == self.m_s8.get()), "source shift register: the first source byte is at byte 9 after 10 bytes");
            check!(!Bit::from(pos == 19) | Bit::from(src.slice::<80, 8>() == self.m_s8.get()), "source shift register: the first source byte is at byte 10 after 11 bytes");
            check!(!Bit::from(pos == 20) | Bit::from(src.slice::<88, 8>() == self.m_s8.get()), "source shift register: the first source byte is at byte 11 after 12 bytes");
            check!(!Bit::from(pos == 21) | Bit::from(src.slice::<96, 8>() == self.m_s8.get()), "source shift register: the first source byte is at byte 12 after 13 bytes");
            check!(!Bit::from(pos == 22) | Bit::from(src.slice::<104, 8>() == self.m_s8.get()), "source shift register: the first source byte is at byte 13 after 14 bytes");
            check!(!Bit::from(pos == 23) | Bit::from(src.slice::<112, 8>() == self.m_s8.get()), "source shift register: the first source byte is at byte 14 after 15 bytes");
            check!(!Bit::from(pos >= 24) | Bit::from(src.slice::<120, 8>() == self.m_s8.get()), "the source register holds the first source byte on top once complete");
            // Payload.
            check!(!p_any | fwd_any, "a payload byte is sent only for a forwarded packet");
            check!(!p_icmp | f_icmp, "ICMP payload only on the ICMP channel");
            check!(!p_udp | f_udp, "UDP payload only on the UDP channel");
            check!(!(p_icmp & p_udp), "one channel per byte");
            check!(!p_any | !sw | last, "while the final byte is held only it is sent, with the frame's last");
            check!(!(p_any & o_last) | last, "the packet ends with the frame (last of the input)");
            check!(!(p_any & o_err) | o_last, "err only together with last (stream contract 2)");
            check!(!(p_any & o_last & !o_err) | Bit::from(m_cnt + 1 == plen), "a good packet carries exactly the payload length bytes");
            check!(!(fwd_any & !sw) | (Bit::from(rem >= 1) & Bit::from(m_cnt + rem == plen)), "forwarding: sent + remaining = payload length");
            check!(!sw | (Bit::from(rem == 0) & Bit::from(m_cnt + 1 == plen)), "final byte held: all others were sent");
            check!(!sw | fwd_any, "a held byte belongs to a forwarded packet");
            cover!(m_icmp, "an ICMPv6 packet is forwarded");
            cover!(m_udp, "a UDP packet is forwarded");
            cover!(take & p39 & !accept & !last, "a header is rejected on byte 40");
            cover!(send_held, "a held final byte is sent after the padding");
            cover!(p_any & o_last & !o_err, "a good packet is completed");

            with!(self <= {
                usrc_ok_r: usrc_n,
                m_ll: local_ll.get(),
                m_gua: local_gua.get(),
                m_gv: gua_valid.get(),
                m_sn: sn_low.get(),
                m_lv: Bit::One,
                m_dec ? m_cnt: U::<16>::from(0u16),
                p_any ? m_cnt: m_cnt + U::<16>::from(1u16),
                take & p0 ? m_b0: b.data,
                take & p8 ? m_s8: b.data,
                take & in_h ? pos: pos + U::<6>::from(1u8),
                take ? ok: ok_n,
                take & p0 ? {
                    esrc: m.src,
                    ets: m.ts,
                    edst: m.dst,
                },
                take & p4 ? plen: b.data.zext::<16>() << 8u32,
                take & p5 ? plen: plen | b.data.zext::<16>(),
                take & p6 ? nxt: b.data,
                take & p7 ? hop: b.data,
                take & in_src ? src: (src << 8u32) | b.data.zext::<128>(),
                take & in_dst ? dst: dst_fin,
                take & dec_fwd ? {
                    f_icmp: dec_icmp,
                    f_udp: dec_udp,
                    rem: plen,
                },
                take & fwd ? rem: rem - U::<16>::from(1u16),
                take & hold ? {
                    sw: Bit::One,
                    held: b.data,
                },
                take & last ? {
                    pos: U::<6>::from(0u8),
                    f_icmp: Bit::Zero,
                    f_udp: Bit::Zero,
                    sw: Bit::Zero,
                },
            });
        }
    }
}
