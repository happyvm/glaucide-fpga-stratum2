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
//!   the solicited-node group; ICMPv6 to either.
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
use txhdl::{lower, with, Trace, Transaction as TransactionDerive, Value as ValueDerive};

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
    pub dst: Reg<U<128>>,
    pub esrc: Reg<U<48>>,
    pub edst: Reg<U<48>>,
    /// Payload bytes left, the current one included (while forwarding).
    pub rem: Reg<U<16>>,
    /// The packet is being forwarded to ICMP / UDP.
    pub f_icmp: Reg<Bit>,
    pub f_udp: Reg<Bit>,
    /// The final packet byte is held, the padding is being swallowed.
    pub sw: Reg<Bit>,
    pub held: Reg<U<8>>,
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
            let accept = ok_n
                & Bit::from(plen != U::<16>::from(0u16))
                & !last
                & ((to_icmp & (own | sn)) | (to_udp & own));
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

            with!(self <= {
                take & in_h ? pos: pos + U::<6>::from(1u8),
                take ? ok: ok_n,
                take & p0 ? {
                    esrc: m.src,
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
