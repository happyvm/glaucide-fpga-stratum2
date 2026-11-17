// SPDX-License-Identifier: Apache-2.0
//! IPv4 receive (task T1.10), domain `DefaultClock` (`clk_sys`, 100 MHz).
//!
//! Ports (flat): `meta` (`Rx<EthMeta>`) and `payload` (`Rx<FrameByte>`) are
//! the IPv4 meta and payload channels of `eth_rx`; `local_ip` (`In<U<32>>`,
//! first wire byte most significant: `192.0.2.1` = `0xC000_0201`) is a config
//! wire; outputs, in this order: `icmp_meta` (`Tx<IpMeta>`), `icmp`
//! (`Tx<FrameByte>`), `udp_meta` (`Tx<IpMeta>`), `udp` (`Tx<FrameByte>`).
//!
//! # Behaviour (rules I0-I6 of `tools/golden/responder.py`)
//!
//! The payload of one Ethernet frame (IPv4 header, data, Ethernet padding) is
//! read one byte per cycle; the 20 header bytes are parsed on the fly and
//! the verdict is taken on byte 20, **before any payload byte is
//! forwarded**. The packet is forwarded only if all of these hold:
//!
//! * I0: the Ethernet destination is not group addressed (`EthMeta.multicast`
//!   clear: `eth_rx` already matched our MAC or broadcast, so this leaves
//!   our unicast address only); E1: the Ethernet source is not group
//!   addressed either (rule E1 of the model: a reply to a group MAC makes no
//!   sense);
//! * I1: version 4 and IHL 5 (byte 0 is `0x45`; options are I2: IHL != 5 is
//!   dropped), total length > 20 (a packet without data is dropped: the
//!   stream contract has no empty frame), header checksum correct (RFC 1071
//!   sum of the 10 words, checksum field included, folds to `0xFFFF`);
//! * I3: no fragment: MF clear and fragment offset 0 (DF is ignored);
//! * I4: TTL not 0;
//! * I5: destination address equals `local_ip` exactly;
//! * I6 (RFC 1122 section 3.2.1.3, RFC 6890; security finding F8): the source
//!   is a valid unicast host address: not in `0.0.0.0/8`, not loopback
//!   (`127.0.0.0/8`), not multicast (`224.0.0.0/4`), not class E
//!   (`240.0.0.0/4`, which includes the limited broadcast `255.255.255.255`)
//!   and not our own address (`local_ip`). Link-local `169.254.0.0/16`,
//!   private and documentation prefixes are accepted. The subnet-directed
//!   broadcast (`local_ip | !netmask`) is NOT filtered here: the unit has no
//!   netmask input. The rule applies to both ICMP and UDP (an echo from
//!   `127.x` or from ourselves is not answered). The comparison with
//!   `local_ip` is registered (`src_ok_r`, valid from byte 16 on, read with
//!   byte 20: the source is complete after byte 16);
//! * the protocol is 1 (ICMP) or 17 (UDP); other protocols are dropped;
//! * the frame has a byte after the header (not `last` on byte 20).
//!
//! A dropped packet emits nothing on any channel. The bytes `local_ip` is
//! compared with are those of the cycle of byte 20 (so it must be stable
//! during a frame).
//!
//! # Outputs
//!
//! For a forwarded packet, on the channels of its protocol: one [`IpMeta`]
//! sent in the cycle byte 20 is taken (so before the first payload byte),
//! then the IPv4 payload (header stripped) as `FrameByte`, truncated to the
//! IPv4 total length (Ethernet padding removed), `last` on its final byte,
//! one byte per cycle at best. The meta and the payload are on separate
//! channels: a consumer must not make popping the meta depend on payload
//! progress. `len` of [`IpMeta`] is total length - 20 and equals the number
//! of payload bytes that follow; `eth_src` is the Ethernet source, `ttl` the
//! received TTL.
//!
//! The meta of the frame is popped from `meta` with its first payload byte
//! (that byte is not taken before the meta is visible); one meta is popped
//! for every frame, dropped or not.
//!
//! # `err` and frames the length cannot be checked on
//!
//! Cut-through means the packet is forwarded before the frame's end is known:
//!
//! * the input `last` carries `err` (bad FCS from `fcs_check`): the output
//!   `last` carries `err`, **also when the packet ended before the padding
//!   did**. To make this possible the final byte of the packet is held back
//!   when it is not the frame's last byte, the padding is swallowed, and the
//!   held byte is sent with `last` when the input `last` arrives (it costs at
//!   most the padding's duration, 46 bytes). A forwarded payload ends with
//!   `err` only in the cases below.
//! * total length longer than the frame (the model drops it): the frame
//!   ends first; since the meta and bytes have been forwarded, the last
//!   input byte is sent with `last & err` (abort) and consumers discard it.
//!   This is the only case, with an input `last & err`, of a forwarded
//!   payload ending with `err`.
//!
//! # Integration
//!
//! `meta` and `payload` come from `eth_rx` through registered channels
//! (never port to port); each output must feed a registered channel or FIFO.
//! One input byte is taken exactly when the output it feeds has room.
use crate::csum::{csum_add, csum_ok};
use crate::eth_rx::EthMeta;
use crate::stream::FrameByte;
use txhdl::comp::{mux, Clock, DefaultClock, In, Reg, Rx, Tx, Unit};
use txhdl::types::{Bit, U};
use txhdl::{assume, check, cover, lower, with, Trace, Transaction as TransactionDerive, Value as ValueDerive};

/// IP protocol numbers.
pub const PROTO_ICMP: u8 = 1;
pub const PROTO_UDP: u8 = 17;

/// Parsed IPv4 header of a forwarded packet, one per packet, before its
/// first payload byte.
#[derive(TransactionDerive, ValueDerive, Clone, Copy, Default, Debug)]
pub struct IpMeta {
    pub src: U<32>,
    pub dst: U<32>,
    pub proto: U<8>,
    /// Payload length in bytes, IPv4 header excluded.
    pub len: U<16>,
    /// Source MAC of the frame (the reply goes there; no ARP table).
    pub eth_src: U<48>,
    pub ttl: U<8>,
    /// Receive timestamp of the frame (copied from `EthMeta.ts`).
    pub rx_ts: U<64>,
}

#[derive(Trace, Default)]
pub struct Ipv4Rx {
    /// Bytes of the IPv4 packet taken so far (header and payload).
    pub pos: Reg<U<16>>,
    /// All header rules checked so far hold.
    pub ok: Reg<Bit>,
    pub tl: Reg<U<16>>,
    pub ttl: Reg<U<8>>,
    pub proto: Reg<U<8>>,
    /// Running header sum (not folded) and high byte of the word in progress.
    pub acc: Reg<U<32>>,
    pub hi: Reg<U<8>>,
    pub src: Reg<U<32>>,
    /// I6 verdict on `src`, registered every cycle (the source is stable from
    /// byte 17 to byte 20, the verdict is read on byte 20).
    pub src_ok_r: Reg<Bit>,
    pub dst: Reg<U<32>>,
    pub esrc: Reg<U<48>>,
    /// Receive timestamp of the frame in progress (from `EthMeta.ts`).
    pub ets: Reg<U<64>>,
    /// The packet is being forwarded to ICMP / UDP.
    pub f_icmp: Reg<Bit>,
    pub f_udp: Reg<Bit>,
    /// The final packet byte is held, the padding is being swallowed.
    pub sw: Reg<Bit>,
    pub held: Reg<U<8>>,
    // Monitors (only read by check!/cover!; pruned in synthesis): the raw header bytes the
    // verdict must rest on, the group bit of the Ethernet destination, and the number of
    // payload bytes sent for the packet in progress.
    pub m_b0: Reg<U<8>>,
    pub m_b6: Reg<U<8>>,
    pub m_b7: Reg<U<8>>,
    pub m_mc: Reg<Bit>,
    pub m_cnt: Reg<U<16>>,
    pub m_lip: Reg<U<32>>,
    pub m_lv: Reg<Bit>,
}

#[lower]
impl
    Unit<
        (Rx<EthMeta>, Rx<FrameByte>, In<U<32>>),
        (Tx<IpMeta>, Tx<FrameByte>, Tx<IpMeta>, Tx<FrameByte>),
    > for Ipv4Rx
{
    async fn run(
        &mut self,
        (meta, payload, local_ip): (Rx<EthMeta>, Rx<FrameByte>, In<U<32>>),
        (icmp_m, icmp_p, udp_m, udp_p): (Tx<IpMeta>, Tx<FrameByte>, Tx<IpMeta>, Tx<FrameByte>),
    ) {
        loop {
            DefaultClock::rising().await;
            let have = payload.peek().is_some();
            let b = payload.head();
            let last = b.last;
            let pos = self.pos.get();
            let ok = self.ok.get();
            let tl = self.tl.get();
            let acc = self.acc.get();
            let hi = self.hi.get();
            let f_icmp = self.f_icmp.get();
            let f_udp = self.f_udp.get();
            let sw = self.sw.get();
            let in_h = Bit::from(pos < 20);
            let odd = pos.bit(0);
            let p0 = Bit::from(pos == 0);
            let p3 = Bit::from(pos == 3);
            let p2 = Bit::from(pos == 2);
            let p6 = Bit::from(pos == 6);
            let p7 = Bit::from(pos == 7);
            let p8 = Bit::from(pos == 8);
            let p9 = Bit::from(pos == 9);
            let p19 = Bit::from(pos == 19);
            let in_src = Bit::from(pos >= 12) & Bit::from(pos < 16);
            let in_dst = Bit::from(pos >= 16) & Bit::from(pos < 20);

            // First byte: the meta of the frame must be there.
            let m = meta.head();
            let mv = Bit::from(meta.peek().is_some());

            // Header rules, byte by byte.
            let c0 = Bit::from(b.data == U::<8>::from(0x45u8)) & !m.multicast & !m.src.bit(40);
            let c6 = Bit::from((b.data & U::<8>::from(0x3Fu8)) == U::<8>::from(0u8));
            let c7 = Bit::from(b.data == U::<8>::from(0u8));
            let c8 = Bit::from(b.data != U::<8>::from(0u8));
            let ok_n = (p0 & c0) | (!p0 & ok & (!p6 | c6) & (!p7 | c7) & (!p8 | c8));

            // Verdict on byte 20 (index 19).
            let word = (hi.zext::<16>() << 8u32) | b.data.zext::<16>();
            let sum_fin = csum_add(acc, word);
            let ck_ok = csum_ok(sum_fin);
            let src = self.src.get();
            let dst_fin = (self.dst.get() << 8u32) | b.data.zext::<32>();
            // I6: 0/8, 127/8, 224/4, 240/4 (incl. 255.255.255.255), our own address.
            let hi8 = src >> 24u32;
            let src_ok_n = Bit::from(hi8 != U::<32>::from(0u32))
                & Bit::from(hi8 != U::<32>::from(127u32))
                & Bit::from((src >> 29u32) != U::<32>::from(7u32))
                & Bit::from(src != local_ip.get());
            let src_ok = self.src_ok_r.get();
            let dst_ok = Bit::from(dst_fin == local_ip.get());
            let proto = self.proto.get();
            let to_icmp = Bit::from(proto == U::<8>::from(PROTO_ICMP));
            let to_udp = Bit::from(proto == U::<8>::from(PROTO_UDP));
            let accept = ok_n & ck_ok & src_ok & dst_ok & Bit::from(tl > 20) & !last;
            let dec_icmp = p19 & accept & to_icmp;
            let dec_udp = p19 & accept & to_udp;
            let dec_fwd = dec_icmp | dec_udp;

            // Payload phase.
            let fwd = (f_icmp | f_udp) & !sw;
            let is_end = Bit::from(pos + U::<16>::from(1u16) == tl);
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

            let ip_meta = IpMeta {
                src,
                dst: dst_fin,
                proto,
                len: tl - U::<16>::from(20u16),
                eth_src: self.esrc.get(),
                ttl: self.ttl.get(),
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

            // ---- formal properties (orchestrator; docs/formal.md, rows "IPv4" and "Flux") ----
            assume!(!have | !b.err | last, "payload: err only together with last (stream contract 2)");
            assume!(!(have & Bit::from(pos >= 1517)) | last, "a frame has at most 1518 bytes (stream contract 3)");
            assume!(!self.m_lv.get() | Bit::from(local_ip.get() == self.m_lip.get()), "local_ip is constant (configuration wire)");
            let m_any = m_icmp | m_udp;
            let m_dec = take & dec_fwd;
            let p_any = p_icmp | p_udp;
            let m_cnt = self.m_cnt.get();
            let fwd_any = f_icmp | f_udp;
            let src_hi = src >> 24u32;
            // The verdict, restated from the raw bytes (independent of `ok`, `src_ok_r`, `accept`).
            check!(!m_any | p19, "a packet is forwarded on byte 20 of the header only");
            check!(!m_any | (Bit::from(self.m_b0.get() == 0x45u8) & !self.m_mc.get() & !self.esrc.get().bit(40)), "version 4, IHL 5, Ethernet destination and source not group addressed (I0, I1, E1)");
            check!(!m_any | (Bit::from((self.m_b6.get() & 0x3Fu8) == 0u8) & Bit::from(self.m_b7.get() == 0u8)), "no fragment: MF clear and offset 0 (I3)");
            check!(!m_any | !Bit::from(self.ttl.get() == 0u8), "TTL not 0 (I4)");
            check!(!m_any | Bit::from(tl > 20), "total length above 20 (I1)");
            check!(!m_any | Bit::from(dst_fin == local_ip.get()), "destination is our address (I5)");
            check!(!m_any | (!Bit::from(src_hi == 0u8) & !Bit::from(src_hi == 127u8) & Bit::from(src_hi < 224u8) & !Bit::from(src == local_ip.get())), "source is a valid unicast host address (I6)");
            check!(!m_any | !last, "a packet has a byte after the header");
            check!(!m_icmp | Bit::from(proto == 1u8), "ICMP meta only for protocol 1");
            check!(!m_udp | Bit::from(proto == 17u8), "UDP meta only for protocol 17");
            check!(!(m_icmp & m_udp), "a packet goes to one protocol");
            check!(!(f_icmp & f_udp), "forwarding to one protocol at a time");
            check!(!(m_any & fwd_any), "a new packet is not decided while one is being forwarded");
            // Invariants tying `ok` to the raw bytes (the induction needs them).
            check!(!(ok & Bit::from(pos >= 1)) | (Bit::from(self.m_b0.get() == 0x45u8) & !self.m_mc.get() & !self.esrc.get().bit(40)), "ok: byte 0 and the group bits were good");
            check!(!(ok & Bit::from(pos >= 7)) | Bit::from((self.m_b6.get() & 0x3Fu8) == 0u8), "ok: flags and offset high were good");
            check!(!(ok & Bit::from(pos >= 8)) | Bit::from(self.m_b7.get() == 0u8), "ok: offset low was good");
            check!(!(ok & Bit::from(pos >= 9)) | !Bit::from(self.ttl.get() == 0u8), "ok: TTL was not 0");
            // Payload.
            check!(!p_any | fwd_any, "a payload byte is sent only for a forwarded packet");
            check!(!p_icmp | f_icmp, "ICMP payload only on the ICMP channel");
            check!(!p_udp | f_udp, "UDP payload only on the UDP channel");
            check!(!(p_icmp & p_udp), "one channel per byte");
            check!(!p_any | !sw | last, "while the final byte is held only it is sent, with the frame's last");
            check!(!(p_any & o_last) | last, "the packet ends with the frame (last of the input)");
            check!(!(p_any & o_err) | o_last, "err only together with last (stream contract 2)");
            check!(!(p_any & o_last & !o_err) | Bit::from(m_cnt + 1 == tl - 20), "a good packet carries exactly total length - 20 payload bytes");
            check!(!(fwd_any & !sw) | (Bit::from(pos >= 20) & Bit::from(pos < tl)), "forwarding: past the header and before the end");
            check!(!(fwd_any & !sw) | Bit::from(m_cnt + 20 == pos), "forwarding: the byte count follows the position");
            check!(!sw | Bit::from(m_cnt + 21 == tl), "final byte held: all others were sent");
            check!(!sw | fwd_any, "a held byte belongs to a forwarded packet");
            check!(!sw | (Bit::from(pos >= tl) & Bit::from(tl > 20)), "final byte held: the position is at or past the total length");
            cover!(m_icmp, "an ICMP packet is forwarded");
            cover!(m_udp, "a UDP packet is forwarded");
            cover!(take & p19 & !accept & !last, "a header is rejected on byte 20");
            cover!(send_held, "a held final byte is sent after the padding");
            cover!(p_any & o_last & o_err & !b.err, "a packet longer than its frame is aborted");
            cover!(p_any & o_last & !o_err, "a good packet is completed");

            with!(self <= {
                src_ok_r: src_ok_n,
                take ? pos: pos + U::<16>::from(1u16),
                take ? ok: ok_n,
                take & p2 ? tl: b.data.zext::<16>() << 8u32,
                take & p3 ? tl: tl | b.data.zext::<16>(),
                take & p8 ? ttl: b.data,
                take & p9 ? proto: b.data,
                take & p0 ? {
                    esrc: m.src,
                    ets: m.ts,
                    m_b0: b.data,
                    m_mc: m.multicast,
                },
                take & p6 ? m_b6: b.data,
                take & p7 ? m_b7: b.data,
                m_lip: local_ip.get(),
                m_lv: Bit::One,
                m_dec ? m_cnt: U::<16>::from(0u16),
                p_any ? m_cnt: m_cnt + U::<16>::from(1u16),
                take & in_src ? src: (src << 8u32) | b.data.zext::<32>(),
                take & in_dst ? dst: dst_fin,
                take & in_h & !odd ? hi: b.data,
                take & in_h & odd ? acc: sum_fin,
                take & dec_fwd ? {
                    f_icmp: dec_icmp,
                    f_udp: dec_udp,
                },
                take & hold ? {
                    sw: Bit::One,
                    held: b.data,
                },
                take & last ? {
                    pos: U::<16>::from(0u16),
                    acc: U::<32>::from(0u32),
                    f_icmp: Bit::Zero,
                    f_udp: Bit::Zero,
                    sw: Bit::Zero,
                },
            });
        }
    }
}
