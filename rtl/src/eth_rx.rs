// SPDX-License-Identifier: Apache-2.0
//! Ethernet receive filter and classifier (task T1.5), domain `DefaultClock`
//! (`clk_sys`, 100 MHz).
//!
//! Ports: `frames` (`Rx<FrameByte>`: a frame without FCS as `fcs_check`
//! (T1.12) produces it, destination MAC through last payload byte, `err` on
//! `last` = bad FCS or bad frame), `local_mac` and `sn_low` (wires),
//! `stamps` (`Rx<Ts>`, the `stamps_out` of `fcs_check`: one receive
//! timestamp per input frame, in frame order), and six output channels, in this order: ARP meta, ARP payload, IPv4 meta,
//! IPv4 payload, IPv6 meta, IPv6 payload (the nested pair form is not
//! accepted by `#[lower]`: ports must be names).
//!
//! # Receive timestamp (T1.13)
//!
//! Exactly one stamp is popped from `stamps` per input frame, in the cycle
//! its first byte (byte 0 of the header) is taken, for every frame: accepted,
//! dropped, or too short to reach byte 14. A forwarded frame carries its own
//! stamp in [`EthMeta::ts`]. `fcs_check` forwards a stamp exactly for the
//! frames it emits, so stamps and frames stay aligned. The stamp is expected
//! with the first byte; if it is late the first byte waits (the input
//! stalls), and if it never comes the unit waits forever.
//!
//! # Conventions
//!
//! * MACs are `U<48>` with byte 0 (the first byte on the wire) in the most
//!   significant byte: `01:23:45:67:89:ab` is `0x0123_4567_89ab`. This holds
//!   for `local_mac`, [`EthMeta::dst`] and [`EthMeta::src`].
//! * `sn_low` is the low 24 bits of the IPv6 unicast address we own; the
//!   solicited-node group is `33:33:ff:` followed by those 24 bits
//!   (RFC 4291 2.7.1, RFC 2464 7).
//! * [`EthMeta::ethertype`] is big endian as read: `0x86DD` for IPv6.
//!
//! # Behaviour
//!
//! The 14-byte header is parsed on the fly (dst 6, src 6, EtherType 2). On
//! its last byte the frame is accepted or dropped:
//!
//! | EtherType | accepted destinations                                         |
//! |-----------|----------------------------------------------------------------|
//! | `0x0806` ARP  | `local_mac`, `ff:ff:ff:ff:ff:ff`                            |
//! | `0x0800` IPv4 | `local_mac`, `ff:ff:ff:ff:ff:ff`                            |
//! | `0x86DD` IPv6 | `local_mac`, `33:33:ff:<sn_low>`, `33:33:00:00:00:01`       |
//!
//! Everything else (other destination, other EtherType including `0x8100`
//! VLAN, IPv6 to broadcast, IPv4/ARP to multicast) is dropped silently.
//!
//! For an accepted frame, on the channels of its class: one [`EthMeta`] on
//! the meta channel, sent in the cycle the 14th header byte is taken, hence
//! strictly before the first payload byte; then the payload (everything after
//! the header, padding included: upper layers use the IP length) as
//! `FrameByte`, `last` on the final byte, `err` copied from the input `last`.
//! The meta and the payload are on separate channels: a consumer that pops
//! the meta late may see the first payload byte first, so it must not make
//! popping the meta depend on payload progress.
//! `multicast` is the group bit (bit 0 of byte 0) of the destination, so it is
//! set for broadcast too.
//!
//! # Cut-through and drops, precisely
//!
//! Payload bytes flow without buffering: one input byte is taken exactly when
//! the selected output accepts it (otherwise the input stalls). The decision
//! is made on the 14th byte, before any payload byte is forwarded, so:
//!
//! * a dropped frame (destination, EtherType) emits nothing on any channel;
//! * a frame of 14 bytes or fewer (runt, or empty payload) emits nothing: the
//!   meta is sent only when a payload byte is known to follow (14th byte not
//!   `last`);
//! * an accepted frame whose input `last` carries `err` has its payload
//!   forwarded with `err` on its last byte, and consumers discard it. There
//!   is no other case where a forwarded payload ends with `err`: the unit
//!   never drops a frame after having forwarded a byte.
//!
//! A frame being forwarded never interleaves with another: the next frame is
//! looked at only after the input `last`, since one byte is taken per cycle in
//! order and the state returns to "header, byte 0" on `last`. A frame is not
//! limited in length here.
//!
//! # Integration
//!
//! Input: `frames` must come through a registered channel (never wired port to
//! port with `fcs_check`, see the netlist note of `stream.rs`). Outputs: each
//! `Tx` must feed a registered channel or FIFO. The configuration wires are
//! sampled in the cycle of the 14th byte only (not held per frame): they must
//! be stable during a frame to give a meaningful result.
//!
//! # Formal
//!
//! `check!`/`cover!` in `run` state the Ethernet row of `docs/formal.md`
//! (proved with `tools/formal/prove.sh eth_rx prove`). The only `assume!` is
//! the input stream contract (`err` only together with `last`) and the
//! configuration (`local_mac` has its group bit clear, i.e. is a unicast
//! address).
use crate::stream::{FrameByte, Ts};
use txhdl::comp::{Clock, DefaultClock, In, Reg, Rx, Tx, Unit};
use txhdl::types::{Bit, U};
use txhdl::{
    assume, check, cover, lower, with, Trace, Transaction as TransactionDerive,
    Value as ValueDerive,
};

/// EtherType of ARP.
pub const ET_ARP: u16 = 0x0806;
/// EtherType of IPv4.
pub const ET_IPV4: u16 = 0x0800;
/// EtherType of IPv6.
pub const ET_IPV6: u16 = 0x86DD;

/// Header of an accepted frame, one per frame, before its first payload byte.
#[derive(TransactionDerive, ValueDerive, Clone, Copy, Default, Debug)]
pub struct EthMeta {
    /// Destination MAC, first wire byte in the most significant byte.
    pub dst: U<48>,
    /// Source MAC, same layout.
    pub src: U<48>,
    pub ethertype: U<16>,
    /// Group bit of the destination (multicast or broadcast).
    pub multicast: Bit,
    /// Receive timestamp of the frame (NTP 32.32), popped from `stamps`.
    pub ts: U<64>,
}

#[derive(Trace, Default)]
pub struct EthRx {
    /// Header bytes taken so far in this frame: 0..=13 in the header, 14 once
    /// past it (payload, forwarded or dropped).
    pub cnt: Reg<U<4>>,
    pub dst: Reg<U<48>>,
    pub src: Reg<U<48>>,
    pub et_hi: Reg<U<8>>,
    /// Stamp of the frame in progress, popped with its byte 0.
    pub ts: Reg<Ts>,
    /// The frame is being forwarded to ARP / IPv4 / IPv6 (one-hot or all 0).
    pub f_arp: Reg<Bit>,
    pub f_v4: Reg<Bit>,
    pub f_v6: Reg<Bit>,
    // Monitors (only read by check!/cover!; pruned in synthesis).
    /// A meta was sent for the frame in progress.
    pub m_meta: Reg<Bit>,
    /// EtherType of the frame in progress, once its 14th byte was taken.
    pub m_ety: Reg<U<16>>,
}

#[lower]
impl
    Unit<
        (Rx<FrameByte>, In<U<48>>, In<U<24>>, Rx<Ts>),
        (
            Tx<EthMeta>,
            Tx<FrameByte>,
            Tx<EthMeta>,
            Tx<FrameByte>,
            Tx<EthMeta>,
            Tx<FrameByte>,
        ),
    > for EthRx
{
    async fn run(
        &mut self,
        (frames, local_mac, sn_low): (Rx<FrameByte>, In<U<48>>, In<U<24>>, Rx<Ts>),
        (arp_m, arp_p, v4_m, v4_p, v6_m, v6_p): (
            Tx<EthMeta>,
            Tx<FrameByte>,
            Tx<EthMeta>,
            Tx<FrameByte>,
            Tx<EthMeta>,
            Tx<FrameByte>,
        ),
    ) {
        loop {
            DefaultClock::rising().await;
            let have = frames.peek().is_some();
            let b = frames.head();
            let last = b.last;
            // Stream contract of the input (stream.rs, rule 2).
            assume!(
                !have | !b.err | last,
                "err only together with last (stream contract 2)"
            );

            let cnt = self.cnt.get();
            let dst = self.dst.get();
            let src = self.src.get();
            let f_arp = self.f_arp.get();
            let f_v4 = self.f_v4.get();
            let f_v6 = self.f_v6.get();
            let fwd = f_arp | f_v4 | f_v6;
            let mac = local_mac.get();
            let sn = sn_low.get();
            // Environment: the configured MAC is a unicast address (a group
            // address as `local_mac` would make broadcast a "unicast").
            assume!(
                !mac.bit(40),
                "local_mac is a unicast address (group bit clear)"
            );

            let in_dst = Bit::from(cnt < 6);
            let in_src = Bit::from(cnt >= 6) & Bit::from(cnt < 12);
            let is_hi = Bit::from(cnt == 12);
            let is_dec = Bit::from(cnt == 13);
            let is_pay = Bit::from(cnt == 14);

            // Classification on the 14th byte (valid when `is_dec`).
            let ety = (self.et_hi.get().zext::<16>() << 8u32) | b.data.zext::<16>();
            let uni = Bit::from(dst == mac);
            let bcast = Bit::from(dst == U::<48>::from(0xffff_ffff_ffffu64));
            let all_nodes = Bit::from(dst == U::<48>::from(0x3333_0000_0001u64));
            let sol_node = Bit::from(dst == (U::<48>::from(0x3333_ff00_0000u64) | sn.zext::<48>()));
            let is_arp = Bit::from(ety == ET_ARP);
            let is_v4 = Bit::from(ety == ET_IPV4);
            let is_v6 = Bit::from(ety == ET_IPV6);
            let to_arp = is_arp & (uni | bcast);
            let to_v4 = is_v4 & (uni | bcast);
            let to_v6 = is_v6 & (uni | sol_node | all_nodes);
            // Forward only when a payload byte follows (not `last` on byte 14).
            let dec_arp = is_dec & to_arp & !last;
            let dec_v4 = is_dec & to_v4 & !last;
            let dec_v6 = is_dec & to_v6 & !last;
            let dec_fwd = dec_arp | dec_v4 | dec_v6;

            let meta_room =
                (dec_arp & arp_m.ready()) | (dec_v4 & v4_m.ready()) | (dec_v6 & v6_m.ready());
            let pay_room = (f_arp & arp_p.ready()) | (f_v4 & v4_p.ready()) | (f_v6 & v6_p.ready());
            // One input byte is taken only when its output accepts it.
            // Byte 0 also needs the frame's stamp.
            let is_b0 = Bit::from(cnt == 0);
            let st_ok = !is_b0 | Bit::from(stamps.peek().is_some());
            let take = have & st_ok & (!dec_fwd | meta_room) & (!fwd | pay_room);
            let _ = frames.recv_if(take);
            let new_ts = stamps.recv_if(take & is_b0).unwrap_or_default();

            let meta = EthMeta {
                dst,
                src,
                ethertype: ety,
                multicast: dst.bit(40),
                ts: self.ts.get(),
            };
            let pay = FrameByte {
                data: b.data,
                last,
                err: b.err,
            };
            let m_arp = take & dec_arp;
            let m_v4 = take & dec_v4;
            let m_v6 = take & dec_v6;
            let p_arp = take & f_arp;
            let p_v4 = take & f_v4;
            let p_v6 = take & f_v6;
            if m_arp.to_bool() {
                arp_m.send(meta);
            }
            if m_v4.to_bool() {
                v4_m.send(meta);
            }
            if m_v6.to_bool() {
                v6_m.send(meta);
            }
            if p_arp.to_bool() {
                arp_p.send(pay);
            }
            if p_v4.to_bool() {
                v4_p.send(pay);
            }
            if p_v6.to_bool() {
                v6_p.send(pay);
            }

            // Invariants and properties (IEEE 802.3 3.1.1, RFC 826/791/8200
            // framing, RFC 4291 2.7.1, RFC 2464 7).
            let m_ety = self.m_ety.get();
            let m_meta = self.m_meta.get();
            let sent_meta = m_arp | m_v4 | m_v6;
            let sent_pay = p_arp | p_v4 | p_v6;
            check!(cnt <= 14, "the header counter stays within 0..=14");
            check!(!fwd | is_pay, "a frame is forwarded only after its header");
            check!(
                !(f_arp & f_v4) & !(f_arp & f_v6) & !(f_v4 & f_v6),
                "a frame goes to one class only"
            );
            check!(
                m_meta == fwd,
                "a meta was sent exactly for a forwarded frame"
            );
            check!(
                !f_arp | Bit::from(m_ety == ET_ARP),
                "an ARP frame carries EtherType 0x0806"
            );
            check!(
                !f_v4 | Bit::from(m_ety == ET_IPV4),
                "an IPv4 frame carries EtherType 0x0800"
            );
            check!(
                !f_v6 | Bit::from(m_ety == ET_IPV6),
                "an IPv6 frame carries EtherType 0x86DD"
            );
            // Destination classes, restated on the bytes of the address.
            let dst_ff = Bit::from((dst ^ U::<48>::from(0xffff_ffff_ffffu64)) == 0u8);
            let dst_sn = Bit::from((dst >> 24u32) == 0x3333ffu32)
                & Bit::from((dst & U::<48>::from(0xff_ffffu64)) == sn.zext::<48>());
            let dst_all = Bit::from((dst >> 8u32) == U::<48>::from(0x33_3300_0000u64))
                & Bit::from((dst & U::<48>::from(0xffu64)) == 1u8);
            check!(
                !(m_arp | m_v4) | (Bit::from(dst == mac) | dst_ff),
                "ARP and IPv4 are forwarded only for unicast or broadcast (IEEE 802.3, RFC 826, RFC 791)"
            );
            check!(
                !m_v6 | (Bit::from(dst == mac) | dst_sn | dst_all),
                "IPv6 is forwarded only for unicast, solicited-node or all-nodes (RFC 4291 2.7.1, RFC 2464 7)"
            );
            check!(
                !m_v6 | !dst_ff,
                "a broadcast frame is never forwarded as IPv6 (RFC 2464 7)"
            );
            check!(
                !sent_meta | (!m_meta & !last),
                "one meta per frame, and a payload byte follows it (stream.rs packet metadata)"
            );
            check!(
                !(m_arp & !Bit::from(ety == ET_ARP))
                    & !(m_v4 & !Bit::from(ety == ET_IPV4))
                    & !(m_v6 & !Bit::from(ety == ET_IPV6)),
                "the meta goes to the channel of its EtherType"
            );
            check!(
                !sent_pay | (m_meta & is_pay),
                "the meta precedes the payload; nothing is sent for a dropped frame"
            );
            check!(
                !(sent_meta & sent_pay),
                "a meta and a payload byte are never sent in the same cycle"
            );
            check!(
                !sent_pay | !b.err | last,
                "forwarded err only together with last (stream contract 2)"
            );
            check!(
                !(take & !fwd & !dec_fwd) | !(sent_meta | sent_pay),
                "a frame dropped or still in its header emits nothing"
            );
            cover!(m_arp & uni, "unicast ARP is forwarded");
            cover!(m_arp & bcast, "broadcast ARP is forwarded");
            cover!(m_v4 & uni, "unicast IPv4 is forwarded");
            cover!(m_v4 & bcast, "broadcast IPv4 is forwarded");
            cover!(m_v6 & uni, "unicast IPv6 is forwarded");
            cover!(m_v6 & sol_node & !uni, "solicited-node IPv6 is forwarded");
            cover!(m_v6 & all_nodes & !uni, "all-nodes IPv6 is forwarded");
            cover!(sent_pay & last & !b.err, "a forwarded payload ends clean");
            cover!(sent_pay & last & b.err, "a forwarded payload ends with err");
            cover!(take & is_dec & is_v6 & bcast, "broadcast IPv6 is dropped");
            cover!(
                take & is_dec & (is_arp | is_v4) & !uni & !bcast,
                "foreign or multicast ARP/IPv4 is dropped"
            );
            cover!(
                take & is_dec & !is_arp & !is_v4 & !is_v6,
                "an unknown EtherType is dropped"
            );
            cover!(take & is_dec & to_v6 & last, "an empty payload is dropped");
            cover!(take & last & !is_dec & !is_pay, "a runt frame is dropped");
            cover!(have & !take, "the input stalls on a busy output");

            let n_hdr = take & !is_pay;
            with!(self <= {
                take & in_dst ? dst: (dst << 8u32) | b.data.zext::<48>(),
                take & in_src ? src: (src << 8u32) | b.data.zext::<48>(),
                take & is_hi ? et_hi: b.data,
                take & is_b0 ? ts: new_ts,
                take & is_dec ? m_ety: ety,
                n_hdr ? cnt: cnt + 1,
                take & dec_fwd ? {
                    f_arp: dec_arp,
                    f_v4: dec_v4,
                    f_v6: dec_v6,
                    m_meta: Bit::One,
                },
                take & last ? {
                    cnt: U::<4>::from(0u8),
                    f_arp: Bit::Zero,
                    f_v4: Bit::Zero,
                    f_v6: Bit::Zero,
                    m_meta: Bit::Zero,
                },
            });
        }
    }
}
