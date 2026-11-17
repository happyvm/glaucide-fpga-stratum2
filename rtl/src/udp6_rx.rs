// SPDX-License-Identifier: Apache-2.0
//! UDP over IPv6, receive side (task T3.4), domain `DefaultClock` (`clk_sys`,
//! 100 MHz). Mirror of `udp4_rx`; the differences are the mandatory
//! checksum and the 128-bit pseudo-header.
//!
//! Ports (flat): `meta` (`Rx<Ip6Meta>`) and `payload` (`Rx<FrameByte>`) are
//! the UDP outputs of `ipv6_rx` (`udp_meta`, `udp`: the UDP datagram, header
//! included, truncated to the IPv6 payload length); outputs `ntp_meta`
//! (`Tx<Meta>`) and `ntp` (`Tx<FrameByte>`) feed the NTP server.
//!
//! # Behaviour (rules of `tools/golden/responder.py`, `udp.py`)
//!
//! The datagram is read one byte per cycle; `meta` is popped together with
//! byte 0 (that byte is not taken before the meta is visible). The 8 header
//! bytes (sport, dport, length, checksum) are parsed on the fly. On **byte
//! 5** (UDP length known, destination port since byte 3) the unit decides
//! whether the datagram can still be forwarded, on **byte 7** (checksum
//! field known) it takes the final decision, before any payload byte is
//! forwarded. The datagram is forwarded only if all of these hold:
//!
//! * destination port is 123 (any other port: dropped, nothing emitted);
//! * the UDP length field equals `Ip6Meta.len` (a shorter or longer field is
//!   dropped at once, byte 5) and is greater than 8: below 8 is malformed
//!   (RFC 768), exactly 8 means an empty payload, which the stream contract
//!   (at least one byte per frame) cannot carry, so it is dropped as well;
//! * the checksum field is not 0 (RFC 8200 section 8.1: the UDP checksum is
//!   mandatory over IPv6, zero is illegal): dropped at once, on byte 7;
//! * the input does not end before byte 8 (a `last` anywhere in the header,
//!   with or without `err`, drops the datagram silently).
//!
//! A dropped datagram emits nothing on any channel; its remaining bytes are
//! swallowed up to `last`. One `meta` is popped for every datagram, dropped
//! or not. As in `udp4_rx`, a UDP length field smaller than the IP payload
//! is dropped (project owner's decision).
//!
//! # Outputs
//!
//! For a forwarded datagram: one [`Meta`] is sent in the cycle byte 7 is
//! taken (so before the first payload byte): `family = 1` (IPv6), `src`,
//! `dst` = `Ip6Meta.src`, `dst` (128 bits), `sport`, `dport` from the UDP
//! header, `len` = UDP length - 8 = the number of payload bytes that follow
//! (1 to 1452), `eth_src` from `Ip6Meta`, `rx_ts` copied from
//! `Ip6Meta.rx_ts`. Then the UDP payload (header stripped) is sent on
//! `ntp`, one byte per cycle at best, in the cycle its input byte is taken,
//! `last` on the final byte. `meta` and `payload` are separate channels: a
//! consumer must not make popping `ntp_meta` depend on progress of `ntp`.
//!
//! # UDP checksum and abort by late `err`
//!
//! The RFC 1071 sum of the pseudo-header (RFC 8200 section 8.1, `pseudo_v6`
//! of `csum.rs`: source, destination, UDP length as 32 bits, next header
//! 17, taken from `Ip6Meta`) plus the whole datagram, checksum field
//! included, must fold to `0xFFFF`. The sum is accumulated while the bytes
//! stream through (cut-through); the verdict is only known at the last
//! byte. If it is bad, the **final** payload byte is sent with `last & err`.
//! Consumers (the NTP server) must drop a payload that ends with `err`: a
//! payload is valid only once its `last` arrived without `err`. `err` is
//! set only together with `last`. It is also set on `last` when
//!
//! * the input `last` carries `err` (bad FCS from below): copied;
//! * the datagram ends early (input `last` before byte `len - 1` after the
//!   meta was sent): that byte is the last one and carries `err`;
//! * the input continues past `len` bytes (not expected from `ipv6_rx`): the
//!   `len`-th byte is sent with `last & err` and the rest is swallowed.
//!
//! The pseudo-header sum is not computed in one cycle (19 chained 32-bit
//! additions): it is spread over the header bytes, see [`words8_into`]
//! (byte 0: source, byte 2: destination, byte 4: length and next header;
//! each address is summed as a balanced tree, 4 additions deep). The
//! checksum verdict of a forwarded byte is computed from `acc` alone (the
//! pseudo-header pieces are only added in the header), so the chain from
//! the meta channel never reaches the payload output. This adds no latency: the forwarded
//! datagram timing is that of `udp4_rx`.
//!
//! # Integration
//!
//! Inputs and outputs must be registered channels or FIFOs (never port to
//! port, see the netlist note of `stream.rs`). An input byte is taken
//! exactly when the output it feeds has room (byte 7 waits for room on
//! `ntp_meta`, payload bytes for room on `ntp`). `ntp_meta` must be popped
//! by the consumer independently of the payload.
use crate::csum::{csum_add, csum_ok, sum8};
use crate::ipv6_rx::Ip6Meta;
use crate::stream::{FrameByte, Meta};
use txhdl::comp::{mux, Clock, DefaultClock, Reg, Rx, Tx, Unit};
use txhdl::types::{Bit, U};
use txhdl::{assume, check, cover, lower, with, Trace};

/// The only UDP port served.
pub const NTP_PORT: u16 = 123;
/// Next header value of UDP.
pub const PROTO_UDP: u8 = 17;

/// Sums the eight 16-bit words of a 128-bit address into `acc`: `acc` plus
/// the balanced tree [`sum8`] (4 additions deep instead of a chain of 8,
/// task T5.3; same integer, no partial sum wraps). Shared with `udp6_tx`.
#[lower]
pub fn words8_into(acc: U<32>, x: U<128>) -> U<32> {
    acc + sum8(x)
}

const HDR: u8 = 0;
const FWD: u8 = 1;
const DROP: u8 = 2;

#[derive(Trace, Default)]
pub struct Udp6Rx {
    /// HDR (bytes 0-7), FWD (payload being forwarded), DROP (swallow).
    pub st: Reg<U<2>>,
    /// Bytes of the datagram taken so far.
    pub pos: Reg<U<16>>,
    /// Running sum (not folded) and high byte of the word in progress.
    pub acc: Reg<U<32>>,
    pub hi: Reg<U<8>>,
    pub mlen: Reg<U<16>>,
    pub msrc: Reg<U<128>>,
    pub mdst: Reg<U<128>>,
    pub esrc: Reg<U<48>>,
    /// Receive timestamp of the datagram's frame (from `Ip6Meta.rx_ts`).
    pub ets: Reg<U<64>>,
    pub sport: Reg<U<16>>,
    pub dport: Reg<U<16>>,
    // Monitors (check!/cover! only): the UDP length field, and the payload bytes sent.
    pub m_ulen: Reg<U<16>>,
    pub m_cnt: Reg<U<16>>,
}

#[lower]
impl Unit<(Rx<Ip6Meta>, Rx<FrameByte>), (Tx<Meta>, Tx<FrameByte>)> for Udp6Rx {
    async fn run(
        &mut self,
        (meta, payload): (Rx<Ip6Meta>, Rx<FrameByte>),
        (ntp_meta, ntp): (Tx<Meta>, Tx<FrameByte>),
    ) {
        loop {
            DefaultClock::rising().await;
            let st = self.st.get();
            let pos = self.pos.get();
            let acc = self.acc.get();
            let hi = self.hi.get();
            let mlen = self.mlen.get();
            let is_h = Bit::from(st == HDR);
            let is_f = Bit::from(st == FWD);
            let is_d = Bit::from(st == DROP);
            // In HDR `pos` is at most 7 (HDR is left at byte 7, or `pos`
            // returns to 0 on a `last`): its low 3 bits decode the header
            // position (shorter than a 16-bit compare, task T5.3).
            let ph = pos.slice::<0, 3>();
            let p0 = is_h & Bit::from(ph == 0);
            let p1 = is_h & Bit::from(ph == 1);
            let p2 = is_h & Bit::from(ph == 2);
            let p3 = is_h & Bit::from(ph == 3);
            let p4 = is_h & Bit::from(ph == 4);
            let p5 = is_h & Bit::from(ph == 5);
            let p7 = is_h & Bit::from(ph == 7);

            let have = Bit::from(payload.peek().is_some());
            let b = payload.head();
            let last = b.last;
            let m = meta.head();
            let mv = Bit::from(meta.peek().is_some());

            let take = have & (!p0 | mv) & (!p7 | ntp_meta.ready()) & (!is_f | ntp.ready());
            let _ = payload.recv_if(take);
            let _ = meta.recv_if(take & p0);

            // Decision on byte 5 (length, port) and byte 7 (checksum != 0).
            let w = (hi.zext::<16>() << 8u32) | b.data.zext::<16>();
            let accept = Bit::from(w == mlen)
                & Bit::from(w > U::<16>::from(8u16))
                & Bit::from(self.dport.get() == U::<16>::from(NTP_PORT))
                & !last;
            let nock = Bit::from(w == U::<16>::from(0u16));

            // Payload byte bookkeeping (state FWD).
            let end_len = Bit::from(pos + U::<16>::from(1u16) == mlen);
            let endb = last | (is_f & end_len);

            // Running sum. The pseudo-header is added in pieces on the even
            // header positions 0 (source), 2 (destination), 4 (length and
            // next header); words of the datagram on odd positions and on
            // the final byte.
            let zero = U::<32>::from(0u32);
            let ext = mux(
                p0,
                words8_into(zero, m.src),
                mux(
                    p2,
                    words8_into(zero, self.mdst.get()),
                    mux(
                        p4,
                        mlen.zext::<32>() + U::<32>::from(PROTO_UDP as u32),
                        zero,
                    ),
                ),
            );
            let base = mux(p0, zero, acc);
            let odd = pos.bit(0);
            let lo = b.data.zext::<16>();
            let word = mux(odd, (hi.zext::<16>() << 8u32) | lo, lo << 8u32);
            let add = odd | endb;
            let wadd = mux(add, word, U::<16>::from(0u16));
            let next = csum_add(base + ext, wadd);
            // The verdict is only used on a forwarded byte (state FWD), where
            // `ext` is 0 and `base` is `acc`: it is computed from `acc`
            // directly, which keeps the pseudo-header additions off its path
            // (task T5.3; same value wherever it is used).
            let ck_good = csum_ok(csum_add(acc, wadd));

            if (take & p7 & !last & !nock).to_bool() {
                ntp_meta.send(Meta {
                    family: Bit::One,
                    src: self.msrc.get(),
                    dst: self.mdst.get(),
                    sport: self.sport.get(),
                    dport: self.dport.get(),
                    len: mlen - U::<16>::from(8u16),
                    eth_src: self.esrc.get(),
                    rx_ts: self.ets.get(),
                });
            }
            let err_o = endb & ((last & b.err) | (last ^ end_len) | !ck_good);
            if (take & is_f).to_bool() {
                ntp.send(FrameByte {
                    data: b.data,
                    last: endb,
                    err: err_o,
                });
            }

            // ---- formal properties (orchestrator; docs/formal.md, row "UDP") ----
            assume!(!have | !b.err | last, "payload: err only together with last (stream contract 2)");
            assume!(!(have & Bit::from(pos >= 1517)) | last, "a frame has at most 1518 bytes (stream contract 3)");
            let m_ulen = self.m_ulen.get();
            let m_cnt = self.m_cnt.get();
            let dport = self.dport.get();
            let meta_go = take & p7 & !last & !nock;
            let pay_go = take & is_f;
            // Header phase: the decision (byte 5) and what it implies for bytes 6 and 7.
            check!(!is_h | Bit::from(pos < 8), "the header phase lasts 8 bytes");
            check!(!(is_h & Bit::from(pos >= 6)) | (Bit::from(m_ulen == mlen) & Bit::from(mlen > 8) & Bit::from(dport == 123u16)), "accepted on byte 5: destination port 123, UDP length = IP length > 8 (U2)");
            check!(!meta_go | (Bit::from(dport == 123u16) & Bit::from(m_ulen == mlen) & Bit::from(mlen > 8)), "a datagram is forwarded only to port 123, with UDP length = IP length > 8");
            check!(!meta_go | Bit::from(mlen - 8 >= 1), "forwarded payload length is at least 1");
            check!(!(take & p7 & last) | !meta_go, "a datagram ending inside its header emits nothing");
            check!(!meta_go | Bit::from(hi != 0u8) | Bit::from(b.data != 0u8), "a zero UDP checksum is never forwarded over IPv6 (RFC 8200 8.1)");
            // Forwarding.
            check!(!is_f | (Bit::from(m_ulen == mlen) & Bit::from(mlen > 8) & Bit::from(dport == 123u16)), "forwarding implies the accepted header");
            check!(!is_f | (Bit::from(pos >= 8) & Bit::from(pos < mlen)), "forwarding: past the header and before the end");
            check!(!is_f | Bit::from(m_cnt + 8 == pos), "forwarding: the byte count follows the position");
            check!(!pay_go | is_f, "a payload byte is sent only in the forwarding state");
            check!(!(pay_go & endb & !err_o) | Bit::from(m_cnt + 1 == mlen - 8), "a datagram that ends without err carries exactly UDP length - 8 payload bytes");
            check!(!(pay_go & err_o) | endb, "err only together with last (stream contract 2)");
            check!(!(pay_go & endb) | (last | end_len), "the datagram ends at its last byte or at the announced length");
            check!(!is_d | !(meta_go | pay_go), "a dropped datagram emits nothing");
            cover!(meta_go, "a datagram is forwarded");
            cover!(take & p5 & !accept & !last, "a datagram is rejected on byte 5");
            cover!(pay_go & endb & !last, "a longer-than-announced input is cut at the UDP length");
            cover!(pay_go & endb & last & Bit::from(end_len), "a datagram ends at the right byte");
            cover!(pay_go & last & !end_len, "a datagram ends early");

            with!(self <= {
                take ? {
                    pos: pos + U::<16>::from(1u16),
                    hi: b.data,
                },
                take & !is_d ? acc: next,
                take & p0 ? {
                    esrc: m.eth_src,
                    ets: m.rx_ts,
                    msrc: m.src,
                    mdst: m.dst,
                    mlen: m.len,
                },
                take & p1 ? sport: w,
                take & p3 ? dport: w,
                take & p5 ? m_ulen: w,
                meta_go ? m_cnt: U::<16>::from(0u16),
                pay_go ? m_cnt: m_cnt + U::<16>::from(1u16),
                take & p5 & !accept ? st: U::<2>::from(DROP),
                take & p7 & !last ? st: mux(nock, U::<2>::from(DROP), U::<2>::from(FWD)),
                take & is_f & endb ? st: U::<2>::from(DROP),
                take & last ? {
                    st: U::<2>::from(HDR),
                    pos: U::<16>::from(0u16),
                },
            });
        }
    }
}
