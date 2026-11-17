// SPDX-License-Identifier: Apache-2.0
//! UDP over IPv4, receive side (task T2.2), domain `DefaultClock` (`clk_sys`,
//! 100 MHz).
//!
//! Ports (flat): `meta` (`Rx<IpMeta>`) and `payload` (`Rx<FrameByte>`) are
//! the UDP outputs of `ipv4_rx` (`udp_meta`, `udp`: the UDP datagram, header
//! included, truncated to the IPv4 total length); outputs `ntp_meta`
//! (`Tx<Meta>`) and `ntp` (`Tx<FrameByte>`) feed the NTP server.
//!
//! # Behaviour (rules U1, U2 of `tools/golden/responder.py`)
//!
//! The datagram is read one byte per cycle; `meta` is popped together with
//! byte 0 (that byte is not taken before the meta is visible). The 8 header
//! bytes (sport, dport, length, checksum) are parsed on the fly. On **byte
//! 5** (the UDP length is known, the destination port since byte 3) the unit
//! decides, before any byte is forwarded. The datagram is forwarded only if
//! all of these hold:
//!
//! * U2: destination port is 123 (any other port: dropped, nothing emitted);
//! * the UDP length field equals `IpMeta.len` (the datagram is exactly what
//!   the IP layer delivered; a shorter or longer field is dropped at once)
//!   and is greater than 8: a length below 8 is malformed (RFC 768), exactly
//!   8 means an empty payload, which the stream contract (at least one byte
//!   per frame) cannot carry, so it is dropped as well;
//! * the input does not end before byte 8 (a `last` anywhere in the header,
//!   with or without `err`, drops the datagram silently).
//!
//! A dropped datagram emits nothing on any channel; its remaining bytes are
//! swallowed up to `last`. One `meta` is popped for every datagram.
//!
//! Deviation from the Python model, on purpose: `udp.py` accepts a UDP
//! length field smaller than the IP payload and ignores the extra bytes; this
//! unit drops that datagram (project owner's decision).
//!
//! # Outputs
//!
//! For a forwarded datagram: one [`Meta`] is sent in the cycle byte 7 is taken
//! (so before the first payload byte): `family = 0` (IPv4), `src`/`dst` =
//! `IpMeta.src`/`dst` in the low 32 bits (upper bits 0), `sport`, `dport` from
//! the UDP header, `len` = UDP length - 8 = the number of payload bytes that
//! follow (1 to 1472), `eth_src` from `IpMeta`, `rx_ts` = 0 (the NTP server
//! pairs the hardware timestamp with the frame itself). Then the UDP payload
//! (header stripped) is sent on `ntp`, one byte per cycle at best, one byte in
//! the cycle its input byte is taken, `last` on the final byte. `meta` and
//! `payload` are separate channels: a consumer must not make popping the
//! `ntp_meta` depend on progress of `ntp`.
//!
//! # UDP checksum and abort by late `err`
//!
//! A UDP checksum field of 0 means "no checksum" and is accepted over IPv4
//! (RFC 768). Otherwise the RFC 1071 sum of pseudo-header (`pseudo_v4` of
//! `csum.rs`: source, destination, protocol 17, UDP length, taken from
//! `IpMeta`) plus the whole datagram, checksum field included, must fold to
//! `0xFFFF`. The sum is accumulated while the bytes stream through
//! (cut-through); the verdict is only known at the last byte. If the
//! checksum is bad, the **final** payload byte is sent with `last & err`.
//! `tx_path` discards frames ending in `err` downstream, and every consumer
//! of `ntp` (the NTP server) must drop a payload that ends with `err`:
//! a payload is valid only once its `last` arrived without `err`.
//! `err` is set only together with `last`. It is also set on `last`
//! when
//!
//! * the input `last` carries `err` (bad FCS from below): copied;
//! * the datagram ends early (input `last` before byte `len - 1` after the
//!   meta was sent): that byte is the last one and carries `err`;
//! * the input continues past `len` bytes (not expected from `ipv4_rx`):
//!   the `len`-th byte is sent with `last & err` and the rest is swallowed.
//!
//! # Integration
//!
//! Inputs and outputs must be registered channels or FIFOs (never port to
//! port, see the netlist note of `stream.rs`). An input byte is taken exactly
//! when the output it feeds has room (byte 7 waits for room on `ntp_meta`,
//! payload bytes for room on `ntp`). `ntp_meta` must be popped by the
//! consumer independently of the payload.
use crate::csum::{csum_add, csum_fold, pseudo_v4};
use crate::ipv4_rx::IpMeta;
use crate::stream::{FrameByte, Meta};
use txhdl::comp::{mux, Clock, DefaultClock, Reg, Rx, Tx, Unit};
use txhdl::types::{Bit, U};
use txhdl::{lower, with, Trace};

/// The only UDP port served.
pub const NTP_PORT: u16 = 123;
/// IP protocol number of UDP.
pub const PROTO_UDP: u8 = 17;

const HDR: u8 = 0;
const FWD: u8 = 1;
const DROP: u8 = 2;

#[derive(Trace, Default)]
pub struct Udp4Rx {
    /// HDR (bytes 0-7), FWD (payload being forwarded), DROP (swallow).
    pub st: Reg<U<2>>,
    /// Bytes of the datagram taken so far.
    pub pos: Reg<U<16>>,
    /// Running sum (not folded) and high byte of the word in progress.
    pub acc: Reg<U<32>>,
    pub hi: Reg<U<8>>,
    pub mlen: Reg<U<16>>,
    pub msrc: Reg<U<32>>,
    pub mdst: Reg<U<32>>,
    pub esrc: Reg<U<48>>,
    pub sport: Reg<U<16>>,
    pub dport: Reg<U<16>>,
    /// The checksum field is 0: no checksum.
    pub nock: Reg<Bit>,
}

#[lower]
impl Unit<(Rx<IpMeta>, Rx<FrameByte>), (Tx<Meta>, Tx<FrameByte>)> for Udp4Rx {
    async fn run(
        &mut self,
        (meta, payload): (Rx<IpMeta>, Rx<FrameByte>),
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
            let p0 = is_h & Bit::from(pos == 0);
            let p1 = is_h & Bit::from(pos == 1);
            let p3 = is_h & Bit::from(pos == 3);
            let p5 = is_h & Bit::from(pos == 5);
            let p7 = is_h & Bit::from(pos == 7);

            let have = Bit::from(payload.peek().is_some());
            let b = payload.head();
            let last = b.last;
            let m = meta.head();
            let mv = Bit::from(meta.peek().is_some());

            let take = have & (!p0 | mv) & (!p7 | ntp_meta.ready()) & (!is_f | ntp.ready());
            let _ = payload.recv_if(take);
            let _ = meta.recv_if(take & p0);

            // Decision on byte 5.
            let w = (hi.zext::<16>() << 8u32) | b.data.zext::<16>();
            let accept = Bit::from(w == mlen)
                & Bit::from(w > U::<16>::from(8u16))
                & Bit::from(self.dport.get() == U::<16>::from(NTP_PORT))
                & !last;

            // Payload byte bookkeeping (state FWD).
            let end_len = Bit::from(pos + U::<16>::from(1u16) == mlen);
            let endb = last | (is_f & end_len);

            // Running sum: seed (pseudo-header) at byte 0, then words.
            let seed = csum_fold(pseudo_v4(
                m.src,
                m.dst,
                U::<8>::from(PROTO_UDP),
                m.len,
            ));
            let base = mux(p0, seed.zext::<32>(), acc);
            let odd = pos.bit(0);
            let lo = b.data.zext::<16>();
            let word = mux(odd, (hi.zext::<16>() << 8u32) | lo, lo << 8u32);
            let add = odd | endb;
            let next = mux(add, csum_add(base, word), base);
            let sum_ok = Bit::from(csum_fold(next) == U::<16>::from(0xFFFFu16));
            let ck_good = self.nock.get() | sum_ok;

            if (take & p7 & !last).to_bool() {
                ntp_meta.send(Meta {
                    family: Bit::Zero,
                    src: self.msrc.get().zext::<128>(),
                    dst: self.mdst.get().zext::<128>(),
                    sport: self.sport.get(),
                    dport: self.dport.get(),
                    len: mlen - U::<16>::from(8u16),
                    eth_src: self.esrc.get(),
                    rx_ts: U::<64>::from(0u64),
                });
            }
            if (take & is_f).to_bool() {
                ntp.send(FrameByte {
                    data: b.data,
                    last: endb,
                    err: endb & ((last & b.err) | (last ^ end_len) | !ck_good),
                });
            }

            with!(self <= {
                take ? {
                    pos: pos + U::<16>::from(1u16),
                    hi: b.data,
                },
                take & !is_d ? acc: next,
                take & p0 ? {
                    esrc: m.eth_src,
                    msrc: m.src,
                    mdst: m.dst,
                    mlen: m.len,
                },
                take & p1 ? sport: w,
                take & p3 ? dport: w,
                take & p7 ? nock: Bit::from(w == U::<16>::from(0u16)),
                take & p5 & !accept ? st: U::<2>::from(DROP),
                take & p7 & !last ? st: U::<2>::from(FWD),
                take & is_f & endb ? st: U::<2>::from(DROP),
                take & last ? {
                    st: U::<2>::from(HDR),
                    pos: U::<16>::from(0u16),
                },
            });
        }
    }
}
