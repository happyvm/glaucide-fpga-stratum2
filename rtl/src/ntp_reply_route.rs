// SPDX-License-Identifier: Apache-2.0
//! NTP reply router (task T1.11), domain `DefaultClock` (`clk_sys`,
//! 100 MHz): the adapter from the single reply output of `ntp_server`
//! (`NtpReplyMeta` + 48 bytes) to the UDP transmitters of the two
//! families, `udp4_tx` (`UdpTxReq`) and `udp6_tx` (`Udp6TxReq`).
//!
//! # Ports (flat, in this order)
//!
//! Inputs: `meta` (`Rx<NtpReplyMeta>`, the server's `reply_meta`) and
//! `reply` (`Rx<FrameByte>`, the server's `reply`). Outputs: `req4`
//! (`Tx<UdpTxReq>`), `pay4` (`Tx<FrameByte>`) towards `udp4_tx` (`req`,
//! `payload`); `req6` (`Tx<Udp6TxReq>`), `pay6` (`Tx<FrameByte>`) towards
//! `udp6_tx`; `tx_latency` (`Out<U<64>>`, to the server's `tx_latency`).
//!
//! # Behaviour
//!
//! * IDLE: when `meta` holds a reply meta and the request channel of its
//!   family (`family` 0: `req4`, 1: `req6`) has room, the meta is popped and
//!   converted in the same cycle; the unit enters PAY for that family.
//!   * IPv4: `UdpTxReq { dst_mac, dst_ip = dst_ip[31:0], sport, dport, len }`
//!     (`udp4_tx` takes the source address from its `local_ip` wire, which
//!     is the address `ipv4_rx` accepted the request on, so `src_ip` of the
//!     meta is equal to it and not needed);
//!   * IPv6: `Udp6TxReq { dst_mac, dst_ip, src_ip, sport, dport, len }`
//!     (we answer from the address the request was sent to).
//! * PAY: the reply bytes are copied to the payload channel of the family,
//!   one per cycle when present and the channel has room, `last`/`err`
//!   unchanged. After the byte with `last` the unit is in IDLE the next
//!   cycle. The other family's channels are not touched.
//! * `tx_latency` is a register: `TX_LATENCY_V6` if the last meta taken was
//!   IPv6, `TX_LATENCY_V4` otherwise (IPv4 after reset). The server samples
//!   `tx_latency` in cycle S - 1 (S = the cycle reply byte 40 is sent);
//!   since byte 40 cannot pass this unit before its meta was taken (the
//!   bytes queue behind the meta: the server's 2-deep `reply` channel fills
//!   long before byte 40), the value seen at S - 1 is that of the reply in
//!   progress whenever the server had to wait for this unit; when it did
//!   not, the meta was popped at least ~40 cycles before byte 40 (one
//!   channel stage after the server's META cycle). In both cases the right
//!   family's constant is used.
//!
//! # Contract
//!
//! Input: the server's (one meta, then exactly one 48-byte reply ending
//! with `last`). Output: per family, one request then exactly the reply
//! bytes, request strictly first: what `udp4_tx`/`udp6_tx` need ("a
//! producer must send `req` before the payload").
//!
//! # Integration
//!
//! Registered channels on every port (never port to port).
use crate::ntp_server::NtpReplyMeta;
use crate::stream::FrameByte;
use crate::udp4_tx::UdpTxReq;
use crate::udp6_tx::Udp6TxReq;
use txhdl::comp::{mux, Clock, DefaultClock, Out, Reg, Rx, Tx, Unit};
use txhdl::types::{Bit, U};
use txhdl::{lower, with, Trace};

/// Transmit latency K of an IPv4 reply in this top, picoseconds: from the
/// server's sampling of `time` (cycle S - 1, `ntp_server.rs`) to the
/// `ClkTx` edge where `mii_tx` puts the SFD's 0xD nibble on `txd`, for a
/// reply that finds the transmit path idle. It is the `clk_sys` part of the
/// path (`ntp_server` -> route -> `udp4_tx` -> `IpArb4` -> `ipv4_tx` ->
/// `tx_arb` -> `eth_tx` -> `tx_path`, registered channels in between, about
/// 110 cycles) plus `tx_path`'s release and preamble (770..810 ns, its
/// header). Calibrated with `rtl/tests/top.rs`
/// (`t3_error_at_mii_granularity`, 25 MHz MII clocks at five phases): with
/// this value the T3 error against the SFD instant is -10..+10 ns
/// (the first estimate, 1.920 us, gave +25..+45 ns). The PHY's own
/// latency is not included (to be measured on the board, T5.4).
pub const TX_LATENCY_PS_V4: u64 = 1_885_000;
/// Same for IPv6: 20 more header bytes in `ipv6_tx`, 20 `clk_sys` cycles
/// (the same test measures the same error window for both families).
pub const TX_LATENCY_PS_V6: u64 = TX_LATENCY_PS_V4 + 200_000;

/// Picoseconds to NTP 2^-32 s units, rounded to nearest.
pub const fn ps_to_ntp(ps: u64) -> u64 {
    ((ps as u128 * (1u128 << 32) + 500_000_000_000) / 1_000_000_000_000) as u64
}

/// `tx_latency` of an IPv4 reply in this top, 2^-32 s units.
pub const TX_LATENCY_V4: u64 = ps_to_ntp(TX_LATENCY_PS_V4);
/// `tx_latency` of an IPv6 reply in this top, 2^-32 s units.
pub const TX_LATENCY_V6: u64 = ps_to_ntp(TX_LATENCY_PS_V6);

const IDLE: u8 = 0;
const PAY: u8 = 1;

#[derive(Trace, Default)]
pub struct NtpReplyRoute {
    /// IDLE or PAY.
    pub st: Reg<U<1>>,
    /// Family of the reply in progress / of the last meta taken (1 = IPv6).
    pub fam: Reg<Bit>,
}

#[lower]
impl
    Unit<
        (Rx<NtpReplyMeta>, Rx<FrameByte>),
        (
            Tx<UdpTxReq>,
            Tx<FrameByte>,
            Tx<Udp6TxReq>,
            Tx<FrameByte>,
            Out<U<64>>,
        ),
    > for NtpReplyRoute
{
    async fn run(
        &mut self,
        (meta, reply): (Rx<NtpReplyMeta>, Rx<FrameByte>),
        (req4, pay4, req6, pay6, tx_latency): (
            Tx<UdpTxReq>,
            Tx<FrameByte>,
            Tx<Udp6TxReq>,
            Tx<FrameByte>,
            Out<U<64>>,
        ),
    ) {
        loop {
            DefaultClock::rising().await;
            let is_idle = Bit::from(self.st.get() == IDLE);
            let is_pay = Bit::from(self.st.get() == PAY);
            let fam = self.fam.get();
            tx_latency.set(mux(
                fam,
                U::<64>::from(TX_LATENCY_V6),
                U::<64>::from(TX_LATENCY_V4),
            ));

            let m = meta.head();
            let have = Bit::from(meta.peek().is_some());
            let room = mux(m.family, req6.ready(), req4.ready());
            let take = is_idle & have & room;
            let _ = meta.recv_if(take);
            let r4 = UdpTxReq {
                dst_mac: m.dst_mac,
                dst_ip: m.dst_ip.slice::<0, 32>(),
                sport: m.sport,
                dport: m.dport,
                len: m.len,
            };
            let r6 = Udp6TxReq {
                dst_mac: m.dst_mac,
                dst_ip: m.dst_ip,
                src_ip: m.src_ip,
                sport: m.sport,
                dport: m.dport,
                len: m.len,
            };
            if (take & !m.family).to_bool() {
                req4.send(r4);
            }
            if (take & m.family).to_bool() {
                req6.send(r6);
            }

            let b = reply.head();
            let bv = Bit::from(reply.peek().is_some());
            let proom = mux(fam, pay6.ready(), pay4.ready());
            let go = is_pay & bv & proom;
            let _ = reply.recv_if(go);
            if (go & !fam).to_bool() {
                pay4.send(b);
            }
            if (go & fam).to_bool() {
                pay6.send(b);
            }
            let end = go & b.last;
            with!(self <= {
                take ? { st: U::<1>::from(PAY), fam: m.family },
                end ? st: U::<1>::from(IDLE),
            });
        }
    }
}
