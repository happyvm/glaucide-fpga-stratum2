// SPDX-License-Identifier: Apache-2.0
//! The stratum-2 core (task T1.14): the server path of [`ArtyCore`] plus the
//! NTP client side, i.e. what [`ArtyBoardWithClient`] runs.
//!
//! [`ArtyCore`]: crate::top::ArtyCore
//! [`ArtyBoardWithClient`]: crate::top::ArtyBoardWithClient
//!
//! # Hierarchy (what differs from `ArtyCore`)
//!
//! ```text
//! ArtyCoreClient<R, T, C, K>
//! ├── ... rx_path, fcs, eth_rx, ipv4_rx, ipv6_rx, icmp4 as in ArtyCore
//! ├── demux4   NtpDemux   udp4_rx NTP pair -> server v4 | merge source 0
//! ├── demux6   NtpDemux   udp6_rx NTP pair -> server v6 | merge source 1
//! ├── cmerge   MetaArb    mode-4 replies of both families -> client
//! ├── client   NtpClient  requests -> uarb4 / uarb6 source 1
//! │                       samples, miss -> sync (ClientSync<C>)
//! │                       ARP lookup/resolve <-> arp; ND lookup -> nbr, resolve -> icmp6
//! ├── nbr      NbrCache   learn <- icmp6.learn, lookup/rsp <-> client, flush = 0
//! ├── arp      Arp        resolve, lookup, lookup_rsp <-> client
//! ├── icmp6    Icmpv6     resolve <- client (NS), learn -> nbr
//! ├── server   NtpServer  tx_latency from lat (route's value + one stage)
//! ├── route    NtpReplyRoute -> uarb4 / uarb6 source 0
//! ├── lat      LatAdj     route.tx_latency + SERVER_ARB_ADJ -> server
//! ├── uarb4    UdpArb4 -> udp4_tx      uarb6  UdpArb6 -> udp6_tx
//! ├── ... udp4_tx, udp6_tx, arb4, arb6, ipv4_tx, ipv6_tx, tx_arb, eth_tx,
//! │       tx_path, cfg as in ArtyCore
//! ├── clock    K: ClockSource (DisciplinedClock on the board)
//! ├── sync     ClientSync<C>  filter -> selection -> servo, sync state
//! └── idle_h3, idle_p3 (tx_arb source 3), drain_stamps (tx_path.stamps)
//! ```
//!
//! # Channels
//!
//! Every link between two units is a `chan()` (registered, two entries);
//! wires join register outputs to inputs (config, clock, sync state,
//! `route.tx_latency` through `lat`, which adds a constant). New links:
//!
//! | from | to | type |
//! |---|---|---|
//! | `udp4_rx` NTP pair | `demux4` | `Meta`, `FrameByte` |
//! | `udp6_rx` NTP pair | `demux6` | `Meta`, `FrameByte` |
//! | `demux4`/`demux6` `srv` pairs | `server.meta4`/`req4`, `.meta6`/`req6` | `Meta`, `FrameByte` |
//! | `demux4`/`demux6` `cli` pairs | `cmerge` sources 0 / 1 | `Meta`, `FrameByte` |
//! | `cmerge` | `client.meta`, `.rep` | `Meta`, `FrameByte` |
//! | `route` v4 / v6 pairs | `uarb4` / `uarb6` source 0 | `UdpTxReq` / `Udp6TxReq`, `FrameByte` |
//! | `client.req4`/`pay4`, `.req6`/`pay6` | `uarb4` / `uarb6` source 1 | same |
//! | `uarb4` / `uarb6` | `udp4_tx` / `udp6_tx` | same |
//! | `client.arp_lookup`, `.arp_resolve` | `arp.lookup`, `.resolve` | `U<32>` |
//! | `arp.lookup_rsp` | `client.arp_rsp` | `ArpEntry` |
//! | `client.nd_lookup` | `nbr.lookup` | `U<128>` |
//! | `nbr.rsp` | `client.nd_rsp` | `ArpEntry` |
//! | `client.nd_resolve` | `icmp6.resolve` | `U<128>` |
//! | `icmp6.learn` | `nbr.learn` | `NdLearn` |
//! | `client.samples`, `.miss` | `sync.samples`, `.miss` | `Sample`, `U<2>` |
//!
//! ## One answer per lookup
//!
//! `ntp_client` waits for the answer of a lookup without a timeout. `arp`
//! answers every `lookup` in the cycle it takes it (`lookup_rsp` must have
//! room, otherwise the lookup waits in its channel), and so does `nbr`
//! (`rsp`); both use their own channel, which holds what is sent until the
//! client takes it (nothing is ever dropped by a `chan`), and nothing else
//! writes those channels. The client has at most one lookup in flight (state
//! `LWT`) and always takes the answer of the family it asked. So every
//! lookup gets exactly one answer, in order.
//!
//! ## Requests versus replies on the wire
//!
//! * ARP: the client's `resolve` makes `arp` send a request from its own
//!   output (tx_arb source 0); a reply to a received request has priority
//!   over a pending resolve, decided when the output is idle; a request can
//!   only wait for one reply. `tx_arb` is round-robin between its sources.
//! * NS: `icmp6` takes a `resolve` only between two messages; a pending
//!   `resolve` holds back the next received message, so neither starves.
//!   NS, NA and echo replies share `icmp6`'s output (IpArb6 source 0).
//! * UDP: `uarb4`/`uarb6` alternate whole datagrams between the server's
//!   replies and the client's requests (round robin, never interleaved).
//!
//! # Transmit latency of T1 and T3
//!
//! `NtpReplyRoute`'s constants (`TX_LATENCY_V4/V6`) were calibrated for the
//! path `server -> route -> udp4_tx/udp6_tx` with one channel between each.
//! Here the server's path has one more channel stage (`route -> uarb ->
//! udp*_tx`): every byte reaches the store-and-forward `udp*_tx` one cycle
//! later, so the frame leaves 10 ns later. `lat` adds [`SERVER_ARB_ADJ`]
//! (10 ns) to the route's value before the server samples it. The client's
//! path (`client -> uarb -> udp*_tx`) has exactly the stage count of the
//! calibrated one (`server -> route -> udp*_tx`; both `route` and the
//! arbiter forward a byte in the cycle they pop it), so the client's
//! `tx_latency4/6` are the route's constants unchanged
//! ([`CLIENT_TX_LATENCY_V4`], [`CLIENT_TX_LATENCY_V6`]). Both are measured
//! by `rtl/tests/top_client.rs` against the SFD instant (25 MHz MII, the
//! eight 5 ns phases of `clk_tx`): client T1 and server T3 errors are in
//! -25..+10 ns, the window of the server-only top over the same phases. As
//! for the server, the constants assume an idle transmit path: a frame
//! that queues behind another (server reply behind a client request or
//! the reverse, ARP, NS, echo) leaves later than its timestamp says.
//!
//! # Clock (`K`)
//!
//! [`ClockSource`] is the unit that produces `time` from the servo's trim
//! and step: [`DisciplinedClock`] on the board. [`WarpClock`] is a
//! **simulation-only** clock (never in a board build) that runs `A` times
//! faster than real time: it is a `DisciplinedClock` whose output gets an
//! extra `(A - 1) * INCR` per cycle (`time = acc + warp`), so the steps and
//! the trim of the servo still apply exactly to `time`, but one NTP second
//! lasts `1e8 / A` cycles. Every consumer (rx/tx stamps, server, client,
//! filter, selection, servo, sync state) sees the same warped time, so the
//! protocol logic is consistent; what it breaks is the absolute meaning of
//! the hardware timestamps: a latency of n cycles reads as `A * n * 10 ns`
//! instead of `n * 10 ns`, so T1/T2/T3/T4 are off by `(A - 1)` times the
//! real path latencies (the fixed biases in ns of `rx_path`, `tx_path` and
//! the K constants are not scaled), and the trim's effect is `A` times too
//! small relative to the time scale. The system tests use it only to reach
//! the client's seconds-long timers (lookup retries, iburst, deadlines)
//! within a few hundred thousand cycles.
use crate::arp::{Arp, ArpEntry};
use crate::clock::{DisciplinedClock, INCR};
use crate::config::{Config, ConfigSet};
use crate::eth_rx::{EthMeta, EthRx};
use crate::eth_tx::{EthHdr, EthTx};
use crate::fcs_check::FcsCheck;
use crate::icmp_echo::IcmpEcho;
use crate::icmpv6::{Icmpv6, NdLearn};
use crate::ip_arb::{IpArb4, IpArb6};
use crate::ipv4_rx::{IpMeta, Ipv4Rx};
use crate::ipv4_tx::{IpTxReq, Ipv4Tx};
use crate::ipv6_rx::{Ip6Meta, Ipv6Rx};
use crate::ipv6_tx::{Ip6TxReq, Ipv6Tx};
use crate::nbr_cache::NbrCache;
use crate::ntp_client::NtpClient;
use crate::ntp_demux::NtpDemux;
use crate::ntp_reply_route::{ps_to_ntp, NtpReplyRoute, TX_LATENCY_V4, TX_LATENCY_V6};
use crate::ntp_server::{NtpReplyMeta, NtpServer};
use crate::ntp_types::Sample;
use crate::rx_path::RxPath;
use crate::stream::{FrameByte, Meta, Ts};
use crate::top::{ClientSync, Idle, TopCore, TxArbOf, TxPathOf};
use crate::udp4_rx::Udp4Rx;
use crate::udp4_tx::{Udp4Tx, UdpTxReq};
use crate::udp6_rx::Udp6Rx;
use crate::udp6_tx::{Udp6Tx, Udp6TxReq};
use crate::udp_arb::{MetaArb, UdpArb4, UdpArb6};
use txhdl::comp::trace::Traceable;
use txhdl::comp::{chan, join2, signal, tie, Clock, DefaultClock, In, Out, Reg, Rx, Tx, Unit};
use txhdl::netlist::{Lower, Port};
use txhdl::types::{Bit, U};
use txhdl::{lower, Trace};

/// One channel stage (one `clk_sys` cycle), picoseconds.
pub const ARB_STAGE_PS: u64 = 10_000;
/// Added to `route.tx_latency` before the server samples it: the extra
/// channel stage of the UDP arbiter on the server's path (2^-32 s units).
pub const SERVER_ARB_ADJ: u64 = ps_to_ntp(ARB_STAGE_PS);
/// `tx_latency4` of the client: the route's IPv4 constant (same stage count).
pub const CLIENT_TX_LATENCY_V4: u64 = TX_LATENCY_V4;
/// `tx_latency6` of the client: the route's IPv6 constant.
pub const CLIENT_TX_LATENCY_V6: u64 = TX_LATENCY_V6;

// ---------------------------------------------------------------------
// Clock sources
// ---------------------------------------------------------------------

/// A unit with the ports of [`DisciplinedClock`]: `trim`, `step`,
/// `step_en` in; `time`, `stepped` out.
pub trait ClockSource:
    Unit<(In<U<32>>, In<U<96>>, In<Bit>), (Out<U<96>>, Out<Bit>)> + Lower + Port + Traceable + Default
{
}
impl ClockSource for DisciplinedClock {}

/// `(A - 1) * INCR`: what [`WarpAdd`] adds per cycle.
pub const fn warp_step(a: u64) -> u128 {
    (a as u128 - 1) * INCR
}

/// Simulation only: `time = time_in + warp`, `warp` growing by `(A - 1) *
/// INCR` per cycle (one 96-bit adder from a register output to the
/// output wire). See the module documentation.
#[derive(Trace, Default)]
pub struct WarpAdd<const A: u64> {
    pub warp: Reg<U<96>>,
}

#[lower]
impl<const A: u64> Unit<In<U<96>>, Out<U<96>>> for WarpAdd<A> {
    async fn run(&mut self, time_in: In<U<96>>, time: Out<U<96>>) {
        loop {
            DefaultClock::rising().await;
            let w = self.warp.get();
            time.set(time_in.get() + w);
            self.warp.set(w + U::<96>::new(warp_step(A)));
        }
    }
}

/// Simulation only: a [`DisciplinedClock`] running `A` times faster than
/// real time (module documentation). `clock` is the real clock (its `acc`
/// is where a test sets the start time), `add` the warp.
#[derive(Trace, Default)]
pub struct WarpClock<const A: u64> {
    pub clock: DisciplinedClock,
    pub add: WarpAdd<A>,
}

#[lower]
impl<const A: u64> Unit<(In<U<32>>, In<U<96>>, In<Bit>), (Out<U<96>>, Out<Bit>)> for WarpClock<A> {
    async fn run(
        &mut self,
        (trim, step, step_en): (In<U<32>>, In<U<96>>, In<Bit>),
        (time, stepped): (Out<U<96>>, Out<Bit>),
    ) {
        let (raw_o, raw) = signal::<U<96>, DefaultClock>();
        join2(
            self.clock.run((trim, step, step_en), (raw_o, stepped)),
            self.add.run(raw, time),
        )
        .await;
    }
}
impl<const A: u64> ClockSource for WarpClock<A> {}

// ---------------------------------------------------------------------
// Glue
// ---------------------------------------------------------------------

/// `out = inp + SERVER_ARB_ADJ`: the server's transmit latency, the
/// route's family constant plus the arbiter's stage (combinational from the
/// route's register).
#[derive(Trace, Default)]
pub struct LatAdj {}

#[lower]
impl Unit<In<U<64>>, Out<U<64>>> for LatAdj {
    async fn run(&mut self, inp: In<U<64>>, out: Out<U<64>>) {
        loop {
            DefaultClock::rising().await;
            out.set(inp.get() + U::<64>::new(SERVER_ARB_ADJ as u128));
        }
    }
}

// ---------------------------------------------------------------------
// The core
// ---------------------------------------------------------------------

/// The server and client paths between the MII byte streams, generic in the
/// two MII clocks (`R`, `T`), the configuration `C` and the clock `K`. Ports
/// as [`ArtyCore`](crate::top::ArtyCore). The synchronisation source is
/// always [`ClientSync<C>`].
#[derive(Trace)]
pub struct ArtyCoreClient<R: Clock, T: Clock, C: ConfigSet, K: ClockSource> {
    pub rx_path: RxPath<R, DefaultClock>,
    pub fcs: FcsCheck,
    pub eth_rx: EthRx,
    pub arp: Arp,
    pub ipv4_rx: Ipv4Rx,
    pub icmp4: IcmpEcho,
    pub udp4_rx: Udp4Rx,
    pub ipv6_rx: Ipv6Rx,
    pub icmp6: Icmpv6,
    pub udp6_rx: Udp6Rx,
    pub demux4: NtpDemux,
    pub demux6: NtpDemux,
    pub cmerge: MetaArb,
    pub client: NtpClient,
    pub nbr: NbrCache,
    pub server: NtpServer,
    pub route: NtpReplyRoute,
    pub lat: LatAdj,
    pub uarb4: UdpArb4,
    pub uarb6: UdpArb6,
    pub udp4_tx: Udp4Tx,
    pub udp6_tx: Udp6Tx,
    pub arb4: IpArb4,
    pub arb6: IpArb6,
    pub ipv4_tx: Ipv4Tx,
    pub ipv6_tx: Ipv6Tx,
    pub tx_arb: TxArbOf,
    pub eth_tx: EthTx,
    pub tx_path: TxPathOf<T>,
    pub cfg: Config<C>,
    pub clock: K,
    pub sync: ClientSync<C>,
    pub idle_h3: Idle<EthHdr>,
    pub idle_p3: Idle<FrameByte>,
    pub drain_stamps: crate::top::Drain<Ts>,
}

impl<R: Clock, T: Clock, C: ConfigSet, K: ClockSource> Default for ArtyCoreClient<R, T, C, K> {
    fn default() -> Self {
        ArtyCoreClient {
            rx_path: Default::default(),
            fcs: Default::default(),
            eth_rx: Default::default(),
            arp: Default::default(),
            ipv4_rx: Default::default(),
            icmp4: Default::default(),
            udp4_rx: Default::default(),
            ipv6_rx: Default::default(),
            icmp6: Default::default(),
            udp6_rx: Default::default(),
            demux4: Default::default(),
            demux6: Default::default(),
            cmerge: Default::default(),
            client: Default::default(),
            nbr: Default::default(),
            server: Default::default(),
            route: Default::default(),
            lat: Default::default(),
            uarb4: Default::default(),
            uarb6: Default::default(),
            udp4_tx: Default::default(),
            udp6_tx: Default::default(),
            arb4: Default::default(),
            arb6: Default::default(),
            ipv4_tx: Default::default(),
            ipv6_tx: Default::default(),
            tx_arb: Default::default(),
            eth_tx: Default::default(),
            tx_path: Default::default(),
            cfg: Default::default(),
            clock: Default::default(),
            sync: Default::default(),
            idle_h3: Default::default(),
            idle_p3: Default::default(),
            drain_stamps: Default::default(),
        }
    }
}

impl<R: Clock, T: Clock, C: ConfigSet, K: ClockSource> TopCore<R, T> for ArtyCoreClient<R, T, C, K> {}

#[lower]
impl<R: Clock, T: Clock, C: ConfigSet, K: ClockSource>
    Unit<(Rx<FrameByte, R>, In<Bit, R>, In<Bit, T>, In<Bit, T>), Tx<FrameByte, T>>
    for ArtyCoreClient<R, T, C, K>
{
    async fn run(
        &mut self,
        (raw, rx_sfd, tx_sfd, tx_underrun): (Rx<FrameByte, R>, In<Bit, R>, In<Bit, T>, In<Bit, T>),
        tx_frames: Tx<FrameByte, T>,
    ) {
        // Configuration wires.
        let (local_mac_o, local_mac) = signal::<U<48>, DefaultClock>();
        let (local_ip4_o, local_ip4) = signal::<U<32>, DefaultClock>();
        let (local_ll_o, local_ll) = signal::<U<128>, DefaultClock>();
        let (local_gua_o, local_gua) = signal::<U<128>, DefaultClock>();
        let (gua_valid_o, gua_valid) = signal::<Bit, DefaultClock>();
        let (sn_low_o, sn_low) = signal::<U<24>, DefaultClock>();
        let (gateway_o, gateway) = signal::<U<32>, DefaultClock>();
        let (gateway6_o, gateway6) = signal::<U<128>, DefaultClock>();
        let (netmask_o, netmask) = signal::<U<32>, DefaultClock>();
        let (up0_addr_o, up0_addr) = signal::<U<128>, DefaultClock>();
        let (up0_v6_o, up0_v6) = signal::<Bit, DefaultClock>();
        let (up0_en_o, up0_en) = signal::<Bit, DefaultClock>();
        let (up1_addr_o, up1_addr) = signal::<U<128>, DefaultClock>();
        let (up1_v6_o, up1_v6) = signal::<Bit, DefaultClock>();
        let (up1_en_o, up1_en) = signal::<Bit, DefaultClock>();
        let (up2_addr_o, up2_addr) = signal::<U<128>, DefaultClock>();
        let (up2_v6_o, up2_v6) = signal::<Bit, DefaultClock>();
        let (up2_en_o, up2_en) = signal::<Bit, DefaultClock>();
        let (up3_addr_o, up3_addr) = signal::<U<128>, DefaultClock>();
        let (up3_v6_o, up3_v6) = signal::<Bit, DefaultClock>();
        let (up3_en_o, up3_en) = signal::<Bit, DefaultClock>();
        let (poll_log2_o, poll_log2) = signal::<U<8>, DefaultClock>();
        let (precision_o, precision) = signal::<U<8>, DefaultClock>();
        let (rate_limit_o, rate_limit) = signal::<U<16>, DefaultClock>();
        let local_mac_b = local_mac.clone();
        let local_mac_c = local_mac.clone();
        let local_mac_d = local_mac.clone();
        let local_mac_e = local_mac.clone();
        let local_ip4_b = local_ip4.clone();
        let local_ip4_c = local_ip4.clone();
        let local_ip4_d = local_ip4.clone();
        let local_ip4_e = local_ip4.clone();
        let local_ll_b = local_ll.clone();
        let local_ll_c = local_ll.clone();
        let local_gua_b = local_gua.clone();
        let local_gua_c = local_gua.clone();
        let gua_valid_b = gua_valid.clone();
        let gua_valid_c = gua_valid.clone();
        let sn_low_b = sn_low.clone();
        let precision_b = precision.clone();
        let precision_c = precision.clone();
        let poll_log2_b = poll_log2.clone();

        // Clock and synchronisation wires.
        let (time_o, time) = signal::<U<96>, DefaultClock>();
        let (stepped_o, _stepped) = signal::<Bit, DefaultClock>();
        let time_b = time.clone();
        let time_c = time.clone();
        let time_d = time.clone();
        let time_e = time.clone();
        let (synced_o, synced) = signal::<Bit, DefaultClock>();
        let (stratum_o, stratum) = signal::<U<8>, DefaultClock>();
        let (ref_id_o, ref_id) = signal::<U<32>, DefaultClock>();
        let (ref_id6_o, ref_id6) = signal::<U<32>, DefaultClock>();
        let (ref_ts_o, ref_ts) = signal::<U<64>, DefaultClock>();
        let (root_delay_o, root_delay) = signal::<U<32>, DefaultClock>();
        let (root_disp_o, root_disp) = signal::<U<32>, DefaultClock>();
        let (trim_o, trim) = signal::<U<32>, DefaultClock>();
        let (step_o, step) = signal::<U<96>, DefaultClock>();
        let (step_en_o, step_en) = signal::<Bit, DefaultClock>();
        let (route_lat_o, route_lat) = signal::<U<64>, DefaultClock>();
        let (tx_latency_o, tx_latency) = signal::<U<64>, DefaultClock>();

        // Receive path.
        let (rxf_tx, rxf_rx) = chan::<FrameByte, DefaultClock>();
        let (rxs_tx, rxs_rx) = chan::<Ts, DefaultClock>();
        let (fcf_tx, fcf_rx) = chan::<FrameByte, DefaultClock>();
        let (fcs_tx, fcs_rx) = chan::<Ts, DefaultClock>();
        let (arpm_tx, arpm_rx) = chan::<EthMeta, DefaultClock>();
        let (arpp_tx, arpp_rx) = chan::<FrameByte, DefaultClock>();
        let (v4m_tx, v4m_rx) = chan::<EthMeta, DefaultClock>();
        let (v4p_tx, v4p_rx) = chan::<FrameByte, DefaultClock>();
        let (v6m_tx, v6m_rx) = chan::<EthMeta, DefaultClock>();
        let (v6p_tx, v6p_rx) = chan::<FrameByte, DefaultClock>();
        let (i4m_tx, i4m_rx) = chan::<IpMeta, DefaultClock>();
        let (i4p_tx, i4p_rx) = chan::<FrameByte, DefaultClock>();
        let (u4m_tx, u4m_rx) = chan::<IpMeta, DefaultClock>();
        let (u4p_tx, u4p_rx) = chan::<FrameByte, DefaultClock>();
        let (i6m_tx, i6m_rx) = chan::<Ip6Meta, DefaultClock>();
        let (i6p_tx, i6p_rx) = chan::<FrameByte, DefaultClock>();
        let (u6m_tx, u6m_rx) = chan::<Ip6Meta, DefaultClock>();
        let (u6p_tx, u6p_rx) = chan::<FrameByte, DefaultClock>();
        let (n4m_tx, n4m_rx) = chan::<Meta, DefaultClock>();
        let (n4p_tx, n4p_rx) = chan::<FrameByte, DefaultClock>();
        let (n6m_tx, n6m_rx) = chan::<Meta, DefaultClock>();
        let (n6p_tx, n6p_rx) = chan::<FrameByte, DefaultClock>();
        // Mode demultiplexers: server and client pairs, client merge.
        let (s4m_tx, s4m_rx) = chan::<Meta, DefaultClock>();
        let (s4p_tx, s4p_rx) = chan::<FrameByte, DefaultClock>();
        let (s6m_tx, s6m_rx) = chan::<Meta, DefaultClock>();
        let (s6p_tx, s6p_rx) = chan::<FrameByte, DefaultClock>();
        let (c4m_tx, c4m_rx) = chan::<Meta, DefaultClock>();
        let (c4p_tx, c4p_rx) = chan::<FrameByte, DefaultClock>();
        let (c6m_tx, c6m_rx) = chan::<Meta, DefaultClock>();
        let (c6p_tx, c6p_rx) = chan::<FrameByte, DefaultClock>();
        let (cm_tx, cm_rx) = chan::<Meta, DefaultClock>();
        let (cp_tx, cp_rx) = chan::<FrameByte, DefaultClock>();

        // Transmit path.
        let (rm_tx, rm_rx) = chan::<NtpReplyMeta, DefaultClock>();
        let (rp_tx, rp_rx) = chan::<FrameByte, DefaultClock>();
        let (q4_tx, q4_rx) = chan::<UdpTxReq, DefaultClock>();
        let (y4_tx, y4_rx) = chan::<FrameByte, DefaultClock>();
        let (q6_tx, q6_rx) = chan::<Udp6TxReq, DefaultClock>();
        let (y6_tx, y6_rx) = chan::<FrameByte, DefaultClock>();
        let (cq4_tx, cq4_rx) = chan::<UdpTxReq, DefaultClock>();
        let (cy4_tx, cy4_rx) = chan::<FrameByte, DefaultClock>();
        let (cq6_tx, cq6_rx) = chan::<Udp6TxReq, DefaultClock>();
        let (cy6_tx, cy6_rx) = chan::<FrameByte, DefaultClock>();
        let (uq4_tx, uq4_rx) = chan::<UdpTxReq, DefaultClock>();
        let (uy4_tx, uy4_rx) = chan::<FrameByte, DefaultClock>();
        let (uq6_tx, uq6_rx) = chan::<Udp6TxReq, DefaultClock>();
        let (uy6_tx, uy6_rx) = chan::<FrameByte, DefaultClock>();
        let (ie4r_tx, ie4r_rx) = chan::<IpTxReq, DefaultClock>();
        let (ie4p_tx, ie4p_rx) = chan::<FrameByte, DefaultClock>();
        let (ut4r_tx, ut4r_rx) = chan::<IpTxReq, DefaultClock>();
        let (ut4p_tx, ut4p_rx) = chan::<FrameByte, DefaultClock>();
        let (ie6r_tx, ie6r_rx) = chan::<Ip6TxReq, DefaultClock>();
        let (ie6p_tx, ie6p_rx) = chan::<FrameByte, DefaultClock>();
        let (ut6r_tx, ut6r_rx) = chan::<Ip6TxReq, DefaultClock>();
        let (ut6p_tx, ut6p_rx) = chan::<FrameByte, DefaultClock>();
        let (a4r_tx, a4r_rx) = chan::<IpTxReq, DefaultClock>();
        let (a4p_tx, a4p_rx) = chan::<FrameByte, DefaultClock>();
        let (a6r_tx, a6r_rx) = chan::<Ip6TxReq, DefaultClock>();
        let (a6p_tx, a6p_rx) = chan::<FrameByte, DefaultClock>();
        let (h0_tx, h0_rx) = chan::<EthHdr, DefaultClock>();
        let (p0_tx, p0_rx) = chan::<FrameByte, DefaultClock>();
        let (h1_tx, h1_rx) = chan::<EthHdr, DefaultClock>();
        let (p1_tx, p1_rx) = chan::<FrameByte, DefaultClock>();
        let (h2_tx, h2_rx) = chan::<EthHdr, DefaultClock>();
        let (p2_tx, p2_rx) = chan::<FrameByte, DefaultClock>();
        let (h3_tx, h3_rx) = chan::<EthHdr, DefaultClock>();
        let (p3_tx, p3_rx) = chan::<FrameByte, DefaultClock>();
        let (eh_tx, eh_rx) = chan::<EthHdr, DefaultClock>();
        let (ep_tx, ep_rx) = chan::<FrameByte, DefaultClock>();
        let (ef_tx, ef_rx) = chan::<FrameByte, DefaultClock>();
        let (st_tx, st_rx) = chan::<Ts, DefaultClock>();

        // Address resolution and client outputs.
        let (res_tx, res_rx) = chan::<U<32>, DefaultClock>();
        let (lk_tx, lk_rx) = chan::<U<32>, DefaultClock>();
        let (lr_tx, lr_rx) = chan::<ArpEntry, DefaultClock>();
        let (ndres_tx, ndres_rx) = chan::<U<128>, DefaultClock>();
        let (ndlk_tx, ndlk_rx) = chan::<U<128>, DefaultClock>();
        let (ndr_tx, ndr_rx) = chan::<ArpEntry, DefaultClock>();
        let (ndl_tx, ndl_rx) = chan::<NdLearn, DefaultClock>();
        let (smp_tx, smp_rx) = chan::<Sample, DefaultClock>();
        let (miss_tx, miss_rx) = chan::<U<2>, DefaultClock>();

        join2(
            join2(
                join2(
                    join2(
                        self.cfg.run(
                            (),
                            (
                                local_mac_o,
                                local_ip4_o,
                                local_ll_o,
                                local_gua_o,
                                gua_valid_o,
                                sn_low_o,
                                gateway_o,
                                gateway6_o,
                                netmask_o,
                                up0_addr_o,
                                up0_v6_o,
                                up0_en_o,
                                up1_addr_o,
                                up1_v6_o,
                                up1_en_o,
                                up2_addr_o,
                                up2_v6_o,
                                up2_en_o,
                                up3_addr_o,
                                up3_v6_o,
                                up3_en_o,
                                poll_log2_o,
                                precision_o,
                                rate_limit_o,
                            ),
                        ),
                        self.clock.run((trim, step, step_en), (time_o, stepped_o)),
                    ),
                    join2(
                        self.sync.run(
                            (time, precision, poll_log2, smp_rx, miss_rx),
                            (
                                synced_o,
                                stratum_o,
                                ref_id_o,
                                ref_id6_o,
                                ref_ts_o,
                                root_delay_o,
                                root_disp_o,
                                trim_o,
                                step_o,
                                step_en_o,
                            ),
                        ),
                        self.rx_path.run((raw, rx_sfd, time_b), (rxf_tx, rxs_tx)),
                    ),
                ),
                join2(
                    join2(
                        self.fcs
                            .run((rxf_rx, tie(U::<11>::new(0)), rxs_rx), (fcf_tx, fcs_tx)),
                        self.eth_rx.run(
                            (fcf_rx, local_mac, sn_low, fcs_rx),
                            (arpm_tx, arpp_tx, v4m_tx, v4p_tx, v6m_tx, v6p_tx),
                        ),
                    ),
                    join2(
                        self.arp.run(
                            (
                                arpm_rx,
                                arpp_rx,
                                local_mac_b,
                                local_ip4,
                                res_rx,
                                lk_rx,
                                tie(Bit::Zero),
                            ),
                            (h0_tx, p0_tx, lr_tx),
                        ),
                        join2(
                            self.ipv4_rx.run(
                                (v4m_rx, v4p_rx, local_ip4_b),
                                (i4m_tx, i4p_tx, u4m_tx, u4p_tx),
                            ),
                            self.ipv6_rx.run(
                                (v6m_rx, v6p_rx, local_ll, local_gua, gua_valid, sn_low_b),
                                (i6m_tx, i6p_tx, u6m_tx, u6p_tx),
                            ),
                        ),
                    ),
                ),
            ),
            join2(
                join2(
                    join2(
                        join2(
                            join2(
                                self.icmp4.run((i4m_rx, i4p_rx), (ie4r_tx, ie4p_tx)),
                                self.udp4_rx.run((u4m_rx, u4p_rx), (n4m_tx, n4p_tx)),
                            ),
                            join2(
                                self.icmp6.run(
                                    (
                                        i6m_rx,
                                        i6p_rx,
                                        local_ll_b,
                                        local_gua_b,
                                        gua_valid_b,
                                        local_mac_c,
                                        ndres_rx,
                                    ),
                                    (ie6r_tx, ie6p_tx, ndl_tx),
                                ),
                                self.udp6_rx.run((u6m_rx, u6p_rx), (n6m_tx, n6p_tx)),
                            ),
                        ),
                        join2(
                            join2(
                                self.demux4
                                    .run((n4m_rx, n4p_rx), (s4m_tx, s4p_tx, c4m_tx, c4p_tx)),
                                self.demux6
                                    .run((n6m_rx, n6p_rx), (s6m_tx, s6p_tx, c6m_tx, c6p_tx)),
                            ),
                            join2(
                                self.cmerge
                                    .run((c4m_rx, c4p_rx, c6m_rx, c6p_rx), (cm_tx, cp_tx)),
                                self.nbr.run((ndl_rx, ndlk_rx, tie(Bit::Zero)), (ndr_tx,)),
                            ),
                        ),
                    ),
                    join2(
                        join2(
                            self.server.run(
                                (
                                    s4m_rx,
                                    s4p_rx,
                                    s6m_rx,
                                    s6p_rx,
                                    time_c,
                                    synced,
                                    stratum,
                                    ref_id,
                                    ref_id6,
                                    ref_ts,
                                    root_delay,
                                    root_disp,
                                    precision_b,
                                    tx_latency,
                                    rate_limit,
                                    tie(Bit::Zero),
                                ),
                                (rm_tx, rp_tx),
                            ),
                            join2(
                                self.route
                                    .run((rm_rx, rp_rx), (q4_tx, y4_tx, q6_tx, y6_tx, route_lat_o)),
                                self.lat.run(route_lat, tx_latency_o),
                            ),
                        ),
                        join2(
                            self.client.run(
                                (
                                    up0_addr,
                                    up0_v6,
                                    up0_en,
                                    up1_addr,
                                    up1_v6,
                                    up1_en,
                                    up2_addr,
                                    up2_v6,
                                    up2_en,
                                    up3_addr,
                                    up3_v6,
                                    up3_en,
                                    local_ip4_e,
                                    netmask,
                                    gateway,
                                    gateway6,
                                    local_ll_c,
                                    local_gua_c,
                                    gua_valid_c,
                                    local_mac_e,
                                    poll_log2_b,
                                    precision_c,
                                    tie(U::<64>::new(CLIENT_TX_LATENCY_V4 as u128)),
                                    tie(U::<64>::new(CLIENT_TX_LATENCY_V6 as u128)),
                                    time_e,
                                    cm_rx,
                                    cp_rx,
                                    lr_rx,
                                    ndr_rx,
                                ),
                                (
                                    lk_tx, res_tx, ndlk_tx, ndres_tx, cq4_tx, cy4_tx, cq6_tx,
                                    cy6_tx, smp_tx, miss_tx,
                                ),
                            ),
                            join2(
                                self.uarb4
                                    .run((q4_rx, y4_rx, cq4_rx, cy4_rx), (uq4_tx, uy4_tx)),
                                self.uarb6
                                    .run((q6_rx, y6_rx, cq6_rx, cy6_rx), (uq6_tx, uy6_tx)),
                            ),
                        ),
                    ),
                ),
                join2(
                    join2(
                        join2(
                            join2(
                                self.udp4_tx
                                    .run((uq4_rx, uy4_rx, local_ip4_c), (ut4r_tx, ut4p_tx)),
                                self.udp6_tx.run((uq6_rx, uy6_rx), (ut6r_tx, ut6p_tx)),
                            ),
                            join2(
                                self.arb4
                                    .run((ie4r_rx, ie4p_rx, ut4r_rx, ut4p_rx), (a4r_tx, a4p_tx)),
                                self.arb6
                                    .run((ie6r_rx, ie6p_rx, ut6r_rx, ut6p_rx), (a6r_tx, a6p_tx)),
                            ),
                        ),
                        join2(
                            self.ipv4_tx
                                .run((a4r_rx, a4p_rx, local_ip4_d), (h1_tx, p1_tx)),
                            self.ipv6_tx.run((a6r_rx, a6p_rx), (h2_tx, p2_tx)),
                        ),
                    ),
                    join2(
                        join2(
                            self.tx_arb.run(
                                (h0_rx, p0_rx, h1_rx, p1_rx, h2_rx, p2_rx, h3_rx, p3_rx),
                                (eh_tx, ep_tx),
                            ),
                            join2(
                                self.eth_tx.run((eh_rx, ep_rx, local_mac_d), ef_tx),
                                self.tx_path
                                    .run((ef_rx, time_d, tx_sfd, tx_underrun), (st_tx, tx_frames)),
                            ),
                        ),
                        join2(
                            join2(self.idle_h3.run((), h3_tx), self.idle_p3.run((), p3_tx)),
                            self.drain_stamps.run(st_rx, ()),
                        ),
                    ),
                ),
            ),
        )
        .await;
    }
}
