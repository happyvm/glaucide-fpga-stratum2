// SPDX-License-Identifier: Apache-2.0
//! Top level of the NTP server (task T1.11): the server path of the whole
//! design as one hierarchical unit, lowered to one Verilog module tree.
//!
//! # Hierarchy
//!
//! ```text
//! ArtyTop<C, S, P>                          (pins of the PHY, 3 clocks)
//! ├── mii_rx   MiiRx               ClkRx
//! ├── mii_tx   MiiTx               ClkTx
//! ├── phy      PhyInit<P>          clk_sys
//! └── core     ArtyCore<ClkRx, ClkTx, C, S>
//!     ├── rx_path   RxPath<R, clk_sys>    ClkRx -> clk_sys, RX stamps
//!     ├── fcs       FcsCheck
//!     ├── eth_rx    EthRx
//!     ├── arp       Arp                          -> tx_arb source 0
//!     ├── ipv4_rx   Ipv4Rx
//!     │   ├── icmp4     IcmpEcho  -> arb4 source 0 ┐
//!     │   └── udp4_rx   Udp4Rx    -> server (v4)   │
//!     ├── ipv6_rx   Ipv6Rx                         │
//!     │   ├── icmp6     Icmpv6    -> arb6 source 0 │
//!     │   └── udp6_rx   Udp6Rx    -> server (v6)   │
//!     ├── server    NtpServer -> route             │
//!     ├── route     NtpReplyRoute -> udp4_tx / udp6_tx
//!     ├── udp4_tx   Udp4Tx    -> arb4 source 1     │
//!     ├── udp6_tx   Udp6Tx    -> arb6 source 1     │
//!     ├── arb4      IpArb4 -> ipv4_tx              │   -> tx_arb source 1
//!     ├── arb6      IpArb6 -> ipv6_tx              │   -> tx_arb source 2
//!     ├── tx_arb    TxArb (source 3: idle, reserved for the client side)
//!     ├── eth_tx    EthTx
//!     ├── tx_path   TxPathG<.., clk_sys, T>  clk_sys -> ClkTx, TX stamps
//!     ├── cfg       Config<C>        static addresses
//!     ├── clock     DisciplinedClock `time`
//!     ├── sync      S: SyncSource    synchronisation wires, trim, step
//!     └── idle/drain units on the unused channel ends (below)
//! ```
//!
//! `ArtyCore` is generic in its two MII clocks so that the system tests can
//! run it with MII clocks of any period (25 MHz = 4 `clk_sys` periods, or a
//! slightly different ratio) against a model of the MII units: `MiiRx` and
//! `MiiTx` are fixed to `ClkRx`/`ClkTx`, whose simulation period is that of
//! `clk_sys` (see `docs/questions.md`). `ArtyTop` adds the two MII units and
//! the PHY management: it is what the board wrapper instantiates.
//!
//! # Channels between units
//!
//! Every pair of units is joined by a `chan()` (in the netlist a
//! `<top>_txhdl_chan`: a two-entry registered buffer whose `ready` is a
//! register, so no combinational valid/ready loop crosses a module
//! boundary, note "Netlist" of `stream.rs`). Port-to-child pass-through
//! (`ArtyCore`'s `raw` to `rx_path`, `tx_frames` from `tx_path`) carries
//! the channel of the parent (`ArtyTop`). Wires (`In`/`Out`) join register
//! outputs (config, clock, sync source, `mii_rx.sfd`, `mii_tx.sfd`) to
//! inputs, never a handshake.
//!
//! | from | to | type | clock |
//! |---|---|---|---|
//! | `mii_rx.frames` | `rx_path.raw` | `FrameByte` | ClkRx |
//! | `mii_rx.sfd` | `rx_path.sfd` | wire | ClkRx |
//! | `rx_path.frames`, `.stamps` | `fcs.raw`, `.stamps_in` | `FrameByte`, `Ts` | clk_sys |
//! | `fcs.frames`, `.stamps_out` | `eth_rx.frames`, `.stamps` | `FrameByte`, `Ts` | clk_sys |
//! | `eth_rx` ARP pair | `arp.meta`, `.payload` | `EthMeta`, `FrameByte` | clk_sys |
//! | `eth_rx` IPv4 pair | `ipv4_rx` | `EthMeta`, `FrameByte` | clk_sys |
//! | `eth_rx` IPv6 pair | `ipv6_rx` | `EthMeta`, `FrameByte` | clk_sys |
//! | `ipv4_rx` ICMP pair | `icmp4` | `IpMeta`, `FrameByte` | clk_sys |
//! | `ipv4_rx` UDP pair | `udp4_rx` | `IpMeta`, `FrameByte` | clk_sys |
//! | `ipv6_rx` ICMP pair | `icmp6` | `Ip6Meta`, `FrameByte` | clk_sys |
//! | `ipv6_rx` UDP pair | `udp6_rx` | `Ip6Meta`, `FrameByte` | clk_sys |
//! | `udp4_rx` | `server.meta4`, `.req4` | `Meta`, `FrameByte` | clk_sys |
//! | `udp6_rx` | `server.meta6`, `.req6` | `Meta`, `FrameByte` | clk_sys |
//! | `server.reply_meta`, `.reply` | `route` | `NtpReplyMeta`, `FrameByte` | clk_sys |
//! | `route` v4 pair | `udp4_tx` | `UdpTxReq`, `FrameByte` | clk_sys |
//! | `route` v6 pair | `udp6_tx` | `Udp6TxReq`, `FrameByte` | clk_sys |
//! | `route.tx_latency` | `server.tx_latency` | wire | clk_sys |
//! | `icmp4`, `udp4_tx` | `arb4` sources 0, 1 | `IpTxReq`, `FrameByte` | clk_sys |
//! | `icmp6`, `udp6_tx` | `arb6` sources 0, 1 | `Ip6TxReq`, `FrameByte` | clk_sys |
//! | `arb4` | `ipv4_tx` | `IpTxReq`, `FrameByte` | clk_sys |
//! | `arb6` | `ipv6_tx` | `Ip6TxReq`, `FrameByte` | clk_sys |
//! | `arp`, `ipv4_tx`, `ipv6_tx`, idle | `tx_arb` sources 0, 1, 2, 3 | `EthHdr`, `FrameByte` | clk_sys |
//! | `tx_arb` | `eth_tx` | `EthHdr`, `FrameByte` | clk_sys |
//! | `eth_tx.frames` | `tx_path.frames` | `FrameByte` | clk_sys |
//! | `tx_path.stamps` | drain (Phase A; the client in Phase B) | `Ts` | clk_sys |
//! | `tx_path.tx_frames` | `mii_tx.frames` | `FrameByte` | ClkTx |
//! | `mii_tx.sfd`, `.underrun` | `tx_path.sfd`, `.underrun` | wires | ClkTx |
//! | `clock.time` | `rx_path`, `tx_path`, `server`, `sync` | wire `U<96>` | clk_sys |
//! | `sync` outputs | `server` sync wires; `clock.trim/step/step_en` | wires | clk_sys |
//!
//! Unused ends: `arp.resolve`/`.lookup` and `tx_arb` source 3 are fed by
//! [`Idle`] units (valid constant 0); `arp.lookup_rsp` and `tx_path.stamps`
//! go to [`Drain`] units (always ready; `tx_path` would stall otherwise:
//! a stamp consumer must take every stamp). The client hooks (`samples`,
//! `miss` of the sync source) are fed by [`Idle`] units as well.
//! `docs/top-client-hooks.md` lists what the client side must connect.
//!
//! # Tied inputs
//!
//! `fcs.pick` (formal witness) = 0, `arp.flush` = 0, `phy.phy_addr` = 1
//! (Arty strap), `server.kod_enable` = 0 (rate-limited requests get no
//! answer; silence is ntpd's default without `restrict ... kod`),
//! `server.rate_limit` and `.precision` from the configuration.
//!
//! # Synchronisation source (`S`)
//!
//! [`SyncSource`] is the unit that drives the server's synchronisation
//! wires and the clock's trim/step: [`Standalone`] (the board in Phase A:
//! `synced = 0`, stratum 16, LI 3, everything else 0, trim/step 0),
//! [`FixedSync`] (simulation: synchronised stratum 2 with the golden
//! model's reference id, reference timestamp, root delay and dispersion,
//! `tools/golden/vectors/README.md`), [`FixedUnsync`] (the same fields,
//! not synchronised), [`ClientSync`] (Phase B: clock filter -> selection
//! -> servo and sync state, see its doc).
//!
//! The marker of the task ("`WithClient`, default off") is this parameter:
//! [`ArtyBoard`] (Phase A, generated by `top_gen`) uses [`Standalone`];
//! [`ArtyBoardWithClient`] (`top_gen --with-client`) uses [`ClientSync`].
//! The client itself, the mode demultiplexer and the IPv6 neighbor cache
//! are not in this branch: `docs/top-client-hooks.md` lists the channels
//! of [`ArtyCore`] they replace (`idle_*`, `drain_*` units) or cut.
//!
//! # Timing assumptions
//!
//! * `clk_sys` 100 MHz (`DefaultClock`), `ClkRx`/`ClkTx` 25 MHz from the PHY,
//!   unrelated to `clk_sys` and to each other. Every crossing is inside
//!   `rx_path` (ChanCdc, Gray) and `tx_path` (Gray + req/ack handshake): no
//!   other path crosses a domain. The `mii_rx`/`mii_tx` strobes go only to
//!   their own domain's half of those units.
//! * Config wires are constant; sync wires change only with the sync source
//!   (registers); `time` changes every cycle (a register).
//! * Throughput: the RX side is cut-through from `eth_rx` to the server
//!   and the ICMP responders; stalls are absorbed by `rx_path`'s 2048-byte
//!   FIFO (a frame that does not find room is dropped there). The TX side
//!   store-and-forwards in `udp4_tx`/`udp6_tx` (128 bytes) and in
//!   `tx_path` (4096 bytes); `mii_tx` never underruns.
//! * T3 (`server.tx_latency`): the route unit feeds the latency of the
//!   reply's family, constants of `ntp_reply_route.rs` calibrated for this
//!   top on an idle path (`rtl/tests/top.rs`): with 25 MHz MII clocks, the
//!   T3 error against the SFD instant on the MII is -10..+10 ns and the T2
//!   error 0..+5 ns over five clock phases (PHY latencies excluded).
//!
//! # Reset
//!
//! One `rst` net reaches every module (TxHDL netlist), synchronous to each
//! module's clock edge. The board wrapper (`rtl/hdl/arty_top_board.v`)
//! generates it in `clk_sys` from the MMCM `locked` (two-flop synchroniser
//! and a counter) and makes it safe for the two MII domains by stopping
//! their clocks (`BUFGCE`) around the release: reset is asserted with the
//! MII clocks running (their registers see it for many edges), the clocks
//! are gated off, the reset is released in `clk_sys`, then the clocks are
//! re-enabled; each `BUFGCE` enable is synchronised (two flops) in its own
//! ungated clock. So no MII-domain register sees the reset edge near a
//! clock edge. See the wrapper's header.
use crate::arp::{Arp, ArpEntry};
use crate::clock::DisciplinedClock;
use crate::clock_filter::ClockFilter;
use crate::clocks::{ClkRx, ClkTx};
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
use crate::mii_rx::MiiRx;
use crate::mii_tx::MiiTx;
use crate::ntp_reply_route::NtpReplyRoute;
use crate::ntp_server::{NtpReplyMeta, NtpServer};
use crate::ntp_types::{PeerStat, Sample, Selected};
use crate::phy_init::{PhyInit, PhyTiming};
use crate::refid::{refid_ipv4, refid_ipv6};
use crate::rx_path::RxPath;
use crate::selection::Selection;
use crate::servo::Servo;
use crate::stream::{FrameByte, Meta, Ts};
use crate::sync_state::SyncState;
use crate::tx_path::TxPathG;
use crate::udp4_rx::Udp4Rx;
use crate::udp4_tx::{Udp4Tx, UdpTxReq};
use crate::udp6_rx::Udp6Rx;
use crate::udp6_tx::{Udp6Tx, Udp6TxReq};
use std::marker::PhantomData;
use txhdl::comp::trace::Traceable;
use txhdl::comp::{chan, join2, signal, tie, Clock, DefaultClock, In, Out, Reg, Rx, Tx, Unit};
use txhdl::netlist::{Lower, Port};
use txhdl::types::{Bit, U};
use txhdl::types::{Transaction, Value};
use txhdl::{lower, Trace};

/// Reference ID of the golden IPv4 replies (192.0.2.99).
pub const GOLDEN_REF_ID: u32 = 0xC000_0263;
/// Reference ID of the golden IPv6 replies (MD5 of 2001:db8::99).
pub const GOLDEN_REF_ID6: u32 = 0xB70A_6091;
/// Reference timestamp of the golden replies.
pub const GOLDEN_REF_TS: u64 = 0xECC5_8CC0_0000_0000;
/// Root delay / dispersion of the golden replies (16.16).
pub const GOLDEN_ROOT_DELAY: u32 = 0x0000_0333;
pub const GOLDEN_ROOT_DISP: u32 = 0x0000_0666;

// ---------------------------------------------------------------------
// Small helper units
// ---------------------------------------------------------------------

/// A channel source that never sends: the `valid` of its output is
/// constant 0 (a register that stays 0; synthesis removes it). Feeds the
/// inputs that have no producer yet.
#[derive(Trace)]
pub struct Idle<T: Transaction + Value> {
    /// Always 0.
    pub off: Reg<Bit>,
    /// Never sent; gives the output data a driver.
    pub v: Reg<T>,
}

impl<T: Transaction + Value> Default for Idle<T> {
    fn default() -> Self {
        Idle {
            off: Reg::default(),
            v: Reg::default(),
        }
    }
}

#[lower]
impl<T: Transaction + Value> Unit<(), Tx<T>> for Idle<T> {
    async fn run(&mut self, _i: (), out: Tx<T>) {
        loop {
            DefaultClock::rising().await;
            if self.off.get().to_bool() {
                out.send(self.v.get());
            }
        }
    }
}

/// A channel sink that takes everything: `ready` constant 1. Ends the
/// outputs that have no consumer yet.
#[derive(Trace)]
pub struct Drain<T> {
    _t: PhantomData<T>,
}

impl<T> Default for Drain<T> {
    fn default() -> Self {
        Drain { _t: PhantomData }
    }
}

#[lower]
impl<T: Transaction + Value> Unit<Rx<T>, ()> for Drain<T> {
    async fn run(&mut self, inp: Rx<T>, _o: ()) {
        loop {
            DefaultClock::rising().await;
            let _ = inp.recv();
        }
    }
}

// ---------------------------------------------------------------------
// Synchronisation sources
// ---------------------------------------------------------------------

/// Inputs of a synchronisation source: `time` (clock accumulator),
/// `precision`, `poll_log2` (configuration), `samples`, `miss` (from the
/// NTP client, Phase B; idle in Phase A).
pub type SyncIns = (In<U<96>>, In<U<8>>, In<U<8>>, Rx<Sample>, Rx<U<2>>);
/// Outputs: `synced`, `stratum`, `ref_id`, `ref_id6`, `ref_ts`,
/// `root_delay`, `root_disp` (to `ntp_server`), `trim`, `step`, `step_en`
/// (to `DisciplinedClock`).
pub type SyncOuts = (
    Out<Bit>,
    Out<U<8>>,
    Out<U<32>>,
    Out<U<32>>,
    Out<U<64>>,
    Out<U<32>>,
    Out<U<32>>,
    Out<U<32>>,
    Out<U<96>>,
    Out<Bit>,
);

/// A unit that drives the synchronisation wires of the server and the
/// discipline of the clock (see the module documentation).
pub trait SyncSource: Unit<SyncIns, SyncOuts> + Lower + Port + Traceable + Default {}

/// Phase A on the board: never synchronised (LI 3, stratum 16), free
/// running clock (trim 0, no step).
#[derive(Trace, Default)]
pub struct Standalone {}

#[lower]
impl
    Unit<
        (In<U<96>>, In<U<8>>, In<U<8>>, Rx<Sample>, Rx<U<2>>),
        (
            Out<Bit>,
            Out<U<8>>,
            Out<U<32>>,
            Out<U<32>>,
            Out<U<64>>,
            Out<U<32>>,
            Out<U<32>>,
            Out<U<32>>,
            Out<U<96>>,
            Out<Bit>,
        ),
    > for Standalone
{
    async fn run(
        &mut self,
        (_time, _precision, _poll_log2, samples, miss): (
            In<U<96>>,
            In<U<8>>,
            In<U<8>>,
            Rx<Sample>,
            Rx<U<2>>,
        ),
        (synced, stratum, ref_id, ref_id6, ref_ts, root_delay, root_disp, trim, step, step_en): (
            Out<Bit>,
            Out<U<8>>,
            Out<U<32>>,
            Out<U<32>>,
            Out<U<64>>,
            Out<U<32>>,
            Out<U<32>>,
            Out<U<32>>,
            Out<U<96>>,
            Out<Bit>,
        ),
    ) {
        loop {
            DefaultClock::rising().await;
            // Nothing feeds these in Phase A; take whatever comes.
            let _ = samples.recv();
            let _ = miss.recv();
            synced.set(Bit::Zero);
            stratum.set(U::<8>::from(16u8));
            ref_id.set(U::<32>::from(0u32));
            ref_id6.set(U::<32>::from(0u32));
            ref_ts.set(U::<64>::from(0u64));
            root_delay.set(U::<32>::from(0u32));
            root_disp.set(U::<32>::from(0u32));
            trim.set(U::<32>::from(0u32));
            step.set(U::<96>::from(0u32));
            step_en.set(Bit::Zero);
        }
    }
}
impl SyncSource for Standalone {}

/// Simulation: the golden model's server configuration
/// (`tools/golden/vectors/README.md`: reference ids, reference timestamp,
/// root delay and dispersion), free running clock; synchronised at stratum
/// 2 when `SYNCED` ([`FixedSync`]), else `synced = 0`, stratum 16 with the
/// same reference fields ([`FixedUnsync`], the `ntp_unsynchronised` vector).
#[derive(Trace, Default)]
pub struct Fixed<const SYNCED: bool> {}

/// Synchronised at stratum 2 with the golden configuration.
pub type FixedSync = Fixed<true>;
/// Not synchronised, golden reference fields.
pub type FixedUnsync = Fixed<false>;

#[lower]
impl<const SYNCED: bool>
    Unit<
        (In<U<96>>, In<U<8>>, In<U<8>>, Rx<Sample>, Rx<U<2>>),
        (
            Out<Bit>,
            Out<U<8>>,
            Out<U<32>>,
            Out<U<32>>,
            Out<U<64>>,
            Out<U<32>>,
            Out<U<32>>,
            Out<U<32>>,
            Out<U<96>>,
            Out<Bit>,
        ),
    > for Fixed<SYNCED>
{
    async fn run(
        &mut self,
        (_time, _precision, _poll_log2, samples, miss): (
            In<U<96>>,
            In<U<8>>,
            In<U<8>>,
            Rx<Sample>,
            Rx<U<2>>,
        ),
        (synced, stratum, ref_id, ref_id6, ref_ts, root_delay, root_disp, trim, step, step_en): (
            Out<Bit>,
            Out<U<8>>,
            Out<U<32>>,
            Out<U<32>>,
            Out<U<64>>,
            Out<U<32>>,
            Out<U<32>>,
            Out<U<32>>,
            Out<U<96>>,
            Out<Bit>,
        ),
    ) {
        loop {
            DefaultClock::rising().await;
            // Nothing feeds these in Phase A; take whatever comes.
            let _ = samples.recv();
            let _ = miss.recv();
            synced.set(Bit::from_bool(SYNCED));
            stratum.set(U::<8>::from(if SYNCED { 2u8 } else { 16u8 }));
            ref_id.set(U::<32>::from(GOLDEN_REF_ID));
            ref_id6.set(U::<32>::from(GOLDEN_REF_ID6));
            ref_ts.set(U::<64>::from(GOLDEN_REF_TS));
            root_delay.set(U::<32>::from(GOLDEN_ROOT_DELAY));
            root_disp.set(U::<32>::from(GOLDEN_ROOT_DISP));
            trim.set(U::<32>::from(0u32));
            step.set(U::<96>::from(0u32));
            step_en.set(Bit::Zero);
        }
    }
}
impl<const SYNCED: bool> SyncSource for Fixed<SYNCED> {}

// ---------------------------------------------------------------------
// Phase B: the client-side discipline (`WithClient`)
// ---------------------------------------------------------------------

/// Selected fan-out: one `Selected` in, the same to two consumers. Takes
/// an element only when both outputs have room and sends it to both in
/// that cycle (no element is duplicated or lost; a stalled consumer stalls
/// the producer).
#[derive(Trace, Default)]
pub struct ForkSelected {}

#[lower]
impl Unit<Rx<Selected>, (Tx<Selected>, Tx<Selected>)> for ForkSelected {
    async fn run(&mut self, inp: Rx<Selected>, (a, b): (Tx<Selected>, Tx<Selected>)) {
        loop {
            DefaultClock::rising().await;
            let go = Bit::from(inp.peek().is_some()) & a.ready() & b.ready();
            let v = inp.head();
            let _ = inp.recv_if(go);
            if go.to_bool() {
                a.send(v);
                b.send(v);
            }
        }
    }
}

/// Glue of [`ClientSync`]: the 32.32 time (`time >> 32`, registered: one
/// cycle late, irrelevant for the ageing it is used for) for the
/// selection's `now`, and the one Reference ID of the sync state on both
/// of the server's `ref_id` / `ref_id6` (RFC 5905: the reference id names
/// the system peer, whatever the family of the request).
#[derive(Trace, Default)]
pub struct ClientGlue {
    pub now_r: Reg<U<64>>,
}

#[lower]
impl Unit<(In<U<96>>, In<U<32>>), (Out<U<64>>, Out<U<32>>, Out<U<32>>)> for ClientGlue {
    async fn run(
        &mut self,
        (time, ref_in): (In<U<96>>, In<U<32>>),
        (now, ref_id, ref_id6): (Out<U<64>>, Out<U<32>>, Out<U<32>>),
    ) {
        loop {
            DefaultClock::rising().await;
            now.set(self.now_r.get());
            ref_id.set(ref_in.get());
            ref_id6.set(ref_in.get());
            self.now_r.set(time.get().slice::<32, 64>());
        }
    }
}

/// Phase B sync source (`WithClient`): `samples`/`miss` (from the NTP
/// client, T4.2) -> `ClockFilter` -> `Selection` -> fork -> { `Servo` ->
/// `trim`, `step`, `step_en` of the clock; `SyncState` -> `synced`,
/// `stratum`, `ref_id` (= `ref_id6`), `ref_ts`, `root_delay`,
/// `root_disp` of the server }. The servo's `synced` is the sync state's
/// `servo_synced`. `peer_ref0..3` are the reference ids of the
/// configuration's upstream slots, computed at synthesis
/// (`refid_ipv4`/`refid_ipv6` of `ConfigSet::UPSTREAM[i]`). Every link is a
/// registered channel or a register-driven wire (filter, selection, servo
/// and sync state have registered outputs, their headers).
///
/// Until the client exists its `samples`/`miss` are fed by [`Idle`]
/// units in [`ArtyCore`]: the filter never gets a sample, the selection
/// never runs, the servo keeps trim 0 and the state stays unsynchronised
/// (stratum 16), i.e. the same behaviour as [`Standalone`]. See
/// `docs/top-client-hooks.md` for the client's side of the wiring.
#[derive(Trace)]
pub struct ClientSync<C: ConfigSet> {
    pub filter: ClockFilter,
    pub selection: Selection,
    pub fork: ForkSelected,
    pub servo: Servo,
    pub state: SyncState,
    pub glue: ClientGlue,
    _c: PhantomData<C>,
}

impl<C: ConfigSet> Default for ClientSync<C> {
    fn default() -> Self {
        ClientSync {
            filter: Default::default(),
            selection: Default::default(),
            fork: Default::default(),
            servo: Default::default(),
            state: Default::default(),
            glue: Default::default(),
            _c: PhantomData,
        }
    }
}

/// Our Reference ID if upstream slot `i` of `C` is the system peer.
pub const fn peer_ref<C: ConfigSet>(i: usize) -> u32 {
    let (v6, addr, _) = C::UPSTREAM[i];
    if v6 {
        refid_ipv6(addr)
    } else {
        refid_ipv4(addr as u32)
    }
}

#[lower]
impl<C: ConfigSet>
    Unit<
        (In<U<96>>, In<U<8>>, In<U<8>>, Rx<Sample>, Rx<U<2>>),
        (
            Out<Bit>,
            Out<U<8>>,
            Out<U<32>>,
            Out<U<32>>,
            Out<U<64>>,
            Out<U<32>>,
            Out<U<32>>,
            Out<U<32>>,
            Out<U<96>>,
            Out<Bit>,
        ),
    > for ClientSync<C>
{
    async fn run(
        &mut self,
        (time, precision, poll_log2, samples, miss): (
            In<U<96>>,
            In<U<8>>,
            In<U<8>>,
            Rx<Sample>,
            Rx<U<2>>,
        ),
        (synced, stratum, ref_id, ref_id6, ref_ts, root_delay, root_disp, trim, step, step_en): (
            Out<Bit>,
            Out<U<8>>,
            Out<U<32>>,
            Out<U<32>>,
            Out<U<64>>,
            Out<U<32>>,
            Out<U<32>>,
            Out<U<32>>,
            Out<U<96>>,
            Out<Bit>,
        ),
    ) {
        let (st_tx, st_rx) = chan::<PeerStat, DefaultClock>();
        let (sel_tx, sel_rx) = chan::<Selected, DefaultClock>();
        let (sv_tx, sv_rx) = chan::<Selected, DefaultClock>();
        let (ss_tx, ss_rx) = chan::<Selected, DefaultClock>();
        let (now_o, now) = signal::<U<64>, DefaultClock>();
        let (ref_o, ref_i) = signal::<U<32>, DefaultClock>();
        let (state_o, _state) = signal::<U<3>, DefaultClock>();
        let (ssync_o, ssync) = signal::<Bit, DefaultClock>();
        let (leap_o, _leap) = signal::<U<2>, DefaultClock>();
        let time_b = time.clone();
        let time_c = time.clone();
        join2(
            join2(
                join2(
                    self.filter.run((samples, miss), st_tx),
                    self.selection.run((st_rx, now), sel_tx),
                ),
                join2(
                    self.fork.run(sel_rx, (sv_tx, ss_tx)),
                    self.glue.run((time, ref_i), (now_o, ref_id, ref_id6)),
                ),
            ),
            join2(
                self.servo.run(
                    (sv_rx, poll_log2, time_b),
                    (trim, step, step_en, state_o, ssync_o),
                ),
                self.state.run(
                    (
                        ss_rx,
                        time_c,
                        ssync,
                        precision,
                        tie(U::<32>::new(peer_ref::<C>(0) as u128)),
                        tie(U::<32>::new(peer_ref::<C>(1) as u128)),
                        tie(U::<32>::new(peer_ref::<C>(2) as u128)),
                        tie(U::<32>::new(peer_ref::<C>(3) as u128)),
                    ),
                    (
                        synced, stratum, ref_o, ref_ts, root_delay, root_disp, leap_o,
                    ),
                ),
            ),
        )
        .await;
    }
}
impl<C: ConfigSet> SyncSource for ClientSync<C> {}

/// The board build with the client-side discipline wired (Phase B): the
/// same as [`ArtyBoard`] until the NTP client feeds `samples`/`miss`.
pub type ArtyBoardWithClient =
    ArtyTop<crate::config::Board, ClientSync<crate::config::Board>, crate::phy_init::Board>;

// ---------------------------------------------------------------------
// The core: everything between the MII byte streams
// ---------------------------------------------------------------------

/// `tx_path` of the design: 4096-byte memory, `clk_sys` to `T`.
pub type TxPathOf<T> = TxPathG<12, 4096, 13, DefaultClock, T>;

/// The server path between the MII byte streams, generic in the two MII
/// clocks (`R` receive, `T` transmit), the configuration `C` and the
/// synchronisation source `S`.
///
/// Ports: `raw` (`Rx<FrameByte, R>`, `mii_rx.frames`), `rx_sfd`
/// (`In<Bit, R>`, `mii_rx.sfd`), `tx_sfd`, `tx_underrun` (`In<Bit, T>`,
/// `mii_tx.sfd`, `.underrun`) in; `tx_frames` (`Tx<FrameByte, T>`, to
/// `mii_tx.frames`) out.
#[derive(Trace)]
pub struct ArtyCore<R: Clock, T: Clock, C: ConfigSet, S: SyncSource> {
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
    pub server: NtpServer,
    pub route: NtpReplyRoute,
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
    pub clock: DisciplinedClock,
    pub sync: S,
    pub idle_resolve: Idle<U<32>>,
    pub idle_lookup: Idle<U<32>>,
    pub idle_h3: Idle<EthHdr>,
    pub idle_p3: Idle<FrameByte>,
    pub idle_nd_resolve: Idle<U<128>>,
    pub drain_nd_learn: Drain<NdLearn>,
    pub idle_samples: Idle<Sample>,
    pub idle_miss: Idle<U<2>>,
    pub drain_lookup: Drain<ArpEntry>,
    pub drain_stamps: Drain<Ts>,
}

/// The output arbiter (four sources).
pub type TxArbOf = crate::tx_arb::TxArb;

impl<R: Clock, T: Clock, C: ConfigSet, S: SyncSource> Default for ArtyCore<R, T, C, S> {
    fn default() -> Self {
        ArtyCore {
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
            server: Default::default(),
            route: Default::default(),
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
            idle_resolve: Default::default(),
            idle_lookup: Default::default(),
            idle_h3: Default::default(),
            idle_p3: Default::default(),
            idle_nd_resolve: Default::default(),
            drain_nd_learn: Default::default(),
            idle_samples: Default::default(),
            idle_miss: Default::default(),
            drain_lookup: Default::default(),
            drain_stamps: Default::default(),
        }
    }
}

#[lower]
impl<R: Clock, T: Clock, C: ConfigSet, S: SyncSource>
    Unit<(Rx<FrameByte, R>, In<Bit, R>, In<Bit, T>, In<Bit, T>), Tx<FrameByte, T>>
    for ArtyCore<R, T, C, S>
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
        let (gateway_o, _gateway) = signal::<U<32>, DefaultClock>();
        let (gateway6_o, _gateway6) = signal::<U<128>, DefaultClock>();
        let (netmask_o, _netmask) = signal::<U<32>, DefaultClock>();
        let (up0_addr_o, _up0_addr) = signal::<U<128>, DefaultClock>();
        let (up0_v6_o, _up0_v6) = signal::<Bit, DefaultClock>();
        let (up0_en_o, _up0_en) = signal::<Bit, DefaultClock>();
        let (up1_addr_o, _up1_addr) = signal::<U<128>, DefaultClock>();
        let (up1_v6_o, _up1_v6) = signal::<Bit, DefaultClock>();
        let (up1_en_o, _up1_en) = signal::<Bit, DefaultClock>();
        let (up2_addr_o, _up2_addr) = signal::<U<128>, DefaultClock>();
        let (up2_v6_o, _up2_v6) = signal::<Bit, DefaultClock>();
        let (up2_en_o, _up2_en) = signal::<Bit, DefaultClock>();
        let (up3_addr_o, _up3_addr) = signal::<U<128>, DefaultClock>();
        let (up3_v6_o, _up3_v6) = signal::<Bit, DefaultClock>();
        let (up3_en_o, _up3_en) = signal::<Bit, DefaultClock>();
        let (poll_log2_o, poll_log2) = signal::<U<8>, DefaultClock>();
        let (precision_o, precision) = signal::<U<8>, DefaultClock>();
        let (stratum_offset_o, _stratum_offset) = signal::<U<8>, DefaultClock>();
        let (rate_limit_o, rate_limit) = signal::<U<16>, DefaultClock>();
        let local_mac_b = local_mac.clone();
        let local_mac_c = local_mac.clone();
        let local_mac_d = local_mac.clone();
        let local_ip4_b = local_ip4.clone();
        let local_ip4_c = local_ip4.clone();
        let local_ip4_d = local_ip4.clone();
        let local_ll_b = local_ll.clone();
        let local_gua_b = local_gua.clone();
        let gua_valid_b = gua_valid.clone();
        let sn_low_b = sn_low.clone();
        let precision_b = precision.clone();

        // Clock and synchronisation wires.
        let (time_o, time) = signal::<U<96>, DefaultClock>();
        let (stepped_o, _stepped) = signal::<Bit, DefaultClock>();
        let time_b = time.clone();
        let time_c = time.clone();
        let time_d = time.clone();
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

        // Transmit path.
        let (rm_tx, rm_rx) = chan::<NtpReplyMeta, DefaultClock>();
        let (rp_tx, rp_rx) = chan::<FrameByte, DefaultClock>();
        let (q4_tx, q4_rx) = chan::<UdpTxReq, DefaultClock>();
        let (y4_tx, y4_rx) = chan::<FrameByte, DefaultClock>();
        let (q6_tx, q6_rx) = chan::<Udp6TxReq, DefaultClock>();
        let (y6_tx, y6_rx) = chan::<FrameByte, DefaultClock>();
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

        // Unused ends and client hooks.
        let (res_tx, res_rx) = chan::<U<32>, DefaultClock>();
        // IPv6 neighbor discovery hooks of `icmpv6` (T3.6): no resolver in the
        // server-only top (Phase A), the learned neighbors are drained.
        let (ndres_tx, ndres_rx) = chan::<U<128>, DefaultClock>();
        let (ndl_tx, ndl_rx) = chan::<NdLearn, DefaultClock>();
        let (lk_tx, lk_rx) = chan::<U<32>, DefaultClock>();
        let (lr_tx, lr_rx) = chan::<ArpEntry, DefaultClock>();
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
                                stratum_offset_o,
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
                        self.server.run(
                            (
                                n4m_rx,
                                n4p_rx,
                                n6m_rx,
                                n6p_rx,
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
                                .run((rm_rx, rp_rx), (q4_tx, y4_tx, q6_tx, y6_tx, tx_latency_o)),
                            join2(
                                self.udp4_tx
                                    .run((q4_rx, y4_rx, local_ip4_c), (ut4r_tx, ut4p_tx)),
                                self.udp6_tx.run((q6_rx, y6_rx), (ut6r_tx, ut6p_tx)),
                            ),
                        ),
                    ),
                ),
                join2(
                    join2(
                        join2(
                            self.arb4
                                .run((ie4r_rx, ie4p_rx, ut4r_rx, ut4p_rx), (a4r_tx, a4p_tx)),
                            self.arb6
                                .run((ie6r_rx, ie6p_rx, ut6r_rx, ut6p_rx), (a6r_tx, a6p_tx)),
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
                            join2(
                                join2(
                                    self.idle_resolve.run((), res_tx),
                                    self.idle_lookup.run((), lk_tx),
                                ),
                                join2(self.idle_h3.run((), h3_tx), self.idle_p3.run((), p3_tx)),
                            ),
                            join2(
                                join2(
                                    self.idle_samples.run((), smp_tx),
                                    self.idle_miss.run((), miss_tx),
                                ),
                                join2(
                                    join2(
                                        self.drain_lookup.run(lr_rx, ()),
                                        self.drain_stamps.run(st_rx, ()),
                                    ),
                                    join2(
                                        self.idle_nd_resolve.run((), ndres_tx),
                                        self.drain_nd_learn.run(ndl_rx, ()),
                                    ),
                                ),
                            ),
                        ),
                    ),
                ),
            ),
        )
        .await;
    }
}

// ---------------------------------------------------------------------
// The top: MII units, PHY management, core
// ---------------------------------------------------------------------

/// The design as the board wrapper instantiates it (module `arty_top`).
///
/// Ports: `rxd` (`In<U<4>, ClkRx>`), `rx_dv`, `rx_er` (`In<Bit, ClkRx>`),
/// `mdio_i` (`In<Bit>`) in; `txd` (`Out<U<4>, ClkTx>`), `tx_en`
/// (`Out<Bit, ClkTx>`), `phy_rstn`, `mdc`, `mdio_o`, `mdio_oe`, `link_up`,
/// `id_ok`, `phy_ready` (`Out<Bit>`, `clk_sys`) out. `txd`/`tx_en` come
/// straight from `mii_tx`'s registers (IOB-packable); `rxd`/`rx_dv`/`rx_er`
/// go straight into `mii_rx`. The netlist adds the clock inputs `clk`
/// (`clk_sys`), `clk_rx`, `clk_tx` and `rst`.
#[derive(Trace)]
pub struct ArtyTop<C: ConfigSet, S: SyncSource, P: PhyTiming> {
    pub mii_rx: MiiRx,
    pub mii_tx: MiiTx,
    pub phy: PhyInit<P>,
    pub core: ArtyCore<ClkRx, ClkTx, C, S>,
}

impl<C: ConfigSet, S: SyncSource, P: PhyTiming> Default for ArtyTop<C, S, P> {
    fn default() -> Self {
        ArtyTop {
            mii_rx: Default::default(),
            mii_tx: Default::default(),
            phy: Default::default(),
            core: Default::default(),
        }
    }
}

/// The board build: `Board` addresses, never synchronised (Phase A), real
/// PHY timings.
pub type ArtyBoard = ArtyTop<crate::config::Board, Standalone, crate::phy_init::Board>;

#[lower]
impl<C: ConfigSet, S: SyncSource, P: PhyTiming>
    Unit<
        (In<U<4>, ClkRx>, In<Bit, ClkRx>, In<Bit, ClkRx>, In<Bit>),
        (
            Out<U<4>, ClkTx>,
            Out<Bit, ClkTx>,
            Out<Bit>,
            Out<Bit>,
            Out<Bit>,
            Out<Bit>,
            Out<Bit>,
            Out<Bit>,
            Out<Bit>,
        ),
    > for ArtyTop<C, S, P>
{
    async fn run(
        &mut self,
        (rxd, rx_dv, rx_er, mdio_i): (In<U<4>, ClkRx>, In<Bit, ClkRx>, In<Bit, ClkRx>, In<Bit>),
        (txd, tx_en, phy_rstn, mdc, mdio_o, mdio_oe, link_up, id_ok, phy_ready): (
            Out<U<4>, ClkTx>,
            Out<Bit, ClkTx>,
            Out<Bit>,
            Out<Bit>,
            Out<Bit>,
            Out<Bit>,
            Out<Bit>,
            Out<Bit>,
            Out<Bit>,
        ),
    ) {
        let (raw_tx, raw_rx) = chan::<FrameByte, ClkRx>();
        let (rsfd_o, rsfd) = signal::<Bit, ClkRx>();
        let (txf_tx, txf_rx) = chan::<FrameByte, ClkTx>();
        let (tsfd_o, tsfd) = signal::<Bit, ClkTx>();
        let (und_o, und) = signal::<Bit, ClkTx>();
        join2(
            join2(
                self.mii_rx.run((rxd, rx_dv, rx_er), (raw_tx, rsfd_o)),
                self.mii_tx.run(txf_rx, (txd, tx_en, tsfd_o, und_o)),
            ),
            join2(
                self.phy.run(
                    (tie(U::<5>::new(1)), mdio_i),
                    (phy_rstn, mdc, mdio_o, mdio_oe, link_up, id_ok, phy_ready),
                ),
                self.core.run((raw_rx, rsfd, tsfd, und), txf_tx),
            ),
        )
        .await;
    }
}

/// Netlist guard for the board build (used by `top_gen`).
///
/// A lowered unit that names a local value `rst` gets a `wire rst` in a module whose
/// reset input is also `rst`: a redeclaration Vivado rejects. `icmp_echo` and
/// `icmpv6` used to do so; they now call it `end_all`, so this finds NOTHING to fix.
/// It is kept as a guard: if a unit reintroduces the clash, it renames the local wire
/// to `rst_l` (its declaration, its `assign`, and the guarded `if (rst)` reads TxHDL
/// writes at indent 6) and the build FAILS LOUDLY (assert) so the unit gets fixed at
/// the source; any other use of `rst` in such a module panics rather than guesses.
pub fn fix_rst_clash(v: &str) -> String {
    let mut out = String::with_capacity(v.len() + 64);
    let mut fixed = 0;
    for chunk in v.split_inclusive("endmodule\n") {
        if !chunk.contains("\n  wire rst;\n") {
            out.push_str(chunk);
            continue;
        }
        let mut m = String::with_capacity(chunk.len());
        for line in chunk.split_inclusive('\n') {
            let l = match line.trim_end_matches('\n') {
                "  wire rst;" => "  wire rst_l;\n".to_string(),
                "      if (rst) begin" => "      if (rst_l) begin\n".to_string(),
                t if t.starts_with("  assign rst = ") => {
                    line.replacen("assign rst =", "assign rst_l =", 1)
                }
                "  input rst," | "    if (rst) begin" => line.to_string(),
                t => {
                    let has = t
                        .split(|c: char| !(c.is_ascii_alphanumeric() || c == '_'))
                        .any(|w| w == "rst");
                    assert!(
                        !has,
                        "unexpected use of rst in a module with a local rst: {t}"
                    );
                    line.to_string()
                }
            };
            m.push_str(&l);
        }
        fixed += 1;
        out.push_str(&m);
    }
    assert_eq!(
        fixed, 0,
        "a unit names a local value `rst`: rename it at the source (see fix_rst_clash)"
    );
    out
}
