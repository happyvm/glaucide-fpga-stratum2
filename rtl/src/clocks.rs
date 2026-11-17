// SPDX-License-Identifier: Apache-2.0
//! Clock domains of the design. `DefaultClock` is `clk_sys` (100 MHz). The
//! PHY's MII clocks are inputs, 25 MHz each at 100 Mb/s, unrelated to
//! `clk_sys` and to each other: crossing goes through `ChanCdc` (T1.4).
use txhdl::comp::Clock;

/// PHY receive clock (`eth_rx_clk`), 25 MHz.
pub struct ClkRx;
impl Clock for ClkRx {
    const NAME: &'static str = "clk_rx";
}

/// PHY transmit clock (`eth_tx_clk`), 25 MHz.
pub struct ClkTx;
impl Clock for ClkTx {
    const NAME: &'static str = "clk_tx";
}

// Simulation-only variants for the pin-level system tests. The Verilog
// netlist does not depend on `PERIOD`/`PHASE` (they only place the edges in
// simulation), so the board keeps `ClkRx`/`ClkTx`; a design instantiated
// with these types behaves the same but is simulated with the REAL ratio.
// Units: one tick is 5 ns (`clk_sys` = `DefaultClock` = 2 ticks).

/// `eth_rx_clk` at 25 MHz as simulated: 8 ticks (40 ns, 4 `clk_sys`
/// periods), rising edges at odd ticks (between `clk_sys` edges).
pub struct ClkRx25;
impl Clock for ClkRx25 {
    const NAME: &'static str = "clk_rx";
    const PERIOD: u64 = 8;
    const PHASE: u64 = 1;
}

/// `eth_tx_clk` at 25 MHz as simulated: 8 ticks, edges at odd ticks, 10 ns
/// after [`ClkRx25`]'s.
pub struct ClkTx25;
impl Clock for ClkTx25 {
    const NAME: &'static str = "clk_tx";
    const PERIOD: u64 = 8;
    const PHASE: u64 = 3;
}

/// Off-ratio receive clock (45 ns), for the tolerance tests.
pub struct ClkRx45;
impl Clock for ClkRx45 {
    const NAME: &'static str = "clk_rx";
    const PERIOD: u64 = 9;
    const PHASE: u64 = 3;
}

/// Off-ratio transmit clock (35 ns), for the tolerance tests.
pub struct ClkTx35;
impl Clock for ClkTx35 {
    const NAME: &'static str = "clk_tx";
    const PERIOD: u64 = 7;
    const PHASE: u64 = 1;
}

/// A 25 MHz receive clock whose first edge is at tick `P` (0..8, 5 ns
/// steps): the phase sweeps of the timestamp measurements.
pub struct ClkRxP<const P: u64>;
impl<const P: u64> Clock for ClkRxP<P> {
    const NAME: &'static str = "clk_rx";
    const PERIOD: u64 = 8;
    const PHASE: u64 = P;
}

/// A 25 MHz transmit clock whose first edge is at tick `P` (0..8).
pub struct ClkTxP<const P: u64>;
impl<const P: u64> Clock for ClkTxP<P> {
    const NAME: &'static str = "clk_tx";
    const PERIOD: u64 = 8;
    const PHASE: u64 = P;
}
