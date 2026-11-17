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
