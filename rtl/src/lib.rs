// SPDX-License-Identifier: Apache-2.0
//! NTP server on Arty A7-100T, written in TxHDL.
pub mod clock;
pub mod clocks;
pub mod crc32;
pub mod crc_equiv;
pub mod csum;
pub mod eth_rx;
pub mod eth_tx;
pub mod fcs_check;
pub mod formal_smoke;
pub mod mii_rx;
pub mod mii_tx;
pub mod rx_path;
pub mod stream;
pub mod tx_arb;
pub mod tx_path;
