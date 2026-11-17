// SPDX-License-Identifier: Apache-2.0
//! `gen_all <module>`: print the Verilog of a design module.
use ntp_fpga::{
    clock::NtpClock, crc32::FrameCrc, eth_rx::EthRx, eth_tx::EthTx, fcs_check::FcsCheck,
    mii_rx::MiiRx, mii_tx::MiiTx, stream::FrameCounter, tx_arb::TxArb, tx_path::TxPath,
};

fn main() {
    match std::env::args().nth(1).as_deref() {
        Some("ntp_clock") => print!("{}", NtpClock::verilog("ntp_clock")),
        Some("frame_counter") => print!("{}", FrameCounter::verilog("frame_counter")),
        Some("frame_crc") => print!("{}", FrameCrc::verilog("frame_crc")),
        Some("mii_rx") => print!("{}", MiiRx::verilog("mii_rx")),
        Some("mii_tx") => print!("{}", MiiTx::verilog("mii_tx")),
        Some("eth_rx") => print!("{}", EthRx::verilog("eth_rx")),
        Some("eth_tx") => print!("{}", EthTx::verilog("eth_tx")),
        Some("fcs_check") => print!("{}", FcsCheck::verilog("fcs_check")),
        Some("tx_arb") => print!("{}", TxArb::verilog("tx_arb")),
        Some("tx_path") => print!("{}", TxPath::verilog("tx_path")),
        other => {
            eprintln!("unknown module {other:?}");
            std::process::exit(1);
        }
    }
}
