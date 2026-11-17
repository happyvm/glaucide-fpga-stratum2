// SPDX-License-Identifier: Apache-2.0
//! `formal_gen <unit>`: print the Verilog of a unit that carries
//! `check!`/`assume!`/`cover!` statements (under `ifdef FORMAL`).
use ntp_fpga::{
    crc32::FrameCrc, eth_rx::EthRx, eth_tx::EthTx, fcs_check::FcsCheck, formal_smoke::Digit,
    mii_rx::MiiRx, mii_tx::MiiTx, tx_arb::TxArb, tx_path::TxPathFormal,
};

fn main() {
    match std::env::args().nth(1).as_deref() {
        Some("formal_smoke") => print!("{}", Digit::verilog("formal_smoke")),
        Some("frame_crc") => print!("{}", FrameCrc::verilog("frame_crc")),
        Some("csum") => print!("{}", ntp_fpga::csum::Csum::verilog("csum")),
        Some("mii_rx") => print!("{}", MiiRx::verilog("mii_rx")),
        Some("mii_tx") => print!("{}", MiiTx::verilog("mii_tx")),
        Some("eth_rx") => print!("{}", EthRx::verilog("eth_rx")),
        Some("eth_tx") => print!("{}", EthTx::verilog("eth_tx")),
        Some("fcs_check") => print!("{}", FcsCheck::verilog("fcs_check")),
        Some("tx_arb") => print!("{}", TxArb::verilog("tx_arb")),
        // 16-byte buffer instance: the proof does not need the real depth.
        Some("tx_path") => print!("{}", TxPathFormal::verilog("tx_path")),
        other => {
            eprintln!("unknown unit {other:?}");
            std::process::exit(1);
        }
    }
}
