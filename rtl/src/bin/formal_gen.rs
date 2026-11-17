// SPDX-License-Identifier: Apache-2.0
//! `formal_gen <unit>`: print the Verilog of a unit that carries
//! `check!`/`assume!`/`cover!` statements (under `ifdef FORMAL`).
use ntp_fpga::{
    clock::DisciplinedClock, crc32::FrameCrc, eth_rx::EthRx, eth_tx::EthTx, fcs_check::FcsCheck,
    formal_smoke::Digit, mii_rx::MiiRx, mii_tx::MiiTx, tx_arb::TxArb, tx_path::TxPathFormal,
};

fn main() {
    match std::env::args().nth(1).as_deref() {
        Some("arp") => print!("{}", ntp_fpga::arp::Arp::verilog("arp")),
        Some("ipv4_rx") => print!("{}", ntp_fpga::ipv4_rx::Ipv4Rx::verilog("ipv4_rx")),
        Some("ipv4_tx") => print!("{}", ntp_fpga::ipv4_tx::Ipv4Tx::verilog("ipv4_tx")),
        Some("icmp_echo") => print!("{}", ntp_fpga::icmp_echo::IcmpEcho::verilog("icmp_echo")),
        Some("cfg") => print!(
            "{}",
            ntp_fpga::config::Config::<ntp_fpga::config::Board>::verilog("cfg")
        ),
        Some("formal_smoke") => print!("{}", Digit::verilog("formal_smoke")),
        Some("crc_equiv") => print!("{}", ntp_fpga::crc_equiv::CrcEquiv::verilog("crc_equiv")),
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
        Some(u @ ("rx_ingress" | "rx_egress")) => {
            print!("{}", ntp_fpga::rx_path::formal_verilog(u))
        }
        // Shrunken timing instance: same logic, short delays.
        Some("phy_init") => print!(
            "{}",
            ntp_fpga::phy_init::PhyInit::<ntp_fpga::phy_init::Formal>::verilog("phy_init")
        ),
        Some("disciplined_clock") => print!("{}", DisciplinedClock::verilog("disciplined_clock")),
        other => {
            eprintln!("unknown unit {other:?}");
            std::process::exit(1);
        }
    }
}
