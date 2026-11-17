// SPDX-License-Identifier: Apache-2.0
//! `gen_all <module>`: print the Verilog of a design module.
use ntp_fpga::{
    clock::{DisciplinedClock, NtpClock},
    crc32::FrameCrc,
    eth_rx::EthRx,
    eth_tx::EthTx,
    fcs_check::FcsCheck,
    mii_rx::MiiRx,
    mii_tx::MiiTx,
    stream::FrameCounter,
    tx_arb::TxArb,
    tx_path::TxPath,
};

fn main() {
    match std::env::args().nth(1).as_deref() {
        Some("arp") => print!("{}", ntp_fpga::arp::Arp::verilog("arp")),
        Some("ipv4_rx") => print!("{}", ntp_fpga::ipv4_rx::Ipv4Rx::verilog("ipv4_rx")),
        Some("ipv4_tx") => print!("{}", ntp_fpga::ipv4_tx::Ipv4Tx::verilog("ipv4_tx")),
        Some("icmp_echo") => print!("{}", ntp_fpga::icmp_echo::IcmpEcho::verilog("icmp_echo")),
        Some("config_board") => print!(
            "{}",
            ntp_fpga::config::Config::<ntp_fpga::config::Board>::verilog("config_board")
        ),
        Some("udp4_rx") => print!("{}", ntp_fpga::udp4_rx::Udp4Rx::verilog("udp4_rx")),
        Some("udp4_tx") => print!("{}", ntp_fpga::udp4_tx::Udp4Tx::verilog("udp4_tx")),
        Some("ipv6_rx") => print!("{}", ntp_fpga::ipv6_rx::Ipv6Rx::verilog("ipv6_rx")),
        Some("ipv6_tx") => print!("{}", ntp_fpga::ipv6_tx::Ipv6Tx::verilog("ipv6_tx")),
        Some("icmpv6") => print!("{}", ntp_fpga::icmpv6::Icmpv6::verilog("icmpv6")),
        Some("ntp_server") => print!("{}", ntp_fpga::ntp_server::NtpServer::verilog("ntp_server")),
        Some("udp6_rx") => print!("{}", ntp_fpga::udp6_rx::Udp6Rx::verilog("udp6_rx")),
        Some("udp6_tx") => print!("{}", ntp_fpga::udp6_tx::Udp6Tx::verilog("udp6_tx")),
        Some("clock_filter") => print!("{}", ntp_fpga::clock_filter::ClockFilter::verilog("clock_filter")),
        Some("sync_state") => print!("{}", ntp_fpga::sync_state::SyncState::verilog("sync_state")),
        Some("ntp_clock") => print!("{}", NtpClock::verilog("ntp_clock")),
        Some("disciplined_clock") => print!("{}", DisciplinedClock::verilog("disciplined_clock")),
        Some("frame_counter") => print!("{}", FrameCounter::verilog("frame_counter")),
        Some("frame_crc") => print!("{}", FrameCrc::verilog("frame_crc")),
        Some("csum") => print!("{}", ntp_fpga::csum::Csum::verilog("csum")),
        Some("mii_rx") => print!("{}", MiiRx::verilog("mii_rx")),
        Some("mii_tx") => print!("{}", MiiTx::verilog("mii_tx")),
        Some("eth_rx") => print!("{}", EthRx::verilog("eth_rx")),
        Some("eth_tx") => print!("{}", EthTx::verilog("eth_tx")),
        Some("fcs_check") => print!("{}", FcsCheck::verilog("fcs_check")),
        Some("tx_arb") => print!("{}", TxArb::verilog("tx_arb")),
        Some("tx_path") => print!("{}", TxPath::verilog("tx_path")),
        Some("rx_path") => print!("{}", ntp_fpga::rx_path::RxPathMii::verilog("rx_path")),
        Some("phy_init") => print!(
            "{}",
            ntp_fpga::phy_init::PhyInit::<ntp_fpga::phy_init::Board>::verilog("phy_init")
        ),
        other => {
            eprintln!("unknown module {other:?}");
            std::process::exit(1);
        }
    }
}
