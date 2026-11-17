// SPDX-License-Identifier: Apache-2.0
//! `gen_all <module>`: print the Verilog of a design module.
use ntp_fpga::{clock::NtpClock, crc32::FrameCrc, fcs_check::FcsCheck, mii_rx::MiiRx, stream::FrameCounter};

fn main() {
    match std::env::args().nth(1).as_deref() {
        Some("ntp_clock") => print!("{}", NtpClock::verilog("ntp_clock")),
        Some("frame_counter") => print!("{}", FrameCounter::verilog("frame_counter")),
        Some("frame_crc") => print!("{}", FrameCrc::verilog("frame_crc")),
        Some("mii_rx") => print!("{}", MiiRx::verilog("mii_rx")),
        Some("fcs_check") => print!("{}", FcsCheck::verilog("fcs_check")),
        other => {
            eprintln!("unknown module {other:?}");
            std::process::exit(1);
        }
    }
}
