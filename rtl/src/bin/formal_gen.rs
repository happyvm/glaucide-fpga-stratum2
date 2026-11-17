// SPDX-License-Identifier: Apache-2.0
//! `formal_gen <unit>`: print the Verilog of a unit that carries
//! `check!`/`assume!`/`cover!` statements (under `ifdef FORMAL`).
use ntp_fpga::formal_smoke::Digit;
use ntp_fpga::mii_rx::MiiRx;

fn main() {
    match std::env::args().nth(1).as_deref() {
        Some("formal_smoke") => print!("{}", Digit::verilog("formal_smoke")),
        Some("mii_rx") => print!("{}", MiiRx::verilog("mii_rx")),
        other => {
            eprintln!("unknown unit {other:?}");
            std::process::exit(1);
        }
    }
}
