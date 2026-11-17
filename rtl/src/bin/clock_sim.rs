// SPDX-License-Identifier: Apache-2.0
//! `clock_sim`: run 1 ms of the clock, check it advanced 1 ms; or `--verilog`.
use ntp_fpga::clock::{ntp_time, NtpClock};
use txhdl::comp::{signal, DefaultClock, Running, Unit};
use txhdl::types::U;

fn main() {
    if std::env::args().any(|a| a == "--verilog") {
        print!("{}", NtpClock::verilog("ntp_clock"));
        return;
    }
    let (drive, time) = signal::<U<96>, DefaultClock>();
    let mut c = NtpClock::default();
    let mut sim = Running::new(c.run((), drive));
    let cycles = 100_000u64; // 1 ms at 100 MHz
    for _ in 0..cycles {
        sim.cycle();
    }
    let t = ntp_time(time.get().raw());
    let expect = (1u64 << 32) / 1000;
    let err = t as i64 - expect as i64;
    println!("after 1 ms: {t} (expected ~{expect}), error {err} LSB of 2^-32 s");
    // One cycle is 2^32/1e8 = 42.95 LSB; the registered output lags by one.
    assert!(err.abs() <= 86, "clock drift too large");
}
