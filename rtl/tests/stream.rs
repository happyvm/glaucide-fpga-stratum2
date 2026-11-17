// SPDX-License-Identifier: Apache-2.0
//! Reference producer/consumer of the frame-stream contract (T0.1).
use ntp_fpga::stream::{FrameByte, FrameCounter, MAX_FRAME};
use txhdl::comp::{chan, DefaultClock, Running, Unit};
use txhdl::types::U;

fn run(frames: &[usize]) -> Vec<u128> {
    let (tx, rx) = chan::<FrameByte, DefaultClock>();
    let (lens_tx, lens) = chan::<U<16>, DefaultClock>();
    let mut c = FrameCounter::default();
    let mut sim = Running::new(c.run(rx, lens_tx));
    let mut bytes: Vec<FrameByte> = Vec::new();
    for &n in frames {
        for i in 0..n {
            bytes.push(FrameByte::new(i as u8, i + 1 == n, false));
        }
    }
    let mut next = 0;
    let mut got = Vec::new();
    for _ in 0..bytes.len() + 10 {
        if next < bytes.len() && tx.ready().to_bool() {
            tx.send(bytes[next]);
            next += 1;
        }
        if let Some(l) = lens.recv() {
            got.push(l.raw());
        }
        sim.cycle();
    }
    got
}

#[test]
fn frame_lengths_are_reported() {
    assert_eq!(run(&[60, 1, MAX_FRAME, 64]), vec![60, 1, MAX_FRAME as u128, 64]);
}

#[test]
fn frame_byte_roundtrip() {
    let b = FrameByte::new(0xAB, true, true);
    assert_eq!(b.data.raw(), 0xAB);
    assert!(b.last.to_bool() && b.err.to_bool());
}
