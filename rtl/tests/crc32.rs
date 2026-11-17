// SPDX-License-Identifier: Apache-2.0
//! Tests of the Ethernet CRC-32 (T1.3): `crc32_step` and `FrameCrc`.
//! Expected values come from Python's `zlib.crc32`, not from this code.
use ntp_fpga::crc32::{crc32_step, Crc, FrameCrc, INIT, POLY};
use ntp_fpga::stream::FrameByte;
use txhdl::comp::{chan, DefaultClock, Running, Unit};
use txhdl::types::U;

/// A 60-byte frame: destination and source in the `02:00:00:00:00:xx`
/// fixture range, EtherType 0x0800, a 46-byte payload.
fn frame60() -> Vec<u8> {
    let mut f = vec![
        0x02, 0, 0, 0, 0, 0x02, // destination
        0x02, 0, 0, 0, 0, 0x01, // source
        0x08, 0x00, // EtherType
    ];
    f.extend((0..46).map(|i: u32| ((i * 7 + 3) & 0xFF) as u8));
    f
}

/// The FCS of `frame60()`: `zlib.crc32` is 0x350AA502, sent least
/// significant byte first.
const FCS60: [u8; 4] = [0x02, 0xa5, 0x0a, 0x35];

/// `zlib.crc32` of `frame60()` followed by `FCS60`: the residue, complemented.
const RESIDUE_ZLIB: u32 = 0x2144_DF1C;

/// Bitwise reference, written independently of the unit: one bit at a
/// time, least significant bit first, reflected polynomial.
fn reference_crc(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for &byte in data {
        crc ^= byte as u32;
        for _ in 0..8 {
            crc = if crc & 1 == 1 {
                (crc >> 1) ^ POLY
            } else {
                crc >> 1
            };
        }
    }
    !crc
}

/// Fixed-seed LCG (Knuth's MMIX constants), top byte of each state.
fn lcg_bytes(n: usize) -> Vec<u8> {
    let mut s: u64 = 0x0123_4567_89AB_CDEF;
    (0..n)
        .map(|_| {
            s = s
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            (s >> 56) as u8
        })
        .collect()
}

/// Feeds the frames back to back to a `FrameCrc` and collects one `Crc`
/// per frame.
fn run(frames: &[Vec<u8>]) -> Vec<Crc> {
    let (tx, rx) = chan::<FrameByte, DefaultClock>();
    let (out_tx, out) = chan::<Crc, DefaultClock>();
    let mut unit = FrameCrc::default();
    let mut sim = Running::new(unit.run(rx, out_tx));
    let mut bytes: Vec<FrameByte> = Vec::new();
    for f in frames {
        for (i, &d) in f.iter().enumerate() {
            bytes.push(FrameByte::new(d, i + 1 == f.len(), false));
        }
    }
    let mut next = 0;
    let mut got = Vec::new();
    for _ in 0..bytes.len() + 10 {
        if next < bytes.len() && tx.ready().to_bool() {
            tx.send(bytes[next]);
            next += 1;
        }
        if let Some(c) = out.recv() {
            got.push(c);
        }
        sim.cycle();
    }
    got
}

fn with_fcs(mut f: Vec<u8>) -> Vec<u8> {
    f.extend_from_slice(&FCS60);
    f
}

#[test]
fn step_matches_reference_on_check_string() {
    let mut crc = U::<32>::from(INIT);
    for &b in b"123456789" {
        crc = crc32_step(crc, U::<8>::from(b));
    }
    assert_eq!(!(crc.raw() as u32), 0xCBF4_3926);
}

#[test]
fn check_string_value() {
    let got = run(&[b"123456789".to_vec()]);
    assert_eq!(got.len(), 1);
    assert_eq!(got[0].value.raw(), 0xCBF4_3926);
    assert!(!got[0].residue_ok.to_bool(), "no FCS in the check string");
}

#[test]
fn frame_without_fcs_has_its_crc_and_no_residue() {
    let got = run(&[frame60()]);
    assert_eq!(got.len(), 1);
    assert_eq!(got[0].value.raw(), 0x350A_A502);
    assert!(!got[0].residue_ok.to_bool());
}

#[test]
fn frame_with_fcs_has_residue() {
    let got = run(&[with_fcs(frame60())]);
    assert_eq!(got.len(), 1);
    assert_eq!(got[0].value.raw(), RESIDUE_ZLIB as u128);
    assert!(got[0].residue_ok.to_bool());
}

#[test]
fn any_single_bit_flip_breaks_the_residue() {
    let good = with_fcs(frame60());
    for byte in 0..good.len() {
        for bit in 0..8 {
            let mut bad = good.clone();
            bad[byte] ^= 1 << bit;
            let got = run(&[bad]);
            assert_eq!(got.len(), 1, "one frame, one CRC (byte {byte} bit {bit})");
            assert!(
                !got[0].residue_ok.to_bool(),
                "flip at byte {byte} bit {bit} must fail the residue"
            );
        }
    }
}

#[test]
fn consecutive_frames_are_independent() {
    let a = with_fcs(frame60());
    let b = b"123456789".to_vec();
    let got = run(&[a.clone(), b.clone(), a.clone()]);
    assert_eq!(got.len(), 3);
    assert!(got[0].residue_ok.to_bool());
    assert_eq!(got[1].value.raw(), 0xCBF4_3926);
    assert!(!got[1].residue_ok.to_bool());
    assert!(got[2].residue_ok.to_bool());
    assert_eq!(got[2].value.raw(), RESIDUE_ZLIB as u128);
}

#[test]
fn step_matches_reference_on_random_bytes() {
    let data = lcg_bytes(1000);
    let mut crc = U::<32>::from(INIT);
    for &b in &data {
        crc = crc32_step(crc, U::<8>::from(b));
    }
    let value = !(crc.raw() as u32);
    assert_eq!(value, reference_crc(&data));
    // zlib.crc32 of the same 1000 bytes.
    assert_eq!(value, 0x60D4_6325);
}
