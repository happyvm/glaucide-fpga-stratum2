// SPDX-License-Identifier: Apache-2.0
//! Tests of the FCS checker (T1.12).
use ntp_fpga::fcs_check::FcsCheck;
use ntp_fpga::stream::FrameByte;
use txhdl::comp::{chan, DefaultClock, Running, Unit};

/// Independent bitwise CRC-32 (reflected, poly 0xEDB88320), complemented.
fn crc32(data: &[u8]) -> u32 {
    let mut c = 0xFFFF_FFFFu32;
    for &b in data {
        c ^= b as u32;
        for _ in 0..8 {
            c = if c & 1 != 0 {
                (c >> 1) ^ 0xEDB8_8320
            } else {
                c >> 1
            };
        }
    }
    !c
}

fn body(n: usize) -> Vec<u8> {
    (0..n).map(|i| (i * 7 + 3) as u8).collect()
}

/// Payload followed by its FCS, little endian.
fn with_fcs(d: &[u8]) -> Vec<u8> {
    let mut v = d.to_vec();
    v.extend_from_slice(&crc32(d).to_le_bytes());
    v
}

/// Raw frame to the input byte list, `err` on the last byte if asked.
fn bytes(raw: &[u8], err: bool) -> Vec<FrameByte> {
    let n = raw.len();
    raw.iter()
        .enumerate()
        .map(|(i, &b)| FrameByte::new(b, i + 1 == n, err && i + 1 == n))
        .collect()
}

#[derive(Debug, Clone, PartialEq)]
struct Got {
    data: u8,
    last: bool,
    err: bool,
}

/// Feed `input`, one byte per cycle when the unit is ready; the consumer
/// accepts at cycle `t` when `take(t)`. Returns the output bytes.
fn run(input: &[FrameByte], take: impl Fn(usize) -> bool) -> Vec<Got> {
    let (tx, rx) = chan::<FrameByte, DefaultClock>();
    let (otx, orx) = chan::<FrameByte, DefaultClock>();
    let mut m = FcsCheck::default();
    let mut sim = Running::new(m.run(rx, otx));
    let mut next = 0;
    let mut out = Vec::new();
    for t in 0..input.len() * 4 + 30 {
        if next < input.len() && tx.ready().to_bool() {
            tx.send(input[next]);
            next += 1;
        }
        if take(t) {
            if let Some(b) = orx.recv() {
                out.push(Got {
                    data: b.data.raw() as u8,
                    last: b.last.to_bool(),
                    err: b.err.to_bool(),
                });
            }
        }
        sim.cycle();
    }
    assert_eq!(next, input.len(), "all input bytes were accepted");
    out
}

/// Stream contract on the output: err only with last, one last per frame
/// at the end. Returns the frames.
fn frames(o: &[Got]) -> Vec<Vec<Got>> {
    let mut all = vec![];
    let mut cur = vec![];
    for b in o {
        assert!(!b.err || b.last, "err without last: {b:?}");
        cur.push(b.clone());
        if b.last {
            all.push(std::mem::take(&mut cur));
        }
    }
    assert!(cur.is_empty(), "bytes after the last last");
    all
}

fn data(f: &[Got]) -> Vec<u8> {
    f.iter().map(|g| g.data).collect()
}

fn one_frame(raw: &[u8], err: bool, expect_err: bool) {
    let o = run(&bytes(raw, err), |_| true);
    let f = frames(&o);
    assert_eq!(f.len(), 1, "one frame out for {} raw bytes", raw.len());
    assert_eq!(
        f[0].len(),
        raw.len() - 4,
        "output length is input length - 4"
    );
    assert_eq!(data(&f[0]), raw[..raw.len() - 4].to_vec());
    assert_eq!(f[0].last().unwrap().err, expect_err);
}

#[test]
fn bitwise_crc_matches_python_zlib() {
    // Vectors from python3 zlib.crc32; FCS bytes little endian.
    assert_eq!(crc32(&body(60)), 0x7fb0bee4);
    assert_eq!(crc32(&body(46)), 0x825b4540);
    let d: Vec<u8> = (0..20).collect();
    assert_eq!(crc32(&d), 0x3bddffa4);
    assert_eq!(crc32(b"123456789"), 0xCBF43926);
}

#[test]
fn good_frames_from_python_vectors() {
    let mut f60 = body(60);
    f60.extend_from_slice(&[0xe4, 0xbe, 0xb0, 0x7f]);
    one_frame(&f60, false, false);
    let mut f46 = body(46);
    f46.extend_from_slice(&[0x40, 0x45, 0x5b, 0x82]);
    one_frame(&f46, false, false);
    let mut f20: Vec<u8> = (0..20).collect();
    f20.extend_from_slice(&[0xa4, 0xff, 0xdd, 0x3b]);
    one_frame(&f20, false, false);
}

#[test]
fn good_64_byte_frame() {
    let raw = with_fcs(&body(60));
    assert_eq!(raw.len(), 64);
    one_frame(&raw, false, false);
}

#[test]
fn bit_flip_in_every_region() {
    let raw = with_fcs(&body(60));
    // Header, payload, each FCS byte.
    for pos in [0, 5, 13, 14, 30, 59, 60, 61, 62, 63] {
        for bit in [0, 7] {
            let mut bad = raw.clone();
            bad[pos] ^= 1 << bit;
            let o = run(&bytes(&bad, false), |_| true);
            let f = frames(&o);
            assert_eq!(f.len(), 1);
            assert_eq!(f[0].len(), 60, "pos {pos}");
            assert!(
                f[0].last().unwrap().err,
                "flip at {pos} bit {bit} is flagged"
            );
            assert!(f[0][..59].iter().all(|g| !g.err));
        }
    }
}

#[test]
fn max_frame_1518() {
    let raw = with_fcs(&body(1514));
    assert_eq!(raw.len(), 1518);
    one_frame(&raw, false, false);
    let mut bad = raw.clone();
    bad[700] ^= 0x10;
    one_frame(&bad, false, true);
}

#[test]
fn five_byte_frame_emits_one_byte() {
    let raw = with_fcs(&[0x42]);
    assert_eq!(raw.len(), 5);
    one_frame(&raw, false, false);
    let mut bad = raw.clone();
    bad[0] ^= 1;
    one_frame(&bad, false, true);
}

#[test]
fn runts_emit_nothing() {
    for n in 1..=4 {
        let raw = with_fcs(&body(n))[..n].to_vec();
        assert!(
            run(&bytes(&raw, false), |_| true).is_empty(),
            "{n} raw bytes"
        );
        assert!(
            run(&bytes(&raw, true), |_| true).is_empty(),
            "{n} raw bytes, err"
        );
    }
    // A good FCS alone (4 bytes) emits nothing either.
    let raw = with_fcs(&[]);
    assert_eq!(raw.len(), 4);
    assert!(run(&bytes(&raw, false), |_| true).is_empty());
}

#[test]
fn input_err_is_carried() {
    // Good FCS but err on last: flagged. Bad FCS and err: flagged.
    one_frame(&with_fcs(&body(60)), true, true);
    let mut bad = with_fcs(&body(60));
    bad[3] ^= 4;
    one_frame(&bad, true, true);
}

#[test]
fn back_to_back_frames_are_independent() {
    let good = with_fcs(&body(60));
    let mut bad = with_fcs(&body(40));
    bad[10] ^= 1;
    let runt = with_fcs(&body(3))[..3].to_vec();
    let mut input = bytes(&good, false);
    input.extend(bytes(&bad, false));
    input.extend(bytes(&runt, false));
    input.extend(bytes(&good, false));
    input.extend(bytes(&with_fcs(&body(1)), true));
    input.extend(bytes(&good, false));
    let f = frames(&run(&input, |_| true));
    assert_eq!(f.len(), 5);
    assert_eq!(data(&f[0]), body(60));
    assert!(!f[0].last().unwrap().err);
    assert_eq!(
        data(&f[1]),
        body(40)
            .iter()
            .enumerate()
            .map(|(i, &b)| if i == 10 { b ^ 1 } else { b })
            .collect::<Vec<_>>()
    );
    assert!(f[1].last().unwrap().err);
    assert!(!f[2].last().unwrap().err, "the runt left no residue behind");
    assert_eq!(f[2].len(), 60);
    assert_eq!(f[3].len(), 1);
    assert!(f[3].last().unwrap().err);
    assert!(!f[4].last().unwrap().err);
    assert_eq!(f[4].len(), 60);
}

#[test]
fn backpressure_keeps_bytes_and_order() {
    let mut input = bytes(&with_fcs(&body(60)), false);
    let mut bad = with_fcs(&body(30));
    bad[2] ^= 0x80;
    input.extend(bytes(&bad, false));
    let patterns: [&dyn Fn(usize) -> bool; 3] = [&|t| t % 3 == 0, &|t| t % 7 < 2, &|t| t > 100];
    for p in patterns {
        let f = frames(&run(&input, p));
        assert_eq!(f.len(), 2);
        assert_eq!(data(&f[0]), body(60));
        assert!(!f[0].last().unwrap().err);
        assert_eq!(f[1].len(), 30);
        assert!(f[1].last().unwrap().err);
    }
}

#[test]
fn output_length_is_input_length_minus_four() {
    for n in 5..=80 {
        let raw = with_fcs(&body(n - 4));
        let f = frames(&run(&bytes(&raw, false), |t| t % 2 == 0));
        assert_eq!(f.len(), 1);
        assert_eq!(f[0].len(), n - 4);
    }
}
