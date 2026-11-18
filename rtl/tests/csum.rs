// SPDX-License-Identifier: Apache-2.0
//! Tests of the RFC 1071 checksum unit and its pseudo-header seeds (T3.2).
//! Expected values come from `tools/golden/csum.py` (embedded as literals)
//! or from the bitwise reference below, not from the unit.
use ntp_fpga::csum::{csum_add, csum_fold, pseudo_v4, pseudo_v6, udp_csum_final, Csum, CsumOut};
use ntp_fpga::stream::FrameByte;
use txhdl::comp::{chan, signal, DefaultClock, Running, Unit};
use txhdl::types::U;

/// One's complement addition of two 16-bit words, written bit by bit: a
/// ripple-carry adder whose carry out is added back at bit 0.
fn ones_add(a: u16, b: u16) -> u16 {
    let mut sum: u16 = 0;
    let mut carry: u32 = 0;
    for i in 0..16 {
        let s = ((a as u32 >> i) & 1) + ((b as u32 >> i) & 1) + carry;
        sum |= ((s & 1) as u16) << i;
        carry = s >> 1;
    }
    if carry == 1 {
        ones_add(sum, 1)
    } else {
        sum
    }
}

/// Reference: the folded one's complement sum of `seed` (folded first) and
/// the big-endian words of `data`, an odd last byte padded with zero. Zero
/// only when everything is zero, as the unit's fold is.
fn reference_sum(seed: u32, data: &[u8]) -> u16 {
    let mut s = ones_add((seed & 0xFFFF) as u16, (seed >> 16) as u16);
    for pair in data.chunks(2) {
        let lo = pair.get(1).copied().unwrap_or(0) as u16;
        s = ones_add(s, ((pair[0] as u16) << 8) | lo);
    }
    s
}

/// The `CsumOut` the reference predicts for a stream.
fn reference(seed: u32, data: &[u8]) -> (u16, bool) {
    let folded = reference_sum(seed, data);
    (!folded, folded == 0xFFFF)
}

/// Fixed-seed LCG (Knuth's MMIX constants), top byte of each state.
fn lcg_bytes(n: usize, seed: u64) -> Vec<u8> {
    let mut s = seed;
    (0..n)
        .map(|_| {
            s = s
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            (s >> 56) as u8
        })
        .collect()
}

/// Feeds the streams, each with its seed. The seed signal is set with the
/// first byte of a stream, which the unit samples when it takes that byte.
/// The channel holds two bytes, so the producer must not change the seed
/// before the previous first byte is taken: with `sequential`, a stream
/// starts only after the previous one's checksum is out (the unit takes
/// its last byte first). With `garbage`, the seed is set to that value at
/// the second byte, once the first is surely taken: the unit must ignore
/// it. Returns one `CsumOut` per stream.
fn run(streams: &[(Vec<u8>, u32)], garbage: Option<u32>, sequential: bool) -> Vec<CsumOut> {
    let (tx, rx) = chan::<FrameByte, DefaultClock>();
    let (seed_out, seed) = signal::<U<32>, DefaultClock>();
    let (out_tx, out) = chan::<CsumOut, DefaultClock>();
    let mut unit = Csum::default();
    let mut sim = Running::new(unit.run((rx, seed), out_tx));

    // (byte, index in its stream, seed to set when it is sent) in order.
    let mut queue: Vec<(FrameByte, usize, Option<u32>)> = Vec::new();
    for (data, s) in streams {
        let n = data.len();
        for (i, &d) in data.iter().enumerate() {
            let set = if i == 0 { Some(*s) } else { garbage };
            queue.push((FrameByte::new(d, i + 1 == n, false), i, set));
        }
    }
    // Cycles the first byte of the current stream has been out for: the
    // second byte, the one carrying the garbage seed, waits for this.
    const FIRST_TAKEN: usize = 16;
    let mut next = 0;
    let mut got = Vec::new();
    let mut stream_index = 0;
    let mut since_first = FIRST_TAKEN;
    let limit = queue.len() * 8 + 200;
    for _ in 0..limit {
        if next < queue.len() && tx.ready().to_bool() {
            let (byte, index, set) = queue[next];
            let gate = if index == 0 {
                // A new stream's seed waits for the previous checksum.
                !sequential || got.len() >= stream_index
            } else {
                index != 1 || since_first >= FIRST_TAKEN
            };
            if gate {
                if index == 0 {
                    stream_index += 1;
                    since_first = 0;
                }
                if let Some(v) = set {
                    seed_out.set(U::<32>::from(v));
                }
                tx.send(byte);
                next += 1;
            }
        }
        since_first = since_first.saturating_add(1);
        if let Some(c) = out.recv() {
            got.push(c);
        }
        sim.cycle();
        if got.len() == streams.len() {
            break;
        }
    }
    assert_eq!(got.len(), streams.len(), "one checksum per stream");
    got
}

fn check_one(data: &[u8], seed: u32) {
    let got = run(&[(data.to_vec(), seed)], None, true);
    let (sum, ok) = reference(seed, data);
    assert_eq!(got[0].sum.raw() as u16, sum, "sum for {} bytes", data.len());
    assert_eq!(got[0].ok.to_bool(), ok, "ok for {} bytes", data.len());
}

#[test]
fn rfc1071_section3_example() {
    // RFC 1071 section 3: the words 0001 f203 f4f5 f6f7 sum to ddf2.
    let data = [0x00, 0x01, 0xf2, 0x03, 0xf4, 0xf5, 0xf6, 0xf7];
    let got = run(&[(data.to_vec(), 0)], None, true);
    assert_eq!(got[0].sum.raw(), 0x220D);
    assert!(!got[0].ok.to_bool());
}

#[test]
fn python_vectors() {
    // tools/golden/csum.py: internet_checksum of each string (seed 0).
    let cases: [(&[u8], u128); 7] = [
        (&[0x00, 0x01, 0xf2, 0x03, 0xf4, 0xf5, 0xf6, 0xf7], 0x220D),
        (&[0xab], 0x54FF),
        (&[0u8; 8], 0xFFFF),
        (b"abc", 0x3B9D),
        (b"123456789", 0xF62A),
        (&[1, 2, 3], 0xFBFD),
        (
            &[
                0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
            ],
            0xA59B,
        ),
    ];
    for (data, expect) in cases {
        let got = run(&[(data.to_vec(), 0)], None, true);
        assert_eq!(got[0].sum.raw(), expect, "checksum of {data:?}");
    }
}

#[test]
fn all_zero_data_gives_ffff_and_not_ok() {
    let got = run(&[(vec![0u8; 8], 0)], None, true);
    assert_eq!(got[0].sum.raw(), 0xFFFF);
    assert!(!got[0].ok.to_bool(), "a zero sum is not 0xFFFF");
}

#[test]
fn ffff_word_is_ok_and_sums_to_zero() {
    let got = run(&[(vec![0xFF, 0xFF], 0)], None, true);
    assert_eq!(got[0].sum.raw(), 0x0000);
    assert!(got[0].ok.to_bool(), "a sum of 0xFFFF is ok");
}

#[test]
fn one_byte_and_odd_and_even_lengths() {
    for n in [1usize, 2, 3, 5, 6, 7, 64, 65] {
        check_one(&lcg_bytes(n, 0x51 + n as u64), 0);
    }
}

#[test]
fn long_frames_1517_and_1518() {
    check_one(&lcg_bytes(1517, 7), 0);
    check_one(&lcg_bytes(1518, 8), 0);
}

#[test]
fn random_streams_match_reference_back_to_back() {
    // 1000 streams of pseudo-random length 1..=300, back to back, seed 0.
    let lens = lcg_bytes(1000, 0x2026_1117);
    let streams: Vec<(Vec<u8>, u32)> = (0..1000)
        .map(|k| {
            let n = 1 + (lens[k] as usize * 300) / 256;
            (lcg_bytes(n, 0x1000 + k as u64), 0)
        })
        .collect();
    let got = run(&streams, None, false);
    for (k, ((data, seed), c)) in streams.iter().zip(&got).enumerate() {
        let (sum, ok) = reference(*seed, data);
        assert_eq!(c.sum.raw() as u16, sum, "stream {k}, {} bytes", data.len());
        assert_eq!(c.ok.to_bool(), ok, "stream {k}");
    }
}

#[test]
fn accumulator_bound_holds_for_64k_streams() {
    // Arithmetic bound of the module documentation: a 64 KiB stream with
    // the folded seed (at most 0xFFFF) sums to at most 0xFFFF * (1 + 32768),
    // which fits 32 bits. The unit's own contract (1518 bytes, enforced by
    // its assume!) is what the simulation and the proof cover.
    let bound: u64 = 0xFFFF * (1 + 32768);
    assert_eq!(bound, 2_147_516_415);
    assert!(bound < 1u64 << 32);
    // The largest 1518-byte stream of 0xFF words, with the largest seed,
    // matches the reference.
    check_one(&vec![0xFFu8; 1518], 0xFFFF_FFFF);
}

#[test]
fn ipv4_header_with_good_checksum_is_ok_and_flipped_bit_is_not() {
    // IPv4 header of tools/golden/vectors/ping4.hex (checksum 7912).
    let header: [u8; 20] = [
        0x45, 0x00, 0x00, 0x3c, 0x3d, 0xab, 0x40, 0x00, 0x40, 0x01, 0x79, 0x12, 0xc0, 0x00, 0x02,
        0x02, 0xc0, 0x00, 0x02, 0x01,
    ];
    let good = run(&[(header.to_vec(), 0)], None, true);
    assert!(good[0].ok.to_bool(), "a correct header is ok");
    assert_eq!(
        good[0].sum.raw(),
        0,
        "the sum of a correct header is 0xFFFF"
    );

    let mut bad = header;
    bad[5] ^= 0x10;
    let got = run(&[(bad.to_vec(), 0)], None, true);
    assert!(!got[0].ok.to_bool(), "a flipped bit is not ok");

    // Header with the checksum field zeroed: the output is the field value.
    let mut zeroed = header;
    zeroed[10] = 0;
    zeroed[11] = 0;
    let got = run(&[(zeroed.to_vec(), 0)], None, true);
    assert_eq!(got[0].sum.raw(), 0x7912);
}

#[test]
fn seed_is_sampled_once_per_stream() {
    // Three streams, each with its own seed; later bytes carry a garbage
    // seed that must be ignored.
    let a = lcg_bytes(40, 1);
    let b = lcg_bytes(11, 2);
    let c = lcg_bytes(3, 3);
    let streams = [
        (a.clone(), 0x0001_2345),
        (b.clone(), 0x0000_FFFF),
        (c.clone(), 0x8000_0000),
    ];
    let got = run(&streams, Some(0xDEAD_BEEF), true);
    for (k, ((data, seed), out)) in streams.iter().zip(&got).enumerate() {
        let (sum, ok) = reference(*seed, data);
        assert_eq!(out.sum.raw() as u16, sum, "stream {k}");
        assert_eq!(out.ok.to_bool(), ok, "stream {k}");
    }
}

#[test]
fn back_to_back_streams_are_independent() {
    let a = lcg_bytes(33, 9);
    let got_pair = run(&[(a.clone(), 0), (a.clone(), 0)], None, false);
    let alone = run(&[(a.clone(), 0)], None, true);
    assert_eq!(got_pair.len(), 2);
    assert_eq!(got_pair[0].sum.raw(), alone[0].sum.raw());
    assert_eq!(got_pair[1].sum.raw(), alone[0].sum.raw());
    assert_eq!(got_pair[1].ok.to_bool(), alone[0].ok.to_bool());
}

#[test]
fn udp_zero_checksum_is_sent_as_ffff() {
    assert_eq!(udp_csum_final(U::<16>::from(0u16)).raw(), 0xFFFF);
    assert_eq!(udp_csum_final(U::<16>::from(0x1234u16)).raw(), 0x1234);
    assert_eq!(udp_csum_final(U::<16>::from(0xFFFFu16)).raw(), 0xFFFF);
}

#[test]
fn fold_and_add_small_cases() {
    // 0x1_0005 folds to 0x0006 (carry added back), 0xFFFF_FFFF to 0xFFFF.
    assert_eq!(csum_fold(U::<32>::from(0x0001_0005u32)).raw(), 0x0006);
    assert_eq!(csum_fold(U::<32>::from(0xFFFF_FFFFu32)).raw(), 0xFFFF);
    assert_eq!(csum_fold(U::<32>::from(0u32)).raw(), 0);
    let s = csum_add(U::<32>::from(0xFFFFu32), U::<16>::from(0x0002u16));
    assert_eq!(s.raw(), 0x1_0001);
}

#[test]
fn pseudo_v4_seed_matches_python() {
    // src 192.0.2.1, dst 192.0.2.2, UDP (17), length 20 (tools/golden/csum.py).
    let seed = pseudo_v4(
        U::<32>::from(0xC000_0201u32),
        U::<32>::from(0xC000_0202u32),
        U::<8>::from(17u8),
        U::<16>::from(20u16),
    );
    assert_eq!(csum_fold(seed).raw(), 0x8429);
    // UDP datagram: 8-byte header (checksum zero) + 12 bytes of payload.
    let mut seg = vec![0x9c, 0x40, 0x00, 0x7b, 0x00, 0x14, 0x00, 0x00];
    seg.extend(1u8..=12);
    let got = run(&[(seg, seed.raw() as u32)], None, true);
    assert_eq!(got[0].sum.raw(), 0xBADC, "UDP over IPv4 checksum");
}

#[test]
fn pseudo_v6_seed_matches_python() {
    // src 2001:db8::1, dst 2001:db8::2, next header 17, length 20.
    let src = U::<128>::from(0x2001_0db8_0000_0000_0000_0000_0000_0001u128);
    let dst = U::<128>::from(0x2001_0db8_0000_0000_0000_0000_0000_0002u128);
    let seed = pseudo_v6(src, dst, U::<8>::from(17u8), U::<32>::from(20u32));
    assert_eq!(csum_fold(seed).raw(), 0x5B9A);
    let mut seg = vec![0x9c, 0x40, 0x00, 0x7b, 0x00, 0x14, 0x00, 0x00];
    seg.extend(1u8..=12);
    let got = run(&[(seg, seed.raw() as u32)], None, true);
    assert_eq!(got[0].sum.raw(), 0xE36B, "UDP over IPv6 checksum");
}
