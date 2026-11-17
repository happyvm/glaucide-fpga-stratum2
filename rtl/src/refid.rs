// SPDX-License-Identifier: Apache-2.0
//! Reference ID helpers (task T4.6): pure `const fn`s, usable in const
//! contexts (synthesis-time constants) and tested at run time.
//!
//! RFC 5905 §7.3: the Reference ID of a stratum >= 2 server is the IPv4
//! address of its source, or, for an IPv6 source, the first four octets of
//! the MD5 digest of the IPv6 address (16 octets, network order). The
//! golden model is `tools/golden/ntp.py` (`refid_ipv4`, `refid_ipv6`).
//! The returned `u32` has the first wire byte in its most significant byte.
//!
//! * [`md5`]: RFC 1321, single block, messages of at most 55 bytes (so that
//!   the padding fits in the one 64-byte block). Longer input panics (a
//!   compile error in a const context).
//! * [`refid_ipv4`]`(a) = a`.
//! * [`refid_ipv6`]`(a) = md5(a as 16 big endian bytes)[0..4]` as a `u32`
//!   (digest byte 0 most significant). `2001:db8::99` gives `0xb70a6091`,
//!   `192.0.2.99` gives `0xc0000263`.

/// MD5 constants `floor(2^32 * |sin(i + 1)|)`.
const K: [u32; 64] = [
    0xd76aa478, 0xe8c7b756, 0x242070db, 0xc1bdceee, 0xf57c0faf, 0x4787c62a, 0xa8304613, 0xfd469501,
    0x698098d8, 0x8b44f7af, 0xffff5bb1, 0x895cd7be, 0x6b901122, 0xfd987193, 0xa679438e, 0x49b40821,
    0xf61e2562, 0xc040b340, 0x265e5a51, 0xe9b6c7aa, 0xd62f105d, 0x02441453, 0xd8a1e681, 0xe7d3fbc8,
    0x21e1cde6, 0xc33707d6, 0xf4d50d87, 0x455a14ed, 0xa9e3e905, 0xfcefa3f8, 0x676f02d9, 0x8d2a4c8a,
    0xfffa3942, 0x8771f681, 0x6d9d6122, 0xfde5380c, 0xa4beea44, 0x4bdecfa9, 0xf6bb4b60, 0xbebfbc70,
    0x289b7ec6, 0xeaa127fa, 0xd4ef3085, 0x04881d05, 0xd9d4d039, 0xe6db99e5, 0x1fa27cf8, 0xc4ac5665,
    0xf4292244, 0x432aff97, 0xab9423a7, 0xfc93a039, 0x655b59c3, 0x8f0ccc92, 0xffeff47d, 0x85845dd1,
    0x6fa87e4f, 0xfe2ce6e0, 0xa3014314, 0x4e0811a1, 0xf7537e82, 0xbd3af235, 0x2ad7d2bb, 0xeb86d391,
];

/// Per-round left rotations.
const S: [u32; 16] = [7, 12, 17, 22, 5, 9, 14, 20, 4, 11, 16, 23, 6, 10, 15, 21];

/// MD5 of a message of at most 55 bytes (one block).
pub const fn md5(msg: &[u8]) -> [u8; 16] {
    assert!(msg.len() <= 55, "md5: single block only (at most 55 bytes)");
    // Padding: message, 0x80, zeros, 64-bit little endian bit length.
    let mut blk = [0u8; 64];
    let mut i = 0;
    while i < msg.len() {
        blk[i] = msg[i];
        i += 1;
    }
    blk[msg.len()] = 0x80;
    let bits = (msg.len() as u64) * 8;
    let mut j = 0;
    while j < 8 {
        blk[56 + j] = (bits >> (8 * j)) as u8;
        j += 1;
    }
    // Words, little endian.
    let mut m = [0u32; 16];
    let mut w = 0;
    while w < 16 {
        m[w] = (blk[4 * w] as u32)
            | ((blk[4 * w + 1] as u32) << 8)
            | ((blk[4 * w + 2] as u32) << 16)
            | ((blk[4 * w + 3] as u32) << 24);
        w += 1;
    }
    let (mut a, mut b, mut c, mut d) = (0x6745_2301u32, 0xefcd_ab89u32, 0x98ba_dcfeu32, 0x1032_5476u32);
    let mut r = 0;
    while r < 64 {
        let (f, g) = if r < 16 {
            ((b & c) | (!b & d), r)
        } else if r < 32 {
            ((d & b) | (!d & c), (5 * r + 1) % 16)
        } else if r < 48 {
            (b ^ c ^ d, (3 * r + 5) % 16)
        } else {
            (c ^ (b | !d), (7 * r) % 16)
        };
        let t = a
            .wrapping_add(f)
            .wrapping_add(K[r])
            .wrapping_add(m[g]);
        a = d;
        d = c;
        c = b;
        b = b.wrapping_add(t.rotate_left(S[(r / 16) * 4 + r % 4]));
        r += 1;
    }
    let h = [
        0x6745_2301u32.wrapping_add(a),
        0xefcd_ab89u32.wrapping_add(b),
        0x98ba_dcfeu32.wrapping_add(c),
        0x1032_5476u32.wrapping_add(d),
    ];
    let mut out = [0u8; 16];
    let mut k = 0;
    while k < 16 {
        out[k] = (h[k / 4] >> (8 * (k % 4))) as u8;
        k += 1;
    }
    out
}

/// Reference ID for an IPv4 source: the address itself.
pub const fn refid_ipv4(addr: u32) -> u32 {
    addr
}

/// Reference ID for an IPv6 source: the first four octets of the MD5 digest
/// of the 16 address octets (network order), digest byte 0 most significant.
pub const fn refid_ipv6(addr: u128) -> u32 {
    let d = md5(&addr.to_be_bytes());
    u32::from_be_bytes([d[0], d[1], d[2], d[3]])
}
