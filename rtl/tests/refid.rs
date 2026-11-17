// SPDX-License-Identifier: Apache-2.0
//! Tests of the reference id helpers (T4.6). Literals come from Python:
//! `hashlib.md5(msg).hexdigest()` and `tools/golden/ntp.py`
//! (`refid_ipv4`, `refid_ipv6`).
use ntp_fpga::refid::{md5, refid_ipv4, refid_ipv6};

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}

#[test]
fn md5_matches_hashlib() {
    let v: [(&[u8], &str); 9] = [
        (b"", "d41d8cd98f00b204e9800998ecf8427e"),
        (b"a", "0cc175b9c0f1b6a831c399e269772661"),
        (b"abc", "900150983cd24fb0d6963f7d28e17f72"),
        (b"message digest", "f96b697d7cb7938d525a2f31aaf161d0"),
        (
            &[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15],
            "1ac1ef01e96caf1be0d329331a4fc2a8",
        ),
        (&[0xff; 16], "8d79cbc9a4ecdde112fc91ba625b13c2"),
        (
            &[0x20, 0x01, 0x0d, 0xb8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x99],
            "b70a6091e2ffb8262360b51922dab517",
        ),
        (
            &[0x20, 0x01, 0x0d, 0xb8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x01],
            "39ab9b3749629b8f2c7ccf39226f680c",
        ),
        (&[b'x'; 55], "04364420e25c512fd958a70738aa8f72"),
    ];
    for (m, h) in v {
        assert_eq!(hex(&md5(m)), h, "md5 of {m:?}");
    }
}

#[test]
fn md5_is_const_evaluable() {
    const D: [u8; 16] = md5(b"abc");
    assert_eq!(hex(&D), "900150983cd24fb0d6963f7d28e17f72");
}

#[test]
#[should_panic]
fn md5_rejects_two_blocks() {
    let _ = md5(&[0u8; 56]);
}

#[test]
fn ipv4_is_the_address() {
    assert_eq!(refid_ipv4(0xC000_0263), 0xC000_0263); // 192.0.2.99
    const R: u32 = refid_ipv4(0xC000_0201);
    assert_eq!(R, 0xC000_0201);
}

#[test]
fn ipv6_is_the_md5_prefix() {
    // The golden vectors (tools/golden/vectors.py CONFIG).
    assert_eq!(refid_ipv6(0x2001_0DB8_0000_0000_0000_0000_0000_0099), 0xB70A_6091);
    // Board upstream 2001:db8::1.
    const R: u32 = refid_ipv6(0x2001_0DB8_0000_0000_0000_0000_0000_0001);
    assert_eq!(R, 0x39AB_9B37);
}
