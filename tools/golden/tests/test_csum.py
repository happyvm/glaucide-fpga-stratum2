import unittest

from golden import csum


class CsumTest(unittest.TestCase):
    def test_rfc1071_example(self):
        # RFC 1071 section 3: 00 01 f2 03 f4 f5 f6 f7 -> sum 0xddf2 (before complement)
        data = bytes.fromhex("0001f203f4f5f6f7")
        self.assertEqual(csum.ones_complement_sum(data), 0xDDF2)
        self.assertEqual(csum.internet_checksum(data), 0x220D)

    def test_odd_length_padding(self):
        self.assertEqual(csum.ones_complement_sum(b"\x01"), 0x0100)

    def test_verify(self):
        data = bytes.fromhex("0001f203f4f5f6f7")
        self.assertTrue(csum.verify(data + (0x220D).to_bytes(2, "big")))
        self.assertFalse(csum.verify(data + (0x220E).to_bytes(2, "big")))

    def test_ipv4_header_known(self):
        # Widely used worked example: 4500 0073 0000 4000 4011 [b861] c0a8 0001 c0a8 00c7
        hdr = bytes.fromhex("45000073000040004011b861c0a80001c0a800c7")
        self.assertEqual(csum.ipv4_header_checksum(hdr), 0xB861)
        self.assertTrue(csum.verify(hdr))

    def test_crc32_check_value(self):
        self.assertEqual(csum.eth_crc32(b"123456789"), 0xCBF43926)

    def test_fcs_order_and_residue(self):
        frame = bytes(range(60))
        fcs = csum.eth_fcs(frame)
        self.assertEqual(fcs, csum.eth_crc32(frame).to_bytes(4, "little"))
        self.assertTrue(csum.check_fcs(frame + fcs))
        bad = bytearray(frame + fcs)
        bad[5] ^= 1
        self.assertFalse(csum.check_fcs(bytes(bad)))
        # residue of formal.md: CRC over frame+FCS (register without final xor) is 0xC704DD7B,
        # i.e. zlib's value is its complement 0x2144DF1C
        self.assertEqual(csum.eth_crc32(frame + fcs), 0x2144DF1C)

    def test_pseudo_headers(self):
        p4 = csum.pseudo_header_v4(b"\xc0\x00\x02\x01", b"\xc0\x00\x02\x02", 17, 56)
        self.assertEqual(p4.hex(), "c0000201c00002020011" "0038")
        p6 = csum.pseudo_header_v6(bytes(15) + b"\x01", bytes(15) + b"\x02", 58, 32)
        self.assertEqual(len(p6), 40)
        self.assertEqual(p6[32:], bytes.fromhex("00000020000000" "3a"))


if __name__ == "__main__":
    unittest.main()
