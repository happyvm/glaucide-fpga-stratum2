import unittest

from golden import arp, eth, icmp, icmpv6, ipv4, ipv6, ntp, udp
from golden.errors import ParseError

A4, B4 = bytes([192, 0, 2, 1]), bytes([192, 0, 2, 2])
A6, B6 = ipv6.addr("2001:db8::1"), ipv6.addr("2001:db8::2")


class CodecTest(unittest.TestCase):
    def test_eth(self):
        f = eth.EthFrame(eth.BROADCAST, eth.mac("02:00:00:00:00:01"), 0x0806, b"xyz")
        self.assertEqual(eth.EthFrame.parse(f.build()), f)
        self.assertEqual(len(f.build(pad=True)), 60)
        with self.assertRaises(ParseError):
            eth.EthFrame.parse(bytes(13))

    def test_arp(self):
        p = arp.ArpPacket(1, bytes(6), A4, bytes(6), B4)
        self.assertEqual(arp.ArpPacket.parse(p.build() + bytes(18)), p)
        with self.assertRaises(ParseError):
            arp.ArpPacket.parse(p.build()[:27])
        bad = bytearray(p.build())
        bad[4] = 8  # hlen
        with self.assertRaises(ParseError):
            arp.ArpPacket.parse(bytes(bad))

    def test_ipv4(self):
        p = ipv4.IPv4Packet(A4, B4, 17, b"hello", ident=7, ttl=9)
        raw = p.build()
        self.assertEqual(ipv4.IPv4Packet.parse(raw + bytes(10)), p)  # padding trimmed
        for mutate in (lambda b: b[:19], lambda b: b[:-1],
                       lambda b: b[:10] + b"\x00\x00" + b[12:],
                       lambda b: b"\x65" + b[1:]):
            with self.assertRaises(ParseError):
                ipv4.IPv4Packet.parse(mutate(raw))
        opt = ipv4.IPv4Packet(A4, B4, 17, b"x", options=b"\x01\x01\x01\x00")
        self.assertEqual(ipv4.IPv4Packet.parse(opt.build()).options, b"\x01\x01\x01\x00")

    def test_ipv6(self):
        p = ipv6.IPv6Packet(A6, B6, 17, b"hello", hop_limit=3, traffic_class=5, flow_label=0x12345)
        self.assertEqual(ipv6.IPv6Packet.parse(p.build() + bytes(7)), p)
        with self.assertRaises(ParseError):
            ipv6.IPv6Packet.parse(p.build()[:-1])
        with self.assertRaises(ParseError):
            ipv6.IPv6Packet.parse(b"\x40" + p.build()[1:])

    def test_icmp(self):
        e = icmp.IcmpEcho(8, 1, 2, b"data")
        self.assertEqual(icmp.IcmpEcho.parse(e.build()), e)
        b = bytearray(e.build())
        b[-1] ^= 1
        with self.assertRaises(ParseError):
            icmp.IcmpEcho.parse(bytes(b))

    def test_icmpv6(self):
        e = icmpv6.Icmpv6Echo(128, 1, 2, b"data")
        self.assertEqual(icmpv6.Icmpv6Echo.parse(A6, B6, e.build(A6, B6)), e)
        with self.assertRaises(ParseError):
            icmpv6.Icmpv6Echo.parse(B6, B6, e.build(A6, B6))  # pseudo-header mismatch
        ns = icmpv6.NeighborSolicitation(A6, [(1, eth.mac("02:00:00:00:00:02"))])
        self.assertEqual(icmpv6.NeighborSolicitation.parse(A6, B6, ns.build(A6, B6)), ns)
        na = icmpv6.NeighborAdvertisement(A6, False, True, True, [(2, bytes(6))])
        self.assertEqual(icmpv6.NeighborAdvertisement.parse(A6, B6, na.build(A6, B6)), na)
        raw = na.build(A6, B6)
        self.assertEqual(raw[4:8], bytes.fromhex("60000000"))  # S and O set, R clear
        pre = ipv6.addr("2001:db8:1::")
        ra = icmpv6.RouterAdvertisement(64, False, False, 1800, 0, 0,
                                        [(1, bytes(6)), icmpv6.prefix_option(pre, 64, 2592000, 604800)])
        got = icmpv6.RouterAdvertisement.parse(A6, B6, ra.build(A6, B6))
        self.assertEqual(got, ra)
        self.assertEqual(icmpv6.parse_prefix_option(got.options[1][1]),
                         (64, True, True, 2592000, 604800, pre))

    def test_ndp_option_errors(self):
        with self.assertRaises(ParseError):
            icmpv6.parse_options(b"\x01\x00" + bytes(6))   # length 0
        with self.assertRaises(ParseError):
            icmpv6.parse_options(b"\x01\x02" + bytes(6))   # overruns

    def test_udp(self):
        for s, d in ((A4, B4), (A6, B6)):
            u = udp.UdpDatagram(1, 123, b"abc")
            self.assertEqual(udp.UdpDatagram.parse(s, d, u.build(s, d)), u)
            # Stricter than RFC 1122 (decision T2.2): trailing bytes after the
            # UDP length are rejected, the length must match exactly.
            with self.assertRaises(ParseError):
                udp.UdpDatagram.parse(s, d, u.build(s, d) + b"pad")
            b = bytearray(u.build(s, d))
            b[8] ^= 1
            with self.assertRaises(ParseError):
                udp.UdpDatagram.parse(s, d, bytes(b))
            with self.assertRaises(ParseError):
                udp.UdpDatagram.parse(s, d, u.build(s, d)[:7])
        z = udp.UdpDatagram(1, 2, b"x").build(A4, B4, zero_checksum=True)
        self.assertEqual(udp.UdpDatagram.parse(A4, B4, z).payload, b"x")  # v4: 0 accepted
        z6 = bytearray(udp.UdpDatagram(1, 2, b"x").build(A6, B6))
        z6[6:8] = b"\x00\x00"
        with self.assertRaises(ParseError):
            udp.UdpDatagram.parse(A6, B6, bytes(z6))  # v6: 0 refused
        with self.assertRaises(ValueError):
            udp.UdpDatagram(1, 2, b"x").build(A6, B6, zero_checksum=True)

    def test_ntp(self):
        p = ntp.NtpPacket(li=3, vn=4, mode=4, stratum=16, poll=6, precision=-24, root_delay=1,
                          root_dispersion=2, ref_id=b"INIT", ref_ts=3, orig_ts=4, recv_ts=5, xmit_ts=6)
        raw = p.build()
        self.assertEqual(len(raw), 48)
        self.assertEqual(ntp.NtpPacket.parse(raw), p)
        self.assertEqual(ntp.NtpPacket.parse(raw + b"ext!").extensions, b"ext!")
        with self.assertRaises(ParseError):
            ntp.NtpPacket.parse(raw[:47])

    def test_ntp_time_helpers(self):
        self.assertEqual(ntp.seconds_to_ts(1), 1 << 32)
        self.assertEqual(ntp.seconds_to_ts(0.5), 1 << 31)
        self.assertEqual(ntp.ts_to_seconds(3 << 31), 1.5)
        self.assertEqual(ntp.unix_to_ts(0) >> 32, 2208988800)
        self.assertEqual(ntp.ts_to_unix(ntp.unix_to_ts(1763380800)), 1763380800)
        self.assertEqual(ntp.seconds_to_short(1.5), 0x18000)
        self.assertEqual(ntp.short_to_seconds(0x18000), 1.5)
        self.assertEqual(ntp.refid_ipv4("192.0.2.99").hex(), "c0000263")
        self.assertEqual(len(ntp.refid_ipv6("2001:db8::99")), 4)


if __name__ == "__main__":
    unittest.main()
