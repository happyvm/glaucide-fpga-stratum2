"""Packets whose bytes were derived by hand from the RFC layouts (expected bytes are literals)."""
import unittest

from golden import arp, eth, icmp, ipv4, ntp, responder
from golden.responder import Config, respond

MAC1 = eth.mac("02:00:00:00:00:01")
MAC2 = eth.mac("02:00:00:00:00:02")


class HandCheckedTest(unittest.TestCase):
    def test_arp_request_and_reply(self):
        # RFC 826 layout: htype 0001, ptype 0800, hlen 06, plen 04, oper, sha, spa, tha, tpa.
        req = bytes.fromhex(
            "ffffffffffff" "020000000002" "0806"
            "0001" "0800" "06" "04" "0001"
            "020000000002" "c0000202" "000000000000" "c0000201")
        rep = bytes.fromhex(
            "020000000002" "020000000001" "0806"
            "0001" "0800" "06" "04" "0002"
            "020000000001" "c0000201" "020000000002" "c0000202")
        cfg = Config(MAC1, bytes([192, 0, 2, 1]), bytes(16))
        self.assertEqual(eth.EthFrame(eth.BROADCAST, MAC2, 0x0806,
                                      arp.ArpPacket(1, MAC2, bytes([192, 0, 2, 2]), bytes(6),
                                                    bytes([192, 0, 2, 1])).build()).build(), req)
        self.assertEqual(respond(req + bytes(18), cfg), [rep])

    def test_icmp_echo_ipv4(self):
        # IPv4 header sum: 4500+0020+0000+4000+4001+0000+c000+0202+c000+0201 = 0x24924
        #   -> fold 0x4926 -> complement 0xb6d9.
        # ICMP: 0800+0000+1234+0001+6162+6364 = 0xdefb -> checksum 0x2104.
        req = bytes.fromhex(
            "020000000001" "020000000002" "0800"
            "45000020" "00004000" "4001" "b6d9" "c0000202" "c0000201"
            "0800" "2104" "1234" "0001" "61626364")
        # Reply: type 0 => checksum 0x2104 + 0x0800 = 0x2904; IP: ttl 64, src/dst swapped (sum unchanged).
        rep = bytes.fromhex(
            "020000000002" "020000000001" "0800"
            "45000020" "00004000" "4001" "b6d9" "c0000201" "c0000202"
            "0000" "2904" "1234" "0001" "61626364")
        cfg = Config(MAC1, bytes([192, 0, 2, 1]), bytes(16))
        self.assertEqual(ipv4.IPv4Packet.parse(req[14:]).payload[:2], b"\x08\x00")
        self.assertEqual(icmp.IcmpEcho(8, 0x1234, 1, b"abcd").build().hex(), "08002104123400016162" "6364")
        self.assertEqual(respond(req, cfg), [rep])

    def test_ntp_reply_payload(self):
        # RFC 5905 figure 8: byte0 = LI(2) VN(3) mode(3); stratum; poll; precision; root delay;
        # root dispersion; reference ID; reference, origin, receive, transmit timestamps.
        client_xmit = "0123456789abcdef"
        req_payload = bytes.fromhex("23" + "00" * 39 + client_xmit)
        self.assertEqual(len(req_payload), 48)
        # Request is NTPv4 (0x23) with poll 0; reply: LI 0, VN 4, mode 4 = 0x24, stratum 2,
        # poll 0, precision -24 = 0xe8, root delay 0x00000100 (1/256 s), dispersion 0x00000200,
        # refid c0000263.
        expect = bytes.fromhex(
            "24" "02" "00" "e8" "00000100" "00000200" "c0000263"
            "e000000000000001"          # reference timestamp
            "0123456789abcdef"          # origin = client transmit
            "e000000100000000"          # receive T2
            "e000000100001000")         # transmit T3
        cfg = Config(MAC1, bytes([192, 0, 2, 1]), bytes(16), stratum=2, precision=-24,
                     root_delay=0x100, root_dispersion=0x200, ref_ts=0xE000000000000001,
                     recv_ts=0xE000000100000000, xmit_ts=0xE000000100001000)
        from golden import udp
        u = udp.UdpDatagram(5000, 123, req_payload).build(bytes([192, 0, 2, 2]), bytes([192, 0, 2, 1]))
        ip = ipv4.IPv4Packet(bytes([192, 0, 2, 2]), bytes([192, 0, 2, 1]), 17, u).build()
        out = respond(eth.EthFrame(MAC1, MAC2, 0x0800, ip).build(), cfg)
        self.assertEqual(len(out), 1)
        rip = ipv4.IPv4Packet.parse(out[0][14:])
        self.assertEqual(rip.payload[8:], expect)
        self.assertEqual(rip.payload[:4].hex(), "007b1388")  # ports swapped: 123 -> 5000
        self.assertEqual(rip.payload[4:6].hex(), "0038")      # UDP length 8 + 48

    def test_na_icmpv6_body(self):
        # RFC 4861 section 4.4 NA: type 136, code 0, checksum, flags(R,S,O)=0x60000000, target,
        # option 2 (TLLA) length 1 with our MAC. Checksum value cross-checked with scapy.
        from golden import icmpv6, ipv6
        tgt = ipv6.addr("fe80::ff:fe00:1")
        src_ip = ipv6.addr("fe80::ff:fe00:2")
        body = icmpv6.NeighborAdvertisement(tgt, False, True, True, [(2, MAC1)]).build(tgt, src_ip)
        self.assertEqual(body.hex(), "88001b1c" "60000000" "fe80000000000000000000fffe000001"
                         "0201" "020000000001")


if __name__ == "__main__":
    unittest.main()
