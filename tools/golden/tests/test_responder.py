import random
import unittest

from golden import arp, eth, icmp, icmpv6, ipv4, ipv6, ntp, udp, vectors as V
from golden.responder import respond

CFG = V.CONFIG


def parse_eth(fr):
    return eth.EthFrame.parse(fr)


class ResponderTest(unittest.TestCase):
    def setUp(self):
        self.rng = random.Random(1)

    # ---- ARP
    def test_arp_reply(self):
        out = respond(V.rx_arp_request(), CFG)
        self.assertEqual(len(out), 1)
        f = parse_eth(out[0])
        self.assertEqual((f.dst, f.src, f.ethertype), (V.CLI_MAC, V.SRV_MAC, 0x0806))
        self.assertEqual(arp.ArpPacket.parse(f.payload),
                         arp.ArpPacket(2, V.SRV_MAC, V.SRV_IP4, V.CLI_MAC, V.CLI_IP4))

    def test_arp_negative(self):
        self.assertEqual(respond(V.rx_arp_request(tpa=bytes([192, 0, 2, 5])), CFG), [])
        rep = arp.ArpPacket(2, V.CLI_MAC, V.CLI_IP4, V.SRV_MAC, V.SRV_IP4).build()  # a reply, not a request
        self.assertEqual(respond(eth.EthFrame(V.SRV_MAC, V.CLI_MAC, 0x0806, rep).build(), CFG), [])
        # frame to a foreign unicast MAC
        req = arp.ArpPacket(1, V.CLI_MAC, V.CLI_IP4, bytes(6), V.SRV_IP4).build()
        self.assertEqual(respond(eth.EthFrame(V.FOREIGN_MAC, V.CLI_MAC, 0x0806, req).build(), CFG), [])
        # not Ethernet/IPv4
        bad = bytearray(req)
        bad[5] = 5
        self.assertEqual(respond(eth.EthFrame(eth.BROADCAST, V.CLI_MAC, 0x0806, bytes(bad)).build(), CFG), [])
        # multicast sender hardware address
        mc = arp.ArpPacket(1, bytes.fromhex("010000000002"), V.CLI_IP4, bytes(6), V.SRV_IP4).build()
        self.assertEqual(respond(eth.EthFrame(eth.BROADCAST, V.CLI_MAC, 0x0806, mc).build(), CFG), [])

    # ---- ICMP
    def test_ping4(self):
        rx = V.rx_ping4(self.rng, 0xABCD, 9, 40)
        out = respond(rx, CFG)
        self.assertEqual(len(out), 1)
        f = parse_eth(out[0])
        ip = ipv4.IPv4Packet.parse(f.payload)
        req_ip = ipv4.IPv4Packet.parse(parse_eth(rx).payload)
        e = icmp.IcmpEcho.parse(ip.payload)
        r = icmp.IcmpEcho.parse(req_ip.payload)
        self.assertEqual((e.type, e.ident, e.seq, e.data), (0, 0xABCD, 9, r.data))
        self.assertEqual((ip.src, ip.dst, f.dst, f.src), (V.SRV_IP4, V.CLI_IP4, V.CLI_MAC, V.SRV_MAC))

    def test_ping4_negative(self):
        self.assertEqual(respond(V.rx_ping4(self.rng, 1, 1, 8, ttl=0), CFG), [])
        self.assertEqual(respond(V.rx_ping4(self.rng, 1, 1, 8, dst_mac=eth.BROADCAST), CFG), [])
        self.assertEqual(respond(V.rx_ping4(self.rng, 1, 1, 8, df=False, mf=True), CFG), [])
        self.assertEqual(respond(V.rx_ping4(self.rng, 1, 1, 8, options=b"\x01\x01\x01\x00"), CFG), [])
        rx = V.rx_ping4(self.rng, 1, 1, 8)
        self.assertEqual(respond(V._flip_byte(rx, 14 + 22), CFG), [])   # ICMP checksum
        self.assertEqual(respond(V._flip_byte(rx, 14 + 16), CFG), [])   # IP dst/checksum
        # echo reply (type 0) is not answered
        rep = icmp.IcmpEcho(0, 1, 1, b"x").build()
        ip = ipv4.IPv4Packet(V.CLI_IP4, V.SRV_IP4, 1, rep).build()
        self.assertEqual(respond(eth.EthFrame(V.SRV_MAC, V.CLI_MAC, 0x0800, ip).build(), CFG), [])

    def test_ping6(self):
        for dst in (V.SRV_GUA, V.SRV_LL):
            out = respond(V.rx_ping6(self.rng, 7, 3, 24, dst=dst), CFG)
            self.assertEqual(len(out), 1)
            ip = ipv6.IPv6Packet.parse(parse_eth(out[0]).payload)
            e = icmpv6.Icmpv6Echo.parse(ip.src, ip.dst, ip.payload)
            self.assertEqual((e.type, e.ident, e.seq, len(e.data)), (129, 7, 3, 24))
            self.assertEqual((ip.src, ip.dst), (dst, V.CLI_GUA))

    def test_ping6_negative(self):
        rx = V.rx_ping6(self.rng, 1, 1, 8)
        self.assertEqual(respond(V._flip_byte(rx, 14 + 40 + 5), CFG), [])
        self.assertEqual(respond(V.rx_ping6(self.rng, 1, 1, 8, dst=ipv6.addr("2001:db8::99")), CFG), [])
        # hop limit 0
        ip = bytearray(parse_eth(rx).payload)
        ip[7] = 0
        self.assertEqual(respond(eth.EthFrame(V.SRV_MAC, V.CLI_MAC, 0x86DD, bytes(ip)).build(), CFG), [])
        # echo to the solicited-node multicast address is not answered
        sn = ipv6.solicited_node(V.SRV_GUA)
        body = icmpv6.Icmpv6Echo(128, 1, 1, b"").build(V.CLI_GUA, sn)
        pkt = ipv6.IPv6Packet(V.CLI_GUA, sn, 58, body).build()
        self.assertEqual(respond(eth.EthFrame(ipv6.multicast_mac(sn), V.CLI_MAC, 0x86DD, pkt).build(), CFG), [])

    # ---- NDP
    def check_na(self, out, target, dst_ip, dst_mac, solicited=True):
        self.assertEqual(len(out), 1)
        f = parse_eth(out[0])
        ip = ipv6.IPv6Packet.parse(f.payload)
        self.assertEqual(ip.hop_limit, 255)
        self.assertEqual((ip.src, ip.dst, f.dst, f.src), (target, dst_ip, dst_mac, V.SRV_MAC))
        na = icmpv6.NeighborAdvertisement.parse(ip.src, ip.dst, ip.payload)
        self.assertEqual((na.target, na.router, na.solicited, na.override), (target, False, solicited, True))
        self.assertEqual(na.options, [(icmpv6.OPT_TLLA, V.SRV_MAC)])

    def test_ns_multicast_and_unicast(self):
        self.check_na(respond(V.rx_ns(), CFG), V.SRV_LL, V.CLI_LL, V.CLI_MAC)
        self.check_na(respond(V.rx_ns(target=V.SRV_GUA), CFG), V.SRV_GUA, V.CLI_LL, V.CLI_MAC)
        self.check_na(respond(V.rx_ns(target=V.SRV_GUA, via_multicast=False), CFG), V.SRV_GUA, V.CLI_LL, V.CLI_MAC)

    def test_dad_probe(self):
        sn = ipv6.solicited_node(V.SRV_LL)
        body = icmpv6.NeighborSolicitation(V.SRV_LL).build(bytes(16), sn)
        pkt = ipv6.IPv6Packet(bytes(16), sn, 58, body, hop_limit=255).build()
        fr = V._wire(eth.EthFrame(ipv6.multicast_mac(sn), V.CLI_MAC, 0x86DD, pkt).build())
        allnodes = ipv6.addr("ff02::1")
        self.check_na(respond(fr, CFG), V.SRV_LL, allnodes, bytes.fromhex("333300000001"), solicited=False)

    def test_ns_negative(self):
        self.assertEqual(respond(V.rx_ns(hop_limit=64), CFG), [])
        self.assertEqual(respond(V.rx_ns(hop_limit=0), CFG), [])
        self.assertEqual(respond(V.rx_ns(target=ipv6.addr("fe80::1234")), CFG), [])  # not our address
        fr = V.rx_ns()
        self.assertEqual(respond(V._flip_byte(fr, 14 + 40 + 9), CFG), [])           # checksum
        # wrong multicast MAC for the solicited-node address
        f = parse_eth(fr)
        self.assertEqual(respond(eth.EthFrame(bytes.fromhex("333300000001"), f.src, 0x86DD, f.payload).build(), CFG), [])
        # dst IP unrelated to target
        body = icmpv6.NeighborSolicitation(V.SRV_LL).build(V.CLI_LL, V.SRV_GUA)
        pkt = ipv6.IPv6Packet(V.CLI_LL, V.SRV_GUA, 58, body, hop_limit=255).build()
        self.assertEqual(respond(eth.EthFrame(V.SRV_MAC, V.CLI_MAC, 0x86DD, pkt).build(), CFG), [])
        # NA / RA received: nothing sent
        na = icmpv6.NeighborAdvertisement(V.CLI_LL).build(V.CLI_LL, V.SRV_LL)
        pkt = ipv6.IPv6Packet(V.CLI_LL, V.SRV_LL, 58, na, hop_limit=255).build()
        self.assertEqual(respond(eth.EthFrame(V.SRV_MAC, V.CLI_MAC, 0x86DD, pkt).build(), CFG), [])
        ra = icmpv6.RouterAdvertisement(64, False, False, 1800, 0, 0,
                                        [icmpv6.prefix_option(ipv6.addr("2001:db8:1::"), 64, 100, 50)])
        allnodes = ipv6.addr("ff02::1")
        pkt = ipv6.IPv6Packet(V.CLI_LL, allnodes, 58, ra.build(V.CLI_LL, allnodes), hop_limit=255).build()
        self.assertEqual(respond(eth.EthFrame(ipv6.multicast_mac(allnodes), V.CLI_MAC, 0x86DD, pkt).build(), CFG), [])

    # ---- NTP
    def reply_payload(self, out, v6):
        self.assertEqual(len(out), 1)
        f = parse_eth(out[0])
        if v6:
            ip = ipv6.IPv6Packet.parse(f.payload)
            body = ip.payload
        else:
            ip = ipv4.IPv4Packet.parse(f.payload)
            body = ip.payload
        d = udp.UdpDatagram.parse(ip.src, ip.dst, body)
        self.assertEqual((d.sport, d.dport), (123, 40000))
        self.assertEqual(len(d.payload), 48)
        return ip, d, ntp.NtpPacket.parse(d.payload)

    def test_ntp_v4(self):
        req = V.ntp_request(self.rng)
        ip, d, r = self.reply_payload(respond(V.rx_ntp4(req), CFG), False)
        q = ntp.NtpPacket.parse(req)
        self.assertEqual((ip.src, ip.dst), (V.SRV_IP4, V.CLI_IP4))
        self.assertEqual((r.li, r.vn, r.mode, r.stratum, r.poll, r.precision), (0, 4, 4, 2, q.poll, -24))
        self.assertEqual((r.orig_ts, r.recv_ts, r.xmit_ts, r.ref_ts), (q.xmit_ts, V.T_RECV, V.T_XMIT, V.T_REF))
        self.assertEqual(r.ref_id, CFG.ref_id)
        self.assertEqual((r.root_delay, r.root_dispersion), (CFG.root_delay, CFG.root_dispersion))
        self.assertGreaterEqual(r.xmit_ts, r.recv_ts)

    def test_ntp_source_port_rule_n1b(self):
        # N1b (F8): source ports 0, 7, 13, 17, 19, 37 get no answer; any other does.
        req = V.ntp_request(self.rng)
        for sport in (0, 7, 13, 17, 19, 37):
            u = udp.UdpDatagram(sport, 123, req).build(V.CLI_IP4, V.SRV_IP4)
            ip = ipv4.IPv4Packet(V.CLI_IP4, V.SRV_IP4, ipv4.PROTO_UDP, u).build()
            fr = V._wire(eth.EthFrame(V.SRV_MAC, V.CLI_MAC, eth.ETHERTYPE_IPV4, ip).build())
            self.assertEqual(respond(fr, CFG), [], sport)
            u6 = udp.UdpDatagram(sport, 123, req).build(V.CLI_GUA, V.SRV_GUA)
            ip6 = ipv6.IPv6Packet(V.CLI_GUA, V.SRV_GUA, ipv6.NH_UDP, u6).build()
            fr6 = V._wire(eth.EthFrame(V.SRV_MAC, V.CLI_MAC, eth.ETHERTYPE_IPV6, ip6).build())
            self.assertEqual(respond(fr6, CFG), [], sport)
        for sport in (1, 6, 8, 12, 18, 20, 36, 38, 123, 1024, 40000, 65535):
            u = udp.UdpDatagram(sport, 123, req).build(V.CLI_IP4, V.SRV_IP4)
            ip = ipv4.IPv4Packet(V.CLI_IP4, V.SRV_IP4, ipv4.PROTO_UDP, u).build()
            fr = V._wire(eth.EthFrame(V.SRV_MAC, V.CLI_MAC, eth.ETHERTYPE_IPV4, ip).build())
            self.assertEqual(len(respond(fr, CFG)), 1, sport)

    def test_ntp_v6(self):
        for dst, src in ((V.SRV_GUA, V.CLI_GUA), (V.SRV_LL, V.CLI_LL)):
            req = V.ntp_request(self.rng)
            ip, d, r = self.reply_payload(respond(V.rx_ntp6(req, dst=dst, src=src), CFG), True)
            self.assertEqual((ip.src, ip.dst), (dst, src))
            self.assertEqual(r.ref_id, CFG.ref_id_v6)
            self.assertEqual(r.orig_ts, ntp.NtpPacket.parse(req).xmit_ts)
            self.assertEqual((r.mode, r.vn, r.stratum), (4, 4, 2))

    def test_ntp_version_rules(self):
        for vn, want in ((3, 3), (4, 4)):
            _, _, r = self.reply_payload(respond(V.rx_ntp4(V.ntp_request(self.rng, vn=vn)), CFG), False)
            self.assertEqual(r.vn, want)
        for vn in (0, 1, 2, 5, 7):
            self.assertEqual(respond(V.rx_ntp4(V.ntp_request(self.rng, vn=vn)), CFG), [])

    def test_ntp_modes(self):
        for mode in (0, 1, 2, 4, 5, 6, 7):
            self.assertEqual(respond(V.rx_ntp4(V.ntp_request(self.rng, mode=mode)), CFG), [], mode)

    def test_ntp_unsynchronised(self):
        cfg = V._cfg(synchronised=False)
        for out, v6 in ((respond(V.rx_ntp4(V.ntp_request(self.rng)), cfg), False),
                        (respond(V.rx_ntp6(V.ntp_request(self.rng)), cfg), True)):
            _, _, r = self.reply_payload(out, v6)
            self.assertEqual((r.li, r.stratum), (3, 16))

    def test_ntp_request_with_extension_gets_48_bytes(self):
        _, _, r = self.reply_payload(respond(V.rx_ntp4(V.ntp_request(self.rng) + bytes(20)), CFG), False)
        self.assertEqual(r.extensions, b"")

    def test_ntp_negative(self):
        ok = V.ntp_request(self.rng)
        good = V.rx_ntp4(ok)
        self.assertEqual(len(respond(good, CFG)), 1)
        self.assertEqual(respond(V._flip_byte(good, 14 + 10), CFG), [])             # IP checksum
        self.assertEqual(respond(V._flip_byte(good, 14 + 20 + 6), CFG), [])          # UDP checksum
        self.assertEqual(respond(V.rx_ntp4(ok, dport=124), CFG), [])
        self.assertEqual(respond(V.rx_ntp4(ok, ttl=0), CFG), [])
        self.assertEqual(respond(V.rx_ntp4(ok, df=False, mf=True), CFG), [])
        self.assertEqual(respond(V.rx_ntp4(ok, df=False, frag_offset=5), CFG), [])
        self.assertEqual(respond(V.rx_ntp4(ok, options=b"\x01\x01\x01\x00"), CFG), [])
        self.assertEqual(respond(V.rx_ntp4(ok, dst_mac=V.FOREIGN_MAC), CFG), [])
        self.assertEqual(respond(V.rx_ntp4(ok[:47]), CFG), [])                       # short NTP
        self.assertEqual(respond(good[:13], CFG), [])                                # runt
        self.assertEqual(respond(good[:30], CFG), [])                                # truncated IP
        self.assertEqual(respond(b"", CFG), [])
        # wrong destination IP
        u = udp.UdpDatagram(1, 123, ok).build(V.CLI_IP4, bytes([192, 0, 2, 9]))
        ip = ipv4.IPv4Packet(V.CLI_IP4, bytes([192, 0, 2, 9]), 17, u).build()
        self.assertEqual(respond(eth.EthFrame(V.SRV_MAC, V.CLI_MAC, 0x0800, ip).build(), CFG), [])
        # broadcast source
        u = udp.UdpDatagram(1, 123, ok).build(b"\xff" * 4, V.SRV_IP4)
        ip = ipv4.IPv4Packet(b"\xff" * 4, V.SRV_IP4, 17, u).build()
        self.assertEqual(respond(eth.EthFrame(V.SRV_MAC, V.CLI_MAC, 0x0800, ip).build(), CFG), [])
        # other protocol, other EtherType
        ip = ipv4.IPv4Packet(V.CLI_IP4, V.SRV_IP4, 6, bytes(20)).build()
        self.assertEqual(respond(eth.EthFrame(V.SRV_MAC, V.CLI_MAC, 0x0800, ip).build(), CFG), [])
        self.assertEqual(respond(eth.EthFrame(V.SRV_MAC, V.CLI_MAC, 0x88CC, bytes(50)).build(), CFG), [])

    def test_ntp_v4_zero_checksum_accepted(self):
        u = udp.UdpDatagram(40000, 123, V.ntp_request(self.rng)).build(V.CLI_IP4, V.SRV_IP4, zero_checksum=True)
        ip = ipv4.IPv4Packet(V.CLI_IP4, V.SRV_IP4, 17, u).build()
        out = respond(V._wire(eth.EthFrame(V.SRV_MAC, V.CLI_MAC, 0x0800, ip).build()), CFG)
        self.assertEqual(len(out), 1)

    def test_ntp_v6_negative(self):
        ok = V.ntp_request(self.rng)
        good = V.rx_ntp6(ok)
        self.assertEqual(len(respond(good, CFG)), 1)
        self.assertEqual(respond(V._flip_byte(good, 14 + 40 + 6), CFG), [])
        self.assertEqual(respond(V.rx_ntp6(ok, dst=ipv6.addr("2001:db8::77")), CFG), [])
        # zero UDP checksum is illegal in IPv6
        z = bytearray(good)
        z[14 + 40 + 6:14 + 40 + 8] = b"\x00\x00"
        self.assertEqual(respond(bytes(z), CFG), [])
        # hop limit 0
        h = bytearray(good)
        h[14 + 7] = 0
        self.assertEqual(respond(bytes(h), CFG), [])
        # extension header (hop-by-hop, then UDP) is not handled
        ip = ipv6.IPv6Packet.parse(parse_eth(good).payload)
        hbh = bytes([17, 0, 1, 4, 0, 0, 0, 0]) + ip.payload
        pkt = ipv6.IPv6Packet(ip.src, ip.dst, 0, hbh).build()
        self.assertEqual(respond(eth.EthFrame(V.SRV_MAC, V.CLI_MAC, 0x86DD, pkt).build(), CFG), [])
        # NTP to the solicited-node multicast address is ignored
        sn = ipv6.solicited_node(V.SRV_GUA)
        u = udp.UdpDatagram(1, 123, ok).build(V.CLI_GUA, sn)
        pkt = ipv6.IPv6Packet(V.CLI_GUA, sn, 17, u).build()
        self.assertEqual(respond(eth.EthFrame(ipv6.multicast_mac(sn), V.CLI_MAC, 0x86DD, pkt).build(), CFG), [])
        # foreign MAC
        f = parse_eth(good)
        self.assertEqual(respond(eth.EthFrame(V.FOREIGN_MAC, f.src, 0x86DD, f.payload).build(), CFG), [])

    def test_reply_not_larger_than_request_rule_documented(self):
        # The reply is always 48 bytes of NTP; requests are >= 48 (no amplification, formal.md).
        out = respond(V.rx_ntp4(V.ntp_request(self.rng)), CFG)
        self.assertLessEqual(len(out[0]), len(V.rx_ntp4(V.ntp_request(self.rng))))


if __name__ == "__main__":
    unittest.main()
