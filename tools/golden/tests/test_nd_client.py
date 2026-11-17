"""IPv6 neighbor discovery, initiator side (T3.6): solicit() and learn() of the model."""
import unittest

from golden import eth, icmpv6, ipv6, responder
from golden.responder import Config, learn, solicit

MAC = bytes.fromhex("020000000001")
PEER_MAC = bytes.fromhex("020000000002")
LL = ipv6.addr("fe80::ff:fe00:1")
GUA = ipv6.addr("2001:db8::1")
PEER = ipv6.addr("2001:db8::2")
PEER_LL = ipv6.addr("fe80::ff:fe00:2")
CFG = Config(mac=MAC, ipv4=bytes([192, 0, 2, 1]), ipv6_ll=LL, ipv6_global=GUA)


def na_frame(src, dst, target, hop=255, opts=None, dmac=MAC, solicited=True):
    body = icmpv6.NeighborAdvertisement(target, False, solicited, True, opts or []).build(src, dst)
    pkt = ipv6.IPv6Packet(src, dst, ipv6.NH_ICMPV6, body, hop_limit=hop).build()
    return eth.EthFrame(dmac, PEER_MAC, eth.ETHERTYPE_IPV6, pkt).build()


class TestSolicit(unittest.TestCase):
    def test_global_target_uses_global_source(self):
        f = eth.EthFrame.parse(solicit(PEER, CFG))
        self.assertEqual(f.dst, bytes.fromhex("3333ff000002"))
        p = ipv6.IPv6Packet.parse(f.payload)
        self.assertEqual((p.src, p.dst, p.hop_limit), (GUA, ipv6.solicited_node(PEER), 255))
        ns = icmpv6.NeighborSolicitation.parse(p.src, p.dst, p.payload)
        self.assertEqual((ns.target, ns.options), (PEER, [(icmpv6.OPT_SLLA, MAC)]))

    def test_link_local_target_uses_link_local_source(self):
        p = ipv6.IPv6Packet.parse(eth.EthFrame.parse(solicit(PEER_LL, CFG)).payload)
        self.assertEqual(p.src, LL)

    def test_no_global_address(self):
        cfg = Config(mac=MAC, ipv4=bytes(4), ipv6_ll=LL)
        p = ipv6.IPv6Packet.parse(eth.EthFrame.parse(solicit(PEER, cfg)).payload)
        self.assertEqual(p.src, LL)

    def test_own_address_dropped(self):
        self.assertIsNone(solicit(LL, CFG))
        self.assertIsNone(solicit(GUA, CFG))


class TestLearn(unittest.TestCase):
    """F5: an NA is learnt only if it answers our outstanding solicitation; an NS never teaches."""

    def test_na_with_and_without_tlla(self):
        other = bytes.fromhex("0200000000aa")
        self.assertEqual(learn(na_frame(PEER, GUA, PEER, opts=[(2, other)]), CFG, PEER), (PEER, other))
        self.assertEqual(learn(na_frame(PEER, GUA, PEER), CFG, PEER), (PEER, PEER_MAC))

    def test_na_rejected(self):
        self.assertIsNone(learn(na_frame(PEER, GUA, PEER, hop=254), CFG, PEER))
        self.assertIsNone(learn(na_frame(PEER, GUA, ipv6.addr("ff02::1")), CFG, ipv6.addr("ff02::1")))
        self.assertIsNone(learn(na_frame(PEER, GUA, PEER, opts=[(2, MAC), (2, MAC)]), CFG, PEER))

    def test_unsolicited_na_is_not_learnt(self):
        self.assertIsNone(learn(na_frame(PEER, GUA, PEER), CFG))
        self.assertIsNone(learn(na_frame(PEER, GUA, PEER), CFG, None))

    def test_na_without_s_flag(self):
        self.assertIsNone(learn(na_frame(PEER, GUA, PEER, solicited=False), CFG, PEER))

    def test_na_for_another_target(self):
        self.assertIsNone(learn(na_frame(PEER, GUA, PEER), CFG, PEER_LL))
        self.assertIsNone(learn(na_frame(PEER_LL, GUA, PEER_LL), CFG, PEER))

    def test_na_with_spoofed_source(self):
        # RFC 4861 section 7.1.2: a solicited NA comes from the target.
        self.assertIsNone(learn(na_frame(PEER_LL, GUA, PEER), CFG, PEER))

    def test_ns_with_slla_teaches_nothing(self):
        ns = icmpv6.NeighborSolicitation(GUA, [(1, PEER_MAC)])
        dst = ipv6.solicited_node(GUA)
        pkt = ipv6.IPv6Packet(PEER, dst, ipv6.NH_ICMPV6, ns.build(PEER, dst), hop_limit=255).build()
        fr = eth.EthFrame(ipv6.multicast_mac(dst), PEER_MAC, eth.ETHERTYPE_IPV6, pkt).build()
        self.assertIsNone(learn(fr, CFG))
        self.assertIsNone(learn(fr, CFG, PEER))
        self.assertTrue(responder.respond(fr, CFG))  # still answered

    def test_dad_probe_not_learnt(self):
        ns = icmpv6.NeighborSolicitation(GUA, [])
        dst = ipv6.solicited_node(GUA)
        src = bytes(16)
        pkt = ipv6.IPv6Packet(src, dst, ipv6.NH_ICMPV6, ns.build(src, dst), hop_limit=255).build()
        fr = eth.EthFrame(ipv6.multicast_mac(dst), PEER_MAC, eth.ETHERTYPE_IPV6, pkt).build()
        self.assertIsNone(learn(fr, CFG))


if __name__ == "__main__":
    unittest.main()
