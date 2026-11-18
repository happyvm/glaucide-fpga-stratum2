"""Optional cross-check against scapy (skipped when scapy is not installed).

The model never imports scapy. Here every good frame of every vector is dissected by scapy,
all its checksums are deleted and recomputed by scapy, and the result must be byte-identical
to ours: this validates checksums and layouts independently of our own code.
"""
import logging
import os
import unittest

try:
    logging.getLogger("scapy.runtime").setLevel(logging.ERROR)
    from scapy.all import ARP, Ether, ICMP, IP, IPv6, NTP, UDP  # noqa: F401
    from scapy.layers.inet6 import ICMPv6ND_NA, ICMPv6ND_NS, _ICMPv6
    HAVE = True
except Exception:  # pragma: no cover
    HAVE = False

from golden import vectors as V
from tests.test_vectors import EXPECTED, read_hex


def _good_frames():
    for name in sorted(EXPECTED):
        steps = read_hex(os.path.join(V.DEFAULT_OUT, name + ".hex"))
        if name.startswith("neg_"):
            steps = steps[:2]  # the third line is the deliberately broken frame
        for d, fr in steps:
            yield name, d, fr


@unittest.skipUnless(HAVE, "scapy not installed")
class ScapyTest(unittest.TestCase):
    def test_scapy_recomputes_identical_bytes(self):
        n = 0
        for name, d, fr in _good_frames():
            pkt = Ether(fr)
            if not (pkt.haslayer(IP) or pkt.haslayer(IPv6)):
                continue
            for layer in (IP, UDP, ICMP, _ICMPv6):
                if pkt.haslayer(layer) and hasattr(pkt[layer], "chksum"):
                    del pkt[layer].chksum
            if pkt.haslayer(IP):
                del pkt[IP].len
            if pkt.haslayer(IPv6):
                del pkt[IPv6].plen
            if pkt.haslayer(UDP):
                del pkt[UDP].len
            self.assertEqual(bytes(pkt), fr, "%s %s" % (name, d))
            n += 1
        self.assertGreater(n, 20)

    def test_ntp_fields(self):
        for d, fr in read_hex(os.path.join(V.DEFAULT_OUT, "ntp4_basic.hex")):
            n = Ether(fr)[NTP]
            if d == "tx":
                self.assertEqual((n.mode, n.version, n.stratum, n.leap), (4, 4, 2, 0))
            else:
                self.assertEqual((n.mode, n.version), (3, 4))

    def test_ndp_fields(self):
        steps = read_hex(os.path.join(V.DEFAULT_OUT, "ndp_ns_na.hex"))
        self.assertEqual(Ether(steps[0][1])[ICMPv6ND_NS].tgt, "fe80::ff:fe00:1")
        na = Ether(steps[1][1])[ICMPv6ND_NA]
        self.assertEqual((na.R, na.S, na.O), (0, 1, 1))

    def test_icmp_and_arp_fields(self):
        steps = read_hex(os.path.join(V.DEFAULT_OUT, "ping4.hex"))
        self.assertEqual(Ether(steps[1][1])[ICMP].type, 0)
        steps = read_hex(os.path.join(V.DEFAULT_OUT, "arp_request_reply.hex"))
        self.assertEqual(Ether(steps[1][1])[ARP].op, 2)


if __name__ == "__main__":
    unittest.main()
