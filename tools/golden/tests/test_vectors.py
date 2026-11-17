import os
import tempfile
import unittest

from golden import vectors as V
from golden.responder import respond

EXPECTED = {"arp_request_reply", "ping4", "ping6", "ndp_ns_na", "ntp4_basic", "ntp6_basic",
            "ntp_unsynchronised", "ntp_version3", "neg_ipv4_checksum", "neg_udp6_checksum",
            "neg_udp_port", "neg_ipv4_fragment", "neg_ipv4_ttl0", "neg_ndp_hoplimit",
            "neg_ntp_mode4", "neg_foreign_mac", "neg_runt"}


def _slurp(path):
    with open(path, "rb") as fh:
        return fh.read()


def read_hex(path):
    steps = []
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            d, h = line.split(" ")
            steps.append((d, bytes.fromhex(h)))
    return steps


class VectorsTest(unittest.TestCase):
    def test_deterministic(self):
        with tempfile.TemporaryDirectory() as a, tempfile.TemporaryDirectory() as b:
            V.write_all(a)
            V.write_all(b)
            self.assertEqual(sorted(os.listdir(a)), sorted(os.listdir(b)))
            for n in os.listdir(a):
                self.assertEqual(_slurp(os.path.join(a, n)), _slurp(os.path.join(b, n)))

    def test_committed_files_are_current(self):
        with tempfile.TemporaryDirectory() as a:
            V.write_all(a)
            self.assertEqual(sorted(os.listdir(a)), sorted(os.listdir(V.DEFAULT_OUT)))
            for n in os.listdir(a):
                self.assertEqual(_slurp(os.path.join(a, n)), _slurp(os.path.join(V.DEFAULT_OUT, n)), n)

    def test_scenario_set(self):
        self.assertEqual(set(V.scenarios()), EXPECTED)

    def test_files_replay_through_responder(self):
        for name, (_d, cfg, _rx) in V.scenarios().items():
            steps = read_hex(os.path.join(V.DEFAULT_OUT, name + ".hex"))
            i = 0
            while i < len(steps):
                self.assertEqual(steps[i][0], "rx")
                j = i + 1
                exp = []
                while j < len(steps) and steps[j][0] == "tx":
                    exp.append(steps[j][1])
                    j += 1
                self.assertEqual(respond(steps[i][1], cfg), exp, name)
                i = j

    def test_negative_files_second_frame_silent(self):
        for name in EXPECTED:
            if name.startswith("neg_"):
                steps = read_hex(os.path.join(V.DEFAULT_OUT, name + ".hex"))
                self.assertEqual([d for d, _ in steps], ["rx", "tx", "rx"], name)

    def test_positive_files_all_answered(self):
        for name in EXPECTED - {n for n in EXPECTED if n.startswith("neg_")} - {"arp_request_reply"}:
            steps = read_hex(os.path.join(V.DEFAULT_OUT, name + ".hex"))
            self.assertEqual([d for d, _ in steps].count("rx"), [d for d, _ in steps].count("tx"), name)

    def test_reserved_addresses_and_no_fcs_convention(self):
        for name in EXPECTED:
            for d, fr in read_hex(os.path.join(V.DEFAULT_OUT, name + ".hex")):
                if d == "rx" and name != "neg_runt":
                    self.assertGreaterEqual(len(fr), 60)
                if d == "tx":
                    self.assertLessEqual(len(fr), 1514)
                self.assertEqual(fr[0:1] in (b"\x02", b"\xff", b"\x33"), True)
                if fr[12:14] == b"\x08\x00":
                    self.assertEqual(fr[26:29], b"\xc0\x00\x02")  # 192.0.2.0/24


if __name__ == "__main__":
    unittest.main()
