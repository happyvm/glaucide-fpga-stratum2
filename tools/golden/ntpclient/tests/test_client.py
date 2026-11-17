import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import client  # noqa: E402
from client import Reject, Request, Sample, build_reply, make_sample  # noqa: E402

SEC = 1 << 32
A4 = 0xC0000263
A6 = 0x20010DB8000000000000000000000099
T1 = 0xEC000000_00000000 + 100 * SEC


def good(org=T1, t2=T1 + SEC // 100, t3=T1 + SEC // 100 + 1000, **kw):
    base = dict(stratum=1, precision=0xE9, root_delay=0x10, root_disp=0x20,
                ref_id=0x47505300, org=org, t2=t2, t3=t3)
    base.update(kw)
    return build_reply(**base)


class RequestTest(unittest.TestCase):
    def test_layout(self):
        r = client.build_request(6, 0xE8, 0x0123456789ABCDEF)
        self.assertEqual(len(r), 48)
        self.assertEqual(r[:4], bytes([0x23, 0, 6, 0xE8]))
        self.assertEqual(r[4:40], bytes(36))
        self.assertEqual(r[40:], bytes.fromhex("0123456789abcdef"))
        self.assertEqual(r[0] >> 6, 0)
        self.assertEqual((r[0] >> 3) & 7, 4)
        self.assertEqual(r[0] & 7, 3)


class ArithTest(unittest.TestCase):
    def test_symmetric_path(self):
        # server 1 s ahead, 10 ms each way, 1 ms processing
        t1 = T1
        t2 = t1 + SEC + SEC // 100
        t3 = t2 + SEC // 1000
        t4 = t1 + 2 * (SEC // 100) + SEC // 1000
        off, dly, disp = client.arith(t1, t2, t3, t4, 0xE8, 0xE8)
        self.assertEqual(off, SEC)
        self.assertEqual(dly, 2 * (SEC // 100))
        iv = t4 - t1
        self.assertEqual(disp, 2 * (1 << 8) + (client.PHI_Q32 * iv + client.M32) // SEC)

    def test_floor_halving(self):
        # a + b = -1 -> offset -1 (floor), a + b = 1 -> 0
        off, _, _ = client.arith(10, 10, 9, 10, 0xE8, 0xE8)
        self.assertEqual(off, client.M64)
        off, _, _ = client.arith(10, 11, 10, 10, 0xE8, 0xE8)
        self.assertEqual(off, 0)

    def test_negative_delay_clamped(self):
        _, dly, _ = client.arith(0, 0, 5 * SEC, 1, 0xE8, 0xE8)
        self.assertEqual(dly, 0)

    def test_extreme_offsets(self):
        big = (1 << 63) - 2
        off, _, _ = client.arith(0, big, big, 0, 0xE8, 0xE8)
        self.assertEqual(client.s64(off), big)
        off, _, _ = client.arith(0, (-big) & client.M64, (-big) & client.M64, 0, 0xE8, 0xE8)
        self.assertEqual(client.s64(off), -big)

    def test_era_wrap(self):
        t1 = client.M64 - SEC // 40
        x = SEC // 20
        t4 = (t1 + 2 * x) & client.M64
        t2 = (t1 + x) & client.M64
        self.assertLess(t2, t1)
        off, dly, _ = client.arith(t1, t2, t2, t4, 0xE8, 0xE8)
        self.assertEqual(off, 0)
        self.assertEqual(dly, 2 * x)

    def test_iv_negative(self):
        _, _, disp = client.arith(10 * SEC, 10 * SEC, 10 * SEC, 9 * SEC, 0xE8, 0xE8)
        self.assertEqual(disp, 2 << 8)

    def test_prec_floor(self):
        self.assertEqual(client.prec_floor(0xE8), 1 << 8)
        self.assertEqual(client.prec_floor(0xE0), 1)
        self.assertEqual(client.prec_floor(0xDF), 1)
        self.assertEqual(client.prec_floor(0x05), 1 << 32)


class AcceptTest(unittest.TestCase):
    req4 = Request(A4, False, T1)
    req6 = Request(A6, True, T1)

    def test_good(self):
        t4 = T1 + SEC // 50
        s = make_sample(2, self.req4, good(), t4)
        self.assertIsInstance(s, Sample)
        self.assertEqual((s.peer, s.t4, s.stratum, s.leap, s.ref_id, s.root_delay, s.root_disp),
                         (2, t4, 1, 0, 0x47505300, 0x10, 0x20))
        self.assertIsInstance(make_sample(1, self.req6, good(), t4), Sample)
        self.assertIsInstance(make_sample(1, self.req4, good(vn=3, li=1), t4), Sample)
        self.assertIsInstance(make_sample(1, self.req4, good(ext=bytes(20)), t4), Sample)

    def test_rejects(self):
        t4 = T1 + SEC // 50
        cases = [
            (dict(), dict(err=True), client.R_ERR, False),
            (dict(), dict(src=A4 + 1), client.R_SOURCE, False),
            (dict(), dict(v6=True), client.R_SOURCE, False),
            (dict(), dict(sport=1234), client.R_SOURCE, False),
            (dict(mode=3), dict(), client.R_MODE, False),
            (dict(vn=2), dict(), client.R_VERSION, False),
            (dict(vn=5), dict(), client.R_VERSION, False),
            (dict(org=T1 + 1), dict(), client.R_ORIGIN, False),
            (dict(stratum=0), dict(), client.R_KOD, True),
            (dict(stratum=16), dict(), client.R_STRATUM, True),
            (dict(li=3), dict(), client.R_LEAP, True),
        ]
        for rkw, mkw, reason, miss in cases:
            r = make_sample(0, self.req4, good(**rkw), t4, **mkw)
            self.assertEqual(r, Reject(reason, miss), (rkw, mkw))
        self.assertEqual(make_sample(0, self.req4, good()[:47], t4), Reject(client.R_SHORT))
        idle = Request(A4, False, T1, outstanding=False)
        self.assertEqual(make_sample(0, idle, good(), t4), Reject(client.R_IDLE))

    def test_spoofed_kod_is_not_a_miss(self):
        r = make_sample(0, self.req4, good(stratum=0, org=T1 ^ 1), T1)
        self.assertEqual(r, Reject(client.R_ORIGIN, False))


class AddressTest(unittest.TestCase):
    def test_next_hop4(self):
        self.assertEqual(client.next_hop4(0xC0000263, 0xC0000201, 0xFFFFFF00, 0xC00002FE),
                         0xC0000263)
        self.assertEqual(client.next_hop4(0xC6336401, 0xC0000201, 0xFFFFFF00, 0xC00002FE),
                         0xC00002FE)

    def test_next_hop6(self):
        gua, gw = 0x20010DB8000000000000000000000001, 0xFE800000000000000000000000000001
        ll_peer = 0xFE800000000000000000000000000063
        self.assertEqual(client.next_hop6(ll_peer, gua, True, gw), ll_peer)
        self.assertEqual(client.next_hop6(A6, gua, True, gw), A6)
        self.assertEqual(client.next_hop6(A6, gua, False, gw), gw)
        other = 0x20010DB8000100000000000000000001
        self.assertEqual(client.next_hop6(other, gua, True, gw), gw)
        self.assertEqual(client.src6(ll_peer, 1, gua, True), 1)
        self.assertEqual(client.src6(A6, 1, gua, True), gua)
        self.assertEqual(client.src6(A6, 1, gua, False), 1)

    def test_schedule(self):
        self.assertEqual(client.schedule(6, 6), [0, 2, 4, 6, 70, 134])
        self.assertEqual(client.poll_interval(2), 16)
        self.assertEqual(client.poll_interval(30), 1 << 17)


class VectorTest(unittest.TestCase):
    def test_vectors_up_to_date(self):
        path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            "vectors", "client.txt")
        with open(path) as fh:
            text = fh.read()
        want = client.HEADER + "".join(client.vector_line(c) + "\n" for c in client.gen_cases())
        self.assertEqual(text, want)
        self.assertGreater(text.count(" -> S "), 150)


if __name__ == "__main__":
    unittest.main()
