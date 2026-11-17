import math
import os
import random
import unittest
from fractions import Fraction

from ntpclient import filter as F
from ntpclient.filter import (ClockFilter, FloatFilter, Sample, MAXDISP, PHI_Q32, M64,
                              from_signed, to_signed)

SEC = 1 << 32
MS = SEC // 1000
T0 = 0xEC00_0000 * SEC


def smp(peer=0, off=0, dly=10 * MS, disp=1000, t4=T0, **kw):
    return Sample(peer, from_signed(off), dly, disp, t4, **kw)


def feed(flt, samples):
    st = None
    for s in samples:
        st = flt.sample(s)
    return st


class ModelTests(unittest.TestCase):
    def test_single_sample(self):
        f = ClockFilter()
        st = f.sample(smp(2, off=-5 * MS, dly=7 * MS, disp=1 << 20, t4=T0, stratum=2,
                          leap=0, ref_id=0xC0000201, root_delay=0x123, root_disp=0x456))
        self.assertEqual(st["peer"], 2)
        self.assertEqual(st["valid"], 1)
        self.assertEqual(to_signed(st["offset"]), -5 * MS)
        self.assertEqual(st["delay"], 7 * MS)
        # rank 0: disp/2, ranks 1..7 empty: MAXDISP * (1/4 + ... + 1/256)
        self.assertEqual(st["disp"], (1 << 19) + sum(MAXDISP >> (r + 1) for r in range(1, 8)))
        self.assertEqual(st["jitter"], 0)
        self.assertEqual(st["reach"], 1)
        self.assertEqual(st["t4"], T0)
        self.assertEqual((st["ref_id"], st["root_delay"], st["root_disp"]), (0xC0000201, 0x123, 0x456))

    def test_eight_samples_best_delay(self):
        f = ClockFilter()
        dl = [30, 12, 25, 9, 40, 18, 22, 15]
        samples = [smp(off=(i + 1) * MS, dly=dl[i] * MS, disp=(i + 1) * 1000,
                       t4=T0 + i * 16 * SEC) for i in range(8)]
        st = feed(f, samples)
        self.assertEqual(st["reach"], 0xFF)
        # the best sample (9 ms, index 3) stays the best of the window: the later
        # updates find it not newer than tlast and keep the result of the update at 3
        self.assertEqual(to_signed(st["offset"]), 4 * MS)
        self.assertEqual(st["delay"], 9 * MS)
        self.assertEqual(st["t4"], T0 + 3 * 16 * SEC)
        self.assertEqual(st["valid"], 1)
        # the result of that update: 4 usable samples (delays 9, 12, 25, 30)
        g = ClockFilter()
        st3 = feed(g, samples[:4])
        self.assertEqual({k: v for k, v in st.items() if k != "reach"},
                         {k: v for k, v in st3.items() if k != "reach"})
        self.assertGreater(st["jitter"], 0)

    def test_sorted_by_delay_and_disp_formula(self):
        f = ClockFilter()
        # all samples with the same t4 would be stale after the first accept; use
        # increasing t4 and a final sample that is the best, so all 8 are newer than
        # tlast only if tlast is older: feed 7 first with t4 below, then verify one run
        base = T0
        dl = [50, 40, 30, 20, 10, 60, 70, 5]
        for i in range(7):
            f.peers[0].reach = ((f.peers[0].reach << 1) | 1) & 0xFF
            p = f.peers[0]
            p.head = (p.head - 1) & 7
            p.slots[p.head] = (from_signed(i * MS), dl[i] * MS, 0, base + i)
            p.tnow = base + i
        st = f.sample(smp(off=100 * MS, dly=dl[7] * MS, disp=0, t4=base + 7,
                          stratum=1))
        # no accepted result before: all 8 usable. Sorted by delay: 5,10,20,30,40,50,60,70
        order = [7, 4, 3, 2, 1, 0, 5, 6]
        offs = [100 if k == 7 else k for k in range(8)]
        self.assertEqual(to_signed(st["offset"]), 100 * MS)
        self.assertEqual(st["delay"], 5 * MS)
        # disp: aged by PHI * (tnow - t4) with tnow - t4 of a few LSB: floor -> 0
        self.assertEqual(st["disp"], 0)
        # jitter: rms of (offset_r - 100 ms) over the 7 others
        d2 = sum(((offs[k] - 100) * MS) ** 2 for k in order[1:])
        self.assertEqual(st["jitter"], math.isqrt(d2 // 7))

    def test_best_not_newer_keeps_result(self):
        f = ClockFilter()
        f.sample(smp(dly=10 * MS, disp=1000, t4=T0))
        # a worse and newer sample: the best of the window is still the first,
        # which was already used: the stored result is kept
        st = f.sample(smp(off=3 * MS, dly=30 * MS, disp=2000, t4=T0 + 1000 * SEC))
        self.assertEqual(st["delay"], 10 * MS)
        self.assertEqual(st["t4"], T0)
        self.assertEqual(st["reach"], 3)
        st2 = f.miss(0)
        self.assertEqual({k: v for k, v in st2.items() if k != "reach"},
                         {k: v for k, v in st.items() if k != "reach"})
        self.assertEqual(st2["reach"], 0b110)

    def test_ageing_formula(self):
        f2 = ClockFilter()
        # two usable samples, the old one the best: its dispersion is aged by
        # PHI * 1000 s, the other's is not (it is the newest, tnow)
        p = f2.peers[0]
        p.reach = 1
        p.head = 7
        p.slots[7] = (0, 5 * MS, 100, T0)
        p.tnow = T0
        st = f2.sample(smp(dly=50 * MS, disp=100, t4=T0 + 1000 * SEC))
        aged = 100 + ((1000 * SEC * PHI_Q32) >> 32)
        self.assertEqual(st["delay"], 5 * MS)
        self.assertEqual(st["disp"], (aged >> 1) + (100 >> 2)
                         + sum(MAXDISP >> (r + 1) for r in range(2, 8)))
        self.assertEqual(st["t4"], T0)

    def test_stale_sample_dropped(self):
        f = ClockFilter()
        f.sample(smp(off=1 * MS, dly=10 * MS, t4=T0 + 100 * SEC))
        # better delay but t4 not newer than the last used: best is not newer, kept
        st = f.sample(smp(off=9 * MS, dly=1 * MS, t4=T0 + 100 * SEC))
        self.assertEqual(to_signed(st["offset"]), 1 * MS)
        self.assertEqual(st["reach"], 3)
        st = f.sample(smp(off=9 * MS, dly=1 * MS, t4=T0 + 50 * SEC))
        self.assertEqual(to_signed(st["offset"]), 1 * MS)
        st = f.sample(smp(off=9 * MS, dly=1 * MS, t4=T0 + 101 * SEC))
        self.assertEqual(to_signed(st["offset"]), 9 * MS)

    def test_era_wrap(self):
        f = ClockFilter()
        t = (1 << 64) - 10 * SEC
        f.sample(smp(off=1, t4=t))
        st = f.sample(smp(off=2, dly=5 * MS, t4=(t + 20 * SEC) & M64))   # wrapped past 2^64
        self.assertEqual(to_signed(st["offset"]), 2)

    def test_reach_register_and_valid(self):
        f = ClockFilter()
        self.assertEqual(f.miss(1)["reach"], 0)
        self.assertEqual(f.miss(1)["valid"], 0)
        st = f.sample(smp(1, t4=T0, stratum=3))
        self.assertEqual((st["reach"], st["valid"]), (1, 1))
        for k in range(1, 8):
            st = f.miss(1)
            self.assertEqual(st["reach"], (1 << k) & 0xFF)
            self.assertEqual(st["valid"], 1)
        st = f.miss(1)                      # the 8th miss: reach == 0
        self.assertEqual((st["reach"], st["valid"]), (0, 0))
        # other peers unaffected
        self.assertEqual(f.stat(0)["reach"], 0)

    def test_valid_rule(self):
        for stratum, leap, ok in [(0, 0, 0), (1, 0, 1), (15, 0, 1), (16, 0, 0), (255, 0, 0),
                                  (2, 3, 0), (2, 1, 1), (2, 2, 1)]:
            f = ClockFilter()
            st = f.sample(smp(stratum=stratum, leap=leap))
            self.assertEqual(st["valid"], ok, (stratum, leap))
            self.assertEqual((st["stratum"], st["leap"]), (stratum, leap))

    def test_unusable_first_sample(self):
        f = ClockFilter()
        st = f.sample(smp(disp=MAXDISP, t4=T0))       # disp >= MAXDISP: unusable
        self.assertEqual((st["valid"], st["offset"], st["delay"], st["disp"], st["t4"]), (0, 0, 0, 0, 0))
        self.assertEqual(st["reach"], 1)

    def test_delay_ties_newest_first(self):
        f = ClockFilter()
        f.sample(smp(off=1 * MS, dly=10 * MS, t4=T0, disp=0))
        st = f.sample(smp(off=2 * MS, dly=10 * MS, t4=T0 + SEC, disp=0))
        self.assertEqual(to_signed(st["offset"]), 2 * MS)      # the newer of the tie

    def test_extreme_offsets(self):
        f = ClockFilter()
        hi, lo = (1 << 63) - 1, 1 << 63        # +2^31 s and -2^31 s
        f.sample(Sample(0, hi, 5 * MS, 0, T0))
        st = f.sample(Sample(0, lo, 3 * MS, 0, T0 + SEC))
        self.assertEqual(st["offset"], lo)
        self.assertEqual(st["jitter"], MAXDISP)         # saturated, not wrapped to ~0

    def test_maxdisp_expiry(self):
        f = ClockFilter()
        f.sample(smp(dly=5 * MS, disp=0, t4=T0))
        # 2^21 s later the first sample's dispersion is PHI * 2^21 s = 31 s > MAXDISP:
        # it leaves the list although its delay is better
        st = f.sample(smp(dly=50 * MS, disp=0, t4=T0 + (1 << 21) * SEC))
        self.assertEqual(st["delay"], 50 * MS)

    def test_peers_independent(self):
        f = ClockFilter()
        a = f.sample(smp(0, off=1 * MS, t4=T0))
        f.sample(smp(1, off=7 * MS, t4=T0))
        self.assertEqual(f.stat(0), a)

    def test_isqrt_and_div(self):
        rng = random.Random(1)
        cases = [0, 1, 2, 3, 4, 15, 16, (1 << 75) - 1, 7 << 72, (1 << 76) - 1]
        cases += [rng.getrandbits(rng.randint(1, 76)) for _ in range(3000)]
        for n in cases:
            self.assertEqual(F.isqrt_digit(n), math.isqrt(n), n)
        for _ in range(3000):
            n = rng.getrandbits(rng.randint(1, 75))
            d = rng.randint(1, 7)
            self.assertEqual(F.restoring_div(n, d), n // d)

    def test_fixed_vs_float(self):
        """Random sequences: offset, delay, valid, reach exact; dispersion within
        16 LSB (two floors per rank), jitter within 2 LSB."""
        rng = random.Random(42)
        for trial in range(20):
            fx, fl = ClockFilter(), FloatFilter()
            t4 = [T0] * 4
            for i in range(300):
                p = rng.randrange(4)
                if rng.random() < 0.12:
                    st, ref = fx.miss(p), fl.miss(p)
                else:
                    t4[p] += rng.randint(8, 200) * SEC + rng.randrange(SEC)
                    tt = t4[p] - (rng.choice([0, SEC, 50 * SEC]) if rng.random() < 0.05 else 0)
                    s = smp(p, off=rng.randint(-SEC, SEC), dly=rng.randint(1, 80) * MS + rng.randrange(MS),
                            disp=rng.randrange(1, 1 << 24), t4=tt, stratum=rng.randint(0, 17),
                            leap=rng.randint(0, 3))
                    st, ref = fx.sample(s), fl.sample(s)
                valid, off, dly, disp, jit = ref
                self.assertEqual(bool(st["valid"]), valid)
                self.assertEqual(Fraction(to_signed(st["offset"]), SEC), off)
                self.assertEqual(Fraction(st["delay"], SEC), dly)
                self.assertLessEqual(abs(st["disp"] - disp * SEC), 16)
                self.assertLessEqual(abs(st["jitter"] - jit * SEC), 2)


class VectorFileTests(unittest.TestCase):
    def test_vector_file_is_current(self):
        here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        with open(os.path.join(here, "vectors", "filter.txt")) as fh:
            text = fh.read()
        want = F.HEADER + "".join(l + "\n" for l in F.vector_lines(F.gen_events()))
        self.assertEqual(text, want, "regenerate: cd tools/golden && python3 -m ntpclient.filter")

    def test_vectors_exercise_cases(self):
        evs = F.gen_events()
        stats = F.replay(evs)
        self.assertGreater(sum(1 for k, _ in evs if k == "miss"), 50)
        self.assertGreater(sum(s["valid"] for s in stats), 300)
        self.assertGreater(sum(1 for s in stats if s["jitter"] > 0), 100)
        self.assertTrue(any(s["jitter"] == MAXDISP for s in stats))
        self.assertTrue(any(s["valid"] == 0 and s["reach"] != 0 for s in stats))


if __name__ == "__main__":
    unittest.main()
