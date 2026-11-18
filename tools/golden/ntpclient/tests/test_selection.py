import os
import random
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import selection as sel  # noqa: E402
from selection import MS, S, T0, PeerStat, Selection, st  # noqa: E402

BASE = (sel.MINDISP_Q32 >> 1) + sel.RHO_Q32


def run(evs):
    m = Selection()
    out = None
    for now, s in evs:
        out = m.process(s, now)
    return m, out


class RootDistTest(unittest.TestCase):
    def test_minimum(self):
        p = st(0, 0.0, delay=0, disp=0, jitter=0)
        self.assertEqual(sel.root_dist(p, p.t4), BASE)

    def test_terms(self):
        p = st(0, 0.0, delay=40 * MS, disp=3, jitter=5, rdelay=0x8000, rdisp=0x100)
        half = ((0x8000 << 16) + 40 * MS) >> 1
        self.assertEqual(sel.root_dist(p, p.t4), half + (0x100 << 16) + 3 + 5 + sel.RHO_Q32)

    def test_ageing(self):
        p = st(0, 0.0, delay=0, disp=0, jitter=0)
        age = 1000 * S + 12345
        phi = ((age >> 16) * sel.PHI_Q32) >> 16
        self.assertEqual(sel.root_dist(p, p.t4 + age), BASE + phi)
        # 1000 s at 15 ppm = 15 ms, within one 2^-32 s unit per floor.
        self.assertAlmostEqual(phi / S, 1000.0 * sel.PHI_Q32 / S, delta=2e-9)
        # A clock before t4 does not age; ageing saturates.
        self.assertEqual(sel.root_dist(p, p.t4 - 1), BASE)
        big = sel.root_dist(st(0, t4=0, delay=0, disp=0, jitter=0), sel.M64)
        self.assertEqual(big, BASE + (((sel.AGE_MAX >> 16) * sel.PHI_Q32) >> 16))

    def test_saturation(self):
        p = st(0, delay=sel.M64, disp=sel.M64, jitter=sel.M64)
        self.assertEqual(sel.root_dist(p, p.t4), sel.M64)


class IntersectTest(unittest.TestCase):
    def test_single(self):
        self.assertEqual(sel.intersect([(0, 10, 3)]), (7, 13))

    def test_majority(self):
        # Two agreeing, one far: the intersection of the two.
        self.assertEqual(sel.intersect([(0, 10, 3), (1, 11, 3), (2, 100, 3)]), (8, 13))

    def test_no_majority(self):
        self.assertIsNone(sel.intersect([(0, 10, 3), (1, 100, 3)]))
        self.assertIsNone(sel.intersect([(0, 10, 3), (1, 11, 3), (2, 100, 3), (3, 101, 3)]))

    def test_midpoints_required(self):
        # Overlapping intervals whose midpoints are outside the overlap: no majority.
        self.assertIsNone(sel.intersect([(0, 0, 10), (1, 15, 10)]))
        self.assertEqual(sel.intersect([(0, 0, 10), (1, 5, 10)]), (-5, 10))

    def test_touching_is_empty(self):
        self.assertIsNone(sel.intersect([(0, 0, 5), (1, 10, 5)]))


class BehaviourTest(unittest.TestCase):
    def test_single_exact(self):
        p = st(1, off=-123456789)
        m, out = run([(T0, p)])
        self.assertEqual((out.valid, out.peer, out.offset), (1, 1, p.offset))
        self.assertEqual(out.root_dist, sel.root_dist(p, T0))
        self.assertEqual(out.jitter, p.jitter)  # isqrt(jitter^2)

    def test_falseticker_does_not_influence(self):
        outs = []
        for far in (0.5, 0.9, -3.0, 1e6):
            _, out = run([(T0, st(0, 0.010)), (T0, st(1, 0.011)), (T0, st(2, far))])
            outs.append(out)
        self.assertTrue(all(o == outs[0] for o in outs))
        self.assertEqual(outs[0].valid, 1)
        _, two = run([(T0, st(0, 0.010)), (T0, st(1, 0.011))])
        self.assertEqual(two, outs[0])

    def test_ineligible(self):
        for kw in ({"leap": 3}, {"stratum": 16}, {"stratum": 0}, {"reach": 0}, {"valid": 0},
                   {"rdisp": 0x18000}):
            _, out = run([(T0, st(0, 0.01, **kw))])
            self.assertEqual(out, sel.Selected(), kw)

    def test_maxdist_boundary(self):
        _, out = run([(T0, st(0, delay=0, jitter=0, disp=sel.MAXDIST_Q32 - 1 - BASE))])
        self.assertEqual((out.valid, out.root_dist), (1, sel.MAXDIST_Q32 - 1))
        _, out = run([(T0, st(0, delay=0, jitter=0, disp=sel.MAXDIST_Q32 - BASE))])
        self.assertEqual(out.valid, 0)

    def test_cluster(self):
        m, out = run([(T0, st(0, 0.010, jitter=10, disp=100 * MS)), (T0, st(1, 0.011, jitter=10, disp=100 * MS)),
                      (T0, st(2, 0.012, jitter=10, disp=100 * MS)), (T0, st(3, 0.040, jitter=10, disp=100 * MS))])
        self.assertEqual(m.info["truechimers"], [0, 1, 2, 3])
        self.assertEqual(m.info["survivors"], [0, 1, 2])
        m, out = run([(T0, st(0, 0.010, jitter=50 * MS)), (T0, st(1, 0.011, jitter=50 * MS)),
                      (T0, st(2, 0.012, jitter=50 * MS)), (T0, st(3, 0.040, jitter=50 * MS, disp=100 * MS))])
        self.assertEqual(m.info["survivors"], [0, 1, 2, 3])

    def test_hysteresis(self):
        m = Selection()
        self.assertEqual(m.process(st(0, 0.010, disp=20 * MS), T0).peer, 0)
        self.assertEqual(m.process(st(1, 0.011, disp=10 * MS), T0).peer, 0)  # kept
        self.assertEqual(m.process(st(1, 0.011, disp=1 * MS), T0).peer, 1)  # rd_0 > 3 rd_1
        self.assertEqual(m.process(st(0, 0.010, disp=1 * MS), T0).peer, 1)  # kept (tie)
        self.assertEqual(m.process(st(0, 0.010, disp=1 * MS, stratum=2), T0).peer, 1)
        self.assertEqual(m.process(st(1, 0.011, disp=1 * MS, stratum=3), T0).peer, 0)  # stratum

    def test_two_disagree_resets_system_peer(self):
        m = Selection()
        m.process(st(0, 0.010), T0)
        self.assertEqual(m.process(st(1, 0.5), T0).valid, 0)
        self.assertIsNone(m.sys)


class VectorsTest(unittest.TestCase):
    def test_file_up_to_date(self):
        with open(sel.VECTORS) as f:
            self.assertEqual(f.read(), sel.render(), "run: python3 tools/golden/ntpclient/selection.py")

    def test_parse_roundtrip(self):
        with open(sel.VECTORS) as f:
            sc = sel.parse_vectors(f.read())
        self.assertEqual(len(sc), len(sel.all_scenarios()))
        names = [n for n, _, _ in sc]
        for want in ("single", "two_agree", "two_disagree", "three_one_false", "four_two_false",
                     "leap3", "stratum16", "unreachable", "maxdist_exact", "hysteresis_keep", "tie_identical"):
            self.assertIn(want, names)
        for name, n_en, evs in sc:
            m = Selection(n_en)
            for now, s, want in evs:
                self.assertEqual(m.process(s, now), want, name)

    def test_coverage(self):
        """The vectors exercise every branch of the algorithm."""
        seen = set()
        for _, n_en, evs in sel.all_scenarios():
            m = Selection(n_en)
            for now, s in evs:
                out = m.process(s, now)
                i = m.info
                if i.get("startup_gate"):
                    seen.add("startup_gate_%d" % n_en)
                if not out.valid:
                    seen.add("none_no_candidate" if not any(i["el"]) else "none_no_majority")
                    continue
                seen.add("valid_%d" % len(i["survivors"]))
                if len(i["truechimers"]) > len(i["survivors"]):
                    seen.add("cluster_drop")
                if len(i["truechimers"]) == 4 and len(i["survivors"]) == 4:
                    seen.add("cluster_keep")
                if sum(i["el"]) > len(i["truechimers"]):
                    seen.add("falseticker")
                if out.peer != i["best"]:
                    seen.add("hysteresis_kept")
        for want in ("none_no_candidate", "none_no_majority", "valid_1", "valid_2", "valid_3", "valid_4",
                     "cluster_drop", "cluster_keep", "falseticker", "hysteresis_kept",
                     "startup_gate_2", "startup_gate_3", "startup_gate_4"):
            self.assertIn(want, seen)


class StartupMajorityTest(unittest.TestCase):
    """F4: until the first valid selection, a majority of the enabled slots
    (n_en // 2 + 1 truechimers) is required; afterwards the usual rules."""

    def test_thresholds(self):
        a, b, c, d = (st(i, 0.010 + i * 0.001) for i in range(4))
        for n_en, need in [(0, 1), (1, 1), (2, 2), (3, 2), (4, 3), (7, 4)]:
            for k in range(1, 5):
                m = Selection(n_en)
                out = None
                for p in (a, b, c, d)[:k]:
                    out = m.process(p, T0)
                self.assertEqual(out.valid, int(k >= need), (n_en, k))

    def test_gate_only_before_the_first_selection(self):
        m = Selection(2)
        self.assertEqual(m.process(st(0, 0.010), T0).valid, 0)
        self.assertFalse(m.ever)
        self.assertEqual(m.process(st(1, 0.011), T0).valid, 1)
        self.assertTrue(m.ever)
        # peer 1 lost: peer 0 alone is selected again (no gate any more)
        out = m.process(st(1, 0.011, valid=0), T0)
        self.assertEqual((out.valid, out.peer), (1, 0))

    def test_gated_event_clears_the_system_peer(self):
        m = Selection(4)
        m.process(st(0, 0.010), T0)
        self.assertEqual(m.process(st(1, 0.011), T0).valid, 0)
        self.assertIsNone(m.sys)
        self.assertTrue(m.info["startup_gate"])

    def test_one_liar_of_two_cannot_start(self):
        m = Selection(2)
        year = 365 * 86400 * S
        for k in range(4):
            self.assertEqual(m.process(st(0, off=year, t4=T0 + 2 * k * S), T0 + 2 * k * S).valid, 0)
        self.assertEqual(m.process(st(1, 0.001, t4=T0 + 8 * S), T0 + 8 * S).valid, 0)
        self.assertFalse(m.ever)


class FloatReferenceTest(unittest.TestCase):
    """Random scenarios against a floating-point RFC 5905 reference.

    Tolerances: decisions (valid, system peer, survivors) equal; root distance
    within 1e-9 s; offset and jitter within 1e-9 s + 1e-4 * spread (the
    weights 2^48 // rd have a relative error below 2^-15) + 2^-50 |offset|
    (double precision of offsets up to 2^31 s)."""

    def check(self, evs):
        m = Selection()
        prev = None
        for now, s in evs:
            out = m.process(s, now)
            ref, prev = sel.float_select(m.peers, now, prev)
            if ref is None:
                self.assertEqual(out.valid, 0)
                continue
            self.assertEqual(out.valid, 1)
            self.assertEqual(out.peer, ref["peer"])
            self.assertEqual(m.info["survivors"], ref["survivors"])
            Q = float(1 << 32)
            off = sel.s64(out.offset) / Q
            tol = 1e-9 + 1e-4 * ref["spread"] + 2.0**-50 * abs(ref["offset"])
            self.assertLessEqual(abs(off - ref["offset"]), tol)
            self.assertLessEqual(abs(out.jitter / Q - ref["jitter"]), 1e-9 + 1e-4 * (ref["spread"] + ref["jitter"]))
            self.assertLessEqual(abs(out.root_dist / Q - ref["root_dist"]), 1e-9)

    def test_vectors_random(self):
        for name, evs in sel.random_scenarios(random.Random(sel.SEED), sel.N_RANDOM):
            with self.subTest(name=name):
                self.check(evs)

    def test_more_random(self):
        for name, evs in sel.random_scenarios(random.Random(12345), 500):
            with self.subTest(name=name):
                self.check(evs)

    def test_random_agreement(self):
        """Tight clusters of 4 with varied jitters: clustering and combining."""
        rng = random.Random(777)
        for k in range(300):
            evs = []
            truth = rng.uniform(-1, 1)
            for p in range(4):
                o = truth + rng.gauss(0, 0.005)
                evs.append((T0, PeerStat(p, 1, sel.u64(int(o * S)), rng.randrange(0, 50 * MS),
                                         rng.randrange(0, 50 * MS), rng.randrange(0, 10 * MS), 0xFF,
                                         rng.choice([1, 2]), 0, p, 0, 0, T0)))
            with self.subTest(k=k):
                self.check(evs)


if __name__ == "__main__":
    unittest.main()
