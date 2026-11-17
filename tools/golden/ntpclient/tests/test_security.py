"""Security review (T5.5): what forged upstream samples do to the clock.

The chain clock filter -> selection -> servo of the bit-exact models (the
RTL units replay these models' vectors), driven as `ArtyCoreClient` drives
the units: one PeerStat per sample, one Selected per PeerStat, the servo
fed with every Selected. See docs/security.md (findings F4 and F7).

Run: python3 -m unittest discover tools/golden
"""
import os
import sys
import unittest

_golden = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _golden not in sys.path:
    sys.path.insert(0, _golden)

from ntpclient import filter as F  # noqa: E402
from ntpclient import selection as SEL  # noqa: E402
from ntpclient import servo as SV  # noqa: E402

S = 1 << 32
MS = S // 1000
T0 = 3_900_000_000 * S
POLL = 6
YEAR = 365 * 86400


class Chain:
    """Filter, selection and servo, as wired in the stratum-2 top."""

    def __init__(self):
        self.flt = F.ClockFilter()
        self.sel = SEL.Selection()
        self.servo = SV.Servo()
        self.log = []

    def sample(self, peer, offset_s, t4, delay=10 * MS):
        st = self.flt.sample(F.Sample(peer, int(offset_s * S), delay, 1 * MS, t4, stratum=1))
        stat = SEL.PeerStat(**st)
        s = self.sel.process(stat, t4)
        out = self.servo.update(bool(s.valid), s.offset, s.jitter, t4, POLL)
        self.log.append((peer, s, out))
        return s, out


class StartupStep(unittest.TestCase):
    def test_single_source_decides_the_startup_step(self):
        """F4: after reset the first source to reach 4 samples (slot 0's
        iburst, 6 s after start) is the only eligible peer, the selection
        accepts a single peer, and the servo in NSET steps by any amount
        (no PANICT check). Whoever controls slot 0's four iburst replies
        (on-path, or off-path with the guessable Origin of F3) sets our
        clock one year ahead; the honest slot 1 comes 0.25 s too late."""
        c = Chain()
        for k in range(4):
            s, out = c.sample(0, YEAR, T0 + k * 2 * S)
            if k < 3:
                self.assertEqual(s.valid, 0, "3 samples: root distance too large")
                self.assertEqual(out["kind"], "ignore")
        self.assertEqual(s.valid, 1)
        self.assertEqual(SEL.s64(s.offset), YEAR * S, "selected on one peer")
        self.assertEqual(out["kind"], "step")
        self.assertTrue(out["step_en"])
        self.assertEqual(out["step"], (YEAR * S) << 32, "step word: +1 year")
        # The honest peer then disagrees by a year: no majority, no update,
        # and the servo never steps back (one step per reset).
        for k in range(4):
            s, out = c.sample(1, -YEAR, T0 + k * 2 * S + S // 4)
        self.assertEqual(s.valid, 0, "two peers a year apart: no majority")
        self.assertEqual(c.servo.steps, 1)


class RuntimeLimits(unittest.TestCase):
    def synced_chain(self, peers):
        c = Chain()
        t = T0
        for k in range(8):
            for p in peers:
                c.sample(p, 0.0, t + p * S // 4)
            t += 2 * S if k < 4 else (1 << POLL) * S
        self.assertEqual(c.servo.st, SV.SYNC)
        return c, t

    def test_one_forged_peer_of_two_gives_holdover(self):
        """Two upstreams (the board's configuration), one forged at +10 s
        from now on: the forged peer is first ineligible (its filter jitter
        makes the root distance exceed MAXDIST), then, once its 8 stages
        agree, the two intervals do not intersect: no majority, `valid` = 0,
        the servo holds. The forged offset never reaches the servo."""
        c, t = self.synced_chain([0, 1])
        kinds = set()
        for k in range(16):
            for p in (0, 1):
                s, out = c.sample(p, 10.0 if p == 0 else 0.0, t + p * S // 4)
                if s.valid:
                    self.assertLess(abs(SEL.s64(s.offset)), 2 * MS, "honest offset only")
                    kinds.add("honest")
                else:
                    kinds.add("holdover")
            t += (1 << POLL) * S
        self.assertEqual(kinds, {"honest", "holdover"})
        self.assertEqual(c.servo.steps, 0)
        self.assertLessEqual(abs(SV.ppm(c.servo.outputs(t)[0])), 1.0)

    def test_one_falseticker_of_four_is_ignored(self):
        """Four upstreams, one forged at +10 s: a falseticker, the system
        offset stays that of the three truechimers."""
        c, t = self.synced_chain([0, 1, 2, 3])
        for k in range(4):
            for p in range(4):
                s, out = c.sample(p, 10.0 if p == 0 else 0.001, t + p * S // 4)
                if s.valid:
                    self.assertLess(abs(SEL.s64(s.offset)), 2 * MS)
            t += (1 << POLL) * S
        self.assertEqual(c.servo.steps, 0)

    def test_all_upstreams_forged_slew_at_most_500_ppm(self):
        """F7, on-path attacker controlling every upstream after start-up:
        an offset above STEPT (128 ms) is ignored (SPIK) for STEPOUT (900 s),
        then slewed, never stepped, at most 500 ppm (43 s per day); above
        PANICT (1000 s) it is never followed (PANIC, unsynchronised)."""
        c, t = self.synced_chain([0, 1])
        x = 0.0  # what the attacker has moved our clock by, seconds
        trims = []
        for k in range(1350):  # 1350 x 64 s = 24 h
            off = 10.0 - x
            s, out = c.sample(0, off, t)
            s, out = c.sample(1, off, t + S // 4)
            trim = c.servo.outputs(t + S)[0]
            trims.append(trim)
            x += SV.ppm(trim) * 1e-6 * (1 << POLL)
            t += (1 << POLL) * S
        self.assertEqual(c.servo.steps, 0, "never stepped after start-up")
        self.assertLessEqual(max(abs(SV.ppm(v)) for v in trims), 500.0 + 1e-6)
        self.assertGreater(x, 9.0, "but a persistent 10 s lie is followed within a day")
        self.assertLessEqual(x, 43.3)
        # Above PANICT: never followed.
        c, t = self.synced_chain([0, 1])
        for k in range(30):
            s, out = c.sample(0, 2000.0, t)
            s, out = c.sample(1, 2000.0, t + S // 4)
            t += (1 << POLL) * S
        self.assertEqual(c.servo.st, SV.PANIC)
        self.assertEqual(c.servo.steps, 0)
        self.assertFalse(c.servo.outputs(t)[1], "PANIC: not synchronised")


if __name__ == "__main__":
    unittest.main()
