"""Tests of the servo model (T4.5): fixed-point rules and closed loop.

Run: python3 -m unittest discover tools/golden/ntpclient
"""
import os
import sys
import unittest

# tools/golden on the path: `ntpclient` is a namespace package there.
_golden = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _golden not in sys.path:
    sys.path.insert(0, _golden)

from ntpclient import servo as S  # noqa: E402

Q32 = S.Q32
T0 = 3_900_000_000 * Q32


def q(x):
    return S.q32(x)


def curve(title, log, every=1, limit=60):
    """Prints a settling curve in the test log."""
    print("\n--- %s" % title)
    print("   t(s)    x(ms)  theta(ms)   trim(ppm)   freq(ppm) state kind")
    for e in log[: limit * every : every]:
        print(
            "%7.0f %8.3f %9.3f %11.3f %11.3f %5s %s%s"
            % (e["t"], e["x"] * 1e3, e["theta"] * 1e3, S.ppm(e["trim"]), S.ppm(e["freq"]),
               S.STATE_NAMES[e["state"]], e["kind"], " STEP" if e["step_en"] else "")
        )


def tail_stats(log, t_from):
    xs = [e["x"] for e in log if e["t"] >= t_from]
    return S.rms(xs), max(abs(v) for v in xs)


class FixedPoint(unittest.TestCase):
    def test_constants(self):
        self.assertEqual(S.MAXTRIM, 92_233_720)
        self.assertEqual(S.STEPT_Q, 549_755_814)
        self.assertEqual(S.CQ, 720_575_940)
        # r = a * CQ >> 24 is the trim removing a in one second: 1 ms ->
        # 1000 ppm for 1 s.
        r = (q(1e-3) * S.CQ) >> S.CS
        self.assertAlmostEqual(S.ppm(r), 1000.0, places=3)
        self.assertEqual(S.processing_cycles(4), 53)
        self.assertEqual(S.processing_cycles(10), 59)

    def test_first_update_slews_half(self):
        sv = S.Servo()
        o = sv.update(True, q(0.010), q(1e-4), T0, 6)
        self.assertEqual((o["state"], o["step_en"], o["kind"]), (S.FREQ, False, "process"))
        # Half of 10 ms over 64 s: 78.125 ppm; no integral on the first one.
        self.assertAlmostEqual(S.ppm(o["trim"]), 78.125, places=2)
        self.assertEqual(sv.freq, 0)
        # Negative offsets give the opposite trim.
        sv = S.Servo()
        o2 = sv.update(True, q(-0.010), q(1e-4), T0, 6)
        self.assertEqual(o2["trim"], -o["trim"])

    def test_slew_expires_after_one_poll(self):
        sv = S.Servo()
        sv.update(True, q(0.010), 0, T0, 6)
        self.assertNotEqual(sv.outputs(T0 + 63 * Q32)[0], 0)
        self.assertEqual(sv.outputs(T0 + 64 * Q32)[0], 0)
        self.assertTrue(sv.outputs(T0 + 511 * Q32)[1])
        self.assertFalse(sv.outputs(T0 + 512 * Q32)[1])

    def test_poll_clamp(self):
        for p, pn in [(0, 4), (4, 4), (10, 10), (17, 10), (255, 10)]:
            a, b = S.Servo(), S.Servo()
            self.assertEqual(a.update(True, q(0.01), 0, T0, p)["trim"],
                             b.update(True, q(0.01), 0, T0, pn)["trim"])

    def test_step_only_from_nset(self):
        sv = S.Servo()
        o = sv.update(True, q(-0.5), 0, T0, 6)
        self.assertTrue(o["step_en"])
        self.assertEqual(o["step"], (S.s64(q(-0.5)) << 32) & S.M96)
        self.assertEqual(o["state"], S.FREQ)
        now = T0 - q(0.5) + 64 * Q32
        for off in [0.5, -0.3, 2.0, 999.0, 5000.0, -0.2]:
            o = sv.update(True, q(off), 0, now, 6)
            self.assertFalse(o["step_en"], off)
            now += 64 * Q32
        self.assertEqual(sv.steps, 1)

    def test_step_from_1900(self):
        # Clock at 5 s after reset (1900), true time in 2024: the offset word
        # wraps (beyond +-2^31 s) but the 96-bit step, modulo 2^32 s like the
        # accumulator, lands on the true time.
        true = 3_913_056_000 * Q32 + 0x1234_5678
        now = 5 * Q32
        sv = S.Servo()
        o = sv.update(True, (true - now) & S.M64, 0, now, 6)
        self.assertTrue(o["step_en"])
        acc = ((now << 32) + o["step"]) & S.M96
        self.assertEqual(acc >> 32, true)

    def test_stept_boundary(self):
        for off, step in [(S.STEPT_Q, False), (-S.STEPT_Q, False),
                          (S.STEPT_Q + 1, True), (-S.STEPT_Q - 1, True)]:
            o = S.Servo().update(True, off & S.M64, 0, T0, 6)
            self.assertEqual(o["step_en"], step)

    def test_valid_zero_is_ignored(self):
        sv = S.Servo()
        for k in range(5):
            o = sv.update(False, q(3.0), 0, T0 + k * Q32, 6)
            self.assertEqual((o["state"], o["step_en"], o["trim"]), (S.NSET, False, 0))

    def test_spike_gate_and_states(self):
        sv = S.Servo()
        now = T0
        for off in [0.0004, 0.0002, 0.0001, 0.00005, 0.00002]:
            sv.update(True, q(off), q(1e-4), now, 6)
            now += 64 * Q32
        self.assertEqual(sv.st, S.SYNC)
        f = sv.freq
        o = sv.update(True, q(0.002), q(1e-4), now, 6)
        self.assertEqual((o["state"], o["kind"], sv.freq), (S.SPIK, "ignore", f))
        # 3 * jitter above 2 ms: not a spike.
        sv2 = S.Servo()
        sv2.__dict__.update({k: v for k, v in sv.__dict__.items()})
        sv2.st = S.SYNC
        self.assertEqual(sv2.update(True, q(0.002), q(0.001), now, 6)["kind"], "process")
        # The next normal update leaves SPIK.
        now += 64 * Q32
        self.assertEqual(sv.update(True, q(0.0001), q(1e-4), now, 6)["state"], S.SYNC)
        # Large offset: SPIK, ignored until STEPOUT, then slewed (no step).
        now += 64 * Q32
        o = sv.update(True, q(0.3), q(1e-4), now, 6)
        self.assertEqual((o["state"], o["kind"]), (S.SPIK, "ignore"))
        # STEPOUT counts from the last accepted update (64 s before `now`).
        o = sv.update(True, q(0.3), q(1e-4), now + 835 * Q32, 6)
        self.assertEqual(o["kind"], "ignore")
        now += 837 * Q32 - 901 * Q32
        o = sv.update(True, q(0.3), q(1e-4), now + 901 * Q32, 6)
        self.assertEqual((o["state"], o["kind"], o["step_en"]), (S.SPIK, "process", False))
        self.assertEqual(o["trim"], S.MAXTRIM)
        o = sv.update(True, q(0.29), q(1e-4), now + 965 * Q32, 6)
        self.assertEqual(o["kind"], "process")  # slw: keeps slewing
        # Panic: never followed.
        o = sv.update(True, q(-1001.0), q(1e-4), now + 1029 * Q32, 6)
        self.assertEqual((o["state"], o["kind"], o["synced"]), (S.PANIC, "ignore", False))
        o = sv.update(True, q(0.0001), q(1e-4), now + 1093 * Q32, 6)
        self.assertEqual(o["state"], S.SYNC)
        self.assertEqual(sv.steps, 0)

    def test_anti_windup(self):
        # The output saturates through the slew: the integrator holds.
        sv = S.Servo()
        now = T0
        for k in range(30):
            o = sv.update(True, q(0.1), 0, now, 4)
            self.assertLessEqual(abs(o["trim"]), S.MAXTRIM)
            self.assertLessEqual(abs(sv.freq), S.MAXTRIM)
            now += 16 * Q32
        self.assertEqual(o["trim"], S.MAXTRIM)
        self.assertEqual(sv.freq, 0, "no integration while the output saturates")
        # A small offset (no saturation) integrates; |freq| stays clamped.
        sv = S.Servo()
        now = T0
        for k in range(200):
            o = sv.update(True, q(0.004), 0, now, 4)
            self.assertLessEqual(abs(sv.freq), S.MAXTRIM)
            now += 16 * Q32
        # The integrator stops where freq + slew reaches the clamp.
        self.assertGreater(o["trim"], 0.99 * S.MAXTRIM)
        self.assertLessEqual(o["trim"], S.MAXTRIM)
        self.assertGreater(sv.freq, 0)

    def test_fset_never(self):
        seen = set()
        for _, steps in S._vector_events():
            sv = S.Servo()
            for stp in steps:
                if stp[0] == "ev":
                    seen.add(sv.update(bool(stp[1]), stp[2], stp[3], stp[4], stp[5])["state"])
        self.assertNotIn(S.FSET, seen)
        self.assertEqual(seen, {S.SPIK, S.FREQ, S.SYNC, S.PANIC})

    def test_vectors_up_to_date(self):
        with open(S.VECTOR_PATH) as f:
            self.assertEqual(f.read(), "\n".join(S.make_vectors()) + "\n",
                             "run python3 tools/golden/ntpclient/servo.py")


class ClosedLoop(unittest.TestCase):
    SEEDS = range(1, 9)

    def test_50ms_offset_removed_without_step(self):
        worst_settle = 0
        for seed in self.SEEDS:
            sv, log = S.simulate(x0=-0.050, seed=seed, duration=4 * 3600)
            if seed == 1:
                curve("50 ms initial offset, 0 ppm, poll 64 s, noise 100 us", log, limit=30)
            self.assertEqual(sv.steps, 0)
            self.assertFalse(any(e["step_en"] for e in log))
            st = S.settling_time(log)
            self.assertIsNotNone(st)
            worst_settle = max(worst_settle, st)
            r, m = tail_stats(log, st)
            self.assertLess(r, 1e-3)
        print("50 ms: worst settling time (|x| < 1 ms for good) %.0f s" % worst_settle)
        self.assertLessEqual(worst_settle, 1200)

    def test_500ms_offset_stepped_once(self):
        for seed in self.SEEDS:
            sv, log = S.simulate(x0=-0.5, e_ppm=10, seed=seed, duration=3 * 3600)
            if seed == 1:
                curve("500 ms initial offset, 10 ppm", log, limit=20)
            self.assertEqual(sv.steps, 1)
            self.assertTrue(log[0]["step_en"])
            self.assertFalse(any(e["step_en"] for e in log[1:]))
            self.assertLess(abs(log[1]["x"]), 2e-3)
            st = S.settling_time(log)
            self.assertLessEqual(st, 1200)

    def test_50ppm_learnt_residual_below_1ms(self):
        worst_settle, worst_rms, worst_f = 0, 0, 0
        for seed in self.SEEDS:
            sv, log = S.simulate(e_ppm=50, seed=seed, duration=6 * 3600)
            if seed == 1:
                curve("50 ppm frequency error, poll 64 s, noise 100 us", log, limit=40)
            self.assertEqual(sv.steps, 0)
            st = S.settling_time(log)
            r, m = tail_stats(log, st)
            worst_settle = max(worst_settle, st)
            worst_rms = max(worst_rms, r)
            worst_f = max(worst_f, abs(S.ppm(sv.freq) + 50))
        print("50 ppm: worst settling %.0f s, worst residual RMS %.1f us, "
              "worst final frequency error %.3f ppm" % (worst_settle, worst_rms * 1e6, worst_f))
        self.assertLessEqual(worst_settle, 900)
        self.assertLess(worst_rms, 1e-3)
        self.assertLess(worst_rms, 200e-6)
        self.assertLess(worst_f, 1.0)

    def test_other_polls_and_frequencies(self):
        for poll, e, noise in [(4, 50, 100e-6), (10, 50, 100e-6), (6, -200, 100e-6),
                               (6, 400, 100e-6), (8, 20, 1e-3)]:
            sv, log = S.simulate(e_ppm=e, poll=poll, noise=noise, seed=3, duration=20 * 3600)
            self.assertEqual(sv.steps, 0)
            late = [e_["x"] for e_ in log if e_["t"] >= 10 * 3600]
            r = S.rms(late)
            print("poll %d, %+d ppm, noise %.0f us: RMS after 10 h %.1f us, freq %.3f ppm"
                  % (1 << poll, e, noise * 1e6, r * 1e6, S.ppm(sv.freq)))
            self.assertLess(r, max(1e-3, 2 * noise))
            # The integrator dithers with the measurement noise: about
            # noise / T per update (times the 1/8 gain), so allow 1 ppm plus
            # noise / T.
            self.assertLess(abs(S.ppm(sv.freq) + e), 1.0 + 1e6 * noise / (1 << poll))

    def test_spike_rejected(self):
        for sign in (1, -1):
            k_sp = 60
            sv, log = S.simulate(e_ppm=50, seed=2, duration=2 * 3600,
                                 spikes={k_sp: sign * 0.002})
            ref_sv, ref = S.simulate(e_ppm=50, seed=2, duration=2 * 3600)
            curve("%+d ms spike at event %d" % (2 * sign, k_sp), log[k_sp - 3:k_sp + 4], limit=7)
            ev = log[k_sp]
            self.assertEqual((ev["state"], ev["kind"]), (S.SPIK, "ignore"))
            # Integrator untouched; the trim is the previous slew or, once it
            # expired, the integrator alone.
            self.assertEqual(ev["freq"], log[k_sp - 1]["freq"])
            self.assertIn(ev["trim"], (log[k_sp - 1]["trim"], log[k_sp - 1]["freq"]))
            self.assertEqual(log[k_sp + 1]["state"], S.SYNC)
            # The spike leaves no trace beyond one missed update.
            after = [abs(e["x"]) for e in log[k_sp:]]
            self.assertLess(max(after), 1e-3)
            self.assertEqual(sv.steps, 0)

    def test_holdover(self):
        k0, k1 = 60, 60 + 160  # 160 polls = 10240 s without a valid update
        sv, log = S.simulate(e_ppm=50, seed=4, duration=6 * 3600, outage=(k0, k1))
        curve("holdover (valid = 0 from event %d to %d)" % (k0, k1), log[k0 - 2:k1 + 12], every=8,
              limit=30)
        self.assertEqual(sv.steps, 0)
        f = log[k0 - 1]["freq"]
        for e in log[k0 + 1:k1]:
            self.assertEqual(e["kind"], "ignore")
            self.assertEqual(e["trim"], f, "frequency held, slew expired")
            self.assertFalse(e["step_en"])
        # Drift during the outage: the frequency error left after learning.
        drift = max(abs(e["x"]) for e in log[k0:k1 + 1])
        print("holdover: max |x| over %d s: %.3f ms" % ((k1 - k0) * 64, drift * 1e3))
        self.assertLess(drift, 20e-3)
        self.assertTrue(all(not e["synced"] for e in log[k0 + 9:k1]))
        # Recovery without a step.
        r, m = tail_stats(log, log[k1]["t"] + 1200)
        self.assertLess(m, 1e-3)

    def test_uncorrectable_frequency(self):
        sv, log = S.simulate(e_ppm=600, seed=5, duration=3600)
        self.assertEqual(sv.steps, 0)
        for e in log:
            self.assertLessEqual(abs(e["trim"]), S.MAXTRIM)
            self.assertLessEqual(abs(e["freq"]), S.MAXTRIM)


if __name__ == "__main__":
    unittest.main()
