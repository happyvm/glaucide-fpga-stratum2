import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import sync  # noqa: E402
from sync import Selected, State  # noqa: E402

REFS = [0xC0000263, 0xB70A6091, 0x11111111, 0x22222222]
NOW = 5000 << 32


def sel(**kw):
    base = dict(valid=True, peer=0, jitter=0, stratum=1, root_delay=0x333, root_disp=0x666,
                delay=0x0000_0000_0123_4567, disp=0x0000_0000_0010_0000, t4=NOW)
    base.update(kw)
    return Selected(**base)


class SyncTest(unittest.TestCase):
    def test_helpers(self):
        self.assertEqual(sync.ceil16(0), 0)
        self.assertEqual(sync.ceil16(1), 1)
        self.assertEqual(sync.ceil16(0x10000), 1)
        self.assertEqual(sync.ceil16(0x10001), 2)
        self.assertEqual(sync.sat32(1 << 40), 0xFFFFFFFF)
        self.assertEqual(sync.prec_floor(0xE8), 1 << 8)       # -24
        self.assertEqual(sync.prec_floor(0xE0), 1)            # -32
        self.assertEqual(sync.prec_floor(0xDF), 1)            # -33
        self.assertEqual(sync.prec_floor(0xFF), 1 << 31)      # -1
        self.assertEqual(sync.prec_floor(0), 1 << 32)

    def test_synced_stratum_and_refid(self):
        st = sync.pop(State(), sel(stratum=1, peer=1), NOW)
        out, _ = sync.outputs(st, NOW, True, 0xE8, REFS)
        self.assertTrue(out.synced)
        self.assertEqual((out.stratum, out.leap, out.ref_id, out.ref_ts), (2, 0, REFS[1], NOW))

    def test_unusable_selected(self):
        st = sync.pop(State(), sel(), NOW)
        for bad in (sel(valid=False), sel(stratum=15), sel(stratum=16), sel(stratum=0)):
            st2 = sync.pop(st, bad, NOW + 1)
            out, _ = sync.outputs(st2, NOW + 1, True, 0xE8, REFS)
            self.assertFalse(out.synced)
            self.assertEqual((out.stratum, out.leap), (16, 3))
        out, _ = sync.outputs(sync.pop(State(), sel(stratum=14), NOW), NOW, True, 0xE8, REFS)
        self.assertEqual(out.stratum, 15)

    def test_timeout_and_servo(self):
        st = sync.pop(State(), sel(), NOW)
        edge = NOW + sync.SYNC_TIMEOUT_Q32
        self.assertTrue(sync.outputs(st, edge, True, 0xE8, REFS)[0].synced)
        self.assertFalse(sync.outputs(st, edge + 1, True, 0xE8, REFS)[0].synced)
        self.assertFalse(sync.outputs(st, NOW, False, 0xE8, REFS)[0].synced)

    def test_dispersion_growth_literals(self):
        # Literals reused by rtl/tests/sync_state.rs.
        s = sel(jitter=0x0000_0000_0020_0000, t4=NOW)
        st = sync.pop(State(), s, NOW)
        self.assertEqual(sync.root_delay_of(s), 0x333 + 0x124)
        d0 = sync.root_disp_of(s, NOW, 0xE8)
        d1 = sync.root_disp_of(s, NOW + (100 << 32), 0xE8)
        d2 = sync.root_disp_of(s, NOW + (1000 << 32), 0xE8)
        self.assertEqual((d0, d1, d2), (1686, 1785, 2670))
        self.assertGreater(d1, d0)
        del st

    def test_saturation(self):
        s = sel(root_disp=0xFFFFFFF0, disp=1 << 40)
        self.assertEqual(sync.root_disp_of(s, NOW, 0xE8), 0xFFFFFFFF)
        s = sel(root_delay=0xFFFFFFFF, delay=1 << 32)
        self.assertEqual(sync.root_delay_of(s), 0xFFFFFFFF)

    def test_held_values_when_unsynchronised(self):
        st = sync.pop(State(), sel(peer=1), NOW)
        out, st = sync.outputs(st, NOW, True, 0xE8, REFS)
        out2, _ = sync.outputs(st, NOW, False, 0xE8, REFS)
        self.assertEqual((out2.ref_id, out2.ref_ts, out2.root_delay, out2.root_disp),
                         (out.ref_id, out.ref_ts, out.root_delay, out.root_disp))


if __name__ == "__main__":
    unittest.main()
