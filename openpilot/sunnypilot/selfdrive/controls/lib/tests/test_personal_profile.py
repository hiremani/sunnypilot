import os
import tempfile
import unittest
from unittest import mock

import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib import driver_profile as dp
from openpilot.sunnypilot.selfdrive.controls.lib.personal import heads as hd
from openpilot.sunnypilot.selfdrive.controls.lib.personal import profile as pprof


class TestCurveHead(unittest.TestCase):
  def test_learns_a_dependence(self):
    rng = np.random.default_rng(0)
    x = rng.uniform(5, 20, 600)
    y = 0.5 + 0.04 * x + 0.05 * rng.normal(size=600)          # firmer braking from higher speeds
    h = hd.fit(x, y, min_bin=50, max_bins=6)
    self.assertTrue(h.is_curve)
    self.assertLess(h.predict(6.0), h.predict(18.0))
    self.assertAlmostEqual(h.predict(None), float(np.median(y)))   # no context: the median over everything

  def test_too_few_events_is_a_constant(self):
    h = hd.fit(np.array([1.0, 2.0, 3.0]), np.array([0.9, 1.0, 1.1]), min_bin=20)
    self.assertFalse(h.is_curve)
    self.assertAlmostEqual(h.predict(2.5), 1.0)

  def test_quantile_and_loss(self):
    y = np.arange(101, dtype=float)
    self.assertAlmostEqual(hd.statistic(y, "p75"), 75.0)
    self.assertLess(hd.loss(np.full(101, 75.0), y, "p75"), hd.loss(np.full(101, 50.0), y, "p75"))

  def test_discrete_and_roundtrip(self):
    x = np.array([25.0] * 30 + [35.0] * 30 + [45.0] * 5)
    y = np.array([-4.0] * 30 + [1.0] * 30 + [9.0] * 5)
    h = hd.fit_discrete(x, y, min_per_key=20)
    np.testing.assert_allclose(h.xs, [25.0, 35.0])              # 45 has too few
    h2 = hd.CurveHead.from_dict(h.to_dict())
    self.assertAlmostEqual(h2.predict(30.0), h.predict(30.0))
    bad = h.to_dict()
    bad["xs"] = [35.0, 25.0]
    with self.assertRaises(ValueError):
      hd.CurveHead.from_dict(bad)


def _profile():
  heads = {
    "stop_decel": hd.CurveHead([4.0, 18.0], [0.75, 1.15], 0.97, 108, "v", "stop_decel"),
    "follow_gap_s": hd.CurveHead([10.0, 30.0], [1.95, 1.0], 1.69, 9000, "v", "follow_gap_s"),   # 1.0 is below bounds
    "lead_coast_s": hd.CurveHead([1.0, 7.0], [1.7, 4.2], 2.57, 484, "closing", "lead_coast_s"),
    "junction_turn_mph": hd.CurveHead([70.0, 90.0], [5.0, 7.5], 5.9, 319, "turn_deg", "junction_turn_mph"),
    "coast_s": hd.CurveHead(const=0.0, n=111),
    "launch_accel": hd.CurveHead(const=0.9, n=2),                # too few: not used
    "limit_offsets_mph": hd.CurveHead([25.0, 35.0, 65.0], [-3.9, 1.1, 1.9], 0.0, 66040, "limit_mph", "limit_offsets_mph"),
  }
  use = {k: k != "launch_accel" for k in heads}
  return pprof.PersonalProfile(heads, use)


class TestCruiseAccelHead(unittest.TestCase):
  def test_fills_the_three_bands(self):
    from openpilot.sunnypilot.selfdrive.controls.lib import profile_long as pl
    pp = pprof.PersonalProfile({"cruise_accel": hd.CurveHead([8.0, 24.0], [0.65, 0.27], 0.6, 19203, "v", "cruise_accel_town")},
                               {"cruise_accel": True})
    p = pp.at()
    self.assertAlmostEqual(p.cruise_accel_town, float(np.interp(10.0, [8.0, 24.0], [0.65, 0.27])))
    self.assertAlmostEqual(p.cruise_accel_fast, 0.27)
    self.assertLess(pl.comfort_accel_cap(24.6, p), pl.comfort_accel_cap(24.6))


class TestPersonalProfile(unittest.TestCase):
  def test_values_by_situation(self):
    pp = _profile()
    self.assertAlmostEqual(pp.at(v=4.0).stop_decel, 0.75)
    self.assertAlmostEqual(pp.at(v=18.0).stop_decel, 1.15)
    self.assertAlmostEqual(pp.at().stop_decel, 0.97)             # no situation: over all the driver's stops
    self.assertAlmostEqual(pp.at(closing=7.0).lead_coast_s, 4.2)
    self.assertAlmostEqual(pp.at(turn_deg=70.0).junction_turn_mph, 5.0)
    self.assertAlmostEqual(pp.at().junction_turn_mph, 7.5)       # a right-angle turn by default

  def test_bounds_and_unused_heads(self):
    pp = _profile()
    lo, _ = dp.BOUNDS["follow_gap_s"]
    self.assertEqual(pp.at(v=30.0).follow_gap_s, lo)              # never closer than the profile allows
    self.assertEqual(pp.at().launch_accel, dp.DriverProfile().launch_accel)   # too few events: the default
    self.assertEqual(pp.at().narrow_speed_mph, dp.DriverProfile().narrow_speed_mph)   # no head: the default
    offs = pp.at().limit_offsets_mph
    self.assertEqual(offs["25"], -3.9)
    self.assertEqual(offs["35"], 0.0)                             # never over the limit in town
    self.assertEqual(offs["65"], 1.9)

  def test_file_roundtrip_and_bad_files(self):
    pp = _profile()
    with tempfile.TemporaryDirectory() as d:
      with mock.patch.object(pprof.dp, "profile_path", lambda name="me": os.path.join(d, name + ".json")):
        self.assertIsNone(pprof.load())
        pprof.save(pp)
        back = pprof.load()
        self.assertAlmostEqual(back.at(v=10.0).stop_decel, pp.at(v=10.0).stop_decel)
        pprof.write_raw({"version": 1, "layers": []})             # the old speed-only file
        self.assertIsNone(pprof.load())
        with open(pprof.path(), "w") as f:
          f.write("{")
        self.assertIsNone(pprof.load())

  def test_active_uses_the_personal_model_only_when_both_params_are_set(self):
    pp = _profile()

    class P:
      def __init__(self, on):
        self.on = on

      def get_bool(self, k):
        return self.on.get(k, False)

      def get(self, k, return_default=False):
        return "me"

    with tempfile.TemporaryDirectory() as d:
      with mock.patch.object(dp, "profile_path", lambda name="me": os.path.join(d, name + ".json")), \
           mock.patch.object(pprof.dp, "profile_path", lambda name="me": os.path.join(d, name + ".json")):
        pprof.save(pp)
        self.assertIsNone(dp.active_personal(P({"DriverProfile": True})))
        self.assertIsNone(dp.active_personal(P({"PersonalModel": True})))
        both = P({"DriverProfile": True, "PersonalModel": True})
        self.assertIsNotNone(dp.active_personal(both))
        self.assertAlmostEqual(dp.active(both).stop_decel, 0.97)
        self.assertEqual(dp.active(P({"DriverProfile": False, "PersonalModel": True})).stop_decel,
                         dp.DriverProfile().stop_decel)


if __name__ == "__main__":
  unittest.main()
