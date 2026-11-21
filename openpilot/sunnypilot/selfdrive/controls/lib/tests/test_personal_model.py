import json
import os
import tempfile
import unittest
from unittest import mock

import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib.personal import features as pf
from openpilot.sunnypilot.selfdrive.controls.lib.personal import model as pm
from openpilot.sunnypilot.tools import personal_train as pt


def _inputs(v=10.0, plan=None, **kw):
  plan = plan if plan is not None else [v] * len(pf.T_IDXS)
  return pf.Inputs(v_ego=v, a_ego=0.0, plan_v=plan, plan_a=[0.0] * len(plan), plan_yaw_rate=[0.0] * len(plan), **kw)


class TestFeatures(unittest.TestCase):
  def test_vector_matches_the_names(self):
    f = pf.features(_inputs())
    self.assertEqual(f.shape, (pf.N_FEATURES,))
    self.assertTrue(np.all(np.isfinite(f)))

  def test_no_current_motion_inputs(self):
    # a_ego and the planned acceleration extrapolate the driver's foot (and on the car, openpilot's own command)
    self.assertNotIn("a_ego", pf.FEATURES)
    self.assertFalse(any(n.startswith("plan_a") for n in pf.FEATURES))
    a = pf.features(pf.Inputs(10.0, 2.0, [10.0] * 33, [2.0] * 33, [0.0] * 33))
    b = pf.features(pf.Inputs(10.0, -2.0, [10.0] * 33, [-2.0] * 33, [0.0] * 33))
    np.testing.assert_array_equal(a, b)

  def test_plan_relative_to_speed(self):
    plan = list(10.0 + 0.5 * pf.T_IDXS)          # plan speeds up 0.5 m/s per s
    f = pf.features(_inputs(10.0, plan))
    self.assertAlmostEqual(f[pf.FEATURES.index("plan_v2")], 1.0, places=2)
    self.assertAlmostEqual(pf.residual_target(10.5, plan), -0.5, places=2)

  def test_lead_and_limits(self):
    f = pf.features(_inputs(10.0, lead_present=True, lead_d=15.0, lead_v_rel=-2.0, limit_valid=True, limit=13.4))
    i = pf.FEATURES.index
    self.assertEqual(f[i("lead")], 1.0)
    self.assertAlmostEqual(f[i("lead_gap_s")], 1.5)
    self.assertAlmostEqual(f[i("lead_vrel")], 2.0)       # closing is positive
    self.assertAlmostEqual(f[i("limit_minus_v")], 3.4, places=3)
    g = pf.features(_inputs(10.0))
    self.assertEqual(g[i("lead")], 0.0)
    self.assertEqual(g[i("lead_gap_s")], pf.GAP_NONE_S)
    self.assertEqual(g[i("limit_valid")], 0.0)

  def test_situations(self):
    self.assertEqual(pf.situation(pf.features(_inputs(10.0, lead_present=True, lead_d=20.0))), "lead")
    stop = list(np.maximum(10.0 - 2.0 * pf.T_IDXS, 0.0))
    self.assertEqual(pf.situation(pf.features(_inputs(10.0, stop))), "stop")
    self.assertEqual(pf.situation(pf.features(_inputs(10.0))), "free")


class TestModel(unittest.TestCase):
  def _data(self, n=4000, seed=1):
    rng = np.random.default_rng(seed)
    X = rng.normal(size=(n, pf.N_FEATURES))
    y = 0.8 * np.tanh(X[:, 1]) - 0.4 * X[:, 3] + 0.05 * rng.normal(size=n)
    return X, y

  def test_untrained_output_layer_predicts_no_correction(self):
    X, y = self._data(200)
    m = pm.fit(X, y, epochs=0)
    np.testing.assert_allclose(m.predict(X), 0.0)

  def test_fit_learns_and_beats_no_correction(self):
    X, y = self._data()
    m = pm.fit(X[:3000], y[:3000], X[3000:3500], y[3000:3500], epochs=60)
    err = np.mean(np.abs(m.predict(X[3500:]) - y[3500:]))
    self.assertLess(err, 0.25 * np.mean(np.abs(y[3500:])))

  def test_output_is_clamped(self):
    X, y = self._data(500)
    m = pm.fit(X, 100.0 * y, epochs=5, clip=1.5)
    self.assertLessEqual(np.max(np.abs(m.predict(10.0 * X))), 1.5)

  def test_save_load_roundtrip_and_bad_files(self):
    X, y = self._data(500)
    m = pm.fit(X, y, epochs=3)
    with tempfile.TemporaryDirectory() as d:
      with mock.patch.object(pm.dp, "profile_path", lambda name="me": os.path.join(d, name + ".json")):
        p = pm.save(m)
        self.assertEqual(p, os.path.join(d, "personal", "me.json"))
        np.testing.assert_allclose(pm.load().predict(X), m.predict(X))
        with open(p) as f:
          bad = json.load(f)
        bad["features"] = bad["features"][:-1]
        with open(p, "w") as f:
          json.dump(bad, f)
        self.assertIsNone(pm.load())                      # other features: refuse it
        with open(p, "w") as f:
          f.write("{not json")
        self.assertIsNone(pm.load())
        os.remove(p)
        self.assertIsNone(pm.load())


class TestSplit(unittest.TestCase):
  def test_newest_drives_are_the_test_set(self):
    routes = [f"{i:08x}--abcdefabcd" for i in (0x10, 0x2, 0x30, 0x4, 0x50, 0x6, 0x70, 0x8, 0x90, 0xa)]
    tr, va, te = pt.split(routes)
    self.assertEqual(len(tr) + len(va) + len(te), len(routes))
    self.assertLess(max(int(r[:8], 16) for r in tr), min(int(r[:8], 16) for r in va))
    self.assertLess(max(int(r[:8], 16) for r in va), min(int(r[:8], 16) for r in te))


if __name__ == "__main__":
  unittest.main()
