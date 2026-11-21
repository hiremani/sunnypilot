import json
import os
import tempfile
import unittest
from unittest import mock

from openpilot.sunnypilot.selfdrive.controls.lib import driver_profile as dp
from openpilot.sunnypilot.selfdrive.controls.lib import profile_versions as pv


class TestProfileVersions(unittest.TestCase):
  def setUp(self):
    self.tmp = tempfile.TemporaryDirectory()
    self.env = mock.patch.dict(os.environ, {"DRIVER_PROFILES_DIR": self.tmp.name})
    self.env.start()
    self.clock = 1_790_000_000.0
    self.time = mock.patch.object(pv.time, "time", lambda: self._tick())
    self.time.start()

  def tearDown(self):
    self.time.stop()
    self.env.stop()
    self.tmp.cleanup()

  def _tick(self):
    self.clock += 60.0
    return self.clock

  def _file(self, decel, name="new.tmp"):
    p = os.path.join(self.tmp.name, name)
    with open(p, "w") as f:
      json.dump(dp.to_dict(dp.DriverProfile(stop_decel=decel)), f)
    return p

  def test_install_keeps_what_it_replaced_and_restore_puts_it_back(self):
    with open(dp.profile_path("me"), "w") as f:
      json.dump(dp.to_dict(dp.DriverProfile(stop_decel=1.1)), f)
    pv.install("profile", "me", self._file(0.8), label="new")
    vs = pv.list_versions("profile", "me")
    self.assertEqual([v["label"] for v in vs], ["new", "before new"])
    self.assertTrue(vs[0]["live"])
    self.assertEqual(dp.load("me").stop_decel, 0.8)
    pv.restore("profile", "me", vs[1]["id"])
    self.assertEqual(dp.load("me").stop_decel, 1.1)
    self.assertEqual(len(pv.list_versions("profile", "me")), 2)     # both contents were versions already

  def test_same_content_is_stored_once(self):
    pv.install("profile", "me", self._file(0.8))
    pv.install("profile", "me", self._file(0.8))
    pv.snapshot("profile", "me")
    self.assertEqual(len(pv.list_versions("profile", "me")), 1)

  def test_prune_keeps_the_newest_and_the_live_one(self):
    pv.install("profile", "me", self._file(0.5), label="oldest")
    oldest = pv.list_versions("profile", "me")[0]["id"]
    for i in range(pv.KEEP + 3):
      pv.install("profile", "me", self._file(0.6 + i * 0.01))
    self.assertEqual(len(pv.list_versions("profile", "me")), pv.KEEP)
    self.assertNotIn(oldest, [v["id"] for v in pv.list_versions("profile", "me")])

  def test_refuses_bad_files_ids_and_names(self):
    bad = os.path.join(self.tmp.name, "bad.tmp")
    with open(bad, "w") as f:
      f.write("{nope")
    with self.assertRaises(ValueError):
      pv.install("profile", "me", bad)
    self.assertFalse(os.path.exists(dp.profile_path("me")))       # nothing replaced
    for vid in ("../../etc/passwd", "20260101-000000-deadbeef", None):
      with self.assertRaises(ValueError):
        pv.restore("profile", "me", vid)
    with self.assertRaises(ValueError):
      pv.list_versions("profile", "../x")
    with self.assertRaises(ValueError):
      pv.live_path("other", "me")

  def test_personal_model_versions_live_apart(self):
    m = os.path.join(self.tmp.name, "model.tmp")
    with open(m, "w") as f:
      json.dump({"heads": {}}, f)
    pv.install("personal", "me", m, label="v2")
    self.assertTrue(os.path.isfile(os.path.join(self.tmp.name, "personal", "me.json")))
    self.assertEqual(len(pv.list_versions("personal", "me")), 1)
    self.assertEqual(pv.list_versions("profile", "me"), [])

  def test_old_backups_become_versions(self):
    b = self._file(0.9, ".me.json.bak-20261120")
    os.utime(b, (1_790_000_000, 1_790_000_000))
    out = pv.import_backups("me")
    self.assertEqual(len(out), 1)
    self.assertEqual(out[0]["source"], "backup")
    self.assertIn("20261120", out[0]["label"])


if __name__ == "__main__":
  unittest.main()


class TestWiring(unittest.TestCase):
  def test_turn_values_from_the_profile(self):
    from openpilot.sunnypilot.selfdrive.controls.lib import turn_planner as tp
    p = dp.DriverProfile(turn_start_lead_s=-0.7, turn_radius_left_m=11.0, turn_radius_right_m=9.0, turn_in_deg_s=66.0,
                         turn_unwind_deg_s=150.0, turn_arm_mph=16.7, junction_turn_mph=12.0)
    t = tp.TurnTuning()
    tp.apply_profile(t, p)
    self.assertEqual((t.lead_s, t.radius_left_m, t.radius_right_m, t.turn_in_wheel_deg_s, t.unwind_wheel_deg_s),
                     (-0.7, 11.0, 9.0, 66.0, 150.0))
    self.assertAlmostEqual(t.v_arm_max, 16.7 * 0.44704)
    self.assertAlmostEqual(t.v_turn, 12.0 * 0.44704)

  def test_planner_logs_each_new_version_once(self):
    from types import SimpleNamespace
    from openpilot.sunnypilot.selfdrive.controls.lib import longitudinal_planner as lp
    with tempfile.TemporaryDirectory() as d, mock.patch.dict(os.environ, {"DRIVER_PROFILES_DIR": d}), \
         mock.patch.object(lp, "Params") as P, mock.patch.object(lp.cloudlog, "event") as ev:
      P.return_value.get.return_value = "me"
      fake = SimpleNamespace(profile_on=True, profile_version=None)
      log = lp.LongitudinalPlannerSP._log_profile_version
      with open(dp.profile_path("me"), "w") as f:
        json.dump(dp.to_dict(dp.DriverProfile()), f)
      log(fake)
      log(fake)
      self.assertEqual(ev.call_count, 1)
      with open(dp.profile_path("me"), "w") as f:
        json.dump(dp.to_dict(dp.DriverProfile(stop_decel=0.8)), f)
      log(fake)
      self.assertEqual(ev.call_count, 2)
      self.assertEqual(ev.call_args.kwargs["sha"], pv.live_sha("profile", "me")[:12])
