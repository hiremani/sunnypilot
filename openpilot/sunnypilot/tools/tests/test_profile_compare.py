import json
import os
import tempfile
import unittest
from unittest import mock

from openpilot.sunnypilot.selfdrive.controls.lib import driver_profile as dp
from openpilot.sunnypilot.selfdrive.controls.lib import profile_versions as pv
from openpilot.sunnypilot.tools import profile_compare as pc


class TestResolve(unittest.TestCase):
  def setUp(self):
    self.tmp = tempfile.TemporaryDirectory()
    self.env = mock.patch.dict(os.environ, {"DRIVER_PROFILES_DIR": self.tmp.name})
    self.env.start()
    for decel in (0.9, 0.7):
      src = os.path.join(self.tmp.name, "x.tmp")
      with open(src, "w") as f:
        json.dump(dp.to_dict(dp.DriverProfile(stop_decel=decel)), f)
      pv.install("profile", "me", src, label=str(decel))

  def tearDown(self):
    self.env.stop()
    self.tmp.cleanup()

  def test_specs(self):
    self.assertEqual(pc.resolve("default")[1], dp.DriverProfile())
    self.assertEqual(pc.resolve("me")[1].stop_decel, 0.7)
    vs = pv.list_versions("profile", "me")
    if vs[0]["created"] == vs[1]["created"]:
      self.skipTest("same-second installs: order by id")
    self.assertEqual(pc.resolve("@1")[1].stop_decel, 0.9)
    self.assertEqual(pc.resolve(f"me@{vs[1]['id']}")[1].stop_decel, 0.9)
    label, p, _ = pc.resolve("prev=me@1")
    self.assertEqual((label, p.stop_decel), ("prev", 0.9))
    self.assertEqual(pc.resolve(os.path.join(self.tmp.name, "me.json"))[1].stop_decel, 0.7)
    for bad in ("me@5", "me@20260101-000000-deadbeef", "nobody", os.path.join(self.tmp.name, "nope.json")):
      with self.assertRaises(SystemExit):
        pc.resolve(bad)


class TestRoutesAndSummary(unittest.TestCase):
  def test_newest_drives_skip_parked_ones(self):
    with tempfile.TemporaryDirectory() as root:
      for r in ("00000001--aaaaaaaaaa", "00000002--bbbbbbbbbb", "00000003--cccccccccc"):
        os.makedirs(os.path.join(root, r + "--0"))
      with open(os.path.join(root, "catalog.json"), "w") as f:
        json.dump({"drives": [{"route": "00000003--cccccccccc", "miles": 0.0}]}, f)
      self.assertEqual(pc.pick_routes(root, None, 1), ["00000002--bbbbbbbbbb"])
      self.assertEqual(pc.pick_routes(root, ["00000003--cccccccccc"], 1), ["00000003--cccccccccc"])
      with self.assertRaises(SystemExit):
        pc.pick_routes(root, ["00000009--dddddddddd"], 1)

  def test_summary_weights_speed_by_moving_time_and_pools_turns(self):
    rows = [{"route": "a", "moving_min": 30.0, "assisted_pct": 0, "results": {"A": {
               "mean_abs_mph": 2.0, "faster_5_pct": 10, "slower_5_pct": 0, "time_pct": 100,
               "turns": [{"side": "left", "taken": True, "max_lat": 1.0}, {"side": "right", "taken": False, "max_lat": None}]}}},
            {"route": "b", "moving_min": 10.0, "assisted_pct": 0, "results": {"A": {
               "mean_abs_mph": 6.0, "faster_5_pct": 30, "slower_5_pct": 20, "time_pct": 120,
               "turns": [{"side": "left", "taken": True, "max_lat": 3.0}]}}},
            {"route": "c", "error": "parked"}]
    s = pc.summarize(rows, ["A"])["A"]
    self.assertEqual((s["mean_abs_mph"], s["faster_5_pct"], s["time_pct"]), (3.0, 15, 105))
    self.assertEqual((s["turns"], s["taken"], s["median_off_m"], s["worst_off_m"]), (3, 2, 2.0, 3.0))


if __name__ == "__main__":
  unittest.main()
