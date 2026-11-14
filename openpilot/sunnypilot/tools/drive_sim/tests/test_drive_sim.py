"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import os
import tempfile
import unittest

from openpilot.sunnypilot.tools.drive_sim import server, sim


class TestDriveSim(unittest.TestCase):
  def test_routes_and_segments_need_an_rlog(self):
    with tempfile.TemporaryDirectory() as root:
      for n in (0, 1, 2, 10):
        d = os.path.join(root, f"000000e5--abc--{n}")
        os.mkdir(d)
        if n != 1:
          open(os.path.join(d, "rlog.zst"), "w").close()
      os.mkdir(os.path.join(root, "000000e6--def--0"))   # no rlog: not listed
      self.assertEqual(sim.list_routes(root), [{"route": "000000e5--abc", "segments": 3}])
      names = [os.path.basename(d) for d in sim.segment_dirs(root, "000000e5--abc")]
      self.assertEqual(names, ["000000e5--abc--0", "000000e5--abc--2", "000000e5--abc--10"])
      self.assertEqual([os.path.basename(d) for d in sim.segment_dirs(root, "000000e5--abc", "2-9")], ["000000e5--abc--2"])

  def test_missing_route_is_an_error(self):
    with tempfile.TemporaryDirectory() as root, self.assertRaises(FileNotFoundError):
      sim.extract(root, "nope")

  def test_simulate_runs_the_current_code_on_frames(self):
    # a straight approach north toward no mapped signs: the controllers run, stay idle, and every column is filled
    frames = []
    for i in range(40):
      lat = 37.5 + i * 1e-5
      frames.append({"t": i * 0.05, "gps": (lat, -121.95, 0.0, 8.0, True, 3.0, 1_800_000_000_000 + i * 50),
                     "cs": (8.0, 0.0, 0.0, False, False, False, 0.0, False, False, False, False), "lead": (False, 0.0),
                     "lat_active": True, "stop": False, "curv": 0.0, "vel": [8.0] * 33, "pos": [8.0 * k for k in range(33)],
                     "desire": 0})
    out = sim.simulate({"route": "r", "segs": None, "service": "gpsLocation", "frames": frames})
    self.assertEqual(out["frames"], 40)
    self.assertTrue(all(len(v) == 40 for v in out["cols"].values()))
    self.assertEqual(set(out["cols"]["state"]), {"idle"})

  def test_reload_picks_up_the_modules(self):
    mod = server.reload_code()
    self.assertTrue(hasattr(mod, "simulate"))


if __name__ == "__main__":
  unittest.main()


class TestClosedLoop(unittest.TestCase):
  def _frames(self, n=400, v=8.0):
    out = []
    for i in range(n):
      lat = 37.5 + i * v * 0.05 / 111_195.0   # driving north at v
      out.append({"t": i * 0.05, "gps": (lat, -121.95, 0.0, v, True, 3.0, 1_800_000_000_000 + i * 50),
                  "cs": (v, 0.0, 0.0, False, False, False, 0.0, False, False, False, False), "lead": (False, 0.0),
                  "lat_active": True, "stop": False, "curv": 0.0, "vel": [v] * 33, "pos": [v * k for k in range(33)], "desire": 0})
    return out

  def test_without_signs_it_drives_like_the_recording(self):
    out = sim.simulate_closed_loop({"route": "r", "segs": None, "service": "gpsLocation", "frames": self._frames()})
    self.assertTrue(out["completed"])
    self.assertAlmostEqual(out["simulated_s"], out["recorded_s"], delta=2.0)
    self.assertEqual(out["stops"], [])

  def test_path_drops_jitter_and_measures_distance(self):
    frames = self._frames(n=100)
    path = sim.build_path(frames)
    self.assertAlmostEqual(path[-1][0], 99 * 8.0 * 0.05, delta=2.0)
    self.assertTrue(all(abs(p[3]) < 1.0 or abs(p[3] - 360) < 1.0 for p in path[3:-3]))   # heading north
