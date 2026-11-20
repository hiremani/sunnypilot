"""Junction memory (junction_memory.py): the driver's own start and finish at each junction they turned at before."""
import json
import os
import tempfile
import unittest
from unittest import mock

from openpilot.sunnypilot.selfdrive.controls.lib import junction_memory as jm
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_turn_planner import DT, SlowUnwindCar, planner, t_junction


def _pass(k, d_on, d_out, lat_on=1.5, lat_out=1.5):
  return {"key": k, "d_on": d_on, "lat_on": lat_on, "d_out": d_out, "lat_out": lat_out}


class TestMemoryFile(unittest.TestCase):
  def setUp(self):
    self.dir = tempfile.TemporaryDirectory()
    self.env = mock.patch.dict(os.environ, {"DRIVER_PROFILES_DIR": self.dir.name})
    self.env.start()

  def tearDown(self):
    self.env.stop()
    self.dir.cleanup()

  def test_median_of_the_passes_after_two(self):
    mem = jm.build([_pass("a", 10, 20), _pass("a", 14, 24), _pass("a", 30, 22), _pass("b", 12, 18)])
    self.assertEqual(set(mem), {"a"})                        # b: one pass, not yet remembered
    self.assertEqual((mem["a"]["d_on"], mem["a"]["d_out"], mem["a"]["n"]), (14.0, 22.0, 3))

  def test_save_load_and_bad_entries(self):
    p = jm.save({"a": {"d_on": 12, "lat_on": 1, "d_out": 20, "lat_out": 1, "n": 3},
                 "bad": {"d_on": 400, "lat_on": 1, "d_out": 20, "lat_out": 1}})
    self.assertTrue(p.endswith(os.path.join("junctions", "me.json")))   # not beside the profiles (profile list)
    self.assertEqual(set(jm.load()), {"a"})
    with open(p, "w") as f:
      json.dump({"version": 99, "junctions": {}}, f)
    self.assertEqual(jm.load(), {})
    self.assertEqual(jm.load("../x"), {})                    # a bad profile name

  def test_key_is_the_turn(self):
    self.assertNotEqual(jm.key(1, 2, 37.5, -121.9), jm.key(2, 1, 37.5, -121.9))   # the other direction
    self.assertEqual(jm.key(1, 2, 37.500001, -121.9), jm.key(1, 2, 37.5, -121.9))   # ~0.1 m apart: the same node


class TestPlannerWithMemory(unittest.TestCase):
  def _drive(self, remembered: bool, shape: bool = True):
    p = planner(t_junction(right=True, left=True))
    if shape:
      p.tuning.path_in, p.tuning.path_out = 0.85, 0.40
    if remembered:
      roads = t_junction(right=True, left=True)
      lat, lon = roads.to_latlon(0.0, 0.0)
      p.tuning.junctions = {jm.key(1, 2, lat, lon): {"d_on": 14.0, "lat_on": 2.0, "d_out": 18.0, "lat_out": 1.75, "n": 3}}
    car = SlowUnwindCar(p, 120.0)
    used = False
    for _ in range(int(30.0 / DT)):
      car.step(right=True)
      used = used or p.path is not None
    return p, car, used

  def test_a_remembered_junction_uses_the_drivers_path(self):
    p, car, used = self._drive(True)
    self.assertTrue(used)
    self.assertEqual(p.log.reason, "done")
    self.assertAlmostEqual(car.h, 90.0, delta=8.0)
    self.assertAlmostEqual(car.y, -1.75, delta=1.5)          # in the lane the driver used on the new road

  def test_unknown_junction_or_no_shape_keeps_the_arc(self):
    self.assertFalse(self._drive(False)[2])
    self.assertFalse(self._drive(True, shape=False)[2])
    p, car, _ = self._drive(False)
    self.assertEqual(p.log.reason, "done")


if __name__ == "__main__":
  unittest.main()
