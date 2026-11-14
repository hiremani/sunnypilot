"""The driver profile loader (driver_profile.py): named files, clamped to bounds, defaults on anything wrong."""
import json
import os
import tempfile
import unittest
from unittest import mock

from openpilot.sunnypilot.selfdrive.controls.lib import driver_profile as dp


class FakeParams:
  def __init__(self, **v):
    self.v = v

  def get_bool(self, k):
    return bool(self.v[k])

  def get(self, k, return_default=False):
    return self.v.get(k)


class TestDriverProfile(unittest.TestCase):
  def setUp(self):
    self.dir = tempfile.TemporaryDirectory()
    self.env = mock.patch.dict(os.environ, {"DRIVER_PROFILES_DIR": self.dir.name})
    self.env.start()

  def tearDown(self):
    self.env.stop()
    self.dir.cleanup()

  def write(self, name, values=None, raw=None, version=dp.VERSION):
    with open(os.path.join(self.dir.name, name + ".json"), "w") as f:
      f.write(raw if raw is not None else json.dumps({"version": version, "values": values or {}}))

  def test_values_are_clamped_to_bounds(self):
    self.write("me", {"stop_decel": 9.0, "town_offset_mph": 12.0, "fast_offset_mph": 6.5, "follow_gap_s": "x",
                      "curve_lat_accel": float("nan"), "launch_accel": 1.3})
    p = dp.load("me")
    assert p.stop_decel == 1.5 and p.town_offset_mph == 0.0 and p.fast_offset_mph == 5.0 and p.launch_accel == 1.0
    assert p.follow_gap_s == dp.DriverProfile.follow_gap_s and p.curve_lat_accel == dp.DriverProfile.curve_lat_accel

  def test_anything_wrong_means_defaults(self):
    self.write("broken", raw="{nope")
    self.write("old", {"stop_decel": 0.8}, version=0)
    for name in ("broken", "old", "missing", "../me", ""):
      assert dp.load(name) == dp.DriverProfile(), name

  def test_names(self):
    for good in ("me", "Relaxed drive", "night_2"):
      assert dp.valid_name(good)
    for bad in ("", " me", "me ", "../x", "a/b", "x" * 33, 3, None):
      assert not dp.valid_name(bad)

  def test_list_and_active(self):
    self.write("me", {"stop_decel": 0.95})
    self.write("relaxed", {"stop_decel": 0.8})
    self.write("bad name!", {"stop_decel": 0.9})
    assert [p["name"] for p in dp.list_profiles()] == ["me", "relaxed"]
    assert dp.active(FakeParams(DriverProfile=False, DriverProfileName="relaxed")) == dp.DriverProfile()   # off
    assert dp.active(FakeParams(DriverProfile=True, DriverProfileName="relaxed")).stop_decel == 0.8
    assert dp.active(FakeParams(DriverProfile=True, DriverProfileName="../x")).stop_decel == 0.95          # -> "me"

  def test_never_raises(self):
    class Broken:
      def get_bool(self, k):
        raise KeyError(k)
    assert dp.active(Broken()) == dp.DriverProfile()


if __name__ == "__main__":
  unittest.main()
