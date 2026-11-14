"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import unittest
from types import SimpleNamespace

from openpilot.selfdrive.controls.controlsd import Controls


class FakeParams:
  def __init__(self, **values):
    self.values = values

  def get_bool(self, key):
    if key not in self.values:
      raise KeyError(key)
    return self.values[key]


def controls(params, planner_out=0.05, raises=False):
  c = Controls.__new__(Controls)   # just the map-turn hook, without the rest of controlsd
  c.params, c.gps_service, c._turn_param_frame = params, "gpsLocationExternal", 0
  c.map_turn_assist, c.map_turn_curvature = False, None
  c.sm = {"gpsLocationExternal": SimpleNamespace()}

  def update(*args, **kwargs):
    if raises:
      raise RuntimeError("boom")
    return planner_out
  c.turn_planner = SimpleNamespace(update=update)
  return c


CS = SimpleNamespace(vEgo=4.0, yawRate=0.0, leftBlinker=False, rightBlinker=True, steeringPressed=False, steeringTorque=0.0)
CC = SimpleNamespace(latActive=True)


class TestMapTurnHook(unittest.TestCase):
  def test_off_unless_both_settings_are_on(self):
    self.assertIsNone(controls(FakeParams(MapTurnAssist=False, LaneTurnDesire=True)).update_map_turn(CS, CC))
    self.assertIsNone(controls(FakeParams(MapTurnAssist=True, LaneTurnDesire=False)).update_map_turn(CS, CC))
    self.assertEqual(controls(FakeParams(MapTurnAssist=True, LaneTurnDesire=True)).update_map_turn(CS, CC), 0.05)

  def test_unknown_param_means_off_not_a_crash(self):
    self.assertIsNone(controls(FakeParams(LaneTurnDesire=True)).update_map_turn(CS, CC))

  def test_planner_failure_falls_back_to_the_model(self):
    self.assertIsNone(controls(FakeParams(MapTurnAssist=True, LaneTurnDesire=True), raises=True).update_map_turn(CS, CC))


if __name__ == "__main__":
  unittest.main()
