"""The car's sign reading far below the map's limit is not used (route 0000010d, Warm Springs Blvd: 25 mph read on a
40 mph road)."""
import types
import unittest

from openpilot.common.constants import CV
from openpilot.cereal import custom
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Policy
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_resolver import SpeedLimitResolver

Source = custom.LongitudinalPlanSP.SpeedLimit.Source


def _resolve(car_mph, map_mph, policy=Policy.car_state_priority):
  r = SpeedLimitResolver()
  r.policy = policy
  r.update_params = lambda: None
  r._get_from_car_state = lambda sm: r.limit_solutions.__setitem__(Source.car, car_mph * CV.MPH_TO_MS)
  r._get_from_map_data = lambda sm: r.limit_solutions.__setitem__(Source.map, map_mph * CV.MPH_TO_MS)
  r.update(15.0, types.SimpleNamespace())
  return r.speed_limit * CV.MS_TO_MPH, r.source


class TestCarDropGuard(unittest.TestCase):
  def test_far_below_the_map_uses_the_map(self):
    limit, source = _resolve(25, 40)
    assert abs(limit - 40) < 0.01 and source == Source.map

  def test_close_to_the_map_or_above_keeps_the_car(self):
    for car, mp in ((35, 40), (45, 40), (31, 40)):
      limit, source = _resolve(car, mp)
      assert abs(limit - car) < 0.01 and source == Source.car, (car, mp)

  def test_without_a_map_limit_keeps_the_car(self):
    limit, source = _resolve(25, 0)
    assert abs(limit - 25) < 0.01 and source == Source.car

  def test_car_only_policy_unchanged(self):
    limit, source = _resolve(25, 40, Policy.car_state_only)
    assert abs(limit - 25) < 0.01 and source == Source.car
