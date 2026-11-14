"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import unittest

from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.selfdrived.driver_alerts import BlinkerBlindSpot, SpeedLimitPosted

DT = 0.01
MPH25, MPH35 = 25 * CV.MPH_TO_MS, 35 * CV.MPH_TO_MS


def run(alert, n, *args):
  return sum(bool(alert.update(*args)) for _ in range(n))


class TestSpeedLimitPosted(unittest.TestCase):
  def test_first_reading_is_silent(self):
    self.assertEqual(run(SpeedLimitPosted(DT), 500, MPH25, True, 10.0), 0)

  def test_change_chimes_once_after_holding(self):
    a = SpeedLimitPosted(DT)
    run(a, 100, MPH25, True, 10.0)
    self.assertEqual(run(a, 90, MPH35, True, 10.0), 0)   # not held 1 s yet
    self.assertEqual(run(a, 500, MPH35, True, 10.0), 1)

  def test_one_frame_misread_is_silent(self):
    a = SpeedLimitPosted(DT)
    run(a, 100, MPH25, True, 10.0)
    run(a, 5, MPH35, True, 10.0)
    self.assertEqual(run(a, 500, MPH25, True, 10.0), 0)

  def test_dropout_and_same_limit_back_is_silent(self):
    a = SpeedLimitPosted(DT)
    run(a, 100, MPH25, True, 10.0)
    run(a, 300, 0.0, False, 10.0)
    self.assertEqual(run(a, 500, MPH25, True, 10.0), 0)

  def test_unit_rounding_is_not_a_change(self):
    a = SpeedLimitPosted(DT)
    run(a, 100, MPH25, True, 10.0)
    self.assertEqual(run(a, 500, 40 * CV.KPH_TO_MS, True, 10.0), 0)   # 40 km/h ~ 24.9 mph

  def test_no_chime_while_stopped_then_chimes_on_moving(self):
    a = SpeedLimitPosted(DT)
    run(a, 100, MPH25, True, 10.0)
    self.assertEqual(run(a, 500, MPH35, True, 0.0), 0)
    self.assertEqual(run(a, 10, MPH35, True, 5.0), 1)


class TestBlinkerBlindSpot(unittest.TestCase):
  def test_needs_blinker_and_blind_spot_on_the_same_side(self):
    a = BlinkerBlindSpot(DT)
    self.assertEqual(run(a, 100, True, False, False, True), 0)
    self.assertEqual(run(a, 100, False, False, True, True), 0)
    self.assertGreater(run(a, 100, False, True, False, True), 0)

  def test_brief_flicker_is_ignored(self):
    a = BlinkerBlindSpot(DT)
    self.assertEqual(run(a, 20, True, False, True, False), 0)
    run(a, 1, False, False, False, False)
    self.assertEqual(run(a, 20, True, False, True, False), 0)


if __name__ == "__main__":
  unittest.main()
