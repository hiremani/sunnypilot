"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import unittest
from types import SimpleNamespace

from openpilot.common.constants import CV
from openpilot.selfdrive.ui.sunnypilot.onroad.speed_limit_sign import sign_state

MPH25 = 25 * CV.MPH_TO_MS


def res(limit=MPH25, valid=True, last_valid=True, final=0.0):
  return SimpleNamespace(speedLimitLast=limit, speedLimitValid=valid, speedLimitLastValid=last_valid, speedLimitFinalLast=final)


class TestSpeedLimitSign(unittest.TestCase):
  def test_hidden_when_mode_off_or_no_limit(self):
    self.assertFalse(sign_state(res(), 10.0, 0, False)[0])
    self.assertFalse(sign_state(res(valid=False, last_valid=False), 10.0, 1, False)[0])
    self.assertFalse(sign_state(res(limit=0.0), 10.0, 1, False)[0])

  def test_shows_limit_in_display_units(self):
    self.assertEqual(sign_state(res(), 5.0, 1, False)[1], "25")
    self.assertEqual(sign_state(res(), 5.0, 1, True)[1], "40")

  def test_overspeed_only_in_warning_or_assist(self):
    fast = 30 * CV.MPH_TO_MS
    self.assertFalse(sign_state(res(), fast, 1, False)[2])
    self.assertTrue(sign_state(res(), fast, 2, False)[2])
    self.assertTrue(sign_state(res(), fast, 3, False)[2])
    self.assertFalse(sign_state(res(), 24 * CV.MPH_TO_MS, 2, False)[2])

  def test_overspeed_uses_the_limit_with_offset(self):
    self.assertFalse(sign_state(res(final=30 * CV.MPH_TO_MS), 28 * CV.MPH_TO_MS, 2, False)[2])

  def test_real_capnp_message(self):
    import openpilot.cereal.messaging as messaging
    m = messaging.new_message('longitudinalPlanSP')
    r = m.longitudinalPlanSP.speedLimit.resolver
    r.speedLimitLast = MPH25
    r.speedLimitValid = True
    r.speedLimitLastValid = True
    self.assertEqual(sign_state(m.longitudinalPlanSP.speedLimit.resolver, 5.0, 2, False), (True, "25", False, False))

  def test_stale_reading_greyed(self):
    show, _, _, stale = sign_state(res(valid=False, last_valid=True), 5.0, 1, False)
    self.assertTrue(show and stale)


if __name__ == "__main__":
  unittest.main()
