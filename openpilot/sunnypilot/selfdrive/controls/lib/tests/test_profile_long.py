"""The driver profile's longitudinal rules (profile_long.py)."""
import unittest

from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.controls.lib import profile_long as pl


class TestProfileLong(unittest.TestCase):
  def test_follow_t(self):
    # 15 m/s, measured gap 1.77 s: the MPC's 6 m stop distance is 0.4 s of it
    self.assertAlmostEqual(pl.follow_t(1.77, 15.0), 1.37, places=2)
    self.assertEqual(pl.follow_t(1.77, 2.0), pl.T_FOLLOW_MIN)      # slow: never below aggressive
    self.assertEqual(pl.follow_t(1.0, 30.0), pl.T_FOLLOW_MIN)
    self.assertAlmostEqual(pl.follow_t(2.5, 30.0), 2.3, places=2)

  def test_lead_coast(self):
    # route f5: 42 mph, lead 79 m ahead 5 mph slower -> gap 4.2 s
    v = 41.7 * CV.MPH_TO_MS
    self.assertTrue(pl.lead_coast(4.5, v, True, 79.0, -5 * CV.MPH_TO_MS))
    self.assertFalse(pl.lead_coast(3.0, v, True, 79.0, -5 * CV.MPH_TO_MS))   # outside the driver's gap
    self.assertFalse(pl.lead_coast(4.5, v, True, 79.0, +1.0))                # lead pulling away
    self.assertFalse(pl.lead_coast(4.5, v, True, 79.0, -0.2))                # barely slower
    self.assertFalse(pl.lead_coast(4.5, v, False, 79.0, -3.0))               # no lead
    self.assertFalse(pl.lead_coast(0.0, v, True, 20.0, -3.0))                # off
    self.assertFalse(pl.lead_coast(4.5, 0.5, True, 5.0, -0.6))               # standing

  def test_junction_turn(self):
    self.assertTrue(pl.junction_turn(True, False, 4.0, 0.08))
    self.assertTrue(pl.junction_turn(False, True, 4.0, -0.08))
    self.assertFalse(pl.junction_turn(False, False, 4.0, 0.08))   # no blinker
    self.assertFalse(pl.junction_turn(True, False, 12.0, 0.08))   # too fast
    self.assertFalse(pl.junction_turn(True, False, 4.0, 0.01))    # gentle curve, lane change

  def test_profile_speed(self):
    self.assertIsNone(pl.profile_speed(0.0, -5.0))
    self.assertAlmostEqual(pl.profile_speed(25 * CV.MPH_TO_MS, -5.3) * CV.MS_TO_MPH, 19.7, places=3)
    self.assertAlmostEqual(pl.profile_speed(65 * CV.MPH_TO_MS, 4.5) * CV.MS_TO_MPH, 69.5, places=3)


if __name__ == "__main__":
  unittest.main()
