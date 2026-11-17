"""The low-speed stop test (stop_test.py): off by default, softens only near the end of an unhurried stop."""
import unittest

from openpilot.sunnypilot.selfdrive.controls.lib import stop_test as st


class TestStopTest(unittest.TestCase):
  def test_off_changes_nothing(self):
    for v in (0.0, 0.5, 1.5, 3.0, 10.0):
      self.assertEqual(st.soften(-1.0, v, -0.9, 0), -1.0)

  def test_soft_caps_braking_below_5_mph(self):
    self.assertEqual(st.soften(-1.0, 1.5, -0.9, 1), st.SOFT_DECEL)
    self.assertEqual(st.soften(-1.0, 3.0, -0.9, 1), -1.0)          # above 5 mph: unchanged
    self.assertEqual(st.soften(-0.2, 1.5, -0.2, 1), -0.2)          # already softer: unchanged

  def test_early_ease_floor_falls_with_speed(self):
    self.assertEqual(st.soften(-2.0, 5.0, -0.9, 2), -2.0)          # above 8 mph: unchanged
    hi, lo = st.soften(-2.0, 3.0, -0.9, 2), st.soften(-2.0, 0.5, -0.9, 2)
    self.assertLess(hi, lo)                                        # softer as the car slows
    self.assertGreaterEqual(lo, st.EASE_START_DECEL)
    self.assertLessEqual(lo, st.EASE_END_DECEL)

  def test_a_closing_lead_always_gets_the_full_request(self):
    for variant in (1, 2):
      self.assertEqual(st.soften(-2.5, 1.5, -2.0, variant), -2.5)

  def test_standstill_hold_and_acceleration_unchanged(self):
    for variant in (1, 2):
      self.assertEqual(st.soften(-2.0, 0.0, -0.5, variant), -2.0)    # the hold at standstill
      self.assertEqual(st.soften(0.5, 1.5, 0.5, variant), 0.5)       # pulling away


if __name__ == "__main__":
  unittest.main()
