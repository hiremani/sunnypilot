"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import unittest
from types import SimpleNamespace

from openpilot.selfdrive.ui.sunnypilot.onroad.stop_sign_indicator import (
  BRAKING, COASTING, IDLE, PROCEED_M, STOPPED, Phase, StopSignDisplay,
)

DT = 0.05


def msg(state, distance=0.0, sign_id=123, shadow=False):
  return SimpleNamespace(state=state, distance=distance, signId=sign_id, shadow=shadow)


class TestStopSignDisplay(unittest.TestCase):
  def test_nothing_without_a_sign(self):
    d = StopSignDisplay()
    self.assertEqual(d.update(msg(IDLE, 0.0, sign_id=0), 10.0, DT), Phase.NONE)

  def test_far_sign_hidden_near_sign_tracked(self):
    d = StopSignDisplay()
    self.assertEqual(d.update(msg(IDLE, 250.0), 10.0, DT), Phase.NONE)
    self.assertEqual(d.update(msg(IDLE, 120.0), 10.0, DT), Phase.TRACKED)
    self.assertFalse(d.blinking())

  def test_blinks_while_acting(self):
    d = StopSignDisplay()
    for st in (COASTING, BRAKING):
      self.assertEqual(d.update(msg(st, 30.0), 6.0, DT), Phase.ACTING)
      self.assertTrue(d.blinking())

  def test_stopped_is_steady_and_says_tap_gas(self):
    d = StopSignDisplay()
    d.update(msg(BRAKING, 3.0), 2.0, DT)
    self.assertEqual(d.update(msg(STOPPED, 0.3), 0.0, DT), Phase.STOPPED)
    self.assertFalse(d.blinking())
    self.assertEqual(d.label(False), "tap gas")

  def test_blinks_after_gas_tap_until_through_the_junction(self):
    d = StopSignDisplay()
    d.update(msg(STOPPED, 0.3), 0.0, DT)
    rolled = 0.0
    while rolled < PROCEED_M - 1.0:
      self.assertEqual(d.update(msg(IDLE, 0.0, sign_id=0), 3.0, DT), Phase.PROCEEDING)
      self.assertTrue(d.blinking())
      rolled += 3.0 * DT
    for _ in range(40):
      d.update(msg(IDLE, 0.0, sign_id=0), 3.0, DT)
    self.assertEqual(d.phase, Phase.NONE)

  def test_proceeding_ends_after_a_timeout_even_when_crawling(self):
    d = StopSignDisplay()
    d.update(msg(STOPPED, 0.3), 0.0, DT)
    for _ in range(int(16.0 / DT)):
      d.update(msg(IDLE, 0.0, sign_id=0), 0.0, DT)
    self.assertEqual(d.phase, Phase.NONE)

  def test_next_sign_after_proceeding(self):
    d = StopSignDisplay()
    d.update(msg(STOPPED, 0.3), 0.0, DT)
    for _ in range(200):
      d.update(msg(IDLE, 90.0, sign_id=456), 5.0, DT)
    self.assertEqual(d.phase, Phase.TRACKED)

  def test_distance_label_units(self):
    d = StopSignDisplay()
    d.update(msg(COASTING, 30.0), 6.0, DT)
    self.assertEqual(d.label(False), "98 ft")
    self.assertEqual(d.label(True), "30 m")

  def test_real_capnp_message(self):
    # the live message's state is a capnp enum, not an int (this crashed the UI on the car)
    import openpilot.cereal.messaging as messaging
    m = messaging.new_message('longitudinalPlanSP')
    ss = m.longitudinalPlanSP.stopSign
    ss.state = 'braking'
    ss.distance = 30.0
    ss.signId = -1
    d = StopSignDisplay()
    self.assertEqual(d.update(m.longitudinalPlanSP.stopSign, 6.0, DT), Phase.ACTING)
    ss.state = 'idle'
    ss.distance = 100.0
    self.assertEqual(StopSignDisplay().update(m.longitudinalPlanSP.stopSign, 6.0, DT), Phase.TRACKED)

  def test_shadow_flag_passes_through(self):
    d = StopSignDisplay()
    d.update(msg(COASTING, 30.0, shadow=True), 6.0, DT)
    self.assertTrue(d.shadow)


if __name__ == "__main__":
  unittest.main()
