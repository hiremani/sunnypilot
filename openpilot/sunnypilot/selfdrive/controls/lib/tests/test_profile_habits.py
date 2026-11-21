"""The driver-profile habits added 2026-11-20 and the assists that read them: the stop gap behind a car (MPC), the
open-road pull (comfort cap), the lane-change delay (auto lane change), the bump crossing speed (Speed Bump Assist), the
turn-in wheel rate and the moving turn start (turn planner). Every one is off at the profile's default."""
import math
import unittest
from types import SimpleNamespace as N

from openpilot.sunnypilot.selfdrive.controls.lib import driver_profile as dp
from openpilot.sunnypilot.selfdrive.controls.lib import profile_long as pl
from openpilot.sunnypilot.selfdrive.controls.lib import speed_bump as sb
from openpilot.sunnypilot.selfdrive.controls.lib import turn_planner as tp


class TestDefaultsAreOff(unittest.TestCase):
  def test_defaults_change_nothing(self):
    p = dp.DriverProfile()
    self.assertIsNone(pl.stop_gap(p.stop_gap_m))
    self.assertEqual(pl.comfort_accel_cap(15.0, p), pl.comfort_accel_cap(15.0))
    self.assertEqual(p.lane_change_delay_s, 0.0)
    self.assertEqual(p.bump_cross_mph, 0.0)
    self.assertEqual(p.turn_in_deg_s, 0.0)
    self.assertEqual(p.turn_start_lead_s, tp.TurnTuning().lead_s)   # the planner's own tuned lead

  def test_bounds(self):
    self.assertEqual(dp.clamp("bump_cross_mph", 30.0), 15.0)
    self.assertEqual(dp.clamp("turn_in_deg_s", 500.0), 120.0)
    self.assertEqual(dp.clamp("turn_start_lead_s", 5.0), 1.5)


class TestStopGap(unittest.TestCase):
  def test_floor(self):
    self.assertEqual(pl.stop_gap(4.0), 4.0)
    self.assertEqual(pl.stop_gap(2.5), pl.STOP_GAP_MIN)             # never closer than 4 m
    self.assertIsNone(pl.stop_gap(0.0))

  def test_mpc_stops_that_far_behind(self):
    from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, STOP_DISTANCE
    mpc = LongitudinalMpc()
    lead = N(present=True, status=True, dRel=20.0, vLead=0.0, aLeadK=0.0, aLeadTau=1.5, prob=1.0, modelProb=1.0)
    none = N(present=False, status=False, dRel=0.0, vLead=0.0, aLeadK=0.0, aLeadTau=1.5, prob=0.0, modelProb=0.0)
    rs = N(leadOne=lead, leadTwo=none)
    mpc.set_cur_state(5.0, 0.0)
    mpc.update(rs)
    base = mpc.params[0, 2]
    mpc.update(rs, stop_distance=4.0)
    self.assertAlmostEqual(mpc.params[0, 2] - base, STOP_DISTANCE - 4.0)
    mpc.update(rs, stop_distance=1.0)                                # clamped inside the MPC too
    self.assertAlmostEqual(mpc.params[0, 2] - base, STOP_DISTANCE - 4.0)


class TestCruiseAccel(unittest.TestCase):
  def test_driver_pull_by_speed(self):
    p = dp.DriverProfile(cruise_accel_town=0.65, cruise_accel_mid=0.43, cruise_accel_fast=0.27)
    self.assertAlmostEqual(pl.comfort_accel_cap(10.0, p), 0.65)
    self.assertAlmostEqual(pl.comfort_accel_cap(24.6, p), 0.27)
    self.assertLess(pl.comfort_accel_cap(24.6, p), pl.comfort_accel_cap(24.6))   # gentler than the table at speed
    partial = dp.DriverProfile(cruise_accel_town=0.65)
    self.assertEqual(pl.comfort_accel_cap(24.6, partial), pl.comfort_accel_cap(24.6))   # all three, or the table
    self.assertGreaterEqual(pl.comfort_accel_cap(24.6, dp.DriverProfile(cruise_accel_town=0.1, cruise_accel_mid=0.1,
                                                                       cruise_accel_fast=0.1)), pl.CRUISE_ACCEL_MIN)


class TestBumpCrossing(unittest.TestCase):
  def test_scales_every_kind_with_a_cap(self):
    a = sb.SpeedBumpAssist.__new__(sb.SpeedBumpAssist)
    a.cross_factor = 1.0
    a.set_profile(16.0)
    self.assertAlmostEqual(a.cross_factor, 1.6)
    a.set_profile(0.0)
    self.assertEqual(a.cross_factor, 1.0)
    # what update() does with a bump ahead
    a.set_profile(15.0)
    hump = sb.SpeedBump(1, 0.0, 0.0, "hump").v_cross
    self.assertAlmostEqual(min(hump * a.cross_factor, sb.CROSS_MAX_MPH * 0.44704), 15.0 * 1.5 * 0.44704, places=3)
    cushion = sb.SpeedBump(2, 0.0, 0.0, "cushion").v_cross
    self.assertAlmostEqual(min(cushion * a.cross_factor, sb.CROSS_MAX_MPH * 0.44704), sb.CROSS_MAX_MPH * 0.44704)


class TestLaneChangeDelay(unittest.TestCase):
  def _alc(self, mode, profile_delay):
    from openpilot.sunnypilot.selfdrive.controls.lib import auto_lane_change as alc
    c = alc.AutoLaneChangeController.__new__(alc.AutoLaneChangeController)
    c.lane_change_set_timer = mode
    c.profile_delay = profile_delay
    c.lane_change_wait_timer = 0.0
    c.lane_change_bsm_delay = False
    c.update_lane_change_timers(False)
    return c.lane_change_delay, alc

  def test_replaces_a_timed_delay_only(self):
    from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeMode as M
    self.assertEqual(self._alc(M.TWO_SECONDS, 0.4)[0], 0.4)
    self.assertEqual(self._alc(M.TWO_SECONDS, 0.0)[0], 2.0)
    self.assertEqual(self._alc(M.NUDGE, 0.4)[0], 0.0)               # nudge stays nudge
    self.assertEqual(self._alc(M.NUDGELESS, 0.4)[0], 0.05)          # and nudgeless stays nudgeless


class TestTurnIn(unittest.TestCase):
  def test_slower_turn_in_builds_slower_and_starts_earlier(self):
    v = 4.0
    fast = tp.comfort_curvature_rate(v)
    slow = tp.comfort_curvature_rate(v, turn_in_deg_s=66.0)
    self.assertLess(slow, fast)
    self.assertEqual(tp.comfort_curvature_rate(v, unwind=True, turn_in_deg_s=66.0), tp.comfort_curvature_rate(v, unwind=True))
    a = tp.TurnPlanner(road_source=object())
    b = tp.TurnPlanner(road_source=object())
    b.tuning.turn_in_wheel_deg_s = 66.0
    self.assertGreater(b.start_distance(10.0, 90.0, v), a.start_distance(10.0, 90.0, v))

  def test_start_lead_moves_the_start(self):
    a = tp.TurnPlanner(road_source=object())
    b = tp.TurnPlanner(road_source=object())
    b.tuning.lead_s = a.tuning.lead_s + 1.0
    self.assertAlmostEqual(b.start_distance(10.0, 90.0, 4.0) - a.start_distance(10.0, 90.0, 4.0), 4.0, places=6)
    self.assertTrue(math.isfinite(b.start_distance(10.0, 90.0, 4.0)))


if __name__ == "__main__":
  unittest.main()


class TestRemainingHabits(unittest.TestCase):
  def test_defaults_are_the_assists_own(self):
    from openpilot.sunnypilot.selfdrive.controls.lib import map_caution as mc
    from openpilot.sunnypilot.selfdrive.controls.lib import wet_road as wr
    p = dp.DriverProfile()
    t = tp.TurnTuning()
    self.assertEqual((p.turn_radius_left_m, p.turn_radius_right_m), (t.radius_left_m, t.radius_right_m))
    self.assertAlmostEqual(p.turn_arm_mph * 0.44704, tp.V_ARM_MAX, places=2)
    self.assertEqual(p.signal_coast_s, mc.SIGNAL_S)
    self.assertEqual(p.wet_gap_factor, wr.GAP_FACTOR[wr.WET])
    self.assertEqual(p.wet_offset_mph, wr.SPEED_OFFSET_MPH[wr.WET])
    self.assertEqual(p.curve_entry_decel, 1.0)

  def test_wet_road_uses_the_driver_when_wet_never_less_in_heavy_rain(self):
    from openpilot.sunnypilot.selfdrive.controls.lib import wet_road as wr
    w = wr.WetRoad.__new__(wr.WetRoad)
    w.applied = wr.WET
    w.set_profile(-5.8, 1.29)
    self.assertEqual(w.speed_offset_mph(), -5.8)
    self.assertEqual(w.gap_factor(), 1.29)
    w.applied = wr.HEAVY
    self.assertEqual(w.speed_offset_mph(), -5.8)                     # min(-5.8, -5)
    self.assertEqual(w.gap_factor(), wr.GAP_FACTOR[wr.HEAVY])        # max(1.29, 1.5)
    w.applied = wr.DRY
    self.assertEqual(w.gap_factor(), 1.0)
    w.applied = wr.WET
    w.set_profile(None, None)
    self.assertEqual(w.gap_factor(), wr.GAP_FACTOR[wr.WET])

  def test_signal_coast_window(self):
    from openpilot.sunnypilot.selfdrive.controls.lib import map_caution as mc
    m = mc.MapCaution.__new__(mc.MapCaution)
    m.signal_s = mc.SIGNAL_S
    m.set_profile(6.4)
    self.assertEqual(m.signal_s, 6.4)

  def test_curve_entry_decel(self):
    from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import vision_controller as vc
    import numpy as np
    c = vc.SmartCruiseControlVision.__new__(vc.SmartCruiseControlVision) if hasattr(vc, "SmartCruiseControlVision") else None
    if c is None:
      self.skipTest("no vision controller class")
    c.set_entry_decel(1.09)
    a = np.interp(3.0, vc._ENTERING_SMOOTH_DECEL_BP, [vc._ENTERING_SMOOTH_DECEL_V[0], -c.entry_decel])
    self.assertAlmostEqual(a, -1.09)

  def test_turn_arm_speed(self):
    t = tp.TurnTuning()
    self.assertEqual(t.v_arm_max, tp.V_ARM_MAX)
    t.v_arm_max = 30.0
    self.assertLessEqual(min(t.v_arm_max, tp.V_ARM_APPROACH), tp.V_ARM_APPROACH)   # never above the approach speed


class TestMerge(unittest.TestCase):
  def test_merging_is_never_capped_by_the_gentle_cruise_pull(self):
    p = dp.DriverProfile(cruise_accel_town=0.65, cruise_accel_mid=0.43, cruise_accel_fast=0.27)
    v, limit = 18.0, 29.0     # 40 mph onto a 65 mph freeway
    self.assertTrue(pl.merging(limit, v))
    self.assertFalse(pl.merging(limit, 27.0))             # up to speed
    self.assertFalse(pl.merging(15.6, 8.0))               # a 35 mph street: not a merge
    self.assertAlmostEqual(pl.comfort_accel_cap(v, p, merge=True), pl.comfort_accel_cap(v))     # the table's
    self.assertGreater(pl.comfort_accel_cap(v, p, merge=True), pl.comfort_accel_cap(v, p))
    self.assertAlmostEqual(pl.comfort_accel_cap(v, dp.DriverProfile(merge_accel=0.85), merge=True), 0.85)
