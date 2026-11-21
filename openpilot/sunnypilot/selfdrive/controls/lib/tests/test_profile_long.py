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

  def test_lead_coast_covers_pull_away(self):
    # route f5 12.6 min, pulling away: 10.6 mph, lead 8 m ahead at 7.3 mph. With the measured 2.5 s this is coasting:
    # no separate pull-away rule is needed (closing on a slower lead under 15 mph, the driver was on the accelerator
    # only 1-12% of the time at any distance up to 30 m; behind a lead at our speed, down to ~4.5 m, like the MPC)
    v = 10.6 * CV.MPH_TO_MS
    self.assertTrue(pl.lead_coast(2.49, v, True, 8.0, (7.3 - 10.6) * CV.MPH_TO_MS))

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



class TestTurnExit(unittest.TestCase):
  def test_turn_exit_cap(self):
    self.assertEqual(pl.turn_exit_cap(1.0, 0.7), 0.7)
    self.assertEqual(pl.turn_exit_cap(0.0, 0.7), float("inf"))


class TestComfortAccel(unittest.TestCase):
  def test_gentle_in_town(self):
    # route 00000103: openpilot asked for +1.0 after every turn and stop and the car delivered up to +1.44
    for v in (0.0, 3.0, 8.9, 11.2):
      self.assertLessEqual(pl.comfort_accel_cap(v), 0.6)

  def test_opens_up_on_faster_roads(self):
    self.assertAlmostEqual(pl.comfort_accel_cap(25.0), 1.0)
    self.assertGreater(pl.comfort_accel_cap(15.0), 0.6)

class TestJunctionApproach(unittest.TestCase):
  def test_slows_to_turn_speed_before_the_turn_starts(self):
    # route 00000103: armed for a turn 77 m ahead at 20 mph; following the target (braking at most 1.2 m/s^2 with a
    # 1 s lag, as the planner does) the car is at turn speed when the turn planner's arc starts (~12 m out)
    v, d, a, dt, v_turn, peak = 8.9, 77.0, 0.0, 0.05, 2.57, 0.0
    while d > 12.0:
      target = pl.junction_approach_v(d, v_turn, v)
      want = max(min((target - v) / 1.0, 0.0), -1.2)
      a += (want - a) * dt / 1.0
      v = max(v + a * dt, 0.0)
      d -= v * dt
      peak = min(peak, a)
    self.assertLess(v, v_turn + 0.6)
    self.assertGreater(peak, -1.0)   # gentle: the profile's rate, not the planner's floor

  def test_far_away_no_effect(self):
    self.assertGreater(pl.junction_approach_v(100.0, 2.57, 8.9), 10.0)

  def test_at_the_junction_turn_speed(self):
    self.assertAlmostEqual(pl.junction_approach_v(5.0, 2.57, 3.0), 2.57)


if __name__ == "__main__":
  unittest.main()


class TestProfileSpeedCap(unittest.TestCase):
  """Route 0000010d (Warm Springs Blvd): a lower limit read at 38 mph braked at -1.2 m/s^2, and every gas release
  braked again. The cap now coasts down and keeps the driver's chosen speed until the limit changes."""
  DT = 0.05

  def test_coasts_down_to_a_lower_cap(self):
    cap = pl.ProfileSpeedCap()
    v = 17.0
    cap.update(17.9, 17.9, v, False, self.DT)                 # 40 mph limit, cruising
    vs = [cap.update(11.2, 11.2, v, False, self.DT) for _ in range(40)]   # the limit drops to 25 mph
    assert vs[0] > 16.0                                       # no step down to the new cap
    for a, b in zip(vs, vs[1:], strict=False):
      assert a - b <= pl.PROFILE_COAST_DECEL * self.DT + 1e-9  # comes down no faster than coasting
    cap2 = pl.ProfileSpeedCap()
    cap2.update(11.2, 11.2, 11.0, False, self.DT)
    assert cap2.update(11.2, 11.2, 11.0, False, self.DT) == 11.2   # at or below the cap: the cap itself

  def test_driver_choice_above_the_cap_sticks_until_the_limit_changes(self):
    cap = pl.ProfileSpeedCap()
    cap.update(15.0, 15.6, 15.0, False, self.DT)
    cap.update(15.0, 15.6, 18.5, True, self.DT)               # gas: driving 41 mph above the 34 mph cap
    for _ in range(200):
      v = cap.update(15.0, 15.6, 18.5, False, self.DT)        # foot off the gas
    assert v >= 18.5 - 1e-9                                   # no pull back
    v = cap.update(13.0, 13.4, 18.5, False, self.DT)          # a new posted limit: the choice is dropped (coasting)
    assert v < 18.5 + 0.5

  def test_rises_at_once_and_resets_without_a_limit(self):
    cap = pl.ProfileSpeedCap()
    cap.update(11.2, 11.2, 11.0, False, self.DT)
    assert cap.update(17.9, 17.9, 11.0, False, self.DT) == 17.9
    assert cap.update(None, 0.0, 11.0, False, self.DT) is None


class TestLeadBrake(unittest.TestCase):
  GAP = 1.78   # the driver's following gap (s)
  DT = 0.05

  def _run(self, a, v, d, vr, n=20, lb=None):
    lb = lb or pl.LeadBrake(self.DT)
    out = None
    for _ in range(n):
      out = lb.update(a, self.GAP, v, True, d, vr)
    return out, lb

  def test_need(self):
    # stopped car 60 m ahead, closing at 10 m/s: room = 60 - 6 = 54 m -> 100 / 108
    self.assertAlmostEqual(pl.lead_brake_need(self.GAP, 10.0, 60.0, -10.0), 100.0 / 108.0, places=3)
    self.assertEqual(pl.lead_brake_need(self.GAP, 10.0, 60.0, +1.0), 0.0)            # pulling away
    self.assertEqual(pl.lead_brake_need(self.GAP, 10.0, 5.0, -10.0), float("inf"))   # no room left

  def test_route_00000129_closing_fast_starts_slowing_early(self):
    # 2026-11-20, 3.8 min: 32 mph, a car 70 m ahead closing at 10 m/s. openpilot slowed only gently (-1.0 at 52 m) and
    # needed -2.3 at the end, the driver braking at 8 m. With the driver's 0.59 it slows evenly from here.
    v, d, vr = 32.0 * CV.MPH_TO_MS, 70.4, -10.0
    off, _ = self._run(0.0, v, d, vr)
    self.assertIsNone(off)                                                         # profile without the value: off
    tgt, _ = self._run(0.59, v, d, vr)
    self.assertIsNotNone(tgt)
    v_t, a_t = tgt
    self.assertLessEqual(v_t, v)                                                   # only ever slows
    self.assertLess(a_t, -0.59)
    self.assertGreaterEqual(a_t, -pl.LEAD_BRAKE_A_MAX)
    far, _ = self._run(0.59, v, 145.0, vr)
    self.assertIsNone(far)                                                         # further out: not yet

  def test_needs_to_hold_half_a_second(self):
    v, d, vr = 14.0, 70.0, -10.0
    lb = pl.LeadBrake(self.DT)
    for _ in range(round(pl.LEAD_BRAKE_ON_S / self.DT) - 1):
      self.assertIsNone(lb.update(0.59, self.GAP, v, True, d, vr))                 # radar flicker: nothing
    self.assertIsNotNone(lb.update(0.59, self.GAP, v, True, d, vr))

  def test_following_inside_the_gap_is_left_to_the_mpc(self):
    # route 000000ea: 72 mph, a car 40 m ahead closing at 3.5 m/s, inside the driver's 1.69 s gap: no early slowdown
    out, _ = self._run(0.59, 72 * CV.MPH_TO_MS, 40.0, -3.5)
    self.assertIsNone(out)
    out, _ = self._run(0.59, 32 * CV.MPH_TO_MS, 22.0, -10.0)                       # already needs > START_MAX: MPC's
    self.assertIsNone(out)

  def test_gentle_closing_no_lead_or_far(self):
    self.assertIsNone(self._run(0.59, 12.0, 40.0, -3.0)[0])                        # < 4 m/s closing
    lb = pl.LeadBrake(self.DT)
    for _ in range(20):
      self.assertIsNone(lb.update(0.59, self.GAP, 12.0, False, 40.0, -8.0))        # no lead
    self.assertIsNone(self._run(0.59, 30.0, 200.0, -15.0)[0])                      # beyond the radar range used

  def test_holds_once_slowing_and_releases_when_closed_up(self):
    on, lb = self._run(0.59, 14.0, 70.0, -10.0)
    self.assertIsNotNone(on)
    # closing down to 2 m/s: a fresh start would not, but it keeps slowing
    self.assertIsNotNone(lb.update(0.59, self.GAP, 8.0, True, 16.0, -2.0))
    self.assertIsNone(pl.LeadBrake(self.DT).update(0.59, self.GAP, 8.0, True, 16.0, -2.0))
    # closed up (closing < 1.5 m/s): the MPC follows
    self.assertIsNone(lb.update(0.59, self.GAP, 7.0, True, 18.0, -1.0))
    self.assertFalse(lb.active)

  def test_never_asks_more_than_the_cap_and_never_speeds_up(self):
    for d in (20.0, 40.0, 80.0, 120.0):
      for vr in (-4.5, -6.0, -10.0, -15.0):
        tgt, _ = self._run(0.3, 16.0, d, vr)
        if tgt is not None:
          self.assertLessEqual(tgt[0], 16.0)
          self.assertGreaterEqual(tgt[1], -pl.LEAD_BRAKE_A_MAX)
