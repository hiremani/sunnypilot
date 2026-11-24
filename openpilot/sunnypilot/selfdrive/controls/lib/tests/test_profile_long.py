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
    """n frames arriving at (v, d, vr): the distance closes at vr from frame to frame, as a real lead's does (the
    lead's readings are smoothed along that)."""
    lb = lb or pl.LeadBrake(self.DT)
    out = None
    for i in range(n):
      out = lb.update(a, self.GAP, v, True, d - vr * self.DT * (n - 1 - i), vr)
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
    n = round(pl.LEAD_BRAKE_ON_S / self.DT)
    for i in range(n - 1):
      self.assertIsNone(lb.update(0.59, self.GAP, v, True, d - vr * self.DT * (n - 1 - i), vr))   # flicker: nothing
    self.assertIsNotNone(lb.update(0.59, self.GAP, v, True, d, vr))

  def test_following_inside_the_gap_is_left_to_the_mpc(self):
    # route 000000ea: 72 mph, a car 40 m ahead closing at 3.5 m/s, inside the driver's 1.69 s gap: no early slowdown
    out, _ = self._run(0.59, 72 * CV.MPH_TO_MS, 40.0, -3.5)
    self.assertIsNone(out)
    out, _ = self._run(0.59, 32 * CV.MPH_TO_MS, 22.0, -10.0)                       # already needs > START_MAX: MPC's
    self.assertIsNone(out)

  def test_route_00000129_as_recorded_starts_within_two_seconds(self):
    # The stop as recorded (2026-11-20, 3:42-3:54; data/lead_00000129_stopped_car.json: s before the stop, v_ego,
    # lead dRel, vRel at 20 Hz), the day before LeadBrake existed. The comma has no radar on this car: the vision lead
    # slid from the car being followed to one stopped at a red light, its distance reading 76, 64, 73, 63, 65 m in
    # turn, then dropping 58 -> 28 m in 2 s. The MPC alone asked -1.0 for 2 s, then -2.2, the driver on the brake
    # 5 mph from the car. Fed those frames, the slowdown has to start within about 2 s of the car reading as closing.
    import json
    import os
    with open(os.path.join(os.path.dirname(__file__), "data", "lead_00000129_stopped_car.json")) as f:
      rows = json.load(f)
    lb = pl.LeadBrake(self.DT)
    started = None
    for t, v, d, vr, present in rows:
      out = lb.update(0.59, 1.45, v, bool(present), d, vr)
      if out is not None and started is None:
        started = t
    self.assertIsNotNone(started, "never started slowing")
    self.assertLess(started, -9.0)            # the stopped car reads as closing from about -11 s

  def test_a_new_lead_starts_the_smoothing_again(self):
    lb = pl.LeadBrake(self.DT)
    for _ in range(40):
      lb.update(0.59, self.GAP, 14.0, True, 23.0, 0.0)                              # following at 23 m
    lb.update(0.59, self.GAP, 14.0, True, 70.0, -10.0)                              # another car: stopped, 70 m on
    self.assertAlmostEqual(lb.d_f, 70.0)
    self.assertAlmostEqual(lb.v_f, -10.0)

  def test_slows_a_little_more_than_the_even_need(self):
    tgt, _ = self._run(0.59, 14.0, 90.0, -10.0)
    need = pl.lead_brake_need(self.GAP, 14.0, 90.0, -10.0, pl.LEAD_BRAKE_LAG_S)
    self.assertAlmostEqual(-tgt[1], need * pl.LEAD_BRAKE_MARGIN, places=2)
    self.assertLess(need * pl.LEAD_BRAKE_MARGIN, pl.LEAD_BRAKE_A_MAX)

  def test_gentle_closing_no_lead_or_far(self):
    self.assertIsNone(self._run(0.59, 12.0, 40.0, -3.0)[0])                        # < 4 m/s closing
    lb = pl.LeadBrake(self.DT)
    for _ in range(20):
      self.assertIsNone(lb.update(0.59, self.GAP, 12.0, False, 40.0, -8.0))        # no lead
    self.assertIsNone(self._run(0.59, 30.0, 200.0, -15.0)[0])                      # beyond the radar range used

  def test_holds_once_slowing_and_releases_when_closed_up(self):
    on, lb = self._run(0.59, 14.0, 70.0, -10.0)
    self.assertIsNotNone(on)
    # closing down to 2.5 m/s: a fresh start would not, but it keeps slowing
    self.assertIsNotNone(lb.update(0.59, self.GAP, 8.0, True, 27.0, -2.5))
    self.assertIsNone(self._run(0.59, 8.0, 27.0, -2.5, n=30)[0])
    # closed up (closing < 1.5 m/s): the MPC follows
    off, lb = self._run(0.59, 7.0, 18.0, -1.0, n=40, lb=lb)
    self.assertIsNone(off)
    self.assertFalse(lb.active)

  def test_never_asks_more_than_the_cap_and_never_speeds_up(self):
    for d in (20.0, 40.0, 80.0, 120.0):
      for vr in (-4.5, -6.0, -10.0, -15.0):
        tgt, _ = self._run(0.3, 16.0, d, vr)
        if tgt is not None:
          self.assertLessEqual(tgt[0], 16.0)
          self.assertGreaterEqual(tgt[1], -pl.LEAD_BRAKE_A_MAX)


class TestLeadTakeOff(unittest.TestCase):
  DT = 0.05

  def _run(self, lead, v_ego=0.0, launch=1.0):
    """lead: [(d, v_lead, a_lead)] per frame, the car standing; the request per frame."""
    t = pl.LeadTakeOff()
    return [t.update(v_ego, True, d, v, a, launch, self.DT) for d, v, a in lead]

  def test_standing_lead_asks_nothing(self):
    self.assertTrue(all(a is None for a in self._run([(7.0, 0.02, 0.0)] * 100)))
    t = pl.LeadTakeOff()
    self.assertIsNone(t.update(0.0, False, 0.0, 0.0, 0.0, 1.0, self.DT))

  def test_route_00000129_goes_as_the_lead_rolls(self):
    # the lead as recorded from 154.1 s (2 Hz samples, held): the old plan left its stop at 156.1 asking +0.10
    rec = [(6.8, 0.0), (7.0, 0.12), (6.8, 0.31), (7.4, 0.53), (8.0, 0.98), (7.6, 1.30), (8.0, 1.44), (9.9, 2.01), (10.3, 2.46), (12.3, 3.26)]
    out = self._run([(d, v, 0.5) for d, v in rec for _ in range(10)])
    first = next(i for i, a in enumerate(out) if a is not None) * self.DT
    self.assertLess(first, 1.5)                     # by 155.6 s: 0.5 s sooner
    at_2s = out[int(2.0 / self.DT)]                 # 156.1 s, where the old plan asked +0.10
    self.assertGreater(at_2s, 0.5)
    self.assertLessEqual(max(a for a in out if a is not None), 1.0)   # never over the launch

  def test_a_creeping_or_braking_lead_and_a_close_one(self):
    self.assertTrue(all(a is None for a in self._run([(7.0, 0.2, 0.0)] * 60)))          # creeping under the threshold
    self.assertTrue(all(a is None for a in self._run([(7.0, 1.0, -0.8)] * 60)))         # rolling but braking again
    self.assertTrue(all(a is None for a in self._run([(3.0, 1.5, 0.5)] * 60)))          # too close
    out = self._run([(7.0, 1.5, 0.5)] * 20 + [(8.0, 0.1, -1.0)] * 5)                    # it stops again: nothing at once
    self.assertIsNotNone(out[19])
    self.assertIsNone(out[20])

  def test_fades_as_this_car_catches_the_leads_speed_and_ends(self):
    t = pl.LeadTakeOff()
    for _ in range(10):
      t.update(0.0, True, 8.0, 2.0, 0.5, 1.0, self.DT)
    self.assertAlmostEqual(t.update(0.0, True, 8.0, 2.0, 0.5, 1.0, self.DT), 1.0)
    self.assertAlmostEqual(t.update(1.0, True, 9.0, 2.0, 0.5, 1.0, self.DT), 0.6)
    # the following gap: 1.45 s of this car's speed over the standing gap, or it asks nothing; it builds over 2 m
    self.assertIsNone(t.update(2.0, True, 6.0, 4.0, 0.5, 1.0, self.DT))              # 6.0 m < 3.5 + 2.9
    self.assertAlmostEqual(t.update(2.0, True, 7.4, 4.0, 0.5, 1.0, self.DT), 0.5)   # 1 m of room: half the pull
    self.assertAlmostEqual(t.update(2.0, True, 9.0, 4.0, 0.5, 1.0, self.DT), 1.0)
    self.assertIsNone(t.update(1.0, True, 9.0, 4.0, 0.5, 1.0, self.DT, gap_s=2.0, stop_gap_m=8.0))   # a longer gap asked
    self.assertIsNone(t.update(1.9, True, 9.5, 2.0, 0.5, 1.0, self.DT))      # as fast as the lead: the gap no longer opens
    self.assertIsNone(t.update(2.6, True, 12.0, 6.0, 0.5, 1.0, self.DT))     # rolling: the MPC's from here
    self.assertAlmostEqual(t.update(0.0, True, 8.0, 3.0, 0.5, 0.7, self.DT), 0.7)   # the driver's own launch


class TestLeadFollowCap(unittest.TestCase):
  """The driver's rule: follow the car in front, up to 10 mph over the posted limit; faster than that, let it go."""
  DT = 0.05
  LIMIT = 35 * CV.MPH_TO_MS

  def _settle(self, cap, v_ego, d, v_lead, n=40, limit=None, present=True):
    out = None
    for _ in range(n):
      out = cap.update(self.LIMIT if limit is None else limit, v_ego, present, d, v_lead - v_ego, self.DT)
    return out

  def test_follows_a_car_a_few_mph_over_the_limit(self):
    cap = pl.LeadFollowCap()
    v = self._settle(cap, 36 * CV.MPH_TO_MS, 25.0, 41 * CV.MPH_TO_MS)
    self.assertAlmostEqual(v, 41 * CV.MPH_TO_MS + pl.FOLLOW_LEAD_MARGIN, places=3)   # may rise to the lead's speed

  def test_lets_a_car_more_than_ten_over_go(self):
    cap = pl.LeadFollowCap()
    v = self._settle(cap, 44 * CV.MPH_TO_MS, 30.0, 52 * CV.MPH_TO_MS)
    self.assertAlmostEqual(v, 45 * CV.MPH_TO_MS, places=3)                           # limit + 10, no further

  def test_nothing_without_a_limit_a_lead_or_with_a_far_one(self):
    self.assertIsNone(self._settle(pl.LeadFollowCap(), 16.0, 25.0, 18.0, limit=0.0))
    self.assertIsNone(self._settle(pl.LeadFollowCap(), 16.0, 25.0, 18.0, present=False))
    self.assertIsNone(self._settle(pl.LeadFollowCap(), 16.0, 120.0, 18.0))             # 7.5 s ahead: not being followed
    self.assertIsNotNone(self._settle(pl.LeadFollowCap(), 3.0, 30.0, 6.0))             # slow, 30 m: followed

  def test_comes_down_gently_when_the_lead_goes(self):
    cap = pl.LeadFollowCap()
    v_ego = 40 * CV.MPH_TO_MS
    top = self._settle(cap, v_ego, 25.0, 41 * CV.MPH_TO_MS)
    one = cap.update(self.LIMIT, v_ego, False, 0.0, 0.0, self.DT)
    self.assertIsNotNone(one)                                                        # no step back to the cap
    self.assertLess(top - one, 0.6)
    prev = one
    for _ in range(400):
      v_ego = max(v_ego - pl.PROFILE_COAST_DECEL * self.DT, 30 * CV.MPH_TO_MS)
      cur = cap.update(self.LIMIT, v_ego, False, 0.0, 0.0, self.DT)
      if cur is None:
        break
      self.assertLessEqual(prev - cur, 0.6)                                          # eased, never dropped
      prev = cur

  def test_a_slower_lead_lowers_it_gently_too(self):
    cap = pl.LeadFollowCap()
    self._settle(cap, 40 * CV.MPH_TO_MS, 25.0, 42 * CV.MPH_TO_MS)
    a = cap.update(self.LIMIT, 40 * CV.MPH_TO_MS, True, 25.0, -3.0, self.DT)       # the lead now 3 m/s slower
    self.assertGreater(a, 40 * CV.MPH_TO_MS - 3.0)                                  # the MPC brakes for it, not this cap
