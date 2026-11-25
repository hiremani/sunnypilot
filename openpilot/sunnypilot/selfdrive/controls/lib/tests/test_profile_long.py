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

  def _run(self, a, v, d, vr, n=40, lb=None, a_lead=0.0, prob=1.0):
    """n frames arriving at (v, d, vr): the distance closes at vr from frame to frame, as a real lead's does (the
    lead's readings are smoothed along that)."""
    lb = lb or pl.LeadBrake(self.DT)
    out = None
    for i in range(n):
      out = lb.update(a, self.GAP, v, True, d - vr * self.DT * (n - 1 - i), vr, a_lead, prob)
    return out, lb

  def _recorded(self, name):
    import json
    import os
    with open(os.path.join(os.path.dirname(__file__), "data", name)) as f:
      return json.load(f)

  def test_need(self):
    # stopped car 60 m ahead, closing at 10 m/s: room = 60 - 6 = 54 m -> 100 / 108
    self.assertAlmostEqual(pl.lead_brake_need(self.GAP, 10.0, 60.0, -10.0), 100.0 / 108.0, places=3)
    self.assertEqual(pl.lead_brake_need(self.GAP, 10.0, 60.0, +1.0), 0.0)            # pulling away
    self.assertEqual(pl.lead_brake_need(self.GAP, 10.0, 5.0, -10.0), float("inf"))   # no room left

  def test_stop_need(self):
    # a stopped car 60 m ahead, 6 m to keep: 100 / (2 * 54)
    self.assertAlmostEqual(pl.lead_brake_stop_need(10.0, 60.0, 0.0, 1.5, 6.0), 100.0 / 108.0, places=3)
    # a car at 6 m/s braking at 2: it rolls another 9 m
    self.assertAlmostEqual(pl.lead_brake_stop_need(10.0, 60.0, 6.0, 2.0, 6.0), 100.0 / 126.0, places=3)
    self.assertEqual(pl.lead_brake_stop_need(10.0, 6.2, 0.0, 1.5, 6.0), float("inf"))

  def test_off_without_the_profile_value(self):
    self.assertIsNone(self._run(0.0, 14.0, 70.0, -14.0)[0])

  def test_stopped_car_gets_the_need_and_a_little_more_at_once(self):
    # 35 mph, a stopped car 100 m ahead: the even need is about 1.4; the request is there within about a second
    v, d = 15.6, 100.0
    out, lb = self._run(0.59, v, d, -v, n=40)
    need = pl.lead_brake_stop_need(v, d, 0.0, pl.LEAD_STOPPED_DECEL, pl.STOP_DISTANCE, pl.LEAD_BRAKE_LAG_S)
    self.assertGreater(need, 1.2)
    self.assertAlmostEqual(-out, min(need * pl.LEAD_BRAKE_MARGIN, need + pl.LEAD_BRAKE_MARGIN_MAX), places=2)

  def test_builds_at_the_jerk_limit_from_what_the_planner_asks(self):
    lb = pl.LeadBrake(self.DT)
    outs = [lb.update(0.59, self.GAP, 15.6, True, 100.0 - 15.6 * self.DT * i, -15.6, 0.0, 1.0, 0.2) for i in range(30)]
    first = next(i for i, o in enumerate(outs) if o is not None)
    self.assertEqual(first, round(pl.LEAD_BRAKE_ON_S / self.DT) - 1)               # held for LEAD_BRAKE_ON_S first
    self.assertAlmostEqual(outs[first], -pl.LEAD_BRAKE_JERK * self.DT, places=3)   # from 0, not from the +0.2 asked
    for a, b in zip(outs[first:], outs[first + 1:], strict=False):
      self.assertLessEqual(abs(b - a), pl.LEAD_BRAKE_JERK * self.DT + 1e-6)

  def test_route_00000144_queue_at_a_light(self):
    # 2026-11-24, 8:22-8:33 (data/lead_00000144_queue_at_light.json: s before the stop, v_ego, lead dRel, vRel, aLeadK,
    # modelProb, present, the request then, driver braking; 20 Hz). 35 mph, a queue at a red light first seen at
    # 112 m, read as 19, 10, 5 mph. The request then went 0.7 -> 1.6 over 4 s, behind the need all the way; the driver
    # braked at 41 m and 26 mph, at about 3 m/s^2. Asked for at once it is an even ~1.3.
    rows = self._recorded("lead_00000144_queue_at_light.json")
    lb = pl.LeadBrake(self.DT)
    started = asked_at = None
    for t, v, d, vr, a_lead, prob, present, cmd, _ in rows:
      out = lb.update(0.59, 1.69, v, bool(present), d, vr, a_lead, prob, cmd)
      if out is not None and started is None:
        started = t
      if out is not None and out <= -1.2 and asked_at is None:
        asked_at = t
    drive_asked_at = next(t for t, *_, cmd, _ in rows if cmd <= -1.2)
    self.assertIsNotNone(started, "never started slowing")
    self.assertLess(started, -11.0)                    # from the first second the queue is seen
    self.assertLess(asked_at, drive_asked_at - 1.0)    # 1.2 m/s^2 asked a second or more before the drive did

  def test_route_00000144_car_slowing_to_a_stop(self):
    # the same drive at 5:06-5:19: 40 mph, the car ahead slowing to a stop for a light, vision reading it at 30, 13, 23,
    # 15, 22, 7, 2 mph. The drive asked 0.3 until 71 m, then ramped to 2.0; the driver braked at 26 m (4 m/s^2).
    rows = self._recorded("lead_00000144_slowing_to_a_stop.json")
    lb = pl.LeadBrake(self.DT)
    started = None
    most_before_driver = 0.0
    for t, v, d, vr, a_lead, prob, present, cmd, _ in rows:
      out = lb.update(0.59, 1.69, v, bool(present), d, vr, a_lead, prob, cmd)
      if out is not None and started is None:
        started = t
      if t < -7.0 and out is not None:        # the driver braked at -5 s
        most_before_driver = max(most_before_driver, -out - max(-cmd, 0.0))
    self.assertIsNotNone(started)
    self.assertLess(started, -10.0)
    self.assertGreater(most_before_driver, 0.4)   # clearly more braking than the drive asked, seconds earlier

  def test_route_00000129_as_recorded_starts_within_two_seconds(self):
    # The stop as recorded (2026-11-20, 3:42-3:54; data/lead_00000129_stopped_car.json: s before the stop, v_ego,
    # lead dRel, vRel at 20 Hz). The vision lead slid from the car being followed to one stopped at a red light, its
    # distance reading 76, 64, 73, 63, 65 m in turn, then dropping 58 -> 28 m in 2 s.
    lb = pl.LeadBrake(self.DT)
    started = None
    for t, v, d, vr, present in self._recorded("lead_00000129_stopped_car.json"):
      out = lb.update(0.59, 1.45, v, bool(present), d, vr)
      if out is not None and started is None:
        started = t
    self.assertIsNotNone(started, "never started slowing")
    self.assertLess(started, -9.0)            # the stopped car reads as closing from about -11 s

  def test_following_inside_the_gap_is_left_to_the_mpc(self):
    # route 000000ea: 72 mph, a car 40 m ahead closing at 3.5 m/s, inside the driver's 1.69 s gap: no early slowdown
    self.assertIsNone(self._run(0.59, 72 * CV.MPH_TO_MS, 40.0, -3.5)[0])
    # 00000147 at 6:08: 46 mph, 36 m behind a car doing 42, which eases off at 1.2 m/s^2: following, not a stop
    self.assertIsNone(self._run(0.59, 20.8, 36.0, -1.9, a_lead=-1.2)[0])

  def test_moving_car_closing_fast_slows_early_but_gently(self):
    # 32 mph, a car doing 10 mph 70 m ahead: slows early, never more than the moving-car cap and its margin
    v = 32.0 * CV.MPH_TO_MS
    out, _ = self._run(0.59, v, 70.4, -10.0)
    self.assertIsNotNone(out)
    self.assertLess(out, -0.59)
    self.assertGreaterEqual(out, -(pl.LEAD_BRAKE_A_MOVE_MAX + pl.LEAD_BRAKE_MARGIN_MAX))
    self.assertIsNone(self._run(0.59, v, 145.0, -10.0)[0])                         # further out: not yet

  def test_braking_car_already_much_slower_is_taken_to_stop(self):
    # 40 mph, a car at 20 mph 80 m ahead: steady, the moving-car need (small); braking at 2 m/s^2, it will stand
    v = 17.9
    steady, _ = self._run(0.59, v, 80.0, -9.0)
    braking, _ = self._run(0.59, v, 80.0, -9.0, a_lead=-2.0)
    self.assertLess(braking, -1.2)
    self.assertTrue(steady is None or steady > braking + 0.4)

  def test_uncertain_vision_lead_and_slow_speeds_are_left_alone(self):
    self.assertIsNone(self._run(0.59, 15.6, 100.0, -15.6, prob=0.3)[0])
    self.assertIsNone(self._run(0.59, 2.5, 12.0, -2.5)[0])                         # the last metres are the MPC's
    lb = pl.LeadBrake(self.DT)
    for _ in range(20):
      self.assertIsNone(lb.update(0.59, self.GAP, 12.0, False, 40.0, -8.0))        # no lead
    self.assertIsNone(self._run(0.59, 30.0, 200.0, -15.0)[0])                      # beyond the range used

  def test_a_new_lead_starts_the_smoothing_again(self):
    lb = pl.LeadBrake(self.DT)
    for _ in range(40):
      lb.update(0.59, self.GAP, 14.0, True, 23.0, 0.0)                              # following at 23 m
    lb.update(0.59, self.GAP, 14.0, True, 70.0, -10.0)                              # another car: stopped, 70 m on
    self.assertAlmostEqual(lb.d_f, 70.0)
    self.assertAlmostEqual(lb.v_f, -10.0)

  def test_holds_once_slowing_and_releases_when_closed_up(self):
    on, lb = self._run(0.59, 14.0, 70.0, -10.0)
    self.assertIsNotNone(on)
    self.assertIsNotNone(lb.update(0.59, self.GAP, 8.0, True, 27.0, -2.5))          # still closing: keeps slowing
    off, lb = self._run(0.59, 7.0, 18.0, -0.5, n=60, lb=lb)                         # closed up: the MPC follows
    self.assertIsNone(off)
    self.assertFalse(lb.active)

  def test_fades_as_it_closes_on_a_stopped_car(self):
    # driven in closed loop on a stopped car from 35 mph and 100 m: the braking is highest early and under the
    # soft-stop pass level (1.1 m/s^2) by 3 m/s, where the MPC takes the last metres
    lb = pl.LeadBrake(self.DT)
    v, d, a, peak, peak_d = 15.6, 100.0, 0.0, 0.0, 0.0
    while v > pl.LEAD_BRAKE_V_MIN + 0.05:
      out = lb.update(0.59, 1.69, v, True, d, -v, 0.0, 1.0, a)
      self.assertIsNotNone(out) if d < 95.0 else None
      a += ((out or 0.0) - a) * self.DT / 0.4
      v += a * self.DT
      d -= v * self.DT
      if -a > peak:
        peak, peak_d = -a, d
    self.assertLess(peak, 1.9)
    self.assertGreater(peak_d, 50.0)          # the hardest braking is in the first half
    self.assertLess(-a, 1.1)
    self.assertGreater(d, 7.0)                # with room left for the last metres

  def test_never_more_than_the_cap(self):
    for d in (12.0, 20.0, 40.0, 80.0, 120.0):
      for vr in (-4.5, -6.0, -10.0, -16.0):
        out, _ = self._run(1.0, 16.0, d, vr)
        if out is not None:
          self.assertLessEqual(out, 0.0)
          self.assertGreaterEqual(out, -pl.LEAD_BRAKE_A_MAX)


class TestLeadTakeOff(unittest.TestCase):
  DT = 0.05
  GAP = 1.69   # the driver's following gap (s)

  def _run(self, lead, v_ego=0.0, a_max=1.0):
    """lead: [(d, v_lead, a_lead)] per frame, the car standing; the request per frame."""
    t = pl.LeadTakeOff()
    return [t.update(v_ego, True, d, v, a, a_max, self.DT, self.GAP) for d, v, a in lead]

  def _drive(self, lead_v, d0=4.5, secs=14.0, lag=0.4):
    """Closed loop behind a lead with speed lead_v(t): a car that gives the request after a lag. Rows of
    (t, v, gap, request, lead speed)."""
    t = pl.LeadTakeOff()
    v = a = s = 0.0
    lead_s, rows, prev = d0, [], lead_v(0.0)
    for i in range(round(secs / self.DT)):
      now = i * self.DT
      vl = lead_v(now)
      lead_s += vl * self.DT
      req = t.update(v, True, lead_s - s, vl, (vl - prev) / self.DT, 1.0, self.DT, self.GAP) or 0.0
      prev = vl
      a += (req - a) * self.DT / lag
      v = max(0.0, v + a * self.DT)
      s += v * self.DT
      rows.append((now, v, lead_s - s, req, vl))
    return rows

  def test_gap_is_the_drivers_while_moving_off_and_the_following_gap_at_the_end(self):
    self.assertAlmostEqual(pl.takeoff_gap(0.0, self.GAP), pl.TAKEOFF_GAP_M - pl.TAKEOFF_STAND_TOL_M)   # standing: a little less
    self.assertAlmostEqual(pl.takeoff_gap(5.0, self.GAP), 4.0 + 1.3 * 5.0)                             # 11 mph: 10.5 m
    self.assertAlmostEqual(pl.takeoff_gap(pl.TAKEOFF_END_V, self.GAP), 4.0 + self.GAP * pl.TAKEOFF_END_V)
    self.assertAlmostEqual(pl.takeoff_gap(5.0, 1.0), 4.0 + 1.0 * 5.0)                                  # never more than the following gap

  def test_standing_lead_asks_nothing_however_far(self):
    self.assertTrue(all(a is None for a in self._run([(7.0, 0.02, 0.0)] * 100)))
    self.assertTrue(all(a is None for a in self._run([(13.0, 0.1, 0.0)] * 100)))      # 13 m ahead and standing: no creeping up
    self.assertTrue(all(a is None for a in self._run([(13.0, 0.3, 0.0)] * 100)))      # vision's noise on a standing car
    t = pl.LeadTakeOff()
    self.assertIsNone(t.update(0.0, False, 0.0, 0.0, 0.0, 1.0, self.DT))

  def test_route_00000129_goes_as_the_lead_rolls(self):
    # the lead as recorded from 154.1 s (2 Hz samples, held): the old plan left its stop at 156.1 asking +0.10
    rec = [(6.8, 0.0), (7.0, 0.12), (6.8, 0.31), (7.4, 0.53), (8.0, 0.98), (7.6, 1.30), (8.0, 1.44), (9.9, 2.01), (10.3, 2.46), (12.3, 3.26)]
    out = self._run([(d, v, 0.5) for d, v in rec for _ in range(10)])
    first = next(i for i, a in enumerate(out) if a is not None) * self.DT
    self.assertLess(first, 1.8)                     # 0.2 s after it reads 0.5 m/s
    self.assertGreater(out[int(2.5 / self.DT)], 0.8)
    self.assertLessEqual(max(a for a in out if a is not None), pl.TAKEOFF_A_MAX)

  def test_a_creeping_or_braking_lead_and_a_close_one(self):
    self.assertTrue(all(a is None for a in self._run([(7.0, 0.4, 0.0)] * 60)))          # creeping under the threshold
    self.assertTrue(all(a is None for a in self._run([(7.0, 1.0, -0.8)] * 60)))         # rolling but braking again
    self.assertTrue(all(a is None for a in self._run([(3.0, 1.5, 0.5)] * 60)))          # too close
    out = self._run([(7.0, 1.5, 0.5)] * 20 + [(8.0, 0.0, -1.0)] * 5)                    # it stands again: nothing more
    self.assertIsNotNone(out[19])
    self.assertIsNone(out[20])

  def test_keeps_pulling_past_walking_pace(self):
    # 00000148 at 28:50, the drive before this rule: the take-off asked 1.0 up to 5.6 mph, then the comfort ceiling
    # held it at 0.65 while the lead kept its 1.2-1.5; 9 s on 18 mph, the lead at 26 and 43 m off
    t = pl.LeadTakeOff()
    for _ in range(10):
      t.update(0.0, True, 6.0, 2.0, 1.0, 1.0, self.DT, self.GAP)
    self.assertTrue(t.active)
    self.assertAlmostEqual(t.update(4.0, True, 15.0, 7.0, 1.3, 1.0, self.DT, self.GAP), 1.0)    # 9 mph, lead at 16, 15 m
    self.assertAlmostEqual(t.update(8.0, True, 22.0, 10.0, 1.2, 1.0, self.DT, self.GAP), 1.0)   # 18 mph
    self.assertAlmostEqual(t.update(4.0, True, 15.0, 7.0, 1.3, 0.5, self.DT, self.GAP), 0.5)    # a wet road's ceiling

  def test_never_inside_the_gap_and_never_toward_a_slower_lead(self):
    t = pl.LeadTakeOff()
    for _ in range(10):
      t.update(0.0, True, 6.0, 2.0, 1.0, 1.0, self.DT, self.GAP)
    self.assertIsNone(t.update(4.0, True, 9.0, 7.0, 1.0, 1.0, self.DT, self.GAP))       # 9.0 m < 4 + 1.3 x 4: inside
    self.assertLess(t.update(4.0, True, 9.5, 7.0, 1.0, 1.0, self.DT, self.GAP), 0.5)    # 0.3 m of room: a fraction of it
    self.assertIsNone(t.update(4.0, True, 14.0, 3.0, -1.0, 1.0, self.DT, self.GAP))     # the lead slower: nothing
    self.assertTrue(t.active)

  def test_ends_when_caught_up_or_moved_off_and_starts_only_from_standing(self):
    t = pl.LeadTakeOff()
    for _ in range(10):
      t.update(0.0, True, 6.0, 2.0, 1.0, 1.0, self.DT, self.GAP)
    for _ in range(round(pl.TAKEOFF_SETTLED_S / self.DT) + 1):
      t.update(6.0, True, 14.0, 5.0, 0.0, 1.0, self.DT, self.GAP)                       # the lead 1 m/s slower for a second
    self.assertFalse(t.active)
    for _ in range(20):                                                                 # rolling at 13 mph: not started again
      self.assertIsNone(t.update(6.0, True, 30.0, 9.0, 1.0, 1.0, self.DT, self.GAP))
    for _ in range(10):
      t.update(0.0, True, 6.0, 2.0, 1.0, 1.0, self.DT, self.GAP)
    self.assertIsNone(t.update(pl.TAKEOFF_END_V, True, 60.0, 20.0, 1.0, 1.0, self.DT, self.GAP))   # 30 mph: the MPC's
    self.assertFalse(t.active)

  def test_closed_loop_follows_the_leads_ramp_at_the_gap(self):
    # the lead waits 1 s, then 1.0 m/s^2 to 13 m/s (29 mph): a ramp this car can match
    rows = self._drive(lambda t: min(max(t - 1.0, 0.0) * 1.0, 13.0))
    at7 = next(r for r in rows if r[0] >= 7.0)
    self.assertLess(next(r[0] for r in rows if r[1] > 0.3), 2.5)          # rolling within 1.5 s of it
    self.assertGreater(at7[1], 0.7 * at7[4])                              # 6 s on: most of its speed ...
    for now, v, gap, _, _ in rows:
      if v > 1.0 and v < pl.TAKEOFF_END_V:
        self.assertGreater(gap, pl.takeoff_gap(v, self.GAP) - 0.5)        # ... never inside the gap
        self.assertLess(gap, pl.takeoff_gap(v, self.GAP) + 8.0, now)      # ... and not left behind

  def test_closed_loop_lead_goes_then_stops(self):
    # a queue: the lead does 3 m/s for 3 s and stands again. The request ends as it slows; nothing pulls toward it
    rows = self._drive(lambda t: 3.0 if 1.0 <= t < 4.0 else 0.0, d0=5.0, secs=8.0)
    self.assertEqual(max(r[3] for r in rows if r[0] >= 4.05), 0.0)
    self.assertTrue(all(r[2] > 4.0 for r in rows if r[0] < 4.5))

  def test_route_00000148_lead_drives_off(self):
    # the lead as recorded (data/takeoff_00000148_lead_leaves.json: s from the lead leaving, v_ego, lead dRel, vLead,
    # aLeadK, present, the request then; 20 Hz). Open loop on the recorded frames: the request stays at the ceiling
    # where the drive dropped to 0.65
    import json
    import os
    with open(os.path.join(os.path.dirname(__file__), "data", "takeoff_00000148_lead_leaves.json")) as f:
      rows = json.load(f)
    t = pl.LeadTakeOff()
    asked = [(now, t.update(v, bool(present), d, vl, al, 1.0, self.DT, self.GAP), cmd) for now, v, d, vl, al, present, cmd in rows]
    first = next(now for now, a, _ in asked if a is not None)
    self.assertLess(first, -0.5)                                           # as early as the drive's own (-0.6 s)
    later = [(a or 0.0, cmd) for now, a, cmd in asked if 2.5 <= now <= 8.0]
    self.assertGreater(sum(a for a, _ in later) / len(later), 0.9)         # the drive: 0.65
    self.assertLess(sum(cmd for _, cmd in later) / len(later), 0.7)


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
