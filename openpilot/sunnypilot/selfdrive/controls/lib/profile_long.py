"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

How the longitudinal planner drives by the driver profile (driver_profile.py), as small pure functions:

  follow_t(gap_s, v)        the MPC's t_follow that gives the driver's measured following gap. The profile's gap is
                            what was measured, distance / speed behind a steady lead; the MPC keeps
                            t_follow * v + STOP_DISTANCE behind one, so t_follow = gap - STOP_DISTANCE / v, never below
                            openpilot's "aggressive" 1.25 s (and the profile's own bounds cap it at 2.5 s).
  LeadBrake                 closing fast on a slower car: once an even slowdown to the driver's following gap behind it
                            needs the driver's lead_brake_a, a speed target that slows that evenly (it only ever lowers
                            the speed; the MPC's own lead braking stays underneath). Route 00000129 (2026-11-20, 3.8 min):
                            closing at 11 m/s, openpilot was still at -1.0 m/s^2 with the car 52 m ahead, reached -2.3,
                            and the driver braked at 8 m; the driver starts slowing at a median 0.64 m/s^2 of need when
                            closing that fast (30 slowdowns), openpilot at 0.8-1.2 (4).
  lead_coast(...)           True when the lead is slower and inside the driver's lead_coast_s time gap: the planner then
                            stops accelerating (the cap is 0 m/s^2: coasting; braking stays the MPC's). Route
                            000000f5 (2026-11-15): openpilot still accelerated +0.6 at 42 mph toward a lead 79 m ahead
                            closing at 5 mph, and +0.58 toward one 8 m ahead at 7 mph; both times the driver braked.
  junction_turn(...)        a signaled, sharp, slow turn is under way: cap the speed at the driver's junction speed.
  profile_speed(...)        the cruising speed cap: the posted limit + the driver's own offset for that limit.
  turn_exit_cap(...)        for TURN_EXIT_S after a junction turn, pull away no harder than the driver does out of one.
  comfort_accel_cap(v)      the gentlest of all: openpilot never asks for more than this (0.6 m/s^2 in town).
"""
import math

import numpy as np

from openpilot.common.constants import CV

STOP_DISTANCE = 6.0         # m, long_mpc.STOP_DISTANCE
T_FOLLOW_MIN = 1.25         # s, openpilot's "aggressive"
V_FOLLOW_MIN = 5.0          # m/s: below, the stop distance term is not divided by a near-zero speed

LEAD_COAST_CLOSING = 0.5    # m/s: the lead is this much slower (closing) before coasting
LEAD_COAST_MAX_D = 120.0    # m: radar leads further than this are ignored
LEAD_COAST_ACCEL = 0.0      # m/s^2 cap while coasting

LEAD_BRAKE_MIN_CLOSING = 4.0    # m/s: the closing speed lead_brake_a is measured from (driver_profile_gen)
LEAD_BRAKE_START_CLOSING = 2.0  # m/s: slower than this, closing is left to the MPC
LEAD_BRAKE_MIN_ROOM_M = 10.0    # m outside the following gap still left when it starts on a moving car: inside, it is
                                # following (MPC)
LEAD_BRAKE_FILTER_S = 0.5       # s: the lead's distance, speed and acceleration are smoothed over this before the need
                                # is worked out. This car's leads are vision only: the same stopped car read 76, 64, 73,
                                # 63, 65 m and 17.6 then 8.2 mph within a second
LEAD_BRAKE_NEW_LEAD_M = 12.0    # m: a jump this large in one frame is another car: the smoothing starts again from it
LEAD_BRAKE_MARGIN = 1.25        # slow this much more than the even need (at most LEAD_BRAKE_MARGIN_MAX more): the need
LEAD_BRAKE_MARGIN_MAX = 0.3     # then falls as the car closes, so the slowdown ends gentler than it began. Arriving at
                                # the last 5 mph still braking firmly is what makes this car's brakes grab (SOFT_STOP)
LEAD_BRAKE_ON_S = 0.3           # s the start condition must hold (flicker)
LEAD_BRAKE_HOLD_CLOSING = 1.0   # m/s: once slowing, keep going until the closing speed is down to this
LEAD_BRAKE_MAX_D = 150.0        # m
LEAD_BRAKE_A_MAX = 2.5          # m/s^2: never asks for more (the MPC and the car's own emergency braking are underneath)
LEAD_BRAKE_A_MOVE_MAX = 1.5     # m/s^2: the most toward a car that keeps moving
LEAD_BRAKE_LAG_S = 0.6          # s the car goes on before a request is braking (measured: request -> braking)
LEAD_BRAKE_HOLD = 0.3           # once slowing, keep it while the need is at least this fraction of lead_brake_a
LEAD_BRAKE_JERK = 2.5           # m/s^3: how fast the request builds and fades
LEAD_BRAKE_V_MIN = 3.0          # m/s: below, the last metres are the MPC's
LEAD_BRAKE_PROB = 0.5           # a vision lead less certain than this is not braked for early
LEAD_STOPPED_V = 3.0            # m/s: a lead slower than this is taken as stopping (vision reads a stopped car as
                                # 19, 10, 5 mph in its first second: 00000144 at 8:22, seen at 112 m, braked for as if
                                # it kept rolling, the driver on the brake at 41 m)
LEAD_STOPPED_DECEL = 1.5        # m/s^2: how such a lead is taken to stop, at least
LEAD_BRAKING_DECEL = 0.6        # m/s^2: a lead braking at least this much, and already slower than LEAD_BRAKING_SLOW of
LEAD_BRAKING_SLOW = 0.7         # our speed, is taken to keep braking until it stands (00000144 at 5:06)

JUNCTION_V_MAX = 9.0        # m/s (20 mph, the turn planner's abort speed): above, not a junction turn
JUNCTION_CURVATURE = 1 / 25.0   # 1/m (R < 25 m)
JUNCTION_ACCEL = -0.8       # m/s^2 to get down to the junction speed

# Slowing down before a signaled turn the map turn planner is armed for (carControlSP.mapTurn): reach the junction speed
# APPROACH_MARGIN_M before the junction (the turn planner starts its arc ~10-13 m out), braking at JUNCTION_ACCEL's
# rate, leading by the planner's ~1 s speed-loop lag. Route 00000103 (2026-11-16): with openpilot speed control nothing
# slowed the car before a turn; it held 20 mph through 4 of 5 signaled junctions and the turn assist (starts at 16 mph
# at most) passed every one of them.
APPROACH_MARGIN_M = 15.0
APPROACH_LAG_S = 1.0
V_TURN_DEFAULT = 4.5        # m/s (10 mph) without a driver profile


def junction_approach_v(dist_m: float, v_turn: float, v_ego: float) -> float:
  """The speed target (m/s) dist_m before an armed junction turn."""
  d = max(dist_m - APPROACH_MARGIN_M - v_ego * APPROACH_LAG_S, 0.0)
  return (v_turn * v_turn + 2.0 * abs(JUNCTION_ACCEL) * d) ** 0.5


PROFILE_SPEED_ACCEL = -0.5  # m/s^2 to get down to a lower cap (a new, lower limit)

# The speed-by-limit cap only coasts (driver's choice, 2026-11-18). Route 0000010d (Warm Springs Blvd): the car's sign
# reading dropped from 40 to 25 mph and the cap braked at -1.2 m/s^2 from 38 mph; the driver pressed the gas, and on
# every release it braked again (16 gas take-overs in one drive). Now the cap comes down no faster than
# PROFILE_COAST_DECEL from the current speed, and once the driver drives above it with the gas it stays at the speed
# they chose until the posted limit changes.
PROFILE_COAST_DECEL = 0.3   # m/s^2


class ProfileSpeedCap:
  """The driverSpeed target: the profile's cap for the posted limit, coming down at most PROFILE_COAST_DECEL and
  raised to the speed the driver chose with the gas above it (until the limit changes)."""

  def __init__(self):
    self.v = None         # the current (ramped) target, m/s
    self.floor = 0.0      # the driver's chosen speed above the cap, m/s
    self.limit = 0.0      # the posted limit the floor belongs to, m/s

  def update(self, v_cap: float | None, limit_ms: float, v_ego: float, gas: bool, dt: float) -> float | None:
    if v_cap is None:
      self.v, self.floor, self.limit = None, 0.0, 0.0
      return None
    if abs(limit_ms - self.limit) > 0.5:
      self.floor, self.limit = 0.0, limit_ms
    if gas and v_ego > v_cap:
      self.floor = max(self.floor, v_ego)
    want = max(v_cap, self.floor)
    if self.v is None or want >= self.v:
      self.v = want
    else:
      self.v = max(want, min(self.v, v_ego + 0.5) - PROFILE_COAST_DECEL * dt)
    return self.v

FOLLOW_OVER_MPH = 10.0      # following a car, the speed may go this far over the posted limit and no further
FOLLOW_MAX_GAP_S = 4.0      # a car further ahead than this (or FOLLOW_MAX_D) isn't being followed
FOLLOW_MAX_D = 80.0         # m
FOLLOW_MIN_D = 50.0         # m: ...but a car this close is, whatever the time gap (low speeds)
FOLLOW_LEAD_MARGIN = 0.5    # m/s over the lead's own speed the cap sits (the MPC keeps the gap, not this cap)


TAKEOFF_ARM_V = 2.5         # m/s: it starts only from (nearly) standing; from then on it runs until it ends
TAKEOFF_END_V = 13.4        # m/s (30 mph): moved off; the MPC follows from here
TAKEOFF_LEAD_V = 0.5        # m/s: the lead is rolling (a standing lead reads within 0.02 of zero). Over 30 drives this
TAKEOFF_ON_S = 0.2          # fired 58 times standing behind a car, the car really leaving in 54; waiting for the
                            # distance to open as well was 0.3 s later and no surer, the distance alone wrong half the time
# ... or it is slower still but clearly pulling away. At 0.5 m/s alone the rule fired 0.75 s (median) after the lead
# first moved, and the driver was on the accelerator 0.5-1.3 s after it (0000014f 31.4 min, 00000151 1.8 min,
# 2026-11-25). A steady 0.3-0.4 m/s is a creep or vision's noise and still asks nothing.
TAKEOFF_LEAD_EARLY_V = 0.3  # m/s
TAKEOFF_LEAD_EARLY_A = 0.3  # m/s^2
TAKEOFF_LEAD_FULL_V = 2.0   # m/s: the lead's speed from which the open gap counts in full (a standing car never pulls)
TAKEOFF_MIN_D = 3.5         # m: never from closer than this
TAKEOFF_GAP_M = 4.0         # m and s: the gap the driver keeps while moving off behind a car, 4 m + 1.3 s of our speed
TAKEOFF_GAP_S = 1.3         # (32 of their own take-offs over 30 drives: 5.6 m at 1-4 mph, 10 m at 11, 17 m at 21 mph;
                            # rolling 1.3 s after the lead, 1.6 m/s^2 over the first 3 s against the lead's 1.5)
TAKEOFF_STAND_TOL_M = 1.0    # m
TAKEOFF_BLEND_V = 9.0       # m/s: above, the gap goes over to the following gap, reached at TAKEOFF_END_V
TAKEOFF_K_GAP = 0.25        # 1/s^2: per m the lead is beyond that gap
TAKEOFF_K_V = 0.8           # 1/s: per m/s it is faster
TAKEOFF_K_A = 0.5           # of the lead's own acceleration (vision's, so only part of it)
TAKEOFF_LEAD_A_MAX = 1.5    # m/s^2 of it counted at most
TAKEOFF_GAP_EASE_M = 1.5    # m of room over the gap across which the pull builds
TAKEOFF_A_MAX = 1.0         # m/s^2: this car takes no more from openpilot (CarControllerParams.ACCEL_MAX)
TAKEOFF_MIN_ACCEL = 0.15    # m/s^2: below this there is nothing to ask for
TAKEOFF_LEAD_BRAKING = -0.3 # m/s^2: nothing is asked while the lead brakes this much (a queue creeping up)
TAKEOFF_SLOWER_V = 0.5      # m/s the lead is slower than us ...
TAKEOFF_SETTLED_S = 1.0     # ... for this long: caught up (or it braked): over, until the next standstill


def takeoff_gap(v_ego: float, follow_gap_s: float, stop_gap_m: float = TAKEOFF_GAP_M) -> float:
  """The gap (m) to the car ahead at v_ego while moving off behind it: the standing gap plus TAKEOFF_GAP_S of our
  speed, going over to the following gap between TAKEOFF_BLEND_V and TAKEOFF_END_V (never more than that gap).
  Standing, TAKEOFF_STAND_TOL_M less (a car stopped a little inside its stop gap still moves off), gone by
  TAKEOFF_ARM_V."""
  t = float(np.interp(v_ego, [TAKEOFF_BLEND_V, TAKEOFF_END_V], [min(TAKEOFF_GAP_S, follow_gap_s), follow_gap_s]))
  stand = max(min(stop_gap_m, TAKEOFF_GAP_M), TAKEOFF_MIN_D)
  return max(stand - TAKEOFF_STAND_TOL_M * max(0.0, 1.0 - v_ego / TAKEOFF_ARM_V), TAKEOFF_MIN_D - 0.5) + t * v_ego


class LeadTakeOff:
  """Moving off behind a car as one unit with it: the least acceleration to ask for, or None.

  Standing behind a car the MPC only speeds up once the gap has grown past the following gap, and then by tenths;
  the comfort ceiling (0.5-0.6 m/s^2 in town, the driver's open-road pull) then holds the car back while the lead
  leaves: 00000147 at 16:41, 3 s after the lead rolled 9 mph with the lead at 16 and the gap 18 m (2.8 s), the driver
  on the gas, which drops the car's cruise control. The driver moves off with the car in front: rolling 1.3 s after
  it, with its acceleration, at 4 m + 1.3 s of their speed behind it.

  So, from the lead moving away from a (near) standstill of ours: the gap is takeoff_gap(v); the request is
  TAKEOFF_K_GAP per m the lead is beyond it, TAKEOFF_K_V per m/s it is faster and part of its own acceleration, up
  to TAKEOFF_A_MAX. It asks only with the lead beyond the gap, so it never brings the car inside it, and asks nothing
  once the lead is no faster or brakes: the MPC's and LeadBrake's braking is never overruled. It runs until the car
  does TAKEOFF_END_V or the lead has been slower than it for TAKEOFF_SETTLED_S (caught up); then the MPC follows.
  The open gap counts only as far as the lead is moving (in full from TAKEOFF_LEAD_FULL_V): a car that stands, however
  far ahead, never pulls this car toward it."""

  def __init__(self):
    self.active = False
    self.on_t = 0.0
    self.slower_t = 0.0

  def reset(self) -> None:
    self.active = False
    self.on_t = self.slower_t = 0.0

  def update(self, v_ego: float, lead_present: bool, d_rel: float, v_lead: float, a_lead: float, a_max: float,
             dt: float, follow_gap_s: float = TAKEOFF_GAP_S, stop_gap_m: float = TAKEOFF_GAP_M) -> float | None:
    if not lead_present or d_rel < TAKEOFF_MIN_D:
      self.reset()
      return None
    room = d_rel - takeoff_gap(v_ego, follow_gap_s, stop_gap_m)
    faster = v_lead - v_ego
    if not self.active:
      rolling = v_lead >= TAKEOFF_LEAD_V or (v_lead >= TAKEOFF_LEAD_EARLY_V and a_lead >= TAKEOFF_LEAD_EARLY_A)
      going = (v_ego < TAKEOFF_ARM_V and room > 0.0 and rolling and faster > 0.0
               and a_lead > TAKEOFF_LEAD_BRAKING)
      self.on_t = self.on_t + dt if going else 0.0
      if self.on_t < TAKEOFF_ON_S - 1e-6:
        return None
      self.active, self.slower_t = True, 0.0
    self.slower_t = self.slower_t + dt if faster < -TAKEOFF_SLOWER_V else 0.0
    if v_ego >= TAKEOFF_END_V or self.slower_t >= TAKEOFF_SETTLED_S:
      self.reset()
      return None
    if room <= 0.0 or faster < 0.0 or a_lead <= TAKEOFF_LEAD_BRAKING:
      return None
    moving = min(max(v_lead, 0.0) / TAKEOFF_LEAD_FULL_V, 1.0)
    a = TAKEOFF_K_GAP * room * moving + TAKEOFF_K_V * faster + TAKEOFF_K_A * min(max(a_lead, 0.0), TAKEOFF_LEAD_A_MAX)
    a = min(a, a_max, TAKEOFF_A_MAX) * min(1.0, room / TAKEOFF_GAP_EASE_M)
    return a if a >= TAKEOFF_MIN_ACCEL else None


# The car's own sign reader lowers the set speed by itself (ACC auto speed). When its reading is far under the map's
# limit for the road, it is most likely a sign that does not apply (a school zone's conditional 25, a side road's
# sign): 00000150 2.9 min on Blacow Road, 2026-11-25, the car read 25 where the map says 35 and the driver does 40;
# the set speed went 40 -> 25 and the car braked at 1.2 m/s^2 for 6 s on an open road, and again 7 min later.
SIGN_DROP_MIN = 4.0         # m/s (9 mph): a drop of the set speed at least this large in one step (a button step is 5 mph)
SIGN_DROP_MATCH = 1.0       # m/s: ... to the sign reader's value
SIGN_DROP_MAP_OVER = 4.4    # m/s (10 mph): ... with the map's limit at least this far above it
SIGN_DROP_FRAMES = 10       # frames (0.5 s) the sign reader's value may arrive after the set speed changed


class SignDropHold:
  """The set speed to drive by. Normally the car's. After the car's sign reader dropped it far under the map's limit
  (SIGN_DROP_*), the set speed from before the drop, at most the map's limit + SIGN_DROP_MAP_OVER, for as long as
  the car's set speed stays where the sign put it and the map still says the road is that much faster. Any change of
  the set speed (the driver's buttons, the next sign) ends it, and so does the map agreeing with the sign."""

  def __init__(self):
    self.prev: float | None = None
    self.held: float | None = None
    self.dropped_to = 0.0
    self.before = 0.0         # the set speed before a large drop ...
    self.pending = 0          # ... and the frames left for the sign reader's value to show it was its doing

  def update(self, v_set: float, sign: float, map_limit: float) -> float:
    if self.held is not None:
      if abs(v_set - self.dropped_to) > 0.3 or map_limit - v_set < SIGN_DROP_MAP_OVER:
        self.held = None
    else:
      if self.prev is not None and self.prev - v_set >= SIGN_DROP_MIN:
        self.before, self.dropped_to, self.pending = self.prev, v_set, SIGN_DROP_FRAMES
      elif self.pending and abs(v_set - self.dropped_to) > 0.3:
        self.pending = 0
      if self.pending:
        self.pending -= 1
        if sign > 0.0 and abs(v_set - sign) <= SIGN_DROP_MATCH and map_limit - v_set >= SIGN_DROP_MAP_OVER:
          self.held, self.pending = min(self.before, map_limit + SIGN_DROP_MAP_OVER), 0
    self.prev = v_set
    return max(v_set, self.held) if self.held is not None else v_set


class LeadFollowCap:
  """The driver's rule (2026-11-23): follow the car in front, up to FOLLOW_OVER_MPH over the posted limit; faster than
  that, let it go. While a car is being followed the cruising speed may rise to the lead's speed, at most the limit +
  FOLLOW_OVER_MPH: the profile's own cap (limit + the driver's offset) and the set speed held the car back from a
  lead doing a few mph more. The MPC still keeps the following gap. Needs a posted limit; without one, nothing.

  Rises with the lead at once, comes down at most PROFILE_COAST_DECEL when the lead slows, goes or the limit changes
  (a lift-off, never a brake: the cap above the set speed is only ever an allowance)."""

  def __init__(self):
    self.v: float | None = None

  def update(self, limit_ms: float, v_ego: float, lead_present: bool, d_rel: float, v_rel: float, dt: float) -> float | None:
    """The speed (m/s) the car may cruise at while following, or None."""
    want = None
    if limit_ms > 0.0 and lead_present and d_rel > 0.0:
      following = d_rel <= min(max(FOLLOW_MIN_D, FOLLOW_MAX_GAP_S * v_ego), FOLLOW_MAX_D)
      if following:
        want = min(max(v_ego + v_rel, 0.0) + FOLLOW_LEAD_MARGIN, limit_ms + FOLLOW_OVER_MPH * CV.MPH_TO_MS)
    if want is not None and (self.v is None or want >= self.v):
      self.v = want
    elif self.v is not None:
      floor = want if want is not None else 0.0
      self.v = max(floor, min(self.v, v_ego + 0.5) - PROFILE_COAST_DECEL * dt)
      if want is None and self.v <= v_ego - 2.0:    # well under the car's speed: nothing left to allow
        self.v = None
    return self.v


# A gentle pull-away with openpilot speed control, whatever the profile: the most acceleration openpilot asks for, by
# speed. Route 00000103 (2026-11-16): after every turn and stop openpilot asked for its +1.0 m/s^2 maximum (the stock
# planner wanted 1.3-1.45) and the Ocean delivered up to +1.44: "accelerated fast". The car overshoots a request by
# ~40% at town speeds, so 0.6 asked is ~0.8 felt; on faster roads the cap opens back up to the car's 1.0.
COMFORT_ACCEL_BP = [0.0, 11.2, 20.0]   # m/s (0, 25 mph, 45 mph)
COMFORT_ACCEL_V = [0.6, 0.6, 1.0]      # m/s^2


CRUISE_ACCEL_BP = [10.0, 16.5, 24.6]   # m/s: the middle of the 15-30, 30-45 and 45+ mph bands the profile measures
CRUISE_ACCEL_MIN = 0.2                 # m/s^2: never a slower pull than this from a profile
STOP_GAP_MIN = 4.0                     # m: never stop closer behind a car than this (the MPC's own is 6)


MERGE_LIMIT_MIN = 24.6                 # m/s (55 mph): a fast road...
MERGE_BELOW = 4.5                      # ...with the car this far (10 mph) under its limit: getting up to speed (merging)


def merging(limit_ms: float, v_ego: float) -> bool:
  return limit_ms >= MERGE_LIMIT_MIN and v_ego < limit_ms - MERGE_BELOW


def comfort_accel_cap(v_ego: float, profile=None, merge: bool = False) -> float:
  """The most acceleration openpilot asks for at v_ego: the driver's own open-road pull (profile cruise_accel_*) when
  the profile has it, else the comfort table. Merging (well under a fast road's limit) never below the table's: the
  driver's open-road median at 45+ mph (0.27) would get onto a freeway far too slowly (their merges: 0.39 median,
  0.85 p75, 4 clean ones on 2026-11-20); profile merge_accel once there are enough merges."""
  table = float(np.interp(v_ego, COMFORT_ACCEL_BP, COMFORT_ACCEL_V))
  if merge:
    m = profile.merge_accel if profile is not None and profile.merge_accel > 0.0 else table
    return max(m, CRUISE_ACCEL_MIN)
  if profile is not None:
    vals = (profile.cruise_accel_town, profile.cruise_accel_mid, profile.cruise_accel_fast)
    if all(a > 0.0 for a in vals):
      return max(float(np.interp(v_ego, CRUISE_ACCEL_BP, vals)), CRUISE_ACCEL_MIN)
  return table


def stop_gap(stop_gap_m: float) -> float | None:
  """The MPC's stop distance from the profile (m), or None for the MPC's own."""
  return max(stop_gap_m, STOP_GAP_MIN) if stop_gap_m > 0.0 else None


TURN_EXIT_S = 3.0           # s after a junction turn (fisker_drive_style.TURN_EXIT_S, what the profile measures over)


def follow_t(gap_s: float, v_ego: float) -> float:
  return max(T_FOLLOW_MIN, gap_s - STOP_DISTANCE / max(v_ego, V_FOLLOW_MIN))


def lead_brake_need(follow_gap_s: float, v_ego: float, d_rel: float, v_rel: float, lag_s: float = 0.0) -> float:
  """The even braking (m/s^2) that brings the closing speed to zero at the following gap behind the lead
  (STOP_DISTANCE + follow_gap_s * lead speed), starting lag_s from now; inf with no room left, 0 when not closing."""
  closing = -v_rel
  if closing <= 0.0:
    return 0.0
  v_lead = max(v_ego + v_rel, 0.0)
  room = d_rel - (STOP_DISTANCE + follow_gap_s * v_lead) - closing * lag_s
  return closing * closing / (2.0 * room) if room > 0.5 else float("inf")


# A stop that ends softly. An even stop brakes as hard in its last metre as in its first, and at low speed this car
# brakes harder than it is asked (median braking by request and speed, 2,136 steady frames, drives 00000144-0000014c):
#   asked        under 2 mph   2-3    3-4    4-5    5-6.5   6.5-8 mph
#   0.15-0.45        0.63      0.60   0.43   0.60   0.69    0.63
#   0.65-0.90        2.12      1.67    -     0.97   1.06    1.03
#   0.90-1.20        2.66      2.03   1.74   1.37   1.34    1.40
#   1.20-1.60        3.49      2.17   1.66   1.20   1.30    1.51
#   1.60-2.20        3.89      3.42   3.18   2.99   2.58    1.90
# So a request over 1.6 is a jolt from 6.5 mph down, and only 0.45 or less is delivered gently below 4 mph (the car
# controller's SOFT_STOP_* eases the request there, but not when the planner asks 1.1 or more: a stop still needing
# 1.5 at 5 mph goes through as asked and ends at 3-4 m/s^2; 13 of 13 such stops in 30 drives, 0000014c at 2:38 and
# 18:09). A stop is therefore aimed at STOP_LAND_V with STOP_LAND_M still to go: the braking before that is a little
# firmer (about 6% from 80 m, more close in, at most STOP_LAND_EXTRA more than the even stop), and what is left needs
# about 0.8 at 6.5 mph and less after, which the car delivers as asked and the soft landing can finish.
STOP_LAND_V = 2.9           # m/s (6.5 mph)
STOP_LAND_M = 6.0           # m still to go at that speed (gentle_stop_distance(STOP_LAND_V) + LAND_MARGIN_M)
STOP_LAND_EXTRA = 0.4       # m/s^2: the most the landing may add to the even stop (arriving too hot to land softly)
# The landing itself. Arriving right is not enough: on the last metres the driving model asks about 1.4 times the
# even braking (0000014c at 2:38: 1.74 asked at 6.6 mph with 1.28 needed, 1.49 at 4.2 mph with 0.98), which the car
# then doubles. So while the stop point is known (the standing car ahead, or the model's stop point) and the gentle
# braking below still stops the car LAND_MARGIN_M short of it, a firmer request is replaced by the gentle one
# (landing_floor). Checked again every frame against the measured room: if the room is not there (the car ahead
# rolled back, the estimate moved), or the planner asks LAND_URGENT or firmer, its request goes through untouched.
LAND_LOW_V = 1.8            # m/s (4 mph): below, only 0.45 or less is delivered gently
LAND_REQ_HI = 1.2           # m/s^2 asked between STOP_LAND_V and LAND_LOW_V (delivered ~1.3)
LAND_REQ_LO = 0.45          # m/s^2 asked below LAND_LOW_V (delivered ~0.6)
LAND_GOT_HI = 1.3           # m/s^2 counted on for LAND_REQ_HI
LAND_GOT_LO = 0.55          # m/s^2 counted on for LAND_REQ_LO
LAND_BLEND_V = 0.2          # m/s either side of LAND_LOW_V over which the request goes from one to the other
LAND_MARGIN_M = 1.0         # m the gentle stop must end short of the stop point for it to replace the planner's
LAND_URGENT = -2.2          # m/s^2: a planner asking this or firmer is not landing, it is braking for something
LAND_LEAD_STANDING_V = 1.0  # m/s: a car ahead slower than this stands (its place is a stop point)


def gentle_stop_distance(v_ego: float) -> float:
  """Metres the gentle landing needs to stop from v_ego (counting on LAND_GOT_*)."""
  low = min(v_ego, LAND_LOW_V)
  d = low * low / (2.0 * LAND_GOT_LO)
  if v_ego > LAND_LOW_V:
    d += (v_ego * v_ego - LAND_LOW_V * LAND_LOW_V) / (2.0 * LAND_GOT_HI)
  return d


def landing_floor(v_ego: float, room: float) -> float | None:
  """The gentle request (m/s^2, negative) that may stand in for a firmer one on the last metres to a stop point
   metres ahead, or None when it may not: too fast for the landing yet, or not enough room for it."""
  if v_ego > STOP_LAND_V + 0.3 or room < gentle_stop_distance(v_ego) + LAND_MARGIN_M:
    return None
  k = min(max((v_ego - (LAND_LOW_V - LAND_BLEND_V)) / (2.0 * LAND_BLEND_V), 0.0), 1.0)
  return -(LAND_REQ_LO + (LAND_REQ_HI - LAND_REQ_LO) * k)


def landing_need(v_ego: float, room: float) -> float:
  """The even braking (m/s^2) over `room` metres to the stop point that leaves the car at STOP_LAND_V with
  STOP_LAND_M to go; inside that, or with no room for it, the even stop. inf with no room left."""
  if room <= 0.5:
    return float("inf")
  even = v_ego * v_ego / (2.0 * room)
  if v_ego <= STOP_LAND_V or room <= STOP_LAND_M:
    return even
  land = (v_ego * v_ego - STOP_LAND_V * STOP_LAND_V) / (2.0 * max(room - STOP_LAND_M, 1e-3))
  return min(land, even + STOP_LAND_EXTRA)


def lead_brake_stop_need(v_ego: float, d_rel: float, v_lead: float, lead_decel: float, stop_gap_m: float,
                         lag_s: float = 0.0) -> float:
  """The braking (m/s^2) that stops stop_gap_m behind where a lead braking at lead_decel will stand, starting lag_s
  from now, with a soft landing (landing_need); inf with no room left."""
  room = d_rel + v_lead * v_lead / (2.0 * max(lead_decel, 0.1)) - stop_gap_m - v_ego * lag_s
  return landing_need(v_ego, room)


class LeadBrake:
  """Slowing early for a car ahead, the way the driver does: once the even braking the situation needs reaches the
  driver's lead_brake_a, an acceleration ceiling that brakes a little more than that need, straight away, so the
  braking is highest at the start and fades as the car closes. It only ever adds braking; the MPC's and the driving
  model's own stay underneath.

  Two situations:
  - a car that is (nearly) stopped, or braking firmly and already much slower: taken to stop; the need is to stand
    stop_gap_m behind where it will. This is the queue at a light.
  - a car that keeps moving: the need is to end the closing at the following gap behind it (as before). It starts only
    with LEAD_BRAKE_MIN_ROOM_M left outside the gap (inside is following) and asks LEAD_BRAKE_A_MOVE_MAX at most.

  Before 2026-11-24 this was a speed target through the cruise path: that path ramps slowly and stops at 1.2 m/s^2,
  so the braking started low and ran behind the need all the way (00000144 at 8:22: 0.7 rising to 1.6 over 4 s from
  104 m at 35 mph, the need going 0.9 -> 1.8; 00000147 at 5:06: 3.4 m/s^2 at the end)."""

  def __init__(self, dt: float):
    self.dt = dt
    self.active = False
    self.pending_s = 0.0
    self.d_f: float | None = None   # the lead's smoothed distance, relative speed and acceleration
    self.v_f = 0.0
    self.a_f = 0.0
    self.out: float | None = None
    self.need = 0.0

  def reset(self) -> None:
    self.active = False
    self.pending_s = 0.0
    self.d_f = None
    self.out = None

  def _smooth(self, d_rel: float, v_rel: float, a_lead: float) -> tuple[float, float, float]:
    if self.d_f is None or abs(d_rel - self.d_f) > LEAD_BRAKE_NEW_LEAD_M:
      self.d_f, self.v_f, self.a_f = d_rel, v_rel, a_lead
    else:
      k = min(1.0, self.dt / LEAD_BRAKE_FILTER_S)
      # the distance closes at the relative speed between frames: smooth what is left after that, not the closing
      self.d_f += self.v_f * self.dt
      self.d_f += (d_rel - self.d_f) * k
      self.v_f += (v_rel - self.v_f) * k
      self.a_f += (a_lead - self.a_f) * k
    return self.d_f, self.v_f, self.a_f

  def _need(self, follow_gap_s: float, v_ego: float, d: float, v_rel: float, a_lead: float, stop_gap_m: float,
            a_now: float, starting: bool) -> tuple[float, float]:
    """(need, closing speed). The need is worked out from where the car will be LEAD_BRAKE_LAG_S from now at the
    braking it already has (a request takes that long to become braking): no room is taken off twice once it brakes."""
    v_lead = max(v_ego + v_rel, 0.0)
    closing = v_ego - v_lead
    if closing <= 0.0:
      return 0.0, closing
    lag, a_now = LEAD_BRAKE_LAG_S, min(a_now, 0.0)
    v_then = max(v_ego + a_now * lag, 0.0)
    gone = (v_ego + v_then) / 2.0 * lag          # our way in the lag
    need = 0.0
    closing_then = v_then - v_lead
    room = d + v_lead * lag - gone - (STOP_DISTANCE + follow_gap_s * v_lead)
    if closing_then > 0.0 and room >= (LEAD_BRAKE_MIN_ROOM_M if starting else 0.5):
      need = min(closing_then * closing_then / (2.0 * room), LEAD_BRAKE_A_MOVE_MAX)
    lead_decel = None
    if v_lead < LEAD_STOPPED_V:
      lead_decel = max(-a_lead, LEAD_STOPPED_DECEL)
    elif a_lead <= -LEAD_BRAKING_DECEL and v_lead < LEAD_BRAKING_SLOW * v_ego:
      lead_decel = -a_lead
    if lead_decel is not None:
      need = max(need, lead_brake_stop_need(v_then, d - gone, v_lead, lead_decel, stop_gap_m))
    return need, closing

  def update(self, lead_brake_a: float, follow_gap_s: float, v_ego: float, lead_present: bool, d_rel: float,
             v_rel: float, a_lead: float = 0.0, prob: float = 1.0, a_now: float = 0.0,
             stop_gap_m: float = STOP_DISTANCE) -> float | None:
    """The acceleration ceiling (m/s^2, negative) or None. a_now: what the planner asks now (the request builds from
    there at LEAD_BRAKE_JERK)."""
    if (lead_brake_a < 0.1 or not lead_present or prob < LEAD_BRAKE_PROB or v_ego < LEAD_BRAKE_V_MIN
        or not (0.0 < d_rel < LEAD_BRAKE_MAX_D)):
      self.reset()
      return None
    d, v_rel, a_lead = self._smooth(d_rel, v_rel, a_lead)
    self.need, closing = self._need(follow_gap_s, v_ego, d, v_rel, a_lead, stop_gap_m, a_now, not self.active)
    if self.active:
      self.active = closing >= LEAD_BRAKE_HOLD_CLOSING and self.need >= LEAD_BRAKE_HOLD * lead_brake_a
    else:
      starting = closing >= LEAD_BRAKE_START_CLOSING and self.need >= lead_brake_a
      # a frame outside the window takes time off instead of starting over (one odd reading isn't the end of it)
      self.pending_s = self.pending_s + self.dt if starting else max(0.0, self.pending_s - self.dt)
      self.active = self.pending_s >= LEAD_BRAKE_ON_S - 1e-6
    if not self.active:
      self.out = None
      return None
    self.pending_s = 0.0
    want = -min(self.need * LEAD_BRAKE_MARGIN, self.need + LEAD_BRAKE_MARGIN_MAX, LEAD_BRAKE_A_MAX)
    prev = self.out if self.out is not None else min(a_now, 0.0)
    step = LEAD_BRAKE_JERK * self.dt
    self.out = max(want, prev - step) if want < prev else min(want, prev + step)
    return self.out


# Stopping where the driving model plans to stop, evenly and from the moment it says so.
# The model's 10 s plan shows a coming stop early: in 30 drives (9.4 h) its end speed fell to half the current speed
# (or it came to rest inside the 10 s) before 85 of 94 stops from 18 mph or more, a median 2.9 s before the braking
# that then happened, with the even braking needed at 0.88 m/s^2 instead of 1.28; and the stop distance it implies
# was right within about 20% (median 0.97 of the real one). The braking itself came late and rising: Dynamic
# Experimental Control hands the speed to the model only after a while, and the model's own request starts low
# (0000014c at 18:09, a red light from 42 mph with no car ahead: the plan's end speed began falling 250 m out and
# was at half 195 m out; braking began at 147 m at 1.0 and rose to 2.4, then 3.9 in the last second).
# So: an acceleration ceiling from the model's own stop point. First no more accelerating (the lift-off), then, once
# the stop needs the driver's own start-of-braking level, the braking of a soft-landing stop there (landing_need).
# It only ever adds braking, and only above MODEL_STOP_V_MIN: the last metres stay with the model and the MPC.
# With a car ahead that keeps moving and is nearer than that stop point, the car ahead governs the braking (the
# driver's rule: follow it): 39 of the 49 times the plan said stop and the car then did not were behind a lead. The
# lift-off stays (the plan still says stop), so a lead the camera drops and finds again does not switch the request
# between holding back and pulling (0000014c at 2:25: four times in 2 s).
MODEL_STOP_END_FRAC = 0.5       # the plan's speed at 10 s is at most this share of the current speed ...
MODEL_STOP_END_V = 9.0          # m/s (20 mph) ... and at most this: a stop (or a crawl) is planned
MODEL_STOP_START_V = 8.0        # m/s (18 mph): it starts only above this
MODEL_STOP_V_MIN = 3.0          # m/s: below, the last metres are the model's and the MPC's
MODEL_STOP_ON_S = 0.5           # s the plan must say so before the lift-off (no more accelerating)
MODEL_STOP_BRAKE_S = 1.0        # s it must have said so before braking is added: of 91 times it would have acted in
                                # 1.6 h of openpilot speed control, 87 were followed by a stop or a slowdown to under
                                # half; the others were a plan that said stop for a second or three and then did not
                                # (a light that changed, 00000144 at 3:03; a doubtful half second, 0000013a at 6:00)
MODEL_STOP_OFF_S = 0.3          # s without it before letting go (a frame or two of doubt is not the end of it)
MODEL_STOP_POINT_TAU_S = 0.6    # s: the stop point (a place on the road) is smoothed over this
MODEL_STOP_LAG_S = 0.6          # s the car goes on before a request is braking
MODEL_STOP_JERK = 1.0           # m/s^3: how fast the request builds and fades (the comfort rate)
MODEL_STOP_A_MAX = 2.0          # m/s^2: never asks for more (firmer stops are the model's and the MPC's own)
MODEL_STOP_LEAD_MOVING_V = 3.0  # m/s: a car ahead at least this fast is one that keeps moving
MODEL_STOP_LEAD_MARGIN_M = 5.0  # m: the stop point must be this far short of such a car for this to act
# The braking aims this far short of the model's stop point. Arriving too fast moves the model's own stop point forward,
# past the line (0000014f 13.3 min, 00000150 2.3 min, 2026-11-25: the car stopped exactly where the plan ended, over
# the line). Stopping a little short is the cheap error: the driver creeps up with a light press of the accelerator.
MODEL_STOP_LINE_MARGIN_M = 2.0


def model_stop_distance(v_ego: float, v_end: float, x_end: float, d_rest: float | None) -> float | None:
  """Metres to where the model's plan comes to rest: d_rest when it does so inside its horizon, else the even
  slowdown its end state (speed v_end after x_end metres) implies, carried on to rest. None: no stop planned."""
  if d_rest is not None:
    return max(d_rest, 0.0)
  if v_ego < 0.5 or v_end > MODEL_STOP_END_V or v_end > MODEL_STOP_END_FRAC * v_ego or x_end <= 1.0:
    return None
  return x_end * v_ego * v_ego / (v_ego * v_ego - v_end * v_end)


class ModelStop:
  def __init__(self, dt: float):
    self.dt = dt
    self.active = False       # the plan says stop: no accelerating
    self.braking = False      # ... and the stop needs braking now
    self.on_s = 0.0
    self.off_s = 0.0
    self.s = 0.0              # our way since the first frame (m), to hold the stop point as a place
    self.point: float | None = None
    self.out: float | None = None
    self.need = 0.0
    self.room = 0.0

  def reset(self) -> None:
    self.active = self.braking = False
    self.on_s = self.off_s = 0.0
    self.point = None
    self.out = None

  def room_left(self) -> float | None:
    """Metres to the stop point while one is held (the plan says stop), else None."""
    return self.point - self.s if self.active and self.point is not None else None

  def update(self, start_need: float, v_ego: float, v_end: float, x_end: float, d_rest: float | None,
             lead_present: bool, d_lead: float, v_lead: float, a_now: float) -> float | None:
    """The acceleration ceiling (m/s^2, <= 0) or None. start_need: the driver's lead_brake_a (0 or less: off);
    v_end, x_end: the plan's speed and position at the end of its horizon; d_rest: where it comes to rest inside it."""
    self.s += v_ego * self.dt
    dist = model_stop_distance(v_ego, v_end, x_end, d_rest) if start_need >= 0.1 else None
    # a moving car ahead, short of the stop point: it governs the braking (follow it); the lift-off stays
    follow = (dist is not None and lead_present and v_lead >= MODEL_STOP_LEAD_MOVING_V
              and d_lead < dist + MODEL_STOP_LEAD_MARGIN_M)
    if start_need < 0.1 or v_ego < 0.3:
      self.reset()
      return None
    if dist is None:
      if not self.active:
        self.on_s = 0.0
      self.off_s += self.dt
      if not self.active or self.off_s >= MODEL_STOP_OFF_S - 1e-6:
        self.reset()
        return None
    else:
      self.off_s = 0.0
      point = self.s + dist
      if self.point is None:
        self.point = point
      else:
        self.point += (point - self.point) * min(1.0, self.dt / MODEL_STOP_POINT_TAU_S)
      self.on_s += self.dt
      if not self.active:
        if v_ego < MODEL_STOP_START_V or self.on_s < MODEL_STOP_ON_S - 1e-6:
          return None
        self.active = True
    if self.point is None:    # (cannot be: active implies a point)
      return None
    if v_ego < MODEL_STOP_V_MIN:   # the last metres: no ceiling, but the stop point is kept for the landing
      self.out = None
      return None
    lag = MODEL_STOP_LAG_S
    v_then = max(v_ego + min(a_now, 0.0) * lag, 0.0)
    self.room = self.point - self.s - MODEL_STOP_LINE_MARGIN_M - (v_ego + v_then) / 2.0 * lag
    self.need = landing_need(v_then, self.room)
    if follow:
      self.braking = False
    elif self.need >= start_need and self.on_s >= MODEL_STOP_BRAKE_S - 1e-6:
      self.braking = True
    want = -min(self.need, MODEL_STOP_A_MAX) if self.braking else 0.0
    prev = self.out if self.out is not None else min(a_now, 0.0)
    step = MODEL_STOP_JERK * self.dt
    self.out = max(want, prev - step) if want < prev else min(want, prev + step)
    return min(self.out, 0.0)


def lead_coast(lead_coast_s: float, v_ego: float, lead_present: bool, d_rel: float, v_rel: float) -> bool:
  if lead_coast_s <= 0.0 or not lead_present or v_ego < 1.0:
    return False
  if not (0.0 < d_rel < LEAD_COAST_MAX_D) or v_rel > -LEAD_COAST_CLOSING:
    return False
  return d_rel / v_ego < lead_coast_s


def junction_speed(v_min: float, lat_accel: float, curvature: float) -> float:
  """m/s through a junction turn: the speed the driver's sideways acceleration allows at the turn's tightness now,
  never below their slowest junction speed (v_min) or above a junction turn's JUNCTION_V_MAX. lat_accel 0: v_min
  throughout (the old cap). The driver slows with the tightness (1.26 m/s^2 at 20-33 m radius, 11.8 mph; 0.56 at
  5-8 m, 4.5 mph): holding the apex speed for the whole turn braked them on the way out (2026-11-21, 7 takeovers)."""
  if lat_accel <= 0.0 or abs(curvature) < 1e-4:
    return v_min
  return max(v_min, min(JUNCTION_V_MAX, math.sqrt(lat_accel / abs(curvature))))


def junction_turn(left_blinker: bool, right_blinker: bool, v_ego: float, curvature: float) -> bool:
  return (left_blinker or right_blinker) and v_ego < JUNCTION_V_MAX and abs(curvature) > JUNCTION_CURVATURE


def turn_exit_cap(exit_left_s: float, turn_exit_accel: float) -> float:
  """The most acceleration allowed (m/s^2) with exit_left_s of the turn exit window left; inf when not limiting."""
  return turn_exit_accel if exit_left_s > 0.0 else float("inf")


def profile_speed(limit_ms: float, offset_mph: float) -> float | None:
  """The cap (m/s) for a posted limit (m/s), or None without a limit."""
  if limit_ms <= 0.0:
    return None
  return max(0.0, limit_ms + offset_mph * CV.MPH_TO_MS)
