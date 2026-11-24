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

LEAD_BRAKE_MIN_CLOSING = 4.0    # m/s: slower than this, closing is left to the MPC (the driver and openpilot start alike);
                                # also the closing speed lead_brake_a is measured from (driver_profile_gen)
LEAD_BRAKE_MIN_ROOM_M = 10.0    # m outside the following gap still left when it starts: inside, it is following (MPC)
LEAD_BRAKE_START_MAX = 2.0      # m/s^2: needing more than this already is no slowdown to shape (the MPC's). Was 1.2: a
                                # stopped car first seen by vision at 80 m and 32 mph can already need 1.2-1.4
                                # (00000129 at 3:55: the MPC alone asked 1.0 for 2 s and 2.2 at the end)
LEAD_BRAKE_FILTER_S = 0.5       # s: the lead's distance and speed are smoothed over this before the need is worked out.
                                # This car's leads are vision only: the same stopped car read 76, 64, 73, 63, 65 m and
                                # 17.6 then 8.2 mph within a second
LEAD_BRAKE_NEW_LEAD_M = 12.0    # m: a jump this large in one frame is another car: the smoothing starts again from it
LEAD_BRAKE_MARGIN = 1.1         # slow this much more than the even need: the need then falls as the car closes, so the
                                # slowdown ends gentler than it began, as the driver's does
LEAD_BRAKE_ON_S = 0.5           # s the start condition must hold (radar flicker)
LEAD_BRAKE_HOLD_CLOSING = 1.5   # m/s: once slowing, keep going until the closing speed is down to this
LEAD_BRAKE_MAX_D = 150.0        # m
LEAD_BRAKE_A_MAX = 1.5          # m/s^2: the cap never asks for more (harder is the MPC's job)
LEAD_BRAKE_LAG_S = 1.0          # s: lead the profile by the planner's ~1 s speed-loop lag
LEAD_BRAKE_HOLD = 0.5           # once slowing, keep it while the need is at least this fraction of lead_brake_a

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


TAKEOFF_V_EGO_MAX = 2.5     # m/s: moving off ends here, the MPC follows from then on
TAKEOFF_LEAD_V = 0.25       # m/s: the lead is rolling (a standing lead reads within 0.02 of zero)
TAKEOFF_ON_S = 0.3          # s of that before it counts
TAKEOFF_LEAD_A_MIN = -0.3   # m/s^2: not while the lead is braking again (a queue creeping up)
TAKEOFF_MIN_D = 3.5         # m: never from closer than this
TAKEOFF_GAIN = 0.6          # 1/s: the pull per m/s the lead is faster
TAKEOFF_MIN_ACCEL = 0.15    # m/s^2: below this there is nothing to ask for


class LeadTakeOff:
  """Moving off behind a car that moves off: the least acceleration to ask for, or None.

  Standing behind a car the MPC only speeds up once the gap has grown past the following gap, and then by tenths:
  00000129 at 2:36, the light green, the lead rolling at 154.6 s, the plan off its stop at 156.1 asking +0.10, +0.27
  by 158.7 with the lead 12 m off at 7 mph, the driver on the gas at 159.2 (which drops the car's cruise control).
  The driver moves off as the car in front does. So from the lead rolling for TAKEOFF_ON_S: TAKEOFF_GAIN per m/s it
  is faster than this car, up to the driver's own launch, until this car does TAKEOFF_V_EGO_MAX. It asks nothing as
  soon as the lead is no faster (the gap is then not growing) or brakes, so the MPC's braking is never overruled:
  this only ever raises a request while the gap is opening."""

  def __init__(self):
    self.rolling_t = 0.0
    self.active = False

  def update(self, v_ego: float, lead_present: bool, d_rel: float, v_lead: float, a_lead: float, launch_accel: float,
             dt: float) -> float | None:
    rolling = lead_present and d_rel >= TAKEOFF_MIN_D and v_lead >= TAKEOFF_LEAD_V and a_lead >= TAKEOFF_LEAD_A_MIN
    self.rolling_t = self.rolling_t + dt if rolling else 0.0
    a = None
    if self.rolling_t >= TAKEOFF_ON_S and v_ego < TAKEOFF_V_EGO_MAX:
      a = min(TAKEOFF_GAIN * (v_lead - v_ego), launch_accel)
      a = a if a >= TAKEOFF_MIN_ACCEL else None
    self.active = a is not None
    return a


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


class LeadBrake:
  """Closing fast on a slower car: once an even slowdown to the driver's following gap behind it needs the driver's
  lead_brake_a, a speed target that slows that evenly. It only ever lowers the speed; the MPC's own lead braking stays
  underneath, and everything outside the start window below is the MPC's.

  Starts only where the driver starts (closing >= LEAD_BRAKE_MIN_CLOSING, still LEAD_BRAKE_MIN_ROOM_M outside the
  following gap, the need between lead_brake_a and LEAD_BRAKE_START_MAX) for LEAD_BRAKE_ON_S: a first cut without that
  window, replayed over 72 drives, fired 1417 times for ~0.3 s each, almost all freeway following inside the gap with
  radar noise. Once slowing it holds while the closing speed and the need stay up."""

  def __init__(self, dt: float):
    self.dt = dt
    self.active = False
    self.pending_s = 0.0
    self.d_f: float | None = None   # the lead's smoothed distance and relative speed
    self.v_f = 0.0

  def reset(self) -> None:
    self.active = False
    self.pending_s = 0.0
    self.d_f = None

  def _smooth(self, d_rel: float, v_rel: float) -> tuple[float, float]:
    if self.d_f is None or abs(d_rel - self.d_f) > LEAD_BRAKE_NEW_LEAD_M:
      self.d_f, self.v_f = d_rel, v_rel
    else:
      k = min(1.0, self.dt / LEAD_BRAKE_FILTER_S)
      # the distance closes at the relative speed between frames: smooth what is left after that, not the closing
      self.d_f += self.v_f * self.dt
      self.d_f += (d_rel - self.d_f) * k
      self.v_f += (v_rel - self.v_f) * k
    return self.d_f, self.v_f

  def update(self, lead_brake_a: float, follow_gap_s: float, v_ego: float, lead_present: bool, d_rel: float,
             v_rel: float) -> tuple[float, float] | None:
    """(v_target, a_target) or None."""
    if lead_brake_a < 0.1 or not lead_present or v_ego < 1.0 or not (0.0 < d_rel < LEAD_BRAKE_MAX_D):
      self.reset()
      return None
    d_rel, v_rel = self._smooth(d_rel, v_rel)
    closing = -v_rel
    v_lead = max(v_ego + v_rel, 0.0)
    room = d_rel - (STOP_DISTANCE + follow_gap_s * v_lead) - closing * LEAD_BRAKE_LAG_S
    need = lead_brake_need(follow_gap_s, v_ego, d_rel, v_rel, LEAD_BRAKE_LAG_S)
    if self.active:
      self.active = closing >= LEAD_BRAKE_HOLD_CLOSING and need >= LEAD_BRAKE_HOLD * lead_brake_a
    else:
      starting = (closing >= LEAD_BRAKE_MIN_CLOSING and room >= LEAD_BRAKE_MIN_ROOM_M
                  and lead_brake_a <= need <= LEAD_BRAKE_START_MAX)
      # a frame outside the window takes time off instead of starting over (one odd reading isn't the end of it)
      self.pending_s = self.pending_s + self.dt if starting else max(0.0, self.pending_s - self.dt)
      self.active = self.pending_s >= LEAD_BRAKE_ON_S - 1e-6
    if not self.active:
      return None
    self.pending_s = 0.0
    a = min(need * LEAD_BRAKE_MARGIN, LEAD_BRAKE_A_MAX)
    v_target = v_lead + math.sqrt(2.0 * a * max(room, 0.0))
    return min(v_target, v_ego), -a


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
