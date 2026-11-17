"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

How the longitudinal planner drives by the driver profile (driver_profile.py), as small pure functions:

  follow_t(gap_s, v)        the MPC's t_follow that gives the driver's measured following gap. The profile's gap is
                            what was measured, distance / speed behind a steady lead; the MPC keeps
                            t_follow * v + STOP_DISTANCE behind one, so t_follow = gap - STOP_DISTANCE / v, never below
                            openpilot's "aggressive" 1.25 s (and the profile's own bounds cap it at 2.5 s).
  lead_coast(...)           True when the lead is slower and inside the driver's lead_coast_s time gap: the planner then
                            stops accelerating (the cap is 0 m/s^2: coasting; braking stays the MPC's). Route
                            000000f5 (2026-11-15): openpilot still accelerated +0.6 at 42 mph toward a lead 79 m ahead
                            closing at 5 mph, and +0.58 toward one 8 m ahead at 7 mph; both times the driver braked.
  junction_turn(...)        a signaled, sharp, slow turn is under way: cap the speed at the driver's junction speed.
  profile_speed(...)        the cruising speed cap: the posted limit + the driver's own offset for that limit.
  turn_exit_cap(...)        for TURN_EXIT_S after a junction turn, pull away no harder than the driver does out of one.
  comfort_accel_cap(v)      the gentlest of all: openpilot never asks for more than this (0.6 m/s^2 in town).
"""
import numpy as np

from openpilot.common.constants import CV

STOP_DISTANCE = 6.0         # m, long_mpc.STOP_DISTANCE
T_FOLLOW_MIN = 1.25         # s, openpilot's "aggressive"
V_FOLLOW_MIN = 5.0          # m/s: below, the stop distance term is not divided by a near-zero speed

LEAD_COAST_CLOSING = 0.5    # m/s: the lead is this much slower (closing) before coasting
LEAD_COAST_MAX_D = 120.0    # m: radar leads further than this are ignored
LEAD_COAST_ACCEL = 0.0      # m/s^2 cap while coasting

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

# A gentle pull-away with openpilot speed control, whatever the profile: the most acceleration openpilot asks for, by
# speed. Route 00000103 (2026-11-16): after every turn and stop openpilot asked for its +1.0 m/s^2 maximum (the stock
# planner wanted 1.3-1.45) and the Ocean delivered up to +1.44: "accelerated fast". The car overshoots a request by
# ~40% at town speeds, so 0.6 asked is ~0.8 felt; on faster roads the cap opens back up to the car's 1.0.
COMFORT_ACCEL_BP = [0.0, 11.2, 20.0]   # m/s (0, 25 mph, 45 mph)
COMFORT_ACCEL_V = [0.6, 0.6, 1.0]      # m/s^2


def comfort_accel_cap(v_ego: float) -> float:
  return float(np.interp(v_ego, COMFORT_ACCEL_BP, COMFORT_ACCEL_V))


TURN_EXIT_S = 3.0           # s after a junction turn (fisker_drive_style.TURN_EXIT_S, what the profile measures over)


def follow_t(gap_s: float, v_ego: float) -> float:
  return max(T_FOLLOW_MIN, gap_s - STOP_DISTANCE / max(v_ego, V_FOLLOW_MIN))


def lead_coast(lead_coast_s: float, v_ego: float, lead_present: bool, d_rel: float, v_rel: float) -> bool:
  if lead_coast_s <= 0.0 or not lead_present or v_ego < 1.0:
    return False
  if not (0.0 < d_rel < LEAD_COAST_MAX_D) or v_rel > -LEAD_COAST_CLOSING:
    return False
  return d_rel / v_ego < lead_coast_s


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
