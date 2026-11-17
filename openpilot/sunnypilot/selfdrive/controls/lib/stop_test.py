"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Low-speed stop test (openpilot speed control plan, Phases 0 and 2): soften openpilot's braking request in the last few
mph before a stop, to find out whether the car's own extra braking there follows the request.

Recorded stops (stops.py, 2026-11-17): below ~4 mph the car decelerated at -2.2..-2.9 m/s^2 while openpilot asked for
-0.8..-1.0 and the OEM ACC for ~0, then let go at once at 0 mph: the "head nod" (final jerk 18-20 m/s^3, the driver's
own pedal stops ~1-2). If a softer request below 5 mph makes the car's braking softer, Phase 2 shapes the request; if
not, the car's low-speed stop function does it on its own and Phase 2 works around that instead.

FiskerStopTest (int, cleared at every start, so it is off unless set for a test drive):
  0  off: openpilot's request unchanged
  1  soft: at most SOFT_DECEL below SOFT_BELOW_MPS
  2  early ease: from EASE_FROM_MPS down, at most a braking that eases linearly to EASE_END_DECEL at 0
Only while moving (the standstill hold is unchanged) and only when the planner is not asking for firm braking
(aTarget above FIRM_DECEL): a lead car closing in always gets openpilot's full request.
"""
PARAM = "FiskerStopTest"

SOFT_BELOW_MPS = 2.2      # 5 mph
SOFT_DECEL = -0.3         # m/s^2
EASE_FROM_MPS = 3.6       # 8 mph
EASE_START_DECEL = -1.0
EASE_END_DECEL = -0.5
FIRM_DECEL = -1.2         # the planner asking for this or more: no softening
MOVING_MPS = 0.15


def soften(accel: float, v_ego: float, a_target: float, variant: int) -> float:
  """The braking request to send: `accel` (m/s^2, openpilot's), softened per `variant` near the end of a stop."""
  if variant not in (1, 2) or accel >= 0.0 or v_ego < MOVING_MPS or a_target <= FIRM_DECEL:
    return accel
  if variant == 1:
    return max(accel, SOFT_DECEL) if v_ego < SOFT_BELOW_MPS else accel
  if v_ego >= EASE_FROM_MPS:
    return accel
  floor = EASE_END_DECEL + (EASE_START_DECEL - EASE_END_DECEL) * (v_ego / EASE_FROM_MPS)
  return max(accel, floor)
