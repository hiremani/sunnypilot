"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Two driver alerts that need a little state:
  speed limit posted: a chime when the posted limit changes while driving (not at the drive's first reading, not when
    a reading drops out and comes back the same)
  blinker into a blind spot: the blinker is on toward a side whose blind spot is occupied, at any speed. openpilot's own
    laneChangeBlocked only fires during a lane change (above the lane change speed); this covers turns in town.
"""

LIMIT_MIN_V = 2.0          # m/s: only chime while driving
LIMIT_CHANGE_MS = 1.0      # m/s (~2 mph): smaller changes are rounding noise between km/h and mph sources
LIMIT_HOLD_S = 1.0         # a new limit must hold this long before it chimes (a sign misread for one frame doesn't)
BLIND_SPOT_HOLD_S = 0.3    # the blind spot must be occupied this long with the blinker on


class SpeedLimitPosted:
  def __init__(self, dt: float):
    self.dt = dt
    self.announced = 0.0    # m/s, the limit last chimed (or the first one seen); 0: none yet
    self._candidate = 0.0
    self._held = 0.0

  def update(self, limit: float, valid: bool, v_ego: float) -> bool:
    """True on the frame a new posted limit should chime. limit in m/s."""
    if not valid or limit <= 0.0:
      self._held = 0.0
      return False
    if self.announced == 0.0:
      self.announced = limit   # the drive's first reading: no chime
      return False
    if abs(limit - self.announced) < LIMIT_CHANGE_MS:
      self._held = 0.0
      return False
    if abs(limit - self._candidate) >= LIMIT_CHANGE_MS:
      self._candidate, self._held = limit, 0.0
    self._held += self.dt
    if self._held < LIMIT_HOLD_S or v_ego < LIMIT_MIN_V:
      return False
    self.announced = limit
    self._held = 0.0
    return True


class BlinkerBlindSpot:
  def __init__(self, dt: float):
    self.dt = dt
    self._held = 0.0

  def update(self, left_blinker: bool, right_blinker: bool, left_blindspot: bool, right_blindspot: bool) -> bool:
    risky = (left_blinker and left_blindspot) or (right_blinker and right_blindspot)
    self._held = self._held + self.dt if risky else 0.0
    return self._held >= BLIND_SPOT_HOLD_S
