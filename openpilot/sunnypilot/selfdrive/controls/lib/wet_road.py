"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Wet road: drive a little more cautiously in the rain.

Rain is read from the car itself, at no CAN cost: the front wiper speed (BCM_0x335 BCM_FrntWiprSpd, wipes a minute; the
Ocean's auto wipers follow its rain sensor) and traction control (ESP_0x114 ESP_ActvSig_TCS), both in messages card
already decodes, published in carStateSP. No camera, no network. The state is evaluated once a second.

  dry   -> wet    wipers running (a wipe at least every WIPE_GAP_S) for WET_ON_S: a single wipe or a washer squirt
                  doesn't count
  wet   -> heavy  the wipers moving HEAVY_DUTY of the time over the last DUTY_WINDOW_S (continuous wiping), or the
                  signal at HEAVY_WPM or more, for HEAVY_ON_S; or traction control stepping in while wet (held
                  TCS_HOLD_S)
  heavy -> wet    none of these for HEAVY_OFF_S

The wiper signal is not wipes a minute: on this car it reads a constant 9 while the blade moves and 0 between wipes
(route 000000fe, 2026-11-16: 395 wipes of ~1.1 s, 2-15 s apart on intermittent/auto, continuous for minutes 6-8 and
20). So how wet it is shows in how much of the time the wipers move, sampled every planner tick.
  wet   -> dry    wipers off for DRY_AFTER_S (the road stays wet after the rain stops)

What each level changes (applied with WetRoadAssist; otherwise the level is only detected, published and logged):
  following gap x GAP_FACTOR, curve lateral acceleration x CURVE_FACTOR, stop sign braking at most STOP_DECEL,
  cruising at most the posted limit + SPEED_OFFSET_MPH, acceleration at most ACCEL_MAX (openpilot longitudinal only);
  heavy also turns auto lane change and the map turn assist off (WetRoadLevel param, read by modeld and controlsd).
WetRoadForce holds "wet" whatever the wipers say.
"""
import math
from collections import deque

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

ASSIST_PARAM = "WetRoadAssist"
FORCE_PARAM = "WetRoadForce"
LEVEL_PARAM = "WetRoadLevel"     # the applied level, for modeld (auto lane change) and controlsd (map turn assist)

DRY, WET, HEAVY = 0, 1, 2
NAMES = ("dry", "wet", "heavy")

EVERY_S = 1.0           # evaluated this often
WIPE_GAP_S = 15.0       # a wipe at least this often is "the wipers are running" (intermittent included)
WET_ON_S = 30.0
HEAVY_WPM = 50          # a signal value this high also counts as fast (in case the fast stalk position reads higher;
                        # not seen yet: every recorded wipe read 9)
HEAVY_DUTY = 0.85       # the wipers moving this fraction of the time...
DUTY_WINDOW_S = 60.0    # ...over this long (and at least DUTY_MIN_S of it seen)
DUTY_MIN_S = 30.0
HEAVY_ON_S = 10.0
HEAVY_OFF_S = 120.0
TCS_HOLD_S = 120.0
DRY_AFTER_S = 300.0
PARAMS_EVERY_S = 3.0

GAP_FACTOR = (1.0, 1.25, 1.5)
CURVE_FACTOR = (1.0, 0.85, 0.7)
STOP_DECEL = (math.inf, 0.8, 0.7)          # m/s^2 planned stop sign braking (earlier, softer)
SPEED_OFFSET_MPH = (None, 0.0, -5.0)       # cruising cap over the posted limit
ACCEL_MAX = (math.inf, 1.2, 0.8)           # m/s^2 (softer pull-away)


class WetRoad:
  def __init__(self, params: Params | None = None, put_level=None):
    self.params = params or Params()
    self.put_level = put_level   # tests; else the LEVEL_PARAM is written (non-blocking) on change
    self.enabled = False
    self.force = False
    self.level = DRY             # detected
    self.applied = DRY           # what the planner uses: level (or wet when forced) with WetRoadAssist, else dry
    self.changed = False         # applied changed on this update
    self.wiper = 0               # the largest wiper signal over the last second
    self.duty = 0.0              # fraction of the last DUTY_WINDOW_S the wipers were moving
    self._on_s = 0.0             # this second: time the wiper signal was non-zero
    self._sec_s = 0.0
    self._sec_max = 0
    self._duty_hist: deque[float] = deque(maxlen=round(DUTY_WINDOW_S / EVERY_S))
    self.tcs = False
    self._t = 0.0
    self._next = 0.0
    self._next_params = 0.0
    self._wipe_last = -math.inf
    self._wiping_since: float | None = None
    self._fast_since: float | None = None
    self._fast_last = -math.inf
    self._tcs_last = -math.inf
    self._written: int | None = None

  def _read_params(self) -> None:
    try:
      self.enabled = self.params.get_bool(ASSIST_PARAM)
      self.force = self.params.get_bool(FORCE_PARAM)
    except Exception:
      self.enabled = self.force = False

  def update(self, wiper_wpm: int, tcs_active: bool, dt: float) -> None:
    """Every planner tick (cheap: it only counts time); the state moves once every EVERY_S."""
    self.changed = False
    self._t += dt
    self._sec_s += dt
    if wiper_wpm > 0:   # every tick: a wipe lasts ~1.1 s, a once-a-second sample would miss some
      self._on_s += dt
      self._sec_max = max(self._sec_max, int(wiper_wpm))
    if self._t < self._next:
      return
    self._next = self._t + EVERY_S
    now = self._t
    if now >= self._next_params:
      self._next_params = now + PARAMS_EVERY_S
      self._read_params()

    self.wiper, self.tcs = self._sec_max, bool(tcs_active)
    self._duty_hist.append(min(1.0, self._on_s / max(self._sec_s, 1e-3)))
    self._on_s = self._sec_s = 0.0
    self._sec_max = 0
    seen_s = len(self._duty_hist) * EVERY_S
    self.duty = sum(self._duty_hist) / len(self._duty_hist)
    if self.wiper > 0:
      self._wipe_last = now
    running = now - self._wipe_last <= WIPE_GAP_S
    if running:
      self._wiping_since = now if self._wiping_since is None else self._wiping_since
    else:
      self._wiping_since = None
    if self.wiper >= HEAVY_WPM or (seen_s >= DUTY_MIN_S and self.duty >= HEAVY_DUTY):
      self._fast_since = now if self._fast_since is None else self._fast_since
      self._fast_last = now
    else:
      self._fast_since = None
    if self.tcs:
      self._tcs_last = now

    prev = self.level
    if self.level == DRY:
      if self._wiping_since is not None and now - self._wiping_since >= WET_ON_S:
        self.level = WET
    if self.level >= WET:
      fast = self._fast_since is not None and now - self._fast_since >= HEAVY_ON_S
      slip = now - self._tcs_last <= TCS_HOLD_S
      if fast or slip:
        self.level = HEAVY
      elif self.level == HEAVY and now - self._fast_last > HEAVY_OFF_S:
        self.level = WET
      if self.level >= WET and now - self._wipe_last > DRY_AFTER_S and not slip:
        self.level = DRY

    applied = max(self.level, WET if self.force else DRY) if (self.enabled or self.force) else DRY
    if applied != self.applied or self.level != prev:
      cloudlog.event("wet_road", level=NAMES[self.level], prev=NAMES[prev], applied=NAMES[applied], wiper=self.wiper,
                     duty=round(self.duty, 2), tcs=self.tcs, enabled=self.enabled, force=self.force)
    self.changed = applied != self.applied
    self.applied = applied
    if self.applied != self._written:
      self._write_level()

  def _write_level(self) -> None:
    try:
      if self.put_level is not None:
        self.put_level(self.applied)
      else:
        self.params.put(LEVEL_PARAM, self.applied)   # non-blocking; only on change
      self._written = self.applied
    except Exception:
      cloudlog.exception("wet road: writing the level failed")

  # -- what the planner applies --

  def set_profile(self, wet_offset_mph: float | None, wet_gap_factor: float | None) -> None:
    """The driver's own wet-road speed (mph over the limit) and following gap factor, measured with the wipers on (driver
    profile wet_offset_mph / wet_gap_factor); None = the defaults. Heavy rain is never less cautious than the default."""
    self.profile_offset = wet_offset_mph
    self.profile_gap = wet_gap_factor

  def gap_factor(self) -> float:
    g = getattr(self, "profile_gap", None)
    if g is not None and self.applied >= WET:
      return max(g, GAP_FACTOR[self.applied]) if self.applied == HEAVY else g
    return GAP_FACTOR[self.applied]

  def curve_factor(self) -> float:
    return CURVE_FACTOR[self.applied]

  def stop_decel(self) -> float:
    return STOP_DECEL[self.applied]

  def speed_offset_mph(self) -> float | None:
    o = getattr(self, "profile_offset", None)
    if o is not None and self.applied >= WET:
      return min(o, SPEED_OFFSET_MPH[self.applied]) if self.applied == HEAVY else o
    return SPEED_OFFSET_MPH[self.applied]

  def accel_max(self) -> float:
    return ACCEL_MAX[self.applied]


def lane_changes_and_turns_ok(params: Params) -> bool:
  """False in heavy rain (the applied level): auto lane change and the map turn assist stay off."""
  try:
    return int(params.get(LEVEL_PARAM, return_default=True) or 0) < HEAVY
  except Exception:
    return True
