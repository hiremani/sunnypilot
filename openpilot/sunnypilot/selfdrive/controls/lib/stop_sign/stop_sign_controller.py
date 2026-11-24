"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Stop Sign Assist: a comfortable stop at map stop signs, for cars where openpilot controls the speed.

  idle ──(sign within the coast distance)──> coasting ──(model also wants to stop)──> braking ──(v ~ 0)──> stopped
    ^                                                                                                         │
    └──────────────── the driver presses the gas (that sign is then done) / the sign is passed ───────────────┘

* Coast: from 150 ft out (further at speed, so the braking that follows stays gentle) no speeding up toward the sign;
  from the coast window (time to brake + coast_s) the target drops just under the current speed: a lift-off
  slowdown. Slow approaches hold their speed until that window instead of decaying to a crawl (lift_off_distance).
* Brake: a constant-deceleration profile, v = sqrt(2 * A_COMFORT * d), ending at zero speed STOP_MARGIN before the
  sign's map point (OSM puts the sign at the junction, not at the stop line). Only once the driving model also
  plans to slow for it (the confirmer below): a map sign may belong to the crossing road, and then the car should
  only have coasted.
* The stop is a complete stop: the target is STOP_V_TARGET = 0 and there is deliberately no rolling-stop option.
  NHTSA recall 22V-037 treated a system that rolled through all-way stops at up to 5.6 mph as a safety defect,
  with no incident reported; what the driver does at a sign is no standard for what the system may do.
* Hold: stopped at the sign until the driver taps the accelerator to proceed (the planner's gas override). It never
  pulls away by itself: whether the junction is clear is the driver's call.

The confirmer (model_stop_cue, StopSignAssist._confirm). The model has no stop sign output, and
modelV2.action.shouldStop is no approach signal: it is the standstill-hold flag (v_ego < 0.3 m/s and a small desired
acceleration), false by construction while the car still moves. A fixed "the plan dips below 1.5 m/s" test fires
late or never. So, as FrogPilot (Conditional Experimental Mode) and sunnypilot DEC do, the plan's *shape* is read,
scaled to the current speed, from modelV2 alone:
  (a) planned-speed ratio: min(velocity.x) <= CONFIRM_SLOW_RATIO * v_ego, or min(velocity.x) < CONFIRM_STOP_V (the
      floor that keeps it working at low speed);
  (b) plan end-point shortfall: position.x[-1] short of DEC's speed-indexed expected distance (dec/constants.py
      SLOW_DOWN_BP / SLOW_DOWN_DIST) by at least CONFIRM_SHORTFALL of it. DEC's table is generous at low speed (32 m
      at standstill), so this one is only read from CONFIRM_SHORTFALL_MIN_V up.
Either cue is one raw frame. Raw frames go through a first-order filter (CONFIRM_FILTER_S) and confirm at
CONFIRM_THRESHOLD, about one time constant of continuous agreement, so a noisy frame confirms nothing. Frames with
a relevant lead (radarState.leadOne within the plan's length + LEAD_MARGIN_M) count as "no cue": the plan is then
slowing for the car in front, and the lead policy handles a queue. Frames are only read once the sign is within
confirm_distance (the coast distance plus one filter time constant of travel): a slowdown 300 m before a sign is
not about that sign. Confirmation latches per sign (self.confirmed) and resets with the sign.
THESE THRESHOLDS ARE STARTING VALUES taken from those forks, not measured on this car or model: tune them from
labelled shadow-drive passes (full stop / rolling stop / no response per map sign; the "stop_sign" and
"stop_sign_confirmed" events carry the raw and filtered values), and again after a driving-model change. Driver-
driven passes flatter the confirmer: the model sees the driver's own braking.

Shadow mode (StopSignAssistShadow): the same logic on every drive, engaged or not, ignoring the driver's gas, with its
target never handed to the planner. Its state changes go to the log ("stop_sign" events, plus "stop_sign_confirmed"
when the confirmer latches), to check a car's map signs, GPS and model confirmations before letting it brake.
"""
import math
from dataclasses import dataclass
from enum import IntEnum

import numpy as np

from openpilot.common.filter_simple import FirstOrderFilter

from openpilot.common.gps import get_gps_location_service
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.dec.constants import WMACConstants
from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_map import StopSignMap, find_stop_ahead

PARAM = "StopSignAssist"
SHADOW_PARAM = "StopSignAssistShadow"  # run and log only, never touching the speed (any car, any cruise)

COAST_START_M = 45.72    # m (150 ft): lift off the gas at least this far out
A_COMFORT = 1.0          # m/s^2 planned braking; the planner's cruise floor is -1.2, so this leaves headroom
COAST_TIME_S = 2.0       # at speed, coast this long before the braking point
COAST_MIN_S = 1.5        # ...and never less, whatever the driver profile says. With 0 s (the profile learned from the
                         # driver's own stops, route 00000103, 2026-11-16) the coast point WAS the braking point, so a
                         # sign came into range already "too close" and every sign approached above ~16 mph was left to
                         # the driver: the car drove through a stop sign at 20 mph
A_TAKE_ON_MAX = 1.2      # m/s^2: a sign is still taken on (and braked for) if stopping needs up to this; beyond it, it is
                         # left to the driver. The planner's cruise floor is -1.2: a speed target can't brake harder
COAST_DV = 0.4           # m/s under the current speed while coasting: ~ -0.4 m/s^2, an EV's gentle lift-off
MAX_APPROACH_V = 11.2    # m/s (25 mph): above this the assist doesn't take on a sign; once active for a sign it carries
                         # on. Was 9.0 (20 mph): the driver profile cruises town streets at 20 mph, right at the cap, so a
                         # sign was taken on only when the speed happened to dip under it (route 00000103)
COAST_FLOOR_V = 2.5      # m/s (~5.6 mph): coasting eases off down to here and no further. Only a confirmed stop brings
                         # the car to a halt; an unconfirmed coast used to decay to 0 and hold the car short of the sign
                         # with no way out but the gas (found with drive_sim's closed loop, 2026-11-14)
LAG_S = 1.6              # lead the braking profile by the time the car takes to brake as asked: the planner's ~1 s
                         # speed loop plus the Fisker's request smoothing (carcontroller.smooth_accel_step, ~0.6 s to
                         # build -1.0). With 1.0 s the car crossed the stop point at 3-4 mph from a 10 mph approach
                         # (00000136 at 0:30, 2026-11-22; simulated with the real smoothing: 0.0 m margin at 8-10 mph)
STOP_MARGIN_M = 6.0      # stop this far before the sign's map point (stop_sign_map.DEFAULT_MARGIN_M; a sign whose
                         # junction has a wider cross street carries its own, StopSign.margin_m)
MODEL_STOP_V = 0.5       # m/s: the driving model's plan has stopped where its speed is under this
MODEL_STOP_EARLY_MAX_M = 12.0   # the model's stop point is taken when it is this much before the map's, at most
MODEL_STOP_MAX_DECEL = 3.0      # m/s^2: a planned stop that would need more than this from the current speed isn't read
MODEL_STOP_TAU_S = 1.0   # s: ...smoothed over this. Never later than the map's point: 4 s before the driver's own
                         # stops the model's point was within 3 m in 94% (95 stops, 2026-11-23), when it had one (56%)
STOP_BUFFER_M = 1.5      # ...and aim the braking profile this much shorter still: it ends at zero speed, so any lag at
                         # the end carried the car past the stop point (00000136 at 0:37: 4.6 mph 2 m before it). With
                         # the Fisker's request smoothing, simulated: 1.5-3.0 m short on three response models, never past
SOFT_V = 2.25            # m/s (5 mph): from here down the stop is a soft landing at SOFT_A, as the driver's stops end
SOFT_A = 0.5             # m/s^2 (the driver's last second: 0.7; openpilot's was 2.0-2.1, a head jolt). The Fisker's VCU
                         # clamps the friction brakes on anything firmer below 3 mph, so its controller sends no more
                         # than this there (opendbc fisker soft_stop_accel) and the profile has to leave room for it
STOP_V_TARGET = 0.0      # m/s: a complete stop. No rolling-stop option on purpose (NHTSA recall 22V-037, see above)
PASSED_M = 10.0          # m beyond the sign's map point: dead-reckoned this far past it, the sign is behind the car
STOPPED_V = 0.3          # m/s: at a stop
STOPPED_NEAR_M = 20.0    # ...only counts as stopped *at the sign* this close to it
# the model confirmer (see the docstring): starting values from FrogPilot CEM / sunnypilot DEC, to be tuned from
# labelled shadow-drive passes
CONFIRM_SLOW_RATIO = 0.70        # (a) the plan's slowest point is at most this fraction of the current speed...
CONFIRM_STOP_V = 1.0             # m/s ...or below this, whatever the current speed
CONFIRM_SHORTFALL = 0.30         # (b) the plan ends this fraction short of DEC's expected distance for the speed
CONFIRM_SHORTFALL_MIN_V = 5.0    # m/s: (b) is not read below this (DEC's table exceeds a steady plan at low speed)
CONFIRM_FILTER_S = 1.5           # s: time constant of the filter over the raw cue
CONFIRM_THRESHOLD = 1.0 - 1.0 / math.e   # ~0.63: the filter gets here after ~CONFIRM_FILTER_S of continuous cue
LEAD_MARGIN_M = 10.0             # a lead no further than the plan's end + this is why the plan slows: no cue
GPS_MAX_AGE_S = 2.0
GPS_MAX_ACCURACY_M = 15.0
MIN_GPS_SPEED = 2.0      # m/s: GPS bearing is unreliable below this, so hold on to the last sign


class StopSignState(IntEnum):
  disabled = 0
  idle = 1
  coasting = 2
  braking = 3
  stopped = 4


def coast_distance(v_ego: float, a: float = A_COMFORT, coast_s: float = COAST_TIME_S) -> float:
  return max(COAST_START_M, braking_distance(v_ego, a) + v_ego * max(coast_s, COAST_MIN_S))


def lift_off_distance(v_ego: float, a: float = A_COMFORT, coast_s: float = COAST_TIME_S) -> float:
  """Where the lift-off coast starts: the time-based part of coast_distance, without the 150 ft floor. Between
  COAST_START_M and here (slow approaches only: at 25 mph this is already ~64 m) the assist holds the speed instead -
  no speeding up toward the sign, and no early decay to COAST_FLOOR_V followed by a crawl. At 7 mph with the stop
  confirmed 28 m out it coasted for 8 s, crawling at 5.6 mph, because the braking profile only starts ~6 m before
  the stop point at that speed (00000116 at 1:55, 2026-11-21: the driver pressed the gas)."""
  return braking_distance(v_ego, a) + v_ego * max(coast_s, COAST_MIN_S)


def braking_distance(v_ego: float, a: float = A_COMFORT) -> float:
  """m to stop from v_ego on the braking profile: a down to SOFT_V, the soft landing below, plus the lag."""
  if v_ego <= SOFT_V:
    return v_ego ** 2 / (2 * SOFT_A) + v_ego * LAG_S
  return (v_ego ** 2 - SOFT_V ** 2) / (2 * a) + SOFT_V ** 2 / (2 * SOFT_A) + v_ego * LAG_S


def required_decel(dist_to_stop: float, v_ego: float) -> float:
  """The steady braking (m/s^2) that stops in dist_to_stop, after the planner's lag."""
  return v_ego ** 2 / (2 * max(dist_to_stop - v_ego * LAG_S, 0.5))


def coast_target(v_ego: float, floor: bool = True) -> float:
  """Easing off the gas: a little under the current speed, but never below COAST_FLOOR_V (or below v_ego, when already
  slower than that: no creeping to a stop on a coast). floor False (the stop is confirmed): no floor, it keeps easing
  down into the braking profile. With the floor a confirmed stop held 5.6 mph - and sped back up to it - until the
  braking profile crossed it ~6 m before the stop point (00000136 at 0:35: +0.17 m/s^2 at 7 m, then the overshoot)."""
  if not floor:
    return max(v_ego - COAST_DV, 0.0)
  return max(v_ego - COAST_DV, min(v_ego, COAST_FLOOR_V))


def braking_v_target(dist_to_stop: float, v_ego: float, a: float = A_COMFORT) -> float:
  """The braking profile's speed target: it ends at STOP_V_TARGET (zero) at the stop point, never at a rolling speed."""
  d = max(dist_to_stop - STOP_BUFFER_M - v_ego * LAG_S, 0.0)
  d_soft = SOFT_V ** 2 / (2 * SOFT_A)          # the last metres: the soft landing from SOFT_V
  if d <= d_soft:
    return max(math.sqrt(2 * SOFT_A * d), STOP_V_TARGET)
  return math.sqrt(SOFT_V ** 2 + 2 * a * (d - d_soft))


def confirm_distance(v_ego: float, a: float = A_COMFORT, coast_s: float = COAST_TIME_S) -> float:
  """The confirmer listens from here in: early enough for the filter to settle before the coast begins."""
  return coast_distance(v_ego, a, coast_s) + v_ego * CONFIRM_FILTER_S


@dataclass(frozen=True)
class StopCue:
  """One model frame read for "the plan slows for something ahead". raw is what the filter is fed."""
  v_min: float | None = None       # m/s: the slowest point of the plan; None without a plan
  shortfall: float = 0.0           # fraction of the expected distance the plan's end falls short by (0 when not read)
  slowing: bool = False            # cue (a)
  short: bool = False              # cue (b)
  stop_x: float | None = None      # m ahead where the plan comes to a stop (MODEL_STOP_V); None when it doesn't

  @property
  def raw(self) -> bool:
    return self.slowing or self.short


def model_stop_cue(model, v_ego: float) -> StopCue:
  """Does this frame's plan slow for something ahead? From modelV2 velocity.x / position.x only, scaled to v_ego.
  action.shouldStop is deliberately not read (a standstill-hold flag, see the docstring)."""
  vel = getattr(getattr(model, "velocity", None), "x", None)
  pos = getattr(getattr(model, "position", None), "x", None)
  v_min = float(min(vel)) if vel is not None and len(vel) > 0 else None
  slowing = v_min is not None and (v_min <= CONFIRM_SLOW_RATIO * v_ego or v_min < CONFIRM_STOP_V)
  shortfall = 0.0
  if pos is not None and len(pos) > 0 and v_ego >= CONFIRM_SHORTFALL_MIN_V:
    expected = float(np.interp(v_ego * 3.6, WMACConstants.SLOW_DOWN_BP, WMACConstants.SLOW_DOWN_DIST))
    shortfall = max(0.0, 1.0 - float(pos[-1]) / expected)
  stop_x = None
  if vel is not None and pos is not None and len(vel) == len(pos):
    stop_x = next((float(pos[i]) for i in range(len(vel)) if vel[i] < MODEL_STOP_V), None)
  return StopCue(v_min, shortfall, slowing, shortfall >= CONFIRM_SHORTFALL, stop_x)


def lead_relevant(sm, model) -> bool:
  """A lead close enough to be what the plan slows for. False when radarState isn't there (a test's dict, a
  SubMaster without it)."""
  try:
    lead = sm['radarState'].leadOne
  except (KeyError, AttributeError, TypeError):
    return False
  if not getattr(lead, "present", getattr(lead, "status", False)):
    return False
  pos = getattr(getattr(model, "position", None), "x", None)
  plan_end = float(pos[-1]) if pos is not None and len(pos) > 0 else 0.0
  return bool(lead.dRel <= max(plan_end, 0.0) + LEAD_MARGIN_M)


class StopSignAssist:
  def __init__(self, stop_map: StopSignMap | None = None, params: Params | None = None, shadow: bool = False):
    self.params = params or Params()
    self.shadow = shadow
    self.param = SHADOW_PARAM if shadow else PARAM
    self.enabled = self.params.get_bool(self.param)
    self.gps_service = get_gps_location_service(self.params)
    self.map = stop_map or StopSignMap()
    self.frame = -1

    self.state = StopSignState.disabled
    self.node_id: int | None = None
    self.distance = 0.0            # m to the stop point (sign minus margin, minus what the model's stop point adds)
    self.map_distance = 0.0        # m to the map's stop point (sign minus margin): what GPS and dead reckoning give
    self.margin = STOP_MARGIN_M    # this sign's margin (wider cross street: more)
    self.stop_early = 0.0          # m the model's stop point is before the map's, smoothed (0: the map's point)
    self.confirmed = False         # the model has planned to slow for this sign (latched until the sign changes)
    self.too_late = False          # ...but only once a comfortable stop was no longer possible: never brake for it
    self.max_approach_v = MAX_APPROACH_V
    # braking rate and coast time: the constants, or the active driver profile's (set_profile; always within the
    # profile's bounds, 0.7-1.5 m/s^2 and 0-4 s)
    self.a_comfort = A_COMFORT
    self.coast_s = COAST_TIME_S
    self.a_brake = A_COMFORT       # this sign's braking: a_comfort, or more (up to A_TAKE_ON_MAX) when taken on late
    self.coast_t = 0.0             # s of lift-off coast so far for this sign (a confirmed stop coasts coast_s, then holds)
    self.gave_up = 0               # signs left to the driver because a comfortable stop was no longer possible
    self.confirm_filter = FirstOrderFilter(0.0, CONFIRM_FILTER_S, DT_MDL)
    self.cue = StopCue()           # the last frame the confirmer read, for the log
    self.cue_lead = False          # ...and whether a relevant lead made it count as "no cue"
    self.done_ids: set[int] = set()

    self.output_v_target = V_CRUISE_UNSET
    self.output_a_target = 0.0

  def set_profile(self, stop_decel: float, coast_s: float) -> None:
    self.a_comfort = float(stop_decel)
    self.coast_s = float(coast_s)

  @property
  def a_limit(self) -> float:
    """The firmest braking a sign is still taken on for (never below the profile's own comfortable rate)."""
    return max(A_TAKE_ON_MAX, self.a_comfort)

  @property
  def is_active(self) -> bool:
    return self.state in (StopSignState.coasting, StopSignState.braking, StopSignState.stopped)

  def _reset_sign(self) -> None:
    if self.state != StopSignState.disabled:
      self.state = StopSignState.idle
    self.node_id = None
    self.distance = 0.0
    self.map_distance = 0.0
    self.margin = STOP_MARGIN_M
    self.stop_early = 0.0
    self.confirmed = False
    self.too_late = False
    self.a_brake = self.a_comfort
    self.coast_t = 0.0          # s of lift-off coast so far for this sign (a confirmed stop coasts coast_s, then holds)
    self.confirm_filter.x = 0.0
    self.cue = StopCue()
    self.cue_lead = False

  def _give_up(self, v_ego: float, why: str) -> None:
    """No comfortable stop is possible for this sign any more: leave it to the driver, completely (idle, no coasting,
    no target), and don't take it on again."""
    cloudlog.event("stop_sign_left_to_driver", shadow=self.shadow, sign=self.node_id, dist=round(self.distance, 1),
                   v=round(v_ego, 2), why=why)
    self.gave_up += 1
    self._finish_sign()
    self.state = StopSignState.idle
    self.output_v_target = V_CRUISE_UNSET

  def _finish_sign(self) -> None:
    if self.node_id is not None:
      self.done_ids.add(self.node_id)
    self._reset_sign()

  def _locate(self, gps, v_ego: float, now_s: float):
    """(sign id, distance to its stop point) or None. Dead-reckons while GPS bearing is unusable."""
    fix_ok = (gps.hasFix and gps.horizontalAccuracy <= GPS_MAX_ACCURACY_M and
              now_s - gps.unixTimestampMillis * 1e-3 <= GPS_MAX_AGE_S)
    if fix_ok and gps.speed >= MIN_GPS_SPEED:
      signs = self.map.signs_near(gps.latitude, gps.longitude)
      ahead = find_stop_ahead(gps.latitude, gps.longitude, gps.bearingDeg, signs, self.done_ids, prefer_id=self.node_id)
      if ahead is None:
        # passed it, or turned away: whatever sign we had is done
        if self.node_id is not None:
          self._finish_sign()
        return None
      if ahead.node_id != self.node_id:
        self._reset_sign()
        self.node_id = ahead.node_id
      self.margin = getattr(ahead, "margin", STOP_MARGIN_M)
      return ahead.node_id, ahead.distance - self.margin
    # slow or no fix: keep the current sign, counting down the distance travelled, until the car is clearly past it
    if self.node_id is not None:
      dist = self.map_distance - v_ego * DT_MDL
      if dist < -(self.margin + PASSED_M):
        self._finish_sign()
        return None
      return self.node_id, dist
    return None

  def update(self, sm, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float, now_s: float) -> None:
    prev = (self.state, self.node_id)
    self._update(sm, long_enabled, long_override, v_ego, a_ego, now_s)
    if (self.state, self.node_id) != prev and self.state != StopSignState.disabled:
      cloudlog.event("stop_sign", shadow=self.shadow, state=self.state.name, prev=prev[0].name, sign=self.node_id,
                     dist=round(self.distance, 1), v=round(v_ego, 2), v_target=round(min(self.output_v_target, 99.0), 2),
                     confirmed=self.confirmed, coast_at=round(coast_distance(v_ego, self.a_comfort, self.coast_s), 1), **self._confirm_log())

  def _confirm_log(self) -> dict:
    """The confirmer's raw and filtered values, for tuning its thresholds from shadow drives."""
    return {"confirm_raw": self.cue.raw, "confirm_filt": round(self.confirm_filter.x, 3),
            "plan_v_min": None if self.cue.v_min is None else round(self.cue.v_min, 2),
            "plan_shortfall": round(self.cue.shortfall, 3), "lead": self.cue_lead}

  def _model_stop_point(self, sm, v_ego: float) -> None:
    """Where the driving model's plan stops, when that is before the map's stop point (up to MODEL_STOP_EARLY_MAX_M):
    the camera's view of the junction, a wider cross street or a set-back stop line the map's point doesn't know.
    Only ever earlier, smoothed, and not while the plan is stopping for a car in front."""
    try:
      model = sm['modelV2']
      cue = model_stop_cue(model, v_ego)
      # no stop in the plan, a stop for a lead, or one no car could make from this speed (not a plan for this car
      # now): keep what there is
      if cue.stop_x is None or lead_relevant(sm, model) or cue.stop_x < v_ego ** 2 / (2 * MODEL_STOP_MAX_DECEL):
        return
      early = self.map_distance - cue.stop_x
      target = early if 0.0 < early <= MODEL_STOP_EARLY_MAX_M else 0.0
      self.stop_early += (target - self.stop_early) * min(1.0, DT_MDL / MODEL_STOP_TAU_S)
    except Exception:
      pass   # a test's dict without modelV2: the map's point

  def _confirm(self, sm, v_ego: float) -> None:
    """Feed this frame to the confirmer and latch self.confirmed for the current sign once the filter agrees."""
    if not (self.is_active or self.distance <= confirm_distance(v_ego, self.a_comfort, self.coast_s)):
      self.confirm_filter.x = 0.0   # too far out for a slowdown to be about this sign
      self.cue, self.cue_lead = StopCue(), False
      return
    model = sm['modelV2']
    self.cue = model_stop_cue(model, v_ego)
    self.cue_lead = self.cue.raw and lead_relevant(sm, model)
    self.confirm_filter.update(float(self.cue.raw and not self.cue_lead))
    if self.confirm_filter.x >= CONFIRM_THRESHOLD:
      self.confirmed = True
      # Too late for a comfortable stop: never brake for it. On route 000000e5 (2026-11-14) a cross-street sign
      # confirmed at 2.6 m and 24 mph; braking to a stop from there would be an emergency stop at a junction the driver
      # was driving through. The driver handles this sign.
      self.too_late = self.distance < braking_distance(v_ego, self.a_limit)
      if not self.too_late:
        self.a_brake = max(self.a_brake, min(required_decel(self.distance, v_ego), self.a_limit))
      cloudlog.event("stop_sign_confirmed", shadow=self.shadow, sign=self.node_id, dist=round(self.distance, 1),
                     v=round(v_ego, 2), state=self.state.name, too_late=self.too_late, **self._confirm_log())

  def _update(self, sm, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float, now_s: float) -> None:
    self.frame += 1
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enabled = self.params.get_bool(self.param)

    self.output_a_target = a_ego
    if not (self.enabled and long_enabled):
      self.state = StopSignState.disabled
      self._reset_sign()
      self.output_v_target = V_CRUISE_UNSET
      return

    if long_override and self.is_active:
      # the driver pressed the gas: they're taking this one (drive off from the stop, or a sign the map got wrong)
      self._finish_sign()
      self.state = StopSignState.idle

    located = self._locate(sm[self.gps_service], v_ego, now_s)
    if located is None:
      self.state = StopSignState.idle
      self.output_v_target = V_CRUISE_UNSET
      return
    _, self.map_distance = located
    self._model_stop_point(sm, v_ego)
    self.distance = self.map_distance - self.stop_early

    if not self.confirmed:
      self._confirm(sm, v_ego)
    if self.too_late:
      self._give_up(v_ego, "confirmed too late")
      return

    may_brake = self.confirmed and not self.too_late
    if self.state == StopSignState.stopped or (v_ego < STOPPED_V and may_brake and self.distance < STOPPED_NEAR_M):
      # held here until the driver taps the accelerator to proceed (long_override above); never released by itself
      self.state = StopSignState.stopped
      self.output_v_target = STOP_V_TARGET
    elif not self.is_active and (self.distance > coast_distance(v_ego, self.a_comfort, self.coast_s) or v_ego > self.max_approach_v):
      self.state = StopSignState.idle
      self.output_v_target = V_CRUISE_UNSET
    else:
      if not self.is_active:
        if self.distance < braking_distance(v_ego, self.a_limit):
          # Taking a sign on (it just came into range, or the driver just slowed under max_approach_v) with no room
          # left even for A_TAKE_ON_MAX: leave it to the driver, however early the model confirmed. Found with
          # drive_sim's closed loop: a fast approach taken on late overshot the stop point by 7.6 m.
          self.too_late = True
          self._give_up(v_ego, "taken on too close")
          return
        # a little late (inside the comfortable braking distance, not past A_TAKE_ON_MAX): brake for it a little
        # firmer, on a profile that can actually stop in the room left
        self.a_brake = max(self.a_comfort, min(required_decel(self.distance, v_ego), self.a_limit))
      # hold the speed until the time-based coast window (slow approaches start inside 150 ft), then lift off
      if self.distance > lift_off_distance(v_ego, self.a_comfort, self.coast_s):
        coast_v = v_ego                                   # engaged: no speeding up toward the sign, no decay yet
      elif may_brake:
        # a confirmed stop: lift off for the driver's coast time, then hold until the braking profile takes over. A
        # coast that went on (to the floor, or without one, toward zero) rolled in slowly and braked late
        coast_v = coast_target(v_ego, floor=False) if self.coast_t < max(self.coast_s, COAST_MIN_S) else v_ego
        self.coast_t += DT_MDL
      else:
        coast_v = coast_target(v_ego)                     # not confirmed: coast, never below the floor

      brake_v = braking_v_target(self.distance, v_ego, self.a_brake) if may_brake else coast_v
      self.state = StopSignState.braking if brake_v < coast_v else StopSignState.coasting
      self.output_v_target = min(coast_v, brake_v)
