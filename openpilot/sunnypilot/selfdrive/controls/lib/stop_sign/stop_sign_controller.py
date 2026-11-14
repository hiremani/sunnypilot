"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Stop Sign Assist: a comfortable stop at map stop signs, for cars where openpilot controls the speed.

  idle ──(sign within the coast distance)──> coasting ──(model also wants to stop)──> braking ──(v ~ 0)──> stopped
    ^                                                                                                         │
    └──────────────── the driver presses the gas (that sign is then done) / the sign is passed ───────────────┘

* Coast: from 150 ft out (further at speed, so the braking that follows stays gentle) the target drops just
  under the current speed: no throttle, a lift-off slowdown.
* Brake: a constant-deceleration profile, v = sqrt(2 * A_COMFORT * d), ending STOP_MARGIN before the sign's map
  point (OSM puts the sign at the junction, not at the stop line). Only once the driving model also plans a stop:
  a map sign may belong to the crossing road, and then the car should only have coasted.
* Hold: stopped at the sign until the driver presses the gas. It never pulls away by itself.

Shadow mode (StopSignAssistShadow): the same logic on every drive, engaged or not, ignoring the driver's gas, with its
target never handed to the planner. Its state changes go to the log ("stop_sign" events), to check a car's map signs,
GPS and model confirmations before letting it brake.
"""
import math
from enum import IntEnum

from openpilot.common.gps import get_gps_location_service
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_map import StopSignMap, find_stop_ahead

PARAM = "StopSignAssist"
SHADOW_PARAM = "StopSignAssistShadow"  # run and log only, never touching the speed (any car, any cruise)

COAST_START_M = 45.72    # m (150 ft): lift off the gas at least this far out
A_COMFORT = 1.0          # m/s^2 planned braking; the planner's cruise floor is -1.2, so this leaves headroom
COAST_TIME_S = 2.0       # at speed, coast this long before the braking point
COAST_DV = 0.4           # m/s under the current speed while coasting: ~ -0.4 m/s^2, an EV's gentle lift-off
LAG_S = 1.0              # lead the braking profile by the planner's ~1 s speed-loop time constant
STOP_MARGIN_M = 6.0      # stop this far before the sign's map point
STOPPED_V = 0.3          # m/s: at a stop
STOPPED_NEAR_M = 20.0    # ...only counts as stopped *at the sign* this close to it
MODEL_STOP_V = 1.5       # m/s: the model's plan dips below this => it wants to stop
GPS_MAX_AGE_S = 2.0
GPS_MAX_ACCURACY_M = 15.0
MIN_GPS_SPEED = 2.0      # m/s: GPS bearing is unreliable below this, so hold on to the last sign


class StopSignState(IntEnum):
  disabled = 0
  idle = 1
  coasting = 2
  braking = 3
  stopped = 4


def coast_distance(v_ego: float) -> float:
  return max(COAST_START_M, braking_distance(v_ego) + v_ego * COAST_TIME_S)


def braking_distance(v_ego: float) -> float:
  return v_ego ** 2 / (2 * A_COMFORT) + v_ego * LAG_S


def braking_v_target(dist_to_stop: float, v_ego: float) -> float:
  d = max(dist_to_stop - v_ego * LAG_S, 0.0)
  return math.sqrt(2 * A_COMFORT * d)


def model_wants_stop(model) -> bool:
  if model.action.shouldStop:
    return True
  vel = model.velocity.x
  return len(vel) > 0 and min(vel) < MODEL_STOP_V


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
    self.distance = 0.0            # m to the stop point (sign minus margin)
    self.confirmed = False         # the model has planned a stop for this sign
    self.done_ids: set[int] = set()

    self.output_v_target = V_CRUISE_UNSET
    self.output_a_target = 0.0

  @property
  def is_active(self) -> bool:
    return self.state in (StopSignState.coasting, StopSignState.braking, StopSignState.stopped)

  def _reset_sign(self) -> None:
    if self.state != StopSignState.disabled:
      self.state = StopSignState.idle
    self.node_id = None
    self.distance = 0.0
    self.confirmed = False

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
      ahead = find_stop_ahead(gps.latitude, gps.longitude, gps.bearingDeg, signs, self.done_ids)
      if ahead is None:
        # passed it, or turned away: whatever sign we had is done
        if self.node_id is not None:
          self._finish_sign()
        return None
      if ahead.node_id != self.node_id:
        self._reset_sign()
        self.node_id = ahead.node_id
      return ahead.node_id, ahead.distance - STOP_MARGIN_M
    # slow or no fix: keep the current sign, counting down the distance travelled
    if self.node_id is not None:
      return self.node_id, self.distance - v_ego * DT_MDL
    return None

  def update(self, sm, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float, now_s: float) -> None:
    prev = (self.state, self.node_id)
    self._update(sm, long_enabled, long_override, v_ego, a_ego, now_s)
    if (self.state, self.node_id) != prev and self.state != StopSignState.disabled:
      cloudlog.event("stop_sign", shadow=self.shadow, state=self.state.name, prev=prev[0].name, sign=self.node_id,
                     dist=round(self.distance, 1), v=round(v_ego, 2), v_target=round(min(self.output_v_target, 99.0), 2),
                     confirmed=self.confirmed, coast_at=round(coast_distance(v_ego), 1))

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
    _, self.distance = located

    if not self.confirmed and model_wants_stop(sm['modelV2']):
      self.confirmed = True

    if self.state == StopSignState.stopped or (v_ego < STOPPED_V and self.confirmed and self.distance < STOPPED_NEAR_M):
      self.state = StopSignState.stopped
      self.output_v_target = 0.0
    elif not self.is_active and self.distance > coast_distance(v_ego):
      self.state = StopSignState.idle
      self.output_v_target = V_CRUISE_UNSET
    else:
      coast_v = max(v_ego - COAST_DV, 0.0)
      brake_v = braking_v_target(self.distance, v_ego) if self.confirmed else coast_v
      self.state = StopSignState.braking if brake_v < coast_v else StopSignState.coasting
      self.output_v_target = min(coast_v, brake_v)
