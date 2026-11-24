"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Red Light Assist: lift off early and stop for a red or amber light, from the car's own traffic light recognition
(carStateSP.trafficLightColor / trafficLightDistance: the ego lane's round light, and the distance to its signal head).

Why. The driver lifts off about 17 s before a stop at a light, on seeing the red and the queue; openpilot only reacts
to what its plan or a (vision-only) lead says, late: a stopped car at a red light first read as closing 11 s out, the
slowdown reached 2.2 m/s^2 (00000129 at 3:55). The car's camera reports the colour 8 s or more before the stop in 91 of
117 red-light stops (40 drives, 2026-11-23). The driver's rule: follow the car in front, except when the light turns
non-green; lift off first, brake only if needed and gradually.

  idle -(red or amber for ON_S, in range, room to stop)-> approach -(v ~ 0)-> stopped -(green / gas)-> idle
                    green, the light gone, passed, or the gas: idle (that light is done until it changes)

Where to stop. The distance is to the signal head, which hangs beyond the junction: with the car stopped at the line
it read 10-60 m, median 35 (middle half 28-43). So:
  * the stop point starts at the light's distance minus SETBACK_M (the far end of that range: early, never in the
    junction), dead-reckoned and smoothed while the light is in view;
  * the driving model's own stop point takes over when its plan stops near there (stop_sign_controller.model_stop_cue:
    within 3 m of the driver's stops in 94% four seconds out, when it has one). That is the camera's stop line;
  * a car in front nearer than the stop point: the MPC stops behind it, this only keeps the early lift-off.

How it slows: Stop Sign Assist's profile (same module's functions): hold the speed from the engage range in, lift off
for the driver's coast time, then brake at the driver's rate, ending in the soft landing; a complete stop, held until
the light is green or the driver takes it.

When there is no room for that (the camera first reports many lights around 100 m out; a comfortable stop from 30 mph
with its margins wants 99 m to the stop point): a RED still gets the lift-off at once (the driver's first move, and it
is a red: the car will have to stop), and the braking profile as soon as the falling speed makes it fit; whatever is
still needed beyond that is the MPC's and the driver's, as before, from a lower speed. An AMBER too close to stop for
is left alone (the dilemma zone: slowing into a junction on amber is worse than going).

It always runs and logs (red_light events); it only sets the speed with the RedLightAssist param.
"""
from enum import IntEnum

from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign import stop_sign_controller as ssc

PARAM = "RedLightAssist"

NONE, RED, AMBER, GREEN = 0, 1, 2, 3
STOP_COLORS = (1, 2, 4, 7)     # red, amber, red+orange (about to go green in some countries: still a stop), red with amber
SETBACK_M = 43.0               # m from the signal head back to the stop point, until the model's stop point is known
SETBACK_MIN_M = 10.0           # ...and the least the model's stop point may put it (the closest the driver stopped to a head)
MAX_D = 200.0                  # m: lights further than this are not acted on
ON_S = 0.6                     # s of red/amber before it counts (one frame of another lane's light is nothing)
GREEN_S = 0.4                  # s of green before it is released
LOST_S = 4.0                   # s without the light in view (moving) before it is given up
PASSED_M = 12.0                # m past the stop point: behind the car
MODEL_WINDOW_M = 20.0          # the model's stop point is believed within this of the current stop point
MODEL_TAU_S = 0.8              # s: ...and blended in over this
LIGHT_TAU_S = 1.0              # s: smoothing of the stop point from the light's distance
STOPPED_V = 0.3                # m/s


class RedLightState(IntEnum):
  idle = 0
  approach = 1
  stopped = 2


class RedLightAssist:
  def __init__(self, params: Params | None = None):
    self.params = params or Params()
    self.frame = -1
    self.enabled = self._read_param()
    self.a_comfort = ssc.A_COMFORT
    self.coast_s = ssc.COAST_TIME_S
    self.a_brake = ssc.A_COMFORT
    self.state = RedLightState.idle
    self.output_v_target = V_CRUISE_UNSET
    self.output_a_target = 0.0
    self._logged = None
    self._reset()

  def _read_param(self) -> bool:
    try:
      return bool(self.params.get_bool(PARAM))
    except Exception:
      return False   # code newer than the built params: off

  def _reset(self) -> None:
    self.state = RedLightState.idle
    self.stop_d = 0.0          # m to the stop point
    self.red_t = 0.0
    self.green_t = 0.0
    self.lost_t = 0.0
    self.coast_t = 0.0
    self.model_point = False   # the stop point is the driving model's
    self.braking = False       # on the braking profile (there is room for it); else only lifting off
    self.done = False          # this light was released or left to the driver: nothing until it has gone or changed
    self.reason = ""

  def set_profile(self, stop_decel: float, coast_s: float) -> None:
    self.a_comfort, self.coast_s = stop_decel, coast_s

  @property
  def is_active(self) -> bool:
    return self.state != RedLightState.idle

  @property
  def controls(self) -> bool:
    """Setting the speed: active and switched on."""
    return self.enabled and self.is_active

  def _finish(self, reason: str, done: bool = True) -> None:
    self._reset()
    self.done, self.reason = done, reason

  def _log(self, v_ego: float, color: int, light_d: float) -> None:
    key = (self.state, self.reason, self.model_point)
    if key == self._logged:
      return
    self._logged = key
    try:
      cloudlog.event("red_light", state=self.state.name, reason=self.reason, controls=self.enabled, v=round(v_ego, 2),
                     stop_dist=round(self.stop_d, 1), light_dist=round(light_d, 1), color=int(color), model_point=self.model_point)
    except Exception:
      pass   # logging must never take the planner down

  def update(self, sm, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float, color: int, light_d: float) -> None:
    self.frame += 1
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enabled = self._read_param()
    self.output_v_target, self.output_a_target = V_CRUISE_UNSET, a_ego
    try:
      self._update(sm, long_enabled, long_override, v_ego, color, light_d)
    except Exception:
      cloudlog.exception("red light assist failed")
      self._reset()
    self._log(v_ego, color, light_d)

  def _update(self, sm, long_enabled: bool, long_override: bool, v_ego: float, color: int, light_d: float) -> None:
    seen = color != NONE and 0.0 < light_d <= MAX_D
    stop_color = seen and color in STOP_COLORS
    self.red_t = self.red_t + DT_MDL if stop_color else max(0.0, self.red_t - DT_MDL)
    self.green_t = self.green_t + DT_MDL if (seen and color == GREEN) else 0.0
    if not seen and self.state == RedLightState.idle:
      self.done = False          # the light has gone: the next one starts fresh
    if not long_enabled:
      if self.is_active:
        self._finish("not engaged", done=False)
      return
    if long_override and self.is_active:
      self._finish("driver on the gas")   # going on a red they judged differently, or pulling away: theirs
      return

    if self.state == RedLightState.idle:
      if self.done or self.red_t < ON_S:
        if self.green_t >= GREEN_S:
          self.done = False
        return
      stop_d = light_d - SETBACK_M
      fits = stop_d >= ssc.braking_distance(v_ego, ssc.A_TAKE_ON_MAX)
      if not fits and (color == AMBER or stop_d < 0.0):
        # an amber with no room for a comfortable stop (the dilemma zone), or a light first read at the junction
        self._finish("too close to stop for")
        return
      if stop_d > ssc.coast_distance(v_ego, self.a_comfort, self.coast_s):
        return                     # in view, still far: nothing yet (the timers keep running)
      self.state, self.stop_d, self.reason = RedLightState.approach, stop_d, ""
      self.lost_t = self.coast_t = 0.0
      self.model_point, self.braking = False, False

    # ---- approaching or stopped ----
    if self.green_t >= GREEN_S:
      self._finish("green", done=False)
      return
    if self.state == RedLightState.stopped:
      self.output_v_target = ssc.STOP_V_TARGET     # held until the green (above) or the gas
      return

    self.stop_d -= v_ego * DT_MDL
    if stop_color:
      self.lost_t = 0.0
      if not self.model_point:
        self.stop_d += ((light_d - SETBACK_M) - self.stop_d) * min(1.0, DT_MDL / LIGHT_TAU_S)
    else:
      self.lost_t += DT_MDL
      if self.lost_t > LOST_S and v_ego > 1.0:
        self._finish("light out of view")
        return
    self._model_stop_point(sm, v_ego, light_d if stop_color else None)
    if self.stop_d < -PASSED_M:
      self._finish("passed")
      return
    if v_ego < STOPPED_V and self.stop_d < ssc.STOPPED_NEAR_M:
      self.state = RedLightState.stopped
      self.output_v_target = ssc.STOP_V_TARGET
      return

    if self._lead_before_stop(sm):
      # a car in front, nearer than the stop point: the MPC stops behind it. Keep the lift-off, leave the braking
      self.output_v_target = v_ego
      return
    if not self.braking:
      self.braking = self.stop_d >= ssc.braking_distance(v_ego, ssc.A_TAKE_ON_MAX)
      if self.braking:   # a little late (inside the comfortable distance): a profile that stops in the room there is
        self.a_brake = max(self.a_comfort, min(ssc.required_decel(self.stop_d, v_ego), ssc.A_TAKE_ON_MAX))
    if not self.braking:
      # no room yet for the braking profile: the lift-off alone, never below the coast floor (it can't stop the car on
      # its own from here; the profile takes over once the falling speed makes it fit)
      self.output_v_target = ssc.coast_target(v_ego)
      return
    if self.stop_d > ssc.lift_off_distance(v_ego, self.a_comfort, self.coast_s):
      coast_v = v_ego                                                   # engaged: no speeding up toward a red light
    elif self.coast_t < max(self.coast_s, ssc.COAST_MIN_S):
      coast_v = ssc.coast_target(v_ego, floor=False)                    # lift off, for the driver's coast time
      self.coast_t += DT_MDL
    else:
      coast_v = v_ego
    brake_v = ssc.braking_v_target(self.stop_d, v_ego, self.a_brake)
    self.output_v_target = min(coast_v, brake_v)

  def _model_stop_point(self, sm, v_ego: float, light_d: float | None) -> None:
    """The driving model's stop point, when its plan stops near where the light puts the stop (and no closer to the
    signal head than a car ever stops): the camera's stop line, in place of the setback's estimate."""
    try:
      model = sm['modelV2']
      cue = ssc.model_stop_cue(model, v_ego)
      if cue.stop_x is None or ssc.lead_relevant(sm, model) or cue.stop_x < v_ego ** 2 / (2 * ssc.MODEL_STOP_MAX_DECEL):
        return
      if abs(cue.stop_x - self.stop_d) > MODEL_WINDOW_M:
        return
      if light_d is not None and cue.stop_x > light_d - SETBACK_MIN_M:
        return                     # beyond where any stop line is: the plan stops for something past the light
      self.stop_d += (cue.stop_x - self.stop_d) * min(1.0, DT_MDL / MODEL_TAU_S)
      self.model_point = True
    except Exception:
      pass   # no modelV2 (a test's dict): the light's distance alone

  def _lead_before_stop(self, sm) -> bool:
    try:
      lead = sm['radarState'].leadOne
      return bool(lead.present) and 0.0 < lead.dRel < self.stop_d + ssc.LEAD_MARGIN_M
    except Exception:
      return False
