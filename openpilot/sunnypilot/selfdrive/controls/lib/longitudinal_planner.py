"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import time

from openpilot.cereal import messaging, custom
from opendbc.car import structs
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.sunnypilot.selfdrive.controls.lib.dec.dec import DynamicExperimentalController
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_alerts_helper import E2EAlertsHelper
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.smart_cruise_control import SmartCruiseControl
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import SpeedLimitAssist
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_resolver import SpeedLimitResolver
from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_controller import StopSignAssist
from openpilot.sunnypilot.selfdrive.controls.lib.cpu_budget import CpuBudget
from openpilot.sunnypilot.selfdrive.controls.lib.speed_bump import SpeedBumpAssist
from openpilot.sunnypilot.selfdrive.controls.lib.map_caution import MapCaution
from openpilot.sunnypilot.selfdrive.controls.lib import driver_profile, narrow_street, profile_long
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.models.helpers import get_active_bundle

DecState = custom.LongitudinalPlanSP.DynamicExperimentalControl.DynamicExperimentalControlState
LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource


class LongitudinalPlannerSP:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP, mpc):
    self.events_sp = EventsSP()
    self.resolver = SpeedLimitResolver()
    self.dec = DynamicExperimentalController(CP, mpc)
    self.scc = SmartCruiseControl()
    self.resolver = SpeedLimitResolver()
    self.sla = SpeedLimitAssist(CP, CP_SP)
    # Stop signs need openpilot to own the brakes; with the car's own cruise it stays off. The shadow copy only logs.
    # Neither may keep plannerd from starting (e.g. code newer than the built params): without them, no stop signs.
    self.stop_sign = None
    self.stop_sign_shadow = None
    try:
      self.stop_sign = StopSignAssist() if CP.openpilotLongitudinalControl else None
      self.stop_sign_shadow = StopSignAssist(stop_map=self.stop_sign.map if self.stop_sign else None, shadow=True)
    except Exception:
      cloudlog.exception("stop sign assist unavailable")
    # Speed bumps: the live copy eases off only with openpilot longitudinal; the shadow copy runs on every drive, logs,
    # and is where the "Speed bump ahead" warning comes from whatever drives the speed.
    self.speed_bump = None
    self.speed_bump_shadow = None
    try:
      self.speed_bump = SpeedBumpAssist() if CP.openpilotLongitudinalControl else None
      self.speed_bump_shadow = SpeedBumpAssist(bump_map=self.speed_bump.map if self.speed_bump else None, shadow=True)
    except Exception:
      cloudlog.exception("speed bump assist unavailable")
    # map caution (traffic lights, crosswalks on narrow streets): openpilot longitudinal only
    self.map_caution = None
    try:
      self.map_caution = MapCaution() if CP.openpilotLongitudinalControl else None
    except Exception:
      cloudlog.exception("map caution unavailable")
    # CPU budgets (cpu_budget.py): an extra that runs long is paused, never the planner. Map caution can cap the speed,
    # so a pause leaves it neutral.
    self.budget_stop_shadow = CpuBudget("stop_sign_shadow")
    self.budget_bump_shadow = CpuBudget("speed_bump_shadow", on_pause=self._bump_shadow_quiet)
    self.budget_map_caution = CpuBudget("map_caution", on_pause=self._map_caution_neutral)
    self.generation = int(model_bundle.generation) if (model_bundle := get_active_bundle()) else None
    # Narrow streets (narrow_street.py): always detected and published; the speed cap only with NarrowStreetAssist
    # and openpilot longitudinal. Re-read the param about once a second.
    self.openpilot_long = CP.openpilotLongitudinalControl
    self.narrow = narrow_street.NarrowStreetDetector(DT_MDL)
    self.narrow_street = False
    self.narrow_enabled = False
    self._narrow_param_frame = 0
    # The active driver profile (driver_profile.py; defaults unless DriverProfile is set), re-read every PROFILE_EVERY
    # frames so a switch in the web HUD applies within seconds.
    self.profile = driver_profile.DriverProfile()
    self.profile_on = False
    self.profile_speed_on = False
    self.turn_exit_left = 0.0     # s left of the pull-away window after a junction turn
    self.narrow_speed = narrow_street.NARROW_SPEED
    self.lead_coasting = False
    self.source = LongitudinalPlanSource.cruise
    self.e2e_alerts_helper = E2EAlertsHelper()

    self.output_v_target = 0.
    self.output_a_target = 0.

  def is_e2e(self, sm: messaging.SubMaster) -> bool:
    experimental_mode = sm['selfdriveState'].experimentalMode
    if not self.dec.active():
      return experimental_mode

    return experimental_mode and self.dec.mode() == "blended"

  def update_targets(self, sm: messaging.SubMaster, v_ego: float, a_ego: float, v_cruise: float) -> tuple[float, float]:
    CS = sm['carState']
    v_cruise_cluster_kph = min(CS.vCruiseCluster, V_CRUISE_MAX)
    v_cruise_cluster = v_cruise_cluster_kph * CV.KPH_TO_MS

    long_enabled = sm['carControl'].enabled
    long_override = sm['carControl'].cruiseControl.override

    # Smart Cruise Control
    self.scc.update(sm, long_enabled, long_override, v_ego, a_ego, v_cruise)

    # Speed Limit Resolver
    self.resolver.update(v_ego, sm)

    # Speed Limit Assist
    has_speed_limit = self.resolver.speed_limit_valid or self.resolver.speed_limit_last_valid
    self.sla.update(long_enabled, long_override, v_ego, a_ego, v_cruise_cluster, self.resolver.speed_limit,
                    self.resolver.speed_limit_final_last, has_speed_limit, self.resolver.distance, self.events_sp)

    # Stop Sign Assist (GPS fix times are wall-clock)
    now_s = time.time()  # noqa: TID251
    if self.stop_sign is not None:
      self.stop_sign.update(sm, long_enabled, long_override, v_ego, a_ego, now_s)
    # shadow: as if engaged the whole drive, ignoring the gas; its target goes nowhere. Log-only, so it must never
    # take the planner down with it.
    if self.stop_sign_shadow is not None:
      try:
        self.budget_stop_shadow.run(self.stop_sign_shadow.update, sm, True, False, sm['carState'].vEgo, a_ego, now_s)
      except Exception:
        cloudlog.exception("stop sign shadow update failed")
    if self.speed_bump is not None:
      try:
        self.speed_bump.update(sm, long_enabled, long_override, v_ego, a_ego, now_s)
      except Exception:
        self.speed_bump._finish()   # idle: no target from a failed update
        cloudlog.exception("speed bump update failed")
    if self.speed_bump_shadow is not None:
      try:
        self.budget_bump_shadow.run(self.speed_bump_shadow.update, sm, True, False, sm['carState'].vEgo, a_ego, now_s)
      except Exception:
        cloudlog.exception("speed bump shadow update failed")

    if self._narrow_param_frame % 20 == 0:
      try:
        self.narrow_enabled = Params().get_bool(narrow_street.PARAM)
      except Exception:
        self.narrow_enabled = False
    if self._narrow_param_frame % 100 == 0:
      self.profile = driver_profile.active()
      self.profile_on = driver_profile.is_on()
      self.profile_speed_on = self.profile_on and driver_profile.is_on(param=driver_profile.SPEED_PARAM)
      self.narrow_speed = self.profile.narrow_speed_mph * CV.MPH_TO_MS
      for ss in (self.stop_sign, self.stop_sign_shadow):
        if ss is not None:
          ss.set_profile(self.profile.stop_decel, self.profile.coast_s)
      self.scc.vision.set_lat_accel_max(self.profile.curve_lat_accel)
    self._narrow_param_frame += 1
    try:
      self.narrow_street = self.narrow.update(sm['modelV2'], v_ego)
    except Exception:
      self.narrow_street = False
      cloudlog.exception("narrow street detector failed")

    if self.map_caution is not None:
      try:
        if not self.budget_map_caution.run(self.map_caution.update, sm, long_enabled, long_override, v_ego, a_ego,
                                           self.narrow_street, now_s):
          self._map_caution_neutral()
      except Exception:
        self._map_caution_neutral()
        cloudlog.exception("map caution update failed")

    targets = {
      LongitudinalPlanSource.cruise: (v_cruise, a_ego),
      LongitudinalPlanSource.sccVision: (self.scc.vision.output_v_target, self.scc.vision.output_a_target),
      LongitudinalPlanSource.sccMap: (self.scc.map.output_v_target, self.scc.map.output_a_target),
      LongitudinalPlanSource.speedLimitAssist: (self.sla.output_v_target, self.sla.output_a_target),
    }
    if self.stop_sign is not None:
      targets[LongitudinalPlanSource.stopSign] = (self.stop_sign.output_v_target, self.stop_sign.output_a_target)
    if self.map_caution is not None and self.map_caution.crosswalk_caution:
      targets[LongitudinalPlanSource.crosswalk] = (self.map_caution.v_target, self.map_caution.a_target)
    if self.speed_bump is not None and self.speed_bump.is_active:
      targets[LongitudinalPlanSource.speedBump] = (self.speed_bump.output_v_target, self.speed_bump.output_a_target)
    if self.narrow_street and self.narrow_enabled and self.openpilot_long:
      a_narrow = narrow_street.NARROW_ACCEL if v_ego > self.narrow_speed else a_ego
      targets[LongitudinalPlanSource.narrowStreet] = (self.narrow_speed, a_narrow)

    # driver profile: the junction-turn speed during a signaled sharp slow turn, and the cruising speed cap
    if self.profile_on and self.openpilot_long:
      in_turn = profile_long.junction_turn(CS.leftBlinker, CS.rightBlinker, v_ego, sm['controlsState'].curvature)
      self.turn_exit_left = profile_long.TURN_EXIT_S if in_turn else max(0.0, self.turn_exit_left - DT_MDL)
      if in_turn:
        v_turn = self.profile.junction_turn_mph * CV.MPH_TO_MS
        targets[LongitudinalPlanSource.junctionTurn] = (v_turn, profile_long.JUNCTION_ACCEL if v_ego > v_turn else a_ego)
      limit = self.resolver.speed_limit
      if self.profile_speed_on and self.resolver.speed_limit_valid:
        v_cap = profile_long.profile_speed(limit, self.profile.speed_offset_mph(limit * CV.MS_TO_MPH))
        if v_cap is not None:
          targets[LongitudinalPlanSource.driverSpeed] = (v_cap, profile_long.PROFILE_SPEED_ACCEL if v_ego > v_cap + 0.5 else a_ego)

    self.source = min(targets, key=lambda k: targets[k][0])
    self.output_v_target, self.output_a_target = targets[self.source]
    return self.output_v_target, self.output_a_target

  def _bump_shadow_quiet(self) -> None:
    if self.speed_bump_shadow is not None:
      self.speed_bump_shadow.warn = False

  def _map_caution_neutral(self) -> None:
    if self.map_caution is not None:
      self.map_caution.signal_coast = self.map_caution.crosswalk_caution = False
      self.map_caution.v_target = float('inf')

  def t_follow(self, v_ego: float) -> float | None:
    """The MPC's following gap from the driver profile, or None for the personality's."""
    if not (self.profile_on and self.openpilot_long):
      return None
    return profile_long.follow_t(self.profile.follow_gap_s, v_ego)

  def map_accel_cap(self) -> float:
    """No accelerating toward a mapped traffic light (map caution); inf when it doesn't limit."""
    return self.map_caution.accel_cap() if self.map_caution is not None else float('inf')

  def lead_coast_cap(self, sm: messaging.SubMaster, v_ego: float) -> float:
    """The most acceleration the driver profile allows: none toward a slower lead inside the coasting gap, and the
    driver's own pull-away just after a junction turn (inf when it doesn't limit)."""
    self.lead_coasting = False
    if not (self.profile_on and self.openpilot_long):
      return float('inf')
    lead = sm['radarState'].leadOne
    self.lead_coasting = profile_long.lead_coast(self.profile.lead_coast_s, v_ego, lead.present, lead.dRel, lead.vRel)
    cap = profile_long.LEAD_COAST_ACCEL if self.lead_coasting else float('inf')
    return min(cap, profile_long.turn_exit_cap(self.turn_exit_left, self.profile.turn_exit_accel))

  def update(self, sm: messaging.SubMaster) -> None:
    self.events_sp.clear()
    self.dec.update(sm)
    self.e2e_alerts_helper.update(sm, self.events_sp)

  def publish_longitudinal_plan_sp(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    plan_sp_send = messaging.new_message('longitudinalPlanSP')

    plan_sp_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlanSP = plan_sp_send.longitudinalPlanSP
    longitudinalPlanSP.longitudinalPlanSource = self.source
    longitudinalPlanSP.vTarget = float(self.output_v_target)
    longitudinalPlanSP.aTarget = float(self.output_a_target)
    longitudinalPlanSP.events = self.events_sp.to_msg()
    longitudinalPlanSP.narrowStreet = bool(self.narrow_street)
    longitudinalPlanSP.leadCoast = bool(self.lead_coasting)
    if self.map_caution is not None:
      longitudinalPlanSP.signalCoast = bool(self.map_caution.signal_coast)
      longitudinalPlanSP.crosswalkCaution = bool(self.map_caution.crosswalk_caution)

    # Dynamic Experimental Control
    dec = longitudinalPlanSP.dec
    dec.state = DecState.blended if self.dec.mode() == 'blended' else DecState.acc
    dec.enabled = self.dec.enabled()
    dec.active = self.dec.active()

    # Smart Cruise Control
    smartCruiseControl = longitudinalPlanSP.smartCruiseControl
    # Vision Control
    sccVision = smartCruiseControl.vision
    sccVision.state = self.scc.vision.state
    sccVision.vTarget = float(self.scc.vision.output_v_target)
    sccVision.aTarget = float(self.scc.vision.output_a_target)
    sccVision.currentLateralAccel = float(self.scc.vision.current_lat_acc)
    sccVision.maxPredictedLateralAccel = float(self.scc.vision.max_pred_lat_acc)
    sccVision.enabled = self.scc.vision.is_enabled
    sccVision.active = self.scc.vision.is_active
    # Map Control
    sccMap = smartCruiseControl.map
    sccMap.state = self.scc.map.state
    sccMap.vTarget = float(self.scc.map.output_v_target)
    sccMap.aTarget = float(self.scc.map.output_a_target)
    sccMap.enabled = self.scc.map.is_enabled
    sccMap.active = self.scc.map.is_active

    # Speed Limit
    speedLimit = longitudinalPlanSP.speedLimit
    resolver = speedLimit.resolver
    resolver.speedLimit = float(self.resolver.speed_limit)
    resolver.speedLimitLast = float(self.resolver.speed_limit_last)
    resolver.speedLimitFinal = float(self.resolver.speed_limit_final)
    resolver.speedLimitFinalLast = float(self.resolver.speed_limit_final_last)
    resolver.speedLimitValid = self.resolver.speed_limit_valid
    resolver.speedLimitLastValid = self.resolver.speed_limit_last_valid
    resolver.speedLimitOffset = float(self.resolver.speed_limit_offset)
    resolver.distToSpeedLimit = float(self.resolver.distance)
    resolver.source = self.resolver.source
    assist = speedLimit.assist
    assist.state = self.sla.state
    assist.enabled = self.sla.is_enabled
    assist.active = self.sla.is_active
    assist.vTarget = float(self.sla.output_v_target)
    assist.aTarget = float(self.sla.output_a_target)

    # Stop Sign Assist: the live one when it runs, else the shadow
    ss = self.stop_sign if self.stop_sign is not None and self.stop_sign.enabled else self.stop_sign_shadow
    if ss is not None:
      stopSign = longitudinalPlanSP.stopSign
      stopSign.state = int(ss.state)
      stopSign.active = ss.is_active
      stopSign.shadow = ss.shadow
      stopSign.distance = float(ss.distance)
      stopSign.vTarget = float(ss.output_v_target)
      stopSign.confirmed = ss.confirmed
      stopSign.signId = int(ss.node_id or 0)

    # Speed Bump Assist: the live one when it runs, else the shadow; the warning always from the shadow copy
    sb = self.speed_bump if self.speed_bump is not None and self.speed_bump.enabled else self.speed_bump_shadow
    if sb is not None:
      speedBump = longitudinalPlanSP.speedBump
      speedBump.state = int(sb.state)
      speedBump.active = sb.is_active
      speedBump.shadow = sb.shadow
      speedBump.distance = float(sb.distance)
      speedBump.vCross = float(min(sb.v_cross, 255.0))
      speedBump.vTarget = float(min(sb.output_v_target, 255.0))
      speedBump.bumpId = int(sb.node_id or 0)
      speedBump.warn = bool(self.speed_bump_shadow is not None and self.speed_bump_shadow.warn)

    # E2E Alerts
    e2eAlerts = longitudinalPlanSP.e2eAlerts
    e2eAlerts.greenLightAlert = self.e2e_alerts_helper.green_light_alert
    e2eAlerts.leadDepartAlert = self.e2e_alerts_helper.lead_depart_alert

    pm.send('longitudinalPlanSP', plan_sp_send)
