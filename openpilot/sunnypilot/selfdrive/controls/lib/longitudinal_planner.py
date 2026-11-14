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
from openpilot.sunnypilot.selfdrive.controls.lib import narrow_street
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
    self.generation = int(model_bundle.generation) if (model_bundle := get_active_bundle()) else None
    # Narrow streets (narrow_street.py): always detected and published; the speed cap only with NarrowStreetAssist
    # and openpilot longitudinal. Re-read the param about once a second.
    self.openpilot_long = CP.openpilotLongitudinalControl
    self.narrow = narrow_street.NarrowStreetDetector(DT_MDL)
    self.narrow_street = False
    self.narrow_enabled = False
    self._narrow_param_frame = 0
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
        self.stop_sign_shadow.update(sm, True, False, sm['carState'].vEgo, a_ego, now_s)
      except Exception:
        cloudlog.exception("stop sign shadow update failed")

    if self._narrow_param_frame % 20 == 0:
      try:
        self.narrow_enabled = Params().get_bool(narrow_street.PARAM)
      except Exception:
        self.narrow_enabled = False
    self._narrow_param_frame += 1
    try:
      self.narrow_street = self.narrow.update(sm['modelV2'], v_ego)
    except Exception:
      self.narrow_street = False
      cloudlog.exception("narrow street detector failed")

    targets = {
      LongitudinalPlanSource.cruise: (v_cruise, a_ego),
      LongitudinalPlanSource.sccVision: (self.scc.vision.output_v_target, self.scc.vision.output_a_target),
      LongitudinalPlanSource.sccMap: (self.scc.map.output_v_target, self.scc.map.output_a_target),
      LongitudinalPlanSource.speedLimitAssist: (self.sla.output_v_target, self.sla.output_a_target),
    }
    if self.stop_sign is not None:
      targets[LongitudinalPlanSource.stopSign] = (self.stop_sign.output_v_target, self.stop_sign.output_a_target)
    if self.narrow_street and self.narrow_enabled and self.openpilot_long:
      a_narrow = narrow_street.NARROW_ACCEL if v_ego > narrow_street.NARROW_SPEED else a_ego
      targets[LongitudinalPlanSource.narrowStreet] = (narrow_street.NARROW_SPEED, a_narrow)

    self.source = min(targets, key=lambda k: targets[k][0])
    self.output_v_target, self.output_a_target = targets[self.source]
    return self.output_v_target, self.output_a_target

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

    # E2E Alerts
    e2eAlerts = longitudinalPlanSP.e2eAlerts
    e2eAlerts.greenLightAlert = self.e2e_alerts_helper.green_light_alert
    e2eAlerts.leadDepartAlert = self.e2e_alerts_helper.lead_depart_alert

    pm.send('longitudinalPlanSP', plan_sp_send)
