"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import time

from openpilot.cereal import messaging, custom, log
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
from openpilot.sunnypilot.selfdrive.controls.lib.red_light import RedLightAssist
from openpilot.sunnypilot.selfdrive.controls.lib.red_light import STOP_COLORS as RED_LIGHT_STOP_COLORS
from openpilot.sunnypilot.selfdrive.controls.lib.map_caution import MapCaution
from openpilot.sunnypilot.selfdrive.controls.lib.wet_road import WetRoad
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import STOP_DISTANCE, get_T_FOLLOW
from openpilot.sunnypilot.selfdrive.controls.lib import driver_profile, narrow_street, profile_long
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.models.helpers import get_active_bundle

DecState = custom.LongitudinalPlanSP.DynamicExperimentalControl.DynamicExperimentalControlState
LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource


class LongitudinalPlannerSP:
  def _log_profile_version(self) -> None:
    """One driver_profile event in the log whenever the live profile's name, content or on/off changes."""
    try:
      from openpilot.sunnypilot.selfdrive.controls.lib import profile_versions
      name = driver_profile.active_name(Params())
      ver = (name, profile_versions.live_sha("profile", name), self.profile_on)
      if ver != self.profile_version:
        self.profile_version = ver
        cloudlog.event("driver_profile", name=name, sha=(ver[1] or "")[:12], on=self.profile_on)
    except Exception:
      cloudlog.exception("driver profile version log failed")

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
    # Wet road (wet_road.py): rain from the wipers and traction control; changes only settings the planner already has
    self.wet = WetRoad()
    self.personality = log.LongitudinalPersonality.standard
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
    self.profile_version = None   # (name, sha) last logged: ties the rlog to the profile version that was live
    self.profile_speed_on = False
    self.speed_cap = profile_long.ProfileSpeedCap()   # the driverSpeed target: coasts down, keeps the driver's choice
    self.turn_exit_left = 0.0     # s left of the pull-away window after a junction turn
    self.turn_approach = profile_long.TurnApproach()   # the slow-down before a signaled turn: one rate per approach
    self.narrow_speed = narrow_street.NARROW_SPEED
    self.lead_coasting = False
    self.follow_cap = profile_long.LeadFollowCap()     # following a car: up to the limit + 10 mph
    self.take_off_lead = profile_long.LeadTakeOff()    # moving off as the car in front does
    self.red_light = None
    try:   # Red Light Assist: runs and logs on every drive, sets the speed only with its param (openpilot speed control)
      self.red_light = RedLightAssist()
    except Exception:
      cloudlog.exception("red light assist unavailable")
    self.lead_brake = profile_long.LeadBrake(DT_MDL)   # driver profile lead_brake_a: slowing toward a slower car closing fast
    self.lead_braking = False
    self.lead_brake_a_cap: float | None = None
    self.model_stop = profile_long.ModelStop(DT_MDL)   # stopping where the driving model plans to, evenly
    self.sign_hold = profile_long.SignDropHold()       # a sign reading far under the map's limit does not lower the set speed
    self.model_stop_a_cap: float | None = None
    self.model_stopping = False
    self.land_room: float | None = None                # m to the stop point the soft landing may count on, or None
    self.land_v = 0.0
    self.landing = False
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

    if self.openpilot_long:
      try:   # the car's sign reader dropped the set speed far under the map's limit: drive by the one before
        md = sm['liveMapDataSP']
        was = self.sign_hold.held
        v_set = v_cruise
        v_cruise = self.sign_hold.update(v_set, float(sm['carStateSP'].speedLimit), float(md.speedLimit) if md.speedLimitValid else 0.0)
        if (self.sign_hold.held is None) != (was is None):
          cloudlog.event("sign_drop_hold", on=self.sign_hold.held is not None, set=round(v_set, 2), drive_by=round(v_cruise, 2),
                         sign=round(float(sm['carStateSP'].speedLimit), 2), map=round(float(md.speedLimit), 2))
      except Exception:
        self.sign_hold = profile_long.SignDropHold()

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
    if self.red_light is not None:
      try:   # with openpilot speed control: as engaged as it is; without: as if engaged (log-only there)
        cs_sp = sm['carStateSP']
        on = long_enabled if self.openpilot_long else True
        self.red_light.update(sm, on, long_override if self.openpilot_long else False, v_ego, a_ego,
                              int(cs_sp.trafficLightColor), float(cs_sp.trafficLightDistance))
      except Exception:
        cloudlog.exception("red light assist update failed")
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

    try:
      cs_sp = sm['carStateSP']
      self.wet.update(cs_sp.wiperSpeed, cs_sp.tractionControlActive, DT_MDL)
    except Exception:
      cloudlog.exception("wet road update failed")
    self.personality = sm['selfdriveState'].personality

    if self._narrow_param_frame % 20 == 0:
      try:
        self.narrow_enabled = Params().get_bool(narrow_street.PARAM)
      except Exception:
        self.narrow_enabled = False
    if self._narrow_param_frame % 100 == 0 or self.wet.changed:
      self.profile = driver_profile.active()
      self.profile_on = driver_profile.is_on()
      self._log_profile_version()
      self.profile_speed_on = self.profile_on and driver_profile.is_on(param=driver_profile.SPEED_PARAM)
      self.narrow_speed = self.profile.narrow_speed_mph * CV.MPH_TO_MS
      for ss in (self.stop_sign, self.stop_sign_shadow, self.red_light):
        if ss is not None:
          ss.set_profile(min(self.profile.stop_decel, self.wet.stop_decel()), self.profile.coast_s)
      self.scc.vision.set_lat_accel_max(self.profile.curve_lat_accel * self.wet.curve_factor())
      for sb in (self.speed_bump, self.speed_bump_shadow):
        if sb is not None and hasattr(sb, "set_profile"):
          sb.set_profile(self.profile.bump_cross_mph if self.profile_on else 0.0)
      dflt = driver_profile.DriverProfile()
      on = self.profile_on
      self.scc.vision.set_entry_decel(self.profile.curve_entry_decel if on else dflt.curve_entry_decel)
      if self.map_caution is not None:
        self.map_caution.set_profile(self.profile.signal_coast_s if on else dflt.signal_coast_s)
      self.wet.set_profile(self.profile.wet_offset_mph if on and self.profile.wet_offset_mph != dflt.wet_offset_mph else None,
                           self.profile.wet_gap_factor if on and self.profile.wet_gap_factor != dflt.wet_gap_factor else None)
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
    if self.red_light is not None and self.red_light.controls and self.openpilot_long:
      targets[LongitudinalPlanSource.redLight] = (self.red_light.output_v_target, self.red_light.output_a_target)
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
        v_turn = profile_long.junction_speed(self.profile.junction_turn_mph * CV.MPH_TO_MS, self.profile.junction_lat_accel,
                                             sm['controlsState'].curvature)
        targets[LongitudinalPlanSource.junctionTurn] = (v_turn, profile_long.JUNCTION_ACCEL if v_ego > v_turn else a_ego)
      limit = self.resolver.speed_limit
      lead = sm['radarState'].leadOne
      # following a car: the cruising speed may rise with it, to the limit + 10 mph at most (the driver's rule). It
      # lifts the profile's cap and the set speed alike; the MPC keeps the gap
      v_follow = self.follow_cap.update(limit if self.resolver.speed_limit_valid else 0.0, v_ego, lead.present, lead.dRel,
                                        lead.vRel, DT_MDL)
      if v_follow is not None and v_follow > v_cruise:
        targets[LongitudinalPlanSource.cruise] = (v_follow, a_ego)
      if self.profile_speed_on and self.resolver.speed_limit_valid:
        v_cap = profile_long.profile_speed(limit, self.profile.speed_offset_mph(limit * CV.MS_TO_MPH))
        v_cap = self.speed_cap.update(v_cap, limit, v_ego, bool(sm['carState'].gasPressed), DT_MDL)
        if v_cap is not None:
          if v_follow is not None:
            v_cap = max(v_cap, v_follow)
          targets[LongitudinalPlanSource.driverSpeed] = (v_cap, -profile_long.PROFILE_COAST_DECEL if v_ego > v_cap + 0.5 else a_ego)
      else:
        self.speed_cap.update(None, 0.0, v_ego, False, DT_MDL)
      # slowing early for a car ahead: an acceleration ceiling (lead_brake_cap), worked out here once a frame
      was = self.lead_brake.active
      gate = 1.0 if lead.radar else lead.modelProb   # only a vision lead carries a certainty
      self.lead_brake_a_cap = self.lead_brake.update(self.profile.lead_brake_a, self.profile.follow_gap_s, v_ego, lead.present,
                                                     lead.dRel, lead.vRel, lead.aLeadK, gate, a_ego,
                                                     profile_long.stop_gap(self.profile.stop_gap_m) or profile_long.STOP_DISTANCE)
      if self.lead_brake.active != was:
        cloudlog.event("lead_brake", on=self.lead_brake.active, v=round(v_ego, 2), lead_d=round(lead.dRel, 1),
                       lead_v=round(lead.vLead, 2), lead_a=round(lead.aLeadK, 2), need=round(min(self.lead_brake.need, 9.9), 2))
      self.update_model_stop(sm, v_ego, a_ego, lead, long_enabled and not long_override)
    else:
      self.lead_brake.reset()
      self.lead_brake_a_cap = None
      self.lead_braking = False
      self.follow_cap.v = None
      self.model_stop.reset()
      self.model_stop_a_cap = None
      self.land_room = None

    # map turn assist armed for a signaled turn ahead (or turning): slow to the junction speed before it
    if self.openpilot_long:
      try:
        mt = sm['carControlSP'].mapTurn
        state, dist = int(mt.state), float(mt.junctionDistance)
      except Exception:
        state, dist = 0, 0.0
      v_turn = self.profile.junction_turn_mph * CV.MPH_TO_MS if self.profile_on else profile_long.V_TURN_DEFAULT
      v_turn_target = None
      a_turn = profile_long.JUNCTION_ACCEL
      if state in (1, 3):   # armed, or the junction seen from road speed (the far approach): the same slow-down
        # the driver's own speed where the wheel goes in and how they slow down to it (the profile; else as before)
        v_in = max(self.profile.turn_in_mph * CV.MPH_TO_MS, v_turn) if self.profile_on and self.profile.turn_in_mph > 0.0 else v_turn
        decel = self.profile.turn_approach_decel if self.profile_on and self.profile.turn_approach_decel > 0.0 else -profile_long.JUNCTION_ACCEL
        v_turn_target = self.turn_approach.update(dist, v_in, v_ego, decel)
        a_turn = -self.turn_approach.decel
        if self.turn_approach.started:
          cloudlog.event("junction_approach", state=state, dist=round(dist, 1), v=round(v_ego, 2), v_in=round(v_in, 2),
                         decel=round(self.turn_approach.decel, 2))
      else:
        self.turn_approach.reset()
      if state == 2:
        v_turn_target = profile_long.junction_speed(v_turn, self.profile.junction_lat_accel if self.profile_on else 0.0,
                                                    sm['controlsState'].curvature)
        self.turn_exit_left = profile_long.TURN_EXIT_S   # the driver profile's pull-away after the turn
      if v_turn_target is not None:
        prev = targets.get(LongitudinalPlanSource.junctionTurn, (float('inf'), a_ego))
        if v_turn_target < prev[0]:
          targets[LongitudinalPlanSource.junctionTurn] = (v_turn_target, a_turn if v_ego > v_turn_target else a_ego)
    else:
      self.turn_approach.reset()

    # wet road: cruise no faster than the posted limit + the level's offset
    wet_offset = self.wet.speed_offset_mph()
    if wet_offset is not None and self.openpilot_long and self.resolver.speed_limit_valid:
      v_wet = max(self.resolver.speed_limit + wet_offset * CV.MPH_TO_MS, 0.0)
      targets[LongitudinalPlanSource.wetRoad] = (v_wet, profile_long.PROFILE_SPEED_ACCEL if v_ego > v_wet + 0.5 else a_ego)

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

  def stop_distance(self) -> float | None:
    """The MPC's stop distance behind a car from the driver profile (stop_gap_m), or None for the MPC's own 6 m."""
    if not (self.profile_on and self.openpilot_long):
      return None
    return profile_long.stop_gap(self.profile.stop_gap_m)

  def t_follow(self, v_ego: float) -> float | None:
    """The MPC's following gap from the driver profile, or None for the personality's; longer on a wet road."""
    if not self.openpilot_long:
      return None
    factor = self.wet.gap_factor()
    if self.profile_on:
      return profile_long.follow_t(self.profile.follow_gap_s * factor, v_ego)
    if factor == 1.0:
      return None
    return get_T_FOLLOW(self.personality) * factor

  def map_accel_cap(self, v_ego: float) -> float:
    """No accelerating toward a mapped traffic light (map caution); with openpilot speed control also a softer pull-away
    on a wet road and the comfort ceiling (profile_long.comfort_accel_cap); inf when nothing limits."""
    cap = self.map_caution.accel_cap() if self.map_caution is not None else float('inf')
    if not self.openpilot_long:
      return cap
    merge = self.resolver.speed_limit_valid and profile_long.merging(self.resolver.speed_limit, v_ego)
    return min(cap, self.wet.accel_max(), profile_long.comfort_accel_cap(v_ego, self.profile if self.profile_on else None,
                                                                          merge))

  def take_off(self, sm: messaging.SubMaster, v_ego: float) -> float | None:
    """Moving off behind a car as one unit with it (profile_long.LeadTakeOff): the least acceleration to ask for, or
    None. It goes over the MPC's and the driving model's own request and the comfort ceiling (experimental mode asked
    +0.03..0.23 for 4 s on 00000129 with the lead driving off), by the driver's rule: follow the car in front, except
    at a light that is not green. So not on a red or amber light (the car's own recognition), not while Stop Sign
    Assist or Red Light Assist holds the car, not with the driver on the brake, and only with openpilot speed control.
    Never more than a wet road allows; the caller keeps it under the speed targets (set speed, limit, curve, turn)."""
    tk = self.take_off_lead
    try:
      lead = sm['radarState'].leadOne
      held = (self.stop_sign is not None and self.stop_sign.is_active) or (self.red_light is not None and self.red_light.controls)
      red = int(sm['carStateSP'].trafficLightColor) in RED_LIGHT_STOP_COLORS
      if not self.openpilot_long or held or red or sm['carState'].brakePressed:
        if tk.active:
          cloudlog.event("take_off", on=False, why="held" if held else "red" if red else "brake", v=round(v_ego, 2))
        tk.reset()
        return None
      was = tk.active
      a = tk.update(v_ego, lead.present, lead.dRel, lead.vLead, lead.aLeadK, self.wet.accel_max(), DT_MDL,
                    self.t_follow(v_ego) or get_T_FOLLOW(self.personality), self.stop_distance() or STOP_DISTANCE)
      if tk.active != was:
        cloudlog.event("take_off", on=tk.active, v=round(v_ego, 2), lead_d=round(lead.dRel, 1), lead_v=round(lead.vLead, 2),
                       a=None if a is None else round(a, 2))
      return a
    except Exception:
      tk.reset()
      return None

  def update_model_stop(self, sm: messaging.SubMaster, v_ego: float, a_ego: float, lead, engaged: bool) -> None:
    """The model-stop ceiling (profile_long.ModelStop) and the room the soft landing may count on, once a frame.
    Any problem reading the model means neither."""
    self.land_v = v_ego
    self.land_room = None
    try:
      md = sm['modelV2']
      vx, px = list(md.velocity.x), list(md.position.x)
      if not engaged or len(vx) < 2 or len(vx) != len(px):
        raise ValueError
      d_rest = next((float(px[i]) for i in range(len(vx)) if vx[i] < 0.5), None)
      ms = self.model_stop
      was = (ms.active, ms.braking)
      self.model_stop_a_cap = ms.update(self.profile.lead_brake_a, v_ego, float(vx[-1]), float(px[-1]), d_rest, lead.present,
                                        lead.dRel, lead.vLead, a_ego)
      if (ms.active, ms.braking) != was:
        cloudlog.event("model_stop", on=ms.active, braking=ms.braking, v=round(v_ego, 2), room=round(ms.room, 1),
                       need=round(min(ms.need, 9.9), 2), v_end=round(float(vx[-1]), 1), lead=bool(lead.present))
      # the soft landing's stop point: the nearest of the standing car ahead and where the model plans to rest. A stop
      # that Stop Sign Assist or Red Light Assist is making has its own stop point: no landing there.
      held = (self.stop_sign is not None and self.stop_sign.is_active) or (self.red_light is not None and self.red_light.controls)
      rooms = []
      if lead.present and lead.vLead < profile_long.LAND_LEAD_STANDING_V:
        rooms.append(lead.dRel - (profile_long.stop_gap(self.profile.stop_gap_m) or profile_long.STOP_DISTANCE))
      if d_rest is not None:
        rooms.append(d_rest)
      if rooms and not held:
        self.land_room = min(rooms)
    except Exception:
      self.model_stop.reset()
      self.model_stop_a_cap = None
      self.land_room = None

  def lead_brake_cap(self, a_target: float) -> float:
    """a_target with the early slowdowns applied (for a car ahead, profile_long.LeadBrake; for a stop the driving model
    plans, profile_long.ModelStop): only ever lower. Then the soft landing (profile_long.landing_floor): on the last
    metres to a known stop point a request firmer than the gentle one the car can deliver, but not an urgent one, is
    replaced by the gentle one while that still stops short of the point."""
    cap, source = self.lead_brake_a_cap, LongitudinalPlanSource.leadBrake
    if self.model_stop_a_cap is not None and (cap is None or self.model_stop_a_cap < cap):
      cap, source = self.model_stop_a_cap, LongitudinalPlanSource.modelStop
    acting = cap is not None and cap < a_target
    self.lead_braking = acting and source == LongitudinalPlanSource.leadBrake
    self.model_stopping = acting and source == LongitudinalPlanSource.modelStop
    if acting:
      self.source = source
      a_target = float(cap)
    landing = False
    if self.land_room is not None and profile_long.LAND_URGENT < a_target < 0.0 and not getattr(self, "fcw", False):
      floor = profile_long.landing_floor(self.land_v, self.land_room)
      if floor is not None and floor > a_target:
        a_target, landing = floor, True
    if landing != self.landing:
      self.landing = landing
      cloudlog.event("soft_landing", on=landing, v=round(self.land_v, 2), room=round(self.land_room or 0.0, 1), a=round(a_target, 2))
    return a_target

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
    wet = longitudinalPlanSP.wetRoad
    wet.level = self.wet.level
    wet.applied = self.wet.applied
    wet.wiperSpeed = min(self.wet.wiper, 255)
    wet.tractionControlActive = self.wet.tcs
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
