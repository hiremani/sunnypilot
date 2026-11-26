#!/usr/bin/env python3
import math
from numbers import Number

from openpilot.cereal import log
from opendbc.car.structs import car
import openpilot.cereal.messaging as messaging
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, DT_CTRL, Priority, Ratekeeper
from openpilot.common.swaglog import cloudlog

from opendbc.car.car_helpers import interfaces
from opendbc.car.vehicle_model import VehicleModel
from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle, STEER_ANGLE_SATURATION_THRESHOLD
from openpilot.selfdrive.controls.lib.latcontrol_curvature import LatControlCurvature
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.modeld.modeld import LAT_SMOOTH_SECONDS
from openpilot.selfdrive.locationd.helpers import PoseCalibrator, Pose

from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt
from openpilot.sunnypilot.selfdrive.controls.lib import driver_profile
from openpilot.sunnypilot.selfdrive.controls.lib import junction_memory
from openpilot.sunnypilot.selfdrive.controls.lib import stop_test
from openpilot.sunnypilot.selfdrive.controls.lib.turn_planner import TurnPlanner, apply_profile as apply_turn_profile, kerb_room_from_model
from openpilot.sunnypilot.selfdrive.controls.lib import wet_road
from openpilot.sunnypilot.selfdrive.controls.lib.narrow_street import PARAM as NARROW_PARAM, CenterLineKeeper
from openpilot.common.gps import get_gps_location_service

State = log.SelfdriveState.OpenpilotState
LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

ACTUATOR_FIELDS = tuple(car.CarControl.Actuators.schema.fields.keys())


class Controls(ControlsExt):
  def __init__(self) -> None:
    self.params = Params()
    cloudlog.info("controlsd is waiting for CarParams")
    self.CP = messaging.log_from_bytes(self.params.get("CarParams", block=True), car.CarParams)
    cloudlog.info("controlsd got CarParams")

    # Initialize sunnypilot controlsd extension and base model state
    ControlsExt.__init__(self, self.CP, self.params)

    self.CI = interfaces[self.CP.carFingerprint](self.CP, self.CP_SP)

    # Map turn assist (sunnypilot): GPS for the turn planner. Never required: a missing or bad fix only means no turn.
    self.gps_service = get_gps_location_service(self.params)
    self.sm = messaging.SubMaster(['lateralDelay', 'vehicleParameters', 'lateralTorqueParameters', 'modelV2', 'selfdriveState',
                                   'extrinsicsCalibration', 'deviceMotion', 'longitudinalPlan', 'lateralManeuverPlan', 'carState', 'carOutput',
                                   'driverMonitoringState', 'onroadEvents', 'driverAssistance', self.gps_service] + self.sm_services_ext,
                                  poll='selfdriveState', ignore_alive=[self.gps_service], ignore_avg_freq=[self.gps_service],
                                  ignore_valid=[self.gps_service])
    self.pm = messaging.PubMaster(['carControl', 'controlsState'] + self.pm_services_ext)

    self.steer_limited_by_safety = False
    self.curvature = 0.0
    self.desired_curvature = 0.0

    # Map turn assist: steers a signaled low-speed junction turn from the road tiles (turn_planner.py). Off unless both
    # MapTurnAssist and LaneTurnDesire are set; re-read about once a second.
    self.turn_planner = TurnPlanner()
    self.map_turn_assist = False
    self.map_turn_wet_ok = True
    self.map_turn_curvature: float | None = None
    self._turn_param_frame = 0
    # Center-line keeper (narrow_street.py): with NarrowStreetAssist, keep the car's left side off a seen left line by
    # a small curvature correction, only ever to the right. Re-read with the map turn params.
    self.center_line_keeper = CenterLineKeeper(DT_CTRL)
    self.narrow_street_assist = False
    self.stop_test = 0                   # FiskerStopTest (stop_test.py), re-read with the map turn params

    self.pose_calibrator = PoseCalibrator()
    self.calibrated_pose: Pose | None = None

    self.LoC = LongControl(self.CP, self.CP_SP)
    self.VM = VehicleModel(self.CP)
    self.LaC: LatControl
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      self.LaC = LatControlAngle(self.CP, self.CP_SP, self.CI, DT_CTRL)
    elif self.CP.steerControlType == car.CarParams.SteerControlType.curvature:
      self.LaC = LatControlCurvature(self.CP, self.CP_SP, self.CI, DT_CTRL)
    elif self.CP.lateralTuning.which() == 'pid':
      self.LaC = LatControlPID(self.CP, self.CP_SP, self.CI, DT_CTRL)
    elif self.CP.lateralTuning.which() == 'torque':
      self.LaC = LatControlTorque(self.CP, self.CP_SP, self.CI, DT_CTRL)

    self.LaC = ControlsExt.initialize_lateral_control(self, self.LaC, self.CI, DT_CTRL)

  def update_map_turn(self, CS, CC, model_curvature: float | None = None, model_v2=None) -> float | None:
    """The map turn planner's curvature (1/m, positive turns right, as desiredCurvature) or None. Any failure means no
    turn: the model keeps steering."""
    if self._turn_param_frame % 100 == 0:
      try:   # never let a params problem (e.g. code newer than the built params) take controlsd down
        self.map_turn_assist = self.params.get_bool("MapTurnAssist") and self.params.get_bool("LaneTurnDesire")
        self.map_turn_wet_ok = wet_road.lane_changes_and_turns_ok(self.params)
      except Exception:
        self.map_turn_assist = False
      if self.map_turn_assist:
        try:   # the map worker reads the road tiles, never controlsd; a worker problem must not switch turns off
          self.turn_planner.roads_src.warm()
        except Exception:
          cloudlog.exception("map worker warm-up failed")
      try:   # separately: an unknown key here must not switch the map turn assist off
        self.narrow_street_assist = self.params.get_bool(NARROW_PARAM)
      except Exception:
        self.narrow_street_assist = False
      try:
        self.stop_test = int(self.params.get(stop_test.PARAM, return_default=True) or 0)
      except Exception:
        self.stop_test = 0
      if self._turn_param_frame % 1000 == 0:   # the driver profile's turn starts from a stop (about every 10 s)
        try:   # a problem here keeps the planner's current tuning (default: off)
          profile = driver_profile.active(self.params)
          apply_turn_profile(self.turn_planner.tuning, profile)
          if profile.turn_path_in > 0.0 and profile.turn_path_out > 0.0:
            self.turn_planner.tuning.junctions = junction_memory.load(driver_profile.active_name(self.params))
          else:
            self.turn_planner.tuning.junctions = {}
        except Exception:
          cloudlog.exception("driver profile turn starts failed")
    self._turn_param_frame += 1
    self.map_turn_curvature = None
    if not self.map_turn_assist:
      return None
    if not self.map_turn_wet_ok and self.turn_planner.state == "idle":
      return None   # heavy rain: no new turns (one already under way is finished, never dropped halfway)
    try:
      kerb_room = None   # read only while one blinker is on (the far approach's kerb-lane check), not every tick
      if model_v2 is not None and CS.leftBlinker != CS.rightBlinker:
        kerb_room = kerb_room_from_model(model_v2, CS.leftBlinker)
      self.map_turn_curvature = self.turn_planner.update(DT_CTRL, CS.vEgo, CS.yawRate, CS.leftBlinker, CS.rightBlinker,
                                                         CC.latActive, CS.steeringPressed, self.sm[self.gps_service],
                                                         CS.steeringTorque, model_curvature=model_curvature,
                                                         kerb_room=kerb_room)
    except Exception:
      cloudlog.exception("map turn planner failed")
      self.map_turn_curvature = None
    return self.map_turn_curvature

  def update(self):
    self.sm.update(15)
    if self.sm.updated["extrinsicsCalibration"]:
      self.pose_calibrator.feed_extrinsics_calibration(self.sm['extrinsicsCalibration'])
    if self.sm.updated["deviceMotion"]:
      device_motion = Pose.from_device_motion(self.sm['deviceMotion'])
      self.calibrated_pose = self.pose_calibrator.build_calibrated_pose(device_motion)

  def state_control(self):
    CS = self.sm['carState']

    # Update VehicleModel
    lp = self.sm['vehicleParameters']
    x = max(lp.stiffnessFactor, 0.1)
    sr = max(lp.steerRatio, 0.1)
    self.VM.update_params(x, sr)

    steer_angle_without_offset = math.radians(CS.steeringAngleDeg - lp.angleOffsetDeg)
    self.curvature = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo, lp.roll)

    # Update Torque Params
    if self.CP.lateralTuning.which() == 'torque':
      torque_params = self.sm['lateralTorqueParameters']
      if self.sm.all_checks(['lateralTorqueParameters']) and torque_params.useParams:
        self.LaC.update_torque_parameters(torque_params.latAccelFactorFiltered, torque_params.latAccelOffsetFiltered,
                                           torque_params.frictionCoefficientFiltered)

        self.LaC.extension.update_limits()

      self.LaC.extension.update_model_v2(self.sm['modelV2'])

      self.LaC.extension.update_lateral_lag(self.lat_delay)

    long_plan = self.sm['longitudinalPlan']
    model_v2 = self.sm['modelV2']

    CC = car.CarControl.new_message()
    CC.enabled = self.sm['selfdriveState'].enabled

    # Check which actuators can be enabled
    standstill = abs(CS.vEgo) <= max(self.CP.minSteerSpeed, 0.3) or CS.standstill

    # Get which state to use for active lateral control
    _lat_active = self.get_lat_active(self.sm)

    CC.latActive = _lat_active and not CS.steerFaultTemporary and not CS.steerFaultPermanent and \
                   (not standstill or self.CP.steerAtStandstill)
    CC.longActive = CC.enabled and not any(e.overrideLongitudinal for e in self.sm['onroadEvents']) and \
                    (self.CP.openpilotLongitudinalControl or not self.CP_SP.pcmCruiseSpeed)

    actuators = CC.actuators
    actuators.longControlState = self.LoC.long_control_state

    # Enable blinkers while lane changing
    if model_v2.meta.laneChangeState != LaneChangeState.off:
      CC.leftBlinker = model_v2.meta.laneChangeDirection == LaneChangeDirection.left
      CC.rightBlinker = model_v2.meta.laneChangeDirection == LaneChangeDirection.right

    if not CC.latActive:
      self.LaC.reset()
    if not CC.longActive:
      self.LoC.reset()

    # accel PID loop
    pid_accel_limits = self.CI.get_pid_accel_limits(self.CP, self.CP_SP, CS.vEgo, CS.vCruise * CV.KPH_TO_MS)
    actuators.accel = float(self.LoC.update(CC.longActive, CS, long_plan.aTarget, long_plan.shouldStop, pid_accel_limits))
    if self.stop_test and CC.longActive:   # low-speed stop test (off unless FiskerStopTest is set for a test drive)
      actuators.accel = stop_test.soften(actuators.accel, CS.vEgo, long_plan.aTarget, self.stop_test)

    # Steering PID loop and lateral MPC
    # Reset desired curvature to current to avoid violating the limits on engage
    if self.sm.valid['lateralManeuverPlan']:
      new_desired_curvature = self.sm['lateralManeuverPlan'].desiredCurvature if CC.latActive else self.curvature
    else:
      new_desired_curvature = model_v2.action.desiredCurvature if CC.latActive else self.curvature
      turn_curvature = self.update_map_turn(CS, CC, model_v2.action.desiredCurvature, model_v2)
      if turn_curvature is not None and CC.latActive:
        new_desired_curvature = turn_curvature
      elif self.narrow_street_assist and CC.latActive:
        try:
          new_desired_curvature += self.center_line_keeper.update(model_v2, CS.vEgo, CS.leftBlinker,
                                                                   CS.steeringPressed, CC.latActive,
                                                                   stamp=self.sm.recv_frame['modelV2'])
        except Exception:
          cloudlog.exception("center line keeper failed")
    self.desired_curvature, curvature_limited = clip_curvature(CS.vEgo, self.desired_curvature, new_desired_curvature, lp.roll)
    lat_delay = self.sm["lateralDelay"].lateralDelay + LAT_SMOOTH_SECONDS

    actuators.curvature = self.desired_curvature
    steer, lateral_output, lac_log = self.LaC.update(CC.latActive, CS, self.VM, lp,
                                                     self.steer_limited_by_safety, self.desired_curvature,
                                                     self.calibrated_pose, curvature_limited, lat_delay)
    actuators.torque = float(steer)
    if self.CP.steerControlType == car.CarParams.SteerControlType.curvature:
      actuators.curvature = float(lateral_output)
    else:
      actuators.steeringAngleDeg = float(lateral_output)
    # Ensure no NaNs/Infs
    for p in ACTUATOR_FIELDS:
      attr = getattr(actuators, p)
      if not isinstance(attr, Number):
        continue

      if not math.isfinite(attr):
        cloudlog.error(f"actuators.{p} not finite {actuators.to_dict()}")
        setattr(actuators, p, 0.0)

    return CC, lac_log

  def publish(self, CC, lac_log):
    CS = self.sm['carState']

    # Orientation and angle rates can be useful for carcontroller
    # Only calibrated (car) frame is relevant for the carcontroller
    CC.currentCurvature = self.curvature
    if self.calibrated_pose is not None:
      CC.orientationNED = self.calibrated_pose.orientation.xyz.tolist()
      CC.angularVelocity = self.calibrated_pose.angular_velocity.xyz.tolist()

    CC.cruiseControl.override = CC.enabled and not CC.longActive and (self.CP.openpilotLongitudinalControl or not self.CP_SP.pcmCruiseSpeed)
    CC.cruiseControl.cancel = CS.cruiseState.enabled and (not CC.enabled or not self.CP.pcmCruise)
    CC.cruiseControl.resume = CC.enabled and CS.cruiseState.standstill and not self.sm['longitudinalPlan'].shouldStop

    hudControl = CC.hudControl
    hudControl.setSpeed = float(CS.vCruiseCluster * CV.KPH_TO_MS)
    hudControl.speedVisible = CC.enabled
    hudControl.lanesVisible = CC.enabled
    hudControl.leadVisible = self.sm['longitudinalPlan'].hasLead
    hudControl.leadDistanceBars = self.sm['selfdriveState'].personality.raw + 1
    hudControl.visualAlert = self.sm['selfdriveState'].alertHudVisual

    hudControl.rightLaneVisible = True
    hudControl.leftLaneVisible = True
    if self.sm.valid['driverAssistance']:
      hudControl.leftLaneDepart = self.sm['driverAssistance'].leftLaneDeparture
      hudControl.rightLaneDepart = self.sm['driverAssistance'].rightLaneDeparture

    if self.get_lat_active(self.sm):
      CO = self.sm['carOutput']
      if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
        self.steer_limited_by_safety = abs(CC.actuators.steeringAngleDeg - CO.actuatorsOutput.steeringAngleDeg) > \
                                              STEER_ANGLE_SATURATION_THRESHOLD
      else:
        self.steer_limited_by_safety = abs(CC.actuators.torque - CO.actuatorsOutput.torque) > 1e-2

    # TODO: both controlsState and carControl valids should be set by
    #       sm.all_checks(), but this creates a circular dependency

    # controlsState
    dat = messaging.new_message('controlsState')
    dat.valid = CS.canValid
    cs = dat.controlsState

    cs.curvature = self.curvature
    cs.longitudinalPlanMonoTime = self.sm.logMonoTime['longitudinalPlan']
    cs.lateralPlanMonoTime = self.sm.logMonoTime['modelV2']
    cs.desiredCurvature = self.desired_curvature
    cs.longControlState = self.LoC.long_control_state
    cs.upAccelCmd = float(self.LoC.pid.p)
    cs.uiAccelCmd = float(self.LoC.pid.i)
    cs.ufAccelCmd = float(self.LoC.pid.f)
    cs.forceDecel = bool(self.sm['driverMonitoringState'].noResponseForceDecel or
                         (self.sm['selfdriveState'].state == State.softDisabling))

    # trigger the car's stock driver monitoring escalation
    CC.driverMonitoringEscalation = cs.forceDecel

    lat_tuning = self.CP.lateralTuning.which()
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      cs.lateralControlState.angleState = lac_log
    elif self.CP.steerControlType == car.CarParams.SteerControlType.curvature:
      cs.lateralControlState.curvatureState = lac_log
    elif lat_tuning == 'pid':
      cs.lateralControlState.pidState = lac_log
    elif lat_tuning == 'torque':
      cs.lateralControlState.torqueState = lac_log

    self.pm.send('controlsState', dat)

    # carControl
    cc_send = messaging.new_message('carControl')
    cc_send.valid = CS.canValid
    cc_send.carControl = CC
    self.pm.send('carControl', cc_send)

  def run(self):
    rk = Ratekeeper(100, print_delay_threshold=None)
    while True:
      self.update()
      CC, lac_log = self.state_control()
      self.publish(CC, lac_log)
      self.get_params_sp(self.sm)
      self.run_ext(self.sm, self.pm)
      rk.monitor_time()


def main():
  config_realtime_process(4, Priority.CTRL_HIGH)
  controls = Controls()
  controls.run()


if __name__ == "__main__":
  main()
