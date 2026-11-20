from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.controls.lib.dec.dec import DynamicExperimentalController
from openpilot.sunnypilot.selfdrive.controls.lib.dec.constants import WMACConstants

STOP_CUE_FRAMES = WMACConstants.STOP_CUE_FRAMES

class MockLeadOne:
  def __init__(self, present=0.0):
    self.present = present

class MockRadarState:
  def __init__(self, present=0.0):
    self.leadOne = MockLeadOne(present=present)

class MockCarState:
  def __init__(self, vEgo=0.0, vCruise=0.0, standstill=False):
    self.vEgo = vEgo
    self.vCruise = vCruise
    self.standstill = standstill

class MockModelData:
  def __init__(self, valid=True, plan_v=0.0, endpoint=0.0):
    size = 33 if valid else 10  # incomplete if invalid
    self.position = type("Pos", (), {"x": [endpoint * i / (size - 1) for i in range(size)]})()
    self.orientation = type("Ori", (), {"x": [0.0] * size})()
    self.velocity = type("Vel", (), {"x": [plan_v] * size})()   # plan_v 0: the plan stops (the stop cue)

class MockSelfDriveState:
  def __init__(self, experimentalMode=False):
    self.experimentalMode = experimentalMode

class MockParams:
  def get_bool(self, name):
    return True

def default_sm():
  sm = {
    'carState': MockCarState(vEgo=10.0, vCruise=20.0),
    'radarState': MockRadarState(present=1.0),
    'modelV2': MockModelData(valid=True),
    'selfdriveState': MockSelfDriveState(experimentalMode=True),
  }
  return sm

def mock_cp():
  class CP:
    radarUnavailable = False
  return CP()

def mock_mpc():
  class MPC:
    crash_cnt = 0
  return MPC()

# Fake Kalman Filter that always returns a given value
class FakeKalman:
  def __init__(self, value=1.0):
    self.value = value
  def add_data(self, v): pass
  def get_value(self): return self.value
  def get_confidence(self): return 1.0
  def reset_data(self): pass

class TestDynamicExperimentalController(OpenpilotTestCase):
  def test_initial_mode_is_acc(self, mock_cp, mock_mpc):
    controller = DynamicExperimentalController(mock_cp, mock_mpc, params=MockParams())
    assert controller.mode() == "acc"

  def test_standstill_triggers_blended(self, mock_cp, mock_mpc, default_sm):
    controller = DynamicExperimentalController(mock_cp, mock_mpc, params=MockParams())
    default_sm['carState'].standstill = True
    for _ in range(STOP_CUE_FRAMES + 10):
      controller.update(default_sm)
    assert controller.mode() == "blended"

  def test_emergency_blended_on_fcw(self, mock_cp, mock_mpc, default_sm):
    controller = DynamicExperimentalController(mock_cp, mock_mpc, params=MockParams())
    mock_mpc.crash_cnt = 1  # simulate FCW
    for _ in range(2):
      controller.update(default_sm)
    assert controller.mode() == "blended"

  def test_radarless_slowdown_triggers_blended(self, mock_cp, mock_mpc, default_sm):
    mock_cp.radarUnavailable = True
    controller = DynamicExperimentalController(mock_cp, mock_mpc, params=MockParams())

    # Force conditions to simulate slowdown
    controller._slow_down_filter = FakeKalman(value=1.0)  # ty: ignore[invalid-assignment]
    controller._v_ego_kph = 35.0
    default_sm['modelV2'] = MockModelData(valid=False)  # Incomplete trajectory

    for _ in range(STOP_CUE_FRAMES + 3):
      controller.update(default_sm)

    assert controller.mode() == "blended"

  def _drive(self, controller, sm, plan_v, n, endpoint=20.0):
    sm['modelV2'] = MockModelData(valid=True, plan_v=plan_v, endpoint=endpoint)
    for _ in range(n):
      controller.update(sm)

  def test_a_plan_that_only_slows_is_no_stop(self, mock_cp, mock_mpc, default_sm):
    # Warm Springs Blvd (route 0000010d, 2026-11-17): the plan slowed from 40 to 16 mph with no lead; no hand-over
    controller = DynamicExperimentalController(mock_cp, mock_mpc, params=MockParams())
    default_sm['radarState'] = MockRadarState(present=0.0)
    default_sm['carState'] = MockCarState(vEgo=18.0, vCruise=72.0)
    self._drive(controller, default_sm, plan_v=7.0, n=100, endpoint=60.0)   # short plan, but 7 m/s at its slowest
    assert controller.mode() == "acc"

  def test_a_sustained_stop_plan_hands_over_a_blip_does_not(self, mock_cp, mock_mpc, default_sm):
    default_sm['radarState'] = MockRadarState(present=0.0)
    default_sm['carState'] = MockCarState(vEgo=15.0, vCruise=72.0)
    controller = DynamicExperimentalController(mock_cp, mock_mpc, params=MockParams())
    for _ in range(5):     # half-second stop blips between steady plans
      self._drive(controller, default_sm, plan_v=0.0, n=STOP_CUE_FRAMES // 2)
      self._drive(controller, default_sm, plan_v=15.0, n=5, endpoint=120.0)
    assert controller.mode() == "acc"
    self._drive(controller, default_sm, plan_v=0.0, n=STOP_CUE_FRAMES + 5)   # a red light: the plan stops and stays
    assert controller.mode() == "blended"
