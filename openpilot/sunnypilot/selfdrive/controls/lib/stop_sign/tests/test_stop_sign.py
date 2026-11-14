"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math
from types import SimpleNamespace

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_controller import (
  A_COMFORT, COAST_START_M, STOP_MARGIN_M, StopSignAssist, StopSignState,
)
from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_map import (
  EARTH_R, StopSign, find_stop_ahead, stop_signs_from_cell,
)

LAT0, LON0 = 37.0, -122.0
NOW = 1_800_000_000.0


def north_of(lat, lon, metres):
  return lat + math.degrees(metres / EARTH_R), lon


def east_of(lat, lon, metres):
  return lat, lon + math.degrees(metres / (EARTH_R * math.cos(math.radians(lat))))


class FakeParams:
  def __init__(self, values=None):
    self.values = {"StopSignAssist": True, "UbloxAvailable": False, **(values or {})}

  def get_bool(self, key):
    return self.values.get(key, False)


class FakeMap:
  def __init__(self, signs):
    self.signs = signs

  def signs_near(self, lat, lon):
    return self.signs


def gps(lat, lon, bearing=0.0, speed=10.0, fix=True, acc=3.0, t=NOW):
  return SimpleNamespace(latitude=lat, longitude=lon, bearingDeg=bearing, speed=speed, hasFix=fix,
                         horizontalAccuracy=acc, unixTimestampMillis=int(t * 1e3))


def model(stop=False, min_v=10.0):
  return SimpleNamespace(action=SimpleNamespace(shouldStop=stop), velocity=SimpleNamespace(x=[min_v] * 33))


def sm_at(g, m):
  return {"gpsLocation": g, "modelV2": m}


# ---- map lookup ----

def test_cell_parsing_keeps_only_stop_signs():
  cell = {"nodes": [[1, 1.0, 2.0, {"highway": "stop"}], [2, 1.0, 2.0, {"highway": "traffic_signals"}],
                    [3, 1.0, 2.0, {}], ["bad"]]}
  assert stop_signs_from_cell(cell) == [StopSign(1, 1.0, 2.0)]


def test_finds_sign_straight_ahead():
  lat, lon = north_of(LAT0, LON0, 100)
  ahead = find_stop_ahead(LAT0, LON0, 0.0, [StopSign(7, lat, lon)])
  assert ahead is not None and ahead.node_id == 7
  assert abs(ahead.distance - 100) < 0.5 and ahead.lateral < 0.5


def test_ignores_signs_behind_beside_and_too_far():
  behind = StopSign(1, *north_of(LAT0, LON0, -50))
  beside = StopSign(2, *east_of(*north_of(LAT0, LON0, 60), 30))   # 30 m off the line: a crossing road
  far = StopSign(3, *north_of(LAT0, LON0, 400))
  assert find_stop_ahead(LAT0, LON0, 0.0, [behind, beside, far]) is None


def test_picks_nearest_and_respects_skip_and_heading():
  near = StopSign(1, *north_of(LAT0, LON0, 80))
  further = StopSign(2, *north_of(LAT0, LON0, 200))
  assert find_stop_ahead(LAT0, LON0, 0.0, [further, near]).node_id == 1
  assert find_stop_ahead(LAT0, LON0, 0.0, [further, near], skip_ids={1}).node_id == 2
  # heading east: neither is ahead
  assert find_stop_ahead(LAT0, LON0, 90.0, [near, further]) is None


# ---- controller ----

def make(signs, enabled=True):
  return StopSignAssist(stop_map=FakeMap(signs), params=FakeParams({"StopSignAssist": enabled}))


def run_approach(ctrl, sign_m, v0, confirm_from_m=1e9, steps=4000):
  """Drive north toward a sign `sign_m` ahead, following the controller's target with a simple planner."""
  pos, v, log = 0.0, v0, []
  for _ in range(steps):
    lat, lon = north_of(LAT0, LON0, pos)
    dist_to_sign = sign_m - pos
    m = model(stop=dist_to_sign < confirm_from_m)
    ctrl.update(sm_at(gps(lat, lon, speed=v), m), True, False, v, 0.0, NOW)
    target = ctrl.output_v_target
    # stand-in for the planner's cruise loop (get_cruise_accel): a = clip(v_target - v, -1.2, 0); hold speed when unset
    a = 0.0 if target == V_CRUISE_UNSET else max(min(target - v, 0.0), -1.2)
    v = v + a * DT_MDL
    if v < 0.05:  # the car's stopping controller takes it the last bit
      v = 0.0
    pos += v * DT_MDL
    log.append((dist_to_sign, v, ctrl.state, a))
    if ctrl.state == StopSignState.stopped and v == 0.0:
      break
  return pos, v, log


def test_disabled_does_nothing():
  ctrl = make([StopSign(1, *north_of(LAT0, LON0, 40))], enabled=False)
  ctrl.update(sm_at(gps(LAT0, LON0), model(stop=True)), True, False, 10.0, 0.0, NOW)
  assert ctrl.state == StopSignState.disabled and ctrl.output_v_target == V_CRUISE_UNSET


def test_no_long_control_does_nothing():
  ctrl = make([StopSign(1, *north_of(LAT0, LON0, 40))])
  ctrl.update(sm_at(gps(LAT0, LON0), model(stop=True)), False, False, 10.0, 0.0, NOW)
  assert ctrl.state == StopSignState.disabled and ctrl.output_v_target == V_CRUISE_UNSET


def test_idle_until_coast_distance():
  ctrl = make([StopSign(1, *north_of(LAT0, LON0, 250))])
  ctrl.update(sm_at(gps(LAT0, LON0, speed=11.2), model()), True, False, 11.2, 0.0, NOW)
  assert ctrl.state == StopSignState.idle and ctrl.output_v_target == V_CRUISE_UNSET


def test_coasts_from_150ft_at_low_speed():
  v = 6.0  # ~13 mph: braking distance is short, so the 150 ft floor sets the coast point
  just_outside = StopSign(1, *north_of(LAT0, LON0, COAST_START_M + STOP_MARGIN_M + 2))
  ctrl = make([just_outside])
  ctrl.update(sm_at(gps(LAT0, LON0, speed=v), model()), True, False, v, 0.0, NOW)
  assert ctrl.state == StopSignState.idle
  just_inside = StopSign(1, *north_of(LAT0, LON0, COAST_START_M + STOP_MARGIN_M - 2))
  ctrl = make([just_inside])
  ctrl.update(sm_at(gps(LAT0, LON0, speed=v), model()), True, False, v, 0.0, NOW)
  assert ctrl.state == StopSignState.coasting
  assert v - 0.5 < ctrl.output_v_target < v


def test_comfortable_stop_before_the_sign():
  for mph in (15, 25, 35):
    v0 = mph * 0.44704
    sign_m = 300.0
    ctrl = make([StopSign(1, *north_of(LAT0, LON0, sign_m))])
    pos, v, log = run_approach(ctrl, sign_m, v0)
    assert ctrl.state == StopSignState.stopped and v == 0.0, mph
    stop_short = sign_m - pos
    assert 0.0 <= stop_short <= STOP_MARGIN_M + 4.0, (mph, stop_short)
    # gentle all the way: the stand-in planner never needed more than its 1.2 m/s^2 floor for long
    hard = sum(1 for _, _, _, a in log if a <= -1.2 + 1e-6)
    assert hard * DT_MDL < 1.0, (mph, hard * DT_MDL)
    # coasting started at or before 150 ft from the stop point
    first_active = next(d for d, _, s, _ in log if s != StopSignState.idle)
    assert first_active - STOP_MARGIN_M >= COAST_START_M - v0 * DT_MDL - 0.5, (mph, first_active)


def test_braking_profile_is_comfortable():
  # the target follows v^2 = 2 a d: at 20 m to go it should ask for ~sqrt(2 * 1.0 * 20) = 6.3 m/s or less
  ctrl = make([StopSign(1, *north_of(LAT0, LON0, 20 + STOP_MARGIN_M))])
  ctrl.confirmed = True
  ctrl.update(sm_at(gps(LAT0, LON0, speed=11.0), model(stop=True)), True, False, 11.0, 0.0, NOW)
  assert ctrl.state == StopSignState.braking
  assert ctrl.output_v_target <= math.sqrt(2 * A_COMFORT * 20) + 1e-6


def test_without_model_confirmation_only_coasts():
  sign_m = 120.0
  ctrl = make([StopSign(1, *north_of(LAT0, LON0, sign_m))])
  _, v, log = run_approach(ctrl, sign_m, 11.2, confirm_from_m=-1, steps=int(15 / DT_MDL))
  assert all(s in (StopSignState.idle, StopSignState.coasting) for _, _, s, _ in log)
  assert v > 5.0  # it slowed a little, it did not stop


def test_holds_until_gas_then_releases_that_sign():
  sign = StopSign(1, *north_of(LAT0, LON0, 10))
  ctrl = make([sign])
  ctrl.confirmed = True
  ctrl.node_id = 1
  ctrl.distance = 4.0
  g = gps(LAT0, LON0, speed=0.0)
  for _ in range(int(5 / DT_MDL)):
    ctrl.update(sm_at(g, model(stop=True)), True, False, 0.0, 0.0, NOW)
  assert ctrl.state == StopSignState.stopped and ctrl.output_v_target == 0.0
  ctrl.update(sm_at(g, model(stop=True)), True, True, 0.0, 0.0, NOW)   # gas
  assert not ctrl.is_active and 1 in ctrl.done_ids
  # the same sign does not grab the car again once it's moving
  ctrl.update(sm_at(gps(LAT0, LON0, speed=3.0), model(stop=True)), True, False, 3.0, 0.0, NOW)
  assert ctrl.output_v_target == V_CRUISE_UNSET


def test_bad_gps_never_starts_a_stop():
  ctrl = make([StopSign(1, *north_of(LAT0, LON0, 30))])
  for g in (gps(LAT0, LON0, fix=False), gps(LAT0, LON0, acc=50.0), gps(LAT0, LON0, t=NOW - 10)):
    ctrl.update(sm_at(g, model(stop=True)), True, False, 10.0, 0.0, NOW)
    assert ctrl.output_v_target == V_CRUISE_UNSET


# ---- shadow mode ----

def test_shadow_uses_its_own_param():
  sign = StopSign(1, *north_of(LAT0, LON0, 30))
  live_only = StopSignAssist(stop_map=FakeMap([sign]), params=FakeParams({"StopSignAssist": True}), shadow=True)
  live_only.update(sm_at(gps(LAT0, LON0), model(stop=True)), True, False, 10.0, 0.0, NOW)
  assert live_only.state == StopSignState.disabled
  shadow = StopSignAssist(stop_map=FakeMap([sign]), params=FakeParams({"StopSignAssist": False, "StopSignAssistShadow": True}),
                          shadow=True)
  shadow.update(sm_at(gps(LAT0, LON0), model(stop=True)), True, False, 10.0, 0.0, NOW)
  assert shadow.is_active


def test_state_changes_are_logged(monkeypatch=None):
  import openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_controller as mod
  events = []
  orig = mod.cloudlog.event
  mod.cloudlog.event = lambda name, **kw: events.append((name, kw))
  try:
    ctrl = make([StopSign(1, *north_of(LAT0, LON0, 30))])
    for _ in range(3):
      ctrl.update(sm_at(gps(LAT0, LON0), model(stop=True)), True, False, 10.0, 0.0, NOW)
  finally:
    mod.cloudlog.event = orig
  assert len(events) == 1  # one transition (idle -> braking/coasting), not one per tick
  name, kw = events[0]
  assert name == "stop_sign" and kw["sign"] == 1 and kw["state"] in ("coasting", "braking") and kw["shadow"] is False
