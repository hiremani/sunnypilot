"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math
from dataclasses import replace
from types import SimpleNamespace

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_controller import (
  A_COMFORT, COAST_START_M, CONFIRM_FILTER_S, CONFIRM_THRESHOLD, STOP_MARGIN_M, STOP_V_TARGET, StopSignAssist,
  StopSignState, braking_v_target, confirm_distance, lead_relevant, model_stop_cue,
)
from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign import road_geometry
from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.road_geometry import RoadGeometry, WayAtNode
from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_direction import (
  Applies, angle_diff, approaches_from_ways, parse_direction, sign_applies, travel_from_facing,
)
from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_map import (
  EARTH_R, StopSign, StopSignMap, find_stop_ahead, local_stop_signs, resolve_approaches, stop_signs_from_cell,
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


T_IDXS = [10.0 * (i / 32) ** 2 for i in range(33)]   # the model plan's time base (ModelConstants.T_IDXS)


def plan(v0, decel=0.0, v_floor=0.0, should_stop=False):
  """A model plan from v0 slowing at `decel` m/s^2 down to v_floor (decel 0: a steady plan)."""
  vel, pos, x = [], [], 0.0
  for i, t in enumerate(T_IDXS):
    v = max(v0 - decel * t, v_floor)
    if i:
      x += (vel[-1] + v) / 2 * (t - T_IDXS[i - 1])
    vel.append(v)
    pos.append(x)
  return SimpleNamespace(action=SimpleNamespace(shouldStop=should_stop), velocity=SimpleNamespace(x=vel),
                         position=SimpleNamespace(x=pos))


def model(stop=False, min_v=10.0):
  """stop: a plan that comes to a halt within a few metres; otherwise a steady plan at min_v."""
  return plan(1.0, decel=1.0) if stop else plan(min_v)


def sm_at(g, m, lead=None):
  sm = {"gpsLocation": g, "modelV2": m}
  if lead is not None:
    sm["radarState"] = SimpleNamespace(leadOne=lead)
  return sm


def lead_at(d_rel, present=True):
  return SimpleNamespace(present=present, dRel=d_rel)


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

def make(signs, enabled=True, max_approach_v=99.0):
  """The controller with the approach-speed cap lifted by default: most tests exercise the profiles at any speed."""
  ctrl = StopSignAssist(stop_map=FakeMap(signs), params=FakeParams({"StopSignAssist": enabled}))
  ctrl.max_approach_v = max_approach_v
  return ctrl


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
  assert ctrl.output_v_target == v          # engaged (no speeding up), holding speed until the lift-off window
  from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_controller import lift_off_distance
  near = StopSign(1, *north_of(LAT0, LON0, lift_off_distance(v) + STOP_MARGIN_M - 1))
  ctrl = make([near])
  ctrl.update(sm_at(gps(LAT0, LON0, speed=v), model()), True, False, v, 0.0, NOW)
  assert v - 0.5 < ctrl.output_v_target < v  # inside it: the lift-off coast


def test_slow_confirmed_approach_does_not_crawl():
  # 00000116 at 1:55: 7.4 mph, the stop confirmed 28 m before the stop point; it coasted 8 s, 5.6 mph most of it
  from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_controller import COAST_FLOOR_V
  v0, sign_m = 3.3, 34.0 + STOP_MARGIN_M
  ctrl = make([StopSign(1, *north_of(LAT0, LON0, sign_m))])
  pos, v, log = run_approach(ctrl, sign_m, v0)
  assert ctrl.state == StopSignState.stopped and v == 0.0
  assert 0.0 <= sign_m - pos <= STOP_MARGIN_M + 4.0
  crawl_s = sum(1 for d, vv, s, _ in log if 0.2 < vv <= COAST_FLOOR_V + 0.1 and s == StopSignState.coasting) * DT_MDL
  assert crawl_s < 2.5, crawl_s
  slow_s = sum(1 for _, vv, _, _ in log if 0.2 < vv <= 2.7) * DT_MDL
  assert slow_s < 10.0, slow_s              # was 13.4 s under 6 mph (8.7 s crawling); the soft landing takes ~4.5 s of it


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
  ctrl.node_id = 1   # already this sign's: a newly found sign starts unconfirmed, and one frame no longer confirms
  ctrl.confirmed = True
  ctrl.state = StopSignState.coasting   # already taken on this sign
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


# ---- model confirmation ----

def test_stop_cue_reads_the_shape_of_the_plan():
  # slowing 10 -> 6 m/s: nowhere near a stop, yet the plan's slowest point is under 70% of the current speed
  cue = model_stop_cue(plan(10.0, decel=1.0, v_floor=6.0), 10.0)
  assert cue.slowing and cue.raw and abs(cue.v_min - 6.0) < 1e-6
  assert not model_stop_cue(plan(10.0, decel=1.0, v_floor=7.5), 10.0).raw       # a mild slowdown is not a cue
  # the floor keeps it working at low speed: 1.2 -> 0.9 m/s is no 30% drop, but the plan is all but stopped
  assert model_stop_cue(plan(1.2, decel=0.5, v_floor=0.9), 1.2).slowing
  # a steady plan is no cue at any speed, whichever test reads it (the end-point test is off at low speed)
  for v in (0.5 + 1.0, 2.0, 3.0, 5.0, 7.0, 10.0, 15.0, 20.0, 30.0):
    cue = model_stop_cue(plan(v), v)
    assert not cue.raw, (v, cue)
  # end-point shortfall on its own: speeds that stay above the ratio test but a plan that ends well short
  short = plan(10.0)
  short.position.x = [x * 0.6 for x in short.position.x]
  cue = model_stop_cue(short, 10.0)
  assert cue.short and not cue.slowing and cue.raw and 0.3 < cue.shortfall < 0.5
  short.position.x = [x * 0.85 for x in plan(10.0).position.x]
  assert not model_stop_cue(short, 10.0).raw
  # shouldStop is a standstill-hold flag, not a cue; an empty or missing plan is no cue either
  assert not model_stop_cue(plan(10.0, should_stop=True), 10.0).raw
  empty = SimpleNamespace(action=SimpleNamespace(shouldStop=True), velocity=SimpleNamespace(x=[]), position=SimpleNamespace(x=[]))
  assert not model_stop_cue(empty, 10.0).raw and not model_stop_cue(SimpleNamespace(), 10.0).raw


def approach(ctrl, sign_m, v, model_at, seconds, lead=None):
  """Drive north at a constant v toward a sign, model_at(frame) giving each frame's plan. Returns the frame at which
  the controller confirmed (None if it never did)."""
  confirmed_at = None
  for i in range(int(seconds / DT_MDL)):
    lat, lon = north_of(LAT0, LON0, v * DT_MDL * i)
    ctrl.update(sm_at(gps(lat, lon, speed=v), model_at(i), lead), True, False, v, 0.0, NOW)
    if ctrl.confirmed and confirmed_at is None:
      confirmed_at = i
  return confirmed_at


def test_confirms_on_a_decelerating_plan_long_before_a_stop():
  # 10 m/s, the model planning to slow to 4 m/s: its plan never dips under the old fixed 1.5 m/s test
  ctrl = make([StopSign(1, *north_of(LAT0, LON0, 90))])
  at = approach(ctrl, 90, 10.0, lambda i: plan(10.0, decel=1.0, v_floor=4.0), 4.0)
  assert at is not None and ctrl.confirm_filter.x >= CONFIRM_THRESHOLD
  # ...after about one filter time constant of agreement, still at full speed
  assert CONFIRM_FILTER_S - 0.2 <= at * DT_MDL <= CONFIRM_FILTER_S + 0.3, at
  assert ctrl.state == StopSignState.braking or ctrl.output_v_target < 10.0


def test_one_noisy_frame_does_not_confirm():
  ctrl = make([StopSign(1, *north_of(LAT0, LON0, 90))])
  stop_frames = {10}
  at = approach(ctrl, 90, 10.0, lambda i: plan(10.0, decel=3.0) if i in stop_frames else plan(10.0), 4.0)
  assert at is None and not ctrl.confirmed and ctrl.confirm_filter.x < 0.1
  # nor does a cue that flickers a quarter of the time
  ctrl = make([StopSign(1, *north_of(LAT0, LON0, 90))])
  at = approach(ctrl, 90, 10.0, lambda i: plan(10.0, decel=3.0) if i % 4 == 0 else plan(10.0), 4.0)
  assert at is None and ctrl.confirm_filter.x < CONFIRM_THRESHOLD


def test_constant_speed_plan_never_confirms():
  for v in (6.0, 10.0, 15.0):
    ctrl = make([StopSign(1, *north_of(LAT0, LON0, 200))])   # far enough to be taken on in time even at 15 m/s
    assert approach(ctrl, 200, v, lambda i, v=v: plan(v, should_stop=True), 180 / v) is None, v
    assert ctrl.confirm_filter.x == 0.0 and ctrl.state == StopSignState.coasting, v


def test_relevant_lead_excludes_the_cue():
  slowing = lambda i: plan(10.0, decel=1.0)   # stops in 50 m  # noqa: E731
  ctrl = make([StopSign(1, *north_of(LAT0, LON0, 90))])
  assert approach(ctrl, 90, 10.0, slowing, 4.0, lead=lead_at(30.0)) is None     # the plan slows for the car in front
  assert ctrl.confirm_filter.x == 0.0 and ctrl.cue.raw and ctrl.cue_lead
  # a lead far beyond where the plan ends, or none, is not the reason
  for lead in (lead_at(120.0), lead_at(30.0, present=False), None):
    ctrl = make([StopSign(1, *north_of(LAT0, LON0, 90))])
    assert approach(ctrl, 90, 10.0, slowing, 4.0, lead=lead) is not None, lead
  assert lead_relevant(sm_at(None, plan(10.0, decel=1.0), lead_at(55.0)), plan(10.0, decel=1.0))
  assert not lead_relevant(sm_at(None, plan(10.0, decel=1.0)), plan(10.0, decel=1.0))   # no radarState in sm


def test_confirmation_latches_and_resets_per_sign():
  first, second = StopSign(1, *north_of(LAT0, LON0, 60)), StopSign(2, *north_of(LAT0, LON0, 260))
  ctrl = make([first, second])
  # the cue is there for 2 s, then gone: the first sign stays confirmed
  assert approach(ctrl, 60, 5.0, lambda i: plan(5.0, decel=1.0) if i < 40 else plan(5.0), 4.0) is not None
  assert ctrl.node_id == 1 and ctrl.confirmed
  ctrl.update(sm_at(gps(*north_of(LAT0, LON0, 20), speed=5.0), plan(5.0)), True, True, 5.0, 0.0, NOW)   # gas: sign 1 is done
  assert ctrl.node_id == 2 and not ctrl.confirmed and ctrl.confirm_filter.x == 0.0


def test_slowdown_far_from_the_sign_is_not_about_it():
  v = 10.0
  far = confirm_distance(v) + STOP_MARGIN_M + 60
  ctrl = make([StopSign(1, *north_of(LAT0, LON0, far))])
  assert approach(ctrl, far, v, lambda i: plan(v, decel=1.0), 3.0) is None      # 30 m travelled: still out of range
  assert ctrl.node_id == 1 and ctrl.confirm_filter.x == 0.0
  assert approach(ctrl, far, v, lambda i: plan(v, decel=1.0), 3.0) is None      # (the helper restarts from LAT0)
  near = StopSign(1, *north_of(LAT0, LON0, confirm_distance(v) + STOP_MARGIN_M - 5))
  ctrl = make([near])
  assert approach(ctrl, 0, v, lambda i: plan(v, decel=1.0), 3.0) is not None


def test_confirmation_is_logged(monkeypatch=None):
  import openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_controller as mod
  events = []
  orig = mod.cloudlog.event
  mod.cloudlog.event = lambda name, **kw: events.append((name, kw))
  try:
    ctrl = make([StopSign(1, *north_of(LAT0, LON0, 90))])
    approach(ctrl, 90, 10.0, lambda i: plan(10.0, decel=1.0, v_floor=4.0), 4.0)
  finally:
    mod.cloudlog.event = orig
  latched = [kw for name, kw in events if name == "stop_sign_confirmed"]
  assert len(latched) == 1   # once per sign
  kw = latched[0]
  assert kw["sign"] == 1 and kw["v"] == 10.0 and 55.0 < kw["dist"] < 75.0 and kw["shadow"] is False
  assert kw["confirm_raw"] is True and kw["confirm_filt"] >= round(CONFIRM_THRESHOLD, 3) and kw["plan_v_min"] == 4.0
  # every state change carries the confirmer's raw and filtered values
  changes = [kw for name, kw in events if name == "stop_sign"]
  assert changes and all({"confirm_raw", "confirm_filt", "plan_v_min", "plan_shortfall", "lead"} <= kw.keys() for kw in changes)
  assert changes[0]["confirm_filt"] < CONFIRM_THRESHOLD and changes[0]["confirmed"] is False
  assert changes[-1]["state"] == "braking" and changes[-1]["confirmed"] is True


def test_stop_target_is_zero_not_a_rolling_speed():
  assert STOP_V_TARGET == 0.0
  assert braking_v_target(0.0, 0.0) == 0.0 and braking_v_target(-3.0, 2.0) == 0.0 and braking_v_target(1.5, 2.0) == 0.0
  # the profile only ever falls toward zero as the stop point nears
  targets = [braking_v_target(d, 0.0) for d in (30.0, 10.0, 3.0, 1.0, 0.1, 0.0)]
  assert targets == sorted(targets, reverse=True) and targets[-1] == 0.0
  # a rolling approach (2.4 mph) at the stop point is still asked for zero, and the hold is at zero
  ctrl = make([StopSign(1, *north_of(LAT0, LON0, STOP_MARGIN_M + 0.5))])
  ctrl.node_id = 1
  ctrl.confirmed = True
  ctrl.state = StopSignState.coasting   # already taken on this sign
  ctrl.update(sm_at(gps(LAT0, LON0, speed=2.2), plan(2.2)), True, False, 1.07, 0.0, NOW)
  assert ctrl.state == StopSignState.braking and ctrl.output_v_target == 0.0


# ---- shadow mode ----

def test_shadow_uses_its_own_param():
  sign = StopSign(1, *north_of(LAT0, LON0, 70))   # inside the coast distance at 10 m/s, with room to stop
  live_only = StopSignAssist(stop_map=FakeMap([sign]), params=FakeParams({"StopSignAssist": True}), shadow=True)
  live_only.update(sm_at(gps(LAT0, LON0), model(stop=True)), True, False, 10.0, 0.0, NOW)
  assert live_only.state == StopSignState.disabled
  shadow = StopSignAssist(stop_map=FakeMap([sign]), params=FakeParams({"StopSignAssist": False, "StopSignAssistShadow": True}),
                          shadow=True)
  shadow.max_approach_v = 99.0
  shadow.update(sm_at(gps(LAT0, LON0), model(stop=True)), True, False, 10.0, 0.0, NOW)
  assert shadow.is_active


def test_state_changes_are_logged(monkeypatch=None):
  import openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_controller as mod
  events = []
  orig = mod.cloudlog.event
  mod.cloudlog.event = lambda name, **kw: events.append((name, kw))
  try:
    ctrl = make([StopSign(1, *north_of(LAT0, LON0, 70))])   # with room for a comfortable stop at 10 m/s
    for _ in range(3):
      ctrl.update(sm_at(gps(LAT0, LON0), model(stop=True)), True, False, 10.0, 0.0, NOW)
  finally:
    mod.cloudlog.event = orig
  assert len(events) == 1  # one transition (idle -> braking/coasting), not one per tick
  name, kw = events[0]
  assert name == "stop_sign" and kw["sign"] == 1 and kw["state"] in ("coasting", "braking") and kw["shadow"] is False


def test_two_nodes_at_one_junction_dont_flip_flop():
  a = StopSign(1, *north_of(LAT0, LON0, 100))
  b = StopSign(2, *north_of(LAT0, LON0, 104))   # second node mapped for the same junction
  assert find_stop_ahead(LAT0, LON0, 0.0, [a, b]).node_id == 1
  assert find_stop_ahead(LAT0, LON0, 0.0, [a, b], prefer_id=2).node_id == 2   # keeps the one it had
  far = StopSign(3, *north_of(LAT0, LON0, 180))  # a different junction further on is not "the same"
  assert find_stop_ahead(LAT0, LON0, 0.0, [a, far], prefer_id=3).node_id == 1


def test_controller_keeps_its_sign_through_gps_jitter():
  a = StopSign(1, *north_of(LAT0, LON0, 120))
  b = StopSign(2, *north_of(LAT0, LON0, 121))
  ctrl = make([a, b])
  ids = set()
  for i in range(40):
    jitter = 1.5 if i % 2 else -1.5   # GPS wandering sideways a little
    lat, lon = east_of(LAT0, LON0, jitter)
    ctrl.update(sm_at(gps(lat, lon, speed=8.0), model()), True, False, 8.0, 0.0, NOW)
    ids.add(ctrl.node_id)
  assert len(ids) == 1


def test_prefetch_tiles_cover_the_radius():
  from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.prefetch import tiles_around
  from openpilot.sunnypilot.webhud.maptiles import FEAT_TILE_DEG
  tiles = tiles_around(37.5, -121.95, 15.0)
  # every corner of the 15 km box lands in a returned tile
  for dlat, dlon in ((0.13, 0.165), (-0.13, -0.165), (0.13, -0.165), (-0.13, 0.165)):   # just inside 15 km
    t = (math.floor((37.5 + dlat) / FEAT_TILE_DEG), math.floor((-121.95 + dlon) / FEAT_TILE_DEG))
    assert t in tiles
  assert len(tiles) <= 9   # 0.25 deg tiles: a 30 km box spans at most 3 x 3


# ---- direction of travel ----


def tagged(node_id, lat, lon, **tags):
  return stop_signs_from_cell({"nodes": [[node_id, lat, lon, {"highway": "stop", **tags}]]})[0]


def test_parse_direction():
  assert parse_direction("90") == (90.0,)
  assert parse_direction(" 370.5 ") == (10.5,)
  assert parse_direction("NE") == (45.0,) and parse_direction("nnw") == (337.5,)
  assert parse_direction("N;S") == (0.0, 180.0)
  assert parse_direction("Forward") == "forward" and parse_direction("backward") == "backward"
  assert parse_direction("both") == "both" and parse_direction(" Both ") == "both"
  for bad in (None, "", "all", "45-135", "nan", 90):
    assert parse_direction(bad) is None, bad


def trusting_compass(fn):
  """Run fn with numeric/cardinal directions trusted (they're UNKNOWN by default, see stop_sign_direction)."""
  def wrapper(*args, **kwargs):
    from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign import stop_sign_direction
    saved = stop_sign_direction.COMPASS_DIRECTIONS_TRUSTED
    stop_sign_direction.COMPASS_DIRECTIONS_TRUSTED = True
    try:
      return fn(*args, **kwargs)
    finally:
      stop_sign_direction.COMPASS_DIRECTIONS_TRUSTED = saved
  wrapper.__name__ = fn.__name__
  return wrapper


def test_compass_directions_are_unknown_until_verified():
  # the convention isn't verified yet: a sign tagged with a bearing must never be skipped because of it
  for direction in ("0", "180", "E", "W", "NE;SW"):
    sign = tagged(1, *north_of(LAT0, LON0, 80), direction=direction)
    assert sign.approaches is None and sign_applies(sign, 0.0) == Applies.UNKNOWN, direction
    assert find_stop_ahead(LAT0, LON0, 0.0, [sign]).node_id == 1, direction


@trusting_compass
def test_numeric_direction_is_the_way_the_sign_faces():
  # were compass values ever trusted, they'd be read as the wiki defines them: the sign FACES that way, so the
  # traffic it holds travels the opposite way ("traveling north ... the sign is facing south ... direction=180")
  assert travel_from_facing((180.0, 350.0)) == (0.0, 170.0)
  faces_south = tagged(1, *north_of(LAT0, LON0, 80), direction="180")   # holds northbound traffic
  assert faces_south.approaches == (0.0,)
  assert sign_applies(faces_south, 0.0) == Applies.YES
  assert sign_applies(faces_south, 350.0) == Applies.YES       # wraps around north
  assert sign_applies(faces_south, 90.0) == Applies.NO         # crossing traffic
  assert sign_applies(faces_south, 180.0) == Applies.NO        # oncoming
  assert sign_applies(faces_south, 70.0) == Applies.UNKNOWN    # in between: no decision
  # find_stop_ahead skips it for a car the sign isn't for, keeps it for one it is
  assert find_stop_ahead(LAT0, LON0, 0.0, [faces_south]).node_id == 1
  faces_north = tagged(2, *north_of(LAT0, LON0, 80), direction="0")     # holds southbound traffic
  assert find_stop_ahead(LAT0, LON0, 0.0, [faces_north]) is None
  # the road-tile path reads them the same way
  assert approaches_from_ways("180", [WayAtNode(10, "residential", 1, 3, 0.0, 180.0)]) == (0.0,)


@trusting_compass
def test_cardinal_direction_is_the_way_the_sign_faces():
  sign = tagged(1, *east_of(LAT0, LON0, 60), direction="W")   # faces west: holds eastbound traffic
  assert sign.approaches == (90.0,)
  assert find_stop_ahead(LAT0, LON0, 90.0, [sign]).node_id == 1
  east = tagged(2, *east_of(LAT0, LON0, 60), direction="E")
  assert find_stop_ahead(LAT0, LON0, 90.0, [east]) is None
  assert sign_applies(tagged(3, LAT0, LON0, direction="NE;SW"), 225.0) == Applies.YES


def test_all_way_stop_always_applies():
  for direction in (None, "180", "0", "both", "45-135"):
    tags = {"stop": "all"} if direction is None else {"stop": "all", "direction": direction}
    sign = tagged(1, *north_of(LAT0, LON0, 80), **tags)
    assert sign.all_way and sign.approaches is None and not sign.needs_geometry, direction
    for bearing in (0.0, 90.0, 180.0):
      assert sign_applies(sign, bearing) == Applies.YES, (direction, bearing)
    assert find_stop_ahead(LAT0, LON0, 0.0, [sign]).node_id == 1, direction


@trusting_compass
def test_all_way_stop_is_never_narrowed_by_a_compass_value():
  for direction in ("0", "180", "E"):
    sign = tagged(1, *north_of(LAT0, LON0, 80), direction=direction, stop="all")
    assert sign.approaches is None and sign_applies(sign, 0.0) == Applies.YES, direction
    assert approaches_from_ways(direction, [WayAtNode(10, "residential", 1, 3, 0.0, 180.0)], explicit_only=True) is None


def test_all_way_stop_with_explicit_direction_holds_one_approach():
  # stop=all is put on every per-approach node of a detailed all-way stop: forward/backward on a single way outranks it
  mid_way = [WayAtNode(10, "residential", 1, 3, forward_bearing=0.0, backward_bearing=180.0)]
  junction = mid_way + [WayAtNode(11, "residential", 4, 9, 90.0, 270.0)]
  fwd = tagged(1, *north_of(LAT0, LON0, 80), stop="all", direction="forward")
  bwd = tagged(2, *north_of(LAT0, LON0, 80), stop="all", direction="backward")
  assert fwd.needs_geometry and bwd.needs_geometry and sign_applies(bwd, 0.0) == Applies.YES   # unresolved: all approaches
  assert approaches_from_ways("backward", mid_way, explicit_only=True) == (180.0,)
  bwd_resolved = replace(bwd, approaches=(180.0,))
  assert sign_applies(bwd_resolved, 0.0) == Applies.NO      # forward traffic: the opposing approach's node
  assert sign_applies(bwd_resolved, 180.0) == Applies.YES
  assert find_stop_ahead(LAT0, LON0, 0.0, [bwd_resolved]) is None
  assert find_stop_ahead(LAT0, LON0, 0.0, [replace(fwd, approaches=(0.0,))]).node_id == 1
  # nothing but an explicit forward/backward on a single way narrows it
  assert approaches_from_ways("backward", junction, explicit_only=True) is None   # a junction node
  assert approaches_from_ways("backward", None, explicit_only=True) is None       # no road tile
  minor_end = WayAtNode(10, "residential", 4, 5, forward_bearing=0.0, backward_bearing=None)
  major = WayAtNode(11, "secondary", 3, 9, forward_bearing=90.0, backward_bearing=270.0)
  assert approaches_from_ways(None, [minor_end, major], explicit_only=True) is None
  assert approaches_from_ways(None, [minor_end], explicit_only=True) is None


def test_direction_both_applies_to_both_directions():
  sign = tagged(1, *north_of(LAT0, LON0, 80), direction="both")
  assert sign.approaches is None and not sign.needs_geometry
  for bearing in (0.0, 180.0, 90.0):
    assert sign_applies(sign, bearing) == Applies.YES, bearing
  assert find_stop_ahead(LAT0, LON0, 0.0, [sign]).node_id == 1
  assert approaches_from_ways("both", [WayAtNode(10, "residential", 1, 3, 0.0, 180.0, 5.0, 200.0)]) is None   # not narrowed


def test_direction_fallback_keys():
  where = north_of(LAT0, LON0, 80)
  assert tagged(1, *where, **{"stop:direction": "backward"}).direction == "backward"
  assert tagged(1, *where, **{"traffic_sign:direction": "Forward"}).direction == "Forward"
  # `direction` wins when present, then stop:direction
  both_keys = {"direction": "forward", "stop:direction": "backward", "traffic_sign:direction": "both"}
  assert tagged(1, *where, **both_keys).direction == "forward"
  assert tagged(1, *where, **{"stop:direction": "backward", "traffic_sign:direction": "both"}).direction == "backward"
  assert tagged(1, *where, **{"direction": 5, "stop:direction": "backward"}).direction == "backward"   # not a string
  assert tagged(1, *where).direction is None
  # ...and they resolve like `direction` does
  way = [WayAtNode(10, "residential", 1, 3, forward_bearing=0.0, backward_bearing=180.0)]
  sign = resolve_approaches([tagged(1, *where, **{"stop:direction": "backward"})], SimpleNamespace(ways_at=lambda lat, lon: way))[0]
  assert sign.approaches == (180.0,) and find_stop_ahead(LAT0, LON0, 0.0, [sign]) is None
  assert sign_applies(tagged(1, *where, **{"traffic_sign:direction": "both"}), 0.0) == Applies.YES


def test_untagged_mid_way_stop_holds_traffic_toward_the_nearer_end():
  def mid(start_m, end_m):
    return [WayAtNode(10, "residential", 2, 5, forward_bearing=0.0, backward_bearing=180.0, start_m=start_m, end_m=end_m)]
  assert approaches_from_ways(None, mid(200.0, 8.0)) == (0.0,)       # the end is the junction: travel in node order
  assert approaches_from_ways(None, mid(8.0, 200.0)) == (180.0,)     # the start is: travel against node order
  assert approaches_from_ways(None, mid(40.0, 80.0)) == (180.0,)     # just inside both limits
  # no end is clearly nearer: UNKNOWN
  assert approaches_from_ways(None, mid(100.0, 100.0)) is None       # the middle of the block
  assert approaches_from_ways(None, mid(30.0, 45.0)) is None         # near, but not at most half the other distance
  assert approaches_from_ways(None, mid(41.0, 500.0)) is None        # clearly nearer, but too far to be its stop line
  assert approaches_from_ways(None, mid(None, None)) is None         # geometry without distances
  # a stop=all node is not narrowed by the rule, and an unreadable direction is not guessed around
  assert approaches_from_ways(None, mid(200.0, 8.0), explicit_only=True) is None
  assert approaches_from_ways("45-135", mid(200.0, 8.0)) is None


def test_unknown_direction_keeps_the_sign():
  for tags in ({}, {"direction": "45-135"}, {"direction": "forward"}, {"stop": "minor"}):
    sign = tagged(1, *north_of(LAT0, LON0, 80), **tags)
    assert sign.approaches is None and sign_applies(sign, 0.0) == Applies.UNKNOWN, tags
    assert find_stop_ahead(LAT0, LON0, 0.0, [sign]).node_id == 1, tags


def test_forward_backward_from_way_order():
  # a way drawn south -> north, the sign on its middle node: forward = northbound travel
  way = [WayAtNode(10, "residential", 1, 3, forward_bearing=0.0, backward_bearing=180.0)]
  assert approaches_from_ways("forward", way) == (0.0,)
  assert approaches_from_ways("backward", way) == (180.0,)
  assert approaches_from_ways("forward", None) is None                       # tile not there
  two = way + [WayAtNode(11, "secondary", 4, 9, 90.0, 270.0)]
  assert approaches_from_ways("forward", two) is None                        # a junction node: ambiguous
  assert approaches_from_ways("90", two) is None                             # numeric: UNKNOWN until COMPASS_DIRECTIONS_TRUSTED


def test_untagged_stop_on_a_junction_holds_the_minor_road():
  minor_end = WayAtNode(10, "residential", 4, 5, forward_bearing=0.0, backward_bearing=None)  # ends here, northbound
  major = WayAtNode(11, "secondary", 3, 9, forward_bearing=90.0, backward_bearing=270.0)
  assert approaches_from_ways(None, [minor_end, major]) == (0.0,)
  assert approaches_from_ways(None, [minor_end]) == (0.0,)                   # end of a single way
  assert approaches_from_ways(None, [WayAtNode(10, "residential", 2, 5, 0.0, 180.0)]) is None   # mid-way
  assert approaches_from_ways(None, [minor_end, replace(major, highway="residential")]) is None   # equal roads


TLAT, TLON = 37.1, -121.9   # inside one 0.25 deg road tile (LAT0, LON0 sit on a tile corner)


def write_tile(root, ways):
  """A packed road tile under root, as mapd stores it; ways: [(id, highway class index, [(lat, lon), ...])]."""
  import os
  import capnp
  schema = capnp.load(road_geometry.SCHEMA_PATH)
  lat, lon = ways[0][2][0]
  c_lat, c_lon, name = road_geometry.tile_name(lat, lon)
  msg = schema.Offline.new_message()
  out = msg.init("ways", len(ways))
  for w, (way_id, cls, pts) in zip(out, ways, strict=True):
    w.id, w.highway = way_id, cls
    w.minLat, w.maxLat = min(p[0] for p in pts), max(p[0] for p in pts)
    w.minLon, w.maxLon = min(p[1] for p in pts), max(p[1] for p in pts)
    nodes = w.init("nodes", len(pts))
    for n, (plat, plon) in zip(nodes, pts, strict=True):
      n.latitude, n.longitude = plat, plon
  os.makedirs(os.path.join(root, str(c_lat), str(c_lon)), exist_ok=True)
  with open(os.path.join(root, str(c_lat), str(c_lon), name), "wb") as f:
    f.write(msg.to_bytes_packed())


def test_road_tile_schema_matches_the_pages_reader():
  # osmtile.js decodeTile reads real tiles at these offsets; the schema the tests write fixtures with must agree
  import struct
  import capnp
  schema = capnp.load(road_geometry.SCHEMA_PATH)
  w = schema.Way.new_message(maxSpeed=1.0, minLat=2.0, minLon=3.0, maxLat=4.0, maxLon=5.0, lanes=6, highway=12, oneWay=True,
                             advisorySpeed=7.0, maxSpeedForward=8.0, maxSpeedBackward=9.0, id=1234567890123)
  b = w.to_bytes()
  assert struct.unpack_from("<HH", b, 12) == (10, 4)   # 10 data words, 4 pointers
  d = b[16:]
  assert struct.unpack_from("<5d", d, 0) == (1.0, 2.0, 3.0, 4.0, 5.0)
  assert d[40] == 6 and d[41] & 1 == 1 and struct.unpack_from("<H", d, 42)[0] == 12
  assert struct.unpack_from("<3dq", d, 48) == (7.0, 8.0, 9.0, 1234567890123)


def test_road_geometry_finds_the_ways_at_a_node(tmp_path):
  j = (TLAT, TLON)
  minor = [north_of(TLAT, TLON, -100), north_of(TLAT, TLON, -40), j]             # residential, ends at j heading north
  major = [east_of(TLAT, TLON, -100), j, east_of(TLAT, TLON, 100)]               # secondary through j, eastbound order
  write_tile(str(tmp_path), [(10, 12, minor), (11, 7, major)])
  geo = RoadGeometry(root=str(tmp_path))
  ways = sorted(geo.ways_at(*j), key=lambda w: w.way_id)
  assert [(w.way_id, w.highway, w.index, w.count) for w in ways] == [(10, "residential", 2, 3), (11, "secondary", 1, 3)]
  assert abs(ways[0].forward_bearing - 0.0) < 0.5 and ways[0].backward_bearing is None
  assert abs(ways[1].forward_bearing - 90.0) < 0.5 and abs(ways[1].backward_bearing - 270.0) < 0.5
  assert abs(ways[0].start_m - 100.0) < 0.5 and ways[0].end_m == 0.0      # distance along the way to each end
  assert abs(ways[1].start_m - 100.0) < 0.5 and abs(ways[1].end_m - 100.0) < 0.5
  mid = geo.ways_at(*north_of(TLAT, TLON, -40))
  assert len(mid) == 1 and abs(mid[0].start_m - 60.0) < 0.5 and abs(mid[0].end_m - 40.0) < 0.5
  assert geo.ways_at(*north_of(TLAT, TLON, -70)) == []         # on no node
  assert RoadGeometry(root=str(tmp_path / "none")).ways_at(*j) is None


def test_two_way_stop_skipped_from_the_major_road(tmp_path):
  j = (TLAT, TLON)
  mid = north_of(TLAT, TLON, -40)
  write_tile(str(tmp_path), [(10, 12, [north_of(TLAT, TLON, -100), mid, j]), (11, 7, [east_of(TLAT, TLON, -100), j,
                                                                                         east_of(TLAT, TLON, 100)])])
  geo = RoadGeometry(root=str(tmp_path))
  untagged, all_way = tagged(1, *j), tagged(2, *j, stop="all")
  fwd, bwd = tagged(3, *mid, direction="forward"), tagged(4, *mid, direction="backward")
  untagged, all_way, fwd, bwd = resolve_approaches([untagged, all_way, fwd, bwd], geo)
  # from the major road, heading east toward the junction: the minor road's sign is not ours, the all-way one is
  start = east_of(TLAT, TLON, -80)
  assert find_stop_ahead(*start, 90.0, [untagged]) is None
  assert find_stop_ahead(*start, 90.0, [untagged, all_way]).node_id == 2
  # on the minor road heading north: it is
  south = north_of(TLAT, TLON, -80)
  assert find_stop_ahead(*south, 0.0, [untagged]).node_id == 1
  # forward / backward along the minor way (drawn south -> north)
  assert find_stop_ahead(*south, 0.0, [fwd]).node_id == 3
  assert find_stop_ahead(*south, 0.0, [bwd]) is None
  north = north_of(TLAT, TLON, 20)
  assert find_stop_ahead(*north, 180.0, [bwd]).node_id == 4


def test_opposing_approach_node_skipped_after_the_junction(tmp_path):
  # a north-south road through a junction at j, drawn south -> north and split at j, with each approach's stop node
  # 8 m before the junction: `near` holds northbound traffic, `beyond` (8 m past j) holds southbound traffic
  j = (TLAT, TLON)
  near, beyond = north_of(TLAT, TLON, -8), north_of(TLAT, TLON, 8)
  half = north_of(TLAT, TLON, 150)   # a node in the middle of the northern block
  write_tile(str(tmp_path), [(10, 12, [north_of(TLAT, TLON, -300), near, j]),
                             (11, 12, [j, beyond, half, north_of(TLAT, TLON, 300)]),
                             (12, 12, [east_of(TLAT, TLON, -100), j, east_of(TLAT, TLON, 100)])])
  geo = RoadGeometry(root=str(tmp_path))
  south = north_of(TLAT, TLON, -80)
  for tags in ({}, {"stop": "all", "direction": "forward"}, {"stop": "all", "direction": "backward"}, {"stop": "all"}):
    a, b, c = resolve_approaches([tagged(1, *near, **tags), tagged(2, *beyond, **tags), tagged(3, *half, **tags)], geo)
    if tags == {"stop": "all"}:
      # an all-way stop with no direction holds every approach, as before
      assert a.approaches is None and b.approaches is None
      assert find_stop_ahead(*south, 0.0, [b]).node_id == 2
    elif tags.get("direction") == "forward":
      # both nodes tagged forward: both hold northbound traffic, stop=all or not
      assert find_stop_ahead(*south, 0.0, [b]).node_id == 2
    elif tags.get("direction") == "backward":
      assert find_stop_ahead(*south, 0.0, [a, b]) is None                     # stop=all, yet not for forward traffic
      assert find_stop_ahead(*north_of(TLAT, TLON, 80), 180.0, [a, b]).node_id == 2
    else:
      # untagged: the nearest-junction rule. Northbound stops at `near` and not again at `beyond`
      assert angle_diff(a.approaches[0], 0.0) < 0.5 and angle_diff(b.approaches[0], 180.0) < 0.5
      assert find_stop_ahead(*south, 0.0, [a, b]).node_id == 1
      assert find_stop_ahead(*south, 0.0, [b]) is None
      assert find_stop_ahead(*north_of(TLAT, TLON, 80), 180.0, [a, b]).node_id == 2
      assert c.approaches is None                                             # mid-block: can't tell, kept
      assert find_stop_ahead(*north_of(TLAT, TLON, 100), 0.0, [c]).node_id == 3


def test_stop_sign_map_resolves_in_the_background(tmp_path):
  import json
  import time
  from openpilot.sunnypilot.webhud.maptiles import FEAT_DEG, MapFeatures
  j = (TLAT, TLON)
  write_tile(str(tmp_path / "roads"), [(10, 12, [north_of(TLAT, TLON, -100), j]),
                                       (11, 7, [east_of(TLAT, TLON, -100), j, east_of(TLAT, TLON, 100)])])
  feats = tmp_path / "features"
  feats.mkdir()
  klat, klon = math.floor(TLAT / FEAT_DEG), math.floor(TLON / FEAT_DEG)
  (feats / f"{klat}_{klon}.json").write_text(json.dumps({"nodes": [[1, j[0], j[1], {"highway": "stop"}]]}))
  m = StopSignMap(MapFeatures(root=str(feats), download=False), RoadGeometry(root=str(tmp_path / "roads")))
  deadline = time.monotonic() + 10.0
  signs = []
  while time.monotonic() < deadline:
    signs = m.signs_near(*j)
    if signs and signs[0].approaches is not None:
      break
    time.sleep(0.01)
  assert len(signs) == 1 and signs[0].approaches is not None and angle_diff(signs[0].approaches[0], 0.0) < 0.5
  assert find_stop_ahead(*east_of(TLAT, TLON, -80), 90.0, signs) is None


# ---- unittest collection ----
# tools/test_runner.py is unittest based and only collects TestCase methods: expose every test_* function above as
# one, handing a fresh temporary directory to those that take tmp_path (as pytest would).


def test_confirmation_too_late_never_brakes():
  # route 000000e5: a cross-street sign confirmed at 2.6 m and 24 mph. Too late for a comfortable stop: coast, don't brake
  from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_controller import braking_distance
  v = 10.7   # 24 mph
  ctrl = make([StopSign(1, *north_of(LAT0, LON0, STOP_MARGIN_M + 3.0))])
  ctrl.node_id = 1
  ctrl.state = StopSignState.coasting
  ctrl.confirm_filter.x = 0.99          # the next frame latches
  ctrl.update(sm_at(gps(LAT0, LON0, speed=v), model(stop=True)), True, False, v, 0.0, NOW)
  assert ctrl.gave_up == 1 and 1 in ctrl.done_ids and 3.0 < braking_distance(v)
  assert ctrl.state == StopSignState.idle and ctrl.output_v_target > 200   # left to the driver: no effect on speed
  for _ in range(10):
    ctrl.update(sm_at(gps(LAT0, LON0, speed=v), model(stop=True)), True, False, v, 0.0, NOW)
    assert ctrl.state == StopSignState.idle


def test_confirmation_in_time_still_brakes():
  v = 8.0
  ctrl = make([StopSign(1, *north_of(LAT0, LON0, STOP_MARGIN_M + 60.0))])
  ctrl.node_id = 1
  ctrl.state = StopSignState.coasting
  ctrl.confirm_filter.x = 0.99
  ctrl.update(sm_at(gps(LAT0, LON0, speed=v), model(stop=True)), True, False, v, 0.0, NOW)
  assert ctrl.confirmed and not ctrl.too_late


def test_prefetch_road_tiles_match_the_road_tile_grid():
  from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.prefetch import road_tiles_for, tiles_around
  from openpilot.sunnypilot.webhud.maptiles import tile_name
  tiles = tiles_around(37.5167, -121.9579, 2.0)
  roads = road_tiles_for(tiles)
  assert tile_name(37.5167, -121.9579) in roads          # the tile the car is on
  assert len(roads) == len(tiles)                        # one road tile per 0.25 deg feature tile



def test_unconfirmed_coast_never_stops_the_car():
  # found with drive_sim's closed loop: an unconfirmed coast decayed to 0 and held the car short of the sign
  from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_controller import COAST_FLOOR_V
  ctrl = make([StopSign(1, *north_of(LAT0, LON0, STOP_MARGIN_M + 30.0))])
  v = 6.0
  for _ in range(400):   # 20 s of following the coast target, model never confirming
    ctrl.update(sm_at(gps(LAT0, LON0, speed=v), model()), True, False, v, 0.0, NOW)
    if ctrl.output_v_target < 200:
      v = max(0.0, v + max(min(ctrl.output_v_target - v, 0.0), -1.2) * 0.05)
  assert not ctrl.confirmed
  assert v >= COAST_FLOOR_V - 0.05


def test_coast_below_the_floor_holds_speed():
  from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_controller import coast_target
  assert coast_target(1.5) == 1.5
  assert coast_target(10.0) == 10.0 - 0.4


def test_fast_approach_is_left_to_the_driver():
  from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_controller import MAX_APPROACH_V, coast_distance
  v = MAX_APPROACH_V + 1.0
  near = STOP_MARGIN_M + coast_distance(MAX_APPROACH_V - 1.0) - 5.0   # within coasting range at both speeds
  ctrl = make([StopSign(1, *north_of(LAT0, LON0, near))], max_approach_v=MAX_APPROACH_V)
  ctrl.update(sm_at(gps(LAT0, LON0, speed=v), model(stop=True)), True, False, v, 0.0, NOW)
  assert ctrl.state == StopSignState.idle and ctrl.output_v_target > 200
  v = MAX_APPROACH_V - 1.0                 # slowed down: now it takes the sign on
  ctrl.update(sm_at(gps(LAT0, LON0, speed=v), model(stop=True)), True, False, v, 0.0, NOW)
  assert ctrl.is_active


def test_sign_taken_on_too_close_is_never_braked_for():
  # closed loop on route e5: confirmed early while above the approach cap, taken on only once slowed under it, then
  # braked from too close and overshot by 7.6 m
  from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_controller import (A_TAKE_ON_MAX, MAX_APPROACH_V,
                                                                                         braking_distance)
  v_fast, v_slow = MAX_APPROACH_V + 2.0, MAX_APPROACH_V - 0.5
  d = braking_distance(v_slow, A_TAKE_ON_MAX) - 4.0     # no room for a stop at v_slow, even braking firmer
  ctrl = make([StopSign(1, *north_of(LAT0, LON0, STOP_MARGIN_M + d))], max_approach_v=MAX_APPROACH_V)
  ctrl.node_id = 1
  ctrl.confirmed = True                  # the model confirmed while the car was still too fast to take it on
  ctrl.update(sm_at(gps(LAT0, LON0, speed=v_fast), model(stop=True)), True, False, v_fast, 0.0, NOW)
  assert not ctrl.is_active
  for _ in range(5):
    ctrl.update(sm_at(gps(LAT0, LON0, speed=v_slow), model(stop=True)), True, False, v_slow, 0.0, NOW)
    assert ctrl.state == StopSignState.idle and ctrl.output_v_target > 200
  assert ctrl.gave_up == 1 and 1 in ctrl.done_ids


def _closed_loop(start_m, v, ctrl, steps=900):
  """Drive north toward a sign at the origin + start_m, following the assist's speed target the way the planner does
  (at most -1.2 m/s^2, a 1 s lag). Returns (final speed, distance left to the stop point, peak decel)."""
  x, a, peak = 0.0, 0.0, 0.0
  for _ in range(steps):
    ctrl.update(sm_at(gps(*north_of(LAT0, LON0, x), speed=max(v, 0.0)), model(stop=True)), True, False, v, a, NOW)
    want = 0.0 if ctrl.output_v_target > 200 else max(min(ctrl.output_v_target - v, 0.0), -1.2)
    a += (want - a) * DT_MDL / 1.0
    v = max(0.0, v + a * DT_MDL)
    x += v * DT_MDL
    peak = min(peak, a)
  return v, start_m - STOP_MARGIN_M - x, peak


def test_profile_without_coast_still_stops_from_20_mph():
  # route 00000103 (2026-11-16): profile coast 0 s, stop braking 0.94 m/s^2, 20 mph. The sign was left to the driver at
  # 51 m ("taken on too close") and the car drove through it.
  from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_controller import MAX_APPROACH_V
  ctrl = make([StopSign(1, *north_of(LAT0, LON0, 150.0))], max_approach_v=MAX_APPROACH_V)
  ctrl.set_profile(0.94, 0.0)
  v, left, peak = _closed_loop(150.0, 8.97, ctrl)
  assert ctrl.gave_up == 0
  assert v < 0.3 and -1.0 < left < 4.0, (v, left)
  assert peak > -1.25


def test_taken_on_a_little_late_brakes_firmer_and_stops():
  # in range only once already inside the comfortable braking distance (e.g. slowed under the approach cap late)
  from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_controller import A_TAKE_ON_MAX, braking_distance
  v = 9.0
  d = (braking_distance(v) + braking_distance(v, A_TAKE_ON_MAX)) / 2
  ctrl = make([StopSign(1, *north_of(LAT0, LON0, STOP_MARGIN_M + d))])
  ctrl.node_id = 1
  ctrl.confirmed = True                  # the model confirmed while the car was still too fast to take it on
  v, left, _ = _closed_loop(STOP_MARGIN_M + d, v, ctrl)
  assert ctrl.gave_up == 0 and v < 0.3 and left > -1.5, (v, left)


def test_crawling_past_a_sign_releases_it():
  # closed loop on route e5: crawling under the GPS speed threshold, a sign behind the car was kept for 1.8 km
  ctrl = make([StopSign(1, *north_of(LAT0, LON0, 30))])
  ctrl.node_id, ctrl.state, ctrl.distance = 1, StopSignState.coasting, -STOP_MARGIN_M
  for _ in range(400):   # 20 s at 1.3 m/s, GPS speed too low for its bearing
    ctrl.update(sm_at(gps(LAT0, LON0, speed=1.3), model()), True, False, 1.3, 0.0, NOW)
  assert ctrl.node_id is None and 1 in ctrl.done_ids and ctrl.output_v_target > 200

def test_local_stop_signs_parse_and_skip_bad_entries(tmp_path):
  import json
  p = tmp_path / "local_stop_signs.json"
  assert local_stop_signs(str(p)) == []
  p.write_text(json.dumps({"signs": [
    {"id": -1, "lat": LAT0, "lon": LON0, "all_way": True},
    {"id": -2, "lat": LAT0, "lon": LON0, "approaches": [450.0]},
    {"id": 5, "lat": LAT0, "lon": LON0, "all_way": True},      # positive id: could clash with OSM
    {"id": -3, "lat": LAT0, "lon": LON0},                       # holds nothing
    {"id": -4, "lon": LON0, "all_way": True},                   # no lat
  ]}))
  signs = local_stop_signs(str(p))
  assert [s.node_id for s in signs] == [-1, -2]
  assert signs[0].all_way and signs[0].approaches is None and not signs[0].needs_geometry
  assert signs[1].approaches == (90.0,)
  assert sign_applies(signs[0], 123.0) == Applies.YES


def test_local_stop_sign_merges_into_its_cell(tmp_path):
  import json
  from openpilot.sunnypilot.webhud.maptiles import FEAT_DEG
  features = tmp_path / "features"
  features.mkdir()
  key = (math.floor(LAT0 / FEAT_DEG), math.floor(LON0 / FEAT_DEG))
  cell = features / "cell.json"
  cell.write_text(json.dumps({"nodes": [[7, LAT0, LON0, {"highway": "stop", "direction": "both"}]]}))
  far = LAT0 + 3 * FEAT_DEG
  (tmp_path / "local_stop_signs.json").write_text(json.dumps({"signs": [
    {"id": -1, "lat": north_of(LAT0, LON0, 100)[0], "lon": LON0, "all_way": True},
    {"id": -2, "lat": far, "lon": LON0, "all_way": True},       # another cell
  ]}))
  feats = SimpleNamespace(root=str(features), cell_path=lambda klat, klon: str(cell) if (klat, klon) == key else None)
  smap = StopSignMap(features=feats, geometry=SimpleNamespace(ways_at=lambda lat, lon: None))
  smap._load(key)
  assert sorted(s.node_id for s in smap._cells[key]) == [-1, 7]
  ahead = find_stop_ahead(*north_of(LAT0, LON0, 60), 0.0, smap._cells[key])
  assert ahead is not None and ahead.node_id == -1


def _as_method(fn):
  import inspect
  import pathlib
  import tempfile

  def method(self):
    if "tmp_path" in inspect.signature(fn).parameters:
      with tempfile.TemporaryDirectory() as d:
        fn(tmp_path=pathlib.Path(d))
    else:
      fn()
  method.__name__ = fn.__name__
  method.__doc__ = fn.__doc__
  return method


def _test_case():
  import unittest
  funcs = {k: v for k, v in globals().items() if k.startswith("test_") and callable(v)}
  return type("TestStopSign", (unittest.TestCase,), {k: _as_method(v) for k, v in funcs.items()})


TestStopSign = _test_case()



def _fisker_approach(v0, stop_d, confirm_d, k=1.0, delay=0.35, tau=0.3):
  """A stop with the Fisker's request smoothing (carcontroller.smooth_accel_step, 100 Hz) between a planner stand-in
  (a = clip(k (target - v), -1.2, +1.0), 20 Hz) and a car following the request after `delay` with time constant
  `tau`; the model confirms the stop `confirm_d` before the stop point. Returns m short of the stop point (- = past)."""
  from opendbc.car.fisker.carcontroller import smooth_accel_step, soft_stop_accel
  sign_m = stop_d + STOP_MARGIN_M
  ctrl = make([StopSign(1, *north_of(LAT0, LON0, sign_m))])
  pos, v, a, sent, step, filt, cmd = 0.0, v0, 0.0, 0.0, 0.0, 0.0, 0.0
  q = [0.0] * round(delay / 0.01)
  for n in range(40000):
    if n % 5 == 0:
      lat, lon = north_of(LAT0, LON0, pos)
      ctrl.update(sm_at(gps(lat, lon, speed=v), model(stop=sign_m - pos < confirm_d + STOP_MARGIN_M)), True, False, v, a, NOW)
      tgt = ctrl.output_v_target
      cmd = 0.0 if tgt == V_CRUISE_UNSET else max(min(k * (tgt - v), 1.0), -1.2)
    sent, step, filt = smooth_accel_step(soft_stop_accel(cmd, v, v == 0.0), sent, step, filt)
    q.append(sent)
    a += (q.pop(0) - a) * 0.01 / tau
    v = max(0.0, v + a * 0.01)
    pos += v * 0.01
    if (v == 0.0 and n > 50) or pos > sign_m + 5:
      break
  return stop_d - pos


def test_stops_short_of_the_stop_point_with_the_cars_lag():
  # 00000136 at 0:30: 10.5 mph, confirmed 23.7 m out; braking began 5.9 m before the stop point and the car crossed it
  # at 3-4 mph. Every approach must stop clear of the point on a car that brakes late.
  for mph, stop_d, confirm_d in ((10.5, 30.7, 23.7), (8.0, 30.0, 25.0), (13.0, 40.0, 35.0), (16.0, 50.0, 45.0), (22.0, 100.0, 100.0),
                                 (25.0, 120.0, 120.0)):
    for k, delay in ((1.5, 0.2), (1.0, 0.35), (0.8, 0.5)):
      short = _fisker_approach(mph * 0.44704, stop_d, confirm_d, k, delay)
      assert 0.8 <= short <= 4.0, (mph, k, delay, short)


def test_confirmed_stop_coasts_then_holds_not_down_to_the_floor():
  from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_controller import coast_target, COAST_FLOOR_V
  assert coast_target(2.6, floor=False) == 2.6 - 0.4     # confirmed: no floor
  assert coast_target(2.6) == COAST_FLOOR_V              # unconfirmed: the floor stays (never stops for a sign the model ignores)


def test_the_stop_ends_softly():
  # the driver's stops end at about 0.7 m/s^2 in the last second; openpilot's ended at 2.0-2.1 (a head jolt)
  from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_controller import SOFT_A, SOFT_V, braking_v_target, braking_distance
  assert SOFT_A <= 0.7
  d_soft = SOFT_V ** 2 / (2 * SOFT_A)
  # on the profile below SOFT_V the deceleration is SOFT_A: v^2 = 2 a d
  from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_controller import STOP_BUFFER_M
  for d in (0.5, 1.5, 3.0, d_soft):
    v = braking_v_target(d + STOP_BUFFER_M, 0.0)
    assert abs(v * v / (2 * d) - SOFT_A) < 1e-6, (d, v)
  # and above it the comfort rate, continuous at the knee
  assert abs(braking_v_target(d_soft + STOP_BUFFER_M, 0.0) - SOFT_V) < 1e-6
  assert braking_v_target(d_soft + 10.0 + STOP_BUFFER_M, 0.0) > SOFT_V
  assert braking_distance(SOFT_V) < braking_distance(SOFT_V + 1.0)
