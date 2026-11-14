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
  A_COMFORT, COAST_START_M, STOP_MARGIN_M, StopSignAssist, StopSignState,
)
from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign import road_geometry
from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.road_geometry import RoadGeometry, WayAtNode
from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_direction import (
  Applies, angle_diff, approaches_from_ways, parse_direction, sign_applies,
)
from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_map import (
  EARTH_R, StopSign, StopSignMap, find_stop_ahead, resolve_approaches, stop_signs_from_cell,
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
  for bad in (None, "", "both", "all", "45-135", "nan", 90):
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
def test_numeric_direction():
  north_bound = tagged(1, *north_of(LAT0, LON0, 80), direction="0")
  assert sign_applies(north_bound, 0.0) == Applies.YES
  assert sign_applies(north_bound, 350.0) == Applies.YES        # wraps around north
  assert sign_applies(north_bound, 90.0) == Applies.NO          # crossing traffic
  assert sign_applies(north_bound, 180.0) == Applies.NO         # oncoming
  assert sign_applies(north_bound, 70.0) == Applies.UNKNOWN     # in between: no decision
  # find_stop_ahead skips it for a car the sign isn't for, keeps it for one it is
  assert find_stop_ahead(LAT0, LON0, 0.0, [north_bound]).node_id == 1
  south_bound = tagged(2, *north_of(LAT0, LON0, 80), direction="180")
  assert find_stop_ahead(LAT0, LON0, 0.0, [south_bound]) is None


@trusting_compass
def test_cardinal_direction():
  sign = tagged(1, *east_of(LAT0, LON0, 60), direction="E")
  assert sign.approaches == (90.0,)
  assert find_stop_ahead(LAT0, LON0, 90.0, [sign]).node_id == 1
  west = tagged(2, *east_of(LAT0, LON0, 60), direction="W")
  assert find_stop_ahead(LAT0, LON0, 90.0, [west]) is None
  assert sign_applies(tagged(3, LAT0, LON0, direction="NE;SW"), 225.0) == Applies.YES


def test_all_way_stop_always_applies():
  sign = tagged(1, *north_of(LAT0, LON0, 80), direction="180", stop="all")
  assert sign.all_way and sign_applies(sign, 0.0) == Applies.YES
  assert find_stop_ahead(LAT0, LON0, 0.0, [sign]).node_id == 1


def test_unknown_direction_keeps_the_sign():
  for tags in ({}, {"direction": "both"}, {"direction": "forward"}, {"stop": "minor"}):
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
