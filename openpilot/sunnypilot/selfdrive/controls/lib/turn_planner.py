"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Map-based junction turn planner: steers the car through a signaled turn at a junction from mapd's offline road tiles.

With a blinker on at low speed the driving model gets a turn desire but does not start the turn itself (16 recorded
turns: it only follows once the driver steers). This planner finds the junction ahead on the current road where a way
leaves toward the blinker side, plans a circular arc through it and commands that arc's curvature until the car has
rotated the planned heading change, then hands lateral control back to the model (update() returns None).

Sign conventions (established from the recorded turns, 000000e5--0507d93e38 and 000000dd--ad372d473d):
  * bearing (GPS / map): degrees clockwise from north, so a RIGHT turn increases it.
  * carState.yawRate: positive = counter-clockwise (LEFT); every recorded right turn had yawRate < 0.
  * carState.steeringAngleDeg: positive = LEFT; right turns showed -140..-280 deg.
  * curvature (modelV2.action.desiredCurvature, controlsState.curvature/desiredCurvature, what controlsd clips and
    sends): positive = RIGHT. controlsd computes the measured curvature as -VM.calc_curvature(steeringAngle), and on
    every recorded right turn both it and the model's desiredCurvature were positive (e.g. e5 t~410 s: angle -226 deg,
    controlsState.curvature +0.086, yawRate -0.29). So this module returns +1/R for a right turn and -1/R for a left.

The car only steers through the turn while lateral control is active; the driver can override at any time (the car
port yields to torque), and a sustained press against the turn releases the planner for the rest of that blinker.

Map data: only road tiles already on disk are read (the same files road_geometry.RoadGeometry reads), on a background
thread: update() never blocks. RoadSource(sync=True) reads inline, for the simulator and tests.
"""
import math
import os
import threading
import time
from dataclasses import dataclass, field

import numpy as np

EARTH_R = 6371000.0

# ---- arming ----
V_ARM_MAX = 7.2                 # m/s (16 mph): the planner only starts a turn at or below this speed
V_ABORT = 9.0                   # m/s (20 mph): release above this speed at any point
JUNCTION_LOOKAHEAD_M = 35.0     # junctions this far ahead along the current road are considered
TURN_MIN_DEG = 45.0             # a branch is a turn on the blinker side when it leaves this much...
TURN_MAX_DEG = 135.0            # ...and at most this much (beyond is a U-turn or a road doubling back)
STRAIGHT_MAX_DEG = 30.0         # where the current way ends, the walk ahead follows a way continuing within this
GPS_MAX_ACCURACY_M = 10.0       # fixes worse than this are ignored
GPS_BEARING_MIN_SPEED = 2.0     # m/s: below this the GPS course is noise; the heading is carried on by the yaw rate

# ---- map matching ----
MATCH_MAX_LATERAL_M = 12.0      # the car's fix this far from a way's centreline can still be on it (GPS + lane offset)
MATCH_MAX_BEARING_DEG = 40.0    # ...and the way's direction this close to the car's heading
MATCH_BEARING_WEIGHT = 0.15     # m of matching cost per degree of bearing difference
MATCH_STICKY_M = 3.0            # cost bonus for the way matched last time (no flipping at junctions)
NO_MATCH_ABORT_S = 1.0          # armed, but no way matched for this long: release
ARRIVAL_LOOK_M = 12.0           # the approach heading is measured over this much road before the junction
BRANCH_LOOK_M = 12.0            # a branch's heading is measured over this much of it after the junction
PASSED_M = 3.0                  # armed, but the junction is this far behind: release (it was missed)

# ---- the turn ----
A_LAT_MAX = 2.5                 # m/s^2: the arc's radius is never tighter than v^2 / A_LAT_MAX
R_MIN = 6.0                     # m: tightest arc ever planned (a late start may tighten the arc down to this)
R_FROM_STOP_MAX = 30.0          # m: widest arc a turn started early from a stop (TurnTuning.stop_start_*_m) is fitted to
LATE_SLACK_M = 2.0              # a start so late the arc would need to be this much tighter than R_MIN is abandoned;
                                # within the slack the R_MIN arc is flown and ends a little past the planned exit
V_START_MIN = 0.5               # m/s: armed and stopped (at the stop line), the turn waits for the car to move off
CURV_RATE_MIN = 0.03            # 1/m/s: floor for the expected curvature build-up rate (lead time estimate)
STALL_S = 2.0                   # turning, but the heading hasn't progressed STALL_DEG in this long while moving: release
STALL_DEG = 3.0
STALL_MIN_V = 0.5               # m/s: below this the car is waiting (pedestrian, cross traffic), not stalling
OVERRIDE_S = 0.3                # driver torque against the turn for this long: release
BLINKER_OFF_S = 1.0             # turning, the blinker off this long (longer than a flash gap) once the turn is...
BLINKER_OFF_DONE = 0.5          # ...this fraction done: the blinker cancelled itself, the turn is over
TURN_TIMEOUT_S = 20.0           # a turn that takes longer than this is released whatever else happens
DRIVER_TURNED_DEG = TURN_MIN_DEG  # the car has turned this much toward the blinker side since that blinker came on,
                                  # not steered by the planner: the driver made the turn; no planner turn on that
                                  # blinker. The blinker stays on for a moment after a hand-made turn, and the lagging
                                  # GPS fix still puts the car short of the junction node, so the road the car came
                                  # from looked like a branch on the blinker side a few metres ahead: the planner
                                  # armed and steered hard back toward it (left onto Doane St from Gatewood St,
                                  # 2026-11-14).

# ---- car (Fisker Ocean): how fast the commanded curvature can actually build up ----
ACTUATOR_DELAY_S = 0.15         # CarParams.steerActuatorDelay
STEER_RATIO_X_WHEELBASE = 42.0  # rad of wheel per 1/m of curvature (fit of steeringAngle vs yawRate/v, 2-8 m/s turns)
ANGLE_RATE_BP = (0.0, 5.0, 25.0)        # m/s ...
ANGLE_RATE_DEG_S = (250.0, 150.0, 20.0)       # ... wheel-angle rate limit winding up (ANGLE_RATE_LIMIT_UP x 100 Hz)
ANGLE_RATE_DOWN_DEG_S = (500.0, 200.0, 30.0)  # ... and unwinding (ANGLE_RATE_LIMIT_DOWN x 100 Hz)
MAX_LATERAL_JERK = 5.0          # m/s^3 (drive_helpers.clip_curvature)
MAX_CURVATURE = 0.2             # 1/m (drive_helpers)

LOG_TICK_S = 0.5                # armed or turning: a "map_turn_tick" log event this often (state changes log at once)

LOCAL_RADIUS_M = 250.0          # road ways within this of the load point are kept in local coordinates
LOAD_RETRY_S = 2.0              # s: wait this long after a road load that found nothing
RELOAD_M = 120.0                # reload once the car is this far from the load point


@dataclass
class TurnTuning:
  """The constants tuned against the 13 recorded signaled turns of 000000e5--0507d93e38 and 000000dd--ad372d473d
  (drive_sim evaluate_turns: ghost path vs recorded path). Start values were right 9 m / left 13 m, lead +0.3 s,
  release 2 deg; the grid search (radius 6..18 m per side, lead -1.0..+0.5 s, taper 0..25 deg, release -6..+3 deg,
  lane offset 0..3 m) settled on the values below; the score was flat within ~5% around them."""
  radius_right_m: float = 10.0  # arc radius of a right turn (8..12 m scored within 5%; recorded rights were 9-12 m)
  radius_left_m: float = 10.0   # arc radius of a left turn (13 m swung ~3 m inside the recorded lefts' start; 9-11 best)
  lead_s: float = -0.45         # correction to the start lead beyond the actuator delay and half the curvature build-up
                                # time; negative: the recorded drivers start later than that model alone says
  taper_deg: float = 15.0       # the commanded curvature ramps down linearly over the last taper_deg of the turn...
  taper_min: float = 0.3        # ...to this fraction of the arc's curvature
  release_deg: float = 0.0      # release this short of the planned heading change, on top of what the car turns while
                                # the wheel unwinds (unwind_heading_deg at the measured curvature)
  lane_offset_m: float = 0.0    # the car drives this far right of the centreline: corners cut on rights, widen on lefts
                                # (0 scored best: GPS and OSM centreline offsets are as large as the lane offset)
  stop_start_left_m: float = 0.0   # armed and stopped (at the stop line) this close to the junction or closer: start
  stop_start_right_m: float = 0.0  # the turn as the car moves off, the arc fitted to the distance left; 0 = off. From
                                   # the driver profile (turn_stop_start_*_m): from a stop the driver starts a left turn
                                   # ~25 m before the junction node in a ~20 m arc, the planner's own start is ~10 m
                                   # (4 turns, routes f0 f3 f5 f6; f6 is the Doane St turn the driver led by 2 s)


def _wrap(a: float) -> float:
  return (a + 180.0) % 360.0 - 180.0


def _bearing(dx: float, dy: float) -> float:
  return math.degrees(math.atan2(dx, dy)) % 360.0


def curvature_rate(v: float, unwind: bool = False) -> float:
  """1/m/s: how fast the car's curvature can grow (or, unwinding, shrink) at speed v: the slower of the jerk limit
  and the wheel-angle rate limit."""
  v = max(v, 1.0)
  jerk = MAX_LATERAL_JERK / v ** 2
  rates = ANGLE_RATE_DOWN_DEG_S if unwind else ANGLE_RATE_DEG_S
  wheel = math.radians(float(np.interp(v, ANGLE_RATE_BP, rates))) / STEER_RATIO_X_WHEELBASE
  return max(min(jerk, wheel), CURV_RATE_MIN)


def unwind_heading_deg(v: float, curvature: float) -> float:
  """deg the car still turns after the command drops to zero while it drives at v on |curvature|: through the
  actuator delay, then while the wheel unwinds at its rate limit (the curvature falls linearly)."""
  k = abs(curvature)
  return math.degrees(v * k * (ACTUATOR_DELAY_S + 0.5 * k / curvature_rate(v, unwind=True)))


# ---- local road network ----

@dataclass
class LocalWay:
  way_id: int
  highway: str
  oneway: bool
  xy: np.ndarray            # (n, 2) metres east, north of the LocalRoads origin, in OSM node order
  keys: list                # per node: its position key, shared by every way through that node


@dataclass
class Match:
  way: int                  # index into LocalRoads.ways
  seg: int                  # segment seg -> seg + 1
  t: float                  # 0..1 along the segment
  direction: int            # +1 travelling in node order, -1 against
  lateral: float            # m, signed: positive = the car is right of the centreline
  bearing_err: float        # deg


@dataclass
class Branch:
  way_id: int
  highway: str
  bearing: float            # deg, leaving the junction
  change: float             # deg, signed: positive = right


@dataclass
class Junction:
  x: float
  y: float
  distance: float           # m along the road from the car's projection
  arrival_bearing: float    # deg
  branch: Branch            # the chosen branch on the blinker side


def _key(lat: float, lon: float) -> tuple[int, int]:
  return round(lat * 1e7), round(lon * 1e7)


class LocalRoads:
  """The ways around a point, in local metric coordinates, with their shared nodes indexed."""

  def __init__(self, lat0: float, lon0: float, ways: list[tuple[int, str, bool, list[tuple[float, float]]]]):
    self.lat0, self.lon0 = lat0, lon0
    self._cos = math.cos(math.radians(lat0))
    self.ways: list[LocalWay] = []
    self.nodes: dict[tuple[int, int], list[tuple[int, int]]] = {}
    seg_a, seg_b, seg_w, seg_i = [], [], [], []
    for way_id, highway, oneway, pts in ways:
      if len(pts) < 2:
        continue
      xy = np.array([self.to_xy(la, lo) for la, lo in pts], dtype=np.float64)
      keys = [_key(la, lo) for la, lo in pts]
      wi = len(self.ways)
      self.ways.append(LocalWay(int(way_id), highway, bool(oneway), xy, keys))
      for ni, k in enumerate(keys):
        self.nodes.setdefault(k, []).append((wi, ni))
      seg_a.append(xy[:-1])
      seg_b.append(xy[1:])
      seg_w.append(np.full(len(xy) - 1, wi))
      seg_i.append(np.arange(len(xy) - 1))
    if seg_a:
      self._a, self._b = np.concatenate(seg_a), np.concatenate(seg_b)
      self._w, self._i = np.concatenate(seg_w), np.concatenate(seg_i)
    else:
      self._a = self._b = np.zeros((0, 2))
      self._w = self._i = np.zeros(0, dtype=int)

  def to_xy(self, lat: float, lon: float) -> tuple[float, float]:
    return math.radians(lon - self.lon0) * self._cos * EARTH_R, math.radians(lat - self.lat0) * EARTH_R

  def match(self, x: float, y: float, bearing: float, prefer_way_id: int | None = None) -> Match | None:
    """The way segment the car is on: near its position and running along its heading (either way unless oneway)."""
    if not len(self._a):
      return None
    d = self._b - self._a
    l2 = np.maximum((d ** 2).sum(axis=1), 1e-9)
    p = np.array([x, y])
    t = np.clip(((p - self._a) * d).sum(axis=1) / l2, 0.0, 1.0)
    proj = self._a + d * t[:, None]
    dist = np.hypot(*(p - proj).T)
    seg_bear = np.degrees(np.arctan2(d[:, 0], d[:, 1]))
    err_fwd = np.abs((bearing - seg_bear + 180.0) % 360.0 - 180.0)
    err_bwd = 180.0 - err_fwd
    oneway = np.array([w.oneway for w in self.ways])[self._w]
    err_bwd = np.where(oneway, 999.0, err_bwd)
    fwd = err_fwd <= err_bwd
    err = np.where(fwd, err_fwd, err_bwd)
    cost = dist + MATCH_BEARING_WEIGHT * err
    if prefer_way_id is not None:
      ids = np.array([w.way_id for w in self.ways])[self._w]
      cost = cost - np.where(ids == prefer_way_id, MATCH_STICKY_M, 0.0)
    ok = (dist <= MATCH_MAX_LATERAL_M) & (err <= MATCH_MAX_BEARING_DEG)
    if not ok.any():
      return None
    k = int(np.flatnonzero(ok)[np.argmin(cost[ok])])
    direction = 1 if fwd[k] else -1
    # signed lateral: cross product of the travel direction with the offset (positive = car right of centreline)
    ux, uy = d[k] * direction / math.sqrt(l2[k])
    ox, oy = p - proj[k]
    lateral = ox * uy - oy * ux
    return Match(int(self._w[k]), int(self._i[k]), float(t[k]), direction, float(lateral), float(err[k]))

  # -- walking the road ahead --

  def _point_along(self, wi: int, ni: int, direction: int, look: float) -> tuple[float, float] | None:
    """The point look metres from node ni along way wi in direction (stops at the way's end)."""
    xy = self.ways[wi].xy
    acc = 0.0
    i = ni
    while 0 <= i + direction < len(xy):
      a, b = xy[i], xy[i + direction]
      seg = float(np.hypot(*(b - a)))
      if acc + seg >= look:
        f = (look - acc) / max(seg, 1e-9)
        return float(a[0] + f * (b[0] - a[0])), float(a[1] + f * (b[1] - a[1]))
      acc += seg
      i += direction
    return None if i == ni else (float(xy[i][0]), float(xy[i][1]))

  def branches(self, key, x: float, y: float) -> list[tuple[int, int, int, float]]:
    """Every way leaving a node: (way index, node index, direction, bearing)."""
    out = []
    for wj, nj in self.nodes.get(key, []):
      w = self.ways[wj]
      for direction in (1, -1):
        if direction == -1 and w.oneway:
          continue   # can't drive a oneway against its node order
        if not 0 <= nj + direction < len(w.xy):
          continue
        pt = self._point_along(wj, nj, direction, BRANCH_LOOK_M)
        if pt is None:
          continue
        out.append((wj, nj, direction, _bearing(pt[0] - x, pt[1] - y)))
    return out

  def junction_ahead(self, m: Match, x: float, y: float, side: str, lookahead: float = JUNCTION_LOOKAHEAD_M) -> Junction | None:
    """The nearest junction ahead along the matched road with a branch leaving toward side ('left'/'right') by
    TURN_MIN_DEG..TURN_MAX_DEG. The walk follows the road through junctions where it continues straight."""
    wi, direction = m.way, m.direction
    w = self.ways[wi]
    a, b = w.xy[m.seg], w.xy[m.seg + 1]
    proj = a + (b - a) * m.t
    ni = m.seg + 1 if direction == 1 else m.seg
    trail = [proj]   # the road behind the current node (for the arrival heading), newest last
    # some road behind the car too, so a junction right ahead still has an arrival heading
    back = self._point_along(wi, ni, -direction, ARRIVAL_LOOK_M + float(np.hypot(*(w.xy[ni] - proj))))
    if back is not None:
      trail.insert(0, np.array(back))
    dist = float(np.hypot(*(w.xy[ni] - proj)))
    sign = 1.0 if side == "right" else -1.0
    visited = set()
    while dist <= lookahead:
      w = self.ways[wi]
      node = w.xy[ni]
      key = w.keys[ni]
      if (wi, ni) in visited:
        break
      visited.add((wi, ni))
      trail.append(node)
      arrival = self._arrival_bearing(trail)
      best, cont = None, None
      for wj, nj, dj, bear in self.branches(key, float(node[0]), float(node[1])):
        change = _wrap(bear - arrival)
        if TURN_MIN_DEG <= sign * change <= TURN_MAX_DEG:
          if best is None or abs(abs(change) - 90.0) < abs(abs(best[4]) - 90.0):
            best = (wj, nj, dj, bear, change)
        if abs(change) <= STRAIGHT_MAX_DEG and (cont is None or abs(change) < abs(cont[3])):
          if not (wj == wi and dj == direction):
            cont = (wj, nj, dj, change)
      if best is not None:
        bw = self.ways[best[0]]
        return Junction(float(node[0]), float(node[1]), dist, arrival, Branch(bw.way_id, bw.highway, best[3], best[4]))
      # carry on along the road: the same way if it continues, else the straightest continuation
      if 0 <= ni + direction < len(w.xy):
        nxt = ni + direction
      elif cont is not None:
        wi, ni, direction = cont[0], cont[1], cont[2]
        w = self.ways[wi]
        nxt = ni + direction
      else:
        break
      dist += float(np.hypot(*(w.xy[nxt] - w.xy[ni])))
      ni = nxt
    return None

  @staticmethod
  def _arrival_bearing(trail) -> float:
    end = trail[-1]
    acc = 0.0
    start = trail[0]
    for i in range(len(trail) - 1, 0, -1):
      seg = float(np.hypot(*(trail[i] - trail[i - 1])))
      if acc + seg >= ARRIVAL_LOOK_M:
        f = (ARRIVAL_LOOK_M - acc) / max(seg, 1e-9)
        start = trail[i] + (trail[i - 1] - trail[i]) * f
        break
      acc += seg
    return _bearing(float(end[0] - start[0]), float(end[1] - start[1]))


# ---- road tiles ----

def read_local_roads(root: str, lat: float, lon: float, radius: float = LOCAL_RADIUS_M) -> LocalRoads | None:
  """The ways within radius of (lat, lon) from the road tiles on disk; None when no tile there is on disk. Blocking."""
  import capnp
  from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.road_geometry import HIGHWAY_CLASSES, SCHEMA_PATH, TRAVERSAL_LIMIT_WORDS
  from openpilot.sunnypilot.webhud.maptiles import tile_name

  dlat = radius / EARTH_R * 180.0 / math.pi
  dlon = dlat / max(math.cos(math.radians(lat)), 0.01)
  paths = set()
  for la in (lat - dlat, lat + dlat):
    for lo in (lon - dlon, lon + dlon):
      c_lat, c_lon, name = tile_name(la, lo)
      paths.add(os.path.join(root, str(c_lat), str(c_lon), name))
  schema = capnp.load(SCHEMA_PATH)
  ways, seen, found = [], set(), False
  for path in sorted(paths):
    if not os.path.isfile(path):
      continue
    found = True
    with open(path, "rb") as f:
      tile = schema.Offline.from_bytes_packed(f.read(), traversal_limit_in_words=TRAVERSAL_LIMIT_WORDS)
    for w in tile.ways:
      has_bbox = bool(w.minLat or w.maxLat)
      if has_bbox and (w.maxLat < lat - dlat or w.minLat > lat + dlat or w.maxLon < lon - dlon or w.minLon > lon + dlon):
        continue
      if w.id in seen:
        continue
      pts = [(n.latitude, n.longitude) for n in w.nodes]
      if not has_bbox and not any(abs(p[0] - lat) <= dlat and abs(p[1] - lon) <= dlon for p in pts):
        continue
      seen.add(w.id)
      cls = HIGHWAY_CLASSES[w.highway] if w.highway < len(HIGHWAY_CLASSES) else "unknown"
      ways.append((int(w.id), cls, bool(w.oneWay), pts))
  return LocalRoads(lat, lon, ways) if found else None


class RoadSource:
  """LocalRoads around the car, (re)loaded in the map worker process as it moves (a thread with use_worker=False).
  get() never blocks unless sync."""

  def __init__(self, root: str | None = None, sync: bool = False, use_worker: bool = True):
    if root is None:
      from openpilot.common.hardware.hw import Paths
      root = os.path.join(Paths.mapd_root(), "offline")
    self.root = root
    self.sync = sync
    self.roads: LocalRoads | None = None
    self._loading = False
    self._lock = threading.Lock()
    # on the device the tiles are read in the map worker process (map_worker.py), never on a controlsd thread
    self.use_worker = not sync and use_worker
    self._worker = None
    self._retry_at = 0.0    # after a load that found nothing, don't ask again every tick

  def warm(self) -> None:
    """Start the map worker before the first turn needs it."""
    if self.use_worker:
      self._get_worker().start()

  def _get_worker(self):
    if self._worker is None:
      from openpilot.sunnypilot.selfdrive.controls.lib.map_worker import MapWorker
      self._worker = MapWorker.shared()
    return self._worker

  def _loaded(self, roads) -> None:
    with self._lock:
      if roads is not None:
        self.roads = roads
      else:
        self._retry_at = time.monotonic() + LOAD_RETRY_S
      self._loading = False

  def _load(self, lat: float, lon: float) -> None:
    roads = None
    try:
      roads = read_local_roads(self.root, lat, lon)
    except Exception:
      from openpilot.common.swaglog import cloudlog
      cloudlog.exception("turn planner: reading road tiles failed")
    with self._lock:
      if roads is not None:
        self.roads = roads
      self._loading = False

  def get(self, lat: float, lon: float) -> LocalRoads | None:
    with self._lock:
      roads = self.roads
      fresh = roads is not None and math.hypot(*roads.to_xy(lat, lon)) <= RELOAD_M
      if fresh or self._loading or time.monotonic() < self._retry_at:
        return roads if roads is not None and math.hypot(*roads.to_xy(lat, lon)) <= LOCAL_RADIUS_M else None
      self._loading = True
    if self.sync:
      self._load(lat, lon)
      return self.roads
    if self.use_worker:
      self._get_worker().request_roads(self.root, lat, lon, self._loaded)
    else:
      threading.Thread(target=self._load, args=(lat, lon), daemon=True).start()
    return roads if roads is not None and math.hypot(*roads.to_xy(lat, lon)) <= LOCAL_RADIUS_M else None


class StaticRoads:
  """A fixed LocalRoads (tests, hand-made geometry)."""

  def __init__(self, roads: LocalRoads):
    self.roads = roads

  def get(self, lat: float, lon: float) -> LocalRoads | None:
    return self.roads


# ---- the planner ----

IDLE, ARMED, TURNING, DONE = "idle", "armed", "turning", "done"


@dataclass
class TurnLog:
  state: str = IDLE
  side: str = ""
  junction_dist: float = 0.0     # m to the junction (armed)
  start_dist: float = 0.0        # m from the junction where the turn starts
  planned_deg: float = 0.0       # signed heading change planned (positive = right)
  progress_deg: float = 0.0      # heading turned so far (turning)
  radius: float = 0.0
  curvature: float = 0.0         # 1/m commanded (positive = right)
  reason: str = ""               # why the last turn ended
  way_id: int = 0                # the matched way
  branch_way_id: int = 0
  lateral: float = 0.0           # m right of the matched centreline
  extra: dict = field(default_factory=dict)


class TurnPlanner:
  """update() once per control tick; returns the desired curvature (1/m, positive = right) while turning, else None."""

  def __init__(self, road_source=None, tuning: TurnTuning | None = None):
    self.roads_src = road_source if road_source is not None else RoadSource()
    self.tuning = tuning or TurnTuning()
    self.log = TurnLog()
    self._reset()
    self.heading: float | None = None    # deg, the car's heading (GPS course, carried on by the yaw rate)
    self._last_fix = None
    self._done_side: str | None = None   # a turn on this side ended: no new one until that blinker goes off
    self._side_turned = 0.0              # deg turned toward the blinker side since that blinker came on
    self._turn_side = ""                 # the blinker side _side_turned counts for
    self._prefer_way: int | None = None
    self.pos: tuple[float, float] | None = None   # car position in the current roads' frame at the last fix
    # what the planner did goes to the log ("map_turn" on every state change, "map_turn_tick" while armed or turning),
    # so an incident is a log lookup, not a replay. log_event(name, **fields); None: cloudlog.event
    self.log_event = None
    self._logged_state = IDLE
    self._log_t = 0.0

  def _reset(self) -> None:
    self.state = IDLE
    self.side = ""
    self.junction: Junction | None = None
    self.junction_xy = (0.0, 0.0)
    self.roads: LocalRoads | None = None
    self.dist = 0.0
    self.planned = 0.0
    self.progress = 0.0
    self.radius = 0.0
    self.curvature = 0.0
    self.t_turn = 0.0
    self.stall_t = 0.0
    self.stall_ref = 0.0
    self.override_t = 0.0
    self.no_match_t = 0.0
    self.waited_stopped = False   # armed and stopped (at the stop line) at some point
    self.blinker_off_t = 0.0      # s the blinker has been off while turning

  def _end(self, reason: str) -> None:
    if self.state in (ARMED, TURNING):
      self._done_side = self.side
    side = self.side
    self._reset()
    if reason:
      self.log.reason = reason
    self.state = DONE if side and self._done_side == side else IDLE
    self.side = side if self.state == DONE else ""

  def radius_for(self, side: str, v: float) -> float:
    r = self.tuning.radius_right_m if side == "right" else self.tuning.radius_left_m
    return max(r, v ** 2 / A_LAT_MAX, R_MIN)

  def start_distance(self, radius: float, change_deg: float, v: float) -> float:
    """Distance before the junction node at which to start turning: the arc's tangent length, shifted by the lane
    offset (the car's lane corner sits before the centreline junction on a right, past it on a left), plus the
    distance covered while the actuator delay passes and the curvature builds up."""
    half = math.radians(abs(change_deg)) / 2.0
    corner = self.tuning.lane_offset_m * math.tan(math.radians(change_deg) / 2.0)
    build = (1.0 / radius) / curvature_rate(v)
    return radius * math.tan(half) + corner + v * (ACTUATOR_DELAY_S + 0.5 * build + self.tuning.lead_s)

  def _new_fix(self, gps) -> bool:
    if gps is None or not getattr(gps, "hasFix", False):
      return False
    if getattr(gps, "horizontalAccuracy", 0.0) > GPS_MAX_ACCURACY_M:
      return False
    key = (gps.latitude, gps.longitude, getattr(gps, "unixTimestampMillis", None))
    if key == self._last_fix:
      return False
    self._last_fix = key
    return True

  def update(self, dt: float, v_ego: float, yaw_rate: float, left_blinker: bool, right_blinker: bool, lat_active: bool,
             steering_pressed: bool, gps, steering_torque: float = 0.0) -> float | None:
    """
    dt: s since the last call. v_ego: m/s. yaw_rate: carState.yawRate, rad/s, positive = left.
    left_blinker / right_blinker: carState (held through the flash). lat_active: carControl.latActive.
    steering_pressed / steering_torque: carState (torque positive = left). gps: a gpsLocationExternal-like object
    (latitude, longitude, bearingDeg, speed, hasFix, horizontalAccuracy, unixTimestampMillis) or None.
    Returns the curvature to command (1/m, positive = RIGHT) while turning, None otherwise.
    """
    side = "left" if left_blinker and not right_blinker else "right" if right_blinker and not left_blinker else ""
    if side != self._done_side:
      self._done_side = None
      if self.state == DONE:
        self.state, self.side = IDLE, ""
    if side != self._turn_side:
      self._turn_side, self._side_turned = side, 0.0
    elif side:
      self._side_turned += (math.degrees(yaw_rate) if side == "left" else -math.degrees(yaw_rate)) * dt
    # Only while idle: once armed, a driver starting the planned turn ahead of the planner is the cooperative case
    # (carcontroller coop_signaled_turn), and a junction passed while armed already ends the turn.
    if side and self.state == IDLE and self._side_turned >= DRIVER_TURNED_DEG:
      self.state, self.side = ARMED, side   # _end marks the side done, so nothing arms until the blinker goes off
      self._end("driver turned")

    # heading: GPS course when it means something, else carried on by the yaw rate (positive yaw = left = bearing down)
    new_fix = self._new_fix(gps)
    if new_fix and gps.speed >= GPS_BEARING_MIN_SPEED:
      self.heading = gps.bearingDeg % 360.0
    elif self.heading is not None:
      self.heading = (self.heading - math.degrees(yaw_rate) * dt) % 360.0

    if self.state == TURNING:
      out = self._turning(dt, v_ego, yaw_rate, side, lat_active, steering_pressed, steering_torque)
    elif self.state == ARMED:
      out = self._armed(dt, v_ego, side, lat_active, gps, new_fix)
    else:
      out = None
      if self.state == IDLE and side and lat_active and v_ego <= V_ARM_MAX and new_fix and self.heading is not None:
        self._try_arm(side, v_ego, gps)
    self._fill_log()
    self._emit_log(dt, v_ego, left_blinker, right_blinker, lat_active, steering_pressed, steering_torque, out)
    return out

  def _emit_log(self, dt, v_ego, left_blinker, right_blinker, lat_active, pressed, torque, out) -> None:
    changed = self.state != self._logged_state
    self._log_t += dt
    if not changed and not (self.state in (ARMED, TURNING) and self._log_t >= LOG_TICK_S):
      return
    lg = self.log
    fields = {"state": self.state, "side": self.side, "reason": lg.reason, "junction_dist": round(lg.junction_dist, 1),
              "planned_deg": round(lg.planned_deg, 1), "progress_deg": round(lg.progress_deg, 1),
              "radius": round(lg.radius, 1), "curvature": None if out is None else round(out, 4), "v": round(v_ego, 2),
              "heading": None if self.heading is None else round(self.heading, 1), "way": lg.way_id,
              "branch": lg.branch_way_id, "blinker": "L" if left_blinker else "R" if right_blinker else "",
              "lat_active": bool(lat_active), "pressed": bool(pressed), "torque": round(torque, 2)}
    if changed:
      fields["prev"] = self._logged_state
    try:
      if self.log_event is not None:
        self.log_event("map_turn" if changed else "map_turn_tick", **fields)
      else:
        from openpilot.common.swaglog import cloudlog
        cloudlog.event("map_turn" if changed else "map_turn_tick", **fields)
    except Exception:
      pass   # logging must never take the turn planner down
    self._logged_state = self.state
    self._log_t = 0.0

  def _try_arm(self, side: str, v: float, gps) -> None:
    roads = self.roads_src.get(gps.latitude, gps.longitude)
    if roads is None:
      return
    x, y = roads.to_xy(gps.latitude, gps.longitude)
    m = roads.match(x, y, self.heading, self._prefer_way)
    self.pos = (x, y)
    if m is None:
      self._prefer_way = None
      return
    self._prefer_way = roads.ways[m.way].way_id
    self.log.way_id, self.log.lateral = self._prefer_way, m.lateral
    j = roads.junction_ahead(m, x, y, side)
    if j is None:
      return
    self.state, self.side, self.roads, self.junction = ARMED, side, roads, j
    self.junction_xy = (j.x, j.y)
    self.planned = j.branch.change
    self.dist = self._along(x, y)
    self.log.reason = ""

  def _along(self, x: float, y: float) -> float:
    """m from (x, y) to the junction along the approach heading."""
    h = math.radians(self.junction.arrival_bearing)
    return (self.junction_xy[0] - x) * math.sin(h) + (self.junction_xy[1] - y) * math.cos(h)

  def _armed(self, dt, v, side, lat_active, gps, new_fix) -> float | None:
    if side != self.side:
      self._reset()
      self.log.reason = "blinker off"
      return None
    if not lat_active:
      self._end("lateral inactive")
      return None
    if v > V_ABORT:
      self._end("too fast")
      return None
    self.dist -= v * dt
    if new_fix:
      x, y = self.roads.to_xy(gps.latitude, gps.longitude)
      self.pos = (x, y)
      self.dist = self._along(x, y)
      m = self.roads.match(x, y, self.heading, self._prefer_way) if self.heading is not None else None
      self.no_match_t = 0.0 if m is not None else self.no_match_t
      if m is not None:
        self.log.lateral = m.lateral
    self.no_match_t += dt
    if self.no_match_t > NO_MATCH_ABORT_S:
      self._end("no map match")
      return None
    if self.dist < -PASSED_M:
      self._end("passed the junction")
      return None
    if v > V_ARM_MAX:
      return None   # wait: the turn only starts at a turn speed
    if v < V_START_MIN:
      self.waited_stopped = True
    r = self.radius_for(self.side, v)
    start = self.start_distance(r, self.planned, v)
    stop_start = self.tuning.stop_start_left_m if self.side == "left" else self.tuning.stop_start_right_m
    early = self.waited_stopped and v >= V_START_MIN and start < self.dist <= stop_start
    if self.dist > start and not early:
      return None
    if v < V_START_MIN:
      return None   # stopped at the line: wait for the car to move off
    if early:
      # moving off from the stop line, where the driver starts this turn: the arc that ends on the branch from here
      half = math.radians(abs(self.planned)) / 2.0
      fixed = start - r * math.tan(half)
      r = min(R_FROM_STOP_MAX, max(r, (self.dist - fixed) / math.tan(half)))
    elif self.dist < start - 0.5:
      # started late (slowed down or armed close in, or moving off from the stop line): tighten the arc to fit
      half = math.radians(abs(self.planned)) / 2.0
      fixed = self.start_distance(r, self.planned, v) - r * math.tan(half)
      r_fit = (self.dist - fixed) / math.tan(half)
      if r_fit < max(R_MIN - LATE_SLACK_M, v ** 2 / A_LAT_MAX):
        self._end("too late")
        return None
      r = max(r_fit, R_MIN, v ** 2 / A_LAT_MAX)
    self.state, self.radius, self.progress, self.t_turn = TURNING, r, 0.0, 0.0
    self.stall_t, self.stall_ref, self.override_t = 0.0, 0.0, 0.0
    self.log.start_dist = self.dist
    return self._turning(0.0, v, 0.0, side, lat_active, False, 0.0)

  def _turning(self, dt, v, yaw_rate, side, lat_active, pressed, torque) -> float | None:
    sign = 1.0 if self.side == "right" else -1.0
    if not lat_active:
      self._end("lateral inactive")
      return None
    # The blinker cancels itself once the wheel comes back from the turn: the turn is over, whoever made it. Route
    # 000000f6 (2026-11-14, left onto Doane St): it cancelled with the turn 56% done, 3.3 s before the planner thought
    # the turn was done, and the planner went on commanding a left turn onto the straight road. Only a blinker that
    # stays off (older logs show it flashing off for 0.3-0.4 s) once the turn is well under way: early in the turn a
    # blinker going off is not the turn being over. The other blinker is a change of mind: release at once.
    self.blinker_off_t = self.blinker_off_t + dt if side != self.side else 0.0
    if side and side != self.side:
      self._end("other blinker")
      return None
    if v > V_ABORT:
      self._end("too fast")
      return None
    # progress in the turn's direction only (yawRate positive = left, so a right turn has yawRate < 0)
    self.progress += max(0.0, -sign * math.degrees(yaw_rate)) * dt
    # The turn counts from the road the car arrived on, not from when the planner took over: a driver who starts
    # the turn by hand turns the car some way before the planner does. Route 000000f6 (2026-11-14): the driver had
    # turned 19 deg when the planner started, so it counted 19 deg short all through the turn and was still
    # commanding a hard left (wheel 127 deg, 16 mph) on the straight road after the turn.
    if self.heading is not None and self.junction is not None:
      self.progress = max(self.progress, sign * _wrap(self.heading - self.junction.arrival_bearing))
    self.t_turn += dt
    remaining = abs(self.planned) - self.progress
    if self.blinker_off_t >= BLINKER_OFF_S and self.progress >= BLINKER_OFF_DONE * abs(self.planned):
      self._end("blinker off")
      return None
    k_meas = -yaw_rate / max(v, 0.5)   # the car's own curvature now (positive = right)
    if remaining <= self.tuning.release_deg + unwind_heading_deg(v, max(0.0, sign * k_meas)):
      self._end("done")
      return None
    if self.t_turn > TURN_TIMEOUT_S:
      self._end("timeout")
      return None
    if v > STALL_MIN_V:
      self.stall_t += dt
    if self.progress - self.stall_ref >= STALL_DEG:
      self.stall_ref, self.stall_t = self.progress, 0.0
    elif self.stall_t > STALL_S:
      self._end("not turning")
      return None
    # driver torque against the turn (positive torque = left): a right turn is opposed by positive torque
    against = pressed and (torque * sign > 0.0)
    self.override_t = self.override_t + dt if against else 0.0
    if self.override_t > OVERRIDE_S:
      self._end("driver override")
      return None
    kappa = min(1.0 / self.radius, A_LAT_MAX / max(v, 0.1) ** 2, MAX_CURVATURE)
    taper = 1.0 if self.tuning.taper_deg <= 0 else min(1.0, max(self.tuning.taper_min, remaining / self.tuning.taper_deg))
    self.curvature = sign * kappa * taper
    return self.curvature

  def _fill_log(self) -> None:
    lg = self.log
    lg.state, lg.side = self.state, self.side
    lg.junction_dist = self.dist if self.state in (ARMED, TURNING) else 0.0
    lg.planned_deg, lg.progress_deg = self.planned, self.progress
    lg.radius, lg.curvature = self.radius, self.curvature if self.state == TURNING else 0.0
    lg.branch_way_id = self.junction.branch.way_id if self.junction else 0
