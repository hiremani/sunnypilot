"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Speed Bump Assist: ease off before a mapped speed bump, the way a driver lifts off the accelerator.

The bumps are OpenStreetMap traffic_calming nodes, already cached by the web HUD's map features (webhud/maptiles.py
MapFeatures, FEAT_DEG cells under <mapd root>/features/; Stop Sign Assist reads the same cells). Each kind gets a
crossing speed:
  bump, mini_bumps          10 mph   short and sharp
  hump                      15 mph   longer, rounder
  cushion, table            20 mph   gentle (cushions let wide wheels straddle them)
Other kinds (rumble_strip, dip, chicane, island, choker, "yes", "no") are not slowed for.

  idle ──(bump within the easing distance)──> easing ──(at the bump)──> crossing ──(passed)──> idle
                                                 └──── the driver presses the gas: that bump is theirs ────┘

* Easing: the target speed follows v = sqrt(v_bump^2 + 2 * A_EASE * d): a lift-off slowdown (an EV's regen, about
  0.6 m/s^2), so from 25 mph the car starts easing ~110 m out (a second early, for the planner's lag) and arrives
  at the crossing speed without the brake.
* Too late for that: up to A_MAX (light braking). A bump that can't be met even then is left to the driver; a hard
  stop for a bump would be worse than the bump.
* Crossing: hold the crossing speed until the car is CROSS_M past the bump's map point (the rear wheels too).

Speed Bump Assist needs openpilot to control the speed. Whatever drives the speed, the planner also raises a
"Speed bump ahead" warning (self.warn) once per bump, when one is near and the car is faster than its crossing speed;
selfdrived turns that into a chime with SpeedBumpAlert. Shadow mode (SpeedBumpAssistShadow) runs the same logic on
every drive and only logs ("speed_bump" events), to check the map's bumps before letting it slow the car.

Bumps OpenStreetMap is missing, and mapped ones that aren't there, go in <mapd root>/local_speed_bumps.json (kept
beside the cached map data, out of git):
  {"bumps": [{"id": -1, "lat": 37.5, "lon": -121.9, "kind": "bump", "note": "..."}], "ignore": [6107009436]}
"""
import json
import math
import os
from dataclasses import dataclass
from enum import IntEnum

from openpilot.common.constants import CV
from openpilot.common.gps import get_gps_location_service
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.controls.lib.map_near import REFRESH_M, CellMap
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.webhud.maptiles import FEAT_DEG, MapFeatures

PARAM = "SpeedBumpAssist"
SHADOW_PARAM = "SpeedBumpAssistShadow"
ALERT_PARAM = "SpeedBumpAlert"
LOCAL_FILE = "local_speed_bumps.json"

BASE_CROSS_MPH = 10.0      # the plain bump's crossing speed: the driver profile's bump_cross_mph scales all kinds by it
CROSS_MAX_MPH = 25.0       # never faster over any mapped bump, whatever the profile
CROSSING_MPH = {"bump": 10.0, "mini_bumps": 10.0, "hump": 15.0, "cushion": 20.0, "table": 20.0}

EARTH_R = 6371000.0
LOOKAHEAD_M = 150.0        # bumps this far ahead along the heading
MAX_LATERAL_M = 7.0        # ...and this close to the line the car drives on (a bump is in the road, not beside it)
MAX_BEARING_ERR_DEG = 20.0
A_EASE = 0.6               # m/s^2: lifting off the accelerator
A_MAX = 1.0                # m/s^2: at most light braking when a bump comes up late
LAG_S = 1.0                # the planner's ~1 s speed-loop time constant
CROSS_M = 6.0              # m past the bump's point before speeding up again (a car length, rear wheels over too)
MAX_APPROACH_V = 18.0      # m/s (40 mph): faster than this, a "bump" ahead is more likely a map error or another road
WARN_S = 6.0               # warn this many seconds out...
WARN_MIN_M = 40.0          # ...but from no further out than this...
WARN_LATE_M = 15.0         # ...and not at all once closer than this (too late to act on; the replays showed 7-8 m warnings)
WARN_MARGIN_V = 2.0 * CV.MPH_TO_MS   # only warn when faster than the crossing speed by more than this
GPS_MAX_AGE_S = 2.0
GPS_MAX_ACCURACY_M = 15.0
MIN_GPS_SPEED = 2.0
RETRY_S = 60.0


@dataclass(frozen=True)
class SpeedBump:
  node_id: int
  lat: float
  lon: float
  kind: str

  @property
  def v_cross(self) -> float:
    return CROSSING_MPH[self.kind] * CV.MPH_TO_MS


@dataclass(frozen=True)
class BumpAhead:
  node_id: int
  distance: float   # m along the heading
  v_cross: float    # m/s


def bumps_from_cell(cell: dict) -> list[SpeedBump]:
  out = []
  for node in cell.get("nodes", []):
    try:
      node_id, lat, lon, tags = node
    except (TypeError, ValueError):
      continue
    kind = tags.get("traffic_calming") if isinstance(tags, dict) else None
    if kind in CROSSING_MPH:
      out.append(SpeedBump(int(node_id), float(lat), float(lon), kind))
  return out


def local_bumps(path: str) -> tuple[list[SpeedBump], set[int]]:
  """(added bumps, OSM ids to ignore) from `path`; nothing when there is no file. A bad entry is skipped."""
  if not os.path.isfile(path):
    return [], set()
  with open(path) as f:
    data = json.load(f)
  added = []
  for e in data.get("bumps", []):
    try:
      node_id = int(e["id"])
      kind = str(e.get("kind", "bump"))
      if node_id >= 0 or kind not in CROSSING_MPH:
        raise ValueError("a local bump needs a negative id and a known kind")
      added.append(SpeedBump(node_id, float(e["lat"]), float(e["lon"]), kind))
    except (KeyError, TypeError, ValueError) as err:
      cloudlog.warning(f"speed bump: skipping local bump {e!r}: {err}")
  ignore = set()
  for i in data.get("ignore", []):
    try:
      ignore.add(int(i))
    except (TypeError, ValueError):
      pass
  return added, ignore


def find_bump_ahead(lat: float, lon: float, bearing_deg: float, bumps, skip_ids=frozenset()) -> BumpAhead | None:
  """The nearest bump in front of the car on its line of travel, or None."""
  cos_lat = math.cos(math.radians(lat))
  hx, hy = math.sin(math.radians(bearing_deg)), math.cos(math.radians(bearing_deg))
  best = None
  for b in bumps:
    if b.node_id in skip_ids:
      continue
    dx = math.radians(b.lon - lon) * cos_lat * EARTH_R
    dy = math.radians(b.lat - lat) * EARTH_R
    along = dx * hx + dy * hy
    if not (0.0 < along <= LOOKAHEAD_M):
      continue
    lateral = abs(dx * hy - dy * hx)
    if lateral > MAX_LATERAL_M:
      continue
    if along > 2 * MAX_LATERAL_M and math.degrees(math.atan2(lateral, along)) > MAX_BEARING_ERR_DEG:
      continue
    if best is None or along < best.distance:
      best = BumpAhead(b.node_id, along, b.v_cross)
  return best


class SpeedBumpMap(CellMap):
  """The bumps in the 3 x 3 feature cells around the car (map_near.CellMap; read in the map worker on the device)."""
  KIND = "bumps"
  RETRY_S = RETRY_S

  def __init__(self, features: MapFeatures | None = None):
    super().__init__(LOOKAHEAD_M + REFRESH_M + 10.0, use_worker=features is None)
    self.features = features or MapFeatures()
    self.local_path = os.path.join(os.path.dirname(os.path.normpath(self.features.root)), LOCAL_FILE)
    if self._worker is not None:
      self._worker.start()

  def _compute(self, key: tuple[int, int], cell: dict | None = None) -> list[SpeedBump] | None:
    if cell is None:
      path = self.features.cell_path(*key)
      if path is None or not os.path.isfile(path):
        return None
      with open(path) as f:
        cell = json.load(f)
    bumps = bumps_from_cell(cell)
    try:
      added, ignore = local_bumps(self.local_path)
    except Exception:
      cloudlog.exception(f"speed bump: reading {self.local_path} failed")
      added, ignore = [], set()
    bumps = [b for b in bumps if b.node_id not in ignore]
    return bumps + [b for b in added if (math.floor(b.lat / FEAT_DEG), math.floor(b.lon / FEAT_DEG)) == key]

  def bumps_near(self, lat: float, lon: float) -> list[SpeedBump]:
    """The loaded bumps near the car (rebuilt every REFRESH_M or so, not every tick)."""
    return self.near(lat, lon)


def easing_v_target(dist: float, v_cross: float, v_ego: float, a: float = A_EASE) -> float:
  """The speed to be at now so the car eases down to v_cross at the bump (dist m ahead) at deceleration a."""
  d = max(dist - v_ego * LAG_S, 0.0)
  return math.sqrt(v_cross ** 2 + 2.0 * a * d)


def easing_distance(v_ego: float, v_cross: float, a: float = A_EASE) -> float:
  """How far out easing starts: where the easing profile meets the current speed, plus one speed-loop lag of travel
  so the car (which follows its target about LAG_S late) stays on the profile instead of falling behind it."""
  if v_ego <= v_cross:
    return 0.0
  return (v_ego ** 2 - v_cross ** 2) / (2.0 * a) + 2.0 * v_ego * LAG_S


class SpeedBumpState(IntEnum):
  disabled = 0
  idle = 1
  easing = 2
  crossing = 3


class SpeedBumpAssist:
  def __init__(self, bump_map: SpeedBumpMap | None = None, params: Params | None = None, shadow: bool = False):
    self.params = params or Params()
    self.shadow = shadow
    self.param = SHADOW_PARAM if shadow else PARAM
    self.enabled = self._read_enabled()
    self.gps_service = get_gps_location_service(self.params)
    self.map = bump_map or SpeedBumpMap()
    self.frame = -1
    self.state = SpeedBumpState.disabled
    self.node_id: int | None = None
    self.distance = 0.0
    self.v_cross = V_CRUISE_UNSET
    self.a_plan = A_EASE           # the deceleration this bump is planned at (A_EASE, or A_MAX when taken on late)
    self.cross_factor = 1.0        # the driver's crossing speed over the default's (set_profile)
    self.done_ids: set[int] = set()
    self.warned_ids: set[int] = set()
    self.warn = False              # a "Speed bump ahead" warning is due this frame (rising once per bump)
    self.output_v_target = V_CRUISE_UNSET
    self.output_a_target = 0.0

  def set_profile(self, bump_cross_mph: float) -> None:
    """The driver profile's crossing speed for a plain bump (mph; 0 = the defaults): every kind scales by it."""
    self.cross_factor = bump_cross_mph / BASE_CROSS_MPH if bump_cross_mph >= 5.0 else 1.0

  def _read_enabled(self) -> bool:
    """The live copy runs with SpeedBumpAssist; the shadow copy (log only, and the source of the warning whatever drives
    the speed) with SpeedBumpAssistShadow or SpeedBumpAlert."""
    if not self.shadow:
      return self.params.get_bool(PARAM)
    return self.params.get_bool(SHADOW_PARAM) or self.params.get_bool(ALERT_PARAM)

  @property
  def is_active(self) -> bool:
    return self.state in (SpeedBumpState.easing, SpeedBumpState.crossing)

  def _reset(self) -> None:
    if self.state != SpeedBumpState.disabled:
      self.state = SpeedBumpState.idle
    self.node_id = None
    self.distance = 0.0
    self.v_cross = V_CRUISE_UNSET

  def _finish(self) -> None:
    if self.node_id is not None:
      self.done_ids.add(self.node_id)
      if len(self.done_ids) > 500:
        self.done_ids.clear()
        self.warned_ids.clear()
    self._reset()

  def _locate(self, gps, v_ego: float, now_s: float) -> bool:
    fix_ok = (gps.hasFix and gps.horizontalAccuracy <= GPS_MAX_ACCURACY_M and
              now_s - gps.unixTimestampMillis * 1e-3 <= GPS_MAX_AGE_S)
    if fix_ok and gps.speed >= MIN_GPS_SPEED:
      ahead = find_bump_ahead(gps.latitude, gps.longitude, gps.bearingDeg, self.map.bumps_near(gps.latitude, gps.longitude),
                              self.done_ids)
      if ahead is None:
        if self.node_id is not None and self.distance > -CROSS_M:
          # the bump's point is behind the GPS now; carry on by dead reckoning until the car is past it
          self.distance -= v_ego * DT_MDL
          return True
        self._finish()
        return False
      if ahead.node_id != self.node_id:
        self._reset()
        self.node_id = ahead.node_id
      self.distance = ahead.distance
      self.v_cross = min(ahead.v_cross * self.cross_factor, CROSS_MAX_MPH * CV.MPH_TO_MS)
      return True
    if self.node_id is not None:
      self.distance -= v_ego * DT_MDL
      if self.distance < -CROSS_M:
        self._finish()
        return False
      return True
    return False

  def update(self, sm, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float, now_s: float) -> None:
    prev = (self.state, self.node_id)
    self._update(sm, long_enabled, long_override, v_ego, a_ego, now_s)
    if (self.state, self.node_id) != prev and self.state != SpeedBumpState.disabled:
      cloudlog.event("speed_bump", shadow=self.shadow, state=self.state.name, prev=prev[0].name, bump=self.node_id,
                     dist=round(self.distance, 1), v=round(v_ego, 2), v_cross=round(min(self.v_cross, 99.0), 2),
                     v_target=round(min(self.output_v_target, 99.0), 2))

  def _update(self, sm, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float, now_s: float) -> None:
    self.frame += 1
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enabled = self._read_enabled()
    self.output_a_target = a_ego
    self.output_v_target = V_CRUISE_UNSET
    self.warn = False
    if not (self.enabled and long_enabled):
      self.state = SpeedBumpState.disabled
      self._reset()
      return
    if self.state == SpeedBumpState.disabled:
      self.state = SpeedBumpState.idle

    if long_override and self.is_active:
      self._finish()   # the driver pressed the gas: this bump is theirs

    if not self._locate(sm[self.gps_service], v_ego, now_s):
      self.state = SpeedBumpState.idle
      return

    near = self.distance <= max(WARN_MIN_M, v_ego * WARN_S)
    if near and self.distance > WARN_LATE_M and v_ego > self.v_cross + WARN_MARGIN_V and self.node_id not in self.warned_ids:
      self.warned_ids.add(self.node_id)
      self.warn = True

    if self.distance <= 0.0:
      self.state = SpeedBumpState.crossing
      self.output_v_target = self.v_cross
      return
    if not self.is_active:
      if v_ego > MAX_APPROACH_V or self.distance > easing_distance(v_ego, self.v_cross):
        self.state = SpeedBumpState.idle
        return
      # taking a bump on: is there room to get down to its speed with at most light braking?
      if v_ego > self.v_cross and self.distance < easing_distance(v_ego, self.v_cross, A_MAX):
        cloudlog.event("speed_bump_left_to_driver", shadow=self.shadow, bump=self.node_id, dist=round(self.distance, 1),
                       v=round(v_ego, 2))
        self._finish()
        self.state = SpeedBumpState.idle
        return
      self.a_plan = A_EASE if self.distance >= easing_distance(v_ego, self.v_cross, A_EASE) - 1.0 else A_MAX
    self.state = SpeedBumpState.easing
    self.output_v_target = min(easing_v_target(self.distance, self.v_cross, v_ego, self.a_plan), max(v_ego, self.v_cross))
