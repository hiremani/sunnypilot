"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Map caution: two gentle rules from OpenStreetMap points the web HUD already caches (webhud/maptiles.py MapFeatures),
for openpilot longitudinal. Each is off unless its param is set.

* Traffic signals (TrafficSignalCoast): a mapped highway=traffic_signals node on the car's line of travel, between
  SIGNAL_MIN_M and max(SIGNAL_MIN_M, SIGNAL_S of travel) ahead: no accelerating toward it (the acceleration cap is 0,
  a lift-off). Never braking: a red light is the driving model's to see (experimental / DEC), the map can't know the
  colour. Not under SIGNAL_MIN_V, and not once the light is closer than SIGNAL_MIN_M, so pulling away from the stop
  line on green and crossing the junction are never held back.
* Crosswalks (CrosswalkCaution): on a street the narrow-street detector flags (no lane lines, curbs close: parked cars
  and people stepping out), a marked crosswalk ahead within CROSSWALK_M caps the speed at CROSSWALK_MPH.
"""
import json
import math
import os
import threading
import time
from dataclasses import dataclass

from openpilot.common.constants import CV
from openpilot.common.gps import get_gps_location_service
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.webhud.maptiles import FEAT_DEG, MapFeatures

SIGNAL_PARAM = "TrafficSignalCoast"
CROSSWALK_PARAM = "CrosswalkCaution"

EARTH_R = 6371000.0
MAX_LATERAL_M = 7.0
MAX_BEARING_ERR_DEG = 20.0
SIGNAL_S = 4.0              # s of travel: the coast window ahead of a light...
SIGNAL_MIN_M = 25.0         # ...from at least this far out, and nothing closer (the junction itself: never hold back)
SIGNAL_MAX_M = 150.0
SIGNAL_MIN_V = 5.0          # m/s (11 mph)
CROSSWALK_M = 40.0
CROSSWALK_MPH = 12.0
CROSSWALK_ACCEL = -0.5      # m/s^2 to get down to it
GPS_MAX_AGE_S = 2.0
GPS_MAX_ACCURACY_M = 15.0
MIN_GPS_SPEED = 2.0
RETRY_S = 60.0
UNMARKED = ("unmarked", "no", "informal")


@dataclass(frozen=True)
class MapPoint:
  node_id: int
  lat: float
  lon: float
  kind: str     # "signal" or "crosswalk"


def points_from_cell(cell: dict) -> list[MapPoint]:
  out = []
  for node in cell.get("nodes", []):
    try:
      node_id, lat, lon, tags = node
    except (TypeError, ValueError):
      continue
    if not isinstance(tags, dict):
      continue
    hw = tags.get("highway")
    if hw == "traffic_signals":
      out.append(MapPoint(int(node_id), float(lat), float(lon), "signal"))
    elif hw == "crossing" and tags.get("crossing") not in UNMARKED and tags.get("crossing:markings") != "no":
      out.append(MapPoint(int(node_id), float(lat), float(lon), "crosswalk"))
  return out


def find_point_ahead(lat: float, lon: float, bearing_deg: float, points, kind: str, max_m: float) -> tuple[int, float] | None:
  """(node id, metres ahead) of the nearest point of this kind on the car's line of travel, or None."""
  cos_lat = math.cos(math.radians(lat))
  hx, hy = math.sin(math.radians(bearing_deg)), math.cos(math.radians(bearing_deg))
  best = None
  for p in points:
    if p.kind != kind:
      continue
    dx = math.radians(p.lon - lon) * cos_lat * EARTH_R
    dy = math.radians(p.lat - lat) * EARTH_R
    along = dx * hx + dy * hy
    if not (0.0 < along <= max_m):
      continue
    lateral = abs(dx * hy - dy * hx)
    if lateral > MAX_LATERAL_M:
      continue
    if along > 2 * MAX_LATERAL_M and math.degrees(math.atan2(lateral, along)) > MAX_BEARING_ERR_DEG:
      continue
    if best is None or along < best[1]:
      best = (p.node_id, along)
  return best


class PointMap:
  """Signals and crosswalks in the 3 x 3 feature cells around the car, loaded in the background."""

  def __init__(self, features: MapFeatures | None = None):
    self.features = features or MapFeatures()
    self._cells: dict[tuple[int, int], list[MapPoint]] = {}
    self._pending: set[tuple[int, int]] = set()
    self._failed: dict[tuple[int, int], float] = {}
    self._lock = threading.Lock()

  def _load(self, key: tuple[int, int]) -> None:
    points = None
    try:
      path = self.features.cell_path(*key)
      if path is not None and os.path.isfile(path):
        with open(path) as f:
          points = points_from_cell(json.load(f))
    except Exception:
      cloudlog.exception(f"map caution: loading feature cell {key} failed")
    with self._lock:
      self._pending.discard(key)
      if points is not None:
        self._cells[key] = points
        self._failed.pop(key, None)
      else:
        self._failed[key] = time.monotonic()

  def points_near(self, lat: float, lon: float) -> list[MapPoint]:
    klat, klon = math.floor(lat / FEAT_DEG), math.floor(lon / FEAT_DEG)
    out: list[MapPoint] = []
    with self._lock:
      for key in [(klat + i, klon + j) for i in (-1, 0, 1) for j in (-1, 0, 1)]:
        if key in self._cells:
          out.extend(self._cells[key])
        elif key not in self._pending and time.monotonic() - self._failed.get(key, -RETRY_S) >= RETRY_S:
          self._pending.add(key)
          threading.Thread(target=self._load, args=(key,), daemon=True).start()
      if len(self._cells) > 64:
        for key in [k for k in self._cells if abs(k[0] - klat) > 3 or abs(k[1] - klon) > 3]:
          del self._cells[key]
    return out


class MapCaution:
  def __init__(self, point_map: PointMap | None = None, params: Params | None = None):
    self.params = params or Params()
    self.gps_service = get_gps_location_service(self.params)
    self.map = point_map or PointMap()
    self.signal_on = self.crosswalk_on = False
    self.frame = -1
    self.signal_id: int | None = None
    self.signal_dist = 0.0
    self.signal_coast = False         # not accelerating toward a mapped light
    self.crosswalk_id: int | None = None
    self.crosswalk_dist = 0.0
    self.crosswalk_caution = False    # capped for a crosswalk on a narrow street
    self.v_target = float("inf")
    self.a_target = 0.0

  def _read_params(self) -> None:
    self.signal_on = self.params.get_bool(SIGNAL_PARAM)
    self.crosswalk_on = self.params.get_bool(CROSSWALK_PARAM)

  def accel_cap(self) -> float:
    return 0.0 if self.signal_coast else float("inf")

  def update(self, sm, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float, narrow_street: bool,
             now_s: float) -> None:
    self.frame += 1
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self._read_params()
    prev = (self.signal_coast, self.crosswalk_caution)
    self.signal_coast = self.crosswalk_caution = False
    self.v_target, self.a_target = float("inf"), a_ego
    if not (long_enabled and not long_override and (self.signal_on or self.crosswalk_on)):
      self._log(prev, v_ego)
      return
    gps = sm[self.gps_service]
    fix_ok = (gps.hasFix and gps.horizontalAccuracy <= GPS_MAX_ACCURACY_M and
              now_s - gps.unixTimestampMillis * 1e-3 <= GPS_MAX_AGE_S and gps.speed >= MIN_GPS_SPEED)
    if not fix_ok:
      self._log(prev, v_ego)
      return
    points = self.map.points_near(gps.latitude, gps.longitude)
    if self.signal_on and v_ego >= SIGNAL_MIN_V:
      ahead = find_point_ahead(gps.latitude, gps.longitude, gps.bearingDeg, points, "signal", SIGNAL_MAX_M)
      self.signal_id, self.signal_dist = ahead if ahead else (None, 0.0)
      self.signal_coast = ahead is not None and SIGNAL_MIN_M <= ahead[1] <= max(SIGNAL_MIN_M, v_ego * SIGNAL_S)
    if self.crosswalk_on and narrow_street:
      ahead = find_point_ahead(gps.latitude, gps.longitude, gps.bearingDeg, points, "crosswalk", CROSSWALK_M)
      self.crosswalk_id, self.crosswalk_dist = ahead if ahead else (None, 0.0)
      if ahead is not None:
        self.crosswalk_caution = True
        self.v_target = CROSSWALK_MPH * CV.MPH_TO_MS
        self.a_target = CROSSWALK_ACCEL if v_ego > self.v_target else a_ego
    self._log(prev, v_ego)

  def _log(self, prev: tuple[bool, bool], v_ego: float) -> None:
    if (self.signal_coast, self.crosswalk_caution) != prev:
      cloudlog.event("map_caution", signal_coast=self.signal_coast, signal=self.signal_id, signal_dist=round(self.signal_dist, 1),
                     crosswalk=self.crosswalk_caution, crosswalk_id=self.crosswalk_id,
                     crosswalk_dist=round(self.crosswalk_dist, 1), v=round(v_ego, 2))
