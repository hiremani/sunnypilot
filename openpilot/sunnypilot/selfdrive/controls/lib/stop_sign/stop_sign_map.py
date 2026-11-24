"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Stop signs near the car, from the OpenStreetMap point features the webhud already caches (webhud/maptiles.py
MapFeatures: highway=stop nodes, FEAT_DEG cells under <mapd root>/features/). Cells load, and download when
missing, in the map worker process (map_worker.py), so the 20 Hz planner never waits on disk, the network or the
road-tile geometry.

A sign that clearly holds another direction of travel (stop_sign_direction.py: its direction tag, or the road it
sits on in mapd's road tiles, road_geometry.py) is skipped; one that can't be told apart is kept as before. That
includes a stop=all node with an explicit forward/backward: it is one approach's node of an all-way stop. The
road-tile lookup runs as the cell is read and only adds the approaches it could resolve.

Signs OpenStreetMap is missing can be added in <mapd root>/local_stop_signs.json (kept beside the cached map data, out of
git, since it holds the driver's own streets):
  {"signs": [{"id": -1, "lat": 37.5, "lon": -121.9, "all_way": true, "note": "Main & 1st"},
             {"id": -2, "lat": 37.6, "lon": -121.8, "approaches": [90.0], "note": "eastbound only"}]}
id must be negative (OSM ids are positive). all_way holds every approach; approaches lists the bearings (deg) of the
traffic it holds. Each is merged into its feature cell when the cell loads.
"""
import json
import math
import os
from dataclasses import dataclass, replace

from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.controls.lib.map_near import REFRESH_M, CellMap
from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.road_geometry import RoadGeometry
from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_direction import (
  Applies, approaches_from_tags, approaches_from_ways, direction_from_tags, is_both, is_relative, sign_applies,
)
from openpilot.sunnypilot.webhud.maptiles import FEAT_DEG, MapFeatures

EARTH_R = 6371000.0
LOOKAHEAD_M = 300.0        # only signs this far ahead along the heading
MAX_LATERAL_M = 12.0       # ...and this close to the line the car is driving on
MAX_BEARING_ERR_DEG = 25.0  # ...and roughly in front (a sign off to the side is on a crossing road)
SAME_JUNCTION_M = 25.0     # signs this close along the road are one junction (OSM often maps one per approach)
RETRY_S = 60.0             # wait this long before trying a cell that failed to load again
LOCAL_SIGNS_FILE = "local_stop_signs.json"


@dataclass(frozen=True)
class StopSign:
  node_id: int
  lat: float
  lon: float
  direction: str | None = None                   # the OSM direction tag (or a synonym key's value), as mapped
  all_way: bool = False                          # stop=all: every approach, unless an explicit forward/backward resolves
  approaches: tuple[float, ...] | None = None    # bearings of the traffic it holds; None: not known
  margin_m: float | None = None                  # m before the map point where the car stops (stop_margin); None: default

  @property
  def needs_geometry(self) -> bool:
    if self.approaches is not None or is_both(self.direction):
      return False
    return is_relative(self.direction) if self.all_way else True


DEFAULT_MARGIN_M = 6.0   # m before a sign's map point where the car stops: a two-lane residential cross street
MARGIN_MAX_M = 16.0
MARGIN_SETBACK_M = 2.5   # m from the cross street's edge back to the stop line
LANE_M = 3.5
# half the carriageway of a cross street by its class, where OSM gives no lane count (m)
HALF_WIDTH_M = {"motorway": 10.5, "trunk": 10.0, "primary": 8.5, "secondary": 7.0, "tertiary": 4.5}
HALF_WIDTH_DEFAULT_M = 3.5


def stop_margin(ways) -> float | None:
  """m before a sign's map point to stop at, from the ways meeting at its node. OSM maps a stop at the junction node,
  so the stop line is half the cross street's width (plus a setback) before it: 6 m on a two-lane residential
  street, as before, and further back the wider the widest street there is (the driver, 2026-11-23: on wider cross
  streets the stop was a few metres too far on). None (the default margin) without road tiles, or when the node is
  not a junction (one way through it: the sign is mapped on its own approach)."""
  if not ways or len({w.way_id for w in ways}) < 2:
    return None
  half = max((w.lanes * LANE_M / 2.0) if w.lanes > 0 else HALF_WIDTH_M.get(w.highway, HALF_WIDTH_DEFAULT_M) for w in ways)
  return min(max(half + MARGIN_SETBACK_M, DEFAULT_MARGIN_M), MARGIN_MAX_M)


@dataclass(frozen=True)
class StopAhead:
  node_id: int
  distance: float  # m along the heading
  lateral: float   # m off the heading line
  margin: float = DEFAULT_MARGIN_M   # m before the map point where the car stops


def stop_signs_from_cell(cell: dict) -> list[StopSign]:
  out = []
  for node in cell.get("nodes", []):
    try:
      node_id, lat, lon, tags = node
    except (TypeError, ValueError):
      continue
    if isinstance(tags, dict) and tags.get("highway") == "stop":
      direction = direction_from_tags(tags)
      all_way = tags.get("stop") == "all"
      # only an explicit forward/backward (road tiles, resolve_approaches) narrows an all-way stop, never a compass value
      out.append(StopSign(int(node_id), float(lat), float(lon), direction, all_way,
                          None if all_way else approaches_from_tags(direction)))
  return out


def local_stop_signs(path: str) -> list[StopSign]:
  """The hand-added signs in `path` (see the module docstring); [] when there is no file. A bad entry is skipped."""
  if not os.path.isfile(path):
    return []
  with open(path) as f:
    data = json.load(f)
  out = []
  for e in data.get("signs", []):
    try:
      node_id = int(e["id"])
      if node_id >= 0:
        raise ValueError("local stop sign ids must be negative")
      approaches = e.get("approaches")
      approaches = tuple(float(a) % 360.0 for a in approaches) if approaches else None
      all_way = bool(e.get("all_way", False))
      if not all_way and approaches is None:
        raise ValueError("a local stop sign needs all_way or approaches")
      out.append(StopSign(node_id, float(e["lat"]), float(e["lon"]), None, all_way, approaches))
    except (KeyError, TypeError, ValueError) as err:
      cloudlog.warning(f"stop sign: skipping local stop sign {e!r}: {err}")
  return out


def resolve_approaches(signs: list[StopSign], geometry: RoadGeometry) -> list[StopSign]:
  """The signs with what the road tiles add: approaches for forward/backward and untagged signs (for a stop=all sign,
  an explicit forward/backward only). Blocking."""
  out = []
  for s in signs:
    ways = geometry.ways_at(s.lat, s.lon)
    if s.needs_geometry:
      approaches = approaches_from_ways(s.direction, ways, explicit_only=s.all_way)
      if approaches is not None:
        s = replace(s, approaches=approaches)
    margin = stop_margin(ways)
    if margin is not None:
      s = replace(s, margin_m=margin)
    out.append(s)
  return out


def find_stop_ahead(lat: float, lon: float, bearing_deg: float, signs, skip_ids=frozenset(),
                    prefer_id: int | None = None) -> StopAhead | None:
  """The nearest stop sign in front of the car on its line of travel, or None. prefer_id, the sign already being
  approached, is kept while it is still ahead and part of the same junction as the nearest one, so two nodes mapped
  for one junction don't trade places every few frames."""
  cos_lat = math.cos(math.radians(lat))
  hx, hy = math.sin(math.radians(bearing_deg)), math.cos(math.radians(bearing_deg))
  best = None
  preferred = None
  for s in signs:
    if s.node_id in skip_ids:
      continue
    if sign_applies(s, bearing_deg) == Applies.NO:
      continue   # for another direction of travel (a crossing road's two-way stop, the opposing approach's node)
    dx = math.radians(s.lon - lon) * cos_lat * EARTH_R  # east, m
    dy = math.radians(s.lat - lat) * EARTH_R            # north, m
    along = dx * hx + dy * hy
    if not (0.0 < along <= LOOKAHEAD_M):
      continue
    lateral = abs(dx * hy - dy * hx)
    if lateral > MAX_LATERAL_M:
      continue
    # close in, a few metres sideways is a big angle; only check the bearing once it's meaningful
    if along > 2 * MAX_LATERAL_M and math.degrees(math.atan2(lateral, along)) > MAX_BEARING_ERR_DEG:
      continue
    ahead = StopAhead(s.node_id, along, lateral, s.margin_m if s.margin_m is not None else DEFAULT_MARGIN_M)
    if s.node_id == prefer_id:
      preferred = ahead
    if best is None or along < best.distance:
      best = ahead
  if preferred is not None and best is not None and preferred.distance <= best.distance + SAME_JUNCTION_M:
    return preferred
  return best


class StopSignMap(CellMap):
  """The stop signs in the 3 x 3 feature cells around the car (map_near.CellMap): on the device each cell is read, and
  its approaches resolved from the road tiles, in the map worker process."""
  KIND = "stops"
  RETRY_S = RETRY_S

  def __init__(self, features: MapFeatures | None = None, geometry: RoadGeometry | None = None):
    super().__init__(LOOKAHEAD_M + REFRESH_M + 10.0, use_worker=features is None and geometry is None)
    self.features = features or MapFeatures()
    self.geometry = geometry or RoadGeometry()
    self.local_path = os.path.join(os.path.dirname(os.path.normpath(self.features.root)), LOCAL_SIGNS_FILE)
    if self._worker is not None:
      self._worker.start()

  def _local_in_cell(self, key: tuple[int, int]) -> list[StopSign]:
    try:
      signs = local_stop_signs(self.local_path)
    except Exception:
      cloudlog.exception(f"stop sign: reading {self.local_path} failed")
      return []
    return [s for s in signs if (math.floor(s.lat / FEAT_DEG), math.floor(s.lon / FEAT_DEG)) == key]

  def _compute(self, key: tuple[int, int], cell: dict | None = None) -> list[StopSign] | None:
    """The cell's signs (OSM and local) with the approaches the road tiles resolve. Blocking: seconds for a big cell."""
    if cell is None:
      path = self.features.cell_path(*key)
      if path is None or not os.path.isfile(path):
        return None
      with open(path) as f:
        cell = json.load(f)
    signs = stop_signs_from_cell(cell) + self._local_in_cell(key)
    if signs:   # every sign: the stop margin comes from the ways at its node, the approaches where they are needed
      try:
        signs = resolve_approaches(signs, self.geometry)
      except Exception:
        cloudlog.exception(f"stop sign: road geometry for cell {key} failed")
    return signs

  def signs_near(self, lat: float, lon: float) -> list[StopSign]:
    """The loaded signs near the car (rebuilt every REFRESH_M or so, not every tick)."""
    return self.near(lat, lon)
