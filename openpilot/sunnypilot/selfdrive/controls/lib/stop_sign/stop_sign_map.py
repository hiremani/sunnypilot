"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Stop signs near the car, from the OpenStreetMap point features the webhud already caches (webhud/maptiles.py
MapFeatures: highway=stop nodes, FEAT_DEG cells under <mapd root>/features/). Cells load, and download when
missing, on a background thread, so the 20 Hz planner never waits on disk or the network.

A sign that clearly holds another direction of travel (stop_sign_direction.py: its direction tag, or the road it
sits on in mapd's road tiles, road_geometry.py) is skipped; one that can't be told apart is kept as before. That
includes a stop=all node with an explicit forward/backward: it is one approach's node of an all-way stop. A cell's
signs are usable as soon as the cell is read; the road-tile lookup that follows, on the same background thread,
only adds the approaches it could resolve.
"""
import json
import math
import os
import threading
import time
from dataclasses import dataclass, replace

from openpilot.common.swaglog import cloudlog
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


@dataclass(frozen=True)
class StopSign:
  node_id: int
  lat: float
  lon: float
  direction: str | None = None                   # the OSM direction tag (or a synonym key's value), as mapped
  all_way: bool = False                          # stop=all: every approach, unless an explicit forward/backward resolves
  approaches: tuple[float, ...] | None = None    # bearings of the traffic it holds; None: not known

  @property
  def needs_geometry(self) -> bool:
    if self.approaches is not None or is_both(self.direction):
      return False
    return is_relative(self.direction) if self.all_way else True


@dataclass(frozen=True)
class StopAhead:
  node_id: int
  distance: float  # m along the heading
  lateral: float   # m off the heading line


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


def resolve_approaches(signs: list[StopSign], geometry: RoadGeometry) -> list[StopSign]:
  """The signs with what the road tiles add: approaches for forward/backward and untagged signs (for a stop=all sign,
  an explicit forward/backward only). Blocking."""
  out = []
  for s in signs:
    if s.needs_geometry:
      approaches = approaches_from_ways(s.direction, geometry.ways_at(s.lat, s.lon), explicit_only=s.all_way)
      if approaches is not None:
        s = replace(s, approaches=approaches)
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
    ahead = StopAhead(s.node_id, along, lateral)
    if s.node_id == prefer_id:
      preferred = ahead
    if best is None or along < best.distance:
      best = ahead
  if preferred is not None and best is not None and preferred.distance <= best.distance + SAME_JUNCTION_M:
    return preferred
  return best


class StopSignMap:
  """The stop signs in the 3 x 3 feature cells around the car, loaded in the background."""

  def __init__(self, features: MapFeatures | None = None, geometry: RoadGeometry | None = None):
    self.features = features or MapFeatures()
    self.geometry = geometry or RoadGeometry()
    self._cells: dict[tuple[int, int], list[StopSign]] = {}
    self._pending: set[tuple[int, int]] = set()
    self._failed: dict[tuple[int, int], float] = {}
    self._lock = threading.Lock()

  def _load(self, key: tuple[int, int]) -> None:
    signs: list[StopSign] | None = None
    try:
      path = self.features.cell_path(*key)
      if path is not None and os.path.isfile(path):
        with open(path) as f:
          signs = stop_signs_from_cell(json.load(f))
    except Exception:
      cloudlog.exception(f"stop sign: loading feature cell {key} failed")
    with self._lock:
      self._pending.discard(key)
      if signs is not None:
        self._cells[key] = signs
        self._failed.pop(key, None)
      else:
        self._failed[key] = time.monotonic()
    if signs is None or not any(s.needs_geometry for s in signs):
      return
    try:
      resolved = resolve_approaches(signs, self.geometry)
    except Exception:
      cloudlog.exception(f"stop sign: road geometry for cell {key} failed")
      return
    with self._lock:
      if key in self._cells:
        self._cells[key] = resolved

  def signs_near(self, lat: float, lon: float) -> list[StopSign]:
    klat, klon = math.floor(lat / FEAT_DEG), math.floor(lon / FEAT_DEG)
    keys = [(klat + i, klon + j) for i in (-1, 0, 1) for j in (-1, 0, 1)]
    out: list[StopSign] = []
    with self._lock:
      for key in keys:
        if key in self._cells:
          out.extend(self._cells[key])
        elif key not in self._pending and time.monotonic() - self._failed.get(key, -RETRY_S) >= RETRY_S:
          self._pending.add(key)
          threading.Thread(target=self._load, args=(key,), daemon=True).start()
      # forget cells far behind so a long drive doesn't grow without bound
      if len(self._cells) > 64:
        for key in [k for k in self._cells if abs(k[0] - klat) > 3 or abs(k[1] - klon) > 3]:
          del self._cells[key]
    return out
