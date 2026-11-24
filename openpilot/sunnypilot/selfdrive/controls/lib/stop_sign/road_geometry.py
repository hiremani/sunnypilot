"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The ways a map point lies on, from sunnypilot mapd's offline road tiles: the files the webhud serves too
(webhud/maptiles.py MapTiles: <mapd root>/offline/<lat>/<lon>/<minLat>_<minLon>_<maxLat>_<maxLon>, one packed
Cap'n Proto message per 0.25 deg tile). Each Way keeps its OSM id, highway class, bounding box and its nodes'
coordinates *in OSM node order* (road_tile.capnp), which is what an OSM direction=forward|backward is relative to.
The distance along the way from the node to each end is reported too (the nearest-junction rule for untagged signs).
Ways carry coordinates, not node ids, so a sign node is matched to a way node by position (both datasets copy
the same OSM node, so they agree to well under a metre).

Only tiles already on disk are read (mapd downloads them; the webhud's prefetch does too): a 2 x 2 deg cell is a
large download, not something to start from the planner. Reading is blocking and meant for a background thread
(StopSignMap's loader); a tile decodes in well under a second and the last few are kept.
"""
import math
import os
import threading
from collections import OrderedDict
from dataclasses import dataclass

import numpy as np

from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.webhud.maptiles import tile_name

SCHEMA_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "road_tile.capnp")
# osmtile.js HIGHWAY_CLASSES: Way.highway indexes this
HIGHWAY_CLASSES = ("unknown", "motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link", "secondary",
                   "secondary_link", "tertiary", "tertiary_link", "unclassified", "residential", "living_street")
# road importance, lower is more major (the webhud page's state.js RANK): the minor road is the one a stop holds
RANK = {"motorway": 0, "motorway_link": 0, "trunk": 0, "trunk_link": 0, "primary": 1, "primary_link": 1,
        "secondary": 2, "secondary_link": 2, "tertiary": 3, "tertiary_link": 3, "unclassified": 4, "residential": 4,
        "living_street": 4}
NODE_SNAP_M = 1.0          # a sign this close to a way's node sits on that node
M_PER_DEG = 111_320.0
EARTH_R = 6371000.0
MAX_TILES = 2              # decoded tiles kept (a 3 x 3 feature-cell window spans at most 4 tiles, usually 1)
TRAVERSAL_LIMIT_WORDS = 1 << 40


@dataclass(frozen=True)
class WayAtNode:
  """One way through a sign's node."""
  way_id: int
  highway: str
  index: int                       # the node's position along the way
  count: int                       # nodes in the way
  forward_bearing: float | None    # deg, travel arriving at the node in node order (i-1 -> i); None at the first node
  backward_bearing: float | None   # deg, travel arriving against node order (i+1 -> i); None at the last node
  start_m: float | None = None     # m along the way from its first node to this one
  end_m: float | None = None       # m along the way from this node to its last one
  lanes: int = 0                   # OSM lanes (both directions on a two-way road); 0 = not tagged


def bearing_deg(lat0: float, lon0: float, lat1: float, lon1: float) -> float:
  """Compass bearing from point 0 to point 1 (deg, 0 = north, 90 = east); fine over the metres between way nodes."""
  dx = math.radians(lon1 - lon0) * math.cos(math.radians((lat0 + lat1) / 2))
  dy = math.radians(lat1 - lat0)
  return math.degrees(math.atan2(dx, dy)) % 360.0


def distance_m(lat0: float, lon0: float, lat1: float, lon1: float) -> float:
  """Metres between two nearby points (equirectangular, the same approximation as bearing_deg)."""
  dx = math.radians(lon1 - lon0) * math.cos(math.radians((lat0 + lat1) / 2))
  dy = math.radians(lat1 - lat0)
  return math.hypot(dx, dy) * EARTH_R


def _load_schema():
  import capnp
  return capnp.load(SCHEMA_PATH)


class RoadTile:
  """One decoded tile: the Cap'n Proto reader plus a numpy index of the ways' bounding boxes."""

  def __init__(self, data: bytes, schema=None):
    schema = schema or _load_schema()
    self._root = schema.Offline.from_bytes_packed(data, traversal_limit_in_words=TRAVERSAL_LIMIT_WORDS)
    self._ways = self._root.ways
    self._bbox = np.array([(w.minLat, w.minLon, w.maxLat, w.maxLon) for w in self._ways], dtype=np.float64).reshape(-1, 4)

  def ways_at(self, lat: float, lon: float) -> list[WayAtNode]:
    slat = NODE_SNAP_M / M_PER_DEG
    slon = slat / max(math.cos(math.radians(lat)), 0.01)
    b = self._bbox
    near = (b[:, 0] <= lat + slat) & (b[:, 2] >= lat - slat) & (b[:, 1] <= lon + slon) & (b[:, 3] >= lon - slon)
    near |= ~b.any(axis=1)   # a way without a bounding box: check its nodes
    out: list[WayAtNode] = []
    seen: set[int] = set()
    for idx in np.flatnonzero(near):
      w = self._ways[int(idx)]
      if w.id in seen:
        continue
      pts = [(n.latitude, n.longitude) for n in w.nodes]
      hit = None
      for i, (plat, plon) in enumerate(pts):
        if abs(plat - lat) <= slat and abs(plon - lon) <= slon:
          hit = i
          break
      if hit is None:
        continue
      seen.add(w.id)
      n = len(pts)
      fwd = bearing_deg(*pts[hit - 1], *pts[hit]) if hit > 0 else None
      bwd = bearing_deg(*pts[hit + 1], *pts[hit]) if hit < n - 1 else None
      cls = HIGHWAY_CLASSES[w.highway] if w.highway < len(HIGHWAY_CLASSES) else "unknown"
      seg = [distance_m(*pts[i], *pts[i + 1]) for i in range(n - 1)]
      out.append(WayAtNode(int(w.id), cls, hit, n, fwd, bwd, sum(seg[:hit]), sum(seg[hit:]), int(getattr(w, "lanes", 0))))
    return out


class RoadGeometry:
  """The ways at a point, from the road tiles on disk. Blocking: call it from a background thread."""

  def __init__(self, root: str | None = None):
    if root is None:
      from openpilot.common.hardware.hw import Paths
      root = os.path.join(Paths.mapd_root(), "offline")
    self.root = root
    self._tiles: OrderedDict[str, RoadTile | None] = OrderedDict()
    self._lock = threading.Lock()
    self._schema = None

  def tile_path(self, lat: float, lon: float) -> str:
    c_lat, c_lon, name = tile_name(lat, lon)
    return os.path.join(self.root, str(c_lat), str(c_lon), name)

  def _tile(self, path: str) -> RoadTile | None:
    if path in self._tiles:
      self._tiles.move_to_end(path)
      return self._tiles[path]
    if not os.path.isfile(path):
      return None   # not cached as missing: mapd may download it later
    tile = None
    try:
      if self._schema is None:
        self._schema = _load_schema()
      with open(path, "rb") as f:
        tile = RoadTile(f.read(), self._schema)
    except Exception:
      cloudlog.exception(f"stop sign: reading road tile {path} failed")
    self._tiles[path] = tile   # a broken file stays broken: don't retry it every cell
    while len(self._tiles) > MAX_TILES:
      self._tiles.popitem(last=False)
    return tile

  def ways_at(self, lat: float, lon: float) -> list[WayAtNode] | None:
    """The ways with a node at (lat, lon); None when the road tile isn't available."""
    with self._lock:
      tile = self._tile(self.tile_path(lat, lon))
      return None if tile is None else tile.ways_at(lat, lon)
