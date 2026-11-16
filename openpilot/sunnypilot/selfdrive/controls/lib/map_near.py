"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The map points close to the car, kept between planner ticks.

The stop sign, speed bump and map caution maps each hold every point in the 3 x 3 feature cells around the car (about
16 x 13 km). Scanning all of them every 50 ms tick cost plannerd up to 45 ms a tick on drive 000000fa (System Lagging,
LKAS fault). NearCache keeps only the points within radius_m of where the car was, and rebuilds that list when the car
has moved refresh_m, changes cell, a cell loads or is dropped, or every rebuild_s. As long as radius_m is at least the
caller's lookahead + refresh_m, the nearest point ahead is the same as from the full list.

CellMap is the shared base of those maps: cells, a bucket index, background loading (map_worker.py on the device).
"""
import math
import threading
import time

EARTH_R = 6371000.0
REFRESH_M = 30.0
REBUILD_S = 5.0


class NearCache:
  def __init__(self, radius_m: float, refresh_m: float = REFRESH_M, rebuild_s: float = REBUILD_S):
    self.radius_m = radius_m
    self.refresh_m = refresh_m
    self.rebuild_s = rebuild_s
    self._key = None             # (cell, version) of the cached list
    self._center = (0.0, 0.0)
    self._built = -math.inf
    self._near: list = []

  def get(self, lat: float, lon: float, cell, version: int, fetch_all, now: float | None = None) -> list:
    """The points within radius_m. fetch_all() returns every loaded point (and starts loading missing cells); it is
    called only on a rebuild."""
    now = time.monotonic() if now is None else now
    cos_lat = math.cos(math.radians(lat))
    moved = math.hypot(math.radians(lon - self._center[1]) * cos_lat, math.radians(lat - self._center[0])) * EARTH_R
    if (cell, version) == self._key and moved < self.refresh_m and now - self._built < self.rebuild_s:
      return self._near
    r_lat = math.degrees(self.radius_m / EARTH_R)
    r_lon = r_lat / max(cos_lat, 1e-6)
    self._near = [p for p in fetch_all() if abs(p.lat - lat) <= r_lat and abs(p.lon - lon) <= r_lon]
    self._key = (cell, version)
    self._center = (lat, lon)
    self._built = now
    return self._near


GRID_DEG = 0.01   # index buckets (~1.1 x 0.9 km) inside the 0.05 deg feature cells


class CellMap:
  """Items (anything with .lat / .lon) from the 3 x 3 feature cells around the car, indexed in GRID_DEG buckets.

  A subclass gives KIND ("stops", "bumps", "points") and _compute(key, cell=None) -> list | None, the blocking read of
  one cell (None: not available, retried after RETRY_S). Cells load:
  * on the device (the default): in the map worker process (map_worker.py), never in the calling process, so the
    planner's own process only ever stores the finished list;
  * with use_worker=False (tests, tools, the worker itself): on a thread, or inline through _load(key).
  near(lat, lon) is the per-tick call: the items within the NearCache radius, rebuilt only every REFRESH_M or so from the
  few buckets around the car."""
  KIND = ""
  RETRY_S = 60.0
  MAX_CELLS = 64

  def __init__(self, radius_m: float, use_worker: bool = False):
    self._cells: dict[tuple[int, int], list] = {}
    self._grid: dict[tuple[int, int], dict[tuple[int, int], list]] = {}   # bucket -> {cell key: items}
    self._cell_buckets: dict[tuple[int, int], set[tuple[int, int]]] = {}
    self._pending: set[tuple[int, int]] = set()
    self._failed: dict[tuple[int, int], float] = {}
    self._lock = threading.Lock()
    self._version = 0
    self._near = NearCache(radius_m)
    self._worker = None
    if use_worker:
      from openpilot.sunnypilot.selfdrive.controls.lib.map_worker import MapWorker
      self._worker = MapWorker.shared()

  def _compute(self, key: tuple[int, int], cell: dict | None = None) -> list | None:
    raise NotImplementedError

  def _load(self, key: tuple[int, int]) -> None:
    """Read one cell inline (blocking)."""
    try:
      items = self._compute(key)
    except Exception:
      from openpilot.common.swaglog import cloudlog
      cloudlog.exception(f"map {self.KIND}: loading feature cell {key} failed")
      items = None
    self._store(key, items)

  def _start_load(self, key: tuple[int, int]) -> None:
    if self._worker is not None:
      self._worker.request_cell(key, self.KIND, lambda items, key=key: self._store(key, items))
    else:
      threading.Thread(target=self._load, args=(key,), daemon=True).start()

  def _drop(self, key: tuple[int, int]) -> None:
    # under self._lock
    for b in self._cell_buckets.pop(key, ()):
      by_cell = self._grid.get(b)
      if by_cell is not None:
        by_cell.pop(key, None)
        if not by_cell:
          del self._grid[b]
    self._cells.pop(key, None)

  def _store(self, key: tuple[int, int], items: list | None) -> None:
    with self._lock:
      self._pending.discard(key)
      if items is None:
        self._failed[key] = time.monotonic()
        return
      self._drop(key)
      self._cells[key] = items
      self._failed.pop(key, None)
      buckets: dict[tuple[int, int], list] = {}
      for p in items:
        buckets.setdefault((math.floor(p.lat / GRID_DEG), math.floor(p.lon / GRID_DEG)), []).append(p)
      for b, ps in buckets.items():
        self._grid.setdefault(b, {})[key] = ps
      self._cell_buckets[key] = set(buckets)
      self._version += 1

  def near(self, lat: float, lon: float) -> list:
    from openpilot.sunnypilot.webhud.maptiles import FEAT_DEG
    cell = (math.floor(lat / FEAT_DEG), math.floor(lon / FEAT_DEG))
    return self._near.get(lat, lon, cell, self._version, lambda: self._gather(lat, lon, cell))

  def _gather(self, lat: float, lon: float, cell: tuple[int, int]) -> list:
    """Start loading missing cells around the car; the loaded items in the buckets the NearCache radius touches."""
    klat, klon = cell
    to_load = []
    out: list = []
    r_lat = math.degrees(self._near.radius_m / EARTH_R)
    r_lon = r_lat / max(math.cos(math.radians(lat)), 1e-6)
    with self._lock:
      now = time.monotonic()
      for key in [(klat + i, klon + j) for i in (-1, 0, 1) for j in (-1, 0, 1)]:
        if key not in self._cells and key not in self._pending and now - self._failed.get(key, -math.inf) >= self.RETRY_S:
          self._pending.add(key)
          to_load.append(key)
      if len(self._cells) > self.MAX_CELLS:   # forget cells far behind so a long drive doesn't grow without bound
        for key in [k for k in self._cells if abs(k[0] - klat) > 3 or abs(k[1] - klon) > 3]:
          self._drop(key)
        self._version += 1
      for blat in range(math.floor((lat - r_lat) / GRID_DEG), math.floor((lat + r_lat) / GRID_DEG) + 1):
        for blon in range(math.floor((lon - r_lon) / GRID_DEG), math.floor((lon + r_lon) / GRID_DEG) + 1):
          for ps in self._grid.get((blat, blon), {}).values():
            out.extend(ps)
    for key in to_load:
      self._start_load(key)
    return out
