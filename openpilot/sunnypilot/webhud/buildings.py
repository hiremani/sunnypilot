"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The map's buildings for the page (Oceanscape static/js/buildings.js): GET /map/buildings/<klat>/<klon>, a BLD_DEG
square cell named by its south-west corner in units of BLD_DEG, answered with the cell's Overpass answer as it came
(building ways and simple multipolygons with their outlines; the page parses and extrudes them). The same cells,
query and answer as the Pulse app's MapFeatures.kt and the HUD's serve.py, so a page served from the comma draws the
same buildings as one served from the car's screen.

Each cell is fetched once and kept under <mapd root>/buildings/ as `<klat>_<klon>.json`. The folder is bounded
(CACHE_MAX_BYTES): past it, the cells read longest ago go first. Nothing is fetched while the car moves (the comma's
network then is usually a phone's), and one Overpass request runs at a time, politely.
"""
import math
import os
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from collections.abc import Callable

from openpilot.common.hardware.hw import Paths
from openpilot.common.swaglog import cloudlog

BLD_DEG = 0.01                   # static/js/buildgeom.js BLD_DEG, MapFeatures.kt BLD_DEG, serve.py BLD_DEG
BUILDINGS_QUERY = ('[out:json][timeout:90];(way["building"]({bbox});relation["building"]["type"="multipolygon"]({bbox}););' +
                   'out tags geom qt;')
OVERPASS_URL = "https://overpass-api.de/api/interpreter"
OVERPASS_TIMEOUT_S = 150.0
OVERPASS_TRIES = 3
OVERPASS_RETRY_S = 3.0
RETRY_S = 60.0                   # after a failed cell
CACHE_MAX_BYTES = 200 * 1024 * 1024   # the comma's storage is shared with the drives: keep the buildings modest
USER_AGENT = "sunnypilot-webhud/1.0 (map buildings; github.com/sunnypilot/sunnypilot)"
PREFETCH_MAX_KM = 10.0           # around the car: a 10 km radius is already ~300 cells
PREFETCH_PAUSE_S = 1.0


def cell_bbox(klat: int, klon: int) -> str:
  s, w = klat * BLD_DEG, klon * BLD_DEG
  return f"{s:.4f},{w:.4f},{s + BLD_DEG:.4f},{w + BLD_DEG:.4f}"


def valid_cell(klat: int, klon: int) -> bool:
  return -9000 <= klat < 9000 and -18000 <= klon < 18000


def parse_path(path: str) -> tuple[int, int] | None:
  """`/map/buildings/<klat>/<klon>` -> (klat, klon), or None."""
  parts = path.split("/")
  if len(parts) != 5 or parts[1] != "map" or parts[2] != "buildings":
    return None
  try:
    klat, klon = int(parts[3]), int(parts[4])
  except ValueError:
    return None
  return (klat, klon) if valid_cell(klat, klon) else None


def cells_within(lat: float, lon: float, radius_m: float) -> list[tuple[int, int]]:
  """The BLD_DEG cells whose box comes within radius_m of the point, nearest first."""
  m_lat = 111_320.0
  m_lon = m_lat * max(0.05, math.cos(math.radians(lat)))
  d_lat, d_lon = radius_m / m_lat, radius_m / m_lon
  out = []
  for klat in range(math.floor((lat - d_lat) / BLD_DEG), math.floor((lat + d_lat) / BLD_DEG) + 1):
    for klon in range(math.floor((lon - d_lon) / BLD_DEG), math.floor((lon + d_lon) / BLD_DEG) + 1):
      if not valid_cell(klat, klon):
        continue
      s, w = klat * BLD_DEG, klon * BLD_DEG
      dy = (max(s, min(lat, s + BLD_DEG)) - lat) * m_lat
      dx = (max(w, min(lon, w + BLD_DEG)) - lon) * m_lon
      d = math.hypot(dx, dy)
      if d <= radius_m:
        out.append((d, klat, klon))
  out.sort()
  return [(klat, klon) for _, klat, klon in out]


def overpass_fetch(query: str) -> bytes:
  """Overpass, with a few short retries while it is busy (429, or a 502/503/504 from its proxy)."""
  req = urllib.request.Request(OVERPASS_URL, data=urllib.parse.urlencode({"data": query}).encode(),
                               headers={"User-Agent": USER_AGENT, "Accept": "application/json",
                                        "Content-Type": "application/x-www-form-urlencoded"})
  for attempt in range(OVERPASS_TRIES):
    try:
      with urllib.request.urlopen(req, timeout=OVERPASS_TIMEOUT_S) as r:
        return r.read()
    except urllib.error.HTTPError as e:
      if e.code not in (429, 502, 503, 504) or attempt == OVERPASS_TRIES - 1:
        raise
      time.sleep(OVERPASS_RETRY_S)
  raise OSError("overpass: no answer")


class MapBuildings:
  def __init__(self, root: str | None = None, download: bool = True, fetch: Callable[[str], bytes] = overpass_fetch,
               max_bytes: int = CACHE_MAX_BYTES):
    self.root = root or os.path.join(Paths.mapd_root(), "buildings")
    self.download = download
    self.fetch = fetch
    self.max_bytes = max_bytes
    # set by the server: False while the car moves (no fetching then); cells on disk are served regardless
    self.allow_fetch: Callable[[], bool] = lambda: True
    self._lock = threading.Lock()
    self._failed: dict[tuple[int, int], float] = {}

  def _path(self, klat: int, klon: int) -> str:
    return os.path.join(self.root, f"{klat}_{klon}.json")

  def have(self, klat: int, klon: int) -> bool:
    return os.path.isfile(self._path(klat, klon))

  def cell_path(self, klat: int, klon: int) -> str | None:
    """The cell's file, fetching it first when it isn't on disk; None when it can't be had now."""
    if not valid_cell(klat, klon):
      raise ValueError("not a building cell")
    path = self._path(klat, klon)
    if os.path.isfile(path):
      try:
        os.utime(path)   # read now: the last to go when the folder is trimmed
      except OSError:
        pass
      return path
    return path if self.ensure(klat, klon) else None

  def ensure(self, klat: int, klon: int) -> bool:
    """Fetch the cell unless it's on disk. True when it is."""
    if self.have(klat, klon):
      return True
    if not self.download or not self._may_fetch():
      return False
    with self._lock:   # one Overpass request at a time
      if self.have(klat, klon):
        return True
      if time.monotonic() - self._failed.get((klat, klon), -1e9) < RETRY_S:
        return False
      if not self._may_fetch():
        return False
      try:
        cloudlog.info(f"webhud: fetching buildings {klat}/{klon}")
        body = self.fetch(BUILDINGS_QUERY.replace("{bbox}", cell_bbox(klat, klon)))
        if not body or body.lstrip()[:1] != b"{":
          raise ValueError("not a JSON answer")
        os.makedirs(self.root, exist_ok=True)
        path = self._path(klat, klon)
        with open(path + ".part", "wb") as f:
          f.write(body)
        os.replace(path + ".part", path)
      except Exception:
        cloudlog.exception(f"webhud: buildings {klat}/{klon} failed")
        self._failed[(klat, klon)] = time.monotonic()
        return False
      self._trim()
    return True

  def _may_fetch(self) -> bool:
    try:
      return bool(self.allow_fetch())
    except Exception:
      return False

  def _trim(self) -> None:
    """Keep the folder under max_bytes: drop the cells read longest ago."""
    try:
      entries = []
      for name in os.listdir(self.root):
        if not name.endswith(".json"):
          continue
        st = os.stat(os.path.join(self.root, name))
        entries.append((st.st_mtime, st.st_size, name))
    except OSError:
      return
    total = sum(e[1] for e in entries)
    for _, size, name in sorted(entries):
      if total <= self.max_bytes:
        break
      try:
        os.remove(os.path.join(self.root, name))
        total -= size
      except OSError:
        pass

  def prefetch(self, lat: float, lon: float, radius_km: float, keep_going: Callable[[], bool] = lambda: True,
               pause_s: float = PREFETCH_PAUSE_S) -> tuple[int, int]:
    """Fetch the cells within radius_km of the point, nearest first, until keep_going() says stop.
    Returns (cells, errors). Cells already on disk cost nothing, so a stopped run resumes where it was."""
    radius_km = max(0.0, min(PREFETCH_MAX_KM, float(radius_km)))
    errors = 0
    cells = cells_within(lat, lon, radius_km * 1000.0)
    for klat, klon in cells:
      if not keep_going():
        break
      if self.have(klat, klon):
        continue
      if not self._may_fetch():
        break
      if not self.ensure(klat, klon):
        errors += 1
      time.sleep(pause_s)
    return len(cells), errors

  def ready(self, lat: float, lon: float, radius_km: float) -> tuple[int, int]:
    """(cells within the radius, of them on disk)."""
    cells = cells_within(lat, lon, max(0.0, min(PREFETCH_MAX_KM, float(radius_km))) * 1000.0)
    return len(cells), sum(1 for c in cells if self.have(*c))


class AreaDownload:
  """One "download the buildings around here" at a time, in the background (POST /map/buildings/area). A new
  request replaces the one running; status() says how far it got."""

  def __init__(self, buildings: MapBuildings):
    self.buildings = buildings
    self._lock = threading.Lock()
    self._want: tuple[float, float, float] | None = None
    self._busy = False
    self._errors = 0
    self._thread: threading.Thread | None = None
    self.pause_s = PREFETCH_PAUSE_S

  def start(self, lat: float, lon: float, radius_km: float) -> dict:
    if not (-90 <= lat <= 90 and -180 <= lon <= 180) or not math.isfinite(float(radius_km)):
      raise ValueError("bad position or radius")
    want = (float(lat), float(lon), max(0.0, min(PREFETCH_MAX_KM, float(radius_km))))
    with self._lock:
      self._want = want
      if self._thread is None or not self._thread.is_alive():
        self._thread = threading.Thread(target=self._run, name="webhud-buildings-area", daemon=True)
        self._thread.start()
    return self.status()

  def _run(self) -> None:
    while True:
      with self._lock:
        want = self._want
        self._busy = True
        self._errors = 0
      _, errors = self.buildings.prefetch(*want, keep_going=lambda w=want: self._want == w, pause_s=self.pause_s)
      with self._lock:
        self._errors = errors
        if self._want == want:
          self._busy = False
          return

  def status(self) -> dict:
    with self._lock:
      want, busy, errors = self._want, self._busy, self._errors
    if want is None:
      return {"center": None, "radius_km": None, "cells": 0, "ready": 0, "busy": False, "errors": 0}
    cells, ready = self.buildings.ready(*want)
    return {"center": [round(want[0], 5), round(want[1], 5)], "radius_km": want[2], "cells": cells, "ready": ready,
            "busy": busy, "errors": errors}
