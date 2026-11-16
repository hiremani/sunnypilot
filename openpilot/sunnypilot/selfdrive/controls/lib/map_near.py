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
"""
import math
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
