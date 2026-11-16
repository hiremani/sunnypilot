"""map_near.NearCache: the nearby list the map lookups use gives the same answer as scanning every loaded point."""
import math
import random
import unittest

from openpilot.sunnypilot.selfdrive.controls.lib import map_near, speed_bump as sb
from openpilot.sunnypilot.selfdrive.controls.lib.map_caution import MapPoint, find_point_ahead
from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_map import LOOKAHEAD_M as STOP_LOOKAHEAD_M, StopSignMap

LAT0, LON0 = 37.5, -121.95


def ll(x, y):
  return LAT0 + math.degrees(y / map_near.EARTH_R), LON0 + math.degrees(x / (map_near.EARTH_R * math.cos(math.radians(LAT0))))


class TestNearCache(unittest.TestCase):
  def setUp(self):
    rng = random.Random(7)
    self.points = [MapPoint(i, *ll(rng.uniform(-3000, 3000), rng.uniform(-3000, 3000)), "signal") for i in range(4000)]
    self.points += [MapPoint(10000 + i, *ll(rng.uniform(-3, 3), 140.0 * i), "signal") for i in range(-20, 25)]  # on the road

  def test_same_point_ahead_as_the_full_scan(self):
    cache = map_near.NearCache(150.0 + map_near.REFRESH_M + 10.0)
    fetches = 0

    def fetch():
      nonlocal fetches
      fetches += 1
      return self.points

    n = 0
    for k in range(3000):     # 2.5 km north at 17 m/s, a tick every 0.05 s, with the heading wandering a little
      y = -1500.0 + k * 0.85
      lat, lon = ll(2.0 * math.sin(k / 50), y)
      bearing = 3.0 * math.sin(k / 70)
      near = cache.get(lat, lon, (0, 0), 1, fetch, now=k * 0.05)
      self.assertEqual(find_point_ahead(lat, lon, bearing, near, "signal", 150.0),
                       find_point_ahead(lat, lon, bearing, self.points, "signal", 150.0), k)
      n += 1
    self.assertLess(fetches, n / 20)        # rebuilt every ~30 m, not every tick
    self.assertLess(len(cache._near), 100)  # and the per-tick list is small

  def test_rebuilds_on_a_new_cell_version_or_after_a_while(self):
    cache = map_near.NearCache(200.0)
    calls = []
    lat, lon = ll(0.0, 0.0)
    for args in (((0, 0), 1, 0.0), ((0, 0), 1, 1.0), ((0, 0), 2, 1.1), ((0, 1), 2, 1.2), ((0, 1), 2, 7.0)):
      cache.get(lat, lon, args[0], args[1], lambda: calls.append(1) or [], now=args[2])
    self.assertEqual(len(calls), 4)   # all but the second: same cell, version and place, a second later


class TestMapsUseIt(unittest.TestCase):
  def test_a_cell_loading_shows_up_at_once(self):
    m = StopSignMap(features=type("F", (), {"root": "/nonexistent", "cell_path": lambda *a: None})(), geometry=object())
    lat, lon = ll(0.0, 0.0)
    self.assertEqual(m.signs_near(lat, lon), [])
    sign = type("S", (), {"lat": ll(0, 100)[0], "lon": lon, "node_id": 1})()
    key = (math.floor(lat / 0.05), math.floor(lon / 0.05))
    with m._lock:
      m._cells[key] = [sign]
      m._version += 1
    self.assertEqual(m.signs_near(lat, lon), [sign])
    self.assertGreater(m._near.radius_m, STOP_LOOKAHEAD_M + map_near.REFRESH_M)

  def test_radii_cover_each_lookahead(self):
    bm = sb.SpeedBumpMap(features=type("F", (), {"root": "/nonexistent", "cell_path": lambda *a: None})())
    self.assertGreaterEqual(bm._near.radius_m, sb.LOOKAHEAD_M + map_near.REFRESH_M)


if __name__ == "__main__":
  unittest.main()
