"""Map caution (map_caution.py): no accelerating toward a mapped light, crosswalk speed on narrow streets."""
import math
import unittest
from types import SimpleNamespace

from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.controls.lib import map_caution as mc

LAT0, LON0 = 37.5, -121.95
NOW = 1_800_000_000.0


def ll(x, y):
  return LAT0 + math.degrees(y / mc.EARTH_R), LON0 + math.degrees(x / (mc.EARTH_R * math.cos(math.radians(LAT0))))


class FakeParams:
  def __init__(self, **v):
    self.v = v

  def get_bool(self, k):
    return bool(self.v.get(k, False))


class StaticMap:
  def __init__(self, points):
    self.points = points

  def points_near(self, lat, lon):
    return self.points


def point(y, kind, node_id=1, x=0.0):
  return mc.MapPoint(node_id, *ll(x, y), kind)


def sm_at(y, v, svc):
  lat, lon = ll(0.0, y)
  return {svc: SimpleNamespace(hasFix=True, horizontalAccuracy=3.0, unixTimestampMillis=int(NOW * 1000), latitude=lat,
                               longitude=lon, bearingDeg=0.0, speed=max(v, 2.5))}


def caution(points, **params):
  c = mc.MapCaution(point_map=StaticMap(points), params=FakeParams(**({mc.SIGNAL_PARAM: True, mc.CROSSWALK_PARAM: True} | params)))
  c._read_params()
  return c


class TestPoints(unittest.TestCase):
  def test_cell_kinds(self):
    cell = {"nodes": [[1, 37.5, -121.9, {"highway": "traffic_signals"}],
                      [2, 37.5, -121.9, {"highway": "crossing", "crossing": "marked"}],
                      [3, 37.5, -121.9, {"highway": "crossing", "crossing": "unmarked"}],
                      [4, 37.5, -121.9, {"highway": "crossing", "crossing:markings": "no"}],
                      [5, 37.5, -121.9, {"highway": "stop"}], ["junk"]]}
    self.assertEqual([(p.node_id, p.kind) for p in mc.points_from_cell(cell)], [(1, "signal"), (2, "crosswalk")])


class TestSignalCoast(unittest.TestCase):
  def test_no_accelerating_toward_a_light_ahead(self):
    c = caution([point(100.0, "signal")])
    for y, expect in ((-100.0, False), (40.0, False), (50.0, True), (70.0, True), (80.0, False)):   # 13.4 m/s: 25..53.6 m out
      c.update(sm_at(y, 13.4, c.gps_service), True, False, 13.4, 0.5, False, NOW)
      self.assertEqual(c.signal_coast, expect, y)
      self.assertEqual(c.accel_cap(), 0.0 if expect else float("inf"))

  def test_pulling_away_from_the_light_is_never_held(self):
    c = caution([point(15.0, "signal")])   # stopped at the line, the signal node mid-junction 15 m on
    for v in (0.0, 2.0, 4.0, 6.0, 8.0):
      c.update(sm_at(0.0, v, c.gps_service), True, False, v, 1.0, False, NOW)
      self.assertFalse(c.signal_coast, v)

  def test_off_without_its_setting_or_with_the_gas(self):
    c = caution([point(100.0, "signal")], TrafficSignalCoast=False)
    c.update(sm_at(60.0, 13.4, c.gps_service), True, False, 13.4, 0.5, False, NOW)
    self.assertFalse(c.signal_coast)
    c = caution([point(100.0, "signal")])
    c.update(sm_at(60.0, 13.4, c.gps_service), True, True, 13.4, 0.5, False, NOW)   # driver on the gas
    self.assertFalse(c.signal_coast)

  def test_a_light_on_another_road_is_ignored(self):
    c = caution([point(60.0, "signal", x=20.0)])
    c.update(sm_at(0.0, 13.4, c.gps_service), True, False, 13.4, 0.5, False, NOW)
    self.assertFalse(c.signal_coast)


class TestCrosswalk(unittest.TestCase):
  def test_only_on_narrow_streets(self):
    c = caution([point(30.0, "crosswalk")])
    c.update(sm_at(0.0, 7.0, c.gps_service), True, False, 7.0, 0.0, False, NOW)
    self.assertFalse(c.crosswalk_caution)
    c.update(sm_at(0.0, 7.0, c.gps_service), True, False, 7.0, 0.0, True, NOW)
    self.assertTrue(c.crosswalk_caution)
    self.assertAlmostEqual(c.v_target, mc.CROSSWALK_MPH * CV.MPH_TO_MS)
    self.assertEqual(c.a_target, mc.CROSSWALK_ACCEL)
    c.update(sm_at(-50.0, 7.0, c.gps_service), True, False, 7.0, 0.0, True, NOW)   # 80 m out: not yet
    self.assertFalse(c.crosswalk_caution)
    self.assertEqual(c.v_target, float("inf"))


if __name__ == "__main__":
  unittest.main()
