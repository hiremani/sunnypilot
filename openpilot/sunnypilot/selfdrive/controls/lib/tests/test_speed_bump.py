"""Speed Bump Assist (speed_bump.py): map reading, the easing profile, and a closed-loop approach."""
import json
import math
import os
import tempfile
import unittest
from types import SimpleNamespace

from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.controls.lib import speed_bump as sb

LAT0, LON0 = 37.5, -121.95
NOW = 1_800_000_000.0


def ll(x: float, y: float) -> tuple[float, float]:
  return LAT0 + math.degrees(y / sb.EARTH_R), LON0 + math.degrees(x / (sb.EARTH_R * math.cos(math.radians(LAT0))))


class FakeParams:
  def __init__(self, **values):
    self.values = values

  def get_bool(self, key):
    return bool(self.values.get(key, False))


class StaticMap:
  def __init__(self, bumps):
    self.bumps = bumps

  def bumps_near(self, lat, lon):
    return self.bumps


def gps_at(y: float, v: float, bearing: float = 0.0):
  lat, lon = ll(0.0, y)
  return SimpleNamespace(hasFix=True, horizontalAccuracy=3.0, unixTimestampMillis=int(NOW * 1000), latitude=lat,
                         longitude=lon, bearingDeg=bearing, speed=v)


def bump_at(y: float, kind: str = "bump", node_id: int = 7) -> sb.SpeedBump:
  lat, lon = ll(0.0, y)
  return sb.SpeedBump(node_id, lat, lon, kind)


def assist(bumps, shadow=False, **params):
  p = FakeParams(**({sb.PARAM: True, sb.SHADOW_PARAM: True} | params))
  return sb.SpeedBumpAssist(bump_map=StaticMap(bumps), params=p, shadow=shadow)


def drive(a: sb.SpeedBumpAssist, v0: float, y0: float = -200.0, until_y: float = 30.0, gas_at: float | None = None):
  """A car driving north at v0 that follows the assist's target with a 1 s speed loop (the planner) and limited
  deceleration. Returns per-step (y, v, accel, state)."""
  y, v, dt, out = y0, v0, sb.DT_MDL, []
  while y < until_y:
    gas = gas_at is not None and y >= gas_at
    a.update({a.gps_service: gps_at(y, v)}, True, gas, v, 0.0, NOW)
    target = min(a.output_v_target, v0)
    accel = max(-1.5, min(0.5, (target - v) / sb.LAG_S))
    v = max(0.0, v + accel * dt)
    y += v * dt
    out.append((y, v, accel, a.state, a.warn))
  return out


class TestMap(unittest.TestCase):
  def test_kinds_and_speeds(self):
    cell = {"nodes": [[1, 37.5, -121.9, {"traffic_calming": "bump"}], [2, 37.5, -121.9, {"traffic_calming": "hump"}],
                      [3, 37.5, -121.9, {"traffic_calming": "table"}], [4, 37.5, -121.9, {"traffic_calming": "cushion"}],
                      [5, 37.5, -121.9, {"traffic_calming": "rumble_strip"}], [6, 37.5, -121.9, {"traffic_calming": "yes"}],
                      [7, 37.5, -121.9, {"highway": "stop"}], ["bad"]]}
    got = {b.node_id: round(b.v_cross * CV.MS_TO_MPH) for b in sb.bumps_from_cell(cell)}
    self.assertEqual(got, {1: 10, 2: 15, 3: 20, 4: 20})

  def test_local_file_adds_and_ignores(self):
    with tempfile.TemporaryDirectory() as d:
      path = os.path.join(d, sb.LOCAL_FILE)
      with open(path, "w") as f:
        json.dump({"bumps": [{"id": -1, "lat": 37.5, "lon": -121.9, "kind": "hump"},
                             {"id": 5, "lat": 37.5, "lon": -121.9},            # positive id: skipped
                             {"id": -2, "lat": 37.5, "lon": -121.9, "kind": "x"}],
                   "ignore": [6107009436, "junk"]}, f)
      added, ignore = sb.local_bumps(path)
    self.assertEqual([(b.node_id, b.kind) for b in added], [(-1, "hump")])
    self.assertEqual(ignore, {6107009436})
    self.assertEqual(sb.local_bumps("/nonexistent/x.json"), ([], set()))

  def test_only_bumps_on_the_line_of_travel(self):
    lat, lon = ll(0.0, 0.0)
    on_road = bump_at(50.0, node_id=1)
    beside = sb.SpeedBump(2, *ll(15.0, 40.0), "bump")    # a parking lot beside the road
    behind = bump_at(-20.0, node_id=3)
    far = bump_at(200.0, node_id=4)
    ahead = sb.find_bump_ahead(lat, lon, 0.0, [beside, behind, far, on_road])
    self.assertEqual(ahead.node_id, 1)
    self.assertAlmostEqual(ahead.distance, 50.0, delta=0.5)
    self.assertIsNone(sb.find_bump_ahead(lat, lon, 90.0, [on_road]))   # heading east: it's on a crossing road


class TestProfile(unittest.TestCase):
  def test_lift_off_from_25_mph_starts_about_110_m_out(self):
    d = sb.easing_distance(25 * CV.MPH_TO_MS, 10 * CV.MPH_TO_MS)
    self.assertGreater(d, 100.0)
    self.assertLess(d, sb.LOOKAHEAD_M)

  def test_target_reaches_the_crossing_speed_at_the_bump(self):
    v_cross = 10 * CV.MPH_TO_MS
    self.assertAlmostEqual(sb.easing_v_target(0.0, v_cross, 5.0), v_cross)


class TestApproach(unittest.TestCase):
  def test_eases_down_to_each_kind_without_braking(self):
    for kind, mph in (("bump", 10), ("hump", 15), ("table", 20)):
      a = assist([bump_at(0.0, kind)])
      run = drive(a, 25 * CV.MPH_TO_MS)
      v_at = min(v for y, v, *_ in run if -1.0 <= y <= 1.0)
      self.assertLess(v_at * CV.MS_TO_MPH, mph + 1.5, kind)
      self.assertGreater(v_at * CV.MS_TO_MPH, mph - 2.0, kind)
      self.assertGreater(min(acc for _, _, acc, *_ in run), -sb.A_EASE - 0.05, kind)   # a lift-off, not braking
      self.assertIn(sb.SpeedBumpState.crossing, [s for *_, s, _ in run])
      self.assertEqual(run[-1][3], sb.SpeedBumpState.idle)                           # done once past it

  def test_slower_than_the_crossing_speed_does_nothing(self):
    a = assist([bump_at(0.0)])
    run = drive(a, 8 * CV.MPH_TO_MS)
    self.assertFalse(any(w for *_, w in run))
    self.assertGreater(min(v for _, v, *_ in run), 7.9 * CV.MPH_TO_MS)

  def test_gas_hands_the_bump_to_the_driver(self):
    a = assist([bump_at(0.0)])
    run = drive(a, 25 * CV.MPH_TO_MS, gas_at=-40.0)
    after = [s for y, _, _, s, _ in run if y > -39.0]
    self.assertNotIn(sb.SpeedBumpState.easing, after)
    self.assertIn(7, a.done_ids)

  def test_found_late_uses_light_braking(self):
    a = assist([bump_at(0.0)])
    run = drive(a, 25 * CV.MPH_TO_MS, y0=-85.0)        # comes into view at 85 m: too close for a lift-off alone
    self.assertEqual(a.a_plan, sb.A_MAX)
    self.assertGreater(min(acc for _, _, acc, *_ in run), -sb.A_MAX - 0.05)
    self.assertLess(min(v for y, v, *_ in run if -1.0 <= y <= 1.0) * CV.MS_TO_MPH, 11.5)

  def test_too_close_is_left_to_the_driver(self):
    a = assist([bump_at(0.0)])
    run = drive(a, 25 * CV.MPH_TO_MS, y0=-25.0)        # the bump appears 25 m out at 25 mph
    self.assertTrue(all(s != sb.SpeedBumpState.easing for *_, s, _ in run))
    self.assertGreater(min(acc for _, _, acc, *_ in run), -0.01)

  def test_warns_once_per_bump(self):
    a = assist([bump_at(0.0)], shadow=True)
    run = drive(a, 25 * CV.MPH_TO_MS)
    warn_y = [y for y, *_, w in run if w]
    self.assertEqual(len(warn_y), 1)
    self.assertLess(-warn_y[0], 75.0)     # within ~6 s at 25 mph
    self.assertGreater(-warn_y[0], 30.0)

  def test_off_unless_its_setting_is_on(self):
    a = sb.SpeedBumpAssist(bump_map=StaticMap([bump_at(0.0)]), params=FakeParams())
    run = drive(a, 25 * CV.MPH_TO_MS)
    self.assertTrue(all(s == sb.SpeedBumpState.disabled for *_, s, _ in run))
    shadow_alert_only = sb.SpeedBumpAssist(bump_map=StaticMap([bump_at(0.0)]), params=FakeParams(SpeedBumpAlert=True),
                                           shadow=True)
    self.assertTrue(shadow_alert_only.enabled)


if __name__ == "__main__":
  unittest.main()
