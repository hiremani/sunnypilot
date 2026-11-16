"""Wet road (wet_road.py): rain from the wipers and traction control, and what each level changes."""
import unittest

from openpilot.sunnypilot.selfdrive.controls.lib import wet_road as wr
from openpilot.sunnypilot.tools.drive_summary import wet_summary

DT = 0.05


class FakeParams:
  def __init__(self, **v):
    self.v = v

  def get_bool(self, k):
    return bool(self.v.get(k, False))

  def get(self, k, return_default=False):
    return self.v.get(k, 0)


def make(**params):
  written = []
  w = wr.WetRoad(params=FakeParams(**({wr.ASSIST_PARAM: True} | params)), put_level=written.append)
  return w, written


def run(w, seconds, wiper=0, tcs=False, pattern=None):
  """Drive for `seconds`; pattern(t) -> wiper value overrides the constant."""
  levels = []
  for i in range(round(seconds / DT)):
    w.update(pattern(i * DT) if pattern else wiper, tcs, DT)
    levels.append(w.level)
  return levels


class TestDetection(unittest.TestCase):
  def test_steady_wipers_make_it_wet_after_30s(self):
    w, _ = make()
    run(w, 25, wiper=30)
    self.assertEqual(w.level, wr.DRY)
    run(w, 10, wiper=30)
    self.assertEqual(w.level, wr.WET)

  def test_a_single_wipe_or_a_washer_squirt_is_not_rain(self):
    w, _ = make()
    run(w, 6, pattern=lambda t: 40 if t < 5 else 0)     # wash: a few wipes over 5 s
    run(w, 120)
    self.assertEqual(w.level, wr.DRY)

  def test_intermittent_wipers_count(self):
    w, _ = make()
    run(w, 60, pattern=lambda t: 20 if (t % 8) < 1.5 else 0)   # a wipe every 8 s, the signal at 0 in between
    self.assertEqual(w.level, wr.WET)

  def test_fast_wipers_or_traction_control_make_it_heavy(self):
    w, _ = make()
    run(w, 40, wiper=30)
    run(w, 12, wiper=60)
    self.assertEqual(w.level, wr.HEAVY)
    w, _ = make()
    run(w, 40, wiper=30)
    run(w, 1, wiper=30, tcs=True)
    self.assertEqual(w.level, wr.HEAVY)

  def test_traction_control_on_a_dry_road_is_not_rain(self):
    w, _ = make()
    run(w, 30, tcs=True)
    self.assertEqual(w.level, wr.DRY)

  def test_it_stays_wet_after_the_rain_stops_then_dries(self):
    w, _ = make()
    run(w, 40, wiper=60)                 # into wet, then heavy
    self.assertEqual(w.level, wr.HEAVY)
    run(w, 100, wiper=30)
    self.assertEqual(w.level, wr.HEAVY)   # fast wiping stopped < HEAVY_OFF_S ago
    run(w, 30, wiper=30)
    self.assertEqual(w.level, wr.WET)
    run(w, 280)                          # wipers off 280 s: the road is still wet
    self.assertEqual(w.level, wr.WET)
    run(w, 30)
    self.assertEqual(w.level, wr.DRY)


class TestApplied(unittest.TestCase):
  def test_shadow_detects_but_changes_nothing(self):
    w, written = make(WetRoadAssist=False)
    run(w, 40, wiper=30)
    self.assertEqual((w.level, w.applied), (wr.WET, wr.DRY))
    self.assertEqual((w.gap_factor(), w.curve_factor(), w.accel_max(), w.speed_offset_mph()), (1.0, 1.0, float("inf"), None))
    self.assertEqual(written, [0])

  def test_each_level_changes_what_the_design_says(self):
    w, written = make()
    run(w, 35, wiper=30)
    self.assertEqual((w.gap_factor(), w.curve_factor(), w.stop_decel(), w.speed_offset_mph(), w.accel_max()),
                     (1.25, 0.85, 0.8, 0.0, 1.2))
    run(w, 12, wiper=60)
    self.assertEqual((w.gap_factor(), w.curve_factor(), w.stop_decel(), w.speed_offset_mph(), w.accel_max()),
                     (1.5, 0.7, 0.7, -5.0, 0.8))
    self.assertEqual(written, [0, 1, 2])   # the level param only on change

  def test_force_holds_wet(self):
    w, _ = make(WetRoadAssist=False, WetRoadForce=True)
    run(w, 2)
    self.assertEqual((w.level, w.applied), (wr.DRY, wr.WET))

  def test_changed_flags_the_tick_it_happens(self):
    w, _ = make()
    flags = []
    for _ in range(round(35 / DT)):
      w.update(30, False, DT)
      flags.append(w.changed)
    self.assertEqual(sum(flags), 1)

  def test_heavy_rain_param_stops_lane_changes_and_turns(self):
    self.assertTrue(wr.lane_changes_and_turns_ok(FakeParams(WetRoadLevel=1)))
    self.assertFalse(wr.lane_changes_and_turns_ok(FakeParams(WetRoadLevel=2)))


class TestSummary(unittest.TestCase):
  def test_minutes_per_level(self):
    s = wet_summary([(60.0, "wet", "wet", 30), (180.0, "heavy", "heavy", 60), (300.0, "wet", "wet", 30)], 600.0)
    self.assertEqual(s, {"wet_min": 7.0, "heavy_min": 2.0, "applied_min": 9.0, "max_wiper": 60})


if __name__ == "__main__":
  unittest.main()
