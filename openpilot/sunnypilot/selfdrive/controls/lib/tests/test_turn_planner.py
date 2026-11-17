"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math
import unittest
from types import SimpleNamespace

from openpilot.sunnypilot.selfdrive.controls.lib import turn_planner as tp

LAT0, LON0 = 37.5, -121.95
DT = 0.05


def _ll(x: float, y: float) -> tuple[float, float]:
  return LAT0 + math.degrees(y / tp.EARTH_R), LON0 + math.degrees(x / (tp.EARTH_R * math.cos(math.radians(LAT0))))


def t_junction(right: bool = True, left: bool = False) -> tp.LocalRoads:
  """A north-south main road with a junction node at the origin and side roads leaving east (right for a car going
  north) and/or west."""
  ways = [(1, "residential", False, [_ll(0, y) for y in (-200, -100, -40, 0, 60, 200)])]
  if right:
    ways.append((2, "residential", False, [_ll(x, 0) for x in (0, 30, 150)]))
  if left:
    ways.append((3, "residential", False, [_ll(-x, 0) for x in (0, 30, 150)]))
  return tp.LocalRoads(LAT0, LON0, ways)


class Car:
  """A kinematic car driving north on the main road's right lane, following the planner's curvature when it has one
  (through the actuator delay and the curvature build-up rate the planner assumes), else driving straight."""

  def __init__(self, planner: tp.TurnPlanner, v: float = 4.0, y0: float = -80.0, x0: float = 2.0):
    self.p, self.v, self.x, self.y, self.h, self.t = planner, v, x0, y0, 0.0, 0.0
    self.k = 0.0
    self.queue = [0.0] * round(tp.ACTUATOR_DELAY_S / DT)
    self.outputs: list[float | None] = []
    self.states: list[str] = []

  def step(self, left=False, right=False, lat_active=True, pressed=False, torque=0.0, model=None) -> float | None:
    lat, lon = _ll(self.x, self.y)
    gps = SimpleNamespace(latitude=lat, longitude=lon, bearingDeg=self.h % 360.0, speed=self.v, hasFix=True,
                          horizontalAccuracy=1.0, unixTimestampMillis=int(self.t * 1000))
    yaw = -self.v * self.k   # yawRate is positive to the left; curvature is positive to the right
    out = self.p.update(DT, self.v, yaw, left, right, lat_active, pressed, gps, torque, model_curvature=model)
    self.queue.append(out if out is not None else 0.0)
    target = self.queue.pop(0)
    rate = tp.curvature_rate(self.v, unwind=abs(target) < abs(self.k)) * DT
    self.k += max(-rate, min(rate, target - self.k))
    self.h += math.degrees(self.v * self.k * DT)
    self.x += self.v * math.sin(math.radians(self.h)) * DT
    self.y += self.v * math.cos(math.radians(self.h)) * DT
    self.t += DT
    self.outputs.append(out)
    self.states.append(self.p.state)
    return out

  def drive(self, seconds: float, **kw) -> None:
    for _ in range(int(seconds / DT)):
      self.step(**kw)


def planner(roads: tp.LocalRoads) -> tp.TurnPlanner:
  return tp.TurnPlanner(road_source=tp.StaticRoads(roads))


class TestJunctionDetection(unittest.TestCase):
  def test_finds_the_branch_on_the_blinker_side(self):
    roads = t_junction(right=True, left=True)
    x, y = 2.0, -20.0
    m = roads.match(x, y, 0.0)
    self.assertIsNotNone(m)
    self.assertEqual(roads.ways[m.way].way_id, 1)
    self.assertGreater(m.lateral, 0.0)   # right of the centreline
    j = roads.junction_ahead(m, x, y, "right")
    self.assertAlmostEqual(j.distance, 20.0, delta=0.5)
    self.assertAlmostEqual(j.branch.change, 90.0, delta=1.0)
    self.assertEqual(j.branch.way_id, 2)
    j = roads.junction_ahead(m, x, y, "left")
    self.assertAlmostEqual(j.branch.change, -90.0, delta=1.0)
    self.assertEqual(j.branch.way_id, 3)

  def test_nothing_beyond_the_lookahead_or_without_a_branch(self):
    roads = t_junction(right=True)
    m = roads.match(2.0, -60.0, 0.0)
    self.assertIsNone(roads.junction_ahead(m, 2.0, -60.0, "right"))   # 60 m away
    m = roads.match(2.0, -20.0, 0.0)
    self.assertIsNone(roads.junction_ahead(m, 2.0, -20.0, "left"))

  def test_driving_the_other_way_flips_the_sides(self):
    roads = t_junction(right=True)
    m = roads.match(-2.0, 20.0, 180.0)   # southbound: the east road is now on the left
    self.assertEqual(m.direction, -1)
    self.assertIsNone(roads.junction_ahead(m, -2.0, 20.0, "right"))
    j = roads.junction_ahead(m, -2.0, 20.0, "left")
    self.assertAlmostEqual(j.branch.change, -90.0, delta=1.0)


class TestTurnPlanner(unittest.TestCase):
  def test_right_turn(self):
    car = Car(planner(t_junction(right=True)))
    car.drive(30.0, right=True)
    curv = [o for o in car.outputs if o is not None]
    self.assertTrue(curv, "never turned")
    self.assertTrue(all(c > 0 for c in curv))   # positive curvature = right on this car
    self.assertLessEqual(max(curv), 1 / car.p.tuning.radius_right_m + 1e-6)
    self.assertIn(tp.ARMED, car.states)
    self.assertEqual(car.p.state, tp.DONE)    # released, and holds off while the blinker stays on
    self.assertEqual(car.p.log.reason, "done")
    self.assertAlmostEqual(car.h, 90.0, delta=8.0)
    self.assertIsNone(car.outputs[-1])
    first = car.outputs.index(curv[0])
    self.assertLess(car.y + 0.0, 5.0)         # the car ended up on the east road, not past the junction
    self.assertGreater(first, 0)

  def test_turn_from_the_stop_line(self):
    # stopped 7 m before the junction node (inside the start distance) with the blinker on, then moving off: it waits
    # while stopped, then turns from there on a tighter arc
    p = planner(t_junction(right=True))
    car = Car(p, v=3.0, y0=-30.0)
    while car.y < -7.0:
      car.step()
    car.v = 0.0
    car.drive(3.0, right=True)
    self.assertEqual(p.state, tp.ARMED)
    self.assertTrue(all(o is None for o in car.outputs))
    car.v = 2.0
    car.drive(15.0, right=True)
    self.assertEqual(p.log.reason, "done")
    self.assertAlmostEqual(car.h, 90.0, delta=8.0)
    self.assertGreater(max(o for o in car.outputs if o is not None), 1 / p.tuning.radius_right_m)   # tighter arc

  def test_left_turn(self):
    car = Car(planner(t_junction(right=False, left=True)))
    car.drive(30.0, left=True)
    curv = [o for o in car.outputs if o is not None]
    self.assertTrue(curv)
    self.assertTrue(all(c < 0 for c in curv))
    self.assertEqual(car.p.log.reason, "done")
    self.assertAlmostEqual(car.h, -90.0, delta=8.0)

  def test_no_blinker_no_turn(self):
    car = Car(planner(t_junction(right=True, left=True)))
    car.drive(30.0)
    self.assertTrue(all(o is None for o in car.outputs))
    self.assertEqual(set(car.states), {tp.IDLE})

  def test_too_fast_no_turn(self):
    car = Car(planner(t_junction(right=True)), v=tp.V_ARM_MAX + 1.0)
    car.drive(20.0, right=True)
    self.assertTrue(all(o is None for o in car.outputs))
    self.assertNotIn(tp.TURNING, car.states)

  def test_no_branch_on_that_side(self):
    car = Car(planner(t_junction(right=True)))
    car.drive(30.0, left=True)
    self.assertTrue(all(o is None for o in car.outputs))
    self.assertNotIn(tp.ARMED, car.states)

  def test_lateral_inactive_no_turn(self):
    car = Car(planner(t_junction(right=True)))
    car.drive(30.0, right=True, lat_active=False)
    self.assertTrue(all(o is None for o in car.outputs))

  def test_no_map_no_turn(self):
    car = Car(tp.TurnPlanner(road_source=SimpleNamespace(get=lambda lat, lon: None)))
    car.drive(30.0, right=True)
    self.assertTrue(all(o is None for o in car.outputs))

  def test_blinker_off_before_the_turn_disarms(self):
    car = Car(planner(t_junction(right=True)))
    while car.p.state != tp.ARMED:
      car.step(right=True)
    car.step()
    self.assertEqual(car.p.state, tp.IDLE)
    car.drive(20.0)
    self.assertTrue(all(o is None for o in car.outputs))

  def test_driver_override_releases(self):
    car = Car(planner(t_junction(right=True)))
    while car.p.state != tp.TURNING:
      car.step(right=True)
    car.drive(0.2, right=True, pressed=True, torque=1.5)   # against the right turn, but not long enough
    self.assertEqual(car.p.state, tp.TURNING)
    car.drive(0.4, right=True, pressed=True, torque=1.5)
    self.assertEqual(car.p.state, tp.DONE)
    self.assertEqual(car.p.log.reason, "driver override")
    car.drive(10.0, right=True)                            # no new turn while that blinker stays on
    self.assertIsNone(car.outputs[-1])
    self.assertNotIn(tp.TURNING, car.states[-200:])

  def test_driver_helping_the_turn_is_not_an_override(self):
    car = Car(planner(t_junction(right=True)))
    while car.p.state != tp.TURNING:
      car.step(right=True)
    car.drive(1.0, right=True, pressed=True, torque=-1.5)
    self.assertEqual(car.p.state, tp.TURNING)

  def test_not_turning_releases(self):
    p = planner(t_junction(right=True))
    car = Car(p)
    while p.state != tp.TURNING:
      car.step(right=True)
    gps = SimpleNamespace(latitude=LAT0, longitude=LON0, bearingDeg=0.0, speed=3.0, hasFix=False, horizontalAccuracy=1.0)
    out = 0.0
    for _ in range(int(3.0 / DT)):   # the car doesn't rotate although commanded (yaw rate stays 0)
      out = p.update(DT, 3.0, 0.0, False, True, True, False, gps)
    self.assertIsNone(out)
    self.assertEqual(p.log.reason, "not turning")

  def test_speeding_up_releases(self):
    p = planner(t_junction(right=True))
    car = Car(p)
    while p.state != tp.TURNING:
      car.step(right=True)
    car.v = tp.V_ABORT + 0.5
    self.assertIsNone(car.step(right=True))
    self.assertEqual(p.log.reason, "too fast")

  def test_radius_respects_lateral_accel(self):
    p = planner(t_junction(right=True))
    for side in ("left", "right"):
      for v in (1.0, 3.0, 5.4, 7.0):
        self.assertLessEqual(v ** 2 / p.radius_for(side, v), tp.A_LAT_MAX + 1e-9)


def crossroads() -> tp.LocalRoads:
  """A north-south road and an east-west road crossing at a junction node at the origin."""
  return tp.LocalRoads(LAT0, LON0, [(1, "residential", False, [_ll(0, y) for y in (-200, -40, 0, 40, 200)]),
                                    (2, "residential", False, [_ll(x, 0) for x in (-200, -40, 0, 40, 200)])])


class TestDriverMadeTurn(unittest.TestCase):
  def _manual_left(self, car: Car) -> None:
    """The driver makes the left turn by hand (steering paused, so the planner is not steering) with the blinker on,
    from the north-bound lane onto the west-bound road. The heading is carried on by the yaw rate at once, but the GPS
    fix lags (~1 s, 4-6 m at turn speed), so the position it reports is still short of the junction node, east of it,
    with the south road now on the left a few metres ahead (Doane St from Gatewood St, 2026-11-14)."""
    while car.y < -9.0:
      car.step(left=True, lat_active=False)
    while car.h > -88.0:
      car.k = -1.0 / 9.0
      car.step(left=True, lat_active=False)
    car.k, car.h = 0.0, -90.0
    car.x, car.y = 3.0, 1.0          # where the lagging fix puts the car

  def test_no_second_turn_after_the_driver_turned(self):
    car = Car(planner(crossroads()))
    self._manual_left(car)
    self.assertGreater(car.x, 0.0)   # still short of the node: the south road looks like a left branch just ahead
    n = len(car.outputs)
    car.drive(4.0, left=True)        # steering resumes, the blinker has not cancelled yet
    self.assertTrue(all(o is None for o in car.outputs[n:]))
    self.assertNotIn(tp.TURNING, car.states[n:])
    self.assertNotIn(tp.ARMED, car.states[n:])

  def test_a_new_blinker_can_turn_again(self):
    car = Car(planner(crossroads()))
    self._manual_left(car)
    car.drive(1.0, left=True)
    car.drive(1.0)                   # blinker off: the driver's turn is over
    self.assertEqual(car.p.state, tp.IDLE)


class TestTurnEnds(unittest.TestCase):
  def test_blinker_off_mid_turn_ends_it(self):
    """Route 000000f6 (2026-11-14, left onto Doane St): the blinker cancelled while the planner was still turning, and
    it went on commanding a hard left onto the straight road."""
    car = Car(planner(t_junction(right=False, left=True)))
    while car.p.state != tp.TURNING or car.p.progress < 0.6 * abs(car.p.planned):
      car.step(left=True)
    car.drive(0.4)                                  # a flash gap: still turning
    self.assertEqual(car.p.state, tp.TURNING)
    car.step(left=True)
    for _ in range(int((tp.BLINKER_OFF_S + 0.1) / DT)):   # stays off, the driver holding the car straight (Doane St)
      car.k = 0.0
      car.queue = [0.0] * len(car.queue)
      car.step()
    self.assertEqual(car.p.log.reason, "blinker off")
    car.drive(5.0)
    self.assertTrue(all(o is None for o in car.outputs[-100:]))

  def test_flashing_blinker_or_early_cancel_keeps_turning(self):
    car = Car(planner(t_junction(right=False, left=True)))
    while car.p.state != tp.TURNING:
      car.step(left=True)
    for _ in range(6):                              # a blinker logged flashing (older carState): 0.35 s on, 0.35 off
      car.drive(0.35, left=True)
      car.drive(0.35)
    self.assertIn(car.p.state, (tp.TURNING, tp.DONE))
    self.assertNotEqual(car.p.log.reason, "blinker off")
    car = Car(planner(t_junction(right=False, left=True)))
    while car.p.state != tp.TURNING:
      car.step(left=True)
    car.drive(2.0)                                  # off right at the start of the turn: keep going
    self.assertNotEqual(car.p.log.reason, "blinker off")

  def test_other_blinker_releases_at_once(self):
    car = Car(planner(t_junction(right=False, left=True)))
    while car.p.state != tp.TURNING:
      car.step(left=True)
    car.step(right=True)
    self.assertEqual(car.p.log.reason, "other blinker")

  def test_turn_counts_from_the_arrival_road(self):
    """The driver turned the car 30 deg by hand before the planner took over: the planner turns the rest, not the
    whole 90 deg again (route 000000f6: 19 deg short all through the turn)."""
    car = Car(planner(t_junction(right=False, left=True)))
    while car.y < -16.0:
      car.step(left=True)
    self.assertEqual(car.p.state, tp.ARMED)
    while car.p.state == tp.ARMED:    # the driver starts the turn by hand, ahead of the planner
      car.k = -1.0 / 9.0
      car.step(left=True)
    self.assertEqual(car.p.state, tp.TURNING)
    self.assertLess(car.h, -10.0)
    car.drive(30.0, left=True)
    self.assertEqual(car.p.log.reason, "done")
    self.assertAlmostEqual(car.h, -90.0, delta=10.0)


class TestStartFromStop(unittest.TestCase):
  def _from_stop(self, stop_start_m: float):
    p = planner(t_junction(right=False, left=True))
    p.tuning = tp.TurnTuning(stop_start_left_m=stop_start_m)
    car = Car(p, v=3.0, y0=-40.0)
    while car.y < -22.0:
      car.step(left=True)
    car.v = 0.0
    car.drive(2.0, left=True)          # stopped at the line, 22 m before the junction node
    self.assertEqual(car.p.state, tp.ARMED)
    car.v = 3.0
    car.step(left=True)
    started_at = car.y if car.p.state == tp.TURNING else None
    car.drive(30.0, left=True)
    return car, started_at

  def test_moving_off_starts_the_turn_where_the_driver_does(self):
    car, started_at = self._from_stop(25.0)
    self.assertIsNotNone(started_at)                 # at once, from the stop line
    self.assertAlmostEqual(car.h, -90.0, delta=10.0)  # and still ends on the branch
    self.assertLess(abs(car.y), 6.0)                  # ... about where the branch runs

  def test_off_by_default(self):
    car, started_at = self._from_stop(0.0)
    self.assertIsNone(started_at)                    # the planner's own start, ~10 m before the node
    self.assertAlmostEqual(car.h, -90.0, delta=10.0)

  def test_not_without_stopping(self):
    p = planner(t_junction(right=False, left=True))
    p.tuning = tp.TurnTuning(stop_start_left_m=25.0)
    car = Car(p, v=3.0, y0=-40.0)
    while car.p.state != tp.TURNING:
      car.step(left=True)
    self.assertGreater(car.y, -15.0)                 # rolling through: the normal start


class TestTurnLog(unittest.TestCase):
  def test_every_state_change_and_ticks_are_logged(self):
    p = planner(t_junction(right=False, left=True))
    events = []
    p.log_event = lambda name, **f: events.append((name, f))
    car = Car(p)
    car.drive(30.0, left=True)
    changes = [(f["prev"], f["state"]) for n, f in events if n == "map_turn"]
    self.assertIn((tp.IDLE, tp.ARMED), changes)
    self.assertIn((tp.ARMED, tp.TURNING), changes)
    self.assertTrue(any(prev == tp.TURNING for prev, _ in changes))
    done = [f for n, f in events if n == "map_turn" and f["prev"] == tp.TURNING][0]
    self.assertEqual(done["reason"], "done")
    self.assertGreater(abs(done["progress_deg"]), 60.0)   # how far the finished turn went, not the reset 0
    self.assertGreater(abs(done["planned_deg"]), 60.0)
    ticks = [f for n, f in events if n == "map_turn_tick"]
    self.assertTrue(any(f["state"] == tp.TURNING and f["curvature"] is not None for f in ticks))
    self.assertTrue(all(f["blinker"] == "L" for f in ticks))

  def test_a_failing_logger_never_breaks_the_turn(self):
    p = planner(t_junction(right=False, left=True))

    def boom(name, **f):
      raise RuntimeError("log down")
    p.log_event = boom
    car = Car(p)
    car.drive(30.0, left=True)
    self.assertEqual(car.p.log.reason, "done")

class TestComfort(unittest.TestCase):
  """The driver's spec (2026-11-17): turns slow, easy and non-jerky, settling into the lane with lane centering."""

  def test_curvature_builds_up_and_unwinds_gradually(self):
    car = Car(planner(t_junction(right=True)))
    car.drive(30.0, right=True)
    curv = [o for o in car.outputs if o is not None]
    self.assertEqual(car.p.log.reason, "done")
    up = tp.comfort_curvature_rate(car.v) * DT
    self.assertLessEqual(curv[0], up + 1e-9)                       # no step at the start
    steps = [b - a for a, b in zip(curv, curv[1:], strict=False)]
    self.assertLessEqual(max(steps), up + 1e-9)
    self.assertGreaterEqual(min(steps), -tp.comfort_curvature_rate(car.v, unwind=True) * DT - 1e-9)
    # the wheel never winds faster than the comfort wheel rate (curvature x steer ratio x wheelbase)
    wheel_rate = max(steps) / DT * tp.STEER_RATIO_X_WHEELBASE
    self.assertLessEqual(math.degrees(wheel_rate), tp.TURN_WHEEL_RATE_DEG_S + 1e-6)

  def test_the_end_blends_into_the_model_lane_centering(self):
    # the model sees the new road straight ahead once the car is round: the hand-back lands on its curvature
    # (a model that steers toward the east road's heading, 0.002 1/m per degree short of it)
    car = Car(planner(t_junction(right=True)))
    model_ks = []
    for _ in range(int(30.0 / DT)):
      model_ks.append(min(0.1, max(-0.1, 0.002 * (90.0 - car.h))))
      car.step(right=True, model=model_ks[-1])
    curv = [o for o in car.outputs if o is not None]
    last = car.outputs.index(curv[-1])
    self.assertEqual(car.p.log.reason, "done")
    self.assertAlmostEqual(curv[-1], model_ks[last], delta=0.005)   # handed back onto the model's own command
    self.assertAlmostEqual(car.h, 90.0, delta=10.0)

  def test_arms_while_slowing_but_waits_for_turn_speed(self):
    p = planner(t_junction(right=True))
    car = Car(p, v=10.0, y0=-60.0)          # 22 mph, 60 m out: armed (lookahead 40 m at this speed) but no command
    car.drive(2.5, right=True)
    self.assertEqual(p.state, tp.ARMED)
    self.assertTrue(all(o is None for o in car.outputs))
    car.v = 4.0                              # slowed to turn speed: the turn happens
    car.drive(20.0, right=True)
    self.assertEqual(p.log.reason, "done")
    self.assertAlmostEqual(car.h, 90.0, delta=10.0)

  def test_speeding_up_out_of_a_finished_turn_counts_as_done(self):
    p = planner(t_junction(right=True))
    car = Car(p)
    while p.state != tp.TURNING or p.progress < 0.8 * abs(p.planned):
      car.step(right=True)
    car.v = tp.V_ABORT + 0.5
    car.step(right=True)
    self.assertEqual(p.log.reason, "done")


def divided(lanes: int = 0, gap: float = 20.0) -> tp.LocalRoads:
  """A side road north (x = 0) onto a divided road drawn as two one-way halves: the near half y = 0 eastbound, the far
  half y = gap westbound, `lanes` each."""
  return tp.LocalRoads(LAT0, LON0, [
    (10, "primary", True, [_ll(x, 0) for x in (-200, -60, 0, 60, 200)], lanes),
    (11, "primary", True, [_ll(x, gap) for x in (200, 60, 0, -60, -200)], lanes),
    (1, "residential", False, [_ll(0, y) for y in (-200, -100, -40, 0, gap)])])


def two_way(lanes: int) -> tp.LocalRoads:
  return tp.LocalRoads(LAT0, LON0, [(1, "residential", False, [_ll(0, y) for y in (-200, -100, -40, 0, 60, 200)]),
                                    (2, "residential", False, [_ll(x, 0) for x in (-150, -30, 0, 30, 150)], lanes)])


def turn(roads, side: str) -> Car:
  car = Car(planner(roads))
  car.drive(30.0, left=side == "left", right=side == "right")
  return car


class TestMultiLane(unittest.TestCase):
  def test_left_onto_a_divided_road_crosses_the_near_half_first(self):
    car = turn(divided(), "left")
    first = next(i for i, o in enumerate(car.outputs) if o is not None)
    self.assertEqual(car.p.log.reason, "done")
    self.assertAlmostEqual(car.h % 360, 270.0, delta=8.0)          # westbound, on the far half
    self.assertGreater(car.y, 10.0)
    ys = []
    c2 = Car(planner(divided()))
    for _ in range(first + 1):
      c2.step(left=True)
      ys.append(c2.y)
    self.assertGreater(ys[-1], 0.0)                                 # no steering before the near half is crossed

  def test_right_onto_a_divided_road_takes_the_near_half(self):
    car = turn(divided(), "right")
    self.assertEqual(car.p.log.reason, "done")
    self.assertAlmostEqual(car.h % 360, 90.0, delta=8.0)
    self.assertLess(car.y, 10.0)

  def test_right_turns_end_in_the_rightmost_lane(self):
    base, four, six = (turn(two_way(n), "right").y for n in (2, 4, 6))
    self.assertAlmostEqual(base - four, tp.LANE_W, delta=0.5)      # one lane further right per extra lane each way
    self.assertAlmostEqual(base - six, 2 * tp.LANE_W, delta=0.5)

  def test_left_turns_end_in_the_nearest_lane(self):
    ends = [turn(two_way(n), "left").y for n in (2, 4, 6)]
    self.assertLess(max(ends) - min(ends), 0.2)                    # the lane next to the centre, whatever the count
    one, three = turn(divided(1), "left").y, turn(divided(3), "left").y
    self.assertAlmostEqual(one - three, tp.LANE_W, delta=0.5)       # the inner lane of the far half, by the median

  def test_the_tuned_case_is_unchanged(self):
    for lanes in (0, 2):
      self.assertEqual(tp.exit_shift(tp.Branch(2, "residential", 90.0, 90.0, False, lanes), "right"), 0.0)
      self.assertEqual(tp.exit_shift(tp.Branch(2, "residential", 270.0, -90.0, False, lanes), "left"), 0.0)

def two_streets(near_y: float = 0.0, far_y: float = 65.0) -> tp.LocalRoads:
  """A north-south main road crossed by two east-west streets (both sides), at near_y and far_y."""
  ys = sorted({-200.0, -100.0, -40.0, near_y, far_y, 200.0})
  ways = [(1, "residential", False, [_ll(0, y) for y in ys])]
  for wid, y in ((2, near_y), (3, far_y)):
    ways.append((wid, "residential", False, [_ll(x, y) for x in (-150, -30, 0, 30, 150)]))
  return tp.LocalRoads(LAT0, LON0, ways)


class TestApproach(unittest.TestCase):
  """Route 00000103 (2026-11-16): the driver signals 60-80 m before the turn, often just before a cross street. At 20
  mph the planner took the cross street 5-18 m ahead (no way to slow down for it) and passed it; the turn the driver
  meant never got the turn assist."""

  def test_cross_street_too_close_to_slow_for_is_skipped(self):
    car = Car(planner(two_streets()), v=8.9, y0=-12.0)
    car.drive(0.2, right=True)
    self.assertEqual(car.p.state, tp.ARMED)
    self.assertEqual(car.p.junction.branch.way_id, 3)
    state, dist = car.p.approach()
    self.assertEqual(state, 1)
    self.assertAlmostEqual(dist, 77.0, delta=3.0)

  def test_slow_car_takes_the_near_street(self):
    car = Car(planner(two_streets()), v=4.0, y0=-25.0)
    car.drive(0.2, right=True)
    self.assertIn(car.p.state, (tp.ARMED, tp.TURNING))
    self.assertEqual(car.p.junction.branch.way_id, 2)

  def test_min_distance(self):
    self.assertEqual(tp.min_junction_distance(4.0, 4.5), 0.0)
    self.assertAlmostEqual(tp.min_junction_distance(8.9, 4.5), (8.9 ** 2 - 4.5 ** 2) / 3.0)

  def test_idle_reports_nothing(self):
    self.assertEqual(planner(two_streets()).approach(), (0, 0.0))

class TestSameStreet(unittest.TestCase):
  """Route 000000fe: Warm Springs Blvd bends right where OSM splits it into another way; the bend is not a turn."""

  def bend(self, other_straight: bool = False, name2: str = "Main St") -> tp.LocalRoads:
    ways = [(1, "primary", False, [_ll(0, y) for y in (-200, -100, 0)], 0, "Main St"),
            (2, "primary", False, [_ll(x, 0) for x in (0, 60, 200)], 0, name2),
            (4, "residential", False, [_ll(0, y) for y in (60, 70, 80)], 0, "Far St"),
            (5, "residential", False, [_ll(x, 70) for x in (200, 300)], 0, "Far St")]
    if other_straight:
      ways.append((3, "residential", False, [_ll(0, y) for y in (0, 100, 200)], 0, "Side St"))
    return tp.LocalRoads(LAT0, LON0, ways)

  def test_the_street_bending_is_not_a_turn(self):
    roads = self.bend()
    m = roads.match(2.0, -20.0, 0.0)
    j = roads.junction_ahead(m, 2.0, -20.0, "right", 100.0)
    self.assertTrue(j is None or j.branch.way_id != 2)

  def test_a_differently_named_road_there_is_a_turn(self):
    roads = self.bend(name2="Other Rd")
    m = roads.match(2.0, -20.0, 0.0)
    self.assertEqual(roads.junction_ahead(m, 2.0, -20.0, "right", 100.0).branch.way_id, 2)

  def test_staying_on_the_street_round_its_corner_is_a_turn(self):
    roads = self.bend(other_straight=True)   # something goes straight on: the street's corner is a real junction
    m = roads.match(2.0, -20.0, 0.0)
    self.assertEqual(roads.junction_ahead(m, 2.0, -20.0, "right", 100.0).branch.way_id, 2)


if __name__ == "__main__":
  unittest.main()
