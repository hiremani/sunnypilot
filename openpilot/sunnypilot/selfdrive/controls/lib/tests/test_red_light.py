"""Red Light Assist (red_light.py): lift off and stop for a red light the car's camera reports."""
import unittest
from types import SimpleNamespace

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.controls.lib import red_light as rl
from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign import stop_sign_controller as ssc
from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.tests.test_stop_sign import FakeParams, lead_at, model

MPH = 0.44704


def line_plan(v0, line_m, decel=1.5):
  """A model plan that holds v0 and then brakes at `decel` to stop at the line (as the driving model plans a stop it
  has seen: the stop is in its 10 s plan once the line is close enough at this speed)."""
  from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.tests.test_stop_sign import T_IDXS
  brake_d = v0 * v0 / (2 * decel)
  vel, pos = [], []
  for t in T_IDXS:
    cruise_t = max(line_m - brake_d, 0.0) / max(v0, 0.1)
    if t <= cruise_t:
      v, x = v0, v0 * t
    else:
      tb = t - cruise_t
      d_eff = min(decel, v0 * v0 / (2 * max(line_m, 0.1))) if line_m < brake_d else decel
      v = max(v0 - d_eff * tb, 0.0)
      x = min(line_m, v0 * cruise_t + v0 * tb - 0.5 * d_eff * tb * tb) if v > 0 else line_m
    vel.append(v)
    pos.append(x)
  return SimpleNamespace(action=SimpleNamespace(shouldStop=False), velocity=SimpleNamespace(x=vel), position=SimpleNamespace(x=pos))


def assist(on=True):
  return rl.RedLightAssist(params=FakeParams({rl.PARAM: on}))


class Drive:
  """A car driving toward a light whose signal head is `head_m` from the start, following the assist's speed target
  through the planner's cruise loop (a = clip(target - v, -1.2, 0)); holds its speed when there is none."""

  def __init__(self, a, v0, head_m, color=rl.RED, line_m=None, lead=None, model_stops=False):
    self.a, self.v, self.pos, self.head, self.color = a, v0, 0.0, head_m, color
    self.line = line_m          # where the stop line really is (m from the start): the model's plan stops there
    self.lead, self.model_stops = lead, model_stops
    self.acc, self.targets, self.states = [], [], []
    self.gas = False

  def step(self):
    d = self.head - self.pos
    seen = self.color != rl.NONE and d > 0
    sm = {"modelV2": model()}
    if self.model_stops and self.line is not None and self.line - self.pos > 0.5:
      sm["modelV2"] = line_plan(max(self.v, 1.0), self.line - self.pos)
    if self.lead is not None:
      sm["radarState"] = SimpleNamespace(leadOne=lead_at(self.lead))
    self.a.update(sm, True, self.gas, self.v, 0.0, self.color if seen else rl.NONE, d if seen else 0.0)
    t = self.a.output_v_target
    acc = 0.0 if t == V_CRUISE_UNSET else max(min(t - self.v, 0.0), -1.2)
    self.v = max(self.v + acc * DT_MDL, 0.0)
    if self.v < 0.05:
      self.v = 0.0
    self.pos += self.v * DT_MDL
    self.acc.append(acc)
    self.targets.append(t)
    self.states.append(self.a.state)

  def run(self, seconds):
    for _ in range(int(seconds / DT_MDL)):
      self.step()
    return self


class TestRedLight(unittest.TestCase):
  def test_stops_for_a_red_light_before_the_junction(self):
    for mph in (20, 30):
      d = Drive(assist(), mph * MPH, 260.0).run(60)
      self.assertEqual(d.a.state, rl.RedLightState.stopped, mph)
      self.assertEqual(d.v, 0.0)
      short_of_head = d.head - d.pos
      # at the setback's stop point, give or take the profile's buffer: never nearer the head than the setback
      self.assertGreaterEqual(short_of_head, rl.SETBACK_M, mph)
      self.assertLessEqual(short_of_head, rl.SETBACK_M + ssc.STOP_BUFFER_M + 3.0, mph)
      self.assertGreaterEqual(min(d.acc), -1.2)
      # gradual: it lifts off before it brakes, and the stop ends softly
      first = next(i for i, a in enumerate(d.acc) if a < -0.05)
      self.assertGreater(d.acc[first], -0.6, mph)
      moving = [a for a, s in zip(d.acc, d.states, strict=True) if s == rl.RedLightState.approach]
      self.assertGreater(min(moving[-30:]), -0.8, mph)                      # the last 1.5 s before standstill

  def test_starts_slowing_well_before_the_stop(self):
    d = Drive(assist(), 30 * MPH, 260.0).run(60)
    first = next(i for i, s in enumerate(d.states) if s == rl.RedLightState.approach)
    stopped = next(i for i, s in enumerate(d.states) if s == rl.RedLightState.stopped)
    self.assertGreater((stopped - first) * DT_MDL, 12.0)     # the driver: about 16-17 s out

  def test_the_models_stop_line_replaces_the_estimate(self):
    # the stop line 28 m before the head (the estimate would stop 43 m before it, 15 m early)
    d = Drive(assist(), 30 * MPH, 260.0, line_m=260.0 - 28.0, model_stops=True).run(60)
    self.assertEqual(d.a.state, rl.RedLightState.stopped)
    self.assertTrue(d.a.model_point)
    self.assertLessEqual(abs((d.line - d.pos) - ssc.STOP_BUFFER_M), 2.5)

  def test_green_releases_and_the_car_drives_on(self):
    d = Drive(assist(), 30 * MPH, 260.0).run(16)
    self.assertEqual(d.a.state, rl.RedLightState.approach)
    d.color = rl.GREEN
    d.run(2)
    self.assertEqual(d.a.state, rl.RedLightState.idle)
    self.assertEqual(d.a.output_v_target, V_CRUISE_UNSET)
    self.assertEqual(d.a.reason, "green")

  def test_held_at_the_red_released_on_green(self):
    d = Drive(assist(), 25 * MPH, 200.0).run(60)
    self.assertEqual(d.a.state, rl.RedLightState.stopped)
    self.assertEqual(d.a.output_v_target, 0.0)
    d.color = rl.GREEN
    d.run(1)
    self.assertEqual(d.a.output_v_target, V_CRUISE_UNSET)

  def test_a_red_first_seen_close_gets_the_lift_off_then_the_brakes_when_they_fit(self):
    # 30 mph, the head first read 100 m out (typical): the stop point 57 m ahead, the comfortable profile wants 99
    d = Drive(assist(), 30 * MPH, 100.0)
    d.run(3)
    self.assertEqual(d.a.state, rl.RedLightState.approach)
    self.assertFalse(d.a.braking)
    self.assertLess(d.v, 30 * MPH - 0.5)                    # lifted off: slowing gently
    self.assertGreaterEqual(min(d.acc), -0.5)               # the lift-off only
    # 40 mph from 200 m: lifts off, cannot stop on its own profile; the rest is the MPC's and the driver's
    d = Drive(assist(), 40 * MPH, 260.0).run(12)
    self.assertLess(d.v, 40 * MPH - 1.0)
    self.assertGreaterEqual(min(d.acc), -1.2)

  def test_an_amber_too_close_to_stop_for_is_left_alone(self):
    # 30 mph, the head 70 m ahead: the stop point 27 m ahead, a stop from here needs over 3 m/s^2
    d = Drive(assist(), 30 * MPH, 70.0, color=rl.AMBER).run(8)
    self.assertTrue(all(t == V_CRUISE_UNSET for t in d.targets))
    self.assertEqual(d.a.reason, "too close to stop for")
    self.assertAlmostEqual(d.v, 30 * MPH)

  def test_one_frame_of_red_is_nothing(self):
    a = assist()
    for i in range(100):
      a.update({"modelV2": model()}, True, False, 12.0, 0.0, rl.RED if i == 50 else rl.NONE, 60.0 if i == 50 else 0.0)
      self.assertEqual(a.state, rl.RedLightState.idle)

  def test_the_gas_makes_it_the_drivers_until_the_light_changes(self):
    d = Drive(assist(), 30 * MPH, 260.0).run(16)
    d.gas = True
    d.step()
    d.gas = False
    d.run(3)
    self.assertEqual(d.a.state, rl.RedLightState.idle)       # still red, but this one is the driver's
    self.assertTrue(all(t == V_CRUISE_UNSET for t in d.targets[-40:]))

  def test_a_car_in_front_is_left_to_the_mpc(self):
    d = Drive(assist(), 30 * MPH, 260.0, lead=30.0).run(14)
    self.assertEqual(d.a.state, rl.RedLightState.approach)
    self.assertEqual(d.a.output_v_target, d.v)               # no speeding up, no braking of its own

  def test_light_out_of_view_is_given_up(self):
    d = Drive(assist(), 30 * MPH, 260.0).run(15)
    self.assertEqual(d.a.state, rl.RedLightState.approach)
    d.color = rl.NONE
    d.v = 8.0
    for _ in range(int((rl.LOST_S + 0.5) / DT_MDL)):
      d.a.update({"modelV2": model()}, True, False, 8.0, 0.0, rl.NONE, 0.0)
    self.assertEqual(d.a.state, rl.RedLightState.idle)

  def test_without_the_param_it_runs_but_does_not_control(self):
    d = Drive(assist(on=False), 30 * MPH, 260.0).run(20)
    self.assertTrue(d.a.is_active)
    self.assertFalse(d.a.controls)

  def test_not_engaged_does_nothing(self):
    a = assist()
    for _ in range(100):
      a.update({"modelV2": model()}, False, False, 12.0, 0.0, rl.RED, 80.0)
    self.assertEqual(a.state, rl.RedLightState.idle)
    self.assertEqual(a.output_v_target, V_CRUISE_UNSET)


if __name__ == "__main__":
  unittest.main()
