import unittest

from openpilot.sunnypilot.tools import fisker_drive_style as fds
from openpilot.sunnypilot.tools import habits_extra as hx


def S(t, v, **kw):
  d = {"a": 0.0, "yaw": 0.0, "lb": False, "rb": False, "gas": False, "brake": False, "steer_pressed": False, "cruise": False,
       "lat": False, "lead_d": 0.0, "lead_v": 0.0, "limit": 0.0, "lat_deg": 0.0, "lon_deg": 0.0, "route": "r", "steer_deg": 0.0}
  d.update(kw)
  return fds.Sample(t=t, v=v, **d)


class TestHabits(unittest.TestCase):
  def test_stop_gap_behind_a_stopped_car(self):
    rows = [S(i * 0.1, max(0.0, 8.0 - 0.1 * i), lead_d=4.0 + max(0.0, 8.0 - 0.1 * i), lead_v=0.0) for i in range(120)]
    self.assertAlmostEqual(hx.stop_gaps(rows)[0], 4.0, places=1)
    with_acc = [S(r.t, r.v, lead_d=r.lead_d, cruise=True) for r in rows]
    self.assertEqual(hx.stop_gaps(with_acc), [])                    # the car's cruise stopped it, not the driver

  def test_lane_change_delay(self):
    rows = [S(i * 0.1, 20.0) for i in range(20)]
    rows += [S(2.0 + i * 0.1, 20.0, lb=True, steer_deg=0.0 if i < 6 else 3.0) for i in range(30)]
    self.assertAlmostEqual(hx.lane_change_delays(rows)[0], 0.6, places=1)
    op = [S(r.t, r.v, lb=r.lb, steer_deg=r.steer_deg, lat=True) for r in rows]
    self.assertEqual(hx.lane_change_delays(op), [])                 # openpilot's lane change, not the driver's

  def test_cruise_accel_only_free_road_on_the_gas(self):
    rows = [S(i * 0.1, 15.0, a=0.5, gas=True) for i in range(10)]
    rows += [S(1 + i * 0.1, 15.0, a=0.5, gas=True, lead_d=20.0) for i in range(10)]   # a car 1.3 s ahead
    rows += [S(2 + i * 0.1, 15.0, a=0.5, gas=False) for i in range(10)]
    self.assertEqual(len(hx.cruise_accels(rows)), 10)

  def test_curve_entry_decel(self):
    rows = [S(i * 0.1, 12.0, a=-0.6 if 20 <= i < 50 else 0.0, yaw=0.0 if i < 50 else 0.1) for i in range(80)]
    out = hx.curve_entry_decels(rows)
    self.assertEqual(len(out), 1)
    self.assertAlmostEqual(out[0], 0.6)


if __name__ == "__main__":
  unittest.main()
