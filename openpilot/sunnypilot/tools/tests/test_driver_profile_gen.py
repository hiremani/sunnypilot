import unittest

from openpilot.sunnypilot.tools import driver_profile_gen as g
from openpilot.sunnypilot.tools import fisker_drive_style as fds

MPH = 2.23694


def S(t, v, **kw):
  d = {"a": 0.0, "yaw": 0.0, "lb": False, "rb": False, "gas": True, "brake": False, "steer_pressed": False, "cruise": False,
       "lat": False, "lead_d": 0.0, "lead_v": 0.0, "limit": 65 / MPH, "lat_deg": 0.0, "lon_deg": 0.0, "route": "r", "steer_deg": 0.0}
  d.update(kw)
  return fds.Sample(t=t, v=v, **d)


class TestSpeedOffsets(unittest.TestCase):
  def test_counts_only_settled_cruising(self):
    rows = [S(i * 0.1, 69 / MPH) for i in range(100)]              # 10 s steady at 69 on a 65 road
    out = g.speed_offsets(rows, 55.0)[65]
    self.assertEqual(len(out), 100 - 30)                           # the first 3 s are settling
    self.assertAlmostEqual(out[0], 4.0, places=3)

  def test_easing_toward_a_stop_does_not_pull_it_down(self):
    cruise = [S(i * 0.1, 69 / MPH) for i in range(100)]
    easing = [S(10 + i * 0.1, (69 - i * 0.3) / MPH, a=-0.2, gas=False) for i in range(100)]   # foot off, slowing
    out = g.speed_offsets(cruise + easing, 55.0)[65]
    self.assertAlmostEqual(sorted(out)[len(out) // 2], 4.0, places=3)

  def test_not_with_acc_a_car_close_or_accelerating(self):
    for kw in ({"cruise": True}, {"lead_d": 20.0, "lead_v": 25.0}, {"a": 0.5}, {"gas": False}, {"limit": 0.0}):
      rows = [S(i * 0.1, 25.0, **kw) for i in range(100)]
      self.assertEqual(dict(g.speed_offsets(rows, 55.0)), {}, kw)

  def test_a_break_restarts_the_settling(self):
    rows = [S(i * 0.1, 69 / MPH) for i in range(25)] + [S(2.5, 69 / MPH, brake=True)] + [S(2.6 + i * 0.1, 69 / MPH) for i in range(25)]
    self.assertEqual(dict(g.speed_offsets(rows, 55.0)), {})        # never 3 s unbroken


if __name__ == "__main__":
  unittest.main()


class TestJunctionLatAccel(unittest.TestCase):
  def test_signaled_slow_tight_turns_with_acc_off_only(self):
    turn = [S(i * 0.1, 3.5, yaw=0.3, rb=True, limit=0.0) for i in range(20)]           # R 11.7 m, 1.05 m/s^2
    self.assertEqual(len(g.junction_lat_accels(turn)), 20)
    self.assertAlmostEqual(g.junction_lat_accels(turn)[0], 1.05)
    base = {"v": 3.5, "yaw": 0.3, "rb": True, "limit": 0.0}
    for change in ({"rb": False}, {"cruise": True}, {"yaw": 0.05}, {"v": 12.0}):   # no blinker, ACC, gentle, too fast
      kw = base | change
      v = kw.pop("v")
      rows = [S(i * 0.1, v, **kw) for i in range(5)]
      self.assertEqual(g.junction_lat_accels(rows), [], change)
