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
