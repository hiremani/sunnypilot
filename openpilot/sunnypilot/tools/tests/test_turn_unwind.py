"""The driver profile's wheel-return speed (fisker_drive_style.unwind_rate) from 10 Hz samples."""
import unittest

from openpilot.sunnypilot.tools import fisker_drive_style as fds


def samples(wheel: list[float], yaw: float = 0.4) -> list[fds.Sample]:
  return [fds.Sample(t=0.1 * n, v=3.0, a=0.0, yaw=yaw, lb=False, rb=False, gas=False, brake=False, steer_pressed=True,
                     cruise=False, lat=False, lead_d=0.0, lead_v=0.0, limit=0.0, lat_deg=0.0, lon_deg=0.0, route="r",
                     steer_deg=w) for n, w in enumerate(wheel)]


class TestUnwindRate(unittest.TestCase):
  def test_left_turn_wheel_back_at_80_deg_s(self):
    wind = [24.0 * n for n in range(11)]                 # 0..240 deg in 1 s
    hold = [240.0] * 10
    back = [240.0 - 8.0 * n for n in range(1, 31)]       # 80 deg/s back to 0 in 3 s
    S = samples(wind + hold + back + [0.0] * 10)
    rate = fds.unwind_rate(S, 0, 25, 1)
    self.assertAlmostEqual(rate, 80.0, delta=6.0)

  def test_right_turn_sign(self):
    S = samples([-x for x in [0, 100, 200, 200, 150, 100, 50, 0, 0]], yaw=-0.4)
    self.assertAlmostEqual(fds.unwind_rate(S, 0, 3, -1), 400.0, delta=1.0)   # 160 deg from 0.3 s to 0.7 s

  def test_no_wind_up_or_no_return(self):
    self.assertIsNone(fds.unwind_rate(samples([0, 30, 60, 30, 0]), 0, 2, 1))          # a curve, not a junction turn
    self.assertIsNone(fds.unwind_rate(samples([0, 100, 200] + [200.0] * 20), 0, 2, 1))  # never came back (log ended)


if __name__ == "__main__":
  unittest.main()
