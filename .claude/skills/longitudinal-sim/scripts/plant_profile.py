"""Run the real planner in the plant: following gap and lead coasting with the driver profile off / on."""
from unittest import mock
import openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner as splp
def _no(*a, **k):
  raise RuntimeError('no gps in the plant')
splp.StopSignAssist = _no
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import speed_limit_resolver as slr
slr.SpeedLimitResolver._get_from_map_data = lambda self, sm: None
from openpilot.sunnypilot.selfdrive.controls.lib import driver_profile as dp
from openpilot.selfdrive.test.longitudinal_maneuvers.maneuver import Maneuver

ME = dp.DriverProfile(follow_gap_s=1.77, lead_coast_s=4.2)

def run(on, v_lead, **kw):
  with mock.patch.object(dp, "active", lambda params=None: ME if on else dp.DriverProfile()), \
       mock.patch.object(dp, "is_on", lambda params=None, param=dp.PARAM: on and param == dp.PARAM):
    man = Maneuver('', **kw)
    valid, out = man.evaluate()
  return valid, out

for v in (10.0, 15.0, 25.0):
  for on in (False, True):
    valid, out = run(on, v, duration=100., initial_speed=v, lead_relevancy=True, initial_distance_lead=100,
                     speed_lead_values=[v], breakpoints=[0.], cruise_values=[130.])
    gap = out[-1, 2] - out[-1, 1]
    print(f"follow v={v:4.1f} m/s profile={'on ' if on else 'off'}: {gap:5.1f} m = {gap / v:.2f} s  valid={valid}")

# approach: ego 18.6 m/s (42 mph), set 25 m/s, lead 79 m ahead at 16.4 m/s (f5 at 2.0 min); peak accel in the first 8 s
import numpy as np
for on in (False, True):
  valid, out = run(on, 16.4, duration=20., initial_speed=18.6, lead_relevancy=True, initial_distance_lead=79,
                   speed_lead_values=[16.4], breakpoints=[0.], cruise_values=[90.0])
  t, a, d = out[:, 0], out[:, 5], out[:, 6]
  first = t < 8
  print(f"approach profile={'on ' if on else 'off'}: max accel first 8 s {a[first].max():+.2f}  min gap {d.min():.1f} m  v_end {out[-1,3]:.1f}  valid={valid}")
