"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Where the driver starts a signaled junction turn, measured against the junction the turn planner finds.

Replays the turn planner over a drive's rlogs (lateral control forced on, its output ignored: the driver drove) and, for
each turn it arms for, records where the driver's wheel first passed ONSET_DEG toward the blinker side: the distance to
the junction node along the arrival heading, the speed, whether the car was moving off from a stop (stopped within
FROM_STOP_S before), and the turn's radius (v / yaw rate at the peak yaw rate). Needs the mapd road tiles on disk.
"""
import glob
import os

from openpilot.tools.lib.logreader import LogReader
from openpilot.sunnypilot.selfdrive.controls.lib import turn_planner as tp

ONSET_DEG = 30.0      # deg of wheel toward the blinker side: the driver has started the turn
FROM_STOP_S = 2.0     # s: stopped this recently when the turn started = moving off from a stop
MIN_PEAK_YAW = 0.15   # rad/s: a turn the car actually made (not a blinker for a lane change near a junction)
MAX_ARMED_S = 30.0


def turn_onsets(root: str, route: str, map_root: str | None = None) -> list[dict]:
  """[{side, dist_m, v, from_stop, radius_m, planned, r_plan}] for each signaled junction turn the driver made in the route."""
  planner = tp.TurnPlanner(road_source=tp.RoadSource(root=map_root, sync=True))
  rows: list[dict] = []
  cs = gps = None
  last = None
  cur = None
  stopped_t = -1e9
  segs = sorted(glob.glob(os.path.join(root, route + "--*")), key=lambda p: int(p.rsplit("--", 1)[1]))
  for seg in segs:
    f = os.path.join(seg, "rlog.zst")
    if not os.path.exists(f):
      continue
    try:
      msgs = list(LogReader(f))
    except Exception:
      continue
    for m in msgs:
      w = m.which()
      t = m.logMonoTime * 1e-9
      if w == "carState":
        cs = m.carState
      elif w == "gpsLocationExternal":
        gps = m.gpsLocationExternal
      elif w == "carControl" and cs is not None:
        dt = 0.01 if last is None else t - last
        last = t
        if cs.vEgo < 0.3:
          stopped_t = t
        planner.update(dt, cs.vEgo, cs.yawRate, cs.leftBlinker, cs.rightBlinker, True, False, gps, 0.0)
        if planner.state == tp.ARMED and cur is None:
          cur = {"side": planner.side, "onset": None, "peak": 0.0, "radius_m": None, "t": t, "planned": planner.planned}
        if cur is None:
          continue
        sign = 1.0 if cur["side"] == "left" else -1.0
        if cur["onset"] is None and cs.steeringAngleDeg * sign > ONSET_DEG and planner.state in (tp.ARMED, tp.TURNING):
          cur["onset"] = (planner.dist, cs.vEgo, t - stopped_t < FROM_STOP_S)
          cur["r_plan"] = planner.radius_for(cur["side"], cs.vEgo)
        yaw = cs.yawRate * sign
        if yaw > cur["peak"] and cs.vEgo > 1.0:
          cur["peak"], cur["radius_m"] = yaw, cs.vEgo / yaw
        if planner.state in (tp.IDLE, tp.DONE) or t - cur["t"] > MAX_ARMED_S:
          if cur["onset"] is not None and cur["peak"] > MIN_PEAK_YAW:
            d, v, from_stop = cur["onset"]
            rows.append({"side": cur["side"], "dist_m": d, "v": v, "from_stop": from_stop, "radius_m": cur["radius_m"],
                         "planned": cur.get("planned"), "r_plan": cur.get("r_plan")})
          cur = None
  return rows
