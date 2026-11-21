"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

More of the driver's habits, measured from their drives (only what the driver chose: the car's cruise off, the
driver steering), for driver_profile_gen.py and the personal model's heads (tools/personal_heads.py).

From fisker_drive_style samples (10 Hz):
  stop_gaps            m behind a stopped car where the driver stops
  resume_delays        s from the car ahead moving off to the driver pulling away (measured only)
  curve_entry_decels   m/s^2 the driver slows by in the 4 s before a curve's lateral acceleration builds
  lane_change_delays   s from the blinker to the driver starting the lane change
  cruise_accels        (v, m/s^2) speeding up on open road, no car close ahead
  turn_in_rates        (turn deg, wheel deg/s) winding the wheel into a junction turn
  rolling_share        share of slow-downs to a crawl that were not full stops (measured only)
From rlogs:
  bump_crossings       mph the driver crossed mapped speed bumps at (Speed Bump Assist's shadow log)
  lane_offsets         m the car sits right of the lane centre while the driver steers (measured only)
From turn_habits.turn_onsets (map replay):
  turn_start_leads     s the driver starts a moving junction turn ahead of the turn planner's geometric start
  turn_onset_speeds    mph at which the driver starts a moving junction turn
"""
import glob
import json
import math
import os

from openpilot.sunnypilot.tools import fisker_drive_style as fds

MPH = fds.MPH

STOP_GAP_MAX_M = 15.0
RESUME_LEAD_V = 1.0          # m/s: the car ahead has moved off
RESUME_MAX_S = 10.0
CURVE_LAT = 1.0              # m/s^2: a curve has begun
CURVE_MIN_V = 9.0            # m/s (20 mph)
CURVE_LOOKBACK_S = 4.0
LC_MIN_V = 11.0              # m/s (25 mph): lane changes, not turns
LC_WHEEL_DEG = 1.5           # deg of wheel toward the blinker side: the lane change has started
LC_MAX_S = 6.0
ACCEL_MIN = 0.1              # m/s^2: speeding up
TURN_IN_LO, TURN_IN_HI = 0.2, 0.8


def _cruise_free(S, i, back_s=20.0):
  t0 = S[i].t - back_s
  k = i
  while k >= 0 and S[k].t >= t0:
    if S[k].cruise:
      return False
    k -= 1
  return True


def stop_gaps(S: list) -> list[float]:
  out = []
  for i in range(1, len(S)):
    if S[i].v < fds.V_STOP <= S[i - 1].v and _cruise_free(S, i):
      j = next((k for k in range(i, min(len(S), i + 30)) if S[k].t - S[i].t >= 1.0), None)
      if j is not None and S[j].v < fds.V_STOP and 0 < S[j].lead_d < STOP_GAP_MAX_M and S[j].lead_v < 0.5:
        out.append(S[j].lead_d)
  return out


def resume_delays(S: list) -> list[float]:
  out = []
  i = 0
  while i < len(S):
    s = S[i]
    if s.v < fds.V_STOP and 0 < s.lead_d < STOP_GAP_MAX_M and s.lead_v < 0.3 and not s.cruise:
      j = i
      while j < len(S) and S[j].v < fds.V_STOP and not (S[j].lead_d > 0 and S[j].lead_v > RESUME_LEAD_V):
        j += 1
      if j < len(S) and S[j].v < fds.V_STOP and S[j].lead_v > RESUME_LEAD_V:
        k = next((k for k in range(j, len(S)) if S[k].gas or S[k].v >= fds.V_STOP), None)
        if k is not None and not S[k].cruise:
          d = S[k].t - S[j].t
          if 0.0 <= d <= RESUME_MAX_S:
            out.append(d)
        i = j + 1
        continue
      i = j + 1
      continue
    i += 1
  return out


def curve_entry_decels(S: list) -> list[float]:
  out = []
  last = -1e9
  for i in range(1, len(S)):
    s, p = S[i], S[i - 1]
    if abs(s.v * s.yaw) >= CURVE_LAT > abs(p.v * p.yaw) and s.v > CURVE_MIN_V and s.t - last > 10.0:
      win = [x for x in S[max(0, i - 60):i + 1] if s.t - x.t <= CURVE_LOOKBACK_S]
      if len(win) < 20 or any(x.cruise or (x.lat and not x.steer_pressed) for x in win):
        continue
      if any(0 < x.lead_d < 3.0 * max(x.v, 1.0) for x in win):
        continue                       # slowing for a car, not the curve
      out.append(max(0.0, -min(x.a for x in win)))
      last = s.t
  return out


def lane_change_delays(S: list) -> list[float]:
  out = []
  for i in range(11, len(S)):
    s, p = S[i], S[i - 1]
    for on, was, sign in ((s.lb, p.lb, 1.0), (s.rb, p.rb, -1.0)):
      if not on or was or s.v < LC_MIN_V or s.cruise is None:
        continue
      base = sum(x.steer_deg for x in S[i - 10:i]) / 10.0
      for k in range(i, min(len(S), i + int(LC_MAX_S * 10))):
        x = S[k]
        if (x.lat and not x.steer_pressed) or abs(x.yaw) > 0.1:
          break                        # openpilot steering it, or a turn
        if (x.steer_deg - base) * sign > LC_WHEEL_DEG:
          out.append(x.t - s.t)
          break
  return out


def cruise_accels(S: list) -> list[tuple[float, float]]:
  return [(s.v, s.a) for s in S
          if not s.cruise and s.gas and not s.brake and s.a > ACCEL_MIN and 6.7 < s.v < 30.0 and abs(s.yaw) < 0.05
          and not (0 < s.lead_d < 3.0 * s.v)]


def turn_in_rates(S: list) -> list[tuple[float, float]]:
  """For junction turns the driver steered: the wheel's speed winding in, (TURN_IN_HI - TURN_IN_LO) of its peak over
  the time it took (deg/s)."""
  out = []
  for t in fds.turns(S):
    if t["kind"] != "junction" or t["driver_steered_pct"] <= 50:
      continue
    sign = 1.0 if t["side"] == "left" else -1.0
    seg = [x for x in S if t["t"] - 8 <= x.t <= t["t"] + 15]
    if len(seg) < 20:
      continue
    k_pk = max(range(len(seg)), key=lambda k: seg[k].steer_deg * sign)
    peak = seg[k_pk].steer_deg * sign
    if peak < 60.0:
      continue
    lo = next((k for k in range(k_pk, -1, -1) if seg[k].steer_deg * sign < TURN_IN_LO * peak), None)
    hi = next((k for k in range(lo or 0, k_pk + 1) if seg[k].steer_deg * sign >= TURN_IN_HI * peak), None)
    if lo is None or hi is None or seg[hi].t <= seg[lo].t:
      continue
    out.append((float(t["deg"]), (TURN_IN_HI - TURN_IN_LO) * peak / (seg[hi].t - seg[lo].t)))
  return out


def rolling_share(S: list) -> tuple[int, int]:
  """(rolling slow-downs, full stops) with the driver in control."""
  rolls = sum(1 for r in fds.rolling_slowdowns(S))
  return rolls, len(fds.stops(S))


def bump_crossings(root: str, route: str) -> list[float]:
  """mph: the driver's slowest speed within 5 m of each mapped bump Speed Bump Assist (shadow) logged, cruise off."""
  from openpilot.tools.lib.logreader import LogReader
  out, cur = [], {}
  cruise = True
  for seg in sorted(glob.glob(os.path.join(root, route + "--*")), key=lambda p: int(p.rsplit("--", 1)[1])):
    f = os.path.join(seg, "rlog.zst")
    if not os.path.exists(f):
      continue
    try:
      for m in LogReader(f):
        w = m.which()
        if w == "carState":
          cruise = m.carState.cruiseState.enabled
        elif w == "logMessage" and '"speed_bump"' in m.logMessage:
          try:
            d = json.loads(m.logMessage)["msg"]
          except (ValueError, KeyError):
            continue
          b = d.get("bump")
          if b is None:
            continue
          if abs(d.get("dist", 99.0)) < 5.0 and not cruise:
            cur[b] = min(cur.get(b, 99.0), d.get("v", 99.0))
          elif b in cur and d.get("dist", 0.0) < -5.0:
            out.append(cur.pop(b) * MPH)
    except Exception:
      continue
  out += [v * MPH for v in cur.values()]
  return [v for v in out if v < 40.0]


def lane_offsets(root: str, route: str) -> list[float]:
  """m right of the lane centre (the model's two nearest lane lines at 10 m), 1 Hz, driver steering, > 25 mph."""
  from openpilot.tools.lib.logreader import LogReader
  out = []
  lat, pressed, v, last = True, False, 0.0, -1.0
  for seg in sorted(glob.glob(os.path.join(root, route + "--*")), key=lambda p: int(p.rsplit("--", 1)[1])):
    f = os.path.join(seg, "rlog.zst")
    if not os.path.exists(f):
      continue
    try:
      for m in LogReader(f):
        w = m.which()
        t = m.logMonoTime * 1e-9
        if w == "carControl":
          lat = m.carControl.latActive
        elif w == "carState":
          pressed, v = m.carState.steeringPressed, m.carState.vEgo
        elif w == "modelV2" and t - last >= 1.0 and v > LC_MIN_V and (not lat or pressed):
          ll, probs = m.modelV2.laneLines, m.modelV2.laneLineProbs
          if len(ll) >= 3 and len(probs) >= 3 and probs[1] > 0.7 and probs[2] > 0.7:
            def y_at(line, x=10.0):
              xs, ys = list(line.x), list(line.y)
              k = next((i for i, xx in enumerate(xs) if xx >= x), None)
              return ys[k] if k is not None else None
            yl, yr = y_at(ll[1]), y_at(ll[2])
            if yl is not None and yr is not None and 2.5 < yr - yl < 5.0:
              out.append(-(yl + yr) / 2.0)    # model y is right-positive: the lane centre's y; car right of it = +
              last = t
    except Exception:
      continue
  return out


def turn_start_leads(onsets: list[dict], turn_in_deg_s: float = 0.0) -> tuple[list[float], list[float]]:
  """From turn_habits.turn_onsets rows (moving turns, with the planner's planned angle and radius): the start lead (s)
  that puts the turn planner's 30 deg wheel point where the driver's was, and the driver's speed there (mph)."""
  from openpilot.sunnypilot.selfdrive.controls.lib import turn_planner as tp
  pl = tp.TurnPlanner(road_source=object())     # geometry only: no map needed
  leads, speeds = [], []
  for o in onsets:
    if o.get("from_stop") or o.get("planned") is None or o.get("v", 0.0) < 1.5:
      continue
    v, r = o["v"], o.get("r_plan") or 10.0
    speeds.append(v * MPH)
    pl.tuning.lead_s = 0.0
    pl.tuning.turn_in_wheel_deg_s = turn_in_deg_s   # the build-up the planner will use (the driver's turn-in)
    pl.exit_shift = 0.0
    s0 = pl.start_distance(r, o["planned"], v)
    k30 = math.radians(30.0) / tp.STEER_RATIO_X_WHEELBASE
    to30 = tp.ACTUATOR_DELAY_S + k30 / tp.comfort_curvature_rate(v, turn_in_deg_s=turn_in_deg_s)
    leads.append((o["dist_m"] + v * to30 - s0) / v)
  return leads, speeds

