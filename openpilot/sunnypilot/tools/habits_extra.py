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
  turn_approaches      (mph where the wheel goes in, m/s^2 slowing down to it) for rolling turns made without the blinker
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
APPROACH_WHEEL_DEG = 45.0    # deg of wheel toward the turn: the wheel has gone in
APPROACH_BACK_M = 200.0      # the slow-down is looked for this far before that
APPROACH_MIN_M = 15.0        # ...and counts when it is at least this long
APPROACH_MIN_DV = 2.0        # ...and this much slower at the end (m/s)
APPROACH_ROLLING_V = 1.0     # m/s: slower than this on the way in is a stop, not a rolling turn


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


def turn_approaches(S: list) -> list[tuple[float, float]]:
  """For rolling junction turns the driver made WITHOUT the blinker (the reference: the turn assist never had the wheel)
  at their own speed (the car's cruise off): (mph where the wheel passes APPROACH_WHEEL_DEG, the average m/s^2 from the
  last fastest point in the APPROACH_BACK_M before it down to that speed)."""
  out = []
  for t in fds.turns(S):
    if t["kind"] != "junction" or t["driver_steered_pct"] <= 50 or t["blinker_lead_s"] is not None:
      continue
    sign = 1.0 if t["side"] == "left" else -1.0
    ks = [k for k, x in enumerate(S) if t["t"] - 40 <= x.t <= t["t"] + 15]
    if len(ks) < 20:
      continue
    k_pk = max((k for k in ks if S[k].t >= t["t"] - 8), key=lambda k: S[k].steer_deg * sign, default=None)
    if k_pk is None or S[k_pk].steer_deg * sign < 90.0:
      continue
    k_in = next((k for k in range(k_pk, ks[0], -1) if S[k].steer_deg * sign < APPROACH_WHEEL_DEG), None)
    if k_in is None:
      continue
    # back along the road to APPROACH_BACK_M before the wheel went in
    d, k, k_fast, d_fast = 0.0, k_in, k_in, 0.0
    while k > ks[0] and d < APPROACH_BACK_M:
      d += S[k - 1].v * (S[k].t - S[k - 1].t)
      k -= 1
      if S[k].v > S[k_fast].v:
        k_fast, d_fast = k, d
    app = S[k_fast:k_in + 1]
    if any(x.cruise for x in app) or min(x.v for x in app) < APPROACH_ROLLING_V:
      continue
    v0, v1 = S[k_fast].v, S[k_in].v
    if d_fast < APPROACH_MIN_M or v0 - v1 < APPROACH_MIN_DV:
      continue
    out.append((v1 * MPH, (v0 * v0 - v1 * v1) / (2.0 * d_fast)))
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



# ---- the remaining habits (2026-11-20) ----

PULL_AWAY_GAP_M = 1.0        # the car ahead has moved off: its gap grew this much from where both stopped
CUT_IN_DROP_M = 5.0          # a new car ahead: the gap shrank this much within 0.3 s...
CUT_IN_GAP_S = 2.0           # ...to a time gap under this


def pull_away_gaps(S: list) -> list[tuple[float, float]]:
  """Stopped behind a car (driver in control): when the driver pulled away, (seconds since the car ahead's gap started
  to grow, metres it had grown by). From the gap, not the radar's lead speed: that lags (pull_away read ~0 s)."""
  out = []
  i = 0
  while i < len(S):
    s = S[i]
    if s.v < fds.V_STOP and 0 < s.lead_d < STOP_GAP_MAX_M and not s.cruise:
      d0, j = s.lead_d, i
      t_move = None
      while j < len(S) and S[j].v < fds.V_STOP and S[j].lead_d > 0:
        if t_move is None and S[j].lead_d - d0 > 0.2:
          t_move = S[j].t
        j += 1
      if j < len(S) and t_move is not None and not S[j].cruise and S[j].lead_d > 0:
        grown = S[j].lead_d - d0
        if grown >= 0.0 and S[j].t - t_move <= RESUME_MAX_S:
          out.append((S[j].t - t_move, grown))
      i = j + 1
      continue
    i += 1
  return out


def cut_in_reactions(S: list) -> list[tuple[float, float]]:
  """A car cutting in close (driver in control, on the gas): (seconds to lift off or brake, firmest deceleration in the
  3 s after). Measured only: the MPC brakes for a cut-in by itself."""
  out = []
  for i in range(3, len(S)):
    s, p = S[i], S[i - 3]
    if s.cruise or s.v < 8.0 or not (0 < s.lead_d < CUT_IN_GAP_S * s.v):
      continue
    if not (p.lead_d == 0 or p.lead_d - s.lead_d > CUT_IN_DROP_M) or not p.gas:
      continue
    win = S[i:i + 30]
    lift = next((x.t - s.t for x in win if not x.gas or x.brake), None)
    if lift is not None:
      out.append((lift, max(0.0, -min(x.a for x in win))))
  return out


def lane_offsets_both(root: str, route: str) -> tuple[list[float], list[float]]:
  """lane_offsets for the driver steering and for openpilot steering (lat active, hands off), m right of centre."""
  from openpilot.tools.lib.logreader import LogReader
  drv, op = [], []
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
        elif w == "modelV2" and t - last >= 1.0 and v > LC_MIN_V:
          ll, probs = m.modelV2.laneLines, m.modelV2.laneLineProbs
          if len(ll) < 3 or len(probs) < 3 or probs[1] < 0.7 or probs[2] < 0.7:
            continue
          ys = []
          for line in (ll[1], ll[2]):
            xs = list(line.x)
            k = next((i for i, xx in enumerate(xs) if xx >= 10.0), None)
            ys.append(list(line.y)[k] if k is not None else None)
          if None in ys or not 2.5 < ys[1] - ys[0] < 5.0:
            continue
          off = -(ys[0] + ys[1]) / 2.0
          (op if lat and not pressed else drv).append(off)
          last = t
    except Exception:
      continue
  return drv, op


def wet_samples(root: str, route: str) -> dict:
  """With the wipers running (carStateSP.wiperSpeed > 0 in the last 15 s) vs not, the driver's open-road speed minus
  the posted limit (mph) and time gap behind a steady lead (s), cruise off, 1 Hz."""
  from openpilot.tools.lib.logreader import LogReader
  out = {"wet_offset": [], "dry_offset": [], "wet_gap": [], "dry_gap": []}
  last_wipe, limit, lead, cs, last = -1e9, 0.0, (False, 0.0, 0.0), None, -1.0
  for seg in sorted(glob.glob(os.path.join(root, route + "--*")), key=lambda p: int(p.rsplit("--", 1)[1])):
    f = os.path.join(seg, "rlog.zst")
    if not os.path.exists(f):
      continue
    try:
      for m in LogReader(f):
        w = m.which()
        t = m.logMonoTime * 1e-9
        if w == "carStateSP":
          if getattr(m.carStateSP, "wiperSpeed", 0) > 0:
            last_wipe = t
        elif w == "liveMapDataSP":
          limit = m.liveMapDataSP.speedLimit if m.liveMapDataSP.speedLimitValid else 0.0
        elif w == "radarState":
          l1 = m.radarState.leadOne
          lead = (bool(l1.present), float(l1.dRel), float(l1.vRel))
        elif w == "carState":
          cs = m.carState
          if t - last < 1.0 or cs.cruiseState.enabled or cs.vEgo < 4.5:
            continue
          last = t
          wet = t - last_wipe < 15.0
          if lead[0] and 0 < lead[1] < 100 and abs(lead[2]) < 3 and cs.vEgo > 8:
            out["wet_gap" if wet else "dry_gap"].append(lead[1] / cs.vEgo)
          elif limit > 0 and not (lead[0] and lead[1] < 3 * cs.vEgo) and not cs.brakePressed and abs(cs.aEgo) < 0.6:
            out["wet_offset" if wet else "dry_offset"].append((cs.vEgo - limit) * MPH)
    except Exception:
      continue
  return out


def signal_lift_offs(root: str, route: str) -> dict:
  """Approaches to mapped traffic lights (map_caution's map, on the recorded GPS) with the driver on the gas: for the
  lights the driver then stopped at (within 40 m before the light), the time to the light (s) when they lifted off;
  also how many lights they drove through without stopping."""
  from openpilot.tools.lib.logreader import LogReader
  from openpilot.sunnypilot.selfdrive.controls.lib import map_caution as mc
  try:
    pm = mc.PointMap(mc.MapFeatures(download=False))   # the map files on disk only: no network
  except Exception:
    return {"ttl": [], "through": 0, "stopped": 0}
  ttl, through, stopped = [], 0, 0
  cur = None            # {"id", "lift_ttl", "min_d", "stopped"}
  cs, gas_prev = None, False
  for seg in sorted(glob.glob(os.path.join(root, route + "--*")), key=lambda p: int(p.rsplit("--", 1)[1])):
    f = os.path.join(seg, "rlog.zst")
    if not os.path.exists(f):
      continue
    try:
      for m in LogReader(f):
        w = m.which()
        if w == "carState":
          cs = m.carState
        elif w == "gpsLocationExternal" and cs is not None:
          g = m.gpsLocationExternal
          if not g.hasFix or g.horizontalAccuracy > 15 or cs.cruiseState.enabled:
            gas_prev = bool(cs.gasPressed)
            continue
          ahead = mc.find_point_ahead(g.latitude, g.longitude, g.bearingDeg, pm.points_near(g.latitude, g.longitude),
                                      "signal", mc.SIGNAL_MAX_M) if g.speed >= 1.0 or cur is not None else None
          if ahead is not None and (cur is None or cur["id"] != ahead[0]):
            if cur is not None:
              stopped += cur["stopped"]
              through += not cur["stopped"]
              if cur["stopped"] and cur["lift_ttl"] is not None:
                ttl.append(cur["lift_ttl"])
            cur = {"id": ahead[0], "lift_ttl": None, "stopped": False}
          if cur is not None and ahead is not None and ahead[0] == cur["id"]:
            d = ahead[1]
            if gas_prev and not cs.gasPressed and cs.vEgo > 3.0:
              cur["lift_ttl"] = d / cs.vEgo          # the last lift-off before the light counts
            if cs.vEgo < fds.V_STOP and d < 40.0:
              cur["stopped"] = True
          gas_prev = bool(cs.gasPressed)
    except Exception:
      continue
  if cur is not None:
    stopped += cur["stopped"]
    through += not cur["stopped"]
    if cur["stopped"] and cur["lift_ttl"] is not None:
      ttl.append(cur["lift_ttl"])
  return {"ttl": [x for x in ttl if 0.5 < x < 20.0], "through": through, "stopped": stopped}


def freeway_merges(S: list) -> list[float]:
  """m/s^2: speeding up from 40 to 55 mph onto a road posted 55+, the driver on the gas (cruise off), no car within 2 s
  ahead: the mean acceleration over it."""
  out = []
  i = 1
  while i < len(S):
    if S[i].v * MPH >= 40 > S[i - 1].v * MPH and not S[i].cruise:
      j = i
      while j < len(S) and S[j].v * MPH < 55 and S[j].t - S[i].t < 30 and not S[j].cruise and S[j].v * MPH >= 35:
        j += 1
      if (j < len(S) and S[j].v * MPH >= 55 and S[j].limit * MPH >= 54 and S[j].t > S[i].t
          and not any(0 < x.lead_d < 2.0 * x.v for x in S[i:j])):
        out.append((S[j].v - S[i].v) / (S[j].t - S[i].t))
      i = j + 1
      continue
    i += 1
  return out
