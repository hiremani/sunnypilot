"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Re-run the current Stop Sign Assist and lane-turn code over a recorded drive, for the drive_sim map page.

extract() reads a drive's rlogs once into the per-model-frame inputs those controllers use (GPS, speed, blinkers, yaw
rate, the model's plan, the radar lead) and caches them; simulate() feeds them through the controllers as they are on
disk now, in shadow conditions (engaged throughout, gas ignored), and returns columnar results for the page. Map data
is read from disk only (mapd root: stop-sign features and road tiles); nothing is downloaded.
"""
import glob
import math
import os
import pickle
from types import SimpleNamespace

CACHE_DIR = os.path.expanduser("~/.cache/fisker_drive_sim")
MPH = 2.23694
BLINKER_HOLD_FRAMES = 10   # 0.5 s at the 20 Hz model rate: the same hold carstate applies to the flashing lamp since 62a1c2ec
SIGN_NEAR_M = 40.0         # signs this close to the driven path are drawn and judged
EARTH_R = 6371000.0


# ---- reading ----

def segment_dirs(root: str, route: str, segs: str | None = None) -> list[str]:
  dirs = sorted(glob.glob(os.path.join(root, f"{route}--*")), key=lambda p: int(p.rsplit("--", 1)[1]))
  if segs:
    lo, _, hi = segs.partition("-")
    dirs = [d for d in dirs if int(lo) <= int(d.rsplit("--", 1)[1]) <= int(hi or lo)]
  return [d for d in dirs if _log(d)]


def _log(seg: str) -> str | None:
  for name in ("rlog.zst", "rlog", "rlog.bz2"):
    p = os.path.join(seg, name)
    if os.path.exists(p):
      return p
  return None


def list_routes(root: str) -> list[dict]:
  out: dict[str, int] = {}
  for d in glob.glob(os.path.join(root, "*--*--*")):
    if _log(d):
      route = os.path.basename(d).rsplit("--", 1)[0]
      out[route] = out.get(route, 0) + 1
  return [{"route": r, "segments": n} for r, n in sorted(out.items())]


def extract(root: str, route: str, segs: str | None = None) -> dict:
  """The controllers' inputs at each modelV2 frame, cached by route/segment range and the logs' mtimes."""
  dirs = segment_dirs(root, route, segs)
  if not dirs:
    raise FileNotFoundError(f"no rlogs for {route} under {root}")
  stamp = max(os.path.getmtime(_log(d)) for d in dirs)
  os.makedirs(CACHE_DIR, exist_ok=True)
  cache = os.path.join(CACHE_DIR, f"{route}_{segs or 'all'}.pkl")
  if os.path.exists(cache):
    with open(cache, "rb") as f:
      data = pickle.load(f)
    if data.get("stamp") == stamp and data.get("version") == 3:
      return data

  from openpilot.tools.lib.logreader import LogReader
  frames, t0, service = [], None, None
  cur: dict = {}
  for seg in dirs:
    for m in LogReader(_log(seg)):
      if t0 is None:
        t0 = m.logMonoTime
      w = m.which()
      if w in ("gpsLocationExternal", "gpsLocation"):
        g = getattr(m, w)
        if service is None and g.hasFix:
          service = w
        if w == service:
          cur["gps"] = (g.latitude, g.longitude, g.bearingDeg, g.speed, g.hasFix, g.horizontalAccuracy, g.unixTimestampMillis)
      elif w == "carState":
        cs = m.carState
        cur["cs"] = (cs.vEgo, cs.aEgo, cs.steeringAngleDeg, cs.steeringPressed, cs.leftBlinker, cs.rightBlinker, cs.yawRate,
                     cs.gasPressed, cs.brakePressed, cs.leftBlindspot, cs.rightBlindspot)
      elif w == "radarState":
        lead = m.radarState.leadOne
        cur["lead"] = (bool(lead.present), float(lead.dRel), float(lead.vLead))
      elif w == "longitudinalPlanSP":
        r = m.longitudinalPlanSP.speedLimit.resolver
        cur["limit"] = float(r.speedLimit) if r.speedLimitValid else 0.0
      elif w == "carControl":
        cur["lat"] = m.carControl.latActive
      elif w == "modelV2" and "gps" in cur and "cs" in cur:
        md = m.modelV2
        ds = list(md.meta.desireState)
        frames.append({
          "t": (m.logMonoTime - t0) / 1e9, "gps": cur["gps"], "cs": cur["cs"], "lead": cur.get("lead", (False, 0.0, 0.0)),
          "limit": cur.get("limit", 0.0),
          "lat_active": cur.get("lat", False), "stop": md.action.shouldStop, "curv": md.action.desiredCurvature,
          "vel": [round(v, 3) for v in md.velocity.x], "pos": [round(p, 2) for p in md.position.x],
          "desire": max(range(len(ds)), key=lambda i: ds[i]) if ds else 0,
        })
  data = {"version": 3, "stamp": stamp, "route": route, "segs": segs, "service": service or "gpsLocation", "frames": frames}
  with open(cache, "wb") as f:
    pickle.dump(data, f)
  return data


# ---- simulation ----

def _sync_map():
  """A StopSignMap that loads cells inline from disk (a replay can wait, and must not miss signs while a thread loads)."""
  from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_map import StopSignMap
  from openpilot.sunnypilot.webhud.maptiles import FEAT_DEG, MapFeatures

  class SyncMap(StopSignMap):
    def signs_near(self, lat, lon):
      klat, klon = math.floor(lat / FEAT_DEG), math.floor(lon / FEAT_DEG)
      for key in [(klat + i, klon + j) for i in (-1, 0, 1) for j in (-1, 0, 1)]:
        if key not in self._cells:
          self._load(key)
      return super().signs_near(lat, lon)

  return SyncMap(features=MapFeatures(download=False))


def _dist_m(lat0, lon0, lat1, lon1) -> float:
  dx = math.radians(lon1 - lon0) * math.cos(math.radians(lat0)) * EARTH_R
  dy = math.radians(lat1 - lat0) * EARTH_R
  return math.hypot(dx, dy)


def simulate(data: dict) -> dict:
  from openpilot.common.realtime import DT_MDL
  from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_controller import StopSignAssist
  from openpilot.sunnypilot.selfdrive.controls.lib.lane_turn_desire import LaneTurnController, TurnDirection

  service = data["service"]
  params = SimpleNamespace(get_bool=lambda k: k == "StopSignAssistShadow" or (k == "UbloxAvailable" and service == "gpsLocationExternal"))
  smap = _sync_map()
  ssa = StopSignAssist(stop_map=smap, params=params, shadow=True)

  ltc = LaneTurnController(None)   # simulated as enabled at the default 19 mph, whatever this machine's params say
  ltc.enabled, ltc.lane_turn_value = True, 19.0 / MPH

  cols = {k: [] for k in ("t", "lat", "lon", "mph", "state", "sign", "dist", "target_mph", "confirmed", "too_late", "filt",
                          "turn", "logged_desire", "wheel", "pressed", "blinker", "lat_active", "curv")}
  events = []
  left_hold = right_hold = 0
  prev = None
  track_pts = []
  for i, fr in enumerate(data["frames"]):
    lat, lon, bearing, gspeed, has_fix, acc, ts = fr["gps"]
    v, a, wheel, pressed, lb, rb, yaw, gas, brake, bsl, bsr = fr["cs"]
    sm = {
      service: SimpleNamespace(latitude=lat, longitude=lon, bearingDeg=bearing, speed=gspeed, hasFix=has_fix,
                               horizontalAccuracy=acc, unixTimestampMillis=ts),
      "modelV2": SimpleNamespace(action=SimpleNamespace(shouldStop=fr["stop"]), velocity=SimpleNamespace(x=fr["vel"]),
                                 position=SimpleNamespace(x=fr["pos"])),
      "radarState": SimpleNamespace(leadOne=SimpleNamespace(present=fr["lead"][0], status=fr["lead"][0], dRel=fr["lead"][1])),
    }
    ssa.update(sm, True, False, v, a, ts * 1e-3 + 0.1)

    left_hold = BLINKER_HOLD_FRAMES if lb else max(left_hold - 1, 0)
    right_hold = BLINKER_HOLD_FRAMES if rb else max(right_hold - 1, 0)
    ltc.update_lane_turn(bsl, bsr, left_hold > 0, right_hold > 0, v, yaw_rate=yaw, dt=DT_MDL)
    turn = ltc.get_turn_direction()

    filt = float(getattr(getattr(ssa, "confirm_filter", None), "x", 0.0))
    target = ssa.output_v_target
    row = {
      "t": round(fr["t"], 2), "lat": round(lat, 6), "lon": round(lon, 6), "mph": round(v * MPH, 1), "state": ssa.state.name,
      "sign": ssa.node_id, "dist": round(ssa.distance, 1) if ssa.node_id else None,
      "target_mph": None if target > 200 else round(target * MPH, 1), "confirmed": bool(ssa.confirmed),
      "too_late": bool(getattr(ssa, "too_late", False)), "filt": round(filt, 2),
      "turn": {TurnDirection.turnLeft: "left", TurnDirection.turnRight: "right"}.get(turn, ""),
      "logged_desire": fr["desire"], "wheel": round(wheel, 1), "pressed": bool(pressed),
      "blinker": "L" if left_hold and not right_hold else "R" if right_hold and not left_hold else "",
      "lat_active": bool(fr["lat_active"]), "curv": round(fr["curv"], 4),
    }
    for k, val in row.items():
      cols[k].append(val)
    key = (row["state"], row["sign"], row["confirmed"], row["turn"])
    if key != prev:
      events.append({"i": i, "t": row["t"], "state": row["state"], "sign": row["sign"], "dist": row["dist"], "mph": row["mph"],
                     "confirmed": row["confirmed"], "too_late": row["too_late"], "turn": row["turn"]})
      prev = key
    if i % 10 == 0 and has_fix:
      track_pts.append((lat, lon, bearing))

  return {"route": data["route"], "segs": data["segs"], "frames": len(data["frames"]), "mode": "replay", "cols": cols,
          "events": events, "signs": judge_signs(smap, track_pts)}


def judge_signs(smap, track_pts) -> list[dict]:
  """Every sign near the driven path, with its verdict for the car's bearing where it passed closest."""
  from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_direction import Applies, sign_applies
  if not track_pts:
    return []
  signs = {}
  for lat, lon, _ in track_pts[::2]:
    for s in smap.signs_near(lat, lon):
      if s.node_id not in signs and _dist_m(lat, lon, s.lat, s.lon) <= SIGN_NEAR_M:
        signs[s.node_id] = s
  out = []
  for s in signs.values():
    plat, plon, pbear = min(track_pts, key=lambda p: _dist_m(p[0], p[1], s.lat, s.lon))
    verdict = sign_applies(s, pbear)
    out.append({"id": s.node_id, "lat": s.lat, "lon": s.lon, "direction": s.direction, "all_way": s.all_way,
                "approaches": list(s.approaches) if s.approaches else None,
                "verdict": {Applies.YES: "applies", Applies.NO: "not_ours", Applies.UNKNOWN: "unknown"}[verdict],
                "pass_bearing": round(pbear, 1), "pass_dist": round(_dist_m(plat, plon, s.lat, s.lon), 1)})
  return out


# ---- closed loop ----

PATH_MIN_STEP_M = 1.0       # path points closer than this are dropped (GPS jitter while slow or stopped)
DRIVER_GAIN = 1.0           # 1/s: how hard the simulated driver chases the recorded speed at that spot
DRIVER_ACCEL = (-3.0, 2.0)  # m/s^2 the simulated driver uses
PLANNER_ACCEL_MIN = -1.2    # m/s^2: the cruise loop's floor (longitudinal_planner A_CRUISE_MIN)
STOPPED_V = 0.05


def moving_span(frames: list[dict]) -> tuple[int, int]:
  """First and last frame the recorded car was moving: parked time at either end isn't route (and its GPS wander
  would add fake path)."""
  moving = [i for i, fr in enumerate(frames) if fr["cs"][0] >= 0.3]
  return (moving[0], moving[-1]) if moving else (0, len(frames) - 1)


def stop_runs(frames: list[dict], lo: int, hi: int) -> list[tuple[int, int]]:
  """The recorded stops inside [lo, hi] as (first frame, last frame)."""
  runs, start = [], None
  for i in range(lo, hi + 1):
    if frames[i]["cs"][0] < 0.3:
      start = i if start is None else start
    elif start is not None:
      runs.append((start, i - 1))
      start = None
  if start is not None:
    runs.append((start, hi))
  return runs


def build_path(frames: list[dict], lo: int = 0, hi: int | None = None):
  """The recorded drive as a road path: per kept point (s m along the path, lat, lon, bearing deg, frame index)."""
  pts = []
  hi = len(frames) - 1 if hi is None else hi
  for i in range(lo, hi + 1):
    fr = frames[i]
    lat, lon, _, _, has_fix, _, _ = fr["gps"]
    if not has_fix:
      continue
    if pts and _dist_m(pts[-1][1], pts[-1][2], lat, lon) < PATH_MIN_STEP_M:
      continue
    s = pts[-1][0] + _dist_m(pts[-1][1], pts[-1][2], lat, lon) if pts else 0.0
    pts.append([s, lat, lon, 0.0, i])
  for k in range(len(pts)):   # bearing from the path itself over ~5 m either side, not the GPS course (junk when slow)
    a, b = pts[max(k - 3, 0)], pts[min(k + 3, len(pts) - 1)]
    dy = math.radians(b[1] - a[1]) * EARTH_R
    dx = math.radians(b[2] - a[2]) * math.cos(math.radians(a[1])) * EARTH_R
    pts[k][3] = (math.degrees(math.atan2(dx, dy)) + 360.0) % 360.0 if (dx or dy) else (pts[k - 1][3] if k else 0.0)
  return pts


def _angle_diff(a: float, b: float) -> float:
  return (a - b + 180.0) % 360.0 - 180.0


def simulate_closed_loop(data: dict, proceed_wait_s: float = 2.0) -> dict:
  """The recorded route, driven by a simulated car whose speed obeys Stop Sign Assist.

  Where the assist isn't acting, a simulated driver drives like the recording did at that spot (the recorded speed by
  distance along the path, its stops held as long as they were). Where it commands a lower speed, the car follows it
  through the cruise loop's response (a = target - v, floor PLANNER_ACCEL_MIN). Stopped by the assist, it waits
  proceed_wait_s and then the driver taps the accelerator, as on the car. The model's plan is the recorded frame at the
  same spot, so confirmation timing is approximate where the simulated speed differs a lot from the recording's.
  """
  from openpilot.common.realtime import DT_MDL
  from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_controller import StopSignAssist
  from openpilot.sunnypilot.selfdrive.controls.lib.lane_turn_desire import LaneTurnController, TurnDirection

  frames = data["frames"]
  lo, hi = moving_span(frames)
  path = build_path(frames, lo, hi)
  runs = stop_runs(frames, lo, hi)
  run_of = {}
  for r_i, (a, b) in enumerate(runs):
    for f_i in range(a, b + 1):
      run_of[f_i] = r_i
  if len(path) < 2:
    raise ValueError("no GPS path in these segments")
  service = data["service"]
  params = SimpleNamespace(get_bool=lambda k: k == "StopSignAssistShadow" or (k == "UbloxAvailable" and service == "gpsLocationExternal"))
  smap = _sync_map()
  ssa = StopSignAssist(stop_map=smap, params=params, shadow=True)
  ltc = LaneTurnController(None)
  ltc.enabled, ltc.lane_turn_value = True, 19.0 / MPH

  # the recorded drive by distance: frame index of each path point, so the driver and the model can be looked up by s
  s_of_frame = [0.0] * len(frames)
  k = 0
  for i in range(lo, hi + 1):
    while k + 1 < len(path) and path[k + 1][4] <= i:
      k += 1
    s_of_frame[i] = path[k][0]

  cols = {key: [] for key in ("t", "lat", "lon", "mph", "rec_mph", "state", "sign", "dist", "target_mph", "confirmed",
                              "too_late", "filt", "turn", "logged_desire", "wheel", "pressed", "blinker", "lat_active", "curv")}
  events, track_pts = [], []
  s, v, t = 0.0, frames[path[0][4]]["cs"][0], 0.0
  j = path[0][4]            # the recorded frame the simulated car is "at"
  wait_left = 0             # frames the simulated driver still waits at a recorded stop
  waited = set()            # recorded stops (stop_runs index) already waited out
  stopped_for = 0           # frames held at a stop by the assist
  pk = 0                    # current path point index
  prev_key, prev_bearing = None, path[0][3]
  ts0 = frames[path[0][4]]["gps"][6]
  left_hold = right_hold = 0
  end_s = path[-1][0]
  step = 0
  max_steps = int((hi - lo + 1) * 4)   # a stuck simulation ends; the page shows where
  while s < end_s - 1.0 and step < max_steps:
    step += 1
    while j + 1 <= hi and s_of_frame[j + 1] <= s:
      j += 1
    fr = frames[j]
    while pk + 1 < len(path) and path[pk + 1][0] <= s:
      pk += 1
    a_pt, b_pt = path[pk], path[min(pk + 1, len(path) - 1)]
    f = 0.0 if b_pt[0] == a_pt[0] else (s - a_pt[0]) / (b_pt[0] - a_pt[0])
    lat, lon = a_pt[1] + f * (b_pt[1] - a_pt[1]), a_pt[2] + f * (b_pt[2] - a_pt[2])
    bearing = (a_pt[3] + f * _angle_diff(b_pt[3], a_pt[3])) % 360.0
    yaw = math.radians(_angle_diff(bearing, prev_bearing)) / DT_MDL
    prev_bearing = bearing

    # the simulated driver: the recorded speed here, and the recorded stops held as long as they lasted
    rec_v = fr["cs"][0]
    r_i = run_of.get(j)
    if r_i is not None and r_i not in waited and wait_left == 0:
      a_run, b_run = runs[r_i]
      wait_left = b_run - a_run + 1
      waited.add(r_i)
    if wait_left > 0:
      wait_left -= 1
      driver_v = 0.0
    else:
      nxt = runs[r_i][1] + 1 if r_i is not None else j
      driver_v = max(frames[min(nxt, hi)]["cs"][0], 1.0)

    # the driver taps the gas once the assist has held the car still for proceed_wait_s (from its stopped state, or any
    # other hold while the driver would be moving)
    gas = ssa.is_active and stopped_for >= int(proceed_wait_s / DT_MDL)
    ts = ts0 + int(t * 1000)
    sm = {
      service: SimpleNamespace(latitude=lat, longitude=lon, bearingDeg=bearing, speed=v, hasFix=True,
                               horizontalAccuracy=3.0, unixTimestampMillis=ts),
      "modelV2": SimpleNamespace(action=SimpleNamespace(shouldStop=fr["stop"]), velocity=SimpleNamespace(x=fr["vel"]),
                                 position=SimpleNamespace(x=fr["pos"])),
      "radarState": SimpleNamespace(leadOne=SimpleNamespace(present=fr["lead"][0], status=fr["lead"][0], dRel=fr["lead"][1])),
    }
    ssa.update(sm, True, gas, v, 0.0, ts * 1e-3 + 0.1)
    stopped_for = stopped_for + 1 if ssa.is_active and v < STOPPED_V and driver_v > 0.0 else 0

    a_driver = max(DRIVER_ACCEL[0], min(DRIVER_ACCEL[1], DRIVER_GAIN * (driver_v - v)))
    target = ssa.output_v_target
    if target < 200:
      a_cmd = max(PLANNER_ACCEL_MIN, min(target - v, 0.0))
      accel = min(a_driver, a_cmd)
    else:
      accel = a_driver
    v = max(0.0, v + accel * DT_MDL)
    if v < STOPPED_V and (target < 200 or driver_v == 0.0):
      v = 0.0
    s += v * DT_MDL
    t += DT_MDL

    lb, rb = fr["cs"][4], fr["cs"][5]
    left_hold = BLINKER_HOLD_FRAMES if lb else max(left_hold - 1, 0)
    right_hold = BLINKER_HOLD_FRAMES if rb else max(right_hold - 1, 0)
    ltc.update_lane_turn(fr["cs"][9], fr["cs"][10], left_hold > 0, right_hold > 0, v, yaw_rate=yaw, dt=DT_MDL)
    turn = ltc.get_turn_direction()

    filt = float(getattr(getattr(ssa, "confirm_filter", None), "x", 0.0))
    row = {
      "t": round(t, 2), "lat": round(lat, 6), "lon": round(lon, 6), "mph": round(v * MPH, 1), "rec_mph": round(rec_v * MPH, 1),
      "state": ssa.state.name, "sign": ssa.node_id, "dist": round(ssa.distance, 1) if ssa.node_id else None,
      "target_mph": None if target > 200 else round(target * MPH, 1), "confirmed": bool(ssa.confirmed),
      "too_late": bool(getattr(ssa, "too_late", False)), "filt": round(filt, 2),
      "turn": {TurnDirection.turnLeft: "left", TurnDirection.turnRight: "right"}.get(turn, ""),
      "logged_desire": fr["desire"], "wheel": round(fr["cs"][2], 1), "pressed": bool(fr["cs"][3]),
      "blinker": "L" if left_hold and not right_hold else "R" if right_hold and not left_hold else "",
      "lat_active": bool(fr["lat_active"]), "curv": round(fr["curv"], 4),
    }
    for key, val in row.items():
      cols[key].append(val)
    key = (row["state"], row["sign"], row["confirmed"], row["turn"])
    if key != prev_key:
      events.append({"i": len(cols["t"]) - 1, "t": row["t"], "state": row["state"], "sign": row["sign"], "dist": row["dist"],
                     "mph": row["mph"], "confirmed": row["confirmed"], "too_late": row["too_late"], "turn": row["turn"]})
      prev_key = key
    if step % 10 == 0:
      track_pts.append((lat, lon, bearing))

  stops = []   # where the assist held the car: sign, distance short of its stop point, how long
  for e in events:
    if e["state"] == "stopped":
      stops.append({"t": e["t"], "sign": e["sign"], "dist": e["dist"]})
  return {"route": data["route"], "segs": data["segs"], "frames": len(cols["t"]), "mode": "closed",
          "completed": s >= end_s - 1.0, "recorded_s": round(frames[hi]["t"] - frames[lo]["t"], 1), "simulated_s": round(t, 1),
          "path_m": round(end_s, 1), "stops": stops, "cols": cols, "events": events, "signs": judge_signs(smap, track_pts)}


# ---- turn planner ----

TURN_YAW_ON = 0.12          # rad/s: a recorded turn starts with the yaw rate above this...
TURN_YAW_OFF = 0.05         # ...and lasts while it stays above this
TURN_MIN_DEG = 40.0         # recorded heading change that counts as a turn
TURN_MAX_V = 9.0            # m/s: turns started faster than this aren't junction turns
TURN_BLINKER_BEFORE_S = 8.0  # a blinker on the turn's side this long before (or during) it makes it a signaled turn,
                             # if it was on after the previous recorded turn ended (one blinker, one turn)
GHOST_TAIL_S = 3.0          # after the planner releases, the ghost keeps rolling (wheel unwinding) at most this long
GHOST_STRAIGHT_CURV = 0.004  # ...or until its curvature is this small
REC_AFTER_S = 6.0           # the recorded path compared against runs this long past the recorded turn's end


def recorded_turns(frames: list[dict]) -> list[dict]:
  """The signaled junction turns in a recording: a low-speed heading change >= TURN_MIN_DEG (yaw-rate integral) with
  the blinker of that side on in the TURN_BLINKER_BEFORE_S before it or during it. Bearing convention: + = right."""
  from openpilot.common.realtime import DT_MDL
  out, i, n = [], 0, len(frames)
  prev_end = -1
  while i < n:
    cs = frames[i]["cs"]
    if abs(cs[6]) > TURN_YAW_ON and cs[0] < TURN_MAX_V:
      a = i
      while a > 0 and abs(frames[a - 1]["cs"][6]) > TURN_YAW_OFF:
        a -= 1
      j, h = a, 0.0
      while j < n and abs(frames[j]["cs"][6]) > TURN_YAW_OFF:
        h += frames[j]["cs"][6] * DT_MDL
        j += 1
      change = -math.degrees(h)   # yawRate + = left; bearing change + = right
      if abs(change) >= TURN_MIN_DEG:
        side = "right" if change > 0 else "left"
        k0 = max(0, a - int(TURN_BLINKER_BEFORE_S / DT_MDL), prev_end + 1)
        col = 5 if side == "right" else 4
        if any(frames[k]["cs"][col] for k in range(k0, j)):
          out.append({"start": a, "end": j - 1, "side": side, "change": change})
        prev_end = j - 1
      i = j + 1
    else:
      i += 1
  return out


def _xy(lat0, lon0, lat, lon):
  return (math.radians(lon - lon0) * math.cos(math.radians(lat0)) * EARTH_R, math.radians(lat - lat0) * EARTH_R)


def _latlon(lat0, lon0, x, y):
  return (lat0 + math.degrees(y / EARTH_R), lon0 + math.degrees(x / (EARTH_R * math.cos(math.radians(lat0)))))


def _dist_to_polyline(p, line) -> float:
  best = float("inf")
  for (ax, ay), (bx, by) in zip(line[:-1], line[1:], strict=True):
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    t = 0.0 if l2 < 1e-9 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / l2))
    best = min(best, math.hypot(p[0] - ax - t * dx, p[1] - ay - t * dy))
  return best


def evaluate_turns(data: dict, tuning=None, osm_root: str | None = None) -> dict:
  """Run the turn planner over the recording, and from each point where it starts a turn roll a kinematic "ghost" car
  (heading += v * curvature * dt at the recorded speed) through the curvature it commands, after the car's actuator
  (delay, then the curvature build-up rate of the wheel-rate and jerk limits). While the ghost turns, the planner is fed
  the ghost's own yaw rate, so its heading tracking and release act on the ghost. The driver's recorded steering isn't
  fed in (it would read as an override). Per recorded signaled turn: did the planner trigger, how far from the junction,
  planned vs actual heading change, and the ghost's max distance from the recorded path and its heading error at the end.
  """
  from openpilot.common.realtime import DT_MDL
  from openpilot.sunnypilot.selfdrive.controls.lib import turn_planner as tp

  frames = data["frames"]
  src = tp.RoadSource(root=os.path.join(osm_root, "offline") if osm_root else None, sync=True)
  planner = tp.TurnPlanner(road_source=src, tuning=tuning)
  delay_n = max(0, round(tp.ACTUATOR_DELAY_S / DT_MDL))
  left_hold = right_hold = 0
  ghost = None
  starts, arms, ends = [], [], []
  tp_state = []
  prev_state = tp.IDLE
  for i, fr in enumerate(frames):
    lat, lon, bearing, gspeed, has_fix, acc, ts = fr["gps"]
    v, _, _, _, lb, rb, yaw, *_ = fr["cs"]
    left_hold = BLINKER_HOLD_FRAMES if lb else max(left_hold - 1, 0)
    right_hold = BLINKER_HOLD_FRAMES if rb else max(right_hold - 1, 0)
    gps = SimpleNamespace(latitude=lat, longitude=lon, bearingDeg=bearing, speed=gspeed, hasFix=has_fix,
                          horizontalAccuracy=acc, unixTimestampMillis=ts)
    yaw_in = -v * ghost["k_act"] if ghost is not None else yaw
    cmd = planner.update(DT_MDL, v, yaw_in, left_hold > 0, right_hold > 0, bool(fr["lat_active"]), False, gps)
    st = planner.state
    if st == tp.ARMED and prev_state != tp.ARMED:
      arms.append({"i": i, "dist": planner.dist, "planned": planner.planned, "side": planner.side})
    if st == tp.TURNING and prev_state != tp.TURNING:
      ghost = {"i0": i, "lat0": lat, "lon0": lon, "x": 0.0, "y": 0.0, "h": _pre_turn_heading(frames, i), "k_act": 0.0,
               "queue": [0.0] * delay_n, "pts": [(0.0, 0.0)], "released": None, "planned": planner.planned,
               "start_dist": planner.log.start_dist, "radius": planner.radius, "side": planner.side,
               "junction": _latlon(planner.roads.lat0, planner.roads.lon0, *planner.junction_xy)}
      starts.append(ghost)
    if prev_state in (tp.ARMED, tp.TURNING) and st not in (tp.ARMED, tp.TURNING):
      ends.append({"i": i, "reason": planner.log.reason, "from": prev_state})
    prev_state = st
    tp_state.append(st)
    if ghost is not None:
      if cmd is None and ghost["released"] is None:
        ghost["released"] = i
      k_cmd = cmd if cmd is not None else 0.0
      ghost["queue"].append(k_cmd)
      k_target = ghost["queue"].pop(0)
      unwind = abs(k_target) < abs(ghost["k_act"])
      rate = tp.curvature_rate(v, unwind=unwind) * DT_MDL
      ghost["k_act"] += max(-rate, min(rate, k_target - ghost["k_act"]))
      ghost["h"] += math.degrees(v * ghost["k_act"] * DT_MDL)
      ghost["x"] += v * math.sin(math.radians(ghost["h"])) * DT_MDL
      ghost["y"] += v * math.cos(math.radians(ghost["h"])) * DT_MDL
      ghost["pts"].append((ghost["x"], ghost["y"]))
      if ghost["released"] is not None and (abs(ghost["k_act"]) < GHOST_STRAIGHT_CURV or
                                            (i - ghost["released"]) * DT_MDL > GHOST_TAIL_S):
        ghost["i1"] = i
        ghost = None
  if ghost is not None:
    ghost["i1"] = len(frames) - 1

  turns = []
  for n, rt in enumerate(recorded_turns(frames)):
    a, b = rt["start"], rt["end"]
    lo = a - int(15.0 / DT_MDL)
    g = next((s for s in starts if lo <= s["i0"] <= b), None)
    arm = next((x for x in arms if lo <= x["i"] <= b), None)
    end = next((e for e in ends if arm and e["i"] >= arm["i"]), None)
    row = {"n": n + 1, "t": round(frames[a]["t"], 1), "side": rt["side"], "actual_deg": round(rt["change"], 1),
           "armed": arm is not None, "arm_dist": round(arm["dist"], 1) if arm else None,
           "triggered": g is not None, "trigger_dist": None, "planned_deg": round(arm["planned"], 1) if arm else None,
           "max_lat_err": None, "end_lat_err": None, "end_heading_err": None, "reason": end["reason"] if end else "",
           "radius": None, "trigger_t": None, "trigger_lead_s": None}
    if g is not None:
      i0 = g["i0"]
      lat0, lon0 = g["lat0"], g["lon0"]
      rec = []
      for k in range(max(0, i0 - int(1.0 / DT_MDL)), min(len(frames), b + int(REC_AFTER_S / DT_MDL))):
        la, lo_, *_ = frames[k]["gps"]
        rec.append(_xy(lat0, lon0, la, lo_))
      errs = [_dist_to_polyline(p, rec) for p in g["pts"]]
      exit_heading = _exit_heading(frames, b)
      row.update({"trigger_dist": round(g["start_dist"], 1), "max_lat_err": round(max(errs), 2), "end_lat_err": round(errs[-1], 2),
                  "end_heading_err": round(_angle_diff(g["h"], exit_heading), 1), "radius": round(g["radius"], 1),
                  "trigger_t": round(frames[i0]["t"], 1), "trigger_lead_s": round(frames[a]["t"] - frames[i0]["t"], 2)})
      g["n"] = n + 1
    turns.append(row)
  ghost_out = [{"n": g.get("n"), "side": g["side"], "i0": g["i0"], "junction": [round(c, 7) for c in g["junction"]],
                "lat": [round(_latlon(g["lat0"], g["lon0"], x, y)[0], 7) for x, y in g["pts"][::2]],
                "lon": [round(_latlon(g["lat0"], g["lon0"], x, y)[1], 7) for x, y in g["pts"][::2]]} for g in starts]
  return {"turns": turns, "ghosts": ghost_out, "tp_state": tp_state}


def _pre_turn_heading(frames: list[dict], i: int) -> float:
  """The ghost's starting heading at frame i: the recorded heading there, unless the recorded driver had already begun
  turning (then the heading before that began: the planner-driven car would still be going along the road)."""
  from openpilot.common.realtime import DT_MDL
  k = i
  while k > 0 and abs(frames[k]["cs"][6]) > TURN_YAW_OFF and (i - k) * DT_MDL < 10.0:
    k -= 1
  return _rec_heading(frames, k)


def _exit_heading(frames: list[dict], end: int) -> float:
  """The recorded heading once the turn is over: the track over the first ~8 m driven after the recorded turn's end."""
  lat0, lon0, *_ = frames[end]["gps"]
  for k in range(end + 1, min(len(frames), end + 400)):
    lat1, lon1, *_ = frames[k]["gps"]
    if _dist_m(lat0, lon0, lat1, lon1) >= 8.0:
      dx, dy = _xy(lat0, lon0, lat1, lon1)
      return math.degrees(math.atan2(dx, dy)) % 360.0
  return _rec_heading(frames, end)


def _rec_heading(frames: list[dict], i: int) -> float:
  """The recorded car's heading at frame i from its GPS track (the ~6 m behind it), or the GPS course."""
  lat1, lon1, bearing, *_ = frames[i]["gps"]
  for k in range(i - 1, max(-1, i - 200), -1):
    lat0, lon0, *_ = frames[k]["gps"]
    if _dist_m(lat0, lon0, lat1, lon1) >= 6.0:
      dx, dy = _xy(lat0, lon0, lat1, lon1)
      return math.degrees(math.atan2(dx, dy)) % 360.0
  return bearing


def simulate_turns(data: dict, tuning=None) -> dict:
  """Replay plus the turn planner's evaluation, for the page's "Turn planner" mode."""
  out = simulate(data)
  ev = evaluate_turns(data, tuning)
  out["cols"]["tp_state"] = ev.pop("tp_state")
  out.update(ev)
  out["mode"] = "turns"
  return out


# ---- narrow streets ----

NARROW_PUSH_TQ = 1.0   # Nm: the driver pushing the wheel (not just resting a hand on it)


def extract_narrow(root: str, route: str, segs: str | None = None) -> dict:
  """Per modelV2 frame, aligned 1:1 with extract()'s frames (same logs, same frame filter), the narrow-street
  detector's state and what the center-line keeper would shift, run over the recording as if NarrowStreetAssist were
  on (the keeper ignores the driver's hands here, to show what it would want), plus the driver's own pushes."""
  from openpilot.common.realtime import DT_MDL
  from openpilot.sunnypilot.selfdrive.controls.lib import narrow_street as ns
  dirs = segment_dirs(root, route, segs)
  stamp = max(os.path.getmtime(_log(d)) for d in dirs)
  src = os.path.getmtime(ns.__file__)
  os.makedirs(CACHE_DIR, exist_ok=True)
  cache = os.path.join(CACHE_DIR, f"{route}_{segs or 'all'}_narrow.pkl")
  if os.path.exists(cache):
    with open(cache, "rb") as f:
      out = pickle.load(f)
    if out.get("stamp") == (stamp, src):
      return out

  from openpilot.tools.lib.logreader import LogReader
  keeper, det = ns.CenterLineKeeper(DT_MDL), ns.NarrowStreetDetector(DT_MDL)
  cols = {k: [] for k in ("narrow", "shift", "push", "left_line", "left_prob", "width")}
  service, have_gps, cs, lat_active = None, False, None, False
  for seg in dirs:
    for m in LogReader(_log(seg)):
      w = m.which()
      if w in ("gpsLocationExternal", "gpsLocation"):
        g = getattr(m, w)
        if service is None and g.hasFix:
          service = w
        have_gps = have_gps or w == service
      elif w == "carState":
        cs = m.carState
      elif w == "carControl":
        lat_active = m.carControl.latActive
      elif w == "modelV2" and have_gps and cs is not None:
        md = m.modelV2
        keeper.update(md, cs.vEgo, cs.leftBlinker, False, lat_active)
        mv = ns.ModelView(md)
        push = 0
        if lat_active and abs(cs.steeringTorque) > NARROW_PUSH_TQ and abs(cs.steeringAngleDeg) < 30 and cs.vEgo > 3:
          push = -1 if cs.steeringTorque < 0 else 1    # torque + = left; -1 = pushing RIGHT
        cols["narrow"].append(int(det.update(md, cs.vEgo)))
        cols["shift"].append(round(keeper.shift, 2))
        cols["push"].append(push)
        cols["left_line"].append(round(mv.left_line, 2) if math.isfinite(mv.left_line) else None)
        cols["left_prob"].append(round(mv.left_prob, 2))
        width = mv.right_edge - mv.left_edge
        cols["width"].append(round(width, 1) if math.isfinite(width) else None)
  out = {"stamp": (stamp, src), "cols": cols}
  with open(cache, "wb") as f:
    pickle.dump(out, f)
  return out


def _runs(flags: list, min_n: int, gap_n: int = 0) -> list[tuple[int, int]]:
  """(start, end) index runs where flags are truthy, at least min_n long, merging gaps up to gap_n."""
  runs: list[list[int]] = []
  for i, f in enumerate(flags):
    if not f:
      continue
    if runs and i - runs[-1][1] <= gap_n + 1:
      runs[-1][1] = i
    else:
      runs.append([i, i])
  return [(a, b) for a, b in runs if b - a + 1 >= min_n]


def simulate_narrow(data: dict, root: str) -> dict:
  """Replay plus the narrow-street assist, for the page's "Narrow streets" mode."""
  out = simulate(data)
  nar = extract_narrow(root, data["route"], data["segs"])["cols"]
  n = len(out["cols"]["t"])
  if len(nar["narrow"]) != n:
    raise RuntimeError(f"narrow pass has {len(nar['narrow'])} frames, replay {n}: logs changed? clear {CACHE_DIR}")
  out["cols"].update(nar)
  c = out["cols"]
  spans = [{"i": a, "t": c["t"][a], "dur": round(c["t"][b] - c["t"][a], 1),
            "mph": round(sum(c["mph"][a:b + 1]) / (b - a + 1)),
            "width": round(sum(w for w in c["width"][a:b + 1] if w) / max(1, sum(1 for w in c["width"][a:b + 1] if w)), 1)}
           for a, b in _runs(c["narrow"], 20)]
  pushes = []
  for side, val in (("right", -1), ("left", 1)):
    for a, b in _runs([p == val for p in c["push"]], 10, gap_n=10):
      seg = c["shift"][a:b + 1]
      pushes.append({"i": a, "t": c["t"][a], "dur": round(c["t"][b] - c["t"][a], 1), "side": side,
                     "keeper": round(max(seg), 2), "narrow": any(c["narrow"][a:b + 1]), "mph": c["mph"][a]})
  pushes.sort(key=lambda p: p["i"])
  out.update({"mode": "narrow", "narrow_spans": spans, "pushes": pushes,
              "keeper_active_s": round(sum(1 for s in c["shift"] if s > 0.05) / 20, 1)})
  return out


# ---- your style ----

# The driver's own habits (fisker_drive_style over 000000ea..f2, 2026-11-14): openpilot longitudinal tuned to them.
STYLE = {
  "town_offset_mph": 0.0,      # 25-45 mph zones: the driver ran under the limit (median -5..-8)
  "fast_offset_mph": 5.0,      # 55+ mph zones: median +2, p85 +9
  "fast_zone_mph": 55.0,
  "lat_accel": 2.2,            # m/s^2 in curves (driver p50 1.5, p95 3.2)
  "decel": 1.1,                # m/s^2 planned braking for stops and curves (driver average 1.12)
  "gap_s": 1.65,               # following gap behind a lead (driver median)
  "standstill_gap_m": 4.0,
  "accel_max": 1.0,            # m/s^2: the car's openpilot limit (ACCEL_MAX); the driver launches at ~1.6
  "decel_max": 3.5,
}
CURVE_LOOK_M = 150.0


def path_curvature(path) -> list[float]:
  """|curvature| (1/m) at each path point from the bearing change over ~10 m either side."""
  out = []
  for k in range(len(path)):
    a, b = k, k
    while a > 0 and path[k][0] - path[a][0] < 10.0:
      a -= 1
    while b < len(path) - 1 and path[b][0] - path[k][0] < 10.0:
      b += 1
    ds = path[b][0] - path[a][0]
    out.append(abs(math.radians(_angle_diff(path[b][3], path[a][3]))) / ds if ds > 5.0 else 0.0)
  return out


def style_from_profile(name: str | None) -> tuple[dict, str]:
  """The style dict from a named driver profile (driver_profile.py), and the name used; STYLE if there is none."""
  from openpilot.sunnypilot.selfdrive.controls.lib import driver_profile as dp
  names = [p["name"] for p in dp.list_profiles()]
  name = name if name in names else (dp.DEFAULT_NAME if dp.DEFAULT_NAME in names else (names[0] if names else None))
  if name is None:
    return dict(STYLE), "built-in"
  p = dp.load(name)
  return STYLE | {"town_offset_mph": p.town_offset_mph, "fast_offset_mph": p.fast_offset_mph, "fast_zone_mph": p.fast_zone_mph,
                  "lat_accel": p.curve_lat_accel, "decel": p.stop_decel, "gap_s": p.follow_gap_s,
                  "accel_max": min(p.launch_accel, STYLE["accel_max"])}, name


def simulate_style(data: dict, style: dict | None = None, profile: str | None = None) -> dict:
  """The recorded route driven by openpilot longitudinal tuned to the driver's style: the posted limit (+ offset), the
  driver's lateral accel in curves, the driver's braking to each place the driver stopped (held as long), the
  driver's gap to the lead the recording saw at that spot, and the car's accel limit. Compared with the recorded speed
  at the same place."""
  from openpilot.common.realtime import DT_MDL
  base, profile_used = style_from_profile(profile)
  st = base | (style or {})
  frames = data["frames"]
  lo, hi = moving_span(frames)
  path = build_path(frames, lo, hi)
  if len(path) < 2:
    raise ValueError("no GPS path in these segments")
  curv = path_curvature(path)
  runs = stop_runs(frames, lo, hi)
  s_of_frame = [0.0] * len(frames)
  k = 0
  for i in range(lo, hi + 1):
    while k + 1 < len(path) and path[k + 1][4] <= i:
      k += 1
    s_of_frame[i] = path[k][0]
  # The world runs on the recorded clock: the lead car is where the recording saw it at the same time (as a position
  # along the route), and at each place the driver stopped the car waits until the driver pulled away (a light
  # turning green), with at least a 1 s full stop.
  import bisect
  t0_rec = frames[path[0][4]]["t"]
  times = [frames[i]["t"] for i in range(lo, hi + 1)]
  stop_at = [(s_of_frame[a], frames[b]["t"]) for a, b in runs if b - a >= 10]   # (where, recorded departure time)
  end_s = path[-1][0]

  cols = {key: [] for key in ("t", "lat", "lon", "mph", "rec_mph", "state", "sign", "dist", "target_mph", "confirmed",
                              "too_late", "filt", "turn", "logged_desire", "wheel", "pressed", "blinker", "lat_active", "curv",
                              "why")}
  s, v, a_prev, t = 0.0, frames[path[0][4]]["cs"][0], 0.0, 0.0
  j, pk, si, hold = path[0][4], 0, 0, 0
  why_t: dict[str, float] = {}
  step, max_steps = 0, int((hi - lo + 1) * 4)
  while s < end_s - 1.0 and step < max_steps:
    step += 1
    while j + 1 <= hi and s_of_frame[j + 1] <= s:
      j += 1
    fr = frames[j]
    while pk + 1 < len(path) and path[pk + 1][0] <= s:
      pk += 1
    a_pt, b_pt = path[pk], path[min(pk + 1, len(path) - 1)]
    f = 0.0 if b_pt[0] == a_pt[0] else (s - a_pt[0]) / (b_pt[0] - a_pt[0])
    lat, lon = a_pt[1] + f * (b_pt[1] - a_pt[1]), a_pt[2] + f * (b_pt[2] - a_pt[2])

    # set speed: the posted limit plus the driver's offset; with no limit, what the driver drove here
    limit = fr.get("limit", 0.0)
    if limit > 0.0:
      off = st["fast_offset_mph"] if limit * MPH >= st["fast_zone_mph"] - 2 else st["town_offset_mph"]
      target, why = limit + off / MPH, "limit"
    else:
      target, why = max(fr["cs"][0], 5.0), "your speed (no limit)"
    # curves ahead, braking at the driver's rate
    q = pk
    while q < len(path) and path[q][0] - s < CURVE_LOOK_M:
      if curv[q] > 1e-4:
        vc = math.sqrt(st["lat_accel"] / curv[q])
        vc = math.sqrt(vc * vc + 2 * st["decel"] * max(path[q][0] - s, 0.0))
        if vc < target:
          target, why = vc, "curve"
      q += 1
    # the next place the driver stopped
    while si < len(stop_at) and stop_at[si][0] < s - 3.0 and hold == 0:
      si += 1
    if si < len(stop_at):
      d = stop_at[si][0] - s
      vs = math.sqrt(2 * st["decel"] * max(d, 0.0))
      if vs < target:
        target, why = vs, "stop"
      if d < 1.5 and v < 0.5 and hold == 0:
        hold = max(int(1.0 / DT_MDL), int((stop_at[si][1] - (t0_rec + t)) / DT_MDL))
    if hold > 0:
      hold -= 1
      target, why = 0.0, "stopped"
      if hold == 0:
        si += 1
    # the lead car, on the recorded clock
    ri = lo + min(max(bisect.bisect_left(times, t0_rec + t), 0), len(times) - 1)
    present, d_rel, v_lead = (frames[ri]["lead"] + (0.0,))[:3]
    gap = s_of_frame[ri] + d_rel - s if present else 999.0
    if present and 0.0 < gap < 120:
      want = st["standstill_gap_m"] + st["gap_s"] * v
      vl = max(0.0, v_lead + 0.3 * (gap - want))
      if vl < target:
        target, why = vl, "lead"

    a_want = max(-st["decel_max"], min(st["accel_max"], (target - v) / 1.0))
    a_cmd = a_prev + (a_want - a_prev) * min(1.0, DT_MDL / 0.3)    # the planner's smoothing
    v = max(0.0, v + a_cmd * DT_MDL)
    if target == 0.0 and v < 0.3:
      v = 0.0
    a_prev = a_cmd
    s += v * DT_MDL
    t += DT_MDL
    why_t[why] = why_t.get(why, 0.0) + DT_MDL

    row = {"t": round(t, 2), "lat": round(lat, 6), "lon": round(lon, 6), "mph": round(v * MPH, 1),
           "rec_mph": round(fr["cs"][0] * MPH, 1), "state": "idle", "sign": None, "dist": None,
           "target_mph": round(min(target, 45.0) * MPH, 1), "confirmed": False, "too_late": False, "filt": 0.0, "turn": "",
           "logged_desire": fr["desire"], "wheel": round(fr["cs"][2], 1), "pressed": bool(fr["cs"][3]),
           "blinker": "", "lat_active": bool(fr["lat_active"]), "curv": round(fr["curv"], 4), "why": why}
    for key, val in row.items():
      cols[key].append(val)

  diff = [m - r for m, r in zip(cols["mph"], cols["rec_mph"], strict=True)]
  moving = [d for d, m, r in zip(diff, cols["mph"], cols["rec_mph"], strict=True) if max(m, r) > 3]
  return {"route": data["route"], "segs": data["segs"], "frames": len(cols["t"]), "mode": "style", "cols": cols,
          "events": [], "signs": [], "completed": s >= end_s - 1.0, "path_m": round(end_s, 1),
          "recorded_s": round(frames[hi]["t"] - frames[lo]["t"], 1), "simulated_s": round(t, 1),
          "style": st, "profile": profile_used, "why_s": {k: round(v_, 1) for k, v_ in sorted(why_t.items(), key=lambda x: -x[1])},
          "faster_5_pct": round(100 * sum(d > 5 for d in moving) / max(len(moving), 1)),
          "slower_5_pct": round(100 * sum(d < -5 for d in moving) / max(len(moving), 1)),
          "mean_abs_mph": round(sum(abs(d) for d in moving) / max(len(moving), 1), 1)}
