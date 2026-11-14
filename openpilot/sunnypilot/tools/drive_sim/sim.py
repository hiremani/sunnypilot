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
    if data.get("stamp") == stamp and data.get("version") == 2:
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
        cur["lead"] = (bool(lead.present), float(lead.dRel))
      elif w == "carControl":
        cur["lat"] = m.carControl.latActive
      elif w == "modelV2" and "gps" in cur and "cs" in cur:
        md = m.modelV2
        ds = list(md.meta.desireState)
        frames.append({
          "t": (m.logMonoTime - t0) / 1e9, "gps": cur["gps"], "cs": cur["cs"], "lead": cur.get("lead", (False, 0.0)),
          "lat_active": cur.get("lat", False), "stop": md.action.shouldStop, "curv": md.action.desiredCurvature,
          "vel": [round(v, 3) for v in md.velocity.x], "pos": [round(p, 2) for p in md.position.x],
          "desire": max(range(len(ds)), key=lambda i: ds[i]) if ds else 0,
        })
  data = {"version": 2, "stamp": stamp, "route": route, "segs": segs, "service": service or "gpsLocation", "frames": frames}
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
