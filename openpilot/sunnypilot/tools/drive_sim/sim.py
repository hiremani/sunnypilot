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
  from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_direction import Applies, sign_applies
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

  # every sign near the driven path, with its verdict for the car's bearing where it passed closest
  signs = {}
  for lat, lon, _ in track_pts[::2]:
    for s in smap.signs_near(lat, lon):
      if s.node_id not in signs and _dist_m(lat, lon, s.lat, s.lon) <= SIGN_NEAR_M:
        signs[s.node_id] = s
  sign_out = []
  for s in signs.values():
    plat, plon, pbear = min(track_pts, key=lambda p: _dist_m(p[0], p[1], s.lat, s.lon))
    verdict = sign_applies(s, pbear)
    sign_out.append({"id": s.node_id, "lat": s.lat, "lon": s.lon, "direction": s.direction, "all_way": s.all_way,
                     "approaches": list(s.approaches) if s.approaches else None,
                     "verdict": {Applies.YES: "applies", Applies.NO: "not_ours", Applies.UNKNOWN: "unknown"}[verdict],
                     "pass_bearing": round(pbear, 1), "pass_dist": round(_dist_m(plat, plon, s.lat, s.lon), 1)})

  return {"route": data["route"], "segs": data["segs"], "frames": len(data["frames"]), "cols": cols, "events": events,
          "signs": sign_out}
