#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Reality View plan, Phase 0 (reports/"Tesla Reality View equivalent feasibility.md", "Unmeasured unknowns"): the
questions the drive logs can answer, measured over recorded drives on a PC.

  rv_phase0.py [--root DIR] [--segments N] [--routes R ...] [--json OUT] [--md OUT] [--jobs N]

It reads rlogs only (qlogs keep a sample of CAN and no modelV2). With --root it takes every drive in the catalog that
moved (catalog.json from catalog_routes.py, else every route folder) and N segments spread over each.

What it measures:
  1. The ADAS module's object list (ADAS_Obj1..8 on the ADAS side of the harness, bus 2): where its objects are
     (ahead, beside, behind), how far, how many slots are used, where tracks start and end, and whether it holds a
     car beside or behind when the car's blind-spot flag is on.
  2. The mid-range radar: whether its private link (panda bus 1) or radarTracks carry anything at all.
  3. The driving model on the comma: modelExecutionTime and frameDropPerc, and the device's CPU, GPU and memory,
     split by whether the video stream encoder (encoderd --stream) was running.
  4. Whether openpilot's leads are radar-backed (radarState.leadOne.radar while a lead is present).
  5. Track memory: how long the list keeps a car after it has passed us, and how far back.
"""
import argparse
import glob
import json
import math
import os
import sys
from collections import Counter
from concurrent.futures import ProcessPoolExecutor

# ADAS_Obj1..8 (HUD repo dbc/fisker_ocean_adas_world.dbc, tools/gen_world_dbc.py): non-sequential IDs, 40 ms
OBJ_MSGS = {1: 0x33B, 2: 0x34B, 3: 0x32D, 4: 0x33D, 5: 0x34D, 6: 0x32F, 7: 0x33F, 8: 0x34F}
ADAS_SIDE_BUS = 2          # the OEM ADAS module's side of the harness: what it sends, before the comma changes anything
RADAR_BUS = 1              # the mid-range radar's private CAN-FD link
RADAR_ADDRS = range(0x300, 0x330)
OBJ_CLASSES = {0: "car", 1: "truck", 2: "motorcycle", 3: "bicycle", 4: "pedestrian", 5: "animal",
               6: "unknown", 7: "small", 8: "large"}
TOWN_MPS, HIGHWAY_MPS = 15.6, 22.4     # 35 mph and 50 mph
MOVING_MPS = 1.0
TRACK_GAP_S = 1.0          # an ID unseen this long starts a new track
BESIDE_X = 5.0             # m: |longitudinal| within this and |lateral| over BESIDE_Y counts as beside
BESIDE_Y = 1.0
BSD_REAR_X = 2.0           # m: an object at most this far ahead on the flagged side counts as the blind-spot car
MAX_SAMPLES = 400_000      # per list, so a big run stays in memory


def decode_obj(dat: bytes) -> dict | None:
  """One ADAS_ObjN frame (8 bytes, Motorola) as {id, x, y, w, l, h, heading, cls, conf, class_conf}, or None for an
  empty slot (ID 0). x is m ahead (negative: behind), y m left, at the DBC's 0.2 m/bit."""
  if len(dat) < 8 or dat[0] == 0:
    return None
  long_d, lat_d = dat[1] * 0.2, dat[2] * 0.2
  x = -long_d if (dat[6] >> 6) & 1 else long_d
  y = lat_d if (dat[6] >> 7) & 1 else -lat_d     # sign 0 = right
  return {
    "id": dat[0], "x": round(x, 2), "y": round(y, 2),
    "w": round((dat[3] & 0x3F) * 0.1, 2), "l": round(dat[4] * 0.2, 2), "h": round((dat[6] & 0x3F) * 0.1, 2),
    "heading": dat[5] * 1.5, "cls": OBJ_CLASSES.get(dat[7] & 0x0F, "reserved"), "conf": (dat[3] >> 7) & 1,
    "class_conf": (dat[7] >> 5) & 3,
  }


def speed_bucket(v: float) -> str:
  if v < MOVING_MPS:
    return "parked"
  if v < TOWN_MPS:
    return "town"
  return "road" if v < HIGHWAY_MPS else "highway"


def percentiles(vals, qs=(5, 50, 90, 95, 99)) -> dict | None:
  """{n, min, max, mean, p5, ...} of a list, or None when empty."""
  v = sorted(x for x in vals if x is not None and math.isfinite(x))
  if not v:
    return None
  out = {"n": len(v), "min": round(v[0], 3), "max": round(v[-1], 3), "mean": round(sum(v) / len(v), 3)}
  for q in qs:
    k = (len(v) - 1) * q / 100
    lo, hi = math.floor(k), math.ceil(k)
    out[f"p{q}"] = round(v[lo] + (v[hi] - v[lo]) * (k - lo), 3)
  return out


def share(n: int, d: int) -> float | None:
  return round(n / d, 4) if d else None


class Tracks:
  """Object IDs over time in one segment: a track is one ID seen with no gap longer than TRACK_GAP_S."""

  def __init__(self):
    self.open: dict[int, dict] = {}
    self.done: list[dict] = []

  def see(self, t: float, o: dict) -> None:
    tr = self.open.get(o["id"])
    if tr is not None and t - tr["t1"] > TRACK_GAP_S:
      self.done.append(self.open.pop(o["id"]))
      tr = None
    if tr is None:
      tr = self.open[o["id"]] = {"t0": t, "t1": t, "x0": o["x"], "x1": o["x"], "xmin": o["x"], "behind_s": 0.0,
                                 "ahead_seen": o["x"] > BESIDE_X, "behind_t0": None, "cls": o["cls"]}
    if o["x"] < 0 and tr["ahead_seen"] and tr["behind_t0"] is None:
      tr["behind_t0"] = t
    tr["t1"], tr["x1"], tr["xmin"] = t, o["x"], min(tr["xmin"], o["x"])
    tr["ahead_seen"] = tr["ahead_seen"] or o["x"] > BESIDE_X
    if tr["behind_t0"] is not None:
      tr["behind_s"] = t - tr["behind_t0"]

  def close(self) -> list[dict]:
    self.done.extend(self.open.values())
    self.open = {}
    return self.done


def new_acc() -> dict:
  return {
    "segments": 0, "drives": set(), "seconds": Counter(), "snapshots": Counter(), "slots": Counter(),
    "obj": {b: {"n": 0, "behind": 0, "beside": 0, "x_ahead": [], "x_behind": [], "y_abs": [], "cls_behind": Counter()}
            for b in ("town", "road", "highway", "parked")},
    "tracks": {"n": 0, "first_behind": 0, "first_beside": 0, "life_s": [], "first_x": [], "last_x": [], "passed": 0,
               "passed_behind_s": [], "passed_xmin": []},
    "bsd": {"samples": 0, "with_obj": 0},
    "radar": {"bus1_frames": 0, "tracks_msgs": 0, "track_points": 0},
    "model": {"stream": {"exec_ms": [], "drop": [], "cpu": [], "gpu": [], "mem": []},
              "nostream": {"exec_ms": [], "drop": [], "cpu": [], "gpu": [], "mem": []}},
    "segments_stream": 0,
    "lead": {"present": 0, "radar": 0},
  }


def _push(lst: list, v) -> None:
  if len(lst) < MAX_SAMPLES:
    lst.append(v)


def analyze_segment(path: str) -> dict:
  """Everything this tool measures from one rlog, as a mergeable dict (see new_acc)."""
  from openpilot.tools.lib.logreader import LogReader   # imported here so the pure parts test without openpilot
  acc = new_acc()
  acc["segments"] = 1
  acc["drives"] = {os.path.basename(os.path.dirname(path)).rsplit("--", 1)[0]}
  slot_of = {addr: n for n, addr in OBJ_MSGS.items()}
  latest: dict[int, dict | None] = {}
  tracks = Tracks()
  v, bsd_l, bsd_r = 0.0, False, False
  stream = False
  model_rows, dev_rows = [], []
  last_t = None
  for m in LogReader(path):
    w = m.which()
    t = m.logMonoTime / 1e9
    if w == "carState":
      cs = m.carState
      v, bsd_l, bsd_r = cs.vEgo, cs.leftBlindspot, cs.rightBlindspot
      if last_t is not None:
        acc["seconds"][speed_bucket(v)] += min(t - last_t, 0.1)
      last_t = t
    elif w == "can":
      for c in m.can:
        if c.src == RADAR_BUS and c.address in RADAR_ADDRS:
          acc["radar"]["bus1_frames"] += 1
        if c.src != ADAS_SIDE_BUS or c.address not in slot_of:
          continue
        n = slot_of[c.address]
        latest[n] = decode_obj(bytes(c.dat))
        if n != 1:
          continue
        # slot 1 arrives every 40 ms: a snapshot of all eight slots
        objs = [o for o in latest.values() if o is not None]
        b = speed_bucket(v)
        acc["snapshots"][b] += 1
        acc["slots"][len(objs)] += 1
        g = acc["obj"][b]
        for o in objs:
          g["n"] += 1
          if o["x"] < 0:
            g["behind"] += 1
            g["cls_behind"][o["cls"]] += 1
            _push(g["x_behind"], o["x"])
          else:
            _push(g["x_ahead"], o["x"])
          if abs(o["x"]) <= BESIDE_X and abs(o["y"]) > BESIDE_Y:
            g["beside"] += 1
          _push(g["y_abs"], abs(o["y"]))
          if v >= MOVING_MPS:
            tracks.see(t, o)
        if v >= MOVING_MPS and (bsd_l or bsd_r):
          acc["bsd"]["samples"] += 1
          side = [o for o in objs if o["x"] <= BSD_REAR_X and ((bsd_l and o["y"] > BESIDE_Y) or (bsd_r and o["y"] < -BESIDE_Y))]
          acc["bsd"]["with_obj"] += bool(side)
    elif w == "radarTracks":
      acc["radar"]["tracks_msgs"] += 1
      acc["radar"]["track_points"] += len(m.radarTracks.points)
    elif w == "modelV2":
      model_rows.append((m.modelV2.modelExecutionTime * 1000.0, m.modelV2.frameDropPerc))
    elif w == "deviceState":
      ds = m.deviceState
      cpu = list(ds.cpuUsagePercent)
      dev_rows.append((sum(cpu) / len(cpu) if cpu else None, ds.gpuUsagePercent, ds.memoryUsagePercent))
    elif w == "radarState":
      lead = m.radarState.leadOne
      if lead.present:
        acc["lead"]["present"] += 1
        acc["lead"]["radar"] += bool(lead.radar)
    elif w == "procLog":
      stream = stream or any("--stream" in " ".join(p.cmdline) for p in m.procLog.procs)
  key = "stream" if stream else "nostream"
  acc["segments_stream"] = int(stream)
  mm = acc["model"][key]
  for e, d in model_rows:
    _push(mm["exec_ms"], e)
    _push(mm["drop"], d)
  for c, g_, mem in dev_rows:
    _push(mm["cpu"], c)
    _push(mm["gpu"], g_)
    _push(mm["mem"], mem)
  tr = acc["tracks"]
  for k in tracks.close():
    life = k["t1"] - k["t0"]
    if life < 0.2:     # a single frame or two: flicker, not a track
      continue
    tr["n"] += 1
    tr["first_behind"] += k["x0"] < 0
    tr["first_beside"] += 0 <= k["x0"] <= BESIDE_X
    _push(tr["life_s"], life)
    _push(tr["first_x"], k["x0"])
    _push(tr["last_x"], k["x1"])
    if k["behind_t0"] is not None:
      tr["passed"] += 1
      _push(tr["passed_behind_s"], k["behind_s"])
      _push(tr["passed_xmin"], k["xmin"])
  return acc


def merge(a: dict, b: dict) -> dict:
  """b folded into a (both from new_acc / analyze_segment)."""
  def fold(x, y):
    if isinstance(x, dict) and isinstance(y, dict):
      for k in y:
        x[k] = fold(x[k], y[k]) if k in x else y[k]
      return x
    if isinstance(x, set):
      return x | y
    if isinstance(x, list):
      x.extend(y[:max(0, MAX_SAMPLES - len(x))])
      return x
    return x + y
  return fold(a, b)


def summarize(acc: dict) -> dict:
  out = {"segments": acc["segments"], "drives": len(acc["drives"]),
         "minutes": {k: round(v / 60, 1) for k, v in acc["seconds"].items()}}
  objs = {}
  for b, g in acc["obj"].items():
    if not g["n"]:
      continue
    objs[b] = {"object_frames": g["n"], "snapshots": acc["snapshots"][b],
               "per_snapshot": round(g["n"] / max(acc["snapshots"][b], 1), 2),
               "behind_share": share(g["behind"], g["n"]), "beside_share": share(g["beside"], g["n"]),
               "x_ahead_m": percentiles(g["x_ahead"]), "x_behind_m": percentiles(g["x_behind"]),
               "lateral_abs_m": percentiles(g["y_abs"]), "behind_classes": dict(g["cls_behind"].most_common(6))}
  total_snap = sum(acc["slots"].values())
  out["adas_objects"] = {
    "by_speed": objs,
    "slots_used": {str(k): share(n, total_snap) for k, n in sorted(acc["slots"].items())},
    "all_eight_share": share(acc["slots"][8], total_snap),
  }
  tr = acc["tracks"]
  out["adas_tracks"] = {
    "tracks": tr["n"], "first_seen_behind_share": share(tr["first_behind"], tr["n"]),
    "first_seen_beside_share": share(tr["first_beside"], tr["n"]),
    "lifetime_s": percentiles(tr["life_s"]), "first_x_m": percentiles(tr["first_x"]), "last_x_m": percentiles(tr["last_x"]),
    "passed_us": tr["passed"], "kept_behind_s": percentiles(tr["passed_behind_s"]),
    "farthest_back_m": percentiles(tr["passed_xmin"]),
  }
  out["blind_spot"] = {"flagged_snapshots": acc["bsd"]["samples"],
                       "with_object_on_that_side_share": share(acc["bsd"]["with_obj"], acc["bsd"]["samples"])}
  out["radar"] = dict(acc["radar"])
  model = {}
  for k, mm in acc["model"].items():
    if not mm["exec_ms"] and not mm["cpu"]:
      continue
    drops = [d for d in mm["drop"] if d is not None]
    model[k] = {"model_exec_ms": percentiles(mm["exec_ms"]), "frame_drop_pct": percentiles(drops),
                "frames_with_drops_share": share(sum(1 for d in drops if d > 0), len(drops)),
                "cpu_pct": percentiles(mm["cpu"]), "gpu_pct": percentiles(mm["gpu"]), "mem_pct": percentiles(mm["mem"])}
  out["model"] = model
  out["segments_with_video_stream"] = acc["segments_stream"]
  out["leads"] = {"present_frames": acc["lead"]["present"], "radar_backed_share": share(acc["lead"]["radar"], acc["lead"]["present"])}
  return out


def _p(d, k="p50", unit="", nd=1):
  if not d:
    return "-"
  return f"{d[k]:.{nd}f}{unit}"


def markdown(s: dict, routes: list[str]) -> str:
  L = [f"# Phase 0 log measurements\n\n{s['drives']} drives, {s['segments']} one-minute segments "
       + f"(minutes by speed: {s['minutes']}).\n"]
  L.append("## ADAS object list (bus 2)\n")
  L.append("| Speed | Objects per frame | Behind | Beside | Ahead p50 / p95 / max (m) | Behind p50 / min (m) | Lateral p95 (m) |")
  L.append("|---|---|---|---|---|---|---|")
  for b, g in s["adas_objects"]["by_speed"].items():
    xa, xb = g["x_ahead_m"], g["x_behind_m"]
    behind, beside = 100 * (g["behind_share"] or 0), 100 * (g["beside_share"] or 0)
    L.append(f"| {b} | {g['per_snapshot']} | {behind:.1f}% | {beside:.1f}% | {_p(xa)} / {_p(xa, 'p95')} / {_p(xa, 'max')} | "
             + f"{_p(xb)} / {_p(xb, 'min')} | {_p(g['lateral_abs_m'], 'p95')} |")
  ao = s["adas_objects"]
  L.append(f"\nSlots used per frame: {ao['slots_used']} (all eight: {ao['all_eight_share']}).\n")
  t = s["adas_tracks"]
  L.append(f"Tracks (moving): {t['tracks']}; first seen behind {t['first_seen_behind_share']}, beside {t['first_seen_beside_share']}; "
           + f"lifetime p50 {_p(t['lifetime_s'], unit=' s')}, p90 {_p(t['lifetime_s'], 'p90', ' s')}; "
           + f"last seen at p5 / p50 {_p(t['last_x_m'], 'p5', ' m')} / {_p(t['last_x_m'], unit=' m')}. "
           + f"Passed us and kept behind: {t['passed_us']} tracks, kept p50 {_p(t['kept_behind_s'], unit=' s')} / "
           + f"p90 {_p(t['kept_behind_s'], 'p90', ' s')}, farthest back p50 {_p(t['farthest_back_m'], unit=' m')} / "
           + f"min {_p(t['farthest_back_m'], 'min', ' m')}.\n")
  bs = s["blind_spot"]
  L.append(f"Blind-spot flag on (moving): {bs['flagged_snapshots']} frames; an object on that side at or behind +{BSD_REAR_X:.0f} m in "
           + f"{bs['with_object_on_that_side_share']} of them.\n")
  r = s["radar"]
  L.append(f"## Radar\n\nBus 1 radar frames: {r['bus1_frames']}; radarTracks messages {r['tracks_msgs']} with {r['track_points']} points.\n")
  L.append("## Driving model and device\n")
  L.append("| Video stream | Model exec p50 / p95 / p99 (ms) | Frame drop p95 (%) | Frames with drops | CPU p50 / p95 (%) "
           + "| GPU p50 / p95 (%) | Mem p50 (%) |")
  L.append("|---|---|---|---|---|---|---|")
  for k, mm in s["model"].items():
    e, cpu, gpu = mm["model_exec_ms"], mm["cpu_pct"], mm["gpu_pct"]
    L.append(f"| {k} | {_p(e)} / {_p(e, 'p95')} / {_p(e, 'p99')} | {_p(mm['frame_drop_pct'], 'p95', nd=2)} | "
             + f"{mm['frames_with_drops_share']} | {_p(cpu)} / {_p(cpu, 'p95')} | {_p(gpu)} / {_p(gpu, 'p95')} | {_p(mm['mem_pct'])} |")
  L.append(f"\nSegments with the video stream encoder running: {s['segments_with_video_stream']}.\n")
  ld = s["leads"]
  L.append(f"## Leads\n\nLead present in {ld['present_frames']} radarState frames; radar-backed share {ld['radar_backed_share']}.\n")
  L.append("## Drives\n\n" + ", ".join(sorted(routes)) + "\n")
  return "\n".join(L)


def rlog(seg: str) -> str | None:
  for n in ("rlog.zst", "rlog.bz2", "rlog"):
    p = os.path.join(seg, n)
    if os.path.exists(p):
      return p
  return None


def spread(items: list, n: int) -> list:
  """n items spread evenly over the list (all of them when n >= len)."""
  if n >= len(items):
    return list(items)
  return [items[round(i * (len(items) - 1) / (n - 1))] for i in range(n)] if n > 1 else [items[len(items) // 2]]


def pick_segments(root: str, routes: list[str] | None, per_route: int, min_miles: float) -> list[str]:
  if not routes:
    # every route folder, less the catalog's drives that didn't move (routes newer than the catalog are kept)
    routes = sorted({os.path.basename(d).rsplit("--", 1)[0] for d in glob.glob(os.path.join(root, "*--*--*"))})
    cat = os.path.join(root, "catalog.json")
    if os.path.exists(cat):
      with open(cat) as f:
        still = {d["route"] for d in json.load(f)["drives"] if d.get("miles", 0) < min_miles}
      routes = [r for r in routes if r not in still]
  out = []
  for r in routes:
    segs = sorted(glob.glob(os.path.join(root, f"{r}--*")), key=lambda p: int(p.rsplit("--", 1)[1]))
    logs = [p for p in (rlog(s) for s in segs) if p]
    out.extend(spread(logs, per_route))
  return out


def main(argv: list[str] | None = None) -> int:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--root", default=os.path.expanduser("~/Fisker/routes"))
  ap.add_argument("--routes", nargs="*", help="route names (default: every drive in the catalog that moved)")
  ap.add_argument("--segments", type=int, default=6, help="segments per drive, spread over it")
  ap.add_argument("--min-miles", type=float, default=0.5)
  ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 1))
  ap.add_argument("--json", help="write the summary here")
  ap.add_argument("--md", help="write the markdown summary here")
  args = ap.parse_args(argv)
  segs = pick_segments(args.root, args.routes, args.segments, args.min_miles)
  if not segs:
    print("no rlogs found", file=sys.stderr)
    return 1
  print(f"{len(segs)} segments", file=sys.stderr)
  acc = new_acc()
  with ProcessPoolExecutor(args.jobs) as ex:
    for i, part in enumerate(ex.map(_safe_segment, segs, chunksize=1)):
      if part is not None:
        acc = merge(acc, part)
      if (i + 1) % 20 == 0:
        print(f"  {i + 1}/{len(segs)}", file=sys.stderr)
  s = summarize(acc)
  md = markdown(s, sorted(acc["drives"]))
  if args.json:
    with open(args.json, "w") as f:
      json.dump(s, f, indent=1)
  if args.md:
    with open(args.md, "w") as f:
      f.write(md)
  print(md)
  return 0


def _safe_segment(path: str) -> dict | None:
  try:
    return analyze_segment(path)
  except Exception as e:   # one unreadable segment must not lose the run
    print(f"  skipped {path}: {e}", file=sys.stderr)
    return None


if __name__ == "__main__":
  sys.exit(main())
