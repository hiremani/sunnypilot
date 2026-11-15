#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Learn speed bumps from the car's own jolts: compare what the drives felt with what the map says.

  bump_learn.py ROUTE... [--root DIR] [--features DIR] [--write]

The accelerometer's peak-to-peak between GPS fixes is the jolt; the median over the drive is the road's baseline.
  * Mapped bumps never felt: every pass over a mapped bump (Speed Bump Assist's kinds) at least MIN_V, and none jolted
    more than FELT_RATIO x the baseline -> suggested for "ignore" (a removed bump, or one mapped on a parking aisle
    beside the road).
  * Felt, not mapped: jolts of at least SPIKE_RATIO x the baseline (and SPIKE_MIN), on at least MIN_PASSES separate
    passes within CLUSTER_M of each other, more than NEAR_MAPPED_M from any mapped bump -> suggested as local bumps.
    Potholes, rail crossings and driveway lips jolt too: these are suggestions for the driver to confirm.
Prints the suggestions; --write merges them into <mapd root>/local_speed_bumps.json (ids negative, existing entries
kept).
"""
import argparse
import glob
import json
import math
import os
import statistics
import sys

from openpilot.tools.lib.logreader import LogReader
from openpilot.sunnypilot.selfdrive.controls.lib import speed_bump as sb

MIN_V = 3.0            # m/s: slower passes say little either way
FELT_RATIO = 2.0
SPIKE_RATIO = 4.0      # x the road around it (LOCAL_N fixes either side, moving): an isolated jolt, not a rough road
SPIKE_MIN = 3.0        # m/s^2 peak-to-peak
SPIKE_MAX_V = 16.0     # m/s (35 mph): bumps are on slow streets; faster jolts are joints and potholes
LOCAL_N = 40
CLUSTER_M = 12.0
MIN_PASSES = 2
PASS_GAP_S = 60.0      # spikes this far apart in time are separate passes
NEAR_MAPPED_M = 15.0
ON_BUMP_M = 6.0
MPH = 2.23694


def xy(lat, lon, lat0, lon0):
  return math.radians(lon - lon0) * math.cos(math.radians(lat0)) * sb.EARTH_R, math.radians(lat - lat0) * sb.EARTH_R


def samples(root: str, route: str) -> list[tuple]:
  """[(t, lat, lon, v, jolt)] at each GPS fix."""
  out, v, az = [], 0.0, []
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
      if w == "carState":
        v = m.carState.vEgo
      elif w == "accelerometer":
        a = m.accelerometer.acceleration.v
        if len(a):
          az.append(a[0])
      elif w == "gpsLocationExternal":
        g = m.gpsLocationExternal
        if g.hasFix and g.horizontalAccuracy < 10:
          out.append((m.logMonoTime * 1e-9, g.latitude, g.longitude, v, (max(az) - min(az)) if az else 0.0))
        az = []
  return out


def mapped_bumps(features_dir: str) -> list[sb.SpeedBump]:
  out = []
  for f in glob.glob(os.path.join(features_dir, "*.json")):
    try:
      with open(f) as fh:
        out += sb.bumps_from_cell(json.load(fh))
    except (OSError, ValueError):
      continue
  return out


def learn(routes: list[str], root: str, features_dir: str) -> tuple[dict, list]:
  bumps = mapped_bumps(features_dir)
  passes: dict[int, list[tuple[float, float]]] = {}   # mapped bump id -> [(jolt / baseline, v)]
  spikes = []                                        # (lat, lon, t, ratio)
  for r in routes:
    S = samples(root, r)
    if len(S) < 50:
      continue
    moving = [s[4] for s in S if s[3] >= MIN_V]
    if len(moving) < 50:
      continue
    base = max(statistics.median(moving), 0.05)
    lat0, lon0 = S[0][1], S[0][2]
    pts = [xy(s[1], s[2], lat0, lon0) for s in S]
    near = [(b, *xy(b.lat, b.lon, lat0, lon0)) for b in bumps]
    near = [n for n in near if min(abs(n[1] - p[0]) + abs(n[2] - p[1]) for p in pts[::20]) < 300]
    for b, bx, by in near:
      idx = [k for k, p in enumerate(pts) if math.hypot(p[0] - bx, p[1] - by) < ON_BUMP_M]
      if not idx:
        continue
      groups, cur = [], [idx[0]]
      for k in idx[1:]:
        if S[k][0] - S[cur[-1]][0] > PASS_GAP_S:
          groups.append(cur)
          cur = []
        cur.append(k)
      groups.append(cur)
      for g in groups:
        lo, hi = max(0, g[0] - 3), min(len(S), g[-1] + 4)
        passes.setdefault(b.node_id, []).append((max(s[4] for s in S[lo:hi]) / base, max(S[k][3] for k in g)))
    for i, s in enumerate(S):
      if not (MIN_V <= s[3] <= SPIKE_MAX_V) or s[4] < SPIKE_MIN:
        continue
      around = [x[4] for x in S[max(0, i - LOCAL_N):i - 2] + S[i + 3:i + LOCAL_N] if x[3] >= MIN_V]
      local = max(statistics.median(around), 0.05) if len(around) >= 10 else base
      if s[4] >= SPIKE_RATIO * local:
        spikes.append((s[1], s[2], s[0], s[4] / local))

  never_felt = {bid: p for bid, p in passes.items()
                if len([x for x in p if x[1] >= MIN_V]) >= MIN_PASSES and all(x[0] < FELT_RATIO for x in p if x[1] >= MIN_V)}
  # cluster the spikes, keep places felt on separate passes and away from mapped bumps
  clusters: list[list[tuple]] = []
  for sp in spikes:
    for c in clusters:
      if math.hypot(*xy(sp[0], sp[1], c[0][0], c[0][1])) < CLUSTER_M:
        c.append(sp)
        break
    else:
      clusters.append([sp])
  felt = []
  for c in clusters:
    times = sorted(x[2] for x in c)
    n_pass = 1 + sum(1 for a, b in zip(times, times[1:], strict=False) if b - a > PASS_GAP_S)
    lat, lon = statistics.mean(x[0] for x in c), statistics.mean(x[1] for x in c)
    if n_pass < MIN_PASSES:
      continue
    if any(math.hypot(*xy(b.lat, b.lon, lat, lon)) < NEAR_MAPPED_M for b in bumps):
      continue
    felt.append({"lat": round(lat, 7), "lon": round(lon, 7), "passes": n_pass, "jolt_x": round(max(x[3] for x in c), 1)})
  return never_felt, felt


def main(argv=None) -> int:
  from openpilot.common.hardware.hw import Paths
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("routes", nargs="+")
  ap.add_argument("--root", default="/data/media/0/realdata")
  ap.add_argument("--features", default=os.path.join(Paths.mapd_root(), "features"))
  ap.add_argument("--write", action="store_true", help="merge the suggestions into local_speed_bumps.json")
  args = ap.parse_args(argv)
  never_felt, felt = learn(args.routes, args.root, args.features)
  print(f"mapped bumps never felt ({len(never_felt)}):")
  for bid, p in sorted(never_felt.items()):
    print(f"  {bid}: {len(p)} passes, jolt x{max(x[0] for x in p):.1f} of the road's, up to {max(x[1] for x in p) * MPH:.0f} mph")
  print(f"felt but not mapped ({len(felt)}):")
  for f in felt:
    print(f"  {f['lat']}, {f['lon']}: {f['passes']} passes, jolt x{f['jolt_x']}")
  if args.write and (never_felt or felt):
    path = os.path.join(os.path.dirname(os.path.normpath(args.features)), sb.LOCAL_FILE)
    data = {"bumps": [], "ignore": []}
    if os.path.isfile(path):
      with open(path) as fh:
        data = json.load(fh)
    data.setdefault("bumps", [])
    data.setdefault("ignore", [])
    data["ignore"] = sorted(set(data["ignore"]) | set(never_felt))
    next_id = min([-1] + [int(b["id"]) - 1 for b in data["bumps"] if int(b.get("id", 0)) < 0])
    for f in felt:
      data["bumps"].append({"id": next_id, "lat": f["lat"], "lon": f["lon"], "kind": "bump",
                            "note": f"learned: {f['passes']} passes, jolt x{f['jolt_x']}"})
      next_id -= 1
    with open(path, "w") as fh:
      json.dump(data, fh, indent=1)
    print(f"wrote {path}")
  return 0


if __name__ == "__main__":
  sys.exit(main())
