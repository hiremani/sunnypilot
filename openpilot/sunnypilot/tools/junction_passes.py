"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The driver's junction turns, for the junction memory (selfdrive/controls/lib/junction_memory.py): every low-speed turn
of 55-130 deg the driver steered (openpilot not steering), with or without the blinker, found from carState (speed and
yaw rate); the GPS fixes at 3% and 97% of its heading change, map-matched to the junction the turn left from.

  junction_passes.py [--root R] [--osm O] [--profile me] [--write] ROUTE...
prints one JSON line per pass; --write saves the memory (junctions seen at least junction_memory.MIN_PASSES times).
"""
import argparse
import bisect
import glob
import json
import math
import os
import sys

from openpilot.tools.lib.logreader import LogReader
from openpilot.sunnypilot.selfdrive.controls.lib import junction_memory as jm
from openpilot.sunnypilot.selfdrive.controls.lib import turn_planner as tp

TURN_YAW = 0.12            # rad/s: a turn is under way
TURN_END_YAW = 0.05
EDGE_YAW = 0.02            # the turn's ends: back to straight
MAX_V = 11.0               # m/s: junction turns only (the turn planner arms at ~25 mph)
DEG_RANGE = (55.0, 130.0)
ON, OFF = 0.03, 0.97       # the start and the finish: this fraction of the heading change
MAX_GPS_GAP_S = 1.0
GPS_MAX_ACC_M = 10.0
DRIVER_MAX_OP = 0.2        # openpilot steered at most this fraction of the turn


def _logs(root: str, route: str) -> list[str]:
  segs = sorted(glob.glob(os.path.join(root, f"{route}--*")), key=lambda p: int(p.rsplit("--", 1)[1]))
  out = []
  for s in segs:
    for name in ("rlog.zst", "rlog"):
      f = os.path.join(s, name)
      if os.path.exists(f):
        out.append(f)
        break
  return out


def turns(cs: list[tuple]) -> list[tuple[int, int, int]]:
  """[(sign, a, b)]: index ranges of the junction turns in cs rows (t, v, yaw, lb, rb, op_steering, reverse)."""
  out, n, i = [], len(cs), 0
  while i < n:
    if abs(cs[i][2]) < TURN_YAW or cs[i][1] > MAX_V or cs[i][6]:
      i += 1
      continue
    sgn = 1 if cs[i][2] > 0 else -1
    j, gap = i, 0
    while j + 1 < n and gap < 100:
      j += 1
      gap = gap + 1 if cs[j][2] * sgn < TURN_END_YAW else 0
    j -= gap
    a = i
    while a > 0 and cs[a][2] * sgn > EDGE_YAW and cs[i][0] - cs[a][0] < 6:
      a -= 1
    b = j
    while b < n - 1 and cs[b][2] * sgn > EDGE_YAW and cs[b][0] - cs[j][0] < 6:
      b += 1
    out.append((sgn, a, b))
    i = b + 1
  return out


def passes(root: str, route: str, src) -> list[dict]:
  cs, gps, lat = [], [], False
  for f in _logs(root, route):
    try:
      for m in LogReader(f):
        w = m.which()
        if w == "carControl":
          lat = m.carControl.latActive
        elif w == "carState":
          c = m.carState
          cs.append((m.logMonoTime * 1e-9, c.vEgo, c.yawRate, c.leftBlinker, c.rightBlinker, lat and not c.steeringPressed,
                     str(c.gearShifter) == "reverse"))
        elif w == "gpsLocationExternal":
          g = m.gpsLocationExternal
          if g.hasFix and g.horizontalAccuracy < GPS_MAX_ACC_M:
            gps.append((m.logMonoTime * 1e-9, g.latitude, g.longitude, g.bearingDeg, g.speed))
    except Exception as e:   # a bad segment: skip it
      print(f"skip {f}: {e}", file=sys.stderr)
  if not gps or not cs:
    return []
  gt = [g[0] for g in gps]

  def fix_at(t):
    k = min(bisect.bisect_left(gt, t), len(gps) - 1)
    return gps[k] if abs(gps[k][0] - t) < MAX_GPS_GAP_S else None

  out = []
  for sgn, a, b in turns(cs):
    seg = cs[a:b + 1]
    hs, h = [0.0], 0.0
    for k in range(1, len(seg)):
      h -= math.degrees(seg[k][2]) * (seg[k][0] - seg[k - 1][0])
      hs.append(h)
    total = abs(hs[-1])
    if not DEG_RANGE[0] <= total <= DEG_RANGE[1] or max(r[1] for r in seg) > MAX_V + 1 or any(r[6] for r in seg):
      continue
    k0 = next(k for k, v in enumerate(hs) if abs(v) >= ON * total)
    k1 = next(k for k, v in enumerate(hs) if abs(v) >= OFF * total)
    if sum(r[5] for r in seg[k0:k1]) > DRIVER_MAX_OP * max(1, k1 - k0):
      continue
    side = "left" if sgn > 0 else "right"
    g0, g1 = fix_at(seg[k0][0]), fix_at(seg[k1][0])
    pre = next((g for g in reversed(gps) if g[0] <= seg[k0][0] and g[4] > 2.0 and seg[k0][0] - g[0] < 8.0), None)
    if g0 is None or g1 is None or pre is None:
      continue
    roads = src.get(g0[1], g0[2])
    if roads is None:
      continue
    x0, y0 = roads.to_xy(g0[1], g0[2])
    m = roads.match(x0, y0, pre[3])
    if m is None:
      continue
    j = roads.junction_ahead(m, x0, y0, side, 60.0, 0.0)
    if j is None or abs(abs(j.branch.change) - total) > 30.0:
      continue
    hb, eb = math.radians(j.arrival_bearing), math.radians(j.branch.bearing)
    x1, y1 = roads.to_xy(g1[1], g1[2])
    nlat, nlon = roads.to_latlon(j.x, j.y)
    out.append({"key": jm.key(roads.ways[m.way].way_id, j.branch.way_id, nlat, nlon), "route": route, "side": side,
                "t": round(seg[k0][0] - cs[0][0], 1), "deg": round(total),
                "blinker": any(r[3] if side == "left" else r[4] for r in seg),
                "d_on": round((j.x - x0) * math.sin(hb) + (j.y - y0) * math.cos(hb), 2),
                "lat_on": round((x0 - j.x) * math.cos(hb) - (y0 - j.y) * math.sin(hb), 2),
                "d_out": round((x1 - j.x) * math.sin(eb) + (y1 - j.y) * math.cos(eb), 2),
                "lat_out": round((x1 - j.x) * math.cos(eb) - (y1 - j.y) * math.sin(eb), 2)})
  return out


def main(argv: list[str] | None = None) -> int:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("routes", nargs="+")
  ap.add_argument("--root", default="/data/media/0/realdata")
  ap.add_argument("--osm", default="/data/media/0/osm/offline")
  ap.add_argument("--profile", default="me")
  ap.add_argument("--write", action="store_true", help="save the junction memory for --profile")
  a = ap.parse_args(argv)
  src = tp.RoadSource(root=a.osm, sync=True)
  allp = []
  for r in a.routes:
    for p in passes(a.root, r, src):
      print(json.dumps(p), flush=True)
      allp.append(p)
  if a.write:
    mem = jm.build([p for p in allp if jm.clean(p) is not None])
    print("saved", jm.save(mem, a.profile, {"routes": a.routes, "passes": len(allp)}), len(mem), "junctions", file=sys.stderr)
  return 0


if __name__ == "__main__":
  sys.exit(main())
