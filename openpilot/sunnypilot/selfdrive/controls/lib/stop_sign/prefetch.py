#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Download the map stop signs (webhud MapFeatures tiles) for an area ahead of time, on Wi-Fi, so Stop Sign Assist
doesn't depend on fetching them mid-drive.

  prefetch.py --around LAT LON [--radius-km 15]
  prefetch.py --route 000000e3--e8d98c1676        (every tile a recorded drive passed through)
"""
import argparse
import glob
import math
import sys
import time

from openpilot.sunnypilot.webhud.maptiles import FEAT_TILE_DEG, MapFeatures

KM_PER_DEG_LAT = 111.32


def tiles_around(lat: float, lon: float, radius_km: float) -> list[tuple[int, int]]:
  dlat = radius_km / KM_PER_DEG_LAT
  dlon = radius_km / (KM_PER_DEG_LAT * max(math.cos(math.radians(lat)), 0.01))
  t0, t1 = math.floor((lat - dlat) / FEAT_TILE_DEG), math.floor((lat + dlat) / FEAT_TILE_DEG)
  u0, u1 = math.floor((lon - dlon) / FEAT_TILE_DEG), math.floor((lon + dlon) / FEAT_TILE_DEG)
  return [(t, u) for t in range(t0, t1 + 1) for u in range(u0, u1 + 1)]


def tiles_for_route(route: str) -> list[tuple[int, int]]:
  from openpilot.tools.lib.logreader import LogReader
  tiles: set[tuple[int, int]] = set()
  for seg in sorted(glob.glob(f"/data/media/0/realdata/{route}--*")):
    for m in LogReader(f"{seg}/qlog.zst" if glob.glob(f"{seg}/qlog.zst") else f"{seg}/rlog.zst"):
      w = m.which()
      if w in ("gpsLocationExternal", "gpsLocation"):
        g = getattr(m, w)
        if g.hasFix:
          tiles.add((math.floor(g.latitude / FEAT_TILE_DEG), math.floor(g.longitude / FEAT_TILE_DEG)))
  return sorted(tiles)


def main() -> int:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--around", nargs=2, type=float, metavar=("LAT", "LON"))
  ap.add_argument("--radius-km", type=float, default=15.0)
  ap.add_argument("--route")
  ap.add_argument("--pause-s", type=float, default=2.0, help="between Overpass requests, to be polite")
  args = ap.parse_args()
  if bool(args.around) == bool(args.route):
    ap.error("give exactly one of --around or --route")

  tiles = tiles_around(*args.around, args.radius_km) if args.around else tiles_for_route(args.route)
  features = MapFeatures()
  have = [t for t in tiles if features.have_tile(*t)]
  todo = [t for t in tiles if not features.have_tile(*t)]
  print(f"{len(tiles)} tiles: {len(have)} already on disk, {len(todo)} to fetch")
  failed = []
  for i, t in enumerate(todo):
    ok = features.ensure_tile(*t)
    print(f"[{i + 1}/{len(todo)}] tile {t}: {'ok' if ok else 'FAILED'}", flush=True)
    if not ok:
      failed.append(t)
    if i + 1 < len(todo):
      time.sleep(args.pause_s)
  print(f"done: {len(todo) - len(failed)} fetched, {len(failed)} failed {failed if failed else ''}")
  return 1 if failed else 0


if __name__ == "__main__":
  sys.exit(main())
