#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Fit the turn planner to the driver's own junction turns (run on the Mac): the ghost-path check (drive_sim
evaluate_turns: the planner's commanded curvature rolled through a kinematic car, against the recorded path) over every
recorded signaled turn, for a grid of start leads and arc radii, with the profile's turn-in and unwind rates. The start
lead is shared by both sides; the radius is chosen per side.

Why by simulation: measuring where the driver's wheel passes 30 deg against the map's junction node and solving the
planner's geometry for the lead gave +0.35 s, and the ghost path got worse with it (off-path 2.16 -> 3.62 m); the map
and GPS put the node a few metres off, the simulation doesn't depend on that.

Score per (lead, radius, side): the median of each triggered turn's largest distance from the driver's path, plus
MISS_PENALTY_M for each turn the planner didn't take (as a share of the side's turns).

  turn_tuning_fit.py [--root ~/Fisker/routes] [--profile FILE] [--write]
--write puts turn_start_lead_s, turn_radius_left_m and turn_radius_right_m into FILE (a driver profile JSON).
"""
import argparse
import json
import os
import re
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

import numpy as np

LEADS = (-0.9, -0.7, -0.45, -0.25, 0.0)
RADII = (8.0, 9.0, 10.0, 11.0, 12.0)
MISS_PENALTY_M = 3.0
WORKER = os.path.abspath(__file__)


def worker(route: str, root: str, turn_in: float, unwind: float) -> None:
  """One drive, every grid point: JSON lines on stdout."""
  from openpilot.sunnypilot.tools.drive_sim import sim
  from openpilot.sunnypilot.selfdrive.controls.lib import turn_planner as tp
  osm = os.path.expanduser("~/.comma/media/0/osm")
  try:
    data = sim.extract(root, route)
  except Exception:
    return
  for lead in LEADS:
    for r in RADII:
      tun = tp.TurnTuning()
      tun.lead_s, tun.radius_left_m, tun.radius_right_m = lead, r, r
      tun.turn_in_wheel_deg_s, tun.unwind_wheel_deg_s = turn_in, unwind
      try:
        ev = sim.evaluate_turns(data, tun, osm_root=osm, ghost_unwind_deg_s=120.0)
      except Exception:
        continue
      for t in ev["turns"]:
        print(json.dumps({"route": route, "lead": lead, "r": r, "side": t.get("side"), "triggered": bool(t["triggered"]),
                          "max_lat": t.get("max_lat_err")}), flush=True)


def fit(rows: list[dict]) -> dict:
  def score(sel):
    trig = [x["max_lat"] for x in sel if x["triggered"] and x["max_lat"] is not None]
    if not sel or not trig:
      return float("inf"), 0, len(sel)
    return float(np.median(trig)) + MISS_PENALTY_M * (1.0 - len(trig) / len(sel)), len(trig), len(sel)

  table = {}
  for lead in LEADS:
    for r in RADII:
      for side in ("left", "right"):
        table[(lead, r, side)] = score([x for x in rows if x["lead"] == lead and x["r"] == r and x["side"] == side])
  best = None
  for lead in LEADS:
    br = {side: min(RADII, key=lambda r, s=side: table[(lead, r, s)][0]) for side in ("left", "right")}
    total = sum(table[(lead, br[s], s)][0] * table[(lead, br[s], s)][2] for s in br)
    if best is None or total < best[0]:
      best = (total, lead, br)
  _, lead, br = best
  return {"lead": lead, "radius_left": br["left"], "radius_right": br["right"], "table": table}


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--root", default=os.path.expanduser("~/Fisker/routes"))
  ap.add_argument("--profile", default=None, help="driver profile JSON to read the turn-in / unwind from (and --write)")
  ap.add_argument("--write", action="store_true")
  ap.add_argument("--worker", nargs=4, metavar=("ROUTE", "ROOT", "TURN_IN", "UNWIND"), help=argparse.SUPPRESS)
  a = ap.parse_args()
  if a.worker:
    route, root, tin, uw = a.worker
    worker(route, root, float(tin), float(uw))
    return
  turn_in = unwind = 0.0
  prof = None
  if a.profile:
    with open(a.profile) as f:
      prof = json.load(f)
    turn_in = float(prof.get("values", {}).get("turn_in_deg_s", 0.0))
    unwind = float(prof.get("values", {}).get("turn_unwind_deg_s", 0.0))
  routes = sorted({m.group(0) for d in os.listdir(a.root) for m in [re.match(r"^[0-9a-f]{8}--[0-9a-f]{10}", d)] if m})

  def run(r):
    p = subprocess.run([sys.executable, WORKER, "--worker", r, a.root, str(turn_in), str(unwind)], capture_output=True, text=True)
    return [json.loads(line) for line in p.stdout.splitlines() if line.startswith("{")]

  with ThreadPoolExecutor(8) as ex:
    rows = [x for rs in ex.map(run, routes) for x in rs]
  res = fit(rows)
  print(f"{len(rows) // (len(LEADS) * len(RADII))} recorded turns; turn-in {turn_in:.0f} deg/s, unwind {unwind:.0f} deg/s")
  print(f"{'lead':>6s} {'radius':>6s}  {'left: score, taken/turns':26s} {'right: score, taken/turns':26s}")
  for lead in LEADS:
    for r in RADII:
      lt, rt = res["table"][(lead, r, "left")], res["table"][(lead, r, "right")]
      print(f"{lead:+6.2f} {r:6.1f}  {lt[0]:6.2f} m  {lt[1]:3d}/{lt[2]:<3d}            {rt[0]:6.2f} m  {rt[1]:3d}/{rt[2]:<3d}")
  print(f"best: lead {res['lead']:+.2f} s, radius left {res['radius_left']:.0f} m, right {res['radius_right']:.0f} m")
  if a.write and prof is not None:
    prof.setdefault("values", {}).update({"turn_start_lead_s": res["lead"], "turn_radius_left_m": res["radius_left"],
                                          "turn_radius_right_m": res["radius_right"]})
    prof.setdefault("meta", {}).setdefault("values", {})["turn_tuning_fit"] = {
      "lead": res["lead"], "radius_left": res["radius_left"], "radius_right": res["radius_right"],
      "turns": len(rows) // (len(LEADS) * len(RADII)),
      "note": "ghost-path fit to the driver's recorded turns (tools/turn_tuning_fit.py)"}
    with open(a.profile, "w") as f:
      json.dump(prof, f, indent=1)
    print("wrote", a.profile)


if __name__ == "__main__":
  main()
