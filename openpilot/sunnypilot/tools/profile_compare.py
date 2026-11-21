#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Compare two or more driver profiles on recorded drives (run on the Mac). Each profile drives every chosen route in
drive_sim, for both things it changes:

  speed  simulate_style: openpilot longitudinal with that profile's numbers on the recorded road, lead and stops,
         against the recorded speed (mean |difference| while moving, share of time >5 mph faster / slower, time taken)
  turns  evaluate_turns: the turn planner with that profile's turn values (turn_planner.apply_profile, as controlsd),
         a ghost car through what it commands, against the recorded path at every signaled turn (taken / turns, median
         and worst largest distance off the path)

The recorded drive is the reference, so this says which profile drives most like the recording. On drives where the
assists were in charge the recording is partly the car's own driving; --unassisted keeps only drives where openpilot
steered less than 20% of the moving time.

Profiles (SPEC, optionally LABEL=SPEC):
  default             the built-in values
  me                  a live profile by name (~/.comma/media/0/driver_profiles/me.json)
  me@0, me@1, @2      a kept version (profile_versions.py) by position, newest first (no name = me)
  me@20261121-061500-1a2b3c4d   a kept version by id
  path/to/file.json   a profile file

  profile_compare.py SPEC SPEC [...] [--routes R ...] [--last N] [--root ~/Fisker/routes] [--json OUT] [--pull]
--pull first copies the comma's kept versions (a few KB) to the Mac.
"""
import argparse
import json
import os
import re
import statistics
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor

COMMA = "comma@192.168.0.207"
COMMA_PROFILES = "/data/media/0/driver_profiles"
MIN_MILES = 0.3
MIN_MOVING_MIN = 0.5
ROUTE_RE = re.compile(r"^([0-9a-f]{8}--[0-9a-f]{10})--\d+$")


def resolve(spec: str):
  """(label, DriverProfile, description) for one SPEC."""
  from openpilot.sunnypilot.selfdrive.controls.lib import driver_profile as dp
  from openpilot.sunnypilot.selfdrive.controls.lib import profile_versions as pv
  label, _, body = spec.rpartition("=") if "=" in spec else ("", "", spec)
  if body == "default":
    return label or "default", dp.DriverProfile(), "built-in values"
  if body.endswith(".json") or os.sep in body:
    d = dp.read_file(os.path.expanduser(body))
    if d is None:
      raise SystemExit(f"not a readable profile: {body}")
    return label or os.path.basename(body)[:-5], dp.from_dict(d), body
  if "@" in body:
    name, _, which = body.partition("@")
    name = name or dp.DEFAULT_NAME
    vs = pv.list_versions("profile", name)
    if which.isdigit():
      if int(which) >= len(vs):
        raise SystemExit(f"{name} has {len(vs)} kept versions, no @{which}")
      v = vs[int(which)]
    else:
      v = next((x for x in vs if x["id"] == which), None)
      if v is None:
        raise SystemExit(f"no version {which} of {name}")
    d = dp.read_file(pv.version_path("profile", name, v["id"]))
    return label or f"{name}@{which}", dp.from_dict(d), f"{v['id']} {v['label']}".strip()
  if not os.path.isfile(dp.profile_path(body)):
    raise SystemExit(f"no profile named {body} in {dp.profiles_dir()}")
  return label or body, dp.load(body), f"live {body}.json"


def pull_versions() -> None:
  from openpilot.sunnypilot.selfdrive.controls.lib import driver_profile as dp
  os.makedirs(dp.profiles_dir(), exist_ok=True)
  r = subprocess.run(["scp", "-q", "-r", "-o", "ConnectTimeout=5", f"{COMMA}:{COMMA_PROFILES}/.versions", dp.profiles_dir() + "/"],
                     capture_output=True, text=True, timeout=120)
  print("pulled the comma's kept versions" if r.returncode == 0 else f"could not pull versions: {r.stderr.strip()}")


def pick_routes(root: str, routes: list[str] | None, last: int) -> list[str]:
  have = sorted({m.group(1) for d in os.listdir(root) for m in [ROUTE_RE.match(d)] if m})
  if routes:
    missing = [r for r in routes if r not in have]
    if missing:
      raise SystemExit(f"not under {root}: {' '.join(missing)}")
    return routes
  parked = set()
  try:   # ~/Fisker/bin/catalog_routes.py: skip the comma's parked recordings (no miles) when picking the newest
    with open(os.path.join(root, "catalog.json")) as f:
      parked = {d["route"] for d in json.load(f).get("drives", []) if d.get("miles", 1.0) < MIN_MILES}
  except (OSError, ValueError, KeyError, TypeError):
    pass
  return [r for r in have if r not in parked][-last:]


def run_route(route: str, root: str, variants: list[tuple[str, object]], osm_root: str) -> dict:
  """Every variant on one route: {route, assisted_pct, results: {label: {...}}} or {route, error}."""
  from openpilot.sunnypilot.tools.drive_sim import sim
  from openpilot.sunnypilot.selfdrive.controls.lib import turn_planner as tp
  try:
    data = sim.extract(root, route)
  except Exception as e:
    return {"route": route, "error": f"extract: {e}"}
  frames = data["frames"]
  moving = [f for f in frames if f["cs"][0] > 1.0]
  assisted = round(100 * sum(bool(f["lat_active"]) for f in moving) / max(len(moving), 1))
  moving_min = round(len(moving) * 0.05 / 60, 1)
  if moving_min < MIN_MOVING_MIN:
    return {"route": route, "error": "parked: no driving to compare"}
  out = {"route": route, "assisted_pct": assisted, "moving_min": moving_min, "results": {}}
  for label, prof in variants:
    res: dict = {}
    try:
      s = sim.simulate_style(data, driver_profile=prof, label=label)
      res.update({"mean_abs_mph": s["mean_abs_mph"], "faster_5_pct": s["faster_5_pct"], "slower_5_pct": s["slower_5_pct"],
                  "time_pct": round(100 * s["simulated_s"] / max(s["recorded_s"], 1.0)), "completed": s["completed"]})
    except Exception as e:
      res["speed_error"] = str(e)[:120]
    try:
      tun = tp.TurnTuning()
      tp.apply_profile(tun, prof)
      ev = sim.evaluate_turns(data, tun, osm_root=osm_root, ghost_unwind_deg_s=120.0)
      res["turns"] = [{"side": t["side"], "taken": bool(t["triggered"]), "max_lat": t["max_lat_err"]} for t in ev["turns"]]
    except Exception as e:
      res["turn_error"] = str(e)[:120]
    out["results"][label] = res
  return out


def summarize(rows: list[dict], labels: list[str]) -> dict:
  """Across routes, per label: speed numbers weighted by moving time, turns pooled."""
  out = {}
  for label in labels:
    w = speed = fast = slow = tpct = 0.0
    turns = []
    for r in rows:
      res = r.get("results", {}).get(label)
      if not res:
        continue
      if "mean_abs_mph" in res:
        m = r["moving_min"]
        w += m
        speed += m * res["mean_abs_mph"]
        fast += m * res["faster_5_pct"]
        slow += m * res["slower_5_pct"]
        tpct += m * res["time_pct"]
      turns += res.get("turns", [])
    taken = [t["max_lat"] for t in turns if t["taken"] and t["max_lat"] is not None]
    out[label] = {"moving_min": round(w, 1),
                  "mean_abs_mph": round(speed / w, 2) if w else None, "faster_5_pct": round(fast / w) if w else None,
                  "slower_5_pct": round(slow / w) if w else None, "time_pct": round(tpct / w) if w else None,
                  "turns": len(turns), "taken": len(taken),
                  "median_off_m": round(statistics.median(taken), 2) if taken else None,
                  "worst_off_m": round(max(taken), 2) if taken else None}
  return out


def _fmt(v, suffix=""):
  return "–" if v is None else f"{v}{suffix}"


def print_report(rows: list[dict], summary: dict, descs: dict) -> None:
  labels = list(summary)
  w = max(8, *(len(x) for x in labels))
  print()
  for label in labels:
    print(f"  {label:{w}s}  {descs[label]}")
  print(f"\n{len([r for r in rows if 'error' not in r])} drives, {sum(r.get('moving_min', 0) for r in rows):.0f} min moving")
  head = [f"{'speed |diff|':>12s}", f"{'>5 faster':>9s}", f"{'>5 slower':>9s}", f"{'time':>6s}  ", f"{'turns taken':>11s}",
          f"{'median off':>10s}", f"{'worst off':>9s}"]
  print(f"\n{'':{w}s}  " + " ".join(head))
  best_speed = min((s["mean_abs_mph"] for s in summary.values() if s["mean_abs_mph"] is not None), default=None)
  best_turn = min((s["median_off_m"] for s in summary.values() if s["median_off_m"] is not None), default=None)
  for label, s in summary.items():
    ms = "*" if s["mean_abs_mph"] is not None and s["mean_abs_mph"] == best_speed and len(summary) > 1 else " "
    mt = "*" if s["median_off_m"] is not None and s["median_off_m"] == best_turn and len(summary) > 1 else " "
    cells = [f"{_fmt(s['mean_abs_mph'], ' mph'):>11s}{ms}", f"{_fmt(s['faster_5_pct'], '%'):>9s}",
             f"{_fmt(s['slower_5_pct'], '%'):>9s}", f"{_fmt(s['time_pct'], '%'):>6s}  ", f"{s['taken']:>5d}/{s['turns']:<5d}",
             f"{_fmt(s['median_off_m'], ' m'):>9s}{mt}", f"{_fmt(s['worst_off_m'], ' m'):>9s}"]
    print(f"{label:{w}s}  " + " ".join(cells))
  print("\nper drive (speed |diff| mph; turns taken, median m off the path):")
  print(f"{'route':22s} {'min':>5s} {'op%':>4s}  " + "  ".join(f"{x:>18s}" for x in labels))
  for r in rows:
    if "error" in r:
      print(f"{r['route']:22s} {r['error']}")
      continue
    cells = []
    for label in labels:
      res = r["results"].get(label, {})
      taken = [t["max_lat"] for t in res.get("turns", []) if t["taken"] and t["max_lat"] is not None]
      sp = _fmt(res.get("mean_abs_mph"))
      tu = f"{len(taken)}/{len(res.get('turns', []))} {statistics.median(taken):.1f}" if taken else f"0/{len(res.get('turns', []))}"
      cells.append(f"{sp:>6s} {tu:>11s}")
    print(f"{r['route']:22s} {r['moving_min']:5.1f} {r['assisted_pct']:4d}  " + "  ".join(f"{c:>18s}" for c in cells))
  print("\n* = closest to the recorded driving. time = simulated / recorded drive time. op% = share of moving time openpilot steered.")


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("specs", nargs="+", metavar="SPEC")
  ap.add_argument("--root", default=os.path.expanduser("~/Fisker/routes"))
  ap.add_argument("--routes", nargs="+")
  ap.add_argument("--last", type=int, default=10, help="the newest N drives (default 10) when --routes isn't given")
  ap.add_argument("--unassisted", action="store_true", help="only drives openpilot steered under 20%% of")
  ap.add_argument("--osm-root", default=os.path.expanduser("~/.comma/media/0/osm"))
  ap.add_argument("--jobs", type=int, default=6)
  ap.add_argument("--json", help="write every number to this file")
  ap.add_argument("--pull", action="store_true", help="copy the comma's kept versions first")
  a = ap.parse_args(argv)
  if a.pull:
    pull_versions()
  resolved = [resolve(s) for s in a.specs]
  labels = [r[0] for r in resolved]
  if len(set(labels)) != len(labels):
    raise SystemExit(f"two profiles share a label: {labels}; name them LABEL=SPEC")
  variants = [(r[0], r[1]) for r in resolved]
  descs = {r[0]: r[2] for r in resolved}
  # --routes: exactly those. --last N: newest first, reaching further back until N have driving in them (the comma's
  # parked recordings, newer than the catalog, are found and skipped by run_route).
  pool = pick_routes(a.root, a.routes, 10_000)[::-1] if not a.routes else a.routes
  want = len(pool) if a.routes else a.last
  rows: list[dict] = []
  print(f"{len(variants)} profiles x {want} drives ...", file=sys.stderr)
  with ProcessPoolExecutor(a.jobs) as ex:
    while pool and sum("error" not in r for r in rows) < want:
      n = want - sum("error" not in r for r in rows)
      batch, pool = pool[:n], pool[n:]
      rows += list(ex.map(run_route, batch, [a.root] * len(batch), [variants] * len(batch), [a.osm_root] * len(batch)))
  rows.sort(key=lambda r: r["route"])
  if a.unassisted:
    rows = [r for r in rows if "error" in r or r["assisted_pct"] < 20]
  summary = summarize(rows, labels)
  print_report(rows, summary, descs)
  if a.json:
    with open(a.json, "w") as f:
      json.dump({"profiles": descs, "summary": summary, "drives": rows}, f, indent=1)
    print("wrote", a.json)


if __name__ == "__main__":
  main()
