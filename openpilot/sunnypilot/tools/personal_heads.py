#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Build the personal model's habit heads (selfdrive/controls/lib/personal/profile.py) from the driver's drives, on the
Mac. The events are the ones driver_profile_gen.py measures (same filters: only what the driver chose), each kept with
the situation it happened in: the speed, the closing speed to the lead, the junction turn's angle, the posted limit.

Per head: the newest TEST_FRAC of drives are held out; a curve over the context and a single number (what the JSON
profile stores) are fitted on the rest and compared on them. The curve is kept only when it beats the single number by
CURVE_GAIN; the kept form is then refitted on every drive. Heads go into <driver_profiles>/personal/<name>.json next to
the speed residual (personal_train.py), whose part of the file is kept.

Each drive's events are cached in <root>/_personal_cache/<route>.heads.json.

  personal_heads.py [--root ~/Fisker/routes] [--no-narrow] [--no-turn-starts] [--out FILE] [--dry-run]
"""
import argparse
import json
import os
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib import driver_profile as dp
from openpilot.sunnypilot.selfdrive.controls.lib import profile_long
from openpilot.sunnypilot.selfdrive.controls.lib.personal import heads as hd
from openpilot.sunnypilot.selfdrive.controls.lib.personal import profile as pprof
from openpilot.sunnypilot.tools import driver_profile_gen as gen
from openpilot.sunnypilot.tools import fisker_drive_style as fds
from openpilot.sunnypilot.tools import habits_extra as hx
from openpilot.sunnypilot.tools import personal_dataset as pdset

CACHE_VERSION = 2
TEST_FRAC = 0.2
CURVE_GAIN = 0.03         # a curve must beat the single number by 3% on the held-out drives
MPH = fds.MPH


def coast_events(S: list) -> list[tuple[float, float]]:
  """gen.coast_times with the speed at the release: (v, seconds from releasing the accelerator to braking)."""
  out = []
  stopped = [i for i in range(1, len(S)) if S[i].v < fds.V_STOP <= S[i - 1].v]
  for i in stopped:
    if any(S[k].cruise for k in range(max(0, i - 300), i)):
      continue
    rel = None
    for k in range(i - 1, max(0, i - 300), -1):
      if S[k].gas:
        rel = k + 1
        break
    if rel is None:
      continue
    brk = next((k for k in range(rel, i) if S[k].brake or S[k].a < -0.8), None)
    if brk is not None and S[rel].v > 4.0:
      out.append((S[rel].v, S[brk].t - S[rel].t))
  return out


def lead_coast_events(S: list) -> list[tuple[float, float]]:
  """gen.lead_coast_gaps with the closing speed: (closing m/s, time gap s) where the driver lifted off."""
  out = []
  for p, s in zip(S, S[1:], strict=False):
    if not (p.gas and not s.gas) or s.cruise or s.v < 3.0:
      continue
    if 0 < s.lead_d < 120 and s.lead_v < s.v - gen.LEAD_COAST_CLOSING and s.lead_d / s.v < gen.LEAD_COAST_MAX_GAP_S:
      out.append((s.v - s.lead_v, s.lead_d / s.v))
  return out


def route_events(args) -> tuple[str, dict]:
  root, route, segs, narrow, turn_starts = args
  cache = os.path.join(root, "_personal_cache", route + ".heads.json")
  key = {"v": CACHE_VERSION, "segs": segs, "narrow": narrow, "turn_starts": turn_starts}
  try:
    with open(cache) as f:
      c = json.load(f)
    if c.get("key") == key:
      return route, c["events"]
  except (OSError, ValueError):
    pass
  ev: dict[str, list] = {k: [] for k in ("offset", "follow", "lead_coast", "lead_brake", "stop", "coast", "launch",
                                         "curve", "junction", "exit", "unwind", "start_left", "start_right", "narrow",
                                         "stop_gap", "cruise_accel", "lane_change", "bump", "turn_in")}
  S = fds.load(root, route)
  if len(S) >= 100 and fds.overview(S)["moving_min"] >= gen.MIN_ROUTE_MOVING_MIN:
    for s in S:
      if s.limit <= 0 or s.v < 4.5 or s.cruise or s.brake or abs(s.a) > 0.6:
        continue
      if 0 < s.lead_d < gen.OPEN_ROAD_GAP_S * s.v or abs(s.yaw) > gen.LAT_CURVE_YAW:
        continue
      ev["offset"].append((round(s.limit * MPH / 5) * 5, (s.v - s.limit) * MPH))
    ev["follow"] = [(s.v, s.lead_d / s.v) for s in S
                    if s.v > 8 and 0 < s.lead_d < 100 and not s.cruise and abs(s.lead_v - s.v) < 3]
    ev["lead_coast"] = lead_coast_events(S)
    ev["lead_brake"] = [list(e) for e in gen.lead_brake_events(S)]
    acc_on = [s.t for s in S if s.cruise]
    for st in fds.stops(S):
      if any(st["t"] - 20 <= t <= st["t"] for t in acc_on[::10]):
        continue
      ev["stop"].append((st["from_mph"] / MPH, st["avg_decel"]))
      if st["launch_accel"] is not None:
        ev["launch"].append((0.0, st["launch_accel"]))
    ev["coast"] = coast_events(S)
    ev["curve"] = [(s.v, abs(s.v * s.yaw)) for s in S if not s.lat and s.v > 9 and abs(s.yaw) > gen.LAT_CURVE_YAW]
    for t in fds.turns(S):
      if t["kind"] != "junction":
        continue
      if t["driver_steered_pct"] > 50:
        ev["junction"].append((t["deg"], t["min_mph"]))
        if t.get("exit_accel") is not None and t["exit_accel"] > 0.0:
          ev["exit"].append((t["deg"], t["exit_accel"]))
      if (t["driver_steered_pct"] > 50 or t["lat_engaged_pct"] < 50) and t.get("unwind_deg_s"):
        ev["unwind"].append((t["deg"], t["unwind_deg_s"], t["blinker_lead_s"] is None))
    ev["stop_gap"] = [(0.0, g) for g in hx.stop_gaps(S)]
    ev["cruise_accel"] = hx.cruise_accels(S)[::2]
    ev["lane_change"] = [(0.0, d) for d in hx.lane_change_delays(S)]
    ev["turn_in"] = hx.turn_in_rates(S)
    if turn_starts:
      try:
        for o in gen.turn_habits.turn_onsets(root, route):
          if o["from_stop"] and o["dist_m"] > 0.0:
            ev["start_" + o["side"]].append((0.0, o["dist_m"]))
      except Exception:
        pass
    if narrow:
      ev["narrow"] = [(0.0, v * MPH) for v in gen.narrow_speeds(root, route)]
      ev["bump"] = [(0.0, v) for v in hx.bump_crossings(root, route)]
  ev = {k: [[float(x) for x in e] for e in v] for k, v in ev.items()}
  os.makedirs(os.path.dirname(cache), exist_ok=True)
  with open(cache + ".tmp", "w") as f:
    json.dump({"key": key, "events": ev}, f)
  os.replace(cache + ".tmp", cache)
  return route, ev


def collect(root: str, narrow: bool, turn_starts: bool, workers: int = 8) -> dict[str, dict]:
  ds = pdset.drives(root)
  with ProcessPoolExecutor(max_workers=workers) as ex:
    return dict(ex.map(route_events, [(root, r, s, narrow, turn_starts) for r, s in sorted(ds.items())]))


def stack(evs: dict[str, dict], routes: list[str], kind: str, cols: int = 2) -> np.ndarray:
  rows = [e for r in routes for e in evs[r].get(kind, [])]
  return np.array(rows, dtype=float).reshape(-1, cols) if rows else np.zeros((0, cols))


# head name -> the event kind its (context, value) pairs come from
SOURCES = {
  "follow_gap_s": "follow", "lead_coast_s": "lead_coast", "stop_decel": "stop", "coast_s": "coast", "launch_accel": "launch",
  "curve_lat_accel": "curve", "junction_turn_mph": "junction", "turn_exit_accel": "exit",
  "turn_stop_start_left_m": "start_left", "turn_stop_start_right_m": "start_right", "narrow_speed_mph": "narrow",
  "stop_gap_m": "stop_gap", "cruise_accel": "cruise_accel", "lane_change_delay_s": "lane_change", "bump_cross_mph": "bump",
  "turn_in_deg_s": "turn_in",
}


def xy_for(name: str, evs: dict, routes: list[str], follow_gap: float) -> tuple[np.ndarray, np.ndarray]:
  if name in ("town_offset_mph", "fast_offset_mph", "limit_offsets_mph"):
    a = stack(evs, routes, "offset")
    fast = a[:, 0] >= dp.DriverProfile().fast_zone_mph - 2 if len(a) else np.zeros(0, dtype=bool)
    if name == "town_offset_mph":
      a = a[~fast]
    elif name == "fast_offset_mph":
      a = a[fast]
    return a[:, 0], a[:, 1]
  if name == "lead_brake_a":
    a = stack(evs, routes, "lead_brake", 3)
    need = np.array([profile_long.lead_brake_need(follow_gap, v, d, vr) for v, d, vr in a]) if len(a) else np.zeros(0)
    ok = np.isfinite(need)
    return -a[ok, 2], need[ok]
  if name == "turn_unwind_deg_s":
    a = stack(evs, routes, "unwind", 3)
    nb = a[a[:, 2] > 0.5] if len(a) else a
    a = nb if len(nb) >= pprof.HEADS[name].min_n else a     # turns without the blinker are the reference
    return a[:, 0], a[:, 1]
  a = stack(evs, routes, SOURCES[name])
  return a[:, 0], a[:, 1]


def fit_head(name: str, x, y) -> hd.CurveHead:
  spec = pprof.HEADS[name]
  if spec.x is None:
    return hd.fit(np.zeros(len(y)), y, spec.stat, min_bin=10 ** 9, x_name="", y_name=spec.field)
  if spec.discrete:
    return hd.fit_discrete(x, y, spec.stat, min_per_key=spec.min_bin, x_name=spec.x, y_name=spec.field)
  return hd.fit(x, y, spec.stat, min_bin=spec.min_bin, x_name=spec.x, y_name=spec.field)


def build(evs: dict[str, dict]) -> tuple[dict, dict, dict]:
  routes = sorted(evs, key=lambda r: int(r[:8], 16))
  n_test = max(1, round(len(routes) * TEST_FRAC))
  train, test = routes[:-n_test], routes[-n_test:]
  # the following gap first: lead_brake_a is measured against it
  fx, fy = xy_for("follow_gap_s", evs, routes, 0.0)
  follow_gap = dp.clamp("follow_gap_s", float(np.median(fy))) if len(fy) else dp.DriverProfile().follow_gap_s
  heads, use, report = {}, {}, {}
  for name, spec in pprof.HEADS.items():
    xa, ya = xy_for(name, evs, routes, follow_gap)
    xtr, ytr = xy_for(name, evs, train, follow_gap)
    xte, yte = xy_for(name, evs, test, follow_gap)
    form = "const"
    ev = {"n": int(len(ya)), "n_test": int(len(yte))}
    if spec.x is not None and len(ytr) and len(yte) >= 10:
      curve = fit_head(name, xtr, ytr)
      const = float(np.median(ytr)) if spec.stat == "median" else hd.statistic(ytr, spec.stat)
      if curve.is_curve and not spec.discrete:
        l_const = hd.loss(np.full(len(yte), const), yte, spec.stat)
        l_curve = hd.loss(np.array([curve.predict(x) for x in xte]), yte, spec.stat)
        ev.update({"test_loss_const": round(l_const, 4), "test_loss_curve": round(l_curve, 4),
                   "gain": round(1.0 - l_curve / l_const, 3) if l_const > 0 else 0.0})
        if l_curve < (1.0 - CURVE_GAIN) * l_const:
          form = "curve"
    if spec.discrete:
      form = "curve"      # one value per posted limit, as the JSON's per-limit offsets
    h = fit_head(name, xa, ya) if form == "curve" else hd.fit(np.zeros(len(ya)), ya, spec.stat, min_bin=10 ** 9,
                                                               x_name=spec.x or "", y_name=spec.field)
    heads[name] = h
    use[name] = len(ya) >= spec.min_n and np.isfinite(h.const)
    ev["form"] = "curve" if h.is_curve else "const"
    report[name] = ev
  return heads, use, {"report": report, "follow_gap_for_lead_brake": round(follow_gap, 3), "train": train, "test": test}


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--root", default=os.path.expanduser("~/Fisker/routes"))
  ap.add_argument("--name", default=dp.DEFAULT_NAME)
  ap.add_argument("--out", default=None)
  ap.add_argument("--no-narrow", action="store_true")
  ap.add_argument("--no-turn-starts", action="store_true")
  ap.add_argument("--dry-run", action="store_true")
  ap.add_argument("--workers", type=int, default=8)
  a = ap.parse_args()

  evs = collect(a.root, not a.no_narrow, not a.no_turn_starts, a.workers)
  evs = {r: e for r, e in evs.items() if any(len(v) for v in e.values())}
  heads, use, info = build(evs)
  pp = pprof.PersonalProfile(heads, use)
  ref = pp.at()
  print(f"{len(evs)} drives with events; test drives: {len(info['test'])}")
  print(f"{'habit':24s} {'events':>7s} {'form':6s} {'value (all)':>11s}  {'held-out: const -> curve (gain)':32s} points")
  for name, spec in pprof.HEADS.items():
    h, r = heads[name], info["report"][name]
    val = getattr(ref, spec.field) if name != "limit_offsets_mph" else ref.limit_offsets_mph
    cmp = (f"{r['test_loss_const']:.3f} -> {r['test_loss_curve']:.3f} ({100 * r['gain']:+.0f}%)"
           if "gain" in r else "")
    pts = ", ".join(f"{x:.0f}:{y:.2f}" for x, y in zip(h.xs, h.ys, strict=True)) if h.is_curve else ""
    vs = f"{val:.2f}" if isinstance(val, float) else str(val)
    print(f"{name:24s} {r['n']:7d} {r['form'] if use[name] else 'off':6s} {vs:>11s}  {cmp:32s} {pts}")
  if a.dry_run:
    return
  raw = pprof.read_raw(a.name) if a.out is None else None
  sr = None
  if isinstance(raw, dict):
    sr = raw.get("speed_residual") if raw.get("version") == 2 else (raw if raw.get("version") == 1 else None)
  d = pp.to_dict()
  d["speed_residual"] = sr
  for name, r in info["report"].items():
    d["heads"][name]["eval"] = r
  d["meta"] = {"drives": len(evs), "test_drives": info["test"], "follow_gap_for_lead_brake": info["follow_gap_for_lead_brake"]}
  print("saved", pprof.write_raw(d, a.name, a.out))


if __name__ == "__main__":
  main()
