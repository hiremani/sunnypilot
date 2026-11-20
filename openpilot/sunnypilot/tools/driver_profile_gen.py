#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Measure the driver's own habits from their drives and write the driver profile (selfdrive/controls/lib/
driver_profile.py) the assists and drive_sim read.

  driver_profile_gen.py [ROUTE ...] [--all] [--root DIR] [--name NAME | --out FILE] [--dry-run]

Writes <media>/driver_profiles/NAME.json (NAME defaults to "me"); keep several (e.g. "me", "relaxed") and pick the
active one in the web HUD (or the DriverProfileName param).

Only what the driver chose counts: speed habits only with the car's own ACC off and openpilot not in control of speed,
on open road (no lead within 3 s, not braking for a stop, not in a curve); curve habits only while the driver steered
(not openpilot); stops only those the driver braked for with ACC off. Each value is written clamped to the profile's
bounds; the raw measurement and its sample count go in "meta" next to it. Too few samples: the value keeps its default.

Also measured: the offset for each posted limit with enough open road (limit_offsets_mph), the time gap at which the
driver lifts off the accelerator closing on a slower lead (lead_coast_s), and the speed on narrow streets as the
narrow-street detector sees them (narrow_speed_mph; needs rlogs, which carry the model's road edges).
"""
import argparse
import math
import datetime
import glob
import json
import os
import statistics
import sys
from collections import defaultdict

from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib import driver_profile as dp
from openpilot.sunnypilot.selfdrive.controls.lib import narrow_street as ns
from openpilot.sunnypilot.tools import fisker_drive_style as fds
from openpilot.sunnypilot.tools import turn_habits

MPH = fds.MPH
MIN_SAMPLES = {"speed": 300, "stops": 5, "curve": 100, "turns": 5, "gap": 200, "limit": 1200, "coast": 8,
               "narrow": 200,
               "stop_start": 3}   # 10 Hz samples (narrow: 20 Hz), or events
OPEN_ROAD_GAP_S = 3.0
LAT_CURVE_YAW = 0.05      # rad/s: in a curve
MIN_ROUTE_MOVING_MIN = 3.0


def routes_in(root: str) -> list[str]:
  return sorted({os.path.basename(p).rsplit("--", 1)[0] for p in glob.glob(os.path.join(root, "*--*--*"))})


def speed_offsets(S: list, fast_zone_mph: float) -> dict[int, list[float]]:
  """(v - limit) in mph per posted limit (5 mph bins), on open road with the driver choosing the speed."""
  by: dict[int, list[float]] = defaultdict(list)
  for s in S:
    if s.limit <= 0 or s.v < 4.5 or s.cruise or s.brake or abs(s.a) > 0.6:
      continue
    if 0 < s.lead_d < OPEN_ROAD_GAP_S * s.v or abs(s.yaw) > LAT_CURVE_YAW:
      continue
    by[round(s.limit * MPH / 5) * 5].append((s.v - s.limit) * MPH)
  return by


LEAD_COAST_CLOSING = 0.5   # m/s slower than us
LEAD_COAST_MAX_GAP_S = 6.0  # releases with the lead further than this are about something else


def lead_coast_gaps(S: list) -> list[float]:
  """Time gaps (s) to a slower lead at which the driver, ACC off, took the foot off the accelerator."""
  out = []
  for p, s in zip(S, S[1:], strict=False):
    if not (p.gas and not s.gas) or s.cruise or s.v < 3.0:
      continue
    if 0 < s.lead_d < 120 and s.lead_v < s.v - LEAD_COAST_CLOSING and s.lead_d / s.v < LEAD_COAST_MAX_GAP_S:
      out.append(s.lead_d / s.v)
  return out


def narrow_speeds(root: str, route: str) -> list[float]:
  """Speeds (m/s, 20 Hz) while the narrow-street detector flags the street and the driver chose the speed (ACC off).
  Needs rlogs (modelV2); routes without them give nothing."""
  from openpilot.tools.lib.logreader import LogReader
  out: list[float] = []
  segs = sorted(glob.glob(os.path.join(root, f"{route}--*")), key=lambda p: int(p.rsplit("--", 1)[1]))
  det = ns.NarrowStreetDetector(DT_MDL)
  v, cruise = 0.0, True
  for seg in segs:
    f = os.path.join(seg, "rlog.zst")
    if not os.path.exists(f):
      continue
    try:
      for m in LogReader(f):
        w = m.which()
        if w == "carState":
          v, cruise = m.carState.vEgo, m.carState.cruiseState.enabled
        elif w == "modelV2":
          if det.update(m.modelV2, v) and not cruise and v > ns.NARROW_V_MIN:
            out.append(v)
    except Exception:
      continue
  return out


def coast_times(S: list) -> list[float]:
  """Per full stop with ACC off: seconds from releasing the accelerator to the brake / real deceleration."""
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
      out.append(S[brk].t - S[rel].t)
  return out


def measure(routes: list[str], root: str, narrow: bool = True, turns_from_stop: bool = True) -> tuple[dp.DriverProfile, dict]:
  base = dp.DriverProfile()
  offsets: dict[int, list[float]] = defaultdict(list)
  stops, coasts, lat_acc, turns, gaps, lead_coasts, narrow_v = [], [], [], [], [], [], []
  unwind_turns: list[dict] = []
  stop_starts: dict[str, list[float]] = {"left": [], "right": []}
  used, miles = [], 0.0
  for r in routes:
    S = fds.load(root, r)
    if len(S) < 100:
      continue
    ov = fds.overview(S)
    if ov["moving_min"] < MIN_ROUTE_MOVING_MIN:
      continue
    used.append(r)
    miles += ov["miles"]
    for lim, d in speed_offsets(S, base.fast_zone_mph).items():
      offsets[lim] += d
    acc_on = [s.t for s in S if s.cruise]
    for st in fds.stops(S):
      if not any(st["t"] - 20 <= t <= st["t"] for t in acc_on[::10]):
        stops.append(st)
    coasts += coast_times(S)
    lat_acc += [abs(s.v * s.yaw) for s in S if not s.lat and s.v > 9 and abs(s.yaw) > LAT_CURVE_YAW]
    junctions = [t for t in fds.turns(S) if t["kind"] == "junction"]
    turns += [t for t in junctions if t["driver_steered_pct"] > 50]
    # the wheel's return: every junction turn the driver steered, also with openpilot's steering off and a light grip
    unwind_turns += [t for t in junctions if t["driver_steered_pct"] > 50 or t["lat_engaged_pct"] < 50]
    gaps += [s.lead_d / s.v for s in S if s.v > 8 and 0 < s.lead_d < 100 and not s.cruise and abs(s.lead_v - s.v) < 3]
    lead_coasts += lead_coast_gaps(S)
    if narrow:
      narrow_v += narrow_speeds(root, r)
    if turns_from_stop:
      try:
        for o in turn_habits.turn_onsets(root, r):
          if o["from_stop"] and o["dist_m"] > 0.0:
            stop_starts[o["side"]].append(o["dist_m"])
      except Exception as e:   # no road tiles here, or a bad log: the value keeps its default
        print(f"turn starts skipped for {r}: {e}", file=sys.stderr)

  values, meta = {}, {}

  def put(name, raw, n, need, note=""):
    meta[name] = {"measured": None if raw is None else round(raw, 3), "samples": n, "note": note}
    if raw is not None and n >= need:
      values[name] = dp.clamp(name, raw)
      meta[name]["used"] = values[name]
    else:
      meta[name]["used"] = getattr(base, name)
      meta[name]["note"] = (note + "; " if note else "") + f"too few samples (need {need}): default"

  def weighted_median(zones):
    pts = [x for lim in zones for x in offsets[lim]]
    return (statistics.median(pts), len(pts)) if pts else (None, 0)

  town = [lim for lim in offsets if lim < base.fast_zone_mph - 2]
  fast = [lim for lim in offsets if lim >= base.fast_zone_mph - 2]
  t_med, t_n = weighted_median(town)
  f_med, f_n = weighted_median(fast)
  put("town_offset_mph", t_med, t_n, MIN_SAMPLES["speed"], f"median (speed - limit) on open road below {base.fast_zone_mph:.0f} mph")
  put("fast_offset_mph", f_med, f_n, MIN_SAMPLES["speed"], f"median (speed - limit) on open road {base.fast_zone_mph:.0f} mph and up")
  meta["speed_by_limit"] = {str(lim): {"median_mph": round(statistics.median(d), 1), "samples": len(d)}
                            for lim, d in sorted(offsets.items()) if len(d) >= 50}

  decel = [s["avg_decel"] for s in stops]
  put("stop_decel", statistics.median(decel) if decel else None, len(decel), MIN_SAMPLES["stops"], "median braking to a full stop")
  put("coast_s", statistics.median(coasts) if coasts else None, len(coasts), MIN_SAMPLES["stops"],
      "median accelerator release to braking, before a full stop")
  launch = [s["launch_accel"] for s in stops if s["launch_accel"] is not None]
  put("launch_accel", statistics.median(launch) if launch else None, len(launch), MIN_SAMPLES["stops"],
      "median pulling away below 15 mph (the car caps openpilot at 1.0)")
  p75 = sorted(lat_acc)[int(len(lat_acc) * 0.75)] if lat_acc else None
  put("curve_lat_accel", p75, len(lat_acc), MIN_SAMPLES["curve"], "p75 lateral accel in curves above 20 mph, driver steering")
  tmin = [t["min_mph"] for t in turns]
  put("junction_turn_mph", statistics.median(tmin) if tmin else None, len(tmin), MIN_SAMPLES["turns"],
      "median slowest speed through junction turns the driver steered")
  exits = [t["exit_accel"] for t in turns if t.get("exit_accel") is not None and t["exit_accel"] > 0.0]
  put("turn_exit_accel", statistics.median(exits) if exits else None, len(exits), MIN_SAMPLES["turns"],
      "median pull-away over the 3 s after a junction turn the driver steered, on the accelerator, ACC off, no lead")
  for side in ("left", "right"):
    ds = stop_starts[side]
    put(f"turn_stop_start_{side}_m", statistics.median(ds) if ds else None, len(ds), MIN_SAMPLES["stop_start"],
        f"median distance before the junction node where the driver's wheel passed 30 deg, signaled {side} turns"
        + " moving off from a stop" + ("" if turns_from_stop else " (skipped)"))
  # The wheel's return at the end of a turn, from the turns the driver made without the blinker (the turn assist never
  # had the wheel there); with too few of those, from every junction turn the driver steered.
  unwinds = [t["unwind_deg_s"] for t in unwind_turns if t.get("unwind_deg_s") and t["blinker_lead_s"] is None]
  which = "without the blinker"
  if len(unwinds) < MIN_SAMPLES["turns"]:
    unwinds, which = [t["unwind_deg_s"] for t in unwind_turns if t.get("unwind_deg_s")], "driver steered"
  put("turn_unwind_deg_s", statistics.median(unwinds) if unwinds else None, len(unwinds), MIN_SAMPLES["turns"],
      f"median speed the wheel came back at the end of junction turns ({which}), 90% to 10% of its peak")
  put("follow_gap_s", statistics.median(gaps) if gaps else None, len(gaps), MIN_SAMPLES["gap"],
      "median time gap behind a steady lead, ACC off")
  put("lead_coast_s", statistics.median(lead_coasts) if lead_coasts else None, len(lead_coasts), MIN_SAMPLES["coast"],
      "median time gap at which the driver lifted off closing on a slower lead, ACC off")
  put("narrow_speed_mph", statistics.median(narrow_v) * MPH if narrow_v else None, len(narrow_v), MIN_SAMPLES["narrow"],
      "median speed on streets the narrow-street detector flagged, ACC off" + ("" if narrow else " (skipped)"))
  limit_offsets = {str(lim): round(statistics.median(d), 1) for lim, d in sorted(offsets.items())
                   if len(d) >= MIN_SAMPLES["limit"] and math.isfinite(statistics.median(d))}
  values["limit_offsets_mph"] = dp.clamp_limit_offsets(limit_offsets, base.fast_zone_mph)
  meta["limit_offsets_mph"] = {"measured": limit_offsets, "used": values["limit_offsets_mph"],
                               "note": f"median (speed - limit) per posted limit with >= {MIN_SAMPLES['limit'] / 10:.0f} s of open road"}

  profile = dp.DriverProfile(**({f: getattr(base, f) for f in dp.BOUNDS} | values))
  profile.limit_offsets_mph = values["limit_offsets_mph"]
  info = {"generated": datetime.datetime.now().isoformat(timespec="seconds"), "routes": used, "miles": round(miles, 1),
          "values": meta, "stops": len(stops), "turns": len(turns)}
  return profile, info


def report(profile: dp.DriverProfile, info: dict) -> None:
  base = dp.DriverProfile()
  print(f"{len(info['routes'])} routes, {info['miles']} mi, {info['stops']} stops, {info['turns']} junction turns")
  print(f"{'value':20s} {'measured':>9s} {'used':>7s} {'default':>8s} {'samples':>8s}  note")
  for name in dp.BOUNDS:
    m = info["values"].get(name, {})
    meas = "-" if m.get("measured") is None else f"{m['measured']:.2f}"
    print(f"{name:20s} {meas:>9s} {getattr(profile, name):7.2f} {getattr(base, name):8.2f} {m.get('samples', 0):8d}  {m.get('note', '')}")
  if profile.limit_offsets_mph:
    print("per-limit offsets used:", ", ".join(f"{k} mph {v:+.1f}" for k, v in profile.limit_offsets_mph.items()))
  if info["values"].get("speed_by_limit"):
    print("speed vs limit on open road:", ", ".join(f"{k} mph zone {v['median_mph']:+.1f} ({v['samples'] / 10:.0f} s)"
                                                    for k, v in info["values"]["speed_by_limit"].items()))


def main(argv: list[str] | None = None) -> int:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("routes", nargs="*")
  ap.add_argument("--all", action="store_true", help="every route under --root")
  ap.add_argument("--root", default=fds.DEFAULT_ROOT)
  ap.add_argument("--name", default=dp.DEFAULT_NAME, help="profile name (letters, digits, space, - _)")
  ap.add_argument("--out", default=None, help="write here instead of the named profile's file")
  ap.add_argument("--dry-run", action="store_true", help="measure and print, write nothing")
  ap.add_argument("--no-narrow", action="store_true", help="skip the narrow-street speed (reads every rlog)")
  ap.add_argument("--no-turns", action="store_true", help="skip the turn starts from a stop (reads every rlog, needs road tiles)")
  args = ap.parse_args(argv)
  routes = routes_in(args.root) if args.all else args.routes
  if not routes:
    print("no routes (name them, or --all)", file=sys.stderr)
    return 1
  profile, info = measure(routes, args.root, narrow=not args.no_narrow, turns_from_stop=not args.no_turns)
  report(profile, info)
  if not args.dry_run:
    if not args.out and not dp.valid_name(args.name):
      print(f"bad profile name {args.name!r}", file=sys.stderr)
      return 1
    out = args.out or dp.profile_path(args.name)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w") as f:
      json.dump(dp.to_dict(profile, info), f, indent=1)
    print("wrote", out)
  return 0


if __name__ == "__main__":
  sys.exit(main())
