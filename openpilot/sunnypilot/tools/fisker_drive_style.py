#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

How the driver actually drives, from one or more routes' qlogs, as numbers the assists can be tuned to:
  speed against the posted limit, how and where they brake for stops, launches, turns (speed, blinker lead,
  who steered), curves (lateral accel), following gap, and when and why they take over from steering.

  fisker_drive_style.py ROUTE [ROUTE ...] [--root DIR] [--json OUT]

qlogs are enough (carState at 10 Hz); rlogs are used when a segment has no qlog.
"""
import argparse
import glob
import json
import math
import os
import sys
from collections import Counter
from dataclasses import asdict, dataclass

from openpilot.tools.lib.logreader import LogReader

DEFAULT_ROOT = "/data/media/0/realdata"
MPH = 2.23694
V_STOP = 0.3            # m/s: stopped
TURN_YAW = math.radians(6.0)    # rad/s: a turn stretch starts above this
TURN_MIN_DEG = 60.0
TURN_MAX_V = 13.0       # m/s: faster heading changes are curves / ramps, not junction turns
BLINKER_LOOKBACK_S = 15.0


@dataclass
class Sample:
  t: float
  v: float
  a: float
  yaw: float
  lb: bool
  rb: bool
  gas: bool
  brake: bool
  steer_pressed: bool
  cruise: bool
  lat: bool
  lead_d: float      # m, 0 = no lead
  lead_v: float
  limit: float       # m/s posted limit, 0 = none
  lat_deg: float
  lon_deg: float
  route: str
  steer_deg: float = 0.0   # steering wheel angle, + = left


def seg_logs(root: str, route: str) -> list[str]:
  segs = sorted(glob.glob(os.path.join(root, f"{route}--*")), key=lambda p: int(p.rsplit("--", 1)[1]))
  out = []
  for s in segs:
    for name in ("qlog.zst", "qlog", "rlog.zst", "rlog"):
      if os.path.exists(os.path.join(s, name)):
        out.append(os.path.join(s, name))
        break
  return out


def load(root: str, route: str) -> list[Sample]:
  out: list[Sample] = []
  t0 = None
  lat = False
  lead = (0.0, 0.0)
  limit = 0.0
  gps = (0.0, 0.0)
  for f in seg_logs(root, route):
    rlog = "rlog" in os.path.basename(f)
    n = 0
    for m in LogReader(f):
      w = m.which()
      if w == "carControl":
        lat = m.carControl.latActive
      elif w == "radarState":
        l1 = m.radarState.leadOne
        lead = (l1.dRel, l1.vLead) if l1.present else (0.0, 0.0)
      elif w == "longitudinalPlanSP":
        r = m.longitudinalPlanSP.speedLimit.resolver
        limit = r.speedLimit if r.speedLimitValid else 0.0
      elif w == "gpsLocationExternal" and m.gpsLocationExternal.hasFix:
        gps = (m.gpsLocationExternal.latitude, m.gpsLocationExternal.longitude)
      elif w == "carState":
        n += 1
        if rlog and n % 10:
          continue   # rlog carState is 100 Hz; keep 10 Hz like the qlog
        if t0 is None:
          t0 = m.logMonoTime    # the first message of a route can carry an old time; carState can't
        cs = m.carState
        out.append(Sample((m.logMonoTime - t0) / 1e9, cs.vEgo, cs.aEgo, cs.yawRate, cs.leftBlinker, cs.rightBlinker,
                          cs.gasPressed, cs.brakePressed, cs.steeringPressed, cs.cruiseState.enabled, lat,
                          lead[0], lead[1], limit, gps[0], gps[1], route, cs.steeringAngleDeg))
  return out


def pct(vals: list[float], q: float) -> float:
  if not vals:
    return float("nan")
  s = sorted(vals)
  return s[min(int(len(s) * q), len(s) - 1)]


def where(s: Sample) -> str:
  return f"{s.lat_deg:.5f},{s.lon_deg:.5f}" if s.lat_deg else "no gps"


# ---- analyses: each takes the samples of one route, returns plain dicts ----

def overview(S: list[Sample]) -> dict:
  dist = moving = lat_moving = cruise = 0.0
  for p, s in zip(S, S[1:], strict=False):
    dt = s.t - p.t
    if not 0 < dt < 1.0:
      continue
    dist += s.v * dt
    if s.v > 1.0:
      moving += dt
      lat_moving += dt * s.lat
      cruise += dt * s.cruise
  return {"miles": dist / 1609.34, "moving_min": moving / 60, "steering_engaged_pct": 100 * lat_moving / max(moving, 1),
          "car_acc_pct": 100 * cruise / max(moving, 1)}


def speed_vs_limit(S: list[Sample]) -> dict:
  by: dict[int, list[float]] = {}
  for s in S:
    if s.limit > 0 and s.v > 4.5 and not s.cruise:
      by.setdefault(round(s.limit * MPH / 5) * 5, []).append((s.v - s.limit) * MPH)
  return {lim: {"n_s": len(d) / 10, "median_over_mph": pct(d, .5), "p85_over_mph": pct(d, .85),
                "pct_over_5": 100 * sum(x > 5 for x in d) / len(d)} for lim, d in sorted(by.items()) if len(d) > 100}


def stops(S: list[Sample]) -> list[dict]:
  """Every full stop: where braking started, at what speed, over what distance; how long it stopped; the launch."""
  out = []
  i = 0
  while i < len(S):
    if S[i].v >= V_STOP:
      i += 1
      continue
    j = i
    while j < len(S) and S[j].v < V_STOP:
      j += 1
    if j >= len(S) or S[j - 1].t - S[i].t < 0.8 or i == 0:
      i = j
      continue
    # braking onset: walk back while the car was slowing (a < -0.3), allowing short gaps
    k, quiet = i, 0
    while k > 0 and S[i].t - S[k].t < 40:
      quiet = quiet + 1 if S[k].a > -0.3 else 0
      if quiet >= 10:
        k += 10
        break
      k -= 1
    v0 = S[k].v
    d = sum(s.v * (s.t - p.t) for p, s in zip(S[k:i], S[k + 1:i + 1], strict=False))
    approach = S[k:i + 1]
    brake_frac = sum(s.brake for s in approach) / max(len(approach), 1)
    # launch: time from moving off to 20 mph, if reached within 15 s without stopping again
    t20 = None
    for s in S[j:]:
      if s.t - S[j].t > 15 or s.v < V_STOP:
        break
      if s.v >= 20 / MPH:
        t20 = s.t - S[j].t
        break
    launch_a = [s.a for s in S[j:j + 60] if s.v < 15 / MPH]
    if v0 > 3.0 and d > 3.0:
      out.append({"t": round(S[i].t), "where": where(S[i]), "from_mph": v0 * MPH, "brake_dist_m": d,
                  "avg_decel": v0 ** 2 / (2 * d), "peak_decel": -min(s.a for s in approach), "friction_brake_pct": 100 * brake_frac,
                  "stopped_s": S[j - 1].t - S[i].t, "to_20mph_s": t20, "launch_accel": pct(launch_a, .5) if launch_a else None})
    i = j
  return out


def rolling_slowdowns(S: list[Sample]) -> list[dict]:
  """Slowed to a crawl (< 5 mph) without stopping, from and back to > 12 mph within 20 s: yields, rolling stops."""
  out = []
  for i in range(1, len(S) - 1):
    s = S[i]
    if not (V_STOP <= s.v < 5 / MPH and s.v <= S[i - 1].v and s.v <= S[i + 1].v):
      continue
    win = [x for x in S if abs(x.t - s.t) <= 20]
    if min(x.v for x in win) < V_STOP:
      continue
    before = [x.v for x in win if x.t < s.t]
    after = [x.v for x in win if x.t > s.t]
    if before and after and max(before) > 12 / MPH and max(after) > 12 / MPH:
      if not out or s.t - out[-1]["t"] > 20:
        out.append({"t": round(s.t), "where": where(s), "min_mph": s.v * MPH})
  return out


TURN_EXIT_S = 3.0       # s after a turn over which the pull-away out of it is measured


def exit_accel(S: list[Sample], j: int) -> float | None:
  """m/s^2 the driver pulled away with over TURN_EXIT_S after the turn ending at S[j]: only when they were on the
  accelerator, never braked, had no lead within 30 m, and the car's ACC was off (else it is the car's choice)."""
  end = S[j].t + TURN_EXIT_S
  win = [s for s in S[j:j + 400] if s.t <= end]
  if not win or win[-1].t < end - 0.3:
    return None
  if any(s.brake or s.cruise or 0 < s.lead_d < 30 for s in win) or not any(s.gas for s in win):
    return None
  return (win[-1].v - win[0].v) / (win[-1].t - win[0].t)


UNWIND_FROM, UNWIND_TO = 0.9, 0.1   # the wheel's return: from 90% of its peak toward the turn to 10%
UNWIND_MIN_PEAK_DEG = 90.0          # a junction turn the wheel really wound up for
UNWIND_MAX_S = 10.0


def unwind_rate(S: list[Sample], i: int, j: int, sign: int) -> float | None:
  """deg/s: the wheel's average speed coming back at the end of the turn S[i..j] (sign + = left): 80% of its peak
  over the time from UNWIND_FROM to UNWIND_TO of the peak. None when the wheel never wound up or never came back."""
  lo = max(0, i - 30)
  k = max(range(lo, j + 1), key=lambda n: S[n].steer_deg * sign)
  peak = S[k].steer_deg * sign
  if peak < UNWIND_MIN_PEAK_DEG:
    return None
  a = k
  while a < len(S) - 1 and S[a + 1].steer_deg * sign >= UNWIND_FROM * peak and S[a].t - S[k].t < UNWIND_MAX_S:
    a += 1
  b = a
  while b < len(S) - 1 and S[b].steer_deg * sign > UNWIND_TO * peak and S[b].t - S[a].t < UNWIND_MAX_S:
    b += 1
  if S[b].steer_deg * sign > UNWIND_TO * peak or S[b].t <= S[a].t:
    return None
  return (UNWIND_FROM - UNWIND_TO) * peak / (S[b].t - S[a].t)


def turns(S: list[Sample]) -> list[dict]:
  out = []
  i = 0
  while i < len(S):
    if abs(S[i].yaw) < TURN_YAW:
      i += 1
      continue
    sign = 1 if S[i].yaw > 0 else -1
    j, heading = i, 0.0
    while j + 1 < len(S) and S[j].yaw * sign > TURN_YAW * 0.5:
      heading += S[j].yaw * (S[j + 1].t - S[j].t)
      j += 1
    seg = S[i:j + 1]
    deg = abs(math.degrees(heading))
    if deg >= TURN_MIN_DEG and min(s.v for s in seg) > 0.5:
      side = "left" if sign > 0 else "right"
      onset = None
      for s in S[max(0, i - 200):i + 1]:
        if i and S[i].t - s.t <= BLINKER_LOOKBACK_S and (s.lb if side == "left" else s.rb):
          onset = s
          break
      lead_s = (S[i].t - onset.t) if onset else None
      lead_m = sum(x.v * (y.t - x.t) for x, y in zip(S, S[1:], strict=False) if onset and onset.t <= x.t < S[i].t) if onset else None
      kind = "junction" if max(s.v for s in seg) < TURN_MAX_V and deg <= 135 else ("u/loop" if deg > 135 else "curve/ramp")
      out.append({"t": round(S[i].t), "where": where(S[i]), "side": side, "deg": deg, "kind": kind,
                  "exit_accel": exit_accel(S, j),
                  "entry_mph": S[i].v * MPH, "min_mph": min(s.v for s in seg) * MPH,
                  "peak_lat_accel": max(abs(s.v * s.yaw) for s in seg),
                  "blinker_lead_s": lead_s, "blinker_lead_m": lead_m,
                  "lat_engaged_at_entry": S[i].lat, "lat_engaged_pct": 100 * sum(s.lat for s in seg) / len(seg),
                  "driver_steered_pct": 100 * sum(s.steer_pressed for s in seg) / len(seg),
                  "unwind_deg_s": unwind_rate(S, i, j, sign)})
    i = j + 1
  return out


def curves(S: list[Sample]) -> dict:
  bands = {"20-30 mph": (9, 13.4), "30-45 mph": (13.4, 20), "45+ mph": (20, 60)}
  res = {}
  for name, (lo, hi) in bands.items():
    la = [abs(s.v * s.yaw) for s in S if lo <= s.v < hi and abs(s.yaw) > 0.03]
    if len(la) > 30:
      res[name] = {"n_s": len(la) / 10, "p50": pct(la, .5), "p95": pct(la, .95), "max": max(la)}
  return res


def following(S: list[Sample]) -> dict:
  res = {}
  for label, acc in (("driver", False), ("car ACC", True)):
    gaps = [s.lead_d / s.v for s in S if s.v > 8 and 0 < s.lead_d < 100 and s.cruise == acc and abs(s.lead_v - s.v) < 3]
    if len(gaps) > 50:
      res[label] = {"n_s": len(gaps) / 10, "p25_s": pct(gaps, .25), "p50_s": pct(gaps, .5), "p75_s": pct(gaps, .75)}
  return res


def takeovers(S: list[Sample]) -> dict:
  """Steering disengagements while moving, by what the driver was doing at that moment."""
  why: Counter = Counter()
  spots = []
  for p, s in zip(S, S[1:], strict=False):
    if p.lat and not s.lat and s.v > 1.0:
      if s.brake or p.brake:
        r = "brake"
      elif s.steer_pressed or p.steer_pressed:
        r = "steering"
      elif s.lb or s.rb:
        r = "blinker"
      else:
        r = "other"
      why[r] += 1
      spots.append({"t": round(s.t), "where": where(s), "why": r, "mph": s.v * MPH})
  overrides = sum(1 for p, s in zip(S, S[1:], strict=False) if s.lat and s.steer_pressed and not p.steer_pressed)
  return {"disengagements": dict(why), "steering_overrides_while_engaged": overrides, "spots": spots}


def analyze(S: list[Sample]) -> dict:
  return {"overview": overview(S), "speed_vs_limit": speed_vs_limit(S), "stops": stops(S),
          "rolling_slowdowns": rolling_slowdowns(S), "turns": turns(S), "curves": curves(S),
          "following": following(S), "takeovers": takeovers(S)}


# ---- report ----

def fmt(x, nd=1) -> str:
  if x is None or (isinstance(x, float) and math.isnan(x)):
    return "-"
  return f"{x:.{nd}f}" if isinstance(x, float) else str(x)


def report(results: dict[str, dict]) -> None:
  all_stops = [s for r in results.values() for s in r["stops"]]
  all_turns = [t for r in results.values() for t in r["turns"]]
  for route, r in results.items():
    o = r["overview"]
    print(f"\n=== {route}: {o['miles']:.1f} mi, {o['moving_min']:.0f} min moving, steering engaged {o['steering_engaged_pct']:.0f}%, " +
          f"car ACC {o['car_acc_pct']:.0f}%")
    t = r["takeovers"]
    print(f"  steering disengagements {t['disengagements']}  overrides while engaged {t['steering_overrides_while_engaged']}")

  print("\n--- speed vs posted limit (driver's foot only) ---")
  merged: dict[int, list] = {}
  for r in results.values():
    for lim, d in r["speed_vs_limit"].items():
      merged.setdefault(lim, []).append(d)
  for lim, ds in sorted(merged.items()):
    n = sum(d["n_s"] for d in ds)
    med, p85, over = (sum(d[k] * d["n_s"] for d in ds) / n for k in ("median_over_mph", "p85_over_mph", "pct_over_5"))
    print(f"  {lim} mph zone: {n:.0f} s, median {med:+.1f} mph, p85 {p85:+.1f}, >5 over {over:.0f}% of the time")

  print(f"\n--- full stops: {len(all_stops)} ---")
  if all_stops:
    for k, nd in (("from_mph", 0), ("brake_dist_m", 0), ("avg_decel", 2), ("peak_decel", 2), ("friction_brake_pct", 0),
                  ("stopped_s", 1), ("to_20mph_s", 1), ("launch_accel", 2)):
      vals = [s[k] for s in all_stops if s[k] is not None]
      print(f"  {k:20s} p25 {fmt(pct(vals, .25), nd):>6}  median {fmt(pct(vals, .5), nd):>6}  p75 {fmt(pct(vals, .75), nd):>6}")
  rolls = [x for r in results.values() for x in r["rolling_slowdowns"]]
  print(f"  slow-rolls (< 5 mph, no stop): {len(rolls)}")

  print(f"\n--- turns >= {TURN_MIN_DEG:.0f} deg: {len(all_turns)} ---")
  for kind in ("junction", "curve/ramp", "u/loop"):
    ts = [t for t in all_turns if t["kind"] == kind]
    if not ts:
      continue
    lead = [t["blinker_lead_s"] for t in ts if t["blinker_lead_s"] is not None]
    print(f"  {kind}: {len(ts)}, entry mph median {fmt(pct([t['entry_mph'] for t in ts], .5), 0)}, " +
          f"min mph median {fmt(pct([t['min_mph'] for t in ts], .5), 0)}, " +
          f"peak lat accel median {fmt(pct([t['peak_lat_accel'] for t in ts], .5), 2)} m/s2, " +
          f"blinker used {len(lead)}/{len(ts)}, lead median {fmt(pct(lead, .5))} s, " +
          f"engaged at entry {sum(t['lat_engaged_at_entry'] for t in ts)}/{len(ts)}")
  if all_turns:
    j = [t for t in all_turns if t["kind"] == "junction"]
    over12 = sum(t["entry_mph"] > 12 for t in j)
    late = sum(t["blinker_lead_m"] is not None and t["blinker_lead_m"] < 10 for t in j)
    print(f"  junction turns entered above 12 mph: {over12}/{len(j)};  blinker < 10 m before: {late};  no blinker: " +
          f"{sum(t['blinker_lead_s'] is None for t in j)}")

  print("\n--- curves: lateral accel (m/s2) ---")
  cm: dict[str, list] = {}
  for r in results.values():
    for b, d in r["curves"].items():
      cm.setdefault(b, []).append(d)
  for b, ds in cm.items():
    print(f"  {b}: p50 {max(d['p50'] for d in ds):.2f}, p95 {max(d['p95'] for d in ds):.2f}, max {max(d['max'] for d in ds):.2f}")

  print("\n--- following gap (s) behind a steady lead above 18 mph ---")
  for r in results.values():
    for who, d in r["following"].items():
      print(f"  {who:8s} {d['n_s']:.0f} s: p25 {d['p25_s']:.2f}  median {d['p50_s']:.2f}  p75 {d['p75_s']:.2f}")

  print("\n--- steering takeover spots (steering / other) ---")
  for r in results.values():
    for s in r["takeovers"]["spots"]:
      if s["why"] in ("steering", "other"):
        print(f"  {s['why']:8s} {s['mph']:4.0f} mph  {s['where']}")


def main(argv: list[str] | None = None) -> int:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("routes", nargs="+")
  ap.add_argument("--root", default=DEFAULT_ROOT)
  ap.add_argument("--json", help="also write the full per-event results here")
  args = ap.parse_args(argv)
  results = {}
  for route in args.routes:
    S = load(args.root, route)
    if len(S) < 100:
      print(f"{route}: too little data under {args.root}", file=sys.stderr)
      continue
    results[route] = analyze(S)
  if not results:
    return 1
  report(results)
  if args.json:
    with open(args.json, "w") as f:
      json.dump(results, f, indent=1, default=lambda o: asdict(o) if hasattr(o, "__dataclass_fields__") else str(o))
  return 0


if __name__ == "__main__":
  sys.exit(main())
