#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

A short summary of each drive, written on the device once the car is off, for the web HUD and for review.

  drive_summary.py ROUTE [--root DIR]     summarize one route and print it
  drive_summary.py [--watch]              no route: the manager process (offroad only, DriveSummary param):
                                          summarize every finished route that has no summary yet, newest first

Reads the qlogs (10 Hz carState/carControl, events) and, for the planners' own log events, the rlogs' logMessage.
Writes <media>/drive_summaries/<route>.json:
  distance, time, steering and openpilot speed-control time and how each speed-control run ended, takeovers, alerts,
  lag clusters, map turn assist turns (start, side, how they ended), stop sign and speed bump passes (live or log-only),
  error messages by process, and a few plain "headline" lines.
"""
import argparse
import glob
import json
import os
import sys
import time
from collections import Counter, defaultdict

MPH = 2.23694
SUMMARY_DIR = "drive_summaries"
SETTLE_S = 180.0       # a route whose newest file is younger than this may still be recording
WATCH_EVERY_S = 60.0
LAG_EVENTS = ("selfdrivedLagging", "commIssue", "commIssueAvgFreq")
MAX_ALERTS = 12
# logged at error level on every start, not faults (route 000000f9: fingerprinting, init, the NNLC model check, a
# locationd reset on the first fix)
STARTUP_NOISE_EVENTS = {"fingerprinted", "selfdrived.initialized"}
STARTUP_NOISE_TEXT = ("car doesn't match any Neural Network model", "kalman reset")


def _segments(root: str, route: str) -> list[str]:
  return sorted(glob.glob(os.path.join(root, route + "--*")), key=lambda p: int(p.rsplit("--", 1)[1]))


def _reader(path: str):
  from openpilot.tools.lib.logreader import LogReader
  try:
    return LogReader(path)
  except Exception:
    return []


def _clusters(times: list[float], gap: float = 5.0) -> list[tuple[float, float]]:
  out: list[list[float]] = []
  for t in sorted(times):
    if out and t - out[-1][1] <= gap:
      out[-1][1] = t
    else:
      out.append([t, t])
  return [(a, b) for a, b in out]


def summarize(root: str, route: str) -> dict:
  t0 = None
  miles = moving_s = lat_s = long_s = 0.0
  v_max = 0.0
  last_t = None
  long_prev = False
  long_start = 0.0
  long_runs = []
  v = 0.0
  gas = brake = False
  alerts: Counter = Counter()
  alert_first: dict = {}
  events: Counter = Counter()
  lag_t: list[float] = []
  steer_override_t: list[float] = []
  # qlogs: driving data and events
  for seg in _segments(root, route):
    f = os.path.join(seg, "qlog.zst")
    if not os.path.exists(f):
      continue
    for m in _reader(f):
      t = m.logMonoTime * 1e-9
      t0 = t if t0 is None else t0
      w = m.which()
      if w == "carState":
        cs = m.carState
        v, gas, brake = cs.vEgo, cs.gasPressed, cs.brakePressed
        if last_t is not None and t - last_t < 1.0:
          dt = t - last_t
          miles += v * dt / 1609.34
          moving_s += dt if v > 0.5 else 0.0
        last_t = t
        v_max = max(v_max, v)
      elif w == "carControl":
        cc = m.carControl
        if last_t is not None:
          lat_s += 0.1 if cc.latActive and v > 0.5 else 0.0
          long_s += 0.1 if cc.longActive else 0.0
        if cc.longActive and not long_prev:
          long_start = t
        if long_prev and not cc.longActive:
          long_runs.append({"at_min": round((long_start - t0) / 60, 1), "s": round(t - long_start, 1),
                            "ended": "gas" if gas else "brake" if brake else "other", "mph": round(v * MPH, 1)})
        long_prev = cc.longActive
      elif w == "selfdriveState":
        ss = m.selfdriveState
        if ss.alertText1:
          key = ss.alertText1 + (" / " + ss.alertText2 if ss.alertText2 else "")
          alerts[key] += 1
          alert_first.setdefault(key, round((t - t0) / 60, 1))
      elif w in ("onroadEvents", "onroadEventsSP"):
        lst = m.onroadEvents if w == "onroadEvents" else m.onroadEventsSP.events
        for e in lst:
          name = str(e.name)
          events[name] += 1
          if name in LAG_EVENTS:
            lag_t.append(t - t0)
          elif name == "steerOverride":
            steer_override_t.append(t - t0)
  if t0 is None:
    return {"route": route, "error": "no qlogs"}

  # rlogs: the planners' own log events
  turns, stops, bumps = [], defaultdict(dict), defaultdict(dict)
  wet_changes: list[tuple[float, str, str, int]] = []   # (t, level, applied, wiper)
  t_end = 0.0
  errors: Counter = Counter()
  cur_turn = None
  for seg in _segments(root, route):
    f = os.path.join(seg, "rlog.zst")
    if not os.path.exists(f):
      continue
    for m in _reader(f):
      if m.which() != "logMessage":
        continue
      try:
        j = json.loads(m.logMessage)
      except ValueError:
        continue
      msg, t = j.get("msg"), m.logMonoTime * 1e-9 - t0
      t_end = max(t_end, t)
      noise = (isinstance(msg, dict) and msg.get("event") in STARTUP_NOISE_EVENTS) or \
        any(x in str(msg) for x in STARTUP_NOISE_TEXT)
      if j.get("levelnum", 0) >= 40 and not noise:
        errors[(j.get("ctx", {}).get("daemon", "?"), str(msg)[:80])] += 1
      if not isinstance(msg, dict):
        continue
      ev = msg.get("event")
      if ev == "map_turn":
        if msg.get("state") == "turning":
          cur_turn = {"at_min": round(t / 60, 2), "side": msg.get("side"), "planned_deg": msg.get("planned_deg"),
                      "mph": round((msg.get("v") or 0) * MPH, 1)}
        elif msg.get("prev") == "turning" and cur_turn is not None:
          cur_turn.update(ended=msg.get("reason"), s=round(t - cur_turn["at_min"] * 60, 1),
                          turned_deg=msg.get("progress_deg"))
          turns.append(cur_turn)
          cur_turn = None
      elif ev in ("stop_sign", "stop_sign_confirmed", "stop_sign_left_to_driver"):
        sign = msg.get("sign")
        if sign is None:
          continue
        s = stops[(sign, round(t / 120))]
        s.setdefault("at_min", round(t / 60, 2))
        s.update(sign=sign, shadow=msg.get("shadow"))
        if ev == "stop_sign_confirmed":
          s["confirmed"] = True
        elif ev == "stop_sign_left_to_driver":
          s["left_to_driver"] = msg.get("why")
        else:
          s.setdefault("states", []).append(msg.get("state"))
      elif ev == "wet_road":
        wet_changes.append((t, msg.get("level", "dry"), msg.get("applied", "dry"), msg.get("wiper") or 0))
      elif ev in ("speed_bump", "speed_bump_left_to_driver"):
        bump = msg.get("bump")
        if bump is None:
          continue
        b = bumps[(bump, round(t / 120))]
        b.setdefault("at_min", round(t / 60, 2))
        b.update(bump=bump, shadow=msg.get("shadow"))
        if ev == "speed_bump_left_to_driver":
          b["left_to_driver"] = True
        elif msg.get("state") == "easing":
          b.setdefault("eased_from_m", msg.get("dist"))
          b.setdefault("mph_start", round((msg.get("v") or 0) * MPH, 1))
          b["cross_mph"] = round((msg.get("v_cross") or 0) * MPH, 1)
        elif msg.get("state") == "crossing":
          b["mph_at_bump"] = round((msg.get("v") or 0) * MPH, 1)

  lag = [{"at_min": round(a / 60, 1), "s": round(b - a, 1)} for a, b in _clusters(lag_t)]
  out = {
    "route": route, "summarized": time.strftime("%Y-%m-%dT%H:%M:%S"),
    "miles": round(miles, 1), "moving_min": round(moving_s / 60, 1), "max_mph": round(v_max * MPH),
    "steering_min": round(lat_s / 60, 1), "steer_overrides": len(_clusters(steer_override_t, 2.0)),
    "openpilot_speed_min": round(long_s / 60, 1), "openpilot_speed_runs": long_runs,
    "alerts": [{"text": k, "s": round(n / 10, 1), "first_min": alert_first[k]} for k, n in alerts.most_common(MAX_ALERTS)],
    "events": dict(events.most_common(30)),
    "lag": lag,
    "map_turns": turns,
    "stop_signs": list(stops.values()),
    "speed_bumps": list(bumps.values()),
    "wet_road": wet_summary(wet_changes, t_end),
    "errors": [{"process": d, "msg": msg, "n": n} for (d, msg), n in errors.most_common(10)],
  }
  out["headline"] = headline(out)
  return out


def wet_summary(changes: list[tuple[float, str, str, int]], t_end: float) -> dict:
  """Minutes in each wet road level from the planner's wet_road events (one per change)."""
  mins = {"wet": 0.0, "heavy": 0.0, "applied": 0.0}
  for i, (t, level, applied, _) in enumerate(changes):
    t_next = changes[i + 1][0] if i + 1 < len(changes) else t_end
    dur = max(t_next - t, 0.0) / 60
    if level in mins:
      mins[level] += dur
    if applied != "dry":
      mins["applied"] += dur
  return {"wet_min": round(mins["wet"], 1), "heavy_min": round(mins["heavy"], 1), "applied_min": round(mins["applied"], 1),
          "max_wiper": max((c[3] for c in changes), default=0)}


def headline(s: dict) -> list[str]:
  lines = [f"{s['miles']} mi, {s['moving_min']} min moving; openpilot steered {s['steering_min']} min"]
  if s["openpilot_speed_runs"]:
    ended = Counter(r["ended"] for r in s["openpilot_speed_runs"])
    ends = ", ".join(f"{k} {n}" for k, n in ended.items())
    lines.append(f"openpilot speed control {s['openpilot_speed_min']} min in {len(s['openpilot_speed_runs'])} runs (ended: {ends})")
  if s["lag"]:
    lines.append(f"lag warnings {len(s['lag'])}x, first at {s['lag'][0]['at_min']} min")
  if s["map_turns"]:
    ended = Counter(t.get("ended") for t in s["map_turns"])
    lines.append(f"turn assist steered {len(s['map_turns'])} turns ({', '.join(f'{k} {n}' for k, n in ended.items())})")
  if s["speed_bumps"]:
    shadow = all(b.get("shadow") for b in s["speed_bumps"])
    lines.append(f"{len(s['speed_bumps'])} speed bumps {'seen (log only)' if shadow else 'eased for'}")
  wet = s.get("wet_road") or {}
  if wet.get("wet_min", 0) + wet.get("heavy_min", 0) > 0:
    how = "driven cautiously" if wet.get("applied_min", 0) > 0 else "log only"
    lines.append(f"wet road {wet['wet_min']} min, heavy rain {wet['heavy_min']} min ({how})")
  if s["stop_signs"]:
    lines.append(f"{len(s['stop_signs'])} stop signs approached")
  if s["errors"]:
    lines.append(f"{sum(e['n'] for e in s['errors'])} error log lines (top: {s['errors'][0]['process']})")
  return lines


def summary_dir(root: str) -> str:
  return os.path.join(os.path.dirname(os.path.normpath(root)), SUMMARY_DIR)


def finished_routes(root: str, now: float) -> list[str]:
  """Routes whose newest file is at least SETTLE_S old, newest first."""
  newest: dict[str, float] = {}
  for seg in glob.glob(os.path.join(root, "*--*--*")):
    route = os.path.basename(seg).rsplit("--", 1)[0]
    try:
      mt = max((os.path.getmtime(os.path.join(seg, f)) for f in os.listdir(seg)), default=0.0)
    except OSError:
      continue
    newest[route] = max(newest.get(route, 0.0), mt)
  return [r for r, mt in sorted(newest.items(), key=lambda kv: -kv[1]) if now - mt >= SETTLE_S]


def write_summary(root: str, route: str) -> str:
  s = summarize(root, route)
  d = summary_dir(root)
  os.makedirs(d, exist_ok=True)
  path = os.path.join(d, route + ".json")
  tmp = path + ".tmp"
  with open(tmp, "w") as f:
    json.dump(s, f, indent=1)
  os.replace(tmp, path)
  return path


def watch(root: str) -> None:
  """The manager process: offroad only. Summarize finished routes that have none yet (at most the 5 newest that are
  missing, so the first run doesn't chew through the whole disk), then wait."""
  from openpilot.common.params import Params
  from openpilot.common.swaglog import cloudlog
  try:
    os.nice(19)
  except OSError:
    pass
  params = Params()
  while True:
    try:
      if params.get_bool("DriveSummary"):
        done = {f[:-5] for f in os.listdir(summary_dir(root))} if os.path.isdir(summary_dir(root)) else set()
        now = time.time()  # noqa: TID251 (compared with file mtimes, which are wall-clock)
        for route in [r for r in finished_routes(root, now) if r not in done][:5]:
          if not params.get_bool("IsOffroad"):
            break
          path = write_summary(root, route)
          cloudlog.event("drive_summary", route=route, path=path)
    except Exception:
      cloudlog.exception("drive summary failed")
    time.sleep(WATCH_EVERY_S)


def main(argv=None) -> int:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("route", nargs="?")
  ap.add_argument("--root", default="/data/media/0/realdata")
  ap.add_argument("--watch", action="store_true")
  ap.add_argument("--write", action="store_true", help="also write the summary file")
  args = ap.parse_args(argv if argv is not None else [])   # the manager calls main() with no arguments: watch
  if args.watch or not args.route:
    watch(args.root)
    return 0
  if args.write:
    print(write_summary(args.root, args.route))
  s = summarize(args.root, args.route)
  print("\n".join(s.get("headline", [s.get("error", "")])))
  print(json.dumps({k: v for k, v in s.items() if k != "headline"}, indent=1)[:4000])
  return 0


if __name__ == "__main__":
  sys.exit(main(sys.argv[1:]))
