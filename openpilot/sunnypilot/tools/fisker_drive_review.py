#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Review a Fisker Ocean drive from its rlogs, on the device or on a PC with the route copied over.

  fisker_drive_review.py summary      ROUTE   steering engagement, EPS state/aborts, overrides, tracking, SecOC key
  fisker_drive_review.py turns        ROUTE   every signaled turn below 20 mph: EPS, our TX, driver torque, turn desire
  fisker_drive_review.py stopsigns    ROUTE   Stop Sign Assist events (live or shadow) next to the driver's real stops
  fisker_drive_review.py replay-stops ROUTE   run Stop Sign Assist over the drive as if openpilot long were engaged

ROUTE is the route name ("000000e3--e8d98c1676"); --root is the folder holding its segment folders
(default /data/media/0/realdata, the device's). Use --segments to limit the work, e.g. --segments 2-4.
"""
import argparse
import glob
import json
import math
import os
import sys
from collections import Counter
from collections.abc import Iterator

from opendbc.can.dbc import DBC
from opendbc.can.parser import get_raw_value
from openpilot.tools.lib.logreader import LogReader

DEFAULT_ROOT = "/data/media/0/realdata"
MPH = 2.23694
EPS_STS = {0: "off", 1: "available", 2: "active", 3: "failure"}
DESIRES = ["none", "turnLeft", "turnRight", "laneChangeLeft", "laneChangeRight", "keepLeft", "keepRight"]
LAT_CANDIDATE_ADDR = 0x5D0
TURN_MAX_SPEED = 20 / MPH   # m/s


# ---- reading ----

def segment_dirs(root: str, route: str, segments: str | None = None) -> list[str]:
  dirs = sorted(glob.glob(os.path.join(root, f"{route}--*")), key=lambda p: int(p.rsplit("--", 1)[1]))
  if segments:
    lo, _, hi = segments.partition("-")
    lo_i, hi_i = int(lo), int(hi or lo)
    dirs = [d for d in dirs if lo_i <= int(d.rsplit("--", 1)[1]) <= hi_i]
  return dirs


def log_file(seg: str) -> str | None:
  for name in ("rlog.zst", "rlog", "rlog.bz2", "qlog.zst", "qlog"):
    p = os.path.join(seg, name)
    if os.path.exists(p):
      return p
  return None


def messages(segs: list[str]) -> Iterator[tuple[float, object]]:
  """(seconds since the route's first message, message) over the given segments, in order."""
  t0 = None
  for seg in segs:
    f = log_file(seg)
    if f is None:
      print(f"  (no log in {os.path.basename(seg)})", file=sys.stderr)
      continue
    for m in LogReader(f):
      if t0 is None:
        t0 = m.logMonoTime
      yield (m.logMonoTime - t0) / 1e9, m


class Signals:
  """Decode a few Fisker ADAS signals by name (EPS_0x1C2 / EPS_0x1C4)."""

  def __init__(self):
    dbc = DBC("fisker_ocean_adas")
    msgs = dbc.msgs.values() if isinstance(dbc.msgs, dict) else dbc.msgs
    self.by_addr = {m.address: m for m in msgs}

  def get(self, addr: int, dat: bytes, name: str) -> float:
    s = self.by_addr[addr].sigs[name]
    return get_raw_value(dat, s) * s.factor + s.offset


def percentile(sorted_vals: list[float], q: float) -> float:
  if not sorted_vals:
    return float("nan")
  return sorted_vals[min(int(len(sorted_vals) * q), len(sorted_vals) - 1)]


def group_windows(times: list[float], gap: float) -> list[tuple[float, float]]:
  """Merge sorted timestamps into (start, end) windows, splitting where consecutive times are more than gap apart."""
  out: list[tuple[float, float]] = []
  for t in times:
    if out and t - out[-1][1] <= gap:
      out[-1] = (out[-1][0], t)
    else:
      out.append((t, t))
  return out


# ---- summary ----

def cmd_summary(segs: list[str]) -> None:
  sig = Signals()
  lat = False
  cc_frames = lat_frames = tx = 0
  sts, aborts, abort_onsets = Counter(), Counter(), []
  prev_abort = 0
  intv = n1c4 = 0
  states, alerts, events, logs = Counter(), Counter(), Counter(), Counter()
  overriding_onsets, prev_state = 0, None
  err, last_cmd = [], None
  for t, m in messages(segs):
    w = m.which()
    if w == "carControl":
      lat = m.carControl.latActive
      cc_frames += 1
      lat_frames += lat
    elif w == "sendcan":
      tx += sum(1 for c in m.sendcan if c.address == LAT_CANDIDATE_ADDR)
    elif w == "can" and lat:
      for c in m.can:
        if c.src == 0 and c.address == 0x1C2:
          s, a = int(sig.get(0x1C2, c.dat, "EPS_AdasLatCtrlSts")), int(sig.get(0x1C2, c.dat, "EPS_AbortFb"))
          sts[s] += 1
          aborts[a] += 1
          if a and not prev_abort:
            abort_onsets.append((round(t, 1), a))
          prev_abort = a
        elif c.src == 0 and c.address == 0x1C4:
          n1c4 += 1
          intv += int(sig.get(0x1C4, c.dat, "EPS_DrvrIntvSteerWhlDetd"))
    elif w == "carOutput":
      last_cmd = m.carOutput.actuatorsOutput.steeringAngleDeg
    elif w == "carState" and lat and last_cmd is not None and prev_abort == 0:
      err.append(abs(last_cmd - m.carState.steeringAngleDeg))
    elif w == "selfdriveState":
      st = str(m.selfdriveState.state)
      states[st] += 1
      overriding_onsets += st == "overriding" and prev_state != "overriding"
      prev_state = st
      if m.selfdriveState.alertText1:
        alerts[m.selfdriveState.alertText1] += 1
    elif w == "onroadEvents":
      for e in m.onroadEvents:
        events[str(e.name)] += 1
    elif w in ("logMessage", "errorLogMessage"):
      s = getattr(m, w)
      if any(k in s for k in ("SecOC", "abort", "Abort", "fault", "LKAS", "stop sign")):
        try:
          logs[str(json.loads(s)["msg"])[:100]] += 1
        except (ValueError, KeyError, TypeError):
          logs[s[:100]] += 1

  total = sum(sts.values()) or 1
  print(f"{len(segs)} segments, ~{cc_frames / 6000:.1f} min")
  print(f"steering engaged (latActive): {lat_frames / 100:.0f} s ({100 * lat_frames / max(cc_frames, 1):.0f}%), " +
        f"0x5D0 candidate frames sent: {tx}")
  print("EPS lateral state while engaged:", {EPS_STS.get(k, k): f"{100 * v / total:.1f}%" for k, v in sorted(sts.items())})
  print("EPS abort reasons while engaged:", dict(aborts), " onsets:", abort_onsets[:20])
  print(f"EPS driver-intervention bit while engaged: {100 * intv / max(n1c4, 1):.1f}%")
  print("selfdrive states:", dict(states), " overriding onsets:", overriding_onsets)
  if err:
    err.sort()
    print(f"|command - wheel| while engaged: median {percentile(err, .5):.2f} deg, p95 {percentile(err, .95):.2f}, max {err[-1]:.1f}")
  print("alerts:", alerts.most_common(10))
  print("events:", events.most_common(12))
  print("logs:", logs.most_common(10))


# ---- turns ----

def cmd_turns(segs: list[str], step: float = 0.5) -> None:
  sig = Signals()
  cur: dict = {"tx": 0}
  last = -1.0
  prev_t = None
  print("   t     mph   wheel  eps        tx/step  drvTq  intv  pressed  mads         desire      curvature")
  for t, m in messages(segs):
    w = m.which()
    if w == "can":
      for c in m.can:
        if c.src == 0 and c.address == 0x1C2:
          cur["eps"] = EPS_STS.get(int(sig.get(0x1C2, c.dat, "EPS_AdasLatCtrlSts")), "?")
        elif c.src == 0 and c.address == 0x1C4:
          cur["tq"] = sig.get(0x1C4, c.dat, "EPS_DrvrSteerTq")
          cur["intv"] = int(sig.get(0x1C4, c.dat, "EPS_DrvrIntvSteerWhlDetd"))
    elif w == "sendcan":
      cur["tx"] += sum(1 for c in m.sendcan if c.address == LAT_CANDIDATE_ADDR)
    elif w == "modelV2":
      ds = list(m.modelV2.meta.desireState)
      cur["desire"] = DESIRES[max(range(len(ds)), key=lambda i: ds[i])] if ds else "?"
      cur["curv"] = m.modelV2.action.desiredCurvature
    elif w == "selfdriveStateSP":
      cur["mads"] = str(m.selfdriveStateSP.mads.state)
    elif w == "carState":
      cs = m.carState
      blinker = cs.leftBlinker != cs.rightBlinker
      if blinker and cs.vEgo < TURN_MAX_SPEED and t - last >= step:
        if prev_t is not None and t - prev_t > 3:
          print("   ---")
        side = "L" if cs.leftBlinker else "R"
        print(f"{t:7.1f} {side} {cs.vEgo * MPH:4.1f} {cs.steeringAngleDeg:6.0f}  {cur.get('eps', '?'):10s} {cur['tx']:5d}  " +
              f"{cur.get('tq', 0):5.1f}  {cur.get('intv', '-')!s:4s}  {cs.steeringPressed!s:7s}  {cur.get('mads', '?'):12s} " +
              f"{cur.get('desire', '?'):11s} {cur.get('curv', 0):+.4f}")
        last = prev_t = t
        cur["tx"] = 0


# ---- stop signs ----

def driver_stops(speeds: list[tuple[float, float]], min_s: float = 1.0, v_stop: float = 0.3) -> list[tuple[float, float]]:
  """(start, end) of every stretch at < v_stop m/s lasting at least min_s."""
  out, start = [], None
  for t, v in speeds:
    if v < v_stop:
      start = t if start is None else start
    else:
      if start is not None and t - start >= min_s:
        out.append((start, t))
      start = None
  if start is not None and speeds and speeds[-1][0] - start >= min_s:
    out.append((start, speeds[-1][0]))
  return out


def cmd_stopsigns(segs: list[str]) -> None:
  events, speeds, gps = [], [], []
  for t, m in messages(segs):
    w = m.which()
    if w in ("logMessage", "errorLogMessage"):
      s = getattr(m, w)
      if "stop_sign" in s:
        try:
          msg = json.loads(s).get("msg")
        except ValueError:
          continue
        if isinstance(msg, dict) and msg.get("event") == "stop_sign":
          events.append((t, msg))
    elif w == "carState":
      speeds.append((t, m.carState.vEgo))
    elif w in ("gpsLocationExternal", "gpsLocation"):
      g = getattr(m, w)
      if g.hasFix:
        gps.append((t, g.latitude, g.longitude))

  print(f"Stop Sign Assist events: {len(events)}")
  for t, e in events:
    print(f"{t:7.1f}s {'shadow' if e.get('shadow') else 'LIVE':6s} {e.get('prev', '?'):>8s} -> {e.get('state', '?'):8s} " +
          f"sign={e.get('sign')} d={e.get('dist')}m v={e.get('v', 0) * MPH:.1f}mph target={min(e.get('v_target', 0), 99) * MPH:.1f}mph " +
          f"confirmed={e.get('confirmed')} coast_at={e.get('coast_at')}m")
  print("\nDriver's real stops (>= 1 s below 0.7 mph):")
  for a, b in driver_stops(speeds):
    p = min(gps, key=lambda r: abs(r[0] - a)) if gps else None
    where = f"at {p[1]:.5f},{p[2]:.5f}" if p else "(no GPS)"
    print(f"{a:7.1f}s for {b - a:5.1f}s {where}")


def cmd_replay_stops(segs: list[str]) -> None:
  from types import SimpleNamespace
  from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_controller import StopSignAssist, coast_distance
  from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_map import StopSignMap
  from openpilot.sunnypilot.webhud.maptiles import FEAT_DEG

  class SyncMap(StopSignMap):
    """Load cells inline: a replay can wait, and must not miss the first signs while a thread loads them."""
    def signs_near(self, lat, lon):
      klat, klon = math.floor(lat / FEAT_DEG), math.floor(lon / FEAT_DEG)
      for key in [(klat + i, klon + j) for i in (-1, 0, 1) for j in (-1, 0, 1)]:
        if key not in self._cells:
          self._load(key)
      return super().signs_near(lat, lon)

  service = None
  params = SimpleNamespace(get_bool=lambda k: k == "StopSignAssist" or (k == "UbloxAvailable" and service == "gpsLocationExternal"))
  ctrl = None
  sm: dict = {}
  prev = None
  for t, m in messages(segs):
    w = m.which()
    if w in ("gpsLocationExternal", "gpsLocation"):
      if service is None and getattr(m, w).hasFix:
        service = w
        ctrl = StopSignAssist(stop_map=SyncMap(), params=params)
      sm[w] = getattr(m, w)
    elif w == "carState":
      sm["carState"] = m.carState
    elif w == "modelV2" and ctrl is not None and service in sm and "carState" in sm:
      sm["modelV2"] = m.modelV2
      cs, g = sm["carState"], sm[service]
      ctrl.update(sm, True, cs.gasPressed and ctrl.is_active, cs.vEgo, cs.aEgo, g.unixTimestampMillis * 1e-3 + 0.1)
      key = (ctrl.state, ctrl.node_id)
      if key != prev:
        tgt = "-" if ctrl.output_v_target > 200 else f"{ctrl.output_v_target * MPH:.1f}mph"
        print(f"{t:7.1f}s {ctrl.state.name:9s} sign={ctrl.node_id} d={ctrl.distance:6.1f}m v={cs.vEgo * MPH:4.1f}mph " +
              f"target={tgt} coast_at={coast_distance(cs.vEgo):5.1f}m confirmed={ctrl.confirmed}")
        prev = key
  if ctrl is None:
    print("no GPS fix in these segments")


def main(argv: list[str] | None = None) -> int:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("command", choices=("summary", "turns", "stopsigns", "replay-stops"))
  ap.add_argument("route")
  ap.add_argument("--root", default=DEFAULT_ROOT)
  ap.add_argument("--segments", help="N or N-M")
  args = ap.parse_args(argv)
  segs = segment_dirs(args.root, args.route, args.segments)
  if not segs:
    print(f"no segments of {args.route} under {args.root}", file=sys.stderr)
    return 1
  {"summary": cmd_summary, "turns": cmd_turns, "stopsigns": cmd_stopsigns, "replay-stops": cmd_replay_stops}[args.command](segs)
  return 0


if __name__ == "__main__":
  sys.exit(main())
