"""Every stop to standstill in one or more routes: who was braking, how smooth it was, and what happened after.

Usage (on the device, or the Mac with rlogs under ROOT):  stops.py ROOT ROUTE [ROUTE...] [--segs 12,14,15]

Per stop:
  who        op-long (openpilot speed control commanding), car-ACC (the car's own cruise), driver (brake pedal / no cruise)
  decel      peak deceleration over the last 6 s, m/s^2
  final_jerk peak |jerk| over the last 0.5 s before standstill (the "head nod"), m/s^3, from aEgo smoothed over 0.1 s
  a_end      deceleration in the last 0.1 s before standstill (a "chauffeur stop" ends near 0)
  req_err    op-long only: median |requested accel - actual| over the last 6 s, m/s^2
  rolled_m   distance moved in the 3 s after the stop flag, metres (a held stop is ~0)
  acc        ADAS_Sts_ACC_ICC values seen around the stop (from the fisker_long_handover log lines, when present)
"""
import argparse
import bisect
import glob
import json
import os

from openpilot.tools.lib.logreader import LogReader

WINDOW_S = 6.0
FINAL_S = 0.5
SMOOTH_N = 10         # carState is 100 Hz: 0.1 s
AFTER_S = 3.0
MOVING_MPS = 3.0      # a new stop needs the car to have been above this since the last one


def segments(root, route, segs):
  dirs = sorted(glob.glob(os.path.join(root, f"{route}--*")), key=lambda p: int(p.rsplit("--", 1)[1]))
  if segs:
    keep = {int(s) for s in segs.split(",")}
    dirs = [d for d in dirs if int(d.rsplit("--", 1)[1]) in keep]
  return [f for d in dirs for f in [*glob.glob(os.path.join(d, "rlog.zst")), *glob.glob(os.path.join(d, "rlog"))][:1]]


def smooth(xs, n):
  out, acc = [], 0.0
  for i, x in enumerate(xs):
    acc += x
    if i >= n:
      acc -= xs[i - n]
    out.append(acc / min(i + 1, n))
  return out


def analyse(files):
  rows = []   # t, v, a, standstill, brake, cruise, long_active, accel_req
  acc_events = []   # t, acc_state
  t0 = None
  long_active, accel_req = False, 0.0
  for f in files:
    try:
      for m in LogReader(f):
        w = m.which()
        t = m.logMonoTime * 1e-9
        if t0 is None:
          t0 = t
        if w == "carControl":
          long_active, accel_req = m.carControl.longActive, m.carControl.actuators.accel
        elif w == "carState":
          c = m.carState
          rows.append((t, c.vEgo, c.aEgo, c.standstill, c.brakePressed, c.cruiseState.enabled, long_active, accel_req))
        elif w == "logMessage" and "fisker_long_handover" in m.logMessage:
          try:
            msg = json.loads(m.logMessage).get("msg", {})
            if isinstance(msg, dict):
              acc_events.append((t, msg.get("acc_state")))
          except ValueError:
            pass
    except Exception as e:   # a damaged segment: keep the rest
      print("skipped", f, e)
  ts = [r[0] for r in rows]
  stops, moving = [], False
  for i in range(1, len(rows)):
    t, v = rows[i][0], rows[i][1]
    if v > MOVING_MPS:
      moving = True
    if moving and rows[i][3] and not rows[i - 1][3]:
      moving = False
      a0 = bisect.bisect_left(ts, t - WINDOW_S)
      win = rows[a0:i + 1]
      a_s = smooth([r[2] for r in win], SMOOTH_N)
      jerk = [(a_s[k + 1] - a_s[k]) / max(win[k + 1][0] - win[k][0], 1e-3) for k in range(len(win) - 1)]
      jerk_s = smooth(jerk, SMOOTH_N)
      nf = sum(1 for r in win if t - r[0] <= FINAL_S)
      final_jerk = max((abs(j) for j in jerk_s[-nf:]), default=0.0)
      n_end = sum(1 for r in win if t - r[0] <= 0.1)
      a_end = sum(a_s[-n_end:]) / max(n_end, 1)
      late = win[-100:]
      who = ("op-long" if any(r[6] for r in late) else "driver" if any(r[4] for r in late)
             else "car-ACC" if any(r[5] for r in late) else "driver")
      req_err = None
      if who == "op-long":
        errs = sorted(abs(r[7] - r[2]) for r in win if r[6])
        req_err = errs[len(errs) // 2] if errs else None
      b = bisect.bisect_right(ts, t + AFTER_S)
      rolled = sum(rows[k][1] * (rows[k][0] - rows[k - 1][0]) for k in range(i + 1, min(b, len(rows))))
      accs = sorted({s for (te, s) in acc_events if t - WINDOW_S <= te <= t + AFTER_S})
      stops.append({"min": round((t - t0) / 60, 2), "who": who, "decel": round(min(a_s), 2),
                    "final_jerk": round(final_jerk, 2), "a_end": round(a_end, 2),
                    "req_err": None if req_err is None else round(req_err, 2), "rolled_m": round(rolled, 2),
                    "acc": accs})
  return stops


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("root")
  ap.add_argument("routes", nargs="+")
  ap.add_argument("--segs", default=None, help="comma-separated segment numbers (one route only)")
  args = ap.parse_args()
  for route in args.routes:
    stops = analyse(segments(args.root, route, args.segs))
    print(f"{route}: {len(stops)} stops")
    print("    min  who       decel  final_jerk  a_end  req_err  rolled_m  acc")
    for s in stops:
      print(f"  {s['min']:5.1f}  {s['who']:8s} {s['decel']:+6.2f}  {s['final_jerk']:9.2f}  {s['a_end']:+5.2f}  "
            f"{'' if s['req_err'] is None else s['req_err']:>7}  {s['rolled_m']:8.2f}  {s['acc']}")
    by = {}
    for s in stops:
      by.setdefault(s["who"], []).append(s["final_jerk"])
    for who, js in by.items():
      js.sort()
      print(f"  {who}: {len(js)} stops, final-0.5 s jerk median {js[len(js) // 2]:.2f}, worst {js[-1]:.2f} m/s^3")
  print("ALLDONE")


if __name__ == "__main__":
  main()
