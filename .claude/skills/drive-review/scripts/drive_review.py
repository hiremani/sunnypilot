"""Per-route review of the new features: engagements, openpilot long, alerts, stop sign / narrow / turns.
usage: drive_review.py ROOT ROUTE [ROUTE...]"""
import glob, os, sys
from collections import Counter, defaultdict
from openpilot.tools.lib.logreader import LogReader

MPH = 2.23694


def segs(root, r):
  return sorted(glob.glob(os.path.join(root, r + "--*")), key=lambda p: int(p.rsplit("--", 1)[1]))


def review(root, r):
  t0 = None
  st = defaultdict(float)
  events = Counter()
  ev_times = defaultdict(list)
  alerts = Counter()
  long_on_prev = lat_on_prev = False
  long_spans, lat_spans = [], []
  ls = lt = None
  v = a = 0.0
  gas = brake = False
  accel_err = []
  last_accel = None
  sources = Counter()
  sp_src = Counter()
  narrow_s = 0.0
  msgs = Counter()
  disengage = []
  prev_enabled = False
  miles = 0.0
  last_t = None
  cruise_on = False
  for s in segs(root, r):
    f = os.path.join(s, "rlog.zst")
    if not os.path.exists(f):
      f = os.path.join(s, "qlog.zst")
      if not os.path.exists(f):
        continue
    try:
      lr = list(LogReader(f))
    except Exception as e:
      print("  bad", f, e)
      continue
    for m in lr:
      t = m.logMonoTime * 1e-9
      if t0 is None:
        t0 = t
      w = m.which()
      if w == "carState":
        cs = m.carState
        if last_t is not None:
          miles += cs.vEgo * (t - last_t) / 1609.34
        last_t = t
        v, a, gas, brake = cs.vEgo, cs.aEgo, cs.gasPressed, cs.brakePressed
        cruise_on = cs.cruiseState.enabled
        if cs.vEgo > 0.5:
          st["moving_s"] += 0.01
          if cruise_on:
            st["acc_moving_s"] += 0.01
      elif w == "carControl":
        cc = m.carControl
        lo, la = cc.longActive, cc.latActive
        if lo and not long_on_prev:
          ls = t
        if not lo and long_on_prev and ls is not None:
          long_spans.append((ls - t0, t - ls, v * MPH, gas, brake))
        if la and not lat_on_prev:
          lt = t
        if not la and lat_on_prev and lt is not None:
          lat_spans.append((lt - t0, t - lt))
        long_on_prev, lat_on_prev = lo, la
        if lo:
          st["long_s"] += 0.01
          if v > 0.5:
            st["long_moving_s"] += 0.01
          last_accel = cc.actuators.accel
          if not gas and v > 1.0:
            accel_err.append(a - cc.actuators.accel)
        if la and v > 0.5:
          st["lat_moving_s"] += 0.01
      elif w == "selfdriveState":
        ss = m.selfdriveState
        if ss.alertText1:
          alerts[(ss.alertText1, ss.alertText2[:50])] += 1
        if prev_enabled and not ss.enabled:
          disengage.append(t - t0)
        prev_enabled = ss.enabled
      elif w in ("onroadEvents", "onroadEventsSP"):
        lst = m.onroadEvents if w == "onroadEvents" else m.onroadEventsSP.events
        for e in lst:
          n = str(e.name)
          events[n] += 1
          if not ev_times[n] or t - t0 - ev_times[n][-1] > 5:
            ev_times[n].append(t - t0)
      elif w == "longitudinalPlan":
        sources[str(m.longitudinalPlan.longitudinalPlanSource)] += 1
      elif w == "longitudinalPlanSP":
        lp = m.longitudinalPlanSP
        try:
          sp_src[str(lp.longitudinalPlanSource)] += 1
        except Exception:
          pass
        try:
          if lp.narrowStreet:
            narrow_s += 0.05
        except Exception:
          pass
      elif w == "logMessage":
        lm = m.logMessage
        if any(k in lm for k in ("rror", "xception", "Traceback", "WARNING", "lag", "abort")):
          import json
          try:
            mm = json.loads(lm)
            txt = str(mm.get("msg"))[:100]
            d = mm.get("ctx", {}).get("daemon", "")
          except Exception:
            txt, d = lm[:100], ""
          msgs[(d, txt)] += 1
  print(f"\n=== {r}: {miles:.1f} mi, moving {st['moving_s']/60:.1f} min")
  print(f"  steering active while moving {st['lat_moving_s']/60:.1f} min; car ACC on while moving {st['acc_moving_s']/60:.1f} min;"
        f" openpilot long active {st['long_s']/60:.2f} min ({st['long_moving_s']/60:.2f} moving)")
  print(f"  long engagements: {len(long_spans)}")
  for s0, d, vm, g, b in long_spans[:25]:
    print(f"    at {s0/60:6.1f} min  lasted {d:6.1f}s  ended at {vm:4.1f} mph gas={g} brake={b}")
  if accel_err:
    import statistics
    print(f"  aEgo - accel cmd (no gas, moving): median {statistics.median(accel_err):+.2f}  p10 {sorted(accel_err)[len(accel_err)//10]:+.2f}"
          f"  p90 {sorted(accel_err)[len(accel_err)*9//10]:+.2f}  n={len(accel_err)}")
  print(f"  steering engagements: {len(lat_spans)}, disengages (selfdrive): {len(disengage)}")
  print("  plan sources:", dict(sources.most_common()), "SP:", dict(sp_src.most_common()))
  print(f"  narrowStreet planner flag: {narrow_s:.0f} s")
  print("  events (count, first times min):")
  for n, c in events.most_common(30):
    print(f"    {n:32s} {c:6d}  x{len(ev_times[n])}  " + " ".join(f"{x/60:.1f}" for x in ev_times[n][:10]))
  print("  alerts:")
  for (a1, a2), c in alerts.most_common(15):
    print(f"    {c/100:7.1f}s  {a1} / {a2}")
  print("  log messages:")
  for (d, txt), c in msgs.most_common(15):
    print(f"    {c:5d} {d:10s} {txt}")


if __name__ == "__main__":
  root = sys.argv[1]
  for r in sys.argv[2:]:
    review(root, r)
