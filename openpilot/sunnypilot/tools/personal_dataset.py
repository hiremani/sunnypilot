#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Build the personal model's dataset from rlogs (run on the Mac): at 5 Hz, the features of
selfdrive/controls/lib/personal/features.py and the target, the driver's speed 2 s later minus the model's planned
speed at 2 s. A sample counts only when the driver chose the speed for the whole 2 s: the car's cruise off and
openpilot not in control of speed, in drive, not at a standstill.

Each drive is cached as <root>/_personal_cache/<route>.npz (redone when its rlog set changes).

  personal_dataset.py [--root ~/Fisker/routes] [ROUTE ...]     (no routes: every drive under root)
"""
import argparse
import glob
import json
import os
import re
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib.personal import features as pf

SEG_RE = re.compile(r"^([0-9a-f]{8}--[0-9a-f]{10})--(\d+)$")
CACHE_VERSION = 1
SAMPLE_EVERY = 4          # modelV2 runs at 20 Hz -> 5 Hz samples


def drives(root: str) -> dict[str, list[int]]:
  out: dict[str, list[int]] = {}
  for d in os.listdir(root):
    m = SEG_RE.match(d)
    if m and glob.glob(os.path.join(root, d, "rlog*")):
      out.setdefault(m.group(1), []).append(int(m.group(2)))
  return {r: sorted(s) for r, s in out.items()}


def build_route(root: str, route: str, segs: list[int]) -> dict:
  """Samples of one drive: X (n, N_FEATURES), y (n,), t (n,) seconds from the drive's first carState."""
  from openpilot.tools.lib.logreader import LogReader
  cs_t, cs_v, cs_ok = [], [], []
  snaps = []          # (t, Inputs, plan_v)
  long_active = False
  cs = rs = lm = None
  n_model = 0
  for seg in segs:
    f = glob.glob(os.path.join(root, f"{route}--{seg}", "rlog*"))
    try:
      for m in LogReader(f[0]):
        w = m.which()
        t = m.logMonoTime * 1e-9
        if w == "carControl":
          long_active = m.carControl.longActive
        elif w == "carState":
          cs = m.carState
          cs_t.append(t)
          cs_v.append(cs.vEgo)
          cs_ok.append(not cs.cruiseState.enabled and not long_active and str(cs.gearShifter) == "drive")
        elif w == "radarState":
          rs = m.radarState.leadOne
        elif w == "liveMapDataSP":
          lm = m.liveMapDataSP
        elif w == "modelV2" and cs is not None:
          n_model += 1
          if n_model % SAMPLE_EVERY:
            continue
          md = m.modelV2
          plan_v = list(md.velocity.x)
          if len(plan_v) < 10:
            continue
          x = pf.Inputs(
            v_ego=cs.vEgo, a_ego=cs.aEgo, plan_v=plan_v, plan_a=list(md.acceleration.x),
            plan_yaw_rate=list(md.orientationRate.z),
            lead_present=bool(rs.present) if rs is not None else False, lead_d=float(rs.dRel) if rs is not None else 0.0,
            lead_v_rel=float(rs.vRel) if rs is not None else 0.0,
            limit_valid=bool(lm.speedLimitValid) if lm is not None else False,
            limit=float(lm.speedLimit) if lm is not None else 0.0,
            ahead_valid=bool(lm.speedLimitAheadValid) if lm is not None else False,
            ahead_limit=float(lm.speedLimitAhead) if lm is not None else 0.0,
            ahead_dist=float(lm.speedLimitAheadDistance) if lm is not None else 0.0)
          snaps.append((t, x, plan_v))
    except Exception:
      pass
  if not snaps or not cs_t:
    return {"X": np.zeros((0, pf.N_FEATURES)), "y": np.zeros(0), "t": np.zeros(0)}
  T = np.array(cs_t)
  V = np.array(cs_v)
  OK = np.array(cs_ok, dtype=bool)
  bad_cum = np.concatenate([[0], np.cumsum(~OK)])
  X, y, ts = [], [], []
  for t, x, plan_v in snaps:
    t2 = t + pf.HORIZON_S
    i0 = int(np.searchsorted(T, t))
    i2 = int(np.searchsorted(T, t2))
    if i2 >= len(T) or T[i2] - t2 > 0.2 or i0 >= len(T):
      continue
    if bad_cum[i2 + 1] - bad_cum[i0] > 0:        # the driver did not hold the speed for the whole 2 s
      continue
    if np.any(np.diff(T[i0:i2 + 1]) > 0.5):     # a gap in the log
      continue
    v2 = float(np.interp(t2, T[i0:i2 + 1], V[i0:i2 + 1]))
    if x.v_ego < 0.5 and pf.plan_at(plan_v, pf.HORIZON_S) < 0.5:
      continue                                  # standing still, nothing to learn
    X.append(pf.features(x))
    y.append(pf.residual_target(v2, plan_v))
    ts.append(t - T[0])
  return {"X": np.array(X).reshape(-1, pf.N_FEATURES), "y": np.array(y), "t": np.array(ts)}


def _cached(args) -> tuple[str, int]:
  root, route, segs = args
  cache = os.path.join(root, "_personal_cache")
  os.makedirs(cache, exist_ok=True)
  p = os.path.join(cache, route + ".npz")
  key = json.dumps({"v": CACHE_VERSION, "features": pf.FEATURES, "segs": segs})
  try:
    with np.load(p, allow_pickle=False) as z:
      if str(z["key"]) == key:
        return route, int(len(z["y"]))
  except (OSError, KeyError, ValueError):
    pass
  d = build_route(root, route, segs)
  np.savez(p, X=d["X"], y=d["y"], t=d["t"], key=np.array(key))
  return route, len(d["y"])


def build(root: str, routes: list[str] | None = None, workers: int = 8) -> dict[str, int]:
  ds = drives(root)
  if routes:
    ds = {r: s for r, s in ds.items() if any(r.startswith(x) for x in routes)}
  with ProcessPoolExecutor(max_workers=workers) as ex:
    return dict(ex.map(_cached, [(root, r, s) for r, s in sorted(ds.items())]))


def load(root: str) -> dict[str, dict]:
  """{route: {"X", "y", "t"}} for every cached drive."""
  out = {}
  for p in sorted(glob.glob(os.path.join(root, "_personal_cache", "*.npz"))):
    with np.load(p, allow_pickle=False) as z:
      key = json.loads(str(z["key"]))
      if key.get("v") == CACHE_VERSION and tuple(key.get("features", ())) == pf.FEATURES and len(z["y"]):
        out[os.path.basename(p)[:-4]] = {"X": z["X"], "y": z["y"], "t": z["t"]}
  return out


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("routes", nargs="*")
  ap.add_argument("--root", default=os.path.expanduser("~/Fisker/routes"))
  ap.add_argument("--workers", type=int, default=8)
  a = ap.parse_args()
  n = build(a.root, a.routes, a.workers)
  print(f"{sum(n.values())} samples from {sum(1 for v in n.values() if v)} of {len(n)} drives")


if __name__ == "__main__":
  main()
