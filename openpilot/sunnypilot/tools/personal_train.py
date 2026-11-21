#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Train and evaluate the personal model (run on the Mac, after personal_dataset.py).

Split by drive, in time order: the newest TEST_FRAC of drives are the test set (as the model would meet the next drive),
the VAL_FRAC before them stop the training early, the rest train. Reported against simpler answers, on the test set:
  none      no correction (the driving model as it is)
  constant  one number for everything (what a single profile value does)
  linear    ridge regression on the same features
  mlp       the personal model
MAE / RMSE in m/s of (driver speed at +2 s - model plan at 2 s), overall and per situation (lead, stop, curve, free).

  personal_train.py [--root ~/Fisker/routes] [--save] [--out FILE]
--save writes the model to <driver_profiles>/personal/me.json (or --out).
"""
import argparse
import json
import os
from datetime import datetime

import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib.personal import features as pf
from openpilot.sunnypilot.selfdrive.controls.lib.personal import model as pm
from openpilot.sunnypilot.tools import personal_dataset as pdset

TEST_FRAC = 0.2
VAL_FRAC = 0.15


def split(routes: list[str]) -> tuple[list[str], list[str], list[str]]:
  rs = sorted(routes, key=lambda r: int(r[:8], 16))
  n_test = max(1, round(len(rs) * TEST_FRAC))
  n_val = max(1, round(len(rs) * VAL_FRAC))
  return rs[:-(n_test + n_val)], rs[-(n_test + n_val):-n_test], rs[-n_test:]


def stack(data: dict, routes: list[str]):
  if not routes:
    return np.zeros((0, pf.N_FEATURES)), np.zeros(0)
  return (np.concatenate([data[r]["X"] for r in routes]), np.concatenate([data[r]["y"] for r in routes]))


def ridge(X, y, lam=1.0):
  mu, sd = X.mean(0), np.where(X.std(0) > 1e-6, X.std(0), 1.0)
  Z = np.hstack([(X - mu) / sd, np.ones((len(X), 1))])
  w = np.linalg.solve(Z.T @ Z + lam * np.eye(Z.shape[1]), Z.T @ y)
  return lambda Xn: np.clip(np.hstack([(Xn - mu) / sd, np.ones((len(Xn), 1))]) @ w, -pm.CLIP, pm.CLIP)


def metrics(pred, y):
  r = pred - y
  return {"mae": round(float(np.mean(np.abs(r))), 3), "rmse": round(float(np.sqrt(np.mean(r * r))), 3)}


def report(models: dict, X, y) -> dict:
  tags = np.array([pf.situation(f) for f in X])
  out = {}
  for name, fn in models.items():
    p = fn(X)
    row = {"all": metrics(p, y)}
    for tag in ("lead", "stop", "curve", "free"):
      m = tags == tag
      if m.sum() >= 50:
        row[tag] = metrics(p[m], y[m])
    out[name] = row
  out["_counts"] = {tag: int((tags == tag).sum()) for tag in ("lead", "stop", "curve", "free")}
  return out


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--root", default=os.path.expanduser("~/Fisker/routes"))
  ap.add_argument("--save", action="store_true")
  ap.add_argument("--out", default=None)
  ap.add_argument("--hidden", default="32,32")
  a = ap.parse_args()

  data = pdset.load(a.root)
  tr, va, te = split(list(data))
  Xtr, ytr = stack(data, tr)
  Xva, yva = stack(data, va)
  Xte, yte = stack(data, te)
  print(f"drives: train {len(tr)} ({len(ytr)} samples), val {len(va)} ({len(yva)}), test {len(te)} ({len(yte)})")
  print(f"target on train: mean {ytr.mean():+.3f} m/s, sd {ytr.std():.3f}; test: mean {yte.mean():+.3f}, sd {yte.std():.3f}")

  hidden = tuple(int(h) for h in a.hidden.split(",") if h)
  mlp = pm.fit(Xtr, ytr, Xva, yva, hidden=hidden,
               meta={"trained": datetime.now().isoformat(timespec="seconds"), "train_drives": tr, "val_drives": va,
                     "test_drives": te, "samples": int(len(ytr))})
  const = float(np.median(ytr))
  models = {"none": lambda X: np.zeros(len(X)), "constant": lambda X: np.full(len(X), const),
            "linear": ridge(Xtr, ytr), "mlp": mlp.predict}
  rep = report(models, Xte, yte)
  mlp.meta["test"] = rep
  print(json.dumps(rep, indent=1))
  if a.save:
    print("saved", pm.save(mlp, file=a.out))


if __name__ == "__main__":
  main()
