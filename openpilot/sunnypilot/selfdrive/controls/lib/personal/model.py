"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The personal model: standardise the features, a small MLP (tanh), clamp the output. numpy only, a few thousand
parameters, so it trains on the Mac in seconds and runs on the car for nothing.

File: <driver_profiles>/personal/<profile>.json (a subfolder, like junctions/: the profile list reads every *.json
beside the profiles), {"version", "target", "features", "mean", "std", "layers": [[W, b], ...], "clip", "meta"}.
"""
import json
import math
import os

import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib import driver_profile as dp
from openpilot.sunnypilot.selfdrive.controls.lib.personal.features import FEATURES

VERSION = 1
SUBDIR = "personal"
CLIP = 3.0     # m/s: the correction never exceeds this, whatever the weights say


class PersonalModel:
  def __init__(self, layers, mean, std, clip=CLIP, target="residual_v2", features=FEATURES, meta=None):
    self.layers = [(np.asarray(w, dtype=float), np.asarray(b, dtype=float)) for w, b in layers]
    self.mean = np.asarray(mean, dtype=float)
    self.std = np.asarray(std, dtype=float)
    self.clip = float(clip)
    self.target = target
    self.features = tuple(features)
    self.meta = meta or {}

  def raw(self, X: np.ndarray) -> np.ndarray:
    h = (np.atleast_2d(X) - self.mean) / self.std
    for k, (w, b) in enumerate(self.layers):
      h = h @ w + b
      if k < len(self.layers) - 1:
        h = np.tanh(h)
    return h[:, 0]

  def predict(self, X: np.ndarray) -> np.ndarray:
    return np.clip(self.raw(X), -self.clip, self.clip)

  def to_dict(self) -> dict:
    return {"version": VERSION, "target": self.target, "features": list(self.features), "mean": self.mean.tolist(),
            "std": self.std.tolist(), "layers": [[w.tolist(), b.tolist()] for w, b in self.layers], "clip": self.clip,
            "meta": self.meta}

  @classmethod
  def from_dict(cls, d: dict) -> "PersonalModel":
    if d.get("version") != VERSION or tuple(d.get("features", ())) != FEATURES:
      raise ValueError("personal model: wrong version or features")
    m = cls(d["layers"], d["mean"], d["std"], d.get("clip", CLIP), d.get("target", "residual_v2"), d["features"],
            d.get("meta"))
    if m.mean.shape != (len(FEATURES),) or m.std.shape != (len(FEATURES),) or not np.all(m.std > 0):
      raise ValueError("personal model: bad standardisation")
    if not all(np.all(np.isfinite(w)) and np.all(np.isfinite(b)) for w, b in m.layers):
      raise ValueError("personal model: non-finite weights")
    return m


def path(profile: str = dp.DEFAULT_NAME) -> str:
  return os.path.join(os.path.dirname(dp.profile_path(profile)), SUBDIR, profile + ".json")


def load(profile: str = dp.DEFAULT_NAME) -> PersonalModel | None:
  """The speed-residual model from the personal file (v2: its "speed_residual"; v1: the whole file), or None when there
  is none, it can't be read, or it doesn't match these features."""
  try:
    with open(path(profile)) as f:
      d = json.load(f)
    if isinstance(d, dict) and d.get("version") == 2:
      d = d.get("speed_residual")
    return PersonalModel.from_dict(d)
  except (OSError, ValueError, KeyError, TypeError, AttributeError):
    return None


def save(model: PersonalModel, profile: str = dp.DEFAULT_NAME, file: str | None = None) -> str:
  """Into the personal file's "speed_residual", keeping its heads (a v1 or missing file becomes v2 with no heads)."""
  p = file or path(profile)
  try:
    with open(p) as f:
      d = json.load(f)
  except (OSError, ValueError):
    d = None
  if not isinstance(d, dict) or d.get("version") != 2:
    d = {"version": 2, "heads": {}, "meta": {}}
  d["speed_residual"] = model.to_dict()
  os.makedirs(os.path.dirname(p), exist_ok=True)
  with open(p + ".tmp", "w") as f:
    json.dump(d, f)
  os.replace(p + ".tmp", p)
  return p


# ---- training (Mac) ----

def _huber_grad(r: np.ndarray, delta: float) -> np.ndarray:
  return np.where(np.abs(r) <= delta, r, delta * np.sign(r))


def fit(X: np.ndarray, y: np.ndarray, Xv: np.ndarray | None = None, yv: np.ndarray | None = None, hidden=(32, 32),
        lr=3e-3, epochs=200, batch=512, l2=1e-4, delta=1.0, patience=20, seed=0, clip=CLIP, meta=None) -> PersonalModel:
  """Adam on a Huber loss, early-stopped on the validation set (Xv, yv) when given. The output layer starts at zero, so
  an untrained model predicts no correction."""
  rng = np.random.default_rng(seed)
  mean = X.mean(axis=0)
  std = X.std(axis=0)
  std = np.where(std > 1e-6, std, 1.0)
  sizes = [X.shape[1], *hidden, 1]
  layers = []
  for k in range(len(sizes) - 1):
    scale = 0.0 if k == len(sizes) - 2 else 1.0 / math.sqrt(sizes[k])
    layers.append([rng.normal(0.0, scale, (sizes[k], sizes[k + 1])), np.zeros(sizes[k + 1])])
  m1 = [[np.zeros_like(w), np.zeros_like(b)] for w, b in layers]
  m2 = [[np.zeros_like(w), np.zeros_like(b)] for w, b in layers]
  Xs = (X - mean) / std
  best, best_layers, bad, step = math.inf, None, 0, 0

  def val_loss(ls):
    if Xv is None or yv is None:
      return None
    pm = PersonalModel(ls, mean, std, clip)
    r = pm.predict(Xv) - yv
    return float(np.mean(np.where(np.abs(r) <= delta, 0.5 * r * r, delta * (np.abs(r) - 0.5 * delta))))

  for _ in range(epochs):
    order = rng.permutation(len(Xs))
    for s in range(0, len(order), batch):
      idx = order[s:s + batch]
      acts = [Xs[idx]]
      for k, (w, b) in enumerate(layers):
        z = acts[-1] @ w + b
        acts.append(np.tanh(z) if k < len(layers) - 1 else z)
      g = _huber_grad(acts[-1][:, 0] - y[idx], delta)[:, None] / len(idx)
      step += 1
      for k in range(len(layers) - 1, -1, -1):
        w, b = layers[k]
        gw = acts[k].T @ g + l2 * w
        gb = g.sum(axis=0)
        if k > 0:
          g = (g @ w.T) * (1.0 - acts[k] ** 2)
        for p_i, (p, gp) in enumerate(((w, gw), (b, gb))):
          m1[k][p_i] = 0.9 * m1[k][p_i] + 0.1 * gp
          m2[k][p_i] = 0.999 * m2[k][p_i] + 0.001 * gp * gp
          mh = m1[k][p_i] / (1 - 0.9 ** step)
          vh = m2[k][p_i] / (1 - 0.999 ** step)
          p -= lr * mh / (np.sqrt(vh) + 1e-8)
    vl = val_loss(layers)
    if vl is None:
      continue
    if vl < best - 1e-5:
      best, best_layers, bad = vl, [[w.copy(), b.copy()] for w, b in layers], 0
    else:
      bad += 1
      if bad >= patience:
        break
  return PersonalModel(best_layers or layers, mean, std, clip, meta=meta)
