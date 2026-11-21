"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

A curve head: one of the driver's habits as a function of the one thing it depends on (the following gap by speed, the
braking to a stop by the speed it starts from, the slowest speed through a junction turn by its angle, ...).

Fit: split x into bins of at least min_bin events each (at most max_bins, by quantile), take the statistic of y per
bin (the median, or a quantile such as p75 for the lateral acceleration the driver allows in curves), and interpolate
between the bins' centres (flat beyond the outermost). With too few events for two bins it is a constant: the
statistic over all events, which is exactly what the JSON profile stored. Predictions are clamped to the profile's
bounds by the caller (personal/profile.py).

These heads learn from dozens to a few thousand events per habit, where an MLP would only overfit; the speed
residual (personal/model.py) has 150k samples and keeps its MLP.
"""
import numpy as np


class CurveHead:
  def __init__(self, xs=(), ys=(), const=0.0, n=0, x_name="", y_name="", stat="median"):
    self.xs = np.asarray(xs, dtype=float)
    self.ys = np.asarray(ys, dtype=float)
    self.const = float(const)
    self.n = int(n)
    self.x_name = x_name
    self.y_name = y_name
    self.stat = stat

  @property
  def is_curve(self) -> bool:
    return len(self.xs) >= 2

  def predict(self, x: float | None = None) -> float:
    if x is None or not self.is_curve or not np.isfinite(x):
      return self.const
    return float(np.interp(x, self.xs, self.ys))

  def to_dict(self) -> dict:
    return {"xs": self.xs.round(4).tolist(), "ys": self.ys.round(4).tolist(), "const": round(self.const, 4), "n": self.n,
            "x": self.x_name, "y": self.y_name, "stat": self.stat}

  @classmethod
  def from_dict(cls, d: dict) -> "CurveHead":
    h = cls(d.get("xs", ()), d.get("ys", ()), d["const"], d.get("n", 0), d.get("x", ""), d.get("y", ""),
            d.get("stat", "median"))
    if h.xs.shape != h.ys.shape or not (np.all(np.isfinite(h.xs)) and np.all(np.isfinite(h.ys)) and np.isfinite(h.const)):
      raise ValueError("curve head: bad points")
    if h.is_curve and np.any(np.diff(h.xs) <= 0):
      raise ValueError("curve head: x not increasing")
    return h


def statistic(y: np.ndarray, stat: str) -> float:
  if stat == "median":
    return float(np.median(y))
  if stat.startswith("p"):
    return float(np.percentile(y, float(stat[1:])))
  raise ValueError(stat)


def loss(pred: np.ndarray, y: np.ndarray, stat: str) -> float:
  """The loss the statistic minimises: absolute error for the median, pinball for a quantile."""
  r = y - pred
  if stat == "median":
    return float(np.mean(np.abs(r)))
  q = float(stat[1:]) / 100.0
  return float(np.mean(np.maximum(q * r, (q - 1.0) * r)))


def fit(x: np.ndarray, y: np.ndarray, stat: str = "median", min_bin: int = 20, max_bins: int = 6, x_name: str = "",
        y_name: str = "") -> CurveHead:
  x = np.asarray(x, dtype=float)
  y = np.asarray(y, dtype=float)
  ok = np.isfinite(x) & np.isfinite(y)
  x, y = x[ok], y[ok]
  if len(y) == 0:
    return CurveHead(const=float("nan"), n=0, x_name=x_name, y_name=y_name, stat=stat)
  const = statistic(y, stat)
  k = min(max_bins, len(y) // max(min_bin, 1))
  if k < 2 or len(np.unique(x)) < 2:
    return CurveHead(const=const, n=len(y), x_name=x_name, y_name=y_name, stat=stat)
  edges = np.unique(np.quantile(x, np.linspace(0.0, 1.0, k + 1)))
  xs, ys = [], []
  for lo, hi in zip(edges[:-1], edges[1:], strict=True):
    m = (x >= lo) & ((x < hi) if hi < edges[-1] else (x <= hi))
    if m.sum() >= min_bin:
      xs.append(float(np.median(x[m])))
      ys.append(statistic(y[m], stat))
  if len(xs) < 2:
    return CurveHead(const=const, n=len(y), x_name=x_name, y_name=y_name, stat=stat)
  order = np.argsort(xs)
  xs_a, ys_a = np.array(xs)[order], np.array(ys)[order]
  keep = np.concatenate([[True], np.diff(xs_a) > 1e-6])
  return CurveHead(xs_a[keep], ys_a[keep], const, len(y), x_name, y_name, stat)


def fit_discrete(x: np.ndarray, y: np.ndarray, stat: str = "median", min_per_key: int = 20, x_name: str = "",
                 y_name: str = "") -> CurveHead:
  """For an input that only takes a few values (the posted limit): one point per value with enough events."""
  x = np.asarray(x, dtype=float)
  y = np.asarray(y, dtype=float)
  ok = np.isfinite(x) & np.isfinite(y)
  x, y = x[ok], y[ok]
  if len(y) == 0:
    return CurveHead(const=float("nan"), n=0, x_name=x_name, y_name=y_name, stat=stat)
  const = statistic(y, stat)
  xs, ys = [], []
  for v in np.unique(x):
    m = x == v
    if m.sum() >= min_per_key:
      xs.append(float(v))
      ys.append(statistic(y[m], stat))
  if len(xs) < 2:
    return CurveHead(const=const, n=len(y), x_name=x_name, y_name=y_name, stat=stat)
  return CurveHead(xs, ys, const, len(y), x_name, y_name, stat)
