"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Inputs of the personal model. One function computes them from plain numbers, so the offline dataset (from rlogs) and
the car (from live messages) build exactly the same vector.

Speed target (v1): residual_v2 = the driver's speed 2 s later - the model's planned speed at 2 s.
"""
from dataclasses import dataclass

import numpy as np

from openpilot.selfdrive.modeld.constants import ModelConstants

T_IDXS = np.array(ModelConstants.T_IDXS)
HORIZON_S = 2.0               # the speed target's horizon
GAP_NONE_S = 5.0              # time gap used when there is no lead (or it is further than this)
LIMIT_CLIP = 15.0             # m/s, speed limit minus speed is clipped to +-this
AHEAD_DIST_CLIP = 500.0       # m

# No a_ego: the driver's speed 2 s later is mostly v + 2 s of their current acceleration, so a_ego would just
# extrapolate their foot (MAE 0.88 -> 0.44 m/s, all of it that), and on the car it is openpilot's own acceleration, which
# the correction would then feed back into itself. Likewise no planned acceleration (modelV2.acceleration): it is the
# model's reading of the car's current motion (ablation 2026-11-20: MAE 0.514 with, 0.546 without, so most of the gain
# is the plan shape, not the current motion).
FEATURES = (
  "v_ego",
  "plan_v1", "plan_v2", "plan_v4", "plan_v6",       # the model's planned speed at 1/2/4/6 s, minus v_ego
  "plan_vmin6",                                     # its slowest planned speed in 6 s, minus v_ego (a stop ahead)
  "plan_curv4",                                     # its sharpest planned curvature in 4 s (1/m, abs)
  "lead", "lead_gap_s", "lead_vrel",                # a lead car: present, time gap (s), closing speed (m/s)
  "limit_valid", "limit_minus_v",                   # the map speed limit minus v_ego
  "ahead_valid", "ahead_minus_v", "ahead_dist",     # the next limit ahead, minus v_ego, and its distance (m / 100)
)
N_FEATURES = len(FEATURES)


@dataclass
class Inputs:
  v_ego: float
  a_ego: float
  plan_v: list          # modelV2.velocity.x (33 values at T_IDXS)
  plan_a: list          # modelV2.acceleration.x
  plan_yaw_rate: list   # modelV2.orientationRate.z
  lead_present: bool = False
  lead_d: float = 0.0
  lead_v_rel: float = 0.0
  limit_valid: bool = False
  limit: float = 0.0          # m/s
  ahead_valid: bool = False
  ahead_limit: float = 0.0    # m/s
  ahead_dist: float = 0.0     # m


def plan_at(plan: list, t: float) -> float:
  n = min(len(plan), len(T_IDXS))
  return float(np.interp(t, T_IDXS[:n], np.asarray(plan[:n], dtype=float))) if n else 0.0


def features(x: Inputs) -> np.ndarray:
  v = float(x.v_ego)
  pv = np.asarray(x.plan_v, dtype=float)
  n = min(len(pv), len(T_IDXS))
  if n == 0:
    pv = np.array([v])
    n = 1
  vmin6 = float(np.min(pv[:n][T_IDXS[:n] <= 6.0])) if n > 1 else float(pv[0])
  yaw = np.asarray(x.plan_yaw_rate, dtype=float)[:n]
  vel = np.maximum(pv[:len(yaw)], 1.0)
  curv = float(np.max(np.abs(yaw / vel)[T_IDXS[:len(yaw)] <= 4.0])) if len(yaw) else 0.0
  gap = GAP_NONE_S
  vrel = 0.0
  if x.lead_present and x.lead_d > 0.0:
    gap = min(x.lead_d / max(v, 1.0), GAP_NONE_S)
    vrel = float(np.clip(-x.lead_v_rel, -10.0, 10.0))   # radar vRel < 0 = closing; positive here = closing
  lim = float(np.clip(x.limit - v, -LIMIT_CLIP, LIMIT_CLIP)) if x.limit_valid and x.limit > 0 else 0.0
  ahd = float(np.clip(x.ahead_limit - v, -LIMIT_CLIP, LIMIT_CLIP)) if x.ahead_valid and x.ahead_limit > 0 else 0.0
  ahd_dist = float(np.clip(x.ahead_dist, 0.0, AHEAD_DIST_CLIP)) / 100.0 if x.ahead_valid else AHEAD_DIST_CLIP / 100.0
  out = np.array([
    v,
    plan_at(x.plan_v, 1.0) - v, plan_at(x.plan_v, 2.0) - v, plan_at(x.plan_v, 4.0) - v, plan_at(x.plan_v, 6.0) - v,
    vmin6 - v,
    min(curv, 0.2),
    1.0 if gap < GAP_NONE_S else 0.0, gap, vrel,
    1.0 if x.limit_valid and x.limit > 0 else 0.0, lim,
    1.0 if x.ahead_valid and x.ahead_limit > 0 else 0.0, ahd, ahd_dist,
  ], dtype=float)
  return np.where(np.isfinite(out), out, 0.0)


def situation(f: np.ndarray) -> str:
  """A coarse tag for reporting: following a car, a stop ahead, a curve, or free road."""
  i = FEATURES.index
  if f[i("lead")] > 0.5 and f[i("lead_gap_s")] < 4.0:
    return "lead"
  if f[i("plan_vmin6")] + f[i("v_ego")] < 3.0 and f[i("v_ego")] > 3.0:
    return "stop"
  if f[i("plan_curv4")] > 0.01:
    return "curve"
  return "free"


def residual_target(v_future: float, plan_v: list) -> float:
  return float(v_future) - plan_at(plan_v, HORIZON_S)

