"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Junction memory: where this driver starts and finishes each junction turn they have driven, relative to the map's
junction node. The driver drives the same junctions again and again; the map's road lines and GPS put a turn's start
and end 2-4 m off, but the same error repeats at the same place, so the driver's own passes, measured with the same
map and GPS, give the turn assist the start and end of its path (turn_planner TurnTuning.path_*) at that junction.

File: <driver_profiles>/junctions/<profile>.json (a subfolder: the profile list reads every *.json beside the
profiles), {"version": 1, "junctions": {key: entry}}, built by
sunnypilot/tools/junction_passes.py (driver_profile_gen --junctions). Entry: medians over the passes of
  d_on   m before the node, along the arrival road, where the driver started bending (3% of the heading change)
  lat_on m right of the arrival road's line through the node, there
  d_out  m past the node, along the exit road, where the turn was finished (97%)
  lat_out m right of the exit road's line, there
  n      passes
"""
import json
import math
import os

from openpilot.sunnypilot.selfdrive.controls.lib import driver_profile as dp

VERSION = 1
MIN_PASSES = 2          # a junction is remembered after this many passes
SUBDIR = "junctions"
FIELDS = ("d_on", "lat_on", "d_out", "lat_out")
BOUNDS = {"d_on": (0.0, 40.0), "lat_on": (-8.0, 8.0), "d_out": (0.0, 40.0), "lat_out": (-8.0, 8.0)}


def key(arrival_way_id: int, branch_way_id: int, lat: float, lon: float) -> str:
  """The turn at the node (lat, lon) from one way onto another: the same junction from another direction, or onto
  another road, is another turn."""
  return f"{int(arrival_way_id)}>{int(branch_way_id)}@{lat:.5f},{lon:.5f}"


def path(profile: str = dp.DEFAULT_NAME) -> str:
  return os.path.join(os.path.dirname(dp.profile_path(profile)), SUBDIR, profile + ".json")


def build(passes: list[dict], min_passes: int = MIN_PASSES) -> dict:
  """{key: entry} from [{key, d_on, lat_on, d_out, lat_out}, ...]: the median of each value per key, kept when the
  key has at least min_passes."""
  by: dict[str, list[dict]] = {}
  for p in passes:
    by.setdefault(p["key"], []).append(p)
  out = {}
  for k, ps in by.items():
    if len(ps) < min_passes:
      continue
    e = {"n": len(ps)}
    for f in FIELDS:
      xs = sorted(float(p[f]) for p in ps)
      e[f] = round(xs[len(xs) // 2] if len(xs) % 2 else (xs[len(xs) // 2 - 1] + xs[len(xs) // 2]) / 2.0, 2)
    out[k] = e
  return out


def clean(entry) -> dict | None:
  """An entry with every value finite and inside BOUNDS, else None."""
  if not isinstance(entry, dict):
    return None
  try:
    e = {f: float(entry[f]) for f in FIELDS}
  except (KeyError, TypeError, ValueError):
    return None
  if not all(math.isfinite(v) and BOUNDS[f][0] <= v <= BOUNDS[f][1] for f, v in e.items()):
    return None
  if e["d_on"] <= 0.5 or e["d_out"] <= 0.5:
    return None
  e["n"] = int(entry.get("n", 0)) if isinstance(entry.get("n", 0), (int, float)) else 0
  return e


def load(profile: str = dp.DEFAULT_NAME) -> dict:
  """{key: entry}; {} when there is no file, it can't be read, or the profile name is bad."""
  try:
    with open(path(profile)) as f:   # a bad profile name raises ValueError too
      d = json.load(f)
  except (OSError, ValueError):
    return {}
  if not isinstance(d, dict) or d.get("version") != VERSION or not isinstance(d.get("junctions"), dict):
    return {}
  out = {}
  for k, v in d["junctions"].items():
    e = clean(v)
    if isinstance(k, str) and e is not None:
      out[k] = e
  return out


def save(junctions: dict, profile: str = dp.DEFAULT_NAME, meta: dict | None = None) -> str:
  p = path(profile)
  os.makedirs(os.path.dirname(p), exist_ok=True)
  tmp = p + ".tmp"
  with open(tmp, "w") as f:
    json.dump({"version": VERSION, "junctions": junctions, "meta": meta or {}}, f, indent=1, sort_keys=True)
  os.replace(tmp, p)
  return p
