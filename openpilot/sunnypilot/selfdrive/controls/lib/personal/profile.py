"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The personal model as the driver profile: every habit the JSON profile (driver_profile.py) holds as one number is a
head here, a function of the situation it depends on; at(...) gives the DriverProfile for the situation the car is in,
so every assist that reads the profile reads the personal model instead, unchanged.

File: <driver_profiles>/personal/<profile>.json
  {"version": 2, "heads": {name: CurveHead dict + {"use": bool, "eval": {...}}}, "speed_residual": MLP dict or null,
   "meta": {...}}
Built on the Mac by sunnypilot/tools/personal_heads.py (heads) and personal_train.py (speed_residual).

Safety: every value is clamped to driver_profile.BOUNDS, exactly as the JSON profile's; a head without enough of the
driver's own events ("use": false) gives the DriverProfile default; a file that can't be read gives None and the
caller keeps the JSON profile.
"""
import json
import math
import os
from dataclasses import dataclass

from openpilot.sunnypilot.selfdrive.controls.lib import driver_profile as dp
from openpilot.sunnypilot.selfdrive.controls.lib.personal.heads import CurveHead

VERSION = 2
SUBDIR = "personal"
REF_TURN_DEG = 90.0     # the turn angle used when the caller gives none (a right-angle junction)


@dataclass(frozen=True)
class HeadSpec:
  field: str             # the DriverProfile field it gives
  x: str | None          # the context it depends on (None: a constant)
  stat: str = "median"
  min_n: int = 5         # fewer of the driver's events: the default (the JSON generator's MIN_SAMPLES)
  min_bin: int = 20      # events per curve point
  discrete: bool = False
  note: str = ""


# context keys: v (m/s, current or approach speed), closing (m/s, toward the lead), turn_deg (abs heading change of a
# junction turn), limit_mph (posted limit)
HEADS: dict[str, HeadSpec] = {
  "town_offset_mph": HeadSpec("town_offset_mph", None, min_n=300, note="speed - limit, open road below 55 mph"),
  "fast_offset_mph": HeadSpec("fast_offset_mph", None, min_n=300, note="speed - limit, open road 55 mph and up"),
  "limit_offsets_mph": HeadSpec("limit_offsets_mph", "limit_mph", min_n=300, min_bin=1200, discrete=True,
                                note="speed - limit per posted limit (>= 120 s of open road each)"),
  "follow_gap_s": HeadSpec("follow_gap_s", "v", min_n=200, min_bin=1500, note="time gap behind a steady lead, by speed"),
  "lead_coast_s": HeadSpec("lead_coast_s", "closing", min_n=8, min_bin=60,
                           note="time gap at lifting off toward a slower lead, by closing speed"),
  "lead_brake_a": HeadSpec("lead_brake_a", "closing", min_n=10, min_bin=15,
                           note="braking need when starting to slow toward a car closing fast, by closing speed"),
  "stop_decel": HeadSpec("stop_decel", "v", min_n=5, min_bin=15, note="braking to a full stop, by the speed it starts from"),
  "coast_s": HeadSpec("coast_s", "v", min_n=5, min_bin=15, note="accelerator release to braking before a stop, by speed"),
  "launch_accel": HeadSpec("launch_accel", None, note="pulling away below 15 mph"),
  "curve_lat_accel": HeadSpec("curve_lat_accel", "v", stat="p75", min_n=100, min_bin=400,
                              note="p75 lateral accel in curves the driver steered, by speed"),
  "junction_turn_mph": HeadSpec("junction_turn_mph", "turn_deg", min_bin=25,
                                note="slowest speed through a junction turn the driver steered, by its angle"),
  "turn_exit_accel": HeadSpec("turn_exit_accel", "turn_deg", min_bin=25, note="pull-away after a junction turn, by its angle"),
  "turn_unwind_deg_s": HeadSpec("turn_unwind_deg_s", "turn_deg", min_bin=15,
                                note="the wheel's return at the end of a junction turn, by its angle"),
  "turn_stop_start_left_m": HeadSpec("turn_stop_start_left_m", None, min_n=3, note="left turns from a stop: where the wheel passes 30 deg"),
  "turn_stop_start_right_m": HeadSpec("turn_stop_start_right_m", None, min_n=3, note="right turns from a stop"),
  "narrow_speed_mph": HeadSpec("narrow_speed_mph", None, min_n=200, note="speed on streets the narrow-street detector flags"),
}


class PersonalProfile:
  def __init__(self, heads: dict[str, CurveHead], use: dict[str, bool], speed_residual=None, meta=None):
    self.heads = heads
    self.use = use
    self.speed_residual = speed_residual
    self.meta = meta or {}

  def value(self, name: str, ctx: dict) -> float | None:
    """The head's value (clamped) for this situation, or None when the head isn't used."""
    if not self.use.get(name) or name not in self.heads:
      return None
    spec = HEADS[name]
    x = ctx.get(spec.x) if spec.x else None
    if spec.x == "turn_deg" and x is None:
      x = REF_TURN_DEG
    v = self.heads[name].predict(x)
    return dp.clamp(spec.field, v) if math.isfinite(v) else None

  def at(self, v: float | None = None, closing: float | None = None, turn_deg: float | None = None) -> dp.DriverProfile:
    """The driver profile for this situation (the speed, the closing speed to a lead, a junction turn's angle); with
    no context, each head's value over all of the driver's events (what the JSON profile held)."""
    ctx = {"v": v, "closing": closing, "turn_deg": turn_deg}
    p = dp.DriverProfile()
    for name, spec in HEADS.items():
      if name == "limit_offsets_mph":
        continue
      val = self.value(name, ctx)
      if val is not None:
        setattr(p, spec.field, val)
    if self.use.get("limit_offsets_mph") and "limit_offsets_mph" in self.heads:
      h = self.heads["limit_offsets_mph"]
      if h.is_curve:
        p.limit_offsets_mph = dp.clamp_limit_offsets({str(int(round(x))): y for x, y in zip(h.xs, h.ys, strict=True)},
                                                     p.fast_zone_mph)
    return p

  def to_dict(self) -> dict:
    heads = {}
    for name, h in self.heads.items():
      d = h.to_dict()
      d["use"] = bool(self.use.get(name, False))
      heads[name] = d
    return {"version": VERSION, "heads": heads,
            "speed_residual": self.speed_residual.to_dict() if self.speed_residual is not None else None,
            "meta": self.meta}

  @classmethod
  def from_dict(cls, d: dict) -> "PersonalProfile":
    if not isinstance(d, dict) or d.get("version") != VERSION or not isinstance(d.get("heads"), dict):
      raise ValueError("personal profile: wrong version")
    heads, use = {}, {}
    for name, hd in d["heads"].items():
      if name in HEADS and isinstance(hd, dict):
        heads[name] = CurveHead.from_dict(hd)
        use[name] = bool(hd.get("use", False))
    sr = None
    if d.get("speed_residual"):
      from openpilot.sunnypilot.selfdrive.controls.lib.personal.model import PersonalModel
      sr = PersonalModel.from_dict(d["speed_residual"])
    return cls(heads, use, sr, d.get("meta"))


def path(profile: str = dp.DEFAULT_NAME) -> str:
  return os.path.join(os.path.dirname(dp.profile_path(profile)), SUBDIR, profile + ".json")


def read_raw(profile: str = dp.DEFAULT_NAME) -> dict | None:
  try:
    with open(path(profile)) as f:
      d = json.load(f)
    return d if isinstance(d, dict) else None
  except (OSError, ValueError):
    return None


def load(profile: str = dp.DEFAULT_NAME) -> PersonalProfile | None:
  """The personal profile, or None when there is no file, it can't be read, or it is the old (v1, speed only) format."""
  try:
    d = read_raw(profile)
    return PersonalProfile.from_dict(d) if d is not None else None
  except (ValueError, KeyError, TypeError):
    return None


def write_raw(d: dict, profile: str = dp.DEFAULT_NAME, file: str | None = None) -> str:
  p = file or path(profile)
  os.makedirs(os.path.dirname(p), exist_ok=True)
  with open(p + ".tmp", "w") as f:
    json.dump(d, f, indent=1)
  os.replace(p + ".tmp", p)
  return p


def save(pp: PersonalProfile, profile: str = dp.DEFAULT_NAME, file: str | None = None) -> str:
  return write_raw(pp.to_dict(), profile, file)
