"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The driver profile: the driver's own habits as a few numbers the assists can drive by (sunnypilot/tools/
driver_profile_gen.py measures them from the driver's own drives and writes the file).

Every value has a default (what the assists use without a profile) and hard bounds: a profile can only move a value
inside them, whatever the file says, so it can never take an assist outside its safety envelope (the panda's limits,
the car's ACCEL_MAX and the planner's braking floor still apply on top). A missing, unreadable or malformed file, or
an unknown version, means the defaults; nothing here raises.

Profiles are named: <media>/driver_profiles/<name>.json (names: letters, digits, space, - and _, up to 32). The
DriverProfileName param says which one is active; the generator writes them and the web HUD bridge lists and switches
them (/api/profiles).

Used, with the DriverProfile param, by: Stop Sign Assist (braking rate and coast time), the narrow-street speed cap,
and drive_sim's "Your style" mode (always, to show what the profile would do).
"""
import json
import math
import os
import re
from dataclasses import asdict, dataclass, fields

PARAM = "DriverProfile"
NAME_PARAM = "DriverProfileName"
DEFAULT_NAME = "me"
VERSION = 1
NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9 _-]{0,31}$")


def profiles_dir() -> str:
  if os.environ.get("DRIVER_PROFILES_DIR"):
    return os.environ["DRIVER_PROFILES_DIR"]
  if os.path.isdir("/data/media/0"):
    return "/data/media/0/driver_profiles"
  return os.path.expanduser("~/.comma/media/0/driver_profiles")


def valid_name(name) -> bool:
  return isinstance(name, str) and bool(NAME_RE.match(name)) and not name.endswith(" ")


def profile_path(name: str) -> str:
  if not valid_name(name):
    raise ValueError(f"bad profile name: {name!r}")
  return os.path.join(profiles_dir(), name + ".json")


def list_profiles() -> list[dict]:
  """Every readable profile: {"name", "values" (clamped), "meta"}, sorted by name. Unreadable files are skipped."""
  out = []
  try:
    files = sorted(f for f in os.listdir(profiles_dir()) if f.endswith(".json"))
  except OSError:
    return out
  for f in files:
    name = f[:-5]
    if not valid_name(name):
      continue
    d = read_file(os.path.join(profiles_dir(), f))
    if d is not None:
      out.append({"name": name, "values": asdict(from_dict(d)), "meta": d.get("meta", {})})
  return out


def read_file(path: str) -> dict | None:
  try:
    with open(path) as f:
      d = json.load(f)
  except (OSError, ValueError):
    return None
  return d if isinstance(d, dict) and d.get("version") == VERSION else None


@dataclass
class DriverProfile:
  # speed: set speed = posted limit + offset; town = limits below fast_zone_mph
  town_offset_mph: float = 0.0
  fast_offset_mph: float = 0.0
  fast_zone_mph: float = 55.0
  # stops
  stop_decel: float = 1.0        # m/s^2, planned braking to a stop
  coast_s: float = 2.0           # s of lift-off coast before the braking point
  # pulling away (the car caps openpilot at ACCEL_MAX anyway)
  launch_accel: float = 1.0      # m/s^2
  # curves and turns
  curve_lat_accel: float = 2.0   # m/s^2
  junction_turn_mph: float = 10.0
  # following
  follow_gap_s: float = 1.45     # openpilot's "standard"
  # narrow streets
  narrow_speed_mph: float = 18.0


# (min, max) a profile may set; outside, the value is clamped
BOUNDS = {
  "town_offset_mph": (-10.0, 0.0),     # never over the limit in town
  "fast_offset_mph": (-10.0, 5.0),     # at most 5 over on fast roads
  "fast_zone_mph": (45.0, 65.0),
  "stop_decel": (0.7, 1.5),            # Stop Sign Assist keeps headroom under the planner's -1.2..-3.5 floor
  "coast_s": (0.0, 4.0),
  "launch_accel": (0.5, 1.0),          # the car's openpilot ACCEL_MAX
  "curve_lat_accel": (1.2, 2.5),
  "junction_turn_mph": (5.0, 16.0),    # the turn planner arms at 16 mph at most
  "follow_gap_s": (1.25, 2.5),
  "narrow_speed_mph": (10.0, 25.0),
}


def clamp(name: str, value) -> float:
  default = getattr(DriverProfile, name)
  try:
    v = float(value)
  except (TypeError, ValueError):
    return default
  if not math.isfinite(v):
    return default
  lo, hi = BOUNDS[name]
  return min(max(v, lo), hi)


def from_dict(d: dict) -> DriverProfile:
  values = d.get("values", {}) if isinstance(d, dict) else {}
  if not isinstance(values, dict):
    values = {}
  return DriverProfile(**{f.name: clamp(f.name, values[f.name]) for f in fields(DriverProfile) if f.name in values})


def load(name: str = DEFAULT_NAME) -> DriverProfile:
  """The named profile, clamped to BOUNDS; the defaults when there is none, the name is bad or it can't be read."""
  try:
    d = read_file(profile_path(name))
  except ValueError:
    return DriverProfile()
  return from_dict(d) if d is not None else DriverProfile()


def active_name(params) -> str:
  try:
    name = params.get(NAME_PARAM, return_default=True)
  except Exception:
    return DEFAULT_NAME
  if isinstance(name, bytes):
    name = name.decode(errors="replace")
  return name if valid_name(name) else DEFAULT_NAME


def active(params=None) -> DriverProfile:
  """The active profile if the DriverProfile param is set, else the defaults. Never raises (e.g. code newer than the
  built params): then the defaults."""
  try:
    if params is None:
      from openpilot.common.params import Params
      params = Params()
    return load(active_name(params)) if params.get_bool(PARAM) else DriverProfile()
  except Exception:
    return DriverProfile()


def to_dict(p: DriverProfile, meta: dict | None = None) -> dict:
  return {"version": VERSION, "values": asdict(p), "meta": meta or {}}
