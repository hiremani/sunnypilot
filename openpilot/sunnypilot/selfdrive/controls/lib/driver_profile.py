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

Used, with the DriverProfile param, by the longitudinal planner: Stop Sign Assist (braking rate and coast time), the
narrow-street speed cap, the following gap (follow_gap_s), no accelerating toward a slower lead (lead_coast_s), the
junction-turn speed cap and Smart Cruise Control's curve speed (curve_lat_accel); with DriverProfileSpeed as well, the
cruising speed cap at the posted limit + the driver's offset for it (speed_offset_mph). drive_sim's "Your style" mode
reads it always, to show what the profile would do.
"""
import json
import math
import os
import re
from dataclasses import asdict, dataclass, field, fields

PARAM = "DriverProfile"
NAME_PARAM = "DriverProfileName"
SPEED_PARAM = "DriverProfileSpeed"
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
  turn_exit_accel: float = 1.0   # m/s^2, pulling away out of a junction turn
  # from a stop the turn assist starts a signaled turn as the car moves off when the junction is this close (m); 0 = off
  turn_stop_start_left_m: float = 0.0
  turn_stop_start_right_m: float = 0.0
  # how fast the driver's wheel comes back at the end of a junction turn (deg/s); the turn assist starts straightening
  # early enough for it. 0 = off (the car's own rate limit)
  turn_unwind_deg_s: float = 0.0
  # the driver's own path shape between the lane they are in and the target lane (turn_planner doorway_path), used at
  # junctions in the junction memory (junction_memory.py); 0 = off
  turn_path_in: float = 0.0
  turn_path_out: float = 0.0
  # following
  follow_gap_s: float = 1.45     # openpilot's "standard"
  # narrow streets
  narrow_speed_mph: float = 18.0
  # slower lead ahead: no accelerating toward it inside this time gap (s); 0 = off
  lead_coast_s: float = 0.0
  # closing fast on a slower car: start slowing once an even slowdown to the following gap behind it needs this much
  # braking (m/s^2), as the driver does (profile_long.lead_brake_need); 0 = off (only the MPC's own lead braking)
  lead_brake_a: float = 0.0
  # stopping behind a car: this far back (m); 0 = off (the MPC's 6 m). Never closer than profile_long.STOP_GAP_MIN.
  stop_gap_m: float = 0.0
  # speeding up on open road, no car close ahead (m/s^2, median): 15-30 mph, 30-45 mph, 45+ mph; 0 = off (the comfort
  # table, 0.6 in town rising to 1.0)
  cruise_accel_town: float = 0.0
  cruise_accel_mid: float = 0.0
  cruise_accel_fast: float = 0.0
  # merging: well below a fast road's limit (onto a freeway), the driver's pull (m/s^2, p75 of their merges); 0 = the
  # comfort table's (1.0 at speed). Never the open-road cruise_accel_*: that would merge too gently.
  merge_accel: float = 0.0
  # blinker on -> the driver starts the lane change (s); 0 = off (the AutoLaneChangeTimer setting)
  lane_change_delay_s: float = 0.0
  # mapped speed bumps: the driver's crossing speed for a plain bump (mph; the other kinds scale with it); 0 = off
  bump_cross_mph: float = 0.0
  # junction turns: how fast the driver winds the wheel in (deg/s); 0 = off (turn_planner TURN_WHEEL_RATE_DEG_S)
  turn_in_deg_s: float = 0.0
  # junction turns while moving: the start lead (s) that puts the turn planner's 30 deg wheel point where the driver's is
  turn_start_lead_s: float = -0.45
  # junction turns: the arc radius per side (m), fitted to the driver's own turns by the ghost-path check
  # (tools/turn_tuning_fit.py); the defaults are the turn planner's
  turn_radius_left_m: float = 10.0
  turn_radius_right_m: float = 10.0
  # junction turns: the speed the driver still starts a turn at (p90, mph); the turn planner starts no faster
  turn_arm_mph: float = 16.1
  # curves ahead: the most the curve-speed controller slows by while entering one (m/s^2, the driver's p75)
  curve_entry_decel: float = 1.0
  # mapped traffic lights: stop accelerating this many seconds before one (the driver lifts off at reds)
  signal_coast_s: float = 4.0
  # wet road (wipers running): cruise this far over the posted limit (mph) and follow this much further back
  wet_offset_mph: float = 0.0
  wet_gap_factor: float = 1.25
  # per posted limit (mph, multiple of 5, as a string key): the driver's own offset (mph); where a limit has no entry,
  # town_offset_mph / fast_offset_mph. Clamped like those.
  limit_offsets_mph: dict = field(default_factory=dict)

  def speed_offset_mph(self, limit_mph: float) -> float:
    key = str(int(round(limit_mph / 5.0) * 5))
    if key in self.limit_offsets_mph:
      return self.limit_offsets_mph[key]
    return self.town_offset_mph if limit_mph < self.fast_zone_mph else self.fast_offset_mph


# (min, max) a profile may set; outside, the value is clamped
BOUNDS = {
  "town_offset_mph": (-10.0, 3.0),     # town: at most 3 over (the driver's choice, 2026-11-21: settled cruise is +2.4 on
                                       # 35 mph roads, +3.4 on 40; it was capped at the limit)
  "fast_offset_mph": (-10.0, 5.0),     # at most 5 over on fast roads
  "fast_zone_mph": (45.0, 65.0),
  "stop_decel": (0.7, 1.5),            # Stop Sign Assist keeps headroom under the planner's -1.2..-3.5 floor
  "coast_s": (0.0, 4.0),
  "launch_accel": (0.5, 1.0),          # the car's openpilot ACCEL_MAX
  "curve_lat_accel": (1.2, 2.5),
  "junction_turn_mph": (5.0, 16.0),    # the turn planner arms at 16 mph at most
  "turn_exit_accel": (0.3, 1.0),       # up to the car's openpilot ACCEL_MAX
  "turn_stop_start_left_m": (0.0, 30.0),   # turn_planner.R_FROM_STOP_MAX: no wider arc than that
  "turn_stop_start_right_m": (0.0, 30.0),
  "turn_path_in": (0.0, 0.95),
  "turn_path_out": (0.0, 0.95),
  "turn_unwind_deg_s": (0.0, 120.0),   # fisker values.SMOOTH_UNWIND_RATE: the controller returns the wheel no faster
  "follow_gap_s": (1.25, 2.5),
  "narrow_speed_mph": (10.0, 25.0),
  "lead_coast_s": (0.0, 5.0),
  "lead_brake_a": (0.0, 1.2),
  "stop_gap_m": (0.0, 6.0),            # 0 = off; any other value is at least profile_long.STOP_GAP_MIN
  "cruise_accel_town": (0.0, 1.0),     # up to the car's openpilot ACCEL_MAX
  "cruise_accel_mid": (0.0, 1.0),
  "cruise_accel_fast": (0.0, 1.0),
  "merge_accel": (0.0, 1.0),
  "lane_change_delay_s": (0.0, 3.0),
  "bump_cross_mph": (0.0, 15.0),       # never faster over a plain bump than 15 mph
  "turn_in_deg_s": (0.0, 120.0),       # never faster than the turn planner's own 120 deg/s
  "turn_start_lead_s": (-1.5, 1.5),
  "turn_radius_left_m": (7.0, 15.0),
  "turn_radius_right_m": (7.0, 15.0),
  "turn_arm_mph": (12.0, 16.1),        # lower only: above the planner's own 16 mph it starts turns the driver is still
                                       # braking for, from where the car is then (00000128 turn 3: 21.6 m early at 16.2
                                       # mph, a 21 m arc 27 m off the driver's path; the p90 measured was 16.7)
  "curve_entry_decel": (0.5, 1.5),
  "signal_coast_s": (2.0, 8.0),
  "wet_offset_mph": (-10.0, 0.0),      # never over the limit when wet
  "wet_gap_factor": (1.0, 1.75),          # never later than needing 1.2 m/s^2 (the cap brakes at most LEAD_BRAKE_A_MAX)
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


def clamp_limit_offsets(offsets, fast_zone_mph: float) -> dict:
  """{"25": -5.3, ...} with bad keys / values dropped, each offset clamped to the town or fast-road bounds."""
  out = {}
  if not isinstance(offsets, dict):
    return out
  for k, v in offsets.items():
    try:
      lim = int(k)
    except (TypeError, ValueError):
      continue
    if lim <= 0 or lim % 5 or lim > 90:
      continue
    name = "town_offset_mph" if lim < fast_zone_mph else "fast_offset_mph"
    try:
      fv = float(v)
    except (TypeError, ValueError):
      continue
    if math.isfinite(fv):
      out[str(lim)] = clamp(name, fv)
  return out


def from_dict(d: dict) -> DriverProfile:
  values = d.get("values", {}) if isinstance(d, dict) else {}
  if not isinstance(values, dict):
    values = {}
  p = DriverProfile(**{f.name: clamp(f.name, values[f.name]) for f in fields(DriverProfile)
                       if f.name in values and f.name in BOUNDS})
  p.limit_offsets_mph = clamp_limit_offsets(values.get("limit_offsets_mph"), p.fast_zone_mph)
  return p


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


def is_on(params=None, param: str = PARAM) -> bool:
  """Whether the DriverProfile (or another) param is set. Never raises: False."""
  try:
    if params is None:
      from openpilot.common.params import Params
      params = Params()
    return bool(params.get_bool(param))
  except Exception:
    return False


def to_dict(p: DriverProfile, meta: dict | None = None) -> dict:
  return {"version": VERSION, "values": asdict(p), "meta": meta or {}}
