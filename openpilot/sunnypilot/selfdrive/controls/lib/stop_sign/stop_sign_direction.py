"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Does a map stop sign apply to the car's direction of travel? At a two-way stop the sign holds the minor road, yet a
car on the major road passes through the same junction and can see the node ahead of it. Each sign is reduced to
its *approaches*: the compass bearings of the traffic it holds (None when the map doesn't say), and the car's GPS
bearing is compared against them: YES (within APPLIES_DEG of one), NO (at least NOT_APPLIES_DEG from all of them)
or UNKNOWN (no approaches, or in between). Only NO changes anything: StopSignMap/find_stop_ahead skip that sign.
UNKNOWN keeps the old behaviour, where the driving model's own stop still gates any braking.

OSM conventions relied on (wiki: Tag:highway=stop, Key:direction, Key:stop):
* highway=stop + stop=all: an all-way stop. It holds every approach: always YES.
* direction=forward|backward: relative to the way the stop node lies on, forward meaning travel in the way's node
  order. Resolved against the mapd road tiles (road_geometry.py): the bearing of the way's segment arriving at the
  node in (forward) or against (backward) node order. A node shared by several ways (a junction node) makes
  forward/backward ambiguous, so it stays UNKNOWN; so does a node not found on any way of the tiles, or a tile
  that is not on disk.
* direction=<degrees> or a cardinal (N, NNE, NE, ... 16 points): read as the compass bearing of the *direction of
  travel* the sign applies to (Key:direction's "direction of travel affected" reading for highway=stop/give_way,
  and how iD presents the field), not the way the sign plate faces (which would be the opposite). Several values
  can be given separated by ';'. Values this module can't read (both, all, ranges like 45-135) are UNKNOWN.
  NOT YET VERIFIED against the live wiki (written without network): Key:direction on points is also read as the way
  a feature *faces*, which for a sign plate is the opposite of the traffic it holds. Read the wrong way round, every
  such sign would be skipped, so until checked COMPASS_DIRECTIONS_TRUSTED is False and these values give UNKNOWN.
* No direction: like the webhud page's map (state.js), a stop node at the *end* of a single way holds the traffic
  reaching that end; one on a junction node of ways of different importance (RANK: motorway ... residential) holds
  the most minor road(s) (OSM stop=minor says the same explicitly). Ways of equal rank, or a node in the middle of
  a single way, stay UNKNOWN.

Limits: GPS bearing is used as is, so the tolerances are wide; the road tiles hold drivable classes only
(road_geometry.HIGHWAY_CLASSES), so a sign for a service road or a driveway can't be matched and stays UNKNOWN; the
feature cache (Overpass) and the road tiles are snapshots of OSM taken at different times, and a node moved or
re-drawn in between is simply not matched (UNKNOWN).
"""
import math
from enum import IntEnum

from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.road_geometry import RANK

COMPASS_DIRECTIONS_TRUSTED = False  # see the docstring: numeric/cardinal values give UNKNOWN until the convention is checked
APPLIES_DEG = 60.0        # car bearing this close to an approach: the sign is for us
NOT_APPLIES_DEG = 75.0    # this far from every approach: it's for someone else (a crossing road meets at ~90 deg)

CARDINALS = {name: i * 22.5 for i, name in enumerate(
  ("N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"))}
RELATIVE = ("forward", "backward")


class Applies(IntEnum):
  NO = 0
  YES = 1
  UNKNOWN = 2


def angle_diff(a: float, b: float) -> float:
  """Absolute difference of two bearings, 0 .. 180 deg."""
  return abs((a - b + 180.0) % 360.0 - 180.0)


def parse_direction(value) -> tuple[float, ...] | str | None:
  """An OSM direction value as a tuple of compass bearings, 'forward' / 'backward', or None when unusable."""
  if not isinstance(value, str):
    return None
  v = value.strip()
  if v.lower() in RELATIVE:
    return v.lower()
  out = []
  for part in v.split(";"):
    part = part.strip()
    if part.upper() in CARDINALS:
      out.append(CARDINALS[part.upper()])
      continue
    try:
      deg = float(part)
    except ValueError:
      return None
    if not math.isfinite(deg):
      return None
    out.append(deg % 360.0)
  return tuple(out) or None


def approaches_from_tags(direction: str | None) -> tuple[float, ...] | None:
  """The approaches a direction tag gives without geometry: numeric and cardinal values only."""
  parsed = parse_direction(direction)
  return parsed if isinstance(parsed, tuple) and COMPASS_DIRECTIONS_TRUSTED else None


def approaches_from_ways(direction: str | None, ways) -> tuple[float, ...] | None:
  """The approaches of a sign on the given ways (road_geometry.WayAtNode list, or None if the tile isn't there),
  for a forward/backward direction or none at all. None when the geometry doesn't settle it."""
  if not ways:
    return None
  parsed = parse_direction(direction)
  if isinstance(parsed, tuple):
    return parsed if COMPASS_DIRECTIONS_TRUSTED else None
  if parsed in RELATIVE:
    if len(ways) != 1:
      return None   # a junction node: forward along which way?
    w = ways[0]
    b = w.forward_bearing if parsed == "forward" else w.backward_bearing
    return None if b is None else (b,)
  if direction is not None:
    return None     # a direction we can't read: don't guess from the junction either
  if len(ways) == 1:
    w = ways[0]
    if w.count < 2 or 0 < w.index < w.count - 1:
      return None   # mid-way with no direction: both ways, or a crossing road the tiles don't have
    b = w.forward_bearing if w.index == w.count - 1 else w.backward_bearing
    return None if b is None else (b,)
  ranks = [RANK.get(w.highway, 4) for w in ways]
  minor = max(ranks)
  if min(ranks) == minor:
    return None     # equal roads: which one stops can't be told
  out = []
  for w, r in zip(ways, ranks, strict=True):
    if r == minor:
      out += [b for b in (w.forward_bearing, w.backward_bearing) if b is not None]
  return tuple(out) or None


def sign_applies(sign, bearing_deg: float) -> Applies:
  """Whether `sign` (anything with all_way and approaches, e.g. stop_sign_map.StopSign) holds a car heading bearing_deg."""
  if getattr(sign, "all_way", False):
    return Applies.YES
  approaches = getattr(sign, "approaches", None)
  if not approaches:
    return Applies.UNKNOWN
  best = min(angle_diff(bearing_deg, a) for a in approaches)
  if best <= APPLIES_DEG:
    return Applies.YES
  if best >= NOT_APPLIES_DEG:
    return Applies.NO
  return Applies.UNKNOWN
