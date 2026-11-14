"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Does a map stop sign apply to the car's direction of travel? At a two-way stop the sign holds the minor road, yet a
car on the major road passes through the same junction and can see the node ahead of it; and a car that has just
left a junction passes the opposing approach's stop node a few metres on. Each sign is reduced to its *approaches*:
the compass bearings of the traffic it holds (None when the map doesn't say), and the car's GPS bearing is compared
against them: YES (within APPLIES_DEG of one), NO (at least NOT_APPLIES_DEG from all of them) or UNKNOWN (no
approaches, or in between). Only NO changes anything: StopSignMap/find_stop_ahead skip that sign. UNKNOWN keeps the
old behaviour, where the driving model's own stop still gates any braking.

OSM conventions relied on (wiki: Tag:highway=stop, Key:direction, Key:stop, Key:traffic_sign:direction; routers:
OSRM profiles/lib/obstacles.lua + car.lua, Valhalla lua/graph.lua):
* direction=forward|backward: the direction of travel the sign holds, relative to the way the stop node lies on,
  forward meaning travel in the way's node order. Resolved against the mapd road tiles (road_geometry.py): the
  bearing of the way's segment arriving at the node in (forward) or against (backward) node order. A node shared by
  several ways (a junction node) makes forward/backward ambiguous, so it stays unresolved; so does a node not found
  on any way of the tiles, or a tile that is not on disk.
  The key read is `direction`, else `stop:direction`, else `traffic_sign:direction` (DIRECTION_KEYS: documented
  synonyms, rarely used).
* direction=both: the node holds both directions of travel on its way: always YES.
* highway=stop + stop=all: an all-way stop. The wiki has mappers put stop=all on *every per-approach node* of a
  detailed all-way stop, not only on the junction node (38% of the stop=all nodes in a San Jose sample carry
  forward/backward), so stop=all does not mean "every bearing": an explicit forward/backward resolved on a single
  way OUTRANKS it, or the car would stop again for the opposing approach's node just past the junction. stop=all
  with no resolvable direction (none, both, a junction node, no road tile) holds every approach: always YES.
  Nothing weaker than an explicit forward/backward (the inference rules below, compass values) narrows a stop=all.
* direction=<degrees> or a cardinal (N, NNE, NE, ... 16 points), several separated by ';': NOT USED
  (COMPASS_DIRECTIONS_TRUSTED = False, these give UNKNOWN). The wiki (Key:direction, Key:traffic_sign) defines
  degree and cardinal values as the direction the sign *FACES*, i.e. against the direction of travel it holds
  ("if you encounter a traffic sign when traveling north, then the sign is facing south ... direction=180 or
  direction=S"), and documents them only for sign nodes placed beside the road, not for highway=stop nodes on a
  way (cardinals "are not allowed" on a way). Real data is inconsistent: on stop nodes measured against the
  nearest-junction rule, degree values were 89% facing while cardinal letters were 69% *travel* (a leftover of the
  old North American stop=N;S proposals). OSRM and Valhalla ignore them. They are 0.29% of stop nodes (0.08% in
  California), so UNKNOWN costs almost nothing. Should the flag ever be enabled, the values are read as the wiki
  defines them, as FACING: travel bearing = value + 180 (travel_from_facing), never as travel bearings; given the
  cardinal-letter data above that would still skip signs wrongly, so don't enable it without a cross-check against
  the way geometry.
  Other values this module can't read (all, ranges like 45-135) are UNKNOWN.
* No direction, on a junction node of ways of different importance (RANK: motorway ... residential): holds the most
  minor road(s) (OSM stop=minor says the same explicitly; OSRM does this by road speed). Equal ranks: UNKNOWN.
* No direction, at the *end* of a single way: holds the traffic reaching that end.
* No direction, in the *middle* of a single way: the nearest-junction rule. The node is mapped at the stop line, so
  the sign holds the traffic travelling toward the nearer end of the way (the junction it guards). Measured on 5,969
  South Bay stop nodes this agrees with the explicit forward/backward tag 97.3% of the time, and OSRM ships a 20 m
  version of it. Only applied when the node is clearly nearer one end: at most NEAREST_END_MAX_M from it along the
  way and at most NEAREST_END_RATIO of the distance to the other end; otherwise UNKNOWN.

Limits: GPS bearing is used as is, so the tolerances are wide; the road tiles hold drivable classes only
(road_geometry.HIGHWAY_CLASSES), so a sign for a service road or a driveway can't be matched and stays UNKNOWN; a
way's end is taken as a junction, though ways are also split where a tag changes; the feature cache (Overpass) and
the road tiles are snapshots of OSM taken at different times, and a node moved or re-drawn in between is simply not
matched (UNKNOWN). The feature cache keeps only webhud/maptiles.py KEEP_TAGS: `stop:direction` and
`traffic_sign:direction` are read here but reach this module only once they are added there.
"""
import math
from enum import IntEnum

from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.road_geometry import RANK

# see the docstring: degree/cardinal values give UNKNOWN (wiki: sign-facing, real data inconsistent, routers ignore them).
# If True they are read as FACING (travel = value + 180), not as travel bearings.
COMPASS_DIRECTIONS_TRUSTED = False
APPLIES_DEG = 60.0        # car bearing this close to an approach: the sign is for us
NOT_APPLIES_DEG = 75.0    # this far from every approach: it's for someone else (a crossing road meets at ~90 deg)

CARDINALS = {name: i * 22.5 for i, name in enumerate(
  ("N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"))}
RELATIVE = ("forward", "backward")
BOTH = "both"
DIRECTION_KEYS = ("direction", "stop:direction", "traffic_sign:direction")   # first one present wins
NEAREST_END_MAX_M = 40.0   # nearest-junction rule: the nearer end of the way is at most this far along the way...
NEAREST_END_RATIO = 0.5    # ...and at most this fraction of the distance to the other end


class Applies(IntEnum):
  NO = 0
  YES = 1
  UNKNOWN = 2


def angle_diff(a: float, b: float) -> float:
  """Absolute difference of two bearings, 0 .. 180 deg."""
  return abs((a - b + 180.0) % 360.0 - 180.0)


def direction_from_tags(tags) -> str | None:
  """The sign's direction value as mapped: `direction`, else its documented synonyms (DIRECTION_KEYS)."""
  if not isinstance(tags, dict):
    return None
  for key in DIRECTION_KEYS:
    value = tags.get(key)
    if isinstance(value, str) and value.strip():
      return value
  return None


def parse_direction(value) -> tuple[float, ...] | str | None:
  """An OSM direction value as 'forward' / 'backward' / 'both', a tuple of the compass values *as tagged* (which the
  wiki defines as the way the sign faces, see travel_from_facing), or None when unusable."""
  if not isinstance(value, str):
    return None
  v = value.strip()
  if v.lower() in RELATIVE or v.lower() == BOTH:
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


def is_relative(direction) -> bool:
  """An explicit forward / backward: the only thing that narrows a stop=all."""
  return parse_direction(direction) in RELATIVE


def is_both(direction) -> bool:
  return parse_direction(direction) == BOTH


def travel_from_facing(facing: tuple[float, ...]) -> tuple[float, ...]:
  """Compass values read as the OSM wiki defines them, the way the sign FACES: the traffic it holds travels the
  opposite way. Only reached with COMPASS_DIRECTIONS_TRUSTED (see the docstring for why it is off)."""
  return tuple((f + 180.0) % 360.0 for f in facing)


def approaches_from_tags(direction: str | None) -> tuple[float, ...] | None:
  """The approaches a direction tag gives without geometry: degree and cardinal values only, and only when trusted."""
  parsed = parse_direction(direction)
  return travel_from_facing(parsed) if isinstance(parsed, tuple) and COMPASS_DIRECTIONS_TRUSTED else None


def _nearest_end_approach(w) -> float | None:
  """Nearest-junction rule for a node in the middle of a way: the bearing of travel toward the clearly nearer end."""
  to_start, to_end = getattr(w, "start_m", None), getattr(w, "end_m", None)
  if to_start is None or to_end is None:
    return None
  near, far = min(to_start, to_end), max(to_start, to_end)
  if near > NEAREST_END_MAX_M or near > NEAREST_END_RATIO * far:
    return None   # no end is clearly the junction this sign guards
  # toward the last node is travel in node order, toward the first is against it
  return w.forward_bearing if to_end < to_start else w.backward_bearing


def approaches_from_ways(direction: str | None, ways, explicit_only: bool = False) -> tuple[float, ...] | None:
  """The approaches of a sign on the given ways (road_geometry.WayAtNode list, or None if the tile isn't there),
  for a forward/backward direction or none at all. None when the geometry doesn't settle it. explicit_only (a
  stop=all sign): only an explicit forward/backward on a single way counts, nothing is inferred."""
  if not ways:
    return None
  parsed = parse_direction(direction)
  if parsed in RELATIVE:
    if len(ways) != 1:
      return None   # a junction node: forward along which way?
    w = ways[0]
    b = w.forward_bearing if parsed == "forward" else w.backward_bearing
    return None if b is None else (b,)
  if explicit_only:
    return None
  if isinstance(parsed, tuple):
    return travel_from_facing(parsed) if COMPASS_DIRECTIONS_TRUSTED else None
  if direction is not None:
    return None     # both (every bearing, see sign_applies) or a direction we can't read: don't guess from the junction
  if len(ways) == 1:
    w = ways[0]
    if w.count < 2:
      return None
    if 0 < w.index < w.count - 1:
      b = _nearest_end_approach(w)   # mid-way with no direction
    else:
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
  """Whether `sign` (anything with approaches, all_way and direction, e.g. stop_sign_map.StopSign) holds a car heading
  bearing_deg. Resolved approaches come first: on a stop=all sign they are only ever an explicit forward/backward."""
  approaches = getattr(sign, "approaches", None)
  if not approaches:
    if getattr(sign, "all_way", False) or is_both(getattr(sign, "direction", None)):
      return Applies.YES
    return Applies.UNKNOWN
  best = min(angle_diff(bearing_deg, a) for a in approaches)
  if best <= APPLIES_DEG:
    return Applies.YES
  if best >= NOT_APPLIES_DEG:
    return Applies.NO
  return Applies.UNKNOWN
