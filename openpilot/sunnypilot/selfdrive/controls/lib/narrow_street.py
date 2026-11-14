"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Narrow two-way streets, from what the driving model sees (no radar or object list is available on this car).

Route 000000e8 (2026-11-13, night): on streets with cars parked on both sides and no lane lines, the model's path sat
~0.4 m left of the curbs' center (on wider roads ~0.5 m right of it), up to riding the center line, and the driver
pushed the wheel right for 4-8 s at a time. The model's road edges are the curbs, behind the parked cars, so they
can't say how much room is really left: nothing here steers toward the curb.

CenterLineKeeper (controlsd): when the model sees the line on our left (the center line on a two-way street) with
confidence, keep the car's left side LINE_CLEAR_M right of it, by a curvature correction added to the model's. Only
ever right, at most MAX_SHIFT_M, never past what the right lane line / right road edge allow, and never with the left
blinker on, in a turn, while the driver steers, or above V_KEEP_MAX. Where the left line isn't seen it does nothing.

NarrowStreetDetector (plannerd): no lane lines and curbs less than NARROW_WIDTH_M apart, at town speed, held for
NARROW_ON_S. The planner caps the speed at NARROW_SPEED while it lasts (openpilot longitudinal only), and selfdrived
shows a "Narrow street" prompt: the car can't see oncoming or parked cars, so who yields is the driver's call.

Frames: model y is positive to the RIGHT (device frame: left road edge y < 0). Curvature returned positive = right,
as this port's desiredCurvature (see turn_planner.py).
"""
import numpy as np

PARAM = "NarrowStreetAssist"

# ---- center-line keeper ----
LOOK_X_M = 8.0            # lines and path compared this far ahead
LINE_PROB_MIN = 0.5       # the left line must be this confident
EDGE_STD_MAX = 0.8        # a road edge this uncertain is not used as a limit
HALF_WIDTH_M = 1.0        # Fisker Ocean ~1.98 m wide
LINE_CLEAR_M = 0.3        # our left side at least this far right of the left line
RIGHT_LINE_CLEAR_M = 0.2  # ...without bringing our right side closer than this to a seen right line
RIGHT_EDGE_CLEAR_M = 1.0  # ...or this to a seen right road edge (curb or parked cars: f0 2026-11-14, driver eased left at 0.7 m)
MAX_SHIFT_M = 0.5
SHIFT_RATE = 0.3          # m/s: how fast the applied shift may change
V_KEEP_MIN = 2.0          # m/s
V_KEEP_MAX = 25.0         # m/s (~55 mph)
TURN_CURVATURE = 0.02     # 1/m: the model is turning (R < 50 m): stand aside
LOOKAHEAD_S = 2.0         # pure-pursuit lookahead for the correction
LOOKAHEAD_MIN_M = 12.0

# ---- narrow street ----
NARROW_WIDTH_M = 8.5      # curb to curb
NARROW_RELEASE_WIDTH_M = 9.5
NARROW_LINE_PROB_MAX = 0.3
NARROW_V_MIN = 2.0
NARROW_V_MAX = 13.4       # m/s (~30 mph): above, not a residential street
NARROW_ON_S = 1.5
NARROW_OFF_S = 3.0
NARROW_SPEED = 8.0        # m/s (~18 mph): the driver's own speed on these streets was 11-19 mph
NARROW_ACCEL = -1.0       # m/s^2 used to get down to it


def y_at(xs, ys, x: float) -> float:
  xs, ys = np.asarray(xs, dtype=float), np.asarray(ys, dtype=float)
  if len(xs) == 0 or len(xs) != len(ys):
    return float("nan")
  return float(np.interp(x, xs, ys))


class ModelView:
  """The few numbers both parts need, read from a modelV2 once."""

  def __init__(self, md):
    lines = list(md.laneLines)
    probs = list(md.laneLineProbs)
    edges = list(md.roadEdges)
    stds = list(md.roadEdgeStds)
    self.left_line = y_at(lines[1].x, lines[1].y, LOOK_X_M) if len(lines) > 2 else float("nan")
    self.right_line = y_at(lines[2].x, lines[2].y, LOOK_X_M) if len(lines) > 2 else float("nan")
    self.left_prob = float(probs[1]) if len(probs) > 2 else 0.0
    self.right_prob = float(probs[2]) if len(probs) > 2 else 0.0
    self.left_edge = y_at(edges[0].x, edges[0].y, LOOK_X_M) if len(edges) > 1 else float("nan")
    self.right_edge = y_at(edges[1].x, edges[1].y, LOOK_X_M) if len(edges) > 1 else float("nan")
    self.left_edge_std = float(stds[0]) if len(stds) > 1 else 99.0
    self.right_edge_std = float(stds[1]) if len(stds) > 1 else 99.0
    self.path = y_at(md.position.x, md.position.y, LOOK_X_M)
    self.curvature = float(md.action.desiredCurvature)


def keeper_target_shift(mv: ModelView) -> float:
  """How far right (m, >= 0) the path should move so our left side clears the left line. 0 when there is no
  confident left line or nothing to do."""
  if mv.left_prob < LINE_PROB_MIN or not np.isfinite(mv.left_line) or not np.isfinite(mv.path):
    return 0.0
  want = mv.left_line + LINE_CLEAR_M + HALF_WIDTH_M - mv.path
  if want <= 0.0:
    return 0.0
  room = MAX_SHIFT_M
  if mv.right_prob >= LINE_PROB_MIN and np.isfinite(mv.right_line):
    room = min(room, mv.right_line - RIGHT_LINE_CLEAR_M - HALF_WIDTH_M - mv.path)
  if mv.right_edge_std <= EDGE_STD_MAX and np.isfinite(mv.right_edge):
    room = min(room, mv.right_edge - RIGHT_EDGE_CLEAR_M - HALF_WIDTH_M - mv.path)
  return float(max(0.0, min(want, room)))


class CenterLineKeeper:
  def __init__(self, dt: float):
    self.dt = dt
    self.shift = 0.0          # m, applied (rate limited)
    self.target = 0.0         # m, wanted this frame
    self.active = False

  def update(self, md, v_ego: float, left_blinker: bool, steering_pressed: bool, lat_active: bool) -> float:
    """The curvature (1/m, positive = right) to add to the model's this tick."""
    mv = ModelView(md)
    ok = (lat_active and V_KEEP_MIN <= v_ego <= V_KEEP_MAX and not left_blinker and not steering_pressed
          and abs(mv.curvature) < TURN_CURVATURE)
    self.target = keeper_target_shift(mv) if ok else 0.0
    step = SHIFT_RATE * self.dt
    self.shift += float(np.clip(self.target - self.shift, -step, step))
    if not lat_active:
      self.shift = 0.0
    self.active = self.shift > 0.01
    lookahead = max(LOOKAHEAD_MIN_M, v_ego * LOOKAHEAD_S)
    return 2.0 * self.shift / lookahead ** 2


def narrow_now(mv: ModelView, v_ego: float) -> bool:
  return (mv.left_edge_std <= EDGE_STD_MAX and mv.right_edge_std <= EDGE_STD_MAX
          and np.isfinite(mv.left_edge) and np.isfinite(mv.right_edge)
          and mv.right_edge - mv.left_edge < NARROW_WIDTH_M
          and mv.left_prob < NARROW_LINE_PROB_MAX and mv.right_prob < NARROW_LINE_PROB_MAX
          and NARROW_V_MIN <= v_ego <= NARROW_V_MAX)


def wide_now(mv: ModelView, v_ego: float) -> bool:
  return (v_ego > NARROW_V_MAX + 1.5 or max(mv.left_prob, mv.right_prob) >= LINE_PROB_MIN
          or (np.isfinite(mv.left_edge) and np.isfinite(mv.right_edge) and mv.right_edge - mv.left_edge > NARROW_RELEASE_WIDTH_M))


class NarrowStreetDetector:
  def __init__(self, dt: float):
    self.dt = dt
    self.narrow = False
    self._t = 0.0

  def update(self, md, v_ego: float) -> bool:
    mv = ModelView(md)
    if not self.narrow:
      self._t = self._t + self.dt if narrow_now(mv, v_ego) else 0.0
      if self._t >= NARROW_ON_S:
        self.narrow, self._t = True, 0.0
    else:
      self._t = self._t + self.dt if wide_now(mv, v_ego) else 0.0
      if self._t >= NARROW_OFF_S:
        self.narrow, self._t = False, 0.0
    return self.narrow
