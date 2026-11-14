"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Stop Sign Assist on the driving screen: a red octagon in the top-right corner.
  tracked (a sign ahead for this direction, the assist not acting yet): steady, dimmed, with the distance
  acting (coasting, braking): blinking, with the distance
  stopped: steady, "tap gas"
  proceeding (the driver's gas tap released the sign): blinking until the car has rolled PROCEED_M on
Shadow mode (assist not in control) draws an outline only, so the driver knows it would act but won't brake.
"""
import math
from enum import IntEnum

import pyray as rl

from openpilot.common.swaglog import cloudlog

PROCEED_M = 20.0        # after a stop, keep blinking until the car has rolled this far (through the junction)
PROCEED_MAX_S = 15.0    # ...or this long, whichever is first
BLINK_HZ = 1.4
SHOW_TRACKED_M = 150.0
M_TO_FT = 3.28084  # only show a tracked (not yet acting) sign this close to the stop point

# custom.capnp LongitudinalPlanSP.StopSign.StopSignState
DISABLED, IDLE, COASTING, BRAKING, STOPPED = range(5)


class Phase(IntEnum):
  NONE = 0
  TRACKED = 1
  ACTING = 2
  STOPPED = 3
  PROCEEDING = 4


class StopSignDisplay:
  """What to show, from longitudinalPlanSP.stopSign and the car's speed. No drawing: unit-testable."""

  def __init__(self):
    self.phase = Phase.NONE
    self.distance = 0.0
    self.shadow = False
    self._proceed_m = 0.0
    self._proceed_s = 0.0

  def update(self, stop_sign, v_ego: float, dt: float) -> Phase:
    state = int(getattr(stop_sign.state, "raw", stop_sign.state))   # a capnp enum, or an int in tests
    sign_id = int(stop_sign.signId)
    self.shadow = bool(stop_sign.shadow)
    self.distance = max(0.0, float(stop_sign.distance))

    if state in (COASTING, BRAKING):
      self.phase = Phase.ACTING
    elif state == STOPPED:
      self.phase = Phase.STOPPED
      self._proceed_m = self._proceed_s = 0.0
    elif self.phase in (Phase.STOPPED, Phase.PROCEEDING):
      # released (gas tap): blink while the car pulls through the junction
      self._proceed_m += max(v_ego, 0.0) * dt
      self._proceed_s += dt
      if self._proceed_m < PROCEED_M and self._proceed_s < PROCEED_MAX_S:
        self.phase = Phase.PROCEEDING
        return self.phase
      self.phase = Phase.NONE
    else:
      self.phase = Phase.NONE
    if self.phase == Phase.NONE and state == IDLE and sign_id != 0 and self.distance <= SHOW_TRACKED_M:
      self.phase = Phase.TRACKED   # the next sign ahead
    return self.phase

  def blinking(self) -> bool:
    return self.phase in (Phase.ACTING, Phase.PROCEEDING)

  def label(self, is_metric: bool) -> str:
    if self.phase == Phase.STOPPED:
      return "tap gas"
    if self.phase == Phase.PROCEEDING:
      return "go"
    if is_metric:
      return f"{self.distance:.0f} m"
    return f"{self.distance * M_TO_FT:.0f} ft"


class StopSignIndicator:
  SIZE = 80         # octagon width, px
  MARGIN_X = 24
  MARGIN_Y = 24

  def __init__(self):
    self.display = StopSignDisplay()
    self._last_t = None

  def update(self) -> None:
    from openpilot.selfdrive.ui.ui_state import ui_state
    sm = ui_state.sm
    now = rl.get_time()
    dt = 0.0 if self._last_t is None else min(now - self._last_t, 0.2)
    self._last_t = now
    if sm.recv_frame['longitudinalPlanSP'] < ui_state.started_frame:
      self.display.phase = Phase.NONE
      return
    try:
      self.display.update(sm['longitudinalPlanSP'].stopSign, sm['carState'].vEgo, dt)
    except Exception:
      # a display bug must never take the UI (and with it engagement) down
      self.display.phase = Phase.NONE
      cloudlog.exception("stop sign indicator: update failed")

  def render(self, rect: rl.Rectangle, right_inset: float = 0.0) -> None:
    try:
      self._draw(rect, right_inset)
    except Exception:
      self.display.phase = Phase.NONE
      cloudlog.exception("stop sign indicator: render failed")

  def _draw(self, rect: rl.Rectangle, right_inset: float) -> None:
    from openpilot.selfdrive.ui.ui_state import ui_state
    from openpilot.system.ui.lib.application import FontWeight, gui_app
    from openpilot.system.ui.lib.text_measure import measure_text_cached

    d = self.display
    if d.phase == Phase.NONE:
      return
    alpha = 1.0
    if d.phase == Phase.TRACKED:
      alpha = 0.55
    elif d.blinking() and (rl.get_time() * BLINK_HZ) % 1.0 >= 0.5:
      alpha = 0.2

    r = self.SIZE / 2
    cx = rect.x + rect.width - self.MARGIN_X - right_inset - r
    cy = rect.y + self.MARGIN_Y + r
    center = rl.Vector2(cx, cy)
    red = rl.Color(200, 16, 46, int(255 * alpha))
    white = rl.Color(255, 255, 255, int(255 * alpha))
    circ = r / math.cos(math.pi / 8)   # draw_poly takes the circumradius; r is the flat-to-flat half width
    if d.shadow:
      rl.draw_poly_lines_ex(center, 8, circ, 22.5, 5, red)
    else:
      rl.draw_poly(center, 8, circ, 22.5, white)
      rl.draw_poly(center, 8, circ - 5, 22.5, red)

    font = gui_app.font(FontWeight.BOLD)
    text = "STOP"
    size = 24
    w = measure_text_cached(font, text, size).x
    rl.draw_text_ex(font, text, rl.Vector2(cx - w / 2, cy - size / 2), size, 0, white if not d.shadow else red)

    label = d.label(ui_state.is_metric)
    lsize = 26
    lw = measure_text_cached(font, label, lsize).x
    rl.draw_text_ex(font, label, rl.Vector2(cx - lw / 2, cy + r + 6), lsize, 0, rl.Color(255, 255, 255, int(255 * max(alpha, 0.55))))
