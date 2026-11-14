"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

A compact speed limit sign for the small (mici) driving screen, which has no room for the big UI's SpeedLimitRenderer.
US style (white rectangle, "SPEED LIMIT") in mph, round European style in km/h. The number turns red above the limit
when Speed Limit mode is Warning or Assist, and grey while the current reading is stale (the last limit is kept).
"""
import pyray as rl

from openpilot.common.constants import CV
from openpilot.common.swaglog import cloudlog

MODE_OFF, MODE_WARNING = 0, 2


def sign_state(resolver, v_ego: float, mode: int, is_metric: bool) -> tuple[bool, str, bool, bool]:
  """(show, number, overspeed, stale). No drawing: unit-testable."""
  has_limit = resolver.speedLimitValid or resolver.speedLimitLastValid
  if mode == MODE_OFF or not has_limit or resolver.speedLimitLast <= 0:
    return False, "", False, False
  conv = CV.MS_TO_KPH if is_metric else CV.MS_TO_MPH
  limit = round(resolver.speedLimitLast * conv)
  final = round((resolver.speedLimitFinalLast or resolver.speedLimitLast) * conv)
  overspeed = mode >= MODE_WARNING and round(v_ego * conv) > final
  return True, str(limit), overspeed, not resolver.speedLimitValid


class SpeedLimitSign:
  W, H = 60, 76           # US sign; the round sign is H across
  MARGIN = 20

  def __init__(self):
    self.show = False
    self.number = ""
    self.overspeed = False
    self.stale = False

  @property
  def width(self) -> float:
    from openpilot.selfdrive.ui.ui_state import ui_state
    return (self.H if ui_state.is_metric else self.W) if self.show else 0.0

  def update(self) -> None:
    from openpilot.selfdrive.ui.ui_state import ui_state
    sm = ui_state.sm
    if sm.recv_frame['longitudinalPlanSP'] < ui_state.started_frame:
      self.show = False
      return
    try:
      mode = int(ui_state.speed_limit_mode or 0)
      self.show, self.number, self.overspeed, self.stale = sign_state(sm['longitudinalPlanSP'].speedLimit.resolver,
                                                                      sm['carState'].vEgo, mode, ui_state.is_metric)
    except Exception:
      self.show = False   # a display bug must never take the UI (and with it engagement) down
      cloudlog.exception("speed limit sign: update failed")

  def render(self, rect: rl.Rectangle) -> None:
    if not self.show:
      return
    try:
      self._draw(rect)
    except Exception:
      self.show = False
      cloudlog.exception("speed limit sign: render failed")

  def _draw(self, rect: rl.Rectangle) -> None:
    from openpilot.selfdrive.ui.ui_state import ui_state
    from openpilot.system.ui.lib.application import FontWeight, gui_app
    from openpilot.system.ui.lib.text_measure import measure_text_cached

    bold = gui_app.font(FontWeight.BOLD)
    black = rl.Color(0, 0, 0, 255)
    num_color = rl.Color(220, 30, 30, 255) if self.overspeed else (rl.Color(120, 120, 120, 255) if self.stale else black)

    if ui_state.is_metric:
      d = self.H
      cx, cy = rect.x + rect.width - self.MARGIN - d / 2, rect.y + self.MARGIN + d / 2
      rl.draw_circle(int(cx), int(cy), d / 2, rl.WHITE)
      rl.draw_circle(int(cx), int(cy), d / 2 - 3, rl.Color(200, 16, 46, 255))
      rl.draw_circle(int(cx), int(cy), d / 2 - 11, rl.WHITE)
      size = 34 if len(self.number) < 3 else 28
      w = measure_text_cached(bold, self.number, size)
      rl.draw_text_ex(bold, self.number, rl.Vector2(cx - w.x / 2, cy - w.y / 2), size, 0, num_color)
      return

    x, y = rect.x + rect.width - self.MARGIN - self.W, rect.y + self.MARGIN
    rl.draw_rectangle_rounded(rl.Rectangle(x, y, self.W, self.H), 0.18, 8, rl.WHITE)
    rl.draw_rectangle_rounded_lines_ex(rl.Rectangle(x + 3, y + 3, self.W - 6, self.H - 6), 0.15, 8, 2, black)
    for i, word in enumerate(("SPEED", "LIMIT")):
      w = measure_text_cached(bold, word, 13)
      rl.draw_text_ex(bold, word, rl.Vector2(x + (self.W - w.x) / 2, y + 7 + i * 13), 13, 0, black)
    size = 34
    w = measure_text_cached(bold, self.number, size)
    rl.draw_text_ex(bold, self.number, rl.Vector2(x + (self.W - w.x) / 2, y + 36), size, 0, num_color)
