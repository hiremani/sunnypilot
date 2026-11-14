"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from openpilot.selfdrive.ui.mici.onroad.hud_renderer import HudRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.blind_spot_indicators import BlindSpotIndicators
from openpilot.selfdrive.ui.sunnypilot.onroad.speed_limit_sign import SpeedLimitSign
from openpilot.selfdrive.ui.sunnypilot.onroad.stop_sign_indicator import StopSignIndicator


class HudRendererSP(HudRenderer):
  def __init__(self):
    super().__init__()
    self.blind_spot_indicators = BlindSpotIndicators()
    self.stop_sign_indicator = StopSignIndicator()
    self.speed_limit_sign = SpeedLimitSign()

  def _update_state(self) -> None:
    super()._update_state()
    self.blind_spot_indicators.update()
    self.stop_sign_indicator.update()
    self.speed_limit_sign.update()

  def _render(self, rect: rl.Rectangle) -> None:
    super()._render(rect)
    self.blind_spot_indicators.render(rect)
    self.speed_limit_sign.render(rect)
    inset = self.speed_limit_sign.width + 12 if self.speed_limit_sign.show else 0.0
    self.stop_sign_indicator.render(rect, inset)

  def _has_blind_spot_detected(self) -> bool:

    return self.blind_spot_indicators.detected
