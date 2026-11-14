"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math

from openpilot.cereal import custom

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.common.params import Params

TurnDirection = custom.ModelDataV2SP.TurnDirection

LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS
# A turn is done once the heading has rotated this far since the turn desire began. Until that blinker goes off the
# desire stays off, so the model isn't asked for a second turn on the road it just turned onto (on-vehicle 2026-11-14:
# the blinker still on after a completed turn had the car try to turn again). Magnitude only, so the yaw-rate sign
# convention of the car doesn't matter: a 60 deg rotation below the turn speed with a blinker on is a turn either way.
TURN_DONE_RAD = math.radians(60.0)


class LaneTurnController:
  def __init__(self, desire_helper):
    self.DH = desire_helper
    self.turn_direction = TurnDirection.none
    self.params = Params()
    self.lane_turn_value = float(self.params.get("LaneTurnValue", return_default=True)) * CV.MPH_TO_MS
    self.param_read_counter = 0
    self.enabled = self.params.get_bool("LaneTurnDesire")
    self.heading_change = 0.0   # rad the car has rotated since the current turn desire began
    self.turn_done_side = None  # 'left' / 'right': that turn is done; no desire until that blinker goes off

  def read_params(self):
    self.enabled = self.params.get_bool("LaneTurnDesire")
    value = float(self.params.get("LaneTurnValue", return_default=True)) * CV.MPH_TO_MS
    self.lane_turn_value = min(float(LANE_CHANGE_SPEED_MIN), value)

  def update_params(self) -> None:
    if self.param_read_counter % 50 == 0:
      self.read_params()
    self.param_read_counter += 1

  def update_lane_turn(self, blindspot_left: bool, blindspot_right: bool, left_blinker: bool, right_blinker: bool, v_ego: float,
                       yaw_rate: float = 0.0, dt: float = DT_MDL) -> None:
    side = 'left' if left_blinker and not right_blinker else 'right' if right_blinker and not left_blinker else None
    if side != self.turn_done_side:
      self.turn_done_side = None   # that blinker went off (or the other one came on): turns are possible again

    if side == 'left' and v_ego < self.lane_turn_value and not blindspot_left and self.turn_done_side is None:
      wanted = TurnDirection.turnLeft
    elif side == 'right' and v_ego < self.lane_turn_value and not blindspot_right and self.turn_done_side is None:
      wanted = TurnDirection.turnRight
    else:
      wanted = TurnDirection.none

    if wanted == TurnDirection.none or wanted != self.turn_direction:
      self.heading_change = 0.0
    if wanted != TurnDirection.none:
      self.heading_change += abs(yaw_rate) * dt
      if self.heading_change >= TURN_DONE_RAD:
        self.turn_done_side = side
        self.heading_change = 0.0
        wanted = TurnDirection.none
    self.turn_direction = wanted

  def get_turn_direction(self):
    if not self.enabled:
      return TurnDirection.none
    return self.turn_direction
