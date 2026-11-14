"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import os
import tempfile
import unittest

from openpilot.sunnypilot.tools.fisker_drive_review import Signals, driver_stops, group_windows, main, segment_dirs


class TestFiskerDriveReview(unittest.TestCase):
  def test_segment_dirs_sorted_numerically_and_filtered(self):
    with tempfile.TemporaryDirectory() as root:
      for n in (0, 1, 2, 10, 11):
        os.mkdir(os.path.join(root, f"000000e3--abc--{n}"))
      os.mkdir(os.path.join(root, "000000e4--def--0"))
      names = [os.path.basename(d) for d in segment_dirs(root, "000000e3--abc")]
      assert names == [f"000000e3--abc--{n}" for n in (0, 1, 2, 10, 11)]
      assert [os.path.basename(d) for d in segment_dirs(root, "000000e3--abc", "2-10")] == ["000000e3--abc--2", "000000e3--abc--10"]
      assert [os.path.basename(d) for d in segment_dirs(root, "000000e3--abc", "11")] == ["000000e3--abc--11"]

  def test_driver_stops(self):
    speeds = [(t / 10, 0.0 if 10 <= t < 25 or t >= 40 else 5.0) for t in range(52)]
    # 1.0-2.4 s stopped (1.5 s) counts; the stop still going at the end of the log (4.0-5.1 s) counts too
    assert driver_stops(speeds) == [(1.0, 2.5), (4.0, 5.1)]
    assert driver_stops([(0.0, 5.0), (0.1, 0.0), (0.5, 0.0), (0.6, 5.0)]) == []   # too short

  def test_group_windows(self):
    assert group_windows([1.0, 1.5, 2.0, 9.0, 9.4], gap=3.0) == [(1.0, 2.0), (9.0, 9.4)]
    assert group_windows([], gap=1.0) == []

  def test_signals_decode_the_fields_used(self):
    sig = Signals()
    for addr, name in ((0x1C2, "EPS_AdasLatCtrlSts"), (0x1C2, "EPS_AbortFb"), (0x1C4, "EPS_DrvrSteerTq"), (0x1C4, "EPS_DrvrIntvSteerWhlDetd")):
      assert sig.get(addr, bytes(8), name) >= 0

  def test_missing_route_is_an_error(self):
    with tempfile.TemporaryDirectory() as root:
      assert main(["summary", "nope", "--root", root]) == 1
