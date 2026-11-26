"""
Center-line keeper and narrow-street detector (narrow_street.py) on synthetic model outputs. Model y is positive to
the RIGHT; the keeper's curvature is positive to the right.
"""
import unittest
from types import SimpleNamespace as NS

from openpilot.sunnypilot.selfdrive.controls.lib import narrow_street as ns

XS = [0.0, 5.0, 10.0, 20.0, 40.0]


def line(y):
  return NS(x=XS, y=[y] * len(XS))


def model(left_line=-1.5, right_line=1.5, lp=0.9, rp=0.9, left_edge=-5.0, right_edge=5.0, edge_std=0.3, path=0.0,
          curvature=0.0):
  return NS(laneLines=[line(-5.0), line(left_line), line(right_line), line(5.0)],
            laneLineProbs=[0.1, lp, rp, 0.1],
            roadEdges=[line(left_edge), line(right_edge)], roadEdgeStds=[edge_std, edge_std],
            position=line(path), action=NS(desiredCurvature=curvature))


def settle(keeper, md, v=8.0, n=400, **kw):
  args = {"left_blinker": False, "steering_pressed": False, "lat_active": True} | kw
  k = 0.0
  for _ in range(n):
    k = keeper.update(md, v, **args)
  return k


class TestCenterLineKeeper(unittest.TestCase):
  def test_centered_in_lane_does_nothing(self):
    keeper = ns.CenterLineKeeper(0.01)
    self.assertEqual(settle(keeper, model()), 0.0)
    self.assertFalse(keeper.active)

  def test_riding_the_center_line_moves_right(self):
    # path 0.6 m from the left line: our left side is over it. Want left line + 0.3 + 1.0 -> shift 0.7, capped 0.5
    keeper = ns.CenterLineKeeper(0.01)
    k = settle(keeper, model(left_line=-0.6, right_line=2.6))
    self.assertAlmostEqual(keeper.shift, ns.MAX_SHIFT_M, places=3)
    self.assertGreater(k, 0.0)   # to the right

  def test_never_moves_left(self):
    keeper = ns.CenterLineKeeper(0.01)
    self.assertEqual(settle(keeper, model(left_line=-3.0, right_line=0.8)), 0.0)

  def test_no_confident_left_line_does_nothing(self):
    keeper = ns.CenterLineKeeper(0.01)
    self.assertEqual(settle(keeper, model(left_line=-0.6, lp=0.3)), 0.0)

  def test_right_side_limits(self):
    # right line close: only up to right line - 0.2 - 1.0
    keeper = ns.CenterLineKeeper(0.01)
    settle(keeper, model(left_line=-0.6, right_line=1.4))
    self.assertAlmostEqual(keeper.shift, 0.2, places=2)
    # a seen right edge (curb / parked cars) 2.3 m away leaves 0.3 m with the 1.0 m margin
    keeper = ns.CenterLineKeeper(0.01)
    settle(keeper, model(left_line=-0.6, rp=0.1, right_edge=2.3))
    self.assertAlmostEqual(keeper.shift, 0.3, places=2)
    # right edge too close for any shift
    keeper = ns.CenterLineKeeper(0.01)
    self.assertEqual(settle(keeper, model(left_line=-0.6, rp=0.1, right_edge=1.9)), 0.0)

  def test_stands_aside(self):
    md = model(left_line=-0.6, right_line=2.6)
    for kw, v in (({"left_blinker": True}, 8.0), ({"steering_pressed": True}, 8.0), ({"lat_active": False}, 8.0),
                  ({}, 30.0), ({}, 1.0)):
      keeper = ns.CenterLineKeeper(0.01)
      self.assertEqual(settle(keeper, md, v=v, **kw), 0.0, (kw, v))
    keeper = ns.CenterLineKeeper(0.01)   # turning
    self.assertEqual(settle(keeper, model(left_line=-0.6, right_line=2.6, curvature=0.05)), 0.0)

  def test_rate_limited_and_released(self):
    keeper = ns.CenterLineKeeper(0.01)
    md = model(left_line=-0.6, right_line=2.6)
    keeper.update(md, 8.0, False, False, True)
    self.assertAlmostEqual(keeper.shift, ns.SHIFT_RATE * 0.01)
    settle(keeper, md)
    settle(keeper, model(), n=int(ns.MAX_SHIFT_M / (ns.SHIFT_RATE * 0.01)) + 5)
    self.assertEqual(keeper.shift, 0.0)
    settle(keeper, md)
    keeper.update(md, 8.0, False, False, False)   # disengaged: drop at once
    self.assertEqual(keeper.shift, 0.0)


class TestCenterLineKeeperModelFrames(unittest.TestCase):
  """controlsd runs at 100 Hz, the model at 20: with a stamp the model is read once per model frame."""

  def run_both(self, with_stamp: bool):
    rng, keeper, out, reads = __import__("random").Random(4), ns.CenterLineKeeper(0.01), [], 0
    real = ns.ModelView
    def counting(md):
      nonlocal reads
      reads += 1
      return real(md)
    ns.ModelView = counting
    try:
      md, frame = model(), 0
      for tick in range(1000):
        if tick % 5 == 0:   # a new model frame
          md, frame = model(left_line=rng.uniform(-2.0, -0.4), right_line=rng.uniform(1.0, 3.0), lp=rng.choice([0.9, 0.9, 0.1])), frame + 1
        out.append(keeper.update(md, 8.0, False, tick % 97 == 0, True, stamp=frame if with_stamp else None))
    finally:
      ns.ModelView = real
    return out, reads

  def test_same_output_a_fifth_of_the_reads(self):
    plain, reads_plain = self.run_both(False)
    stamped, reads_stamped = self.run_both(True)
    self.assertEqual(plain, stamped)
    self.assertEqual(reads_plain, 1000)
    self.assertEqual(reads_stamped, 200)
    self.assertTrue(any(k > 0 for k in plain))

  def test_new_frame_after_a_pause_is_read(self):
    # the keeper is not called on every tick (a turn, steering off): the next call must not use an old frame
    keeper = ns.CenterLineKeeper(0.01)
    for _ in range(300):
      keeper.update(model(left_line=-0.6, right_line=2.6), 8.0, False, False, True, stamp=1)
    self.assertGreater(keeper.target, 0.0)
    keeper.update(model(), 8.0, False, False, True, stamp=7)   # centered now
    self.assertEqual(keeper.target, 0.0)


class TestNarrowStreet(unittest.TestCase):
  def narrow_model(self, **kw):
    return model(**({"lp": 0.1, "rp": 0.1, "left_edge": -3.5, "right_edge": 4.0} | kw))

  def run_for(self, det, md, s, v=7.0):
    for _ in range(int(s / 0.05)):
      det.update(md, v)
    return det.narrow

  def test_narrow_after_hold(self):
    det = ns.NarrowStreetDetector(0.05)
    self.assertFalse(self.run_for(det, self.narrow_model(), ns.NARROW_ON_S - 0.2))
    self.assertTrue(self.run_for(det, self.narrow_model(), 0.4))

  def test_not_narrow(self):
    for md, v in ((model(), 7.0),                                           # lane lines
                  (self.narrow_model(right_edge=6.0), 7.0),                 # curbs 9.5 m apart
                  (self.narrow_model(edge_std=1.5), 7.0),                   # edges not trusted
                  (self.narrow_model(), 20.0)):                             # too fast for a residential street
      det = ns.NarrowStreetDetector(0.05)
      self.assertFalse(self.run_for(det, md, 5.0, v=v))

  def test_release_needs_hold(self):
    det = ns.NarrowStreetDetector(0.05)
    self.run_for(det, self.narrow_model(), 3.0)
    self.assertTrue(self.run_for(det, model(), ns.NARROW_OFF_S - 0.3))   # brief wide stretch: still narrow
    self.assertTrue(self.run_for(det, self.narrow_model(), 1.0))
    self.assertFalse(self.run_for(det, model(), ns.NARROW_OFF_S + 0.2))


if __name__ == "__main__":
  unittest.main()
