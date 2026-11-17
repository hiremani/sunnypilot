"""drive_summary's pieces for the speed control plan's Phase 0: who held the wheel in each assisted turn, why each
openpilot speed control period ended, and how each stop felt."""
import unittest

from openpilot.sunnypilot.tools.drive_summary import StopTracker, long_handover_summary, turn_steering


def lat(eps_sts, tx=True, **flags):
  return {"event": "fisker_lat_handover", "eps_sts": eps_sts, "tx": tx, **flags}


class TestTurnSteering(unittest.TestCase):
  def test_share_of_the_turn_the_eps_was_active_and_the_latches_seen(self):
    h = [(0.0, lat(2)), (12.0, lat(1, intervening=True, torque_latch_only=True)), (14.0, lat(2))]
    r = turn_steering(h, 10.0, 16.0)      # active 10-12 and 14-16 of a 6 s turn
    self.assertEqual(r["eps_active_pct"], 67)
    self.assertEqual(r["flags"], ["intervening", "torque_latch_only"])
    self.assertEqual(r["eps_states"], ["active", "available"])

  def test_not_transmitting_is_not_active(self):
    self.assertEqual(turn_steering([(0.0, lat(2, tx=False))], 1.0, 3.0)["eps_active_pct"], 0)

  def test_no_lines_no_figure(self):
    self.assertEqual(turn_steering([], 0.0, 5.0), {})


class TestLongHandover(unittest.TestCase):
  def test_runs_and_why_they_ended(self):
    on = {"commanding": True, "long_active": True, "acc_state": 3, "oem_vld": 1}
    h = [(0, on), (5, {**on, "commanding": False, "gas": True}), (6, on),
         (9, {**on, "commanding": False, "acc_state": 2}), (10, on), (12, {**on, "commanding": False, "long_active": False})]
    self.assertEqual(long_handover_summary(h), {"runs": 3, "ended": {"gas": 1, "acc state 2": 1, "openpilot off": 1}})


class TestStops(unittest.TestCase):
  def _stop(self, decels, long_active=False, brake=False, roll_v=0.0):
    st, t, v = StopTracker(), 0.0, 6.0
    for _ in range(50):                    # cruise at 6 m/s
      st.add(t, v, 0.0, False, False, True, long_active)
      t += 0.1
    for a in decels:
      v = max(v + a * 0.1, 0.05)
      st.add(t, v, a, False, brake, True, long_active)
      t += 0.1
    for _ in range(40):                    # standstill flag, possibly still creeping
      st.add(t, roll_v, 0.0, True, brake, True, long_active)
      t += 0.1
    return st.stops

  def test_who_stopped_and_the_final_jerk(self):
    smooth = self._stop([-1.0] * 55 + [-0.6, -0.4, -0.2, -0.1, 0.0], long_active=True)
    nod = self._stop([-1.0] * 50 + [-2.5] * 8 + [0.0], long_active=True)
    self.assertEqual([s["who"] for s in smooth + nod], ["op-long", "op-long"])
    self.assertLess(smooth[0]["final_jerk"], 5.0)
    self.assertGreater(nod[0]["final_jerk"], 6.0)
    self.assertEqual(self._stop([-1.0] * 60, brake=True)[0]["who"], "driver")

  def test_roll_after_the_standstill_flag(self):
    self.assertAlmostEqual(self._stop([-1.0] * 60, roll_v=0.5)[0]["rolled_m"], 1.5, delta=0.1)


if __name__ == "__main__":
  unittest.main()
