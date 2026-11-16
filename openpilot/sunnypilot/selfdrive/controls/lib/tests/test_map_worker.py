"""The map worker process (map_worker.py) and the CPU budget (cpu_budget.py)."""
import json
import math
import os
import tempfile
import time
import unittest

from openpilot.sunnypilot.selfdrive.controls.lib import cpu_budget, map_worker
from openpilot.sunnypilot.webhud.maptiles import FEAT_DEG

LAT, LON = 37.5003, -121.9502


def wait_for(results: dict, n: int, timeout: float = 30.0) -> None:
  deadline = time.monotonic() + timeout
  while len(results) < n and time.monotonic() < deadline:
    time.sleep(0.02)


class TestMapWorker(unittest.TestCase):
  def setUp(self):
    self.tmp = tempfile.TemporaryDirectory()
    feats = os.path.join(self.tmp.name, "features")
    os.makedirs(feats)
    self.key = (math.floor(LAT / FEAT_DEG), math.floor(LON / FEAT_DEG))
    with open(os.path.join(feats, f"{self.key[0]}_{self.key[1]}.json"), "w") as f:
      json.dump({"nodes": [[1, LAT, LON, {"highway": "stop", "direction": "both"}],
                           [2, LAT + 0.001, LON, {"traffic_calming": "hump"}],
                           [3, LAT + 0.002, LON, {"highway": "traffic_signals"}]]}, f)
    self.w = map_worker.MapWorker(features_root=feats, roads_root=os.path.join(self.tmp.name, "roads"), download=False)

  def tearDown(self):
    self.w.close()
    self.tmp.cleanup()

  def test_one_read_serves_every_map(self):
    got = {}
    for kind in ("stops", "bumps", "points"):
      self.w.request_cell(self.key, kind, lambda items, kind=kind: got.__setitem__(kind, items))
    wait_for(got, 3)
    self.assertEqual([s.node_id for s in got["stops"]], [1])
    self.assertEqual([(b.node_id, b.kind) for b in got["bumps"]], [(2, "hump")])
    self.assertEqual([(p.node_id, p.kind) for p in got["points"]], [(3, "signal")])
    self.assertEqual(self.w.starts, 1)
    again = {}
    self.w.request_cell(self.key, "bumps", lambda items: again.__setitem__("b", items))   # from the cache, at once
    self.assertEqual([b.node_id for b in again["b"]], [2])

  def test_missing_cell_and_roads_give_none(self):
    got = {}
    self.w.request_cell((self.key[0] + 5, self.key[1]), "stops", lambda items: got.__setitem__("cell", items))
    self.w.request_roads(os.path.join(self.tmp.name, "roads"), LAT, LON, lambda r: got.__setitem__("roads", r))
    wait_for(got, 2)
    self.assertEqual(got, {"cell": None, "roads": None})

  def test_a_dead_worker_fails_its_requests_and_is_restarted_later(self):
    self.w.start()
    self.w._proc.kill()
    deadline = time.monotonic() + 10
    while self.w._proc is not None and time.monotonic() < deadline:   # the reader notices
      time.sleep(0.01)
    got = {}
    self.w.request_cell(self.key, "stops", lambda items: got.__setitem__("first", items))
    wait_for(got, 1)
    self.assertIsNone(got["first"])            # owed by the dead worker (or refused while it is down)
    self.w._down_until = 0.0
    self.w.request_cell(self.key, "stops", lambda items: got.__setitem__("second", items))
    wait_for(got, 2)
    self.assertEqual([s.node_id for s in got["second"]], [1])

  def test_the_worker_runs_low_priority(self):
    if not map_worker.launch_prefix():
      self.skipTest("no chrt/taskset here")
    self.w.start()
    pid = self.w._proc.pid
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:   # chrt -> taskset -> nice -> python, all exec'd under one pid
      with open(f"/proc/{pid}/cmdline", "rb") as f:
        if b"map_worker" in f.read() and b"python" in open(f"/proc/{pid}/comm", "rb").read():
          break
      time.sleep(0.01)
    with open(f"/proc/{pid}/stat") as f:
      fields = f.read().rsplit(")", 1)[1].split()
    self.assertEqual(int(fields[16]), 19)                       # nice
    self.assertEqual(int(fields[38]), 0)                        # SCHED_OTHER
    self.assertTrue(os.sched_getaffinity(pid) <= map_worker.WORKER_CPUS)


class TestCpuBudget(unittest.TestCase):
  def setUp(self):
    self.now = 0.0
    self.cost = 0.0
    self.ticks = 0.0
    self.logged = []

  def make(self, **kw):
    def timer():
      self.ticks += self.cost
      return self.ticks
    return cpu_budget.CpuBudget("x", clock=lambda: self.now, timer=timer, log=lambda *a: self.logged.append(a), **kw)

  def test_a_spike_pauses_and_the_pause_ends(self):
    paused = []
    b = self.make(on_pause=lambda: paused.append(1))
    self.cost = 0.001
    self.assertTrue(b.run(lambda: None))
    self.cost = 0.030                       # 30 ms
    self.assertTrue(b.run(lambda: None))
    self.assertEqual(paused, [1])
    self.cost = 0.001
    self.assertFalse(b.run(lambda: None))   # skipped while paused
    self.now += cpu_budget.PAUSE_S + 0.1
    self.assertTrue(b.run(lambda: None))
    self.assertEqual(len(self.logged), 1)

  def test_a_slow_average_pauses_and_repeat_pauses_grow(self):
    b = self.make()
    self.cost = 0.005                       # 5 ms every tick: under the spike limit, over the average
    n = 0
    while b.run(lambda: None):
      n += 1
    self.assertGreater(n, 5)
    first = b.paused_until - self.now
    self.now = b.paused_until
    while b.run(lambda: None):
      pass
    self.assertAlmostEqual(b.paused_until - self.now, 2 * first)

  def test_cheap_never_pauses(self):
    b = self.make()
    self.cost = 0.0005
    self.assertTrue(all(b.run(lambda: None) for _ in range(5000)))
    self.assertEqual(self.logged, [])


if __name__ == "__main__":
  unittest.main()
