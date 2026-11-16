#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The map worker: every heavy map read for plannerd and controlsd, in a process of its own.

Reading one feature cell costs ~50 ms of JSON per map plus 2-5 s of road-tile geometry for its stop signs, and a turn
planner road load 0.3-0.8 s (measured on the device). Done on threads inside plannerd / controlsd, that work holds the
GIL the planner and the 100 Hz controls loop need. Here it runs in a child process at the lowest priority, on the cores
that have room (0-3, away from the real-time cores 4 and 5), with no real-time scheduling. Each cell is parsed once for
all three maps (stop signs, speed bumps, map caution points), and only the finished lists come back.

Protocol: requests are JSON lines on the child's stdin ({"id", "op": "cell", "key"} or {"id", "op": "roads", "lat",
"lon"}); replies are length-prefixed pickles of (id, result) on a private copy of its stdout (stray prints go to
/dev/null). The child exits when its stdin closes, i.e. with its parent.

MapWorker.shared() is the per-process client. A request whose worker is down, can't start, or dies gets None (the
caller retries later); nothing ever falls back to loading inside the calling process.
"""
import json
import os
import pickle
import subprocess
import sys
import threading
import time
from collections import OrderedDict

CELL_CACHE = 27         # finished cells kept for the other maps (3 x 3 around the car, three times over)
RESTART_S = 30.0        # after a failed start or a crash, wait this long before starting again
WORKER_CPUS = {0, 1, 2, 3}


def lower_priority() -> None:
  """No real-time scheduling (inherited from controlsd / plannerd), lowest nice, off the real-time cores."""
  try:
    os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
  except (AttributeError, OSError):
    pass
  try:
    cpus = WORKER_CPUS & os.sched_getaffinity(0) or WORKER_CPUS & set(range(os.cpu_count() or 1))
    if cpus:
      os.sched_setaffinity(0, cpus)
  except (AttributeError, OSError):
    pass
  try:
    os.nice(19)
  except OSError:
    pass


def launch_prefix() -> list[str]:
  """chrt / taskset / nice in front of the worker's command, so it leaves the real-time policy and cores of its parent
  before the interpreter even starts (and Popen can spawn without forking the parent: no preexec_fn)."""
  import shutil
  if not all(shutil.which(t) for t in ("chrt", "taskset", "nice")):
    return []
  cpus = sorted(WORKER_CPUS & set(range(os.cpu_count() or 1)))
  return ["chrt", "-o", "0", "taskset", "-c", ",".join(map(str, cpus)), "nice", "-n", "19"]


class MapWorker:
  _shared: "MapWorker | None" = None
  _shared_lock = threading.Lock()

  @classmethod
  def shared(cls) -> "MapWorker":
    with cls._shared_lock:
      if cls._shared is None:
        cls._shared = cls()
      return cls._shared

  def __init__(self, features_root: str | None = None, roads_root: str | None = None, download: bool = True):
    self.args = []
    if features_root is not None:
      self.args += ["--features", features_root]
    if roads_root is not None:
      self.args += ["--roads", roads_root]
    if not download:
      self.args.append("--no-download")
    self._lock = threading.Lock()
    self._proc: subprocess.Popen | None = None
    self._next_id = 0
    self._cbs: dict[int, tuple[subprocess.Popen, object]] = {}
    self._cells: OrderedDict = OrderedDict()                 # key -> {"stops", "bumps", "points"}
    self._waiting: dict[tuple[int, int], list] = {}          # key -> [(kind, callback)]
    self._down_until = 0.0
    self.starts = 0

  # -- process --

  def _ensure(self) -> bool:
    # under self._lock
    if self._proc is not None and self._proc.poll() is None:
      return True
    if time.monotonic() < self._down_until:
      return False
    try:
      proc = subprocess.Popen(launch_prefix() + [sys.executable, "-m", "openpilot.sunnypilot.selfdrive.controls.lib.map_worker",
                                                 *self.args],
                              stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, close_fds=True)
    except Exception:
      self._log_exception("map worker: start failed")
      self._down_until = time.monotonic() + RESTART_S
      return False
    self._proc = proc
    self.starts += 1
    threading.Thread(target=self._reader, args=(proc,), daemon=True).start()
    return True

  def start(self) -> None:
    """Start the worker ahead of the first request (its imports take a second or two)."""
    with self._lock:
      self._ensure()

  def _send(self, req: dict, cb) -> bool:
    # under self._lock
    if not self._ensure():
      return False
    rid = self._next_id
    self._next_id += 1
    self._cbs[rid] = (self._proc, cb)
    try:
      self._proc.stdin.write((json.dumps({"id": rid, **req}) + "\n").encode())
      self._proc.stdin.flush()
    except (OSError, ValueError):
      del self._cbs[rid]
      self._down_until = time.monotonic() + RESTART_S
      return False
    return True

  def _reader(self, proc: subprocess.Popen) -> None:
    f = proc.stdout
    while True:
      try:
        hdr = f.read(4)
        if len(hdr) < 4:
          break
        data = f.read(int.from_bytes(hdr, "little"))
        rid, result = pickle.loads(data)
      except Exception:
        break
      with self._lock:
        entry = self._cbs.pop(rid, None)
      if entry is not None:
        self._call(entry[1], result)
    # the worker is gone: fail what it still owed, start a new one on a later request
    try:
      proc.kill()
    except Exception:
      pass
    with self._lock:
      owed = [rid for rid, (p, _) in self._cbs.items() if p is proc]
      cbs = [self._cbs.pop(rid)[1] for rid in owed]
      if self._proc is proc:
        self._proc = None
        self._down_until = time.monotonic() + RESTART_S
    if owed:
      self._log_event("map worker exited", owed=len(owed))
    for cb in cbs:
      self._call(cb, None)

  def close(self) -> None:
    with self._lock:
      proc, self._proc = self._proc, None
    if proc is not None:
      try:
        proc.stdin.close()
        proc.wait(timeout=5)
      except Exception:
        proc.kill()

  # -- requests --

  def request_cell(self, key: tuple[int, int], kind: str, cb) -> None:
    """cb(items or None) once cell `key` has been read; every kind of a cell comes from one read."""
    key = (int(key[0]), int(key[1]))
    with self._lock:
      done = self._cells.get(key)
      if done is not None:
        self._cells.move_to_end(key)
      elif key in self._waiting:
        self._waiting[key].append((kind, cb))
        return
      else:
        self._waiting[key] = [(kind, cb)]
        if self._send({"op": "cell", "key": list(key)}, lambda r, key=key: self._cell_done(key, r)):
          return
        waiting = self._waiting.pop(key)
    if done is not None:
      self._call(cb, done.get(kind))
      return
    for _, c in waiting:
      self._call(c, None)

  def _cell_done(self, key: tuple[int, int], result: dict | None) -> None:
    with self._lock:
      waiting = self._waiting.pop(key, [])
      if result is not None and all(v is not None for v in result.values()):
        self._cells[key] = result
        while len(self._cells) > CELL_CACHE:
          self._cells.popitem(last=False)
    for kind, cb in waiting:
      self._call(cb, None if result is None else result.get(kind))

  def request_roads(self, root: str, lat: float, lon: float, cb) -> None:
    """cb(turn_planner.LocalRoads or None)."""
    with self._lock:
      ok = self._send({"op": "roads", "root": root, "lat": lat, "lon": lon}, cb)
    if not ok:
      self._call(cb, None)

  @staticmethod
  def _call(cb, value) -> None:
    try:
      cb(value)
    except Exception:
      MapWorker._log_exception("map worker: callback failed")

  @staticmethod
  def _log_exception(msg: str) -> None:
    try:
      from openpilot.common.swaglog import cloudlog
      cloudlog.exception(msg)
    except Exception:
      pass

  @staticmethod
  def _log_event(event: str, **kw) -> None:
    try:
      from openpilot.common.swaglog import cloudlog
      cloudlog.event(event, **kw)
    except Exception:
      pass


# ---- the worker process ----

def main(argv=None) -> int:
  import argparse
  ap = argparse.ArgumentParser()
  ap.add_argument("--features")
  ap.add_argument("--roads")
  ap.add_argument("--no-download", action="store_true")
  args = ap.parse_args(argv)
  lower_priority()
  # replies go on a private copy of stdout; anything else printed goes nowhere
  out = os.fdopen(os.dup(1), "wb")
  devnull = os.open(os.devnull, os.O_WRONLY)
  os.dup2(devnull, 1)
  sys.stdout = open(os.devnull, "w")

  from openpilot.common.swaglog import cloudlog
  from openpilot.sunnypilot.selfdrive.controls.lib import turn_planner
  from openpilot.sunnypilot.selfdrive.controls.lib.map_caution import PointMap
  from openpilot.sunnypilot.selfdrive.controls.lib.speed_bump import SpeedBumpMap
  from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.road_geometry import RoadGeometry
  from openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_map import StopSignMap
  from openpilot.sunnypilot.webhud.maptiles import MapFeatures

  feats = MapFeatures(root=args.features, download=not args.no_download)
  geometry = RoadGeometry(root=args.roads)
  maps = {"stops": StopSignMap(features=feats, geometry=geometry), "bumps": SpeedBumpMap(features=feats),
          "points": PointMap(features=feats)}

  for line in sys.stdin.buffer:
    try:
      req = json.loads(line)
    except ValueError:
      continue
    result = None
    try:
      if req.get("op") == "cell":
        key = (int(req["key"][0]), int(req["key"][1]))
        path = feats.cell_path(*key)
        if path is not None and os.path.isfile(path):
          with open(path) as f:
            cell = json.load(f)
          result = {kind: m._compute(key, cell) for kind, m in maps.items()}
      elif req.get("op") == "roads":
        result = turn_planner.read_local_roads(req["root"], float(req["lat"]), float(req["lon"]))
    except Exception:
      cloudlog.exception(f"map worker: {req.get('op')} failed")
      result = None
    data = pickle.dumps((req.get("id"), result), protocol=pickle.HIGHEST_PROTOCOL)
    try:
      out.write(len(data).to_bytes(4, "little") + data)
      out.flush()
    except OSError:
      break
  return 0


if __name__ == "__main__":
  sys.exit(main())
