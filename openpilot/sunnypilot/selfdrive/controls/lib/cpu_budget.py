"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

A CPU budget for the planner's optional extras (shadow loggers, map caution).

plannerd has 50 ms a tick and shares core 5 with the UI. An extra that takes too long, once (SPIKE_MS) or on average
(AVG_MS, an exponential average), is paused for PAUSE_S and the pause is logged ("cpu_budget" event): the drive keeps a
planner on time rather than an extra. An extra whose output can steer the speed must be given an on_pause that leaves it
neutral (no target, no cap). Pauses double up to MAX_PAUSE_S while it keeps going over, and start short again after
CALM_S without one.
"""
import time

SPIKE_MS = 15.0
AVG_MS = 3.0
AVG_ALPHA = 0.05
PAUSE_S = 30.0
MAX_PAUSE_S = 600.0
CALM_S = 1200.0       # s without a pause that reset the pause length


class CpuBudget:
  def __init__(self, name: str, spike_ms: float = SPIKE_MS, avg_ms: float = AVG_MS, on_pause=None, clock=time.monotonic,
               timer=time.perf_counter, log=None):
    self.name = name
    self.spike_ms = spike_ms
    self.avg_ms = avg_ms
    self.on_pause = on_pause
    self.clock = clock
    self.timer = timer
    self.log = log
    self.avg = 0.0
    self.paused_until = -float("inf")
    self.pause_s = PAUSE_S
    self.pauses = 0

  @property
  def paused(self) -> bool:
    return self.clock() < self.paused_until

  def run(self, fn, *args) -> bool:
    """fn(*args) unless paused; False when it was skipped."""
    if self.paused:
      return False
    t = self.timer()
    try:
      fn(*args)
    finally:
      ms = (self.timer() - t) * 1000.0
      self.avg += AVG_ALPHA * (ms - self.avg)
      if ms > self.spike_ms or self.avg > self.avg_ms:
        self._pause(ms)
    return True

  def _pause(self, ms: float) -> None:
    now = self.clock()
    if now - self.paused_until > CALM_S:
      self.pause_s = PAUSE_S   # well behaved for a long while: start short again
    self.paused_until = now + self.pause_s
    self.pauses += 1
    self._emit(ms)
    self.pause_s = min(self.pause_s * 2, MAX_PAUSE_S)
    self.avg = 0.0
    if self.on_pause is not None:
      try:
        self.on_pause()
      except Exception:
        pass

  def _emit(self, ms: float) -> None:
    try:
      if self.log is not None:
        self.log(self.name, ms, self.avg)
      else:
        from openpilot.common.swaglog import cloudlog
        cloudlog.event("cpu_budget", feature=self.name, ms=round(ms, 1), avg_ms=round(self.avg, 2), pause_s=self.pause_s,
                       pauses=self.pauses)
    except Exception:
      pass
