"""Replay the Fisker car code (CarState + CarController, as checked out now) over a drive's recorded CAN and controls
messages, and print the steering / speed-control handover lines it logs (fisker_lat_handover, fisker_long_handover).

It answers "why did our controller stop steering (or stop commanding speed) here?" for any recorded drive, with
today's code, before a new drive. The SecOC key is a dummy: nothing is checked against the car.

Usage (Mac or device, from openpilot/, PYTHONPATH=.:../opendbc_repo):
  replay_carcontroller.py ROOT ROUTE --segs 0,1 [--from-min 1.0 --to-min 2.0] [--long]
"""
import argparse
import glob
import logging
import os

from opendbc.car.carlog import carlog
from opendbc.car.can_definitions import CanData
from opendbc.car.fisker.interface import CarInterface
from opendbc.car.fisker.values import CAR
from openpilot.selfdrive.car.helpers import LazyCarControlSP
from openpilot.tools.lib.logreader import LogReader


class Grab(logging.Handler):
  def __init__(self):
    super().__init__()
    self.lines = []
    self.t = 0.0

  def emit(self, record):
    if isinstance(record.msg, dict) and str(record.msg.get("event", "")).startswith("fisker_"):
      self.lines.append((self.t, getattr(self, 'blink', '-'), record.msg))


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("root")
  ap.add_argument("route")
  ap.add_argument("--segs", required=True)
  ap.add_argument("--from-min", type=float, default=0.0)
  ap.add_argument("--to-min", type=float, default=1e9)
  ap.add_argument("--long", action="store_true", help="openpilot longitudinal (alpha) as on the device")
  args = ap.parse_args()

  cp = CarInterface.get_non_essential_params(CAR.FISKER_OCEAN)
  cp.secOcKeyAvailable = True
  cp.openpilotLongitudinalControl = args.long
  cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.FISKER_OCEAN)
  ci = CarInterface(cp, cp_sp)
  ci.CC.secoc_key_verified = True
  ci.CC.secoc_key = bytes(16)
  grab = Grab()
  for h in list(carlog.handlers):   # carlog prints every line itself; keep only ours
    carlog.removeHandler(h)
  carlog.addHandler(grab)

  segs = [int(s) for s in args.segs.split(",")]
  cc = cc_sp = None
  for n in segs:
    f = [*glob.glob(os.path.join(args.root, f"{args.route}--{n}", "rlog*"))][0]
    t0 = None   # each segment is one minute: time it from its own first carState (earlier messages can be stale)
    for m in LogReader(f):
      w = m.which()
      if t0 is None and w == "carState":
        t0 = m.logMonoTime
      tmin = n + ((m.logMonoTime - t0) / 6e10 if t0 is not None else 0.0)
      if w == "carControl":
        cc = m.carControl
      elif w == "carControlSP":
        cc_sp = m.carControlSP
      elif w == "can":
        pkts = [CanData(c.address, bytes(c.dat), c.src) for c in m.can]
        ci.update([(m.logMonoTime, pkts)])
        if cc is not None and cc_sp is not None:
          grab.t = tmin
          grab.blink = 'L' if ci.CS.out.leftBlinker else 'R' if ci.CS.out.rightBlinker else '-'
          ci.apply(cc, LazyCarControlSP(cc_sp), m.logMonoTime)
  keys = ("tx", "lat_active", "intervening", "torque_latch_only", "angle_latch", "eps_recovery", "coop")
  for t, blink, msg in grab.lines:
    if args.from_min <= t <= args.to_min:
      if msg["event"] == "fisker_lat_handover":
        on = ",".join(k for k in keys if msg.get(k))
        print(f"{t:7.3f} min {blink} lat  [{on}] eps {msg['eps_sts']} wheel {msg['wheel']:+7.1f} want {msg['want']:+7.1f} "
              f"tq {msg['torque']:+5.2f} {msg['v'] * 2.237:4.1f} mph")
      else:
        print(f"{t:7.3f} min {blink} long {msg}")


if __name__ == "__main__":
  main()
