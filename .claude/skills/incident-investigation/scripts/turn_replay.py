"""replay the turn planner over whole routes; print each planner turn. usage: tp_route.py MODULE ROUTE..."""
import glob, importlib.util, math, os, sys
from openpilot.tools.lib.logreader import LogReader
spec = importlib.util.spec_from_file_location("tpx", sys.argv[1]); tp = importlib.util.module_from_spec(spec); spec.loader.exec_module(tp)
for r in sys.argv[2:]:
  P = tp.TurnPlanner(road_source=tp.RoadSource(root="/data/media/0/osm/offline", sync=True))
  cs = gps = None; last = None; prev = None; t0 = None; tstart = None; h0 = None
  for s in sorted(glob.glob(f"/data/media/0/realdata/{r}--*"), key=lambda p: int(p.rsplit("--", 1)[1])):
    f = os.path.join(s, "rlog.zst")
    if not os.path.exists(f): continue
    for m in LogReader(f):
      w = m.which(); t = m.logMonoTime * 1e-9
      t0 = t0 or t
      if w == "carState": cs = m.carState
      elif w == "gpsLocationExternal": gps = m.gpsLocationExternal
      elif w == "carControl" and cs is not None:
        dt = 0.01 if last is None else t - last; last = t
        P.update(dt, cs.vEgo, cs.yawRate, cs.leftBlinker, cs.rightBlinker, m.carControl.latActive, cs.steeringPressed, gps, cs.steeringTorque)
        if P.state != prev:
          if P.state == "turning":
            tstart, h0 = t, P.heading
          if prev == "turning":
            print(f"{r} t={tstart - t0:7.1f}s side={P.side or '-':5s} turned {P.log.progress_deg:5.0f}/{abs(P.log.planned_deg) if P.log.planned_deg else 0:3.0f} "
                  f"{t - tstart:4.1f}s end v={cs.vEgo*2.237:4.1f}mph wheel={cs.steeringAngleDeg:6.0f} reason={P.log.reason}")
          prev = P.state
