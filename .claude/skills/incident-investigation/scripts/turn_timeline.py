"""timeline around a junction pass, with the turn planner replayed. usage: doane_tl.py SEGDIR MONO_CENTER [planner_module_path]"""
import importlib.util, math, sys
from openpilot.tools.lib.logreader import LogReader
seg, center = sys.argv[1], int(sys.argv[2])
tp_path = sys.argv[3] if len(sys.argv) > 3 else "/data/openpilot/openpilot/sunnypilot/selfdrive/controls/lib/turn_planner.py"
spec = importlib.util.spec_from_file_location("tpx", tp_path); tp = importlib.util.module_from_spec(spec); spec.loader.exec_module(tp)
P = tp.TurnPlanner(road_source=tp.RoadSource(root="/data/media/0/osm/offline", sync=True))
msgs = []
import os
base = seg.rsplit("--", 1)[0]; n = int(seg.rsplit("--", 1)[1])
for k in (n - 1, n, n + 1):
  f = f"{base}--{k}/rlog.zst"
  if os.path.exists(f):
    msgs += list(LogReader(f))
msgs.sort(key=lambda m: m.logMonoTime)
cs = cc = gps = ctl = None; last_ctl_t = None; last_print = 0
prev_state = None
for m in msgs:
  t = (m.logMonoTime - center) * 1e-9
  w = m.which()
  if w == "carState": cs = m.carState
  elif w == "carControl": cc = m.carControl
  elif w in ("gpsLocationExternal",): gps = m.gpsLocationExternal
  elif w == "controlsState": ctl = m.controlsState
  elif w == "selfdriveState" and -40 < t < 40 and m.selfdriveState.alertText1:
    pass
  if w != "carControl" or cs is None or abs(t) > 40:
    continue
  dt = 0.01 if last_ctl_t is None else t - last_ctl_t
  last_ctl_t = t
  out = P.update(dt, cs.vEgo, cs.yawRate, cs.leftBlinker, cs.rightBlinker, cc.latActive, cs.steeringPressed, gps, cs.steeringTorque)
  ev = ""
  if P.state != prev_state:
    ev = f"  <== planner {prev_state}->{P.state} {P.log.reason} dist={P.log.junction_dist:.1f} planned={P.log.planned_deg:.0f}"
    prev_state = P.state
  if t - last_print >= 0.25 or ev:
    last_print = t
    a = cc.actuators
    g = f"gps {gps.latitude:.6f},{gps.longitude:.6f} b{gps.bearingDeg:4.0f} acc{gps.horizontalAccuracy:.0f}" if gps else ""
    print(f"{t:6.2f} v{cs.vEgo*2.237:4.1f}mph L{int(cs.leftBlinker)}R{int(cs.rightBlinker)} lat{int(cc.latActive)} "
          f"wheel{cs.steeringAngleDeg:7.1f} cmd{a.steeringAngleDeg:7.1f} curv{a.curvature:+.3f} tq{cs.steeringTorque:+5.1f} "
          f"pr{int(cs.steeringPressed)} yaw{math.degrees(cs.yawRate):+5.0f} tp={P.state[:4]} out={'' if out is None else f'{out:+.3f}'} {g}{ev}")
