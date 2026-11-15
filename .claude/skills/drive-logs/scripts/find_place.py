"""Which segments of these routes pass within RADIUS m of a place. usage: find_place.py LAT LON RADIUS ROUTE_PREFIX...
Runs on the comma (qlogs). Prints the closest approach, its logMonoTime (feed it to turn_timeline.py), bearing and speed."""
import glob, math, os, sys
from openpilot.tools.lib.logreader import LogReader
root = "/data/media/0/realdata"
LAT, LON, RADIUS = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
for r in sys.argv[4:]:
  for s in sorted(glob.glob(f"{root}/{r}--*"), key=lambda p: int(p.rsplit("--",1)[1])):
    f = os.path.join(s, "qlog.zst")
    if not os.path.exists(f): continue
    best = None
    try:
      for m in LogReader(f):
        if m.which() in ("gpsLocationExternal", "gpsLocation"):
          g = getattr(m, m.which())
          d = math.hypot((g.latitude-LAT)*111000, (g.longitude-LON)*111000*0.793)
          if best is None or d < best[0]:
            best = (d, m.logMonoTime, g.bearingDeg, g.speed)
    except Exception as e:
      print(s, "err", e); continue
    if best and best[0] < RADIUS:
      print(os.path.basename(s), f"closest {best[0]:.0f} m mono {best[1]} bearing {best[2]:.0f} v {best[3]:.1f}")
