import sys, glob, statistics
from collections import defaultdict
from openpilot.tools.lib.logreader import LogReader
root, r = sys.argv[1], sys.argv[2]
fs = sorted(glob.glob(f"{root}/{r}--*/rlog.zst"), key=lambda p: int(p.split("--")[-1].split("/")[0]))[int(sys.argv[3]):int(sys.argv[4])]
snaps=[]
for f in fs:
  for m in LogReader(f):
    if m.which()=="procLog":
      snaps.append((m.logMonoTime*1e-9, {p.pid:(" ".join(p.cmdline)[-45:] or p.name, p.cpuUser+p.cpuSystem, p.numThreads) for p in m.procLog.procs}))
rate=defaultdict(list)
for (ta,a),(tb,b) in zip(snaps,snaps[1:]):
  for pid,(n,c,th) in b.items():
    if pid in a: rate[n].append((c-a[pid][1])/(tb-ta)*100)
for n,v in sorted(rate.items(), key=lambda kv:-statistics.mean(kv[1]))[:16]:
  print(f"{n:45s} avg {statistics.mean(v):6.1f}  p90 {sorted(v)[len(v)*9//10]:6.1f}")
