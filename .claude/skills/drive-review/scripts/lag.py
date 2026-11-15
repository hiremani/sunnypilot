import sys, glob
from collections import defaultdict
from openpilot.tools.lib.logreader import LogReader
root, r = sys.argv[1], sys.argv[2]
fs = sorted(glob.glob(f"{root}/{r}--*/rlog.zst"), key=lambda p: int(p.split("--")[-1].split("/")[0]))
t0=None; lag=[]; dev=[]; procs=[]; ctl=[]
for f in fs:
  for m in LogReader(f):
    t=m.logMonoTime*1e-9
    if t0 is None: t0=t
    w=m.which()
    if w=="onroadEvents":
      if any(str(e.name) in ("selfdrivedLagging","commIssue") for e in m.onroadEvents): lag.append(t-t0)
    elif w=="deviceState":
      d=m.deviceState
      dev.append((t-t0, max(d.cpuUsagePercent) if len(d.cpuUsagePercent) else 0, sum(d.cpuUsagePercent)/max(1,len(d.cpuUsagePercent)), d.maxTempC, d.memoryUsagePercent, str(d.thermalStatus), d.gpuUsagePercent))
    elif w=="procLog":
      procs.append((t-t0, [(p.name, p.cpuUser+p.cpuSystem) for p in m.procLog.procs]))
    elif w=="controlsState":
      pass
# lag clusters
cl=[]
for x in lag:
  if not cl or x-cl[-1][1]>3: cl.append([x,x])
  else: cl[-1][1]=x
print("lag clusters (min):", [(round(a/60,2), round(b-a,1)) for a,b in cl])
import statistics
print("deviceState overall: cpu avg %.0f%%  max-core p90 %.0f  temp med %.0f max %.0f  mem %.0f%%" % (statistics.mean(d[2] for d in dev), sorted(d[1] for d in dev)[len(dev)*9//10], statistics.median(d[3] for d in dev), max(d[3] for d in dev), statistics.mean(d[4] for d in dev)))
for a,b in cl:
  near=[d for d in dev if a-3<=d[0]<=b+1]
  print(f"  lag at {a/60:.2f}: " + " | ".join(f"t{d[0]-a:+.0f} cpu{d[2]:.0f}/max{d[1]:.0f} {d[3]:.0f}C gpu{d[6]}" for d in near[:6]))
# per-process cpu rate: diff procLog between consecutive samples
def rates(i):
  (ta,pa),(tb,pb)=procs[i],procs[i+1]
  A=defaultdict(float); B=defaultdict(float)
  for n,c in pa: A[n]+=c
  for n,c in pb: B[n]+=c
  return {n:(B[n]-A.get(n,0))/(tb-ta)*100 for n in B}
tot=defaultdict(list)
for i in range(len(procs)-1):
  for n,v in rates(i).items(): tot[n].append(v)
print("top processes, avg % of one core:")
for n,v in sorted(tot.items(), key=lambda kv:-statistics.mean(kv[1]))[:14]:
  print(f"   {n[:40]:40s} {statistics.mean(v):6.1f}  max {max(v):6.1f}")
for a,b in cl[:8]:
  idx=[i for i in range(len(procs)-1) if a-4<=procs[i][0]<=a+1]
  if idx:
    rr=rates(idx[-1])
    print(f"  at {a/60:.2f} top:", ", ".join(f"{n[:22]} {v:.0f}" for n,v in sorted(rr.items(), key=lambda kv:-kv[1])[:6]))
