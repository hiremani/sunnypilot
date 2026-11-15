import sys, glob, statistics
from openpilot.tools.lib.logreader import LogReader
MPH=2.23694
root=sys.argv[1]
for r in sys.argv[2:]:
  fs = sorted(glob.glob(f"{root}/{r}--*/rlog.zst"), key=lambda p: int(p.split("--")[-1].split("/")[0]))
  t0=None; S={}; hist=[]; gaps_op=[]; gaps_me=[]; lo_prev=False; gas_prev=False; out=[]
  setspd=None
  for f in fs:
    for m in LogReader(f):
      t=m.logMonoTime*1e-9
      if t0 is None: t0=t
      w=m.which()
      if w=="radarState": S["rs"]=m.radarState
      elif w=="longitudinalPlan": S["lp"]=m.longitudinalPlan
      elif w=="carState": S["cs"]=m.carState
      elif w=="carControl" and "cs" in S:
        cc=m.carControl; cs=S["cs"]; rs=S.get("rs")
        d=rs.leadOne.dRel if rs and rs.leadOne.dRel>0 else 0; vl=cs.vEgo+(rs.leadOne.vRel if d else 0)
        hist.append((t-t0, cs.vEgo, cc.actuators.accel, cs.aEgo, d, vl, cs.cruiseState.speed))
        hist=hist[-300:]
        if cs.vEgo>8 and 0<d<80 and abs(vl-cs.vEgo)<1.5:
          (gaps_op if cc.longActive else gaps_me if not cs.cruiseState.enabled else []).append(d/cs.vEgo)
        engaged=cs.cruiseState.enabled and S.get("en",True)
        # gas press while openpilot long was active just before
        if cs.gasPressed and not gas_prev and lo_prev:
          past=[h for h in hist if h[0]>=t-t0-2]
          out.append(("GAS", t-t0, cs.vEgo, statistics.mean(h[2] for h in past), d, vl, cs.cruiseState.speed))
        if cs.brakePressed and lo_prev:
          past=[h for h in hist if h[0]>=t-t0-2]
          ttc = d/max(0.1, cs.vEgo-vl) if d and cs.vEgo>vl else 99
          out.append(("BRAKE", t-t0, cs.vEgo, statistics.mean(h[2] for h in past), d, vl, ttc))
        lo_prev=cc.longActive; gas_prev=cs.gasPressed
  print(f"== {r}")
  for k in out:
    if k[0]=="GAS": print(f"  GAS   {k[1]/60:6.2f}m v={k[2]*MPH:4.1f} mph  cmd(2s)={k[3]:+.2f}  lead={k[4]:4.0f}m @{k[5]*MPH:4.1f}  set={k[6]*MPH:4.1f}")
    else: print(f"  BRAKE {k[1]/60:6.2f}m v={k[2]*MPH:4.1f} mph  cmd(2s)={k[3]:+.2f}  lead={k[4]:4.0f}m @{k[5]*MPH:4.1f}  ttc={k[6]:.1f}s")
  for n,g in (("openpilot", gaps_op), ("you, ACC off", gaps_me)):
    if g: print(f"  steady-lead time gap, {n}: median {statistics.median(g):.2f}s  p25 {sorted(g)[len(g)//4]:.2f}  n={len(g)}")
