---
name: drive-review
description: Review one or more drives - engagements, alerts, events, lag and CPU, openpilot longitudinal tracking, driver overrides and following gaps, planner sources and profile features. Use after the user drives, or when asked "what can we learn from the logs".
---

# Drive review

Scripts (run on the device, detached; see `comma-device`):

| Script | Args | What it tells you |
|---|---|---|
| `drive_review.py` | `ROOT ROUTE...` | miles, steering/long engagements and how each ended, accel tracking error, plan sources (incl. `junctionTurn`, `driverSpeed`, `stopSign`, `sccVision`), narrow-street time, events with first times, alerts, error log messages |
| `lag.py` | `ROOT ROUTE` | `selfdrivedLagging`/`commIssue` clusters with CPU/temperature around each, top processes by CPU |
| `procs.py` | `ROOT ROUTE SEG_FROM SEG_TO` | CPU per process by command line (card vs selfdrived vs controlsd...) |
| `overrides.py` | `ROOT ROUTE...` | gas/brake takeovers while openpilot had speed control (with lead and command), steady-lead time gaps openpilot vs driver |

```
scp .claude/skills/drive-review/scripts/*.py comma@192.168.0.207:/data/tmp/
# ROOT on the device is /data/media/0/realdata; ROUTE is <counter>--<hash>
```

## What to look for

- **Lag**: card, selfdrived and controlsd share CPU core 4. Fisker card ran 34-41% of it (SecOC signing of 11-22 candidates per tick); core 4 near 90% busy gives `selfdrivedLagging`/`commIssue` and then an LKAS fault. Don't move card to another core (core 5 has ui, core 6 is isolcpus for camerad); reduce card's work instead.
- **Long engagements** that end with the driver braking: run `overrides.py` and look at the lead and the command in the 2 s before.
- **Plan sources**: which limit was setting the speed; `driverSpeed`/`junctionTurn`/`leadCoast` show the driver profile at work.
- **Startup**: "Cruise Fault: Restart the car to engage" (OEM `accFaulted`) at drive start has been seen before; note it, don't chase it unless it repeats mid-drive.
- **Steering never actuating**: see `secoc-key`.

Report in plain words first (what happened, what it means), numbers second. Separate what the logs show from what you infer.
