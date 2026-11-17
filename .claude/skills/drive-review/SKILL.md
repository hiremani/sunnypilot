---
name: drive-review
description: Review one or more drives - engagements, alerts, events, lag and CPU, openpilot longitudinal tracking, driver overrides and following gaps, planner sources and profile features. Use after the user drives, or when asked "what can we learn from the logs".
---

# Drive review

**Start with the automatic summary.** Since 2026-11-15 the comma writes one per drive once the car is off
(`openpilot/sunnypilot/tools/drive_summary.py`, `DriveSummary` param): `/data/media/0/drive_summaries/<route>.json`,
also `GET http://192.168.0.207:8088/api/drives` (headlines) and `/api/drives?route=NAME` (full). It covers steering
and speed-control time, how each speed-control run ended, alerts, lag, turn-assist turns, stop sign and speed bump
passes, and error lines. Use the scripts below for what it doesn't cover.

Scripts (run on the device, detached; see `comma-device`):

| Script | Args | What it tells you |
|---|---|---|
| `drive_review.py` | `ROOT ROUTE...` | miles, steering/long engagements and how each ended, accel tracking error, plan sources (incl. `junctionTurn`, `driverSpeed`, `stopSign`, `sccVision`), narrow-street time, events with first times, alerts, error log messages |
| `lag.py` | `ROOT ROUTE` | `selfdrivedLagging`/`commIssue` clusters with CPU/temperature around each, top processes by CPU |
| `procs.py` | `ROOT ROUTE SEG_FROM SEG_TO` | CPU per process by command line (card vs selfdrived vs controlsd...) |
| `overrides.py` | `ROOT ROUTE...` | gas/brake takeovers while openpilot had speed control (with lead and command), steady-lead time gaps openpilot vs driver |
| `stops.py` | `ROOT ROUTE... [--segs 12,14]` | every stop to standstill: who stopped (op-long / car-ACC / driver), peak decel, the jerk in the last 0.5 s (the "head nod"), the decel at the stop, op-long request error, how far it rolled after, ACC states from `fisker_long_handover` |
| `replay_carcontroller.py` | `ROOT ROUTE --segs 4,5 [--from-min --to-min] [--long]` (Mac or device, from `openpilot/`) | runs today's Fisker CarState + CarController over a drive's recorded CAN and controls, and prints every steering / speed-control handover change with the latch that caused it. Use it to answer "why did it stop steering here?" for any drive, including ones recorded before the handover log existed. eps_sts in its output is what the car reported at the time (history), the rest is today's code |

```
scp .claude/skills/drive-review/scripts/*.py comma@192.168.0.207:/data/tmp/
# ROOT on the device is /data/media/0/realdata; ROUTE is <counter>--<hash>
```

## What to look for

- **Turn assist not really steering** (driver steering the turn): replay the turn with `replay_carcontroller.py`.
  Recorded turns (f3, f5, 2026-11-17 replay) showed our own 3 Nm torque latch (`intervening,torque_latch_only`)
  tripping on hand torque mid-turn, mostly against the turn (a hand dragged by the wheel), then never releasing
  (holding a turned wheel takes ~2 Nm, the release needs < 0.5 Nm). The EPS itself stayed active whenever we
  requested. Check whether that still happens with the slower wheel ramp (4d336fe82) before changing the threshold.

- **Lag**: card, selfdrived and controlsd share CPU core 4. Fisker card ran 34-41% of it (SecOC signing of 11-22 candidates per tick); core 4 near 90% busy gives `selfdrivedLagging`/`commIssue` and then an LKAS fault. Don't move card to another core (core 5 has ui, core 6 is isolcpus for camerad); reduce card's work instead.
- **Long engagements** that end with the driver braking: run `overrides.py` and look at the lead and the command in the 2 s before.
- **Plan sources**: which limit was setting the speed; `driverSpeed`/`junctionTurn`/`leadCoast` show the driver profile at work.
- **Startup**: "Cruise Fault: Restart the car to engage" (OEM `accFaulted`) at drive start has been seen before; note it, don't chase it unless it repeats mid-drive.
- **Steering never actuating**: see `secoc-key`.

Report in plain words first (what happened, what it means), numbers second. Separate what the logs show from what you infer.
