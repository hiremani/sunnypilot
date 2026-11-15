---
name: incident-investigation
description: Investigate a scary or unexpected moment on a drive (hard steer, unexpected brake or acceleration) - locate it, build a 100 Hz timeline, replay the responsible planner with old and fixed code, write a failing test, fix and verify against all recorded drives. Use when the user reports an incident.
---

# Incident investigation

1. **Locate it.** Ask (or infer) where and roughly when; use `drive-logs` `find_place.py` to get the segment and `logMonoTime`. If the comma is unreachable, start a background reachability loop and meanwhile read the code paths that could produce the behavior - but say clearly that this is a hypothesis until the log confirms it.
2. **Turn assist? Read its log first.** Since 2026-11-15 the turn planner logs `map_turn` (every state change, with the
   end reason) and `map_turn_tick` (every 0.5 s while armed or turning: distance, planned and turned degrees, curvature,
   blinker, torque) to logMessage in the rlog. Speed Bump Assist logs `speed_bump`, map caution `map_caution`, Stop
   Sign Assist `stop_sign`. Grep those around the time before replaying anything.
3. **Timeline.** `scripts/turn_timeline.py SEGDIR MONO_CENTER [planner.py]` prints ±40 s at 4 Hz: speed, blinkers, latActive, wheel vs commanded angle, curvature, driver torque/pressed, yaw rate, GPS, and the turn planner replayed alongside (state changes marked `<==`). Read it for the exact sequence: who was in control, when the driver acted, what the command was when the car moved.
4. **Name the mechanism** in the user's terms (2026-11-14 Doane St: the turn assist took over a turn 20 deg late, counted 91 deg from there, ignored the cancelled blinker, and when the driver let go at 15 mph the controller resumed toward a 129 deg-left target).
5. **Failing test first.** Add a unit test that reproduces the sequence; confirm it fails on the old code (positive control: restore the old file from `git show HEAD:<path>`, run, then put the fix back).
6. **Fix in layers** where it is safety-relevant: the planner that produced the bad target, and the actuator layer that let it through (for example the "no snap-back on release" latch in the Fisker car controller).
7. **Replay every recorded drive** with old and new code: `scripts/turn_replay.py PLANNER.py ROUTE_PREFIX...` prints each planner turn with how and why it ended. A fix that looks right on one drive can break others: the first blinker-off rule stuttered on older drives whose blinker signal flashes (fixed with a 1 s debounce and a half-done condition). Don't deploy until this replay is clean.
8. Commit with the route, time and mechanism in the message; deploy (`comma-deploy`); tell the user what to watch for on the next drive and any interim workaround (e.g. keep Map Turn Assist off).

The scripts read `/data/media/0/realdata` and the map tiles on the device; copy them to `/data/tmp/` and run detached.
