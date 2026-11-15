---
name: longitudinal-sim
description: Simulate openpilot longitudinal behavior (following gap, approach to a slower lead, profile on vs off) with the real planner in the longitudinal plant, and replay recorded drives with drive_sim. Use to check a speed-control change before it reaches the car.
---

# Longitudinal simulation

## Plant (on the device)

The long MPC's acados solver fails on the Mac, so run the plant on the comma. `scripts/plant_profile.py` shows the pattern:
- stub `StopSignAssist` and `SpeedLimitResolver._get_from_map_data` (the plant has no GPS),
- patch `driver_profile.active` / `is_on` to run with the profile off and on,
- `Maneuver(...).evaluate()`; the plant's `cruise_values` are in **m/s**,
- read gap = lead position - ego position, divide by speed for the time gap, and peak acceleration in the first seconds of an approach.

Edit the profile values and scenarios at the top, copy to `/data/tmp/`, run with the device venv and PYTHONPATH (`comma-device`).

## drive_sim (on the Mac)

`openpilot/sunnypilot/tools/drive_sim/` replays a recorded drive. Its "Your style" mode drives the route by the car's own profile rules (`profile_long.py`: MPC gap formula, lead coasting, junction speed, per-limit offsets) and reports what set the speed; compare trip time and mean speed difference against the recording. Narrow-streets mode replays the narrow-street detector and center-line keeper. Turn evaluation compares the turn planner's ghost path with the recorded turns.
