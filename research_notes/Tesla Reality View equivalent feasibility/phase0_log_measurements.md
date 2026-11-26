# Phase 0: what the drive logs say (measured 2026-11-26)

Source: `SP/openpilot/sunnypilot/tools/rv_phase0.py`, run on a Mac over the rlogs in `~/Fisker/routes`. These are
first-hand measurements, not web or code claims. The raw output (JSON and markdown) is not in git; rerun the tool to
get it again:

```
cd openpilot && PYTHONPATH=..:../opendbc_repo:. ../.venv/bin/python -I sunnypilot/tools/rv_phase0.py --segments 6 --json out.json --md out.md
```

## Method

- Drives: every route folder in `~/Fisker/routes` (routes 000000dd, from 2026-11-13, to 00000151, the latest on the Mac), less the
  catalog's drives that did not move. 89 drives.
- Segments: 6 one-minute rlogs per drive, spread from first to last; 510 segments in all. Time by speed: parked
  192 min, town (under 35 mph) 183 min, main road (35-50 mph) 51 min, highway (over 50 mph) 32 min.
- OEM object list: ADAS_Obj1..8 (`0x33B 0x34B 0x32D 0x33D 0x34D 0x32F 0x33F 0x34F`) read on panda bus 2, the ADAS
  module's own side of the harness. Decoded by hand at the DBC's 0.2 m/bit (`OS/dbc/fisker_ocean_adas_world.dbc`); the
  decoder agreed with the opendbc DBC parser on all 158 non-empty frames of a check segment. A snapshot of all eight
  slots is taken each time slot 1 arrives (every 40 ms).
- "Behind": longitudinal distance below 0. "Beside": within 5 m ahead or behind and more than 1 m to the side.
- Tracks: one object ID seen with no gap over 1 s, while moving; tracks shorter than 0.2 s are dropped as flicker.
- Blind spot: `carState.leftBlindspot` / `rightBlindspot` while moving; a frame counts when the list holds an object on
  that side (more than 1 m out) no more than 2 m ahead.
- Radar: frames on panda bus 1 at the radar's addresses (0x300-0x32F), and the points in `radarTracks`.
- Model and device: `modelV2.modelExecutionTime` and `frameDropPerc`; `deviceState` CPU (mean over cores), GPU and
  memory. Split by whether `procLog` shows the stream encoder (`encoderd --stream`) in the segment.
- Leads: `radarState.leadOne.radar` while `leadOne.present`.

## Numbers

### 1. The OEM object list does not see beside or behind

| Speed | Object frames | Objects per snapshot | Behind | Beside | Ahead p5 / p50 / p95 / max | Lateral p50 / p95 |
|---|---|---|---|---|---|---|
| Town | 323,728 | 1.18 | 0.0% | 7.5% | 4.0 / 18.8 / 37.4 / 40.6 m | 4.6 / 7.2 m |
| Main road | 52,413 | 0.68 | 0.0% | 2.7% | 7.2 / 25.6 / 38.8 / 40.6 m | 3.4 / 6.4 m |
| Highway | 47,408 | 0.98 | 0.0% | 4.5% | 5.4 / 24.8 / 38.8 / 40.6 m | 4.6 / 6.6 m |
| Parked | 178,940 | 0.59 | 0.0% | 15.9% | 2.2 / 8.2 / 34.0 / 40.6 m | 4.4 / 7.2 m |

- The longitudinal sign bit was never set: not one object frame out of 602,489 is behind the car.
- "Beside" objects are all just ahead of the bumper (0-5 m), not alongside or behind.
- Slots: 57% of snapshots hold no object; 1: 20%, 2: 11%, 3: 7%, 4 or more: 5%. All eight are full in 0.12%. The
  slot count is not a limit in practice.
- Range: 40.6 m (raw 203) is the most ever decoded. With the HUD's range correction (about 0.25 m/bit from the rear
  axle, `OS/INTERNALS.md:474`) that is about 51 m. Either way, nothing farther is ever listed.
- Tracks (moving): 3,666. None starts behind; 6% start within 5 m ahead. Lifetime p50 2.5 s, p90 10 s, p99 40 s.
  Tracks start at p50 28 m and end at p50 13 m; 5% end within 1.2 m ahead. No track that started ahead ever
  continued behind the car: the list drops a car as it comes alongside.
- Blind spot: the flag was on in 25,775 moving snapshots (about 17 minutes). The list held a car on that side in
  0.27% of them. The car's corner radars do not feed the list.

### 2. The radar sends nothing

- Panda bus 1 frames at the radar's addresses: 0, in every one of the 510 segments (and in full rlogs checked from
  the first drive on record to the last).
- `radarTracks`: 545,362 messages, 0 points.
- `carParams.radarUnavailable` is true.
- So the radar's private link is not reaching the comma on any recorded drive. The 32-slot radar feed the HUD and the
  report describe comes from code and from one 61 s capture, not from these drives. Why it is silent (harness,
  connector, radar power or wake-up) is unknown.

### 3. Driving model and device load (all segments video-off)

- Model execution: p5 25.9 ms, p50 27.5 ms, p95 30.1 ms, p99 30.6 ms (400,000 frames; the first ones after start-up
  reach 2.2 s). The frame budget is 50 ms.
- Frame drops: `frameDropPerc` is 0 in 97.7% of model frames; the largest value is 3.8%.
- Device CPU (mean of cores): p50 58%, p95 68%, p99 77%, max 90%.
- Memory: p50 68%, p95 73%, max 96%.
- `gpuUsagePercent` reads 2 or 3 in every sample, so it says nothing about GPU headroom.
- No segment had the stream encoder running, so the cost of video-on cannot be read from these logs.

### 4. Leads are vision-only

- A lead was present in 216,510 `radarState` frames; 0 were radar-backed.

### 5. Track memory

- Nothing re-detects a car after it passes: the list loses it alongside and never sees behind. Any car the HUD draws
  beside or behind is memory (dead reckoning) only, and the logs cannot check how good that memory is.

## What this changes in the plan

- Phase 4: the rear needs added sensing for any real content. The OEM list will not provide it.
- Before buying anything, find out why the radar link is silent; fixing it is free and gives range beyond ~51 m.
- The comma-side video gate now has its baseline (above). It still needs one drive with the camera view on.
- The HUD's provenance styling matters more than assumed: every car beside or behind is remembered, not seen.

## Caveats

- Lists are capped at 400,000 values each, so the model-time percentiles come from the earlier drives in the run
  (the segments are processed in route order). Counts and shares use every frame.
- 32 minutes of highway driving is thin; range and slot numbers at speed rest on it.
- Object frames count each object once per 40 ms snapshot, so slow-moving objects weigh more.
- The blind-spot match uses a 1 m side offset and +2 m forward limit; a car slightly ahead of the mirror can still
  count. The share is so low that the choice does not change the conclusion.
