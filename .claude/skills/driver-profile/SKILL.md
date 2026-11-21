---
name: driver-profile
description: Rebuild, compare and install the driver profile (me.json) that tunes openpilot to the driver's own habits - following gap, speed by limit, curve and junction speeds, lead coasting, turn starts, stop braking. Use when new drives exist or the user asks what to add to or change in the profile.
---

# Driver profile

- Values and bounds: `openpilot/sunnypilot/selfdrive/controls/lib/driver_profile.py` (`DriverProfile`, `BOUNDS`). Generator: `openpilot/sunnypilot/tools/driver_profile_gen.py` (uses `fisker_drive_style.py`, `turn_habits.py`, narrow-street replay).
- Files: device `/data/media/0/driver_profiles/<name>.json`, Mac `~/.comma/media/0/driver_profiles/`. Params `DriverProfile` (on/off), `DriverProfileName`, `DriverProfileSpeed`. The web HUD switches profiles (`/api/profiles`).
- Read live: plannerd about every 5 s, controlsd's turn assist about every 10 s. Installing a new file needs no reboot.

## Rebuild (on the device, detached; it reads every rlog and takes 30-60 min)

```
R=$(ls /data/media/0/realdata | grep -E "^000000(dd|e5|...)--" | sed "s/--[0-9]*$//" | sort -u | tr "\n" " ")
nice /usr/local/venv/bin/python openpilot/sunnypilot/tools/driver_profile_gen.py $R --out /data/tmp/me_new.json
```
Flags: `--dry-run`, `--no-narrow`, `--no-turns` (skip the slow rlog passes).

## Rules of measurement

Only what the driver chose counts: speed habits with ACC off and openpilot not in control; curves and turns only where the driver steered; stops the driver braked for. Each value needs a minimum sample count, otherwise it keeps the default; every value is clamped to `BOUNDS`. The meta block records measured value, samples and note - show it.

## Install

1. Show the user old vs new values as a table, with what each change will do on the road.
2. Install with versions (keeps what it replaces; the last 10 stay switchable from the HUD, Settings → Driving → Versions):
   `/usr/local/venv/bin/python -m openpilot.sunnypilot.selfdrive.controls.lib.profile_versions install /data/tmp/me_new.json --label "<what changed>"`
   (`list`, `restore ID`, `snapshot`, `import-backups` for the old `.me.json.bak-*`; `--kind personal` for the personal
   model). Then check `driver_profile.active()` returns the new values. Never `cp` over me.json: that skips the version.
   Compare candidates on the Mac first: `sunnypilot/tools/profile_compare.py me@0 new=/path/new.json --last 10 [--pull]`.
3. Copy to the Mac for drive_sim.

## Adding a new profile value

Measure it from the logs first and look at the distribution; if the median equals what the car already does (e.g. turn-exit pull-away 0.99 vs the 1.0 cap), say so. A new value needs: field + bound in `driver_profile.py`, a `put(...)` in the generator with a minimum sample count, a live consumer that is off at the default, and tests.
