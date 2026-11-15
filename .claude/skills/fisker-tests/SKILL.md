---
name: fisker-tests
description: Run the right tests and lint for the Fisker port (opendbc car code and sunnypilot controls/tools) on the Mac, and prove a new test actually catches the bug. Use before any commit or deploy.
---

# Tests

The Mac venvs have no pytest; use unittest.

- opendbc (car code, SecOC, steering handover, accel limits):
  `cd opendbc_repo && .venv/bin/python -m unittest discover -s opendbc/car/fisker/tests -t .` (about 120 tests)
- sunnypilot (from `openpilot/`):
  `PYTHONPATH=.:../opendbc_repo ../.venv/bin/python -m unittest <module>` for example
  `sunnypilot.selfdrive.controls.lib.tests.test_turn_planner`, `...test_driver_profile`, `...test_profile_long`, `...test_map_turn_controlsd`, `...test_narrow_street`, `sunnypilot.webhud.tests.test_server`, `sunnypilot.tools.tests.test_fisker_drive_review`, `sunnypilot.tools.drive_sim` tests.
- Lint: `../.venv/bin/ruff check <paths>` (openpilot), `.venv/bin/ruff check opendbc/car/fisker/` (opendbc).

`test_map_turn_controlsd` prints a "RuntimeError: boom" traceback on purpose (it checks controlsd falls back to the model when the planner fails).

## Positive control

A new test is only evidence if it fails without the fix:
```
cp <file> /tmp/fixed.py; git show HEAD:<repo-relative path> > <file>   # old code
<run the test: expect FAIL>
cp /tmp/fixed.py <file>                                                  # fix back
```
`git stash` is the wrong tool here: it also stashes the new test.

## Can't run on the Mac

The long MPC's acados solver fails every step on the Mac (solution_status 4). Anything that runs the longitudinal planner end to end goes on the device (`longitudinal-sim`).
