---
name: comma-deploy
description: Install local (unpushed) sunnypilot + opendbc commits on the comma with git bundles, keeping commit hashes identical, then reboot safely and verify. Use when asked to deploy, install or push code to the car.
---

# Deploy to the comma

The device's checkout is `/data/openpilot` on branch `mallik-fisker` (older deploys used `claude/project-thread-yleigw`), with the opendbc submodule in `opendbc_repo`. `git push` to the device hangs; bundles work and keep hashes identical (format-patch/am would rewrite them).

1. Find what the device has: `ssh comma@192.168.0.207 'cd /data/openpilot && git log --oneline -1 && cd opendbc_repo && git log --oneline -1'`.
2. Run the tests first (see the `fisker-tests` skill).
3. Bundle from those commits to HEAD (run on the Mac):
   ```
   git bundle create $TMP/sp.bundle <device-sunnypilot-sha>..HEAD
   git -C opendbc_repo bundle create $TMP/odbc.bundle <device-opendbc-sha>..HEAD
   scp $TMP/sp.bundle $TMP/odbc.bundle comma@192.168.0.207:/data/tmp/
   ```
4. On the device, opendbc first, then sunnypilot:
   ```
   cd /data/openpilot/opendbc_repo && git fetch -q /data/tmp/odbc.bundle HEAD
   cd /data/openpilot && git fetch -q --no-recurse-submodules /data/tmp/sp.bundle HEAD \
     && git checkout -q -B mallik-fisker FETCH_HEAD && git submodule update -q opendbc_repo
   git log --oneline -1; (cd opendbc_repo; git log --oneline -1)
   ```
   Plain `git fetch` fails on old-remote submodule refs; always `--no-recurse-submodules`.
5. Reboot only when the car is off (`comma-device` skill). Changes to `common/params_keys.h` or `cereal/*.capnp` need the reboot to rebuild; pure Python changes also only take effect on restart.
6. Verify after boot (about 1 minute): manager running, same commit, and the changed modules import:
   ```
   /usr/local/venv/bin/python -c "import openpilot.selfdrive.controls.controlsd, opendbc.car.fisker.carcontroller"
   ```
   If the panda safety code changed, pandad reflashes on boot ("Panda firmware out of date" -> "Done flashing" in swaglog).

If a background analysis is still running on the device, decide whether it must finish before the reboot kills it (for example a replay that checks the change being deployed).
