---
name: comma-device
description: Reach and work on the comma 4 in the Fisker Ocean over SSH - check whether the car is on, run Python there, run long jobs that survive Wi-Fi drops, and the safety rules for rebooting. Use before any command that touches the device.
---

# Working on the comma

- Host: `ssh comma@192.168.0.207` (home Wi-Fi; ICMP is blocked, so don't ping). The device drops off Wi-Fi while the car drives or sleeps. When it is unreachable, ask the user to wake it (car on, in Wi-Fi range) and retry in a background loop rather than giving up.
- Code: `/data/openpilot` (sunnypilot; the openpilot package is under `/data/openpilot/openpilot`). Drives: `/data/media/0/realdata/<route>--<seg>/{rlog,qlog}.zst`. Map tiles: `/data/media/0/osm/offline`.
- Python on the device: `/usr/local/venv/bin/python` with `PYTHONPATH=/data/openpilot/openpilot:/data/openpilot` (plain `python3` lacks zmq/capnp).

## Is the car on?

`cat /data/params/d/IsOffroad` -> `1` offroad (car off), `0` onroad. There is no `IsOnroad` param. The car is often on but parked.

**Never reboot unless IsOffroad is 1, or the user says so.** Guard it in the same command:
`ssh comma@192.168.0.207 '[ "$(cat /data/params/d/IsOffroad)" = "1" ] && sudo reboot'`

## Long jobs

The Wi-Fi link resets mid-command (rsync of big rlogs, long analyses). Run anything over ~1 minute detached on the device and poll for a sentinel:

```
ssh comma@192.168.0.207 'cd /data/openpilot && export PYTHONPATH=/data/openpilot/openpilot:/data/openpilot;
  nohup sh -c "nice /usr/local/venv/bin/python /data/tmp/job.py ARGS 2>&1 | grep -v Warning; echo ALLDONE" > /data/tmp/job.txt 2>&1 < /dev/null & echo started'
```
then wait in a background loop on `grep -c ALLDONE /data/tmp/job.txt`. Offroad only CPU cores 0-3 are online, so 2-3 parallel jobs are the limit and each runs slower.

## Pitfalls

- `pgrep -f` / `pkill -f` over ssh also match the ssh shell's own command line: use a bracket pattern (`tmp_[r]eplay`).
- Params written from Python need `put_bool(..., block=True)` or they read back stale.
- The manager process shows as `./manager.py`.
- Never print the SecOC key or other params that hold secrets.
