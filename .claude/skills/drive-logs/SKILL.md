---
name: drive-logs
description: Find and fetch drive logs from the comma - list recent routes, find the segment where something happened by place, copy rlogs/qlogs to the Mac. Use when the user mentions a drive, an incident, or asks to pull the latest logs.
---

# Drive logs

- On the device: `ls -t /data/media/0/realdata/ | head -40`. Route names are `<counter>--<hash>`, segments are `--0`, `--1`, ... (1 minute each). `boot` and `crash` are not routes.
- On the Mac: `~/Fisker/routes/<route>--<seg>/`.
- qlogs (10x decimated) have no `modelV2`; use rlogs for anything involving the model, lateral control at 100 Hz, or the planners.

## Find where something happened

The user usually names a place ("after turning left onto Doane from Gatewood"). Get its coordinates (OSM, or `~/.comma/media/0/osm/local_stop_signs.json` for the known junctions) and search on the device:

```
scp .claude/skills/drive-logs/scripts/find_place.py comma@192.168.0.207:/data/tmp/
ssh comma@192.168.0.207 'cd /data/openpilot && PYTHONPATH=/data/openpilot/openpilot:/data/openpilot \
  /usr/local/venv/bin/python /data/tmp/find_place.py 37.51408 -121.95641 60 000000f6 000000f7 000000f8'
```
It prints each segment that passes within the radius with the closest `logMonoTime`, bearing and speed. Use the bearing to tell which pass matches the user's direction of travel.

## Copy

The Wi-Fi link drops on big transfers, so copy only the segments you need, with partial resume and a timeout:

```
cd ~/Fisker/routes && rsync -a --partial --timeout=60 comma@192.168.0.207:/data/media/0/realdata/<seg>/{rlog.zst,qlog.zst} <seg>/
```
If copies keep failing, run the analysis on the device instead (see `drive-review`, `incident-investigation`) and copy back only the text output.
