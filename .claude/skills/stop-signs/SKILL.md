---
name: stop-signs
description: Add or correct stop signs that OpenStreetMap is missing so Stop Sign Assist brakes for them, without editing OSM. Use when the user says the car didn't stop for a sign or a junction is an all-way stop.
---

# Local stop signs

Stop Sign Assist reads OSM stop nodes from mapd's tiles plus `<mapd root>/local_stop_signs.json`:
- device: `/data/media/0/osm/local_stop_signs.json`
- Mac: `~/.comma/media/0/osm/local_stop_signs.json`

```json
{"signs": [
  {"id": -2, "lat": 37.5140786, "lon": -121.9564138, "all_way": true,
   "note": "Doane St & Gatewood St: all-way stop; OSM maps only the southbound approach (driver confirmed)"}
]}
```
- Negative ids, unique. `all_way: true` applies to every approach; otherwise give `approaches` (the directions of travel that face the sign).
- Only add what the driver confirmed. Copy the file to both the Mac and the device. It is kept out of git.
- Editing OpenStreetMap itself is public and needs the user's explicit approval.

Check with the stop-sign replay or the next drive: plan source `stopSign` should appear approaching the junction.
