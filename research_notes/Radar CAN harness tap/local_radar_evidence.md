# Fisker Ocean mid-range radar (MRR) link: local evidence from code, DBCs and logs

Scope: what the local repos and drive logs show about the radar's private CAN-FD link, what the code does once panda bus 1 carries it, what the car monitors on that link, and how to verify a tap from logs. Sources are local files (cited as path:line) and rlogs decoded on the Mac on 2026-11-26. Abbreviations: `SP` = /Users/hiremanirmss/Fisker/sunnypilot, `OS` = /Users/hiremanirmss/Fisker/oceanscape. Items marked **[Assessment]** are my inferences, not facts from a file or a log. Nothing touched the comma or the car.

Probe scripts (scratch, not in git): (a temporary folder, not kept) canstate.py, mrrfault.py. They were run with `SP/.venv/bin/python -I` and `PYTHONPATH=SP/openpilot:SP/opendbc_repo`.

## 1. The radar link's message set, timing and content (DBC)

### Takeaway
The MRR DBC is a draft that one contributor reverse-engineered from one 61 s low-speed capture. That capture is not in the local routes. It describes a two-node CAN-FD link. The ADAS controller sends time sync, ego motion, wheel speeds and clock to the radar. The radar sends a header, 32 object slots of 48 bytes and some auxiliary frames every 65.01 ms. E2E CRC-8 covers most frames. SecOC tails exist on two of them: ADAS→MRR 0x200 and the last object slot 0x32F.

### Cited Findings
- The two DBC copies are byte-identical (`diff` returned no difference): [SP/opendbc_repo/opendbc/dbc/fisker_ocean_mrr.dbc](SP/opendbc_repo/opendbc/dbc/fisker_ocean_mrr.dbc), [OS/dbc/fisker_ocean_mrr.dbc](OS/dbc/fisker_ocean_mrr.dbc). Both have 1429 lines.
- Provenance. The DBC says: "Fisker Ocean mid-range radar (MRR) private CAN-FD bus (panda bus 1). Draft, reverse-engineered from route 6bdf6a23860be7bd/000000b5--bfe13ac451 (61 s, low-speed parking-lot drive). Not in the FM29 ADASBUS matrix." E2E: "CRC-8 poly 0x1D, init 0, xorout 0, computed over [DataID] + bytes 1..N, same scheme as fiskercan.fisker_plain_checksum" — [mrr.dbc:1295](SP/opendbc_repo/opendbc/dbc/fisker_ocean_mrr.dbc).
- The DBC came in one opendbc commit, 05830f17 (2026-10-02, author duncan999007, "fisker: add mid-range radar (bus 1) radar interface … DBC moved here from the web HUD"). The commit also added radar_interface.py, its test, and the interface.py/values.py changes — `git log` / `git show 05830f17` in SP/opendbc_repo.
- The capture route is not on the Mac. `~/Fisker/routes` holds 0000002b and 000000dd onward and has no 000000b5 or 000000b4 folder (directory listing). The HUD calibration was also derived from 000000b5--bfe13ac451 and 000000b4--d0f733ebb2 — [OS/INTERNALS.md:209](OS/INTERNALS.md); [OS/static/js/world/world_model.js:38-40](OS/static/js/world/world_model.js).
- Nodes: `BU_: MRR ADAS`. Every TX message is either ADAS→MRR or MRR→ADAS — [mrr.dbc:14](SP/opendbc_repo/opendbc/dbc/fisker_ocean_mrr.dbc).
- Message set with GenMsgCycleTime ([mrr.dbc:1380-1422](SP/opendbc_repo/opendbc/dbc/fisker_ocean_mrr.dbc)), sizes ([mrr.dbc:16-1290](SP/opendbc_repo/opendbc/dbc/fisker_ocean_mrr.dbc)) and comments ([mrr.dbc:1296-1375](SP/opendbc_repo/opendbc/dbc/fisker_ocean_mrr.dbc)):

| ID | Name | Dir | Bytes | Cycle | Notes from DBC comments |
|---|---|---|---|---|---|
| 0x100 | ADAS_MRR_TimeSync | ADAS→MRR | 16 | 50 ms | AUTOSAR CanTSyn, "10 Hz SYNC/FUP pairs"; type 0x20 SYNC = seconds, 0x28 FUP = ns. Time base of the 0x300/0x500 timestamps |
| 0x200 | ADAS_MRR_VehDyn | ADAS→MRR | 32 | 10 ms | "Ego motion fed to the MRR, 100 Hz". E2E DataID 64 over bytes 1..28; **SecOC MAC in bytes 29..31** (Fresh byte 28, MAC bytes 29-31, [mrr.dbc:30-33](SP/opendbc_repo/opendbc/dbc/fisker_ocean_mrr.dbc)). VehSpd "= carState.vEgo*3.6 exactly"; YawRate same encoding as YRS_YawRate; DrvDir 0/1/2 |
| 0x201 | ADAS_MRR_WheelSpd | ADAS→MRR | 20 | 20 ms | Wheel speeds, directions, pulse counters, 50 Hz, E2E DataID 65; "Pulse-counter wheel order assumed FL,FR,RL,RR" |
| 0x202 | ADAS_MRR_Time | ADAS→MRR | 12 | 1000 ms | Vehicle clock, E2E DataID 67; h/m/s verified against GPS; upper 17 bits "probably the date but did not decode" |
| 0x300 | MRR_Header | MRR→ADAS | 32 | 65 ms | "one per 65.01 ms cycle, sent before the 32 object slots", E2E DataID 16. ObjListTime ~54 ms and MeasTime ~131 ms before receipt; NumObjects "matched 937/937 cycles"; LocalTime "Exactly 65.0117 ms per cycle … drifting ~30 ppm" |
| 0x301 | MRR_Status | MRR→ADAS | 4 | 20 ms | 50 Hz, E2E DataID 53, "Payload constant 0x01 0x00 in this log" |
| 0x310-0x32F | MRR_Obj00..31 | MRR→ADAS | 48 | 65 ms | Object slots, E2E DataIDs 17-50 (DataIDs 31 and 47 are skipped). **Slot 31 (0x32F) has a SecOC MAC in bytes 45..47; freshness byte 44 = 0 throughout** ([mrr.dbc:1265-1268, 1341](SP/opendbc_repo/opendbc/dbc/fisker_ocean_mrr.dbc)) |
| 0x400 | MRR_EgoMotion | MRR→ADAS | 48 | 65 ms | "No E2E (bytes 0-2 = 0)"; EgoSpeed signed, yaw rate, curvature, CurvState; scales are fits |
| 0x401 | MRR_Unk_401 | MRR→ADAS | 20 | 65 ms | "all zero in this log" |
| 0x402 | MRR_Unk_402 | MRR→ADAS | 48 | 65 ms | B5 "Maybe a raw detection count"; B19 "Looks like a temperature" |
| 0x500 | MRR_Env | MRR→ADAS | 48 | 65 ms | E2E DataID 52; blocks "filled in ~30-43% of cycles. Not decoded" |
| 0x600 | MRR_Config | MRR→ADAS | 20 | 1000 ms | E2E DataID 51, constant payload, "Probably mounting/calibration/version data" |

- Object-slot signals ([mrr.dbc:82-117](SP/opendbc_repo/opendbc/dbc/fisker_ocean_mrr.dbc), comments [1342-1362](SP/opendbc_repo/opendbc/dbc/fisker_ocean_mrr.dbc)):
  - ID (16 bit). A persistent track ID; "wraps in 11 bits so far". An empty slot has ID 0, byte 5 = 0x40 and bytes 6..43 = 0. Slots fill from 0x310 upward.
  - MeasHistory is a shift register (1 = detection this cycle). State "Rises 0->3 as a track matures; drops right before a track is deleted". Quality: "13 for mature stationary tracks, 5-10 otherwise; meaning unknown". Age is in 65 ms cycles.
  - DistLong is 0.05 m/bit, 0-409.55 m, "checked against GPS (0.0503 m/bit)".
  - DistLat is 0.05 m/bit, ±204.8 m, "POSITIVE = RIGHT … scale 0.05 matches but only to ~10%".
  - VrelLong and VrelLat are 0.06 m/s/bit. Against GPS, VrelLong fits 0.0625. The DBC keeps 0.06 on purpose because it matches the wheel-speed vEgo.
  - ArelLong and ArelLat are 0.03 m/s² ("0.03 vs 1/32 not resolved"). The *_Std fields are 6-bit codes "scale unknown". DistCorr, VrelCorr and ArelCorr are "probably a correlation term".
  - DynProp values: 5 = stationary, 2 = stationary/slow while ego stopped, 7 = mostly crossing, 1/3/4/6 = moving, 8 = far/uncertain.
  - Heading is 1.40625°/bit, clockwise. Width and Length are "Tentative 0.2 m".
  - Class: 0 = unclassified point, 1 = car-sized, 6 = pedestrian-sized, 8 = small, 3/5 rare. Status is "Bitfield-like". CycleCounter is 6 bit.
- Open questions in the DBC itself:
  - the meaning of Quality;
  - the scales of *_Std and the Corr terms;
  - ArelLong 0.03 vs 1/32;
  - the DistLat scale (only to ~10%);
  - the 0x202 date bits and bytes 7, 9, 10;
  - the 0x201 pulse-counter order;
  - 0x401, 0x402 and 0x500 not decoded;
  - 0x600 contents;
  - 0x301 constant.
  — [mrr.dbc:1300-1375](SP/opendbc_repo/opendbc/dbc/fisker_ocean_mrr.dbc)
- The radar resends cycles: "the radar now and then sends a cycle's header again (same MeasTime and counter, 124 of 923 cycles on 000000b5--bfe13ac451--13)" — [OS/static/js/world/fisker_radar.js:111-112](OS/static/js/world/fisker_radar.js).
- The radar has no elevation. Overhead traffic lights, gantries and bridges read as stopped cars in the lane. One example from the capture: a traffic light that the radar classed as a car 3 m wide and 0.6 m long — [OS/INTERNALS.md:224-229](OS/INTERNALS.md).
- The ADAS controller also publishes an MRR version frame, ADAS_0x67E (1662): MRR_SwVersS, PartVers, HwVers, SwVersM, 1000 ms — [OS/dbc/fisker_ocean_adas_world.dbc:1475-1479, 3049](OS/dbc/fisker_ocean_adas_world.dbc). On three local rlogs it reads `ff00000000000000` in every frame (mrrfault.py, see §4).

### Inferences
- **[Assessment]** Frame budget derived from the cycle table. The radar sends about 37 frames per 65 ms (header, 32 slots, 0x400, 0x401, 0x402, 0x500), plus 0x301 at 50 Hz. That is roughly 620 frames/s. The ADAS side adds 0x200 at 100 Hz, 0x201 at 50 Hz and 0x100 at 20 Hz. The bit rates are not stated in the DBC (see §3). At an assumed 500 kbit/s arbitration and 2 Mbit/s data phase, a 48-byte frame takes roughly 0.25-0.3 ms. That puts the link at very roughly 15% load. This is an estimate, not a measurement.
- **[Assessment]** The SecOC MACs on 0x200 and 0x32F do not matter to a listen-only tap. radar_interface.py never reads the tail bytes, and nothing has to be signed. They would matter only to someone who injects or modifies frames on the link. This port never does that (see §2).
- **[Assessment]** The DBC calls 0x200, 0x201, 0x202 and 0x100 ADAS→MRR messages "fed to the MRR", and the capture was taken on panda bus 1. So in the capture, bus 1 carried both directions of a shared bus. The ADAS module feeds the radar on the private link itself, not through the ADASBUS.

### Gaps
- The radar's arbitration and data bit rates, BRS use, and ISO vs non-ISO CAN-FD are not written in the DBC or in code. The capture probably worked at panda defaults (§3), but nothing records this.
- Whether the private link has only the two nodes (MRR and ADAS) or also the FCM: the FCM's own fault frame has an "FCM and MRR communication" lost flag (§4), which suggests the FCM also hears the MRR somewhere.
- The 61 s capture is the only radar data on record, and it is not available locally. No highway, rain or night radar data exists.

## 2. What the code does with radar data once bus 1 carries it

### Takeaway
The whole software path is already built. When the fingerprint sees 0x300 on bus 1 at start-up, `radarUnavailable` becomes false. Then radar_interface publishes dRel/yRel/vRel tracks, and radard matches them to the model's leads. On this port a radar lead always needs the model's confirmation. The panda never forwards or transmits on bus 1. The HUD decodes bus 1 independently and has an NPU-camera "overhead veto". That veto is display-only and does not feed openpilot control.

### Cited Findings
**Bus assignment and fingerprint**
- `CANBUS.radar = 1  # MRR private link (CAN-FD), receive only: safety never forwards or sends on it`; pt = 0 (vehicle-side ADASBUS), cam = 2 (OEM ADAS module side) — [SP/opendbc_repo/opendbc/car/fisker/values.py:48-57](SP/opendbc_repo/opendbc/car/fisker/values.py). DBC map `Bus.radar: 'fisker_ocean_mrr'` — [values.py:25-29](SP/opendbc_repo/opendbc/car/fisker/values.py).
- `ret.radarUnavailable = RADAR_HEADER not in fingerprint[CANBUS.radar]`; `ret.radarDelay = 0.08  # measured -> published: radar MeasTime vs the cycle's last frame, median ~80 ms on bus 1`. Without the tap "openpilot stays vision-only, like Tesla" — [SP/opendbc_repo/opendbc/car/fisker/interface.py:55-59](SP/opendbc_repo/opendbc/car/fisker/interface.py).
- The fingerprint dict records every bus with src < 128 (`finger[can.src][can.address] = len(can.dat)`). It is collected during fingerprinting, which ends after about 100 frames (FRAME_FINGERPRINT = 100, "1s"), at most 200 — [SP/opendbc_repo/opendbc/car/car_helpers.py:16, 44-79](SP/opendbc_repo/opendbc/car/car_helpers.py).

**Radar interface**
- radar_interface.py parses MRR_Header plus MRR_Obj00..31 at an expected 15 Hz on CANBUS.radar ([radar_interface.py:19-24](SP/opendbc_repo/opendbc/car/fisker/radar_interface.py)). It publishes once the last slot (0x32F) has updated ([:31, 46-52](SP/opendbc_repo/opendbc/car/fisker/radar_interface.py)). It sets `errors.canError` when `not self.rcp.can_valid` ([:59-60](SP/opendbc_repo/opendbc/car/fisker/radar_interface.py)). It drops a repeated cycle with the same (MeasTime, CycleCounter) ([:63-67](SP/opendbc_repo/opendbc/car/fisker/radar_interface.py)). It skips empty slots and slots left over from another cycle ([:75-76](SP/opendbc_repo/opendbc/car/fisker/radar_interface.py)). It filters on State ≥ 1 and Age ≥ 3 cycles ([:15-16, 77-78](SP/opendbc_repo/opendbc/car/fisker/radar_interface.py)). It maps radar IDs to stable track IDs and handles ID wrap by Age ([:80-89](SP/opendbc_repo/opendbc/car/fisker/radar_interface.py)). Only `dRel = DistLong`, `yRel = -DistLat` and `vRel = VrelLong` are passed on ([:95-97](SP/opendbc_repo/opendbc/car/fisker/radar_interface.py)). Class, DynProp, Arel, size, heading and quality are not used.
- When radarUnavailable is true, the base class returns an empty RadarData every 5th call (20 Hz) — [SP/opendbc_repo/opendbc/car/interfaces.py:92-95](SP/opendbc_repo/opendbc/car/interfaces.py). card.py publishes radarTracks with `valid = not any(RD.errors…)` — [SP/openpilot/selfdrive/car/card.py:245-248](SP/openpilot/selfdrive/car/card.py).
- opendbc's CANParser has no checksum rule for any Fisker DBC. dbc.py has special cases only for hyundai_canfd_generated, tesla_model3_party and similar names — [SP/opendbc_repo/opendbc/can/dbc.py:189, 207](SP/opendbc_repo/opendbc/can/dbc.py). So MRR frames are checked for presence and timing only, not for E2E CRC or counter. (`grep -i fisker` in opendbc/can/*.py returned nothing.)

**radard: how leads are formed**
- `RADAR_TO_CAMERA = 1.52` ([SP/openpilot/selfdrive/controls/radard.py:30](SP/openpilot/selfdrive/controls/radard.py)).
- `match_vision_to_track` picks the track with the highest Laplacian likelihood on d, y and v. It accepts the track only if |Δd| < max(25%, 5 m) and the velocity check passes ([radard.py:117-136](SP/openpilot/selfdrive/controls/radard.py)).
- `get_lead` uses radar tracks only when `lead_prob > .5`. Otherwise it falls back to a vision lead with `radar: False` ([radard.py:157-171](SP/openpilot/selfdrive/controls/radard.py)).
- The low-speed radar-only override is disabled for Fisker: "the Fisker MRR has no elevation: overhead lights/gantries and curbs show as in-lane stationary cars, so its leads need the model's confirmation (no radar-only low speed lead)", `low_speed_override=self.CP.brand != "fisker"`. leadTwo never uses the override — [radard.py:260-265](SP/openpilot/selfdrive/controls/radard.py).
- A radar-backed lead carries `radar: True` and `radarTrackId`, and takes dRel, vRel, vLead, aLeadK and aLeadTau from the track's Kalman filter ([radard.py:86-100](SP/openpilot/selfdrive/controls/radard.py)). A vision lead has `aLeadTau 0.3` and model-derived values ([:139-154](SP/openpilot/selfdrive/controls/radard.py)).

**Faults raised from radar errors**
- With `openpilotLongitudinalControl`: radarErrors.canError → `EventName.canError`; radarUnavailableTemporary → radarTempUnavailable; any other radar error → radarFault — [SP/openpilot/selfdrive/selfdrived/selfdrived.py:459-465](SP/openpilot/selfdrive/selfdrived/selfdrived.py).
- canError is IMMEDIATE_DISABLE, PERMANENT and NO_ENTRY, with the alert text "Unknown Vehicle Variant" in this tree — [SP/openpilot/selfdrive/selfdrived/events.py:753-760](SP/openpilot/selfdrive/selfdrived/events.py).
- radarFault and radarTempUnavailable are SOFT_DISABLE and NO_ENTRY ("Radar Error: Restart the Car") — [events.py:698-705](SP/openpilot/selfdrive/selfdrived/events.py).

**Planners that consume radarState.leadOne**
- Stock MPC: `self.mpc.update(sm['radarState'], …)` — [SP/openpilot/selfdrive/controls/lib/longitudinal_planner.py:120, 188](SP/openpilot/selfdrive/controls/lib/longitudinal_planner.py).
- sunnypilot planner (the driver-profile features):
  - follow_cap uses lead.present, dRel and vRel ([SP/openpilot/sunnypilot/selfdrive/controls/lib/longitudinal_planner.py:276-280](SP/openpilot/sunnypilot/selfdrive/controls/lib/longitudinal_planner.py)).
  - lead_brake uses `gate = 1.0 if lead.radar else lead.modelProb   # only a vision lead carries a certainty` ([:294-297](SP/openpilot/sunnypilot/selfdrive/controls/lib/longitudinal_planner.py)).
  - take-off uses vLead and aLeadK ([:398-410](SP/openpilot/sunnypilot/selfdrive/controls/lib/longitudinal_planner.py)).
  - lead_coast uses dRel and vRel ([:480-481](SP/openpilot/sunnypilot/selfdrive/controls/lib/longitudinal_planner.py)).
- Stop Sign Assist: `lead_relevant()` — [SP/openpilot/sunnypilot/selfdrive/controls/lib/stop_sign/stop_sign_controller.py:211-224](SP/openpilot/sunnypilot/selfdrive/controls/lib/stop_sign/stop_sign_controller.py).
- Red light: `_lead_before_stop` — [SP/openpilot/sunnypilot/selfdrive/controls/lib/red_light.py:241-246](SP/openpilot/sunnypilot/selfdrive/controls/lib/red_light.py).
- DEC (experimental-mode controller): lead filter — [SP/openpilot/sunnypilot/selfdrive/controls/lib/dec/dec.py:207-225](SP/openpilot/sunnypilot/selfdrive/controls/lib/dec/dec.py).
- e2e alerts — [SP/openpilot/sunnypilot/selfdrive/controls/lib/e2e_alerts_helper.py:64-65](SP/openpilot/sunnypilot/selfdrive/controls/lib/e2e_alerts_helper.py).
- controlsd_ext copies `lead.radar` through — [SP/openpilot/sunnypilot/selfdrive/controls/controlsd_ext.py:85-97](SP/openpilot/sunnypilot/selfdrive/controls/controlsd_ext.py).

**Panda safety: bus 1 is receive-only**
- Generic forwarding maps bus 0↔2 and gives every other bus `destination_bus = -1`. Bus 1 is never forwarded — [SP/opendbc_repo/opendbc/safety/safety.h:256-266](SP/opendbc_repo/opendbc/safety/safety.h). `fisker_fwd_hook` handles only bus_num 2 and 0 — [SP/opendbc_repo/opendbc/safety/modes/fisker.h:875-934](SP/opendbc_repo/opendbc/safety/modes/fisker.h).
- The only bus-1 TX entries are 0x5D0 (0x1D0 steering candidates) and 0x521 (0x121 accel candidates, long build only), with `check_relay = false` — [fisker.h:759-771](SP/opendbc_repo/opendbc/safety/modes/fisker.h). The tx hook buffers each candidate and then sets `tx = false;  // panda-internal: never transmit onto the radar link`. The comment reads: "Bus 1 is NOT unused here: it is the mid-range radar's private link, which we only receive … The cost is one rejected-frame echo per candidate in the host's can stream" — [fisker.h:618-664](SP/opendbc_repo/opendbc/safety/modes/fisker.h); also [fisker.h:13-16](SP/opendbc_repo/opendbc/safety/modes/fisker.h).
- A rejected TX is pushed back with `rejected = 1` ([SP/panda/board/drivers/can_common.h:186-189](SP/panda/board/drivers/can_common.h)). pandad adds CAN_REJECTED_BUS_OFFSET to src ([SP/openpilot/selfdrive/pandad/panda.cc:262-270](SP/openpilot/selfdrive/pandad/panda.cc)). That gives **src 193 = rejected TX on bus 1**. The carcontroller describes 0x5D0/0x521 as "panda-internal (never on a real bus)" — [SP/opendbc_repo/opendbc/car/fisker/carcontroller.py:63-72, 864, 914](SP/opendbc_repo/opendbc/car/fisker/carcontroller.py).

**Panda CAN controller configuration for bus 1**
- Firmware default for every bus: `can_speed = 5000U` (500 kbit/s), `can_data_speed = 20000U` (2 Mbit/s), `canfd_auto = false` — [SP/panda/board/drivers/can_common.h:129-131](SP/panda/board/drivers/can_common.h). `canfd_enabled` and `brs_enabled` latch true when an FD or BRS frame is received — [SP/panda/board/drivers/fdcan.h:234-237](SP/panda/board/drivers/fdcan.h).
- pandad defines `set_can_speed_kbps` and `set_data_speed_kbps` ([SP/openpilot/selfdrive/pandad/panda.cc:141-155](SP/openpilot/selfdrive/pandad/panda.cc)). A grep of SP/openpilot/selfdrive and SP/openpilot/system found no caller, so no Fisker-specific speed is set.
- **The panda is not silent in a car safety mode.** The default branch of `set_safety_mode` sets `can_silent = false` ([SP/panda/board/main.c:80-86](SP/panda/board/main.c)). Silent ("bus monitoring", FDCAN_CCCR_MON) applies only in SAFETY_SILENT ([main.c:55-58](SP/panda/board/main.c); [SP/panda/board/stm32h7/llfdcan.h:105-108](SP/panda/board/stm32h7/llfdcan.h)). So in fisker mode, panda's bus-1 controller takes part on the radar link: it sends ACK bits and, on errors, error frames.

**Web HUD**
- The bridge streams bus 1 frames that are in the radar DBC and skips empty slots (`BUS_RADAR = 1`) — [SP/openpilot/sunnypilot/webhud/state.py:22-26, 62-69](SP/openpilot/sunnypilot/webhud/state.py).
- `fisker_radar.js` keeps only `src === BUS_RADAR` frames ([OS/static/js/world/fisker_radar.js:15, 68](OS/static/js/world/fisker_radar.js)). It decodes 15 slot fields including Class, DynProp, Heading and size ([:47-50, 140-160](OS/static/js/world/fisker_radar.js)). It times each cycle from MeasTime through the 0x100 CanTSyn offset, with a 0.10 s fallback latency ([:23, 74-81, 104-120](OS/static/js/world/fisker_radar.js)). It hands each cycle out once ([:111-114](OS/static/js/world/fisker_radar.js)). `state()` returns null "while the radar bus is silent (or this harness doesn't tap it)", which shows as the "no radar" chip ([:163](OS/static/js/world/fisker_radar.js); [OS/INTERNALS.md:152-161](OS/INTERNALS.md)).
- world_model.js fuses radar, the ADAS list, openpilot leads and the Pulse NPU detector ("cam") ([OS/static/js/world/world_model.js:7-27](OS/static/js/world/world_model.js)). Its rules:
  - Radar tracks younger than 8 cycles are dropped (`RADAR_MIN_AGE = 8`, [:67](OS/static/js/world/world_model.js)).
  - Radar-alone confidence is 0.85 for a moving object and 0.6 for a standing one ([:100-101](OS/static/js/world/world_model.js)).
  - MEASURED_CALIBRATION applies a Doppler scale of 0.0625/0.06 and a radar yaw of 0.6° ([:166-170](OS/static/js/world/world_model.js)).
  - The **NPU veto**: a standing radar track in the path, 10-60 m ahead (`CAM_VETO_X`), that the NPU detector looks at for 0.6 s (`CAM_VETO_S`) without finding a vehicle is marked `overhead` and gets confidence 0 ([:118, 136, 857-863, 879](OS/static/js/world/world_model.js)).

**Earlier HUD hand-off**
- "The radar alone never shows a point target it hasn't classified … A radar-only object it has classified stays hidden until its radar track has lasted 1.3 s" — [OS/INTERNALS.md:229-237](OS/INTERNALS.md).

### Inferences
- **[Assessment]** After a tap, the visible changes in control would be:
  - leadOne/leadTwo become `radar: True` when a model lead (prob > 0.5) matches a radar track;
  - dRel/vRel come from the radar (range is no longer limited to the ~40-51 m of the OEM list);
  - aLeadK comes from radard's Kalman filter;
  - sunnypilot's lead_brake gate becomes 1.0 instead of modelProb for radar leads.
  Because Fisker disables the low-speed override, a stationary radar return with no model lead creates no lead. Gantries and bridges therefore should not cause phantom braking, but radar cannot add a lead that the model misses.
- **[Assessment]** The HUD's NPU overhead veto lives only in the browser page. radard has no equivalent. Its protection is the model-confirmation rule above.
- **[Assessment]** Main new failure mode. The log shows openpilotLongitudinalControl = True (§3). Once radarUnavailable is false, any loss of radar frames during a drive makes `rcp.can_valid` false, which gives radarErrors.canError, then the canError event, then IMMEDIATE_DISABLE. Causes include a loose tap, radar sleep, or the radar dropping out while the ADAS controller restarts it. Before the tap, the same fault was impossible, because the base interface returns empty data. An intermittent tap is therefore worse for openpilot than no tap.
- **[Assessment]** radarUnavailable is decided once, from the start-up fingerprint (≤ 2 s of frames). If the radar link starts after that window, the drive stays vision-only. The radar link is only proven live in the 61 s capture, and its wake timing is unknown.
- **[Assessment]** Because the panda is not silent on bus 1, adding it to the radar link adds a node that sends ACKs and active error frames. If panda's bit timing does not match the link (500k/2M default, ISO FD), its error frames would corrupt radar and ADAS frames on the real link. The 61 s capture with stock firmware suggests that the defaults fit, but this is not proven (§1 gaps).
- **[Assessment]** The rejected 0x5D0/0x521 candidates (§3: ~1 kHz each) never leave the panda (bus-1 totalTxCnt = 0). Once bus 1 is wired to the radar, a firmware or safety change that stops rejecting them would put about 2000 foreign frames/s onto the radar link. Any change to fisker.h's bus-1 handling needs this in mind.

### Gaps
- No test or log shows radard's match rate, or the dRel and yRel errors, for this radar against the model. The HUD calibration gives Doppler 1/16 and yaw 0.6° from the capture only.
- radarDelay: interface.py states a ~80 ms median "MeasTime vs the cycle's last frame". The DBC says MeasTime is "~131 ms before the frame is received" and ObjListTime ~54 ms. These two local statements do not agree. Which time radarDelay should reference needs checking on new data.

## 3. ADAS→MRR messages, bus 1 in today's logs, and the src 193 frames

### Takeaway
In the logs on the Mac, 0x200/0x201/0x202/0x100 do not appear on any panda bus. They exist only in the MRR DBC, as private-link messages, so they do **not** appear on panda bus 2. Bus 1 receives nothing (0 frames, controller rx counter 0). The only bus-1 traffic in the log is openpilot's own 0x5D0/0x521 candidate frames, which the panda rejects (src 193). The ADAS controller's own fault frames show no MRR communication fault. That suggests the radar link works and the harness simply does not reach it.

### Cited Findings
- Route 00000151--6a38b4d0fc--6 (421 s): frames by stream/src: can src 0: 171,797; src 2: 98,834; src 128: 98,721; src 130: 171,800; **src 193: 112,416**; no src 1 frame. sendcan: bus 0 5,400; bus 1 112,416; bus 2 599 (radar_probe.py).
- In that route, src 193 = 0x5D0 (61,308 frames, ~1024 Hz, 8 bytes) plus 0x521 (51,108, ~853 Hz, 8 bytes). They match sendcan bus 1 one for one, so they are rejected echoes of openpilot's candidate pools (radar_probe.py; code in §2).
- None of 0x100, 0x200, 0x201, 0x202, 0x300, 0x301, 0x400, 0x500 or 0x600 appears on any src in that route (radar_probe.py printed none). The frames that do appear are 0x310 on src 0 (49.8 Hz, 8 bytes) and 0x32F on src 2 (25 Hz, 8 bytes). They are unrelated 8-byte ADASBUS messages with the same IDs (the radar's are 48 bytes): CIM_0x310 / ADAS_0x32F — [OS/dbc/fisker_ocean_ibus.dbc:1913, 2079](OS/dbc/fisker_ocean_ibus.dbc); [OS/static/js/world/fisker_radar.js:8-9](OS/static/js/world/fisker_radar.js). The port's fingerprints include 0x310 and 0x32F as 8-byte ADASBUS IDs — [SP/opendbc_repo/opendbc/car/fisker/fingerprints.py:31-34](SP/opendbc_repo/opendbc/car/fisker/fingerprints.py).
- 0x200/0x201/0x202 are defined in no other local DBC. A grep of fisker_ocean_adas.dbc and OS/dbc/*.dbc for `BO_ 512/513/514` hits only fisker_ocean_mrr.dbc.
- carParams in that route: radarUnavailable True, radarDelay 0.08, openpilotLongitudinalControl True. radarState: 1,199 messages, leadOne.present in 606, **radar-backed 0** (radar_probe.py).
- Panda CAN health over the whole route (pandaStates max, canstate.py):
  - bus 0: rx 11.9 M, tx 1.95 M, 0 errors, 500/2000 kbit/s, FD and BRS true.
  - **bus 1: rx 0, tx 0, 0 errors, 500/2000 kbit/s, canfdEnabled false, brsEnabled false** (the flags stay false because no FD frame was ever received).
  - bus 2: rx 2.0 M, tx 3.39 M, totalErrorCnt 26.3 M, busOffCnt 54, FD and BRS true.
  - safety: fisker, param 3, safetyTxBlocked 502,734, safetyRxInvalid 0.
- Phase 0, across 89 drives and 510 segments: 0 frames at 0x300-0x32F on bus 1 in every segment; radarTracks 545,362 messages with 0 points; radarUnavailable true; 216,510 lead-present radarState frames, 0 radar-backed. "Why it is silent (harness, connector, radar power or wake-up) is unknown" — [SP/research_notes/Tesla Reality View equivalent feasibility/phase0_log_measurements.md:57-67, 79-81](SP/research_notes/Tesla%20Reality%20View%20equivalent%20feasibility/phase0_log_measurements.md).
- ADAS controller fault bits for the MRR, decoded by hand (Motorola 1-bit, start bit = byte·8 + bit) in three rlogs (00000151--6a38b4d0fc--6, 000000dd--ad372d473d--1, 00000110--a19b5651f0--20; mrrfault.py):
  - ADAS_0x481 (src 2): BusHwErr_MRR, ChkInvldSig_MRR and ComLostFlt_MRR are all 0 in every frame. The payload is always `0000000000000000`.
  - ADAS_0x482 (src 2): PrivateCANComE2EFlt_MRR and PrivateCANComLostFlt_MRR are 0. The payload is always `0000e0ffffffffff`. The set bits are 21-23 and up, which have no signal in the DBC; the defined signals end at bit 20 ([OS/dbc/fisker_ocean_adas_world.dbc:1308-1330](OS/dbc/fisker_ocean_adas_world.dbc)).
  - FCM_0x487 (src 0): FCM_ComLostFlt_MRR and FCM_DLCFlt_MRR are 0.
  - ADAS_0x317 SysFltTyp = 3 ("Radar") never occurs.

### Inferences
- **[Assessment]** The OEM controller reports no lost, E2E or hardware fault on its MRR link on any of the three drives checked. So the radar and its private link most likely work normally, and the comma's bus 1 is simply not wired to them. This fits the harness description in values.py: bus 1 is the radar link "when the harness taps it". It contradicts the idea that the radar is dead or asleep. The DBC does not say whether these flags mean "currently failing" or "confirmed".
- **[Assessment]** The ADAS controller feeds the radar ego motion and wheel speed (0x200/0x201) on the private link. The ADASBUS wheel and yaw frames that panda forwards bus 0→2 are the controller's inputs, not the radar's. A tap at the radar end and a tap at the controller end of the private link see the same frames.
- **[Assessment]** Bus 2's 26 M error count and 54 bus-offs are an existing condition of the camera-side splice, unrelated to the radar. They are worth recording in the baseline, because a new bus-1 error count after the tap must be judged separately from them.
- **[Assessment]** The src 193 frames are harmless today and stay harmless after a tap as long as fisker.h keeps `tx = false` for them. The verification below checks bus-1 totalTxCnt = 0 after the tap.

### Gaps
- Only three segments were decoded for the 0x481/0x482/0x487 bits. Phase 0 did not decode them. A sweep across all routes would show whether any drive ever had an MRR fault.
- The MRR version frame 0x67E reads byte 0 = 0xFF and zeros elsewhere. Whether that means "not reported" or a real version is unknown.

## 4. The car's fault monitoring on ADAS links, and what applies to a radar-link tap

### Takeaway
The ADAS controller watches its MRR private link with dedicated flags: lost communication, E2E fault, hardware bus error, invalid signal. It reports them in 0x481/0x482 every 100 ms, and the FCM has its own MRR flags. Local notes give documented timings for the controller's other links, but none for the MRR codes themselves. The comparable rules are a 200 ms missing-message timeout, 3 consecutive E2E errors, a 30 ms bus-off, and healing after the condition clears with 40-cycle ageing. The camera-tap report's procedure (baseline, one change at a time, hold beyond the slowest monitor, stop and roll back on any new code) carries over directly.

### Cited Findings
- MRR-related fault bits and their DTC names from the world DBC comments:
  - `ADAS_BusHwErr_MRR` "Hardware error of ADAS and MRR communication bus_C12A609";
  - `ADAS_ChkInvldSig_MRR` "_C12AE86";
  - `ADAS_ComLostFlt_MRR` "Fail loss of ADAS and MRR communication_U322687";
  - `ADAS_PrivateCANComE2EFlt_MRR` "_U134B82";
  - `ADAS_PrivateCANComLostFlt_MRR` "Lost communication between ADAS and MRR private CAN_U134B87";
  - `FCM_ComLostFlt_MRR` "Fail loss of FCM and MRR communication_U30EA87";
  - `FCM_DLCFlt_MRR` "FCM Check MRR Data Length Code Fault_C10F681".
  0x481, 0x482 and 0x487 are sent every 100 ms — [OS/dbc/fisker_ocean_adas_world.dbc:1250-1282, 1315-1326, 1341, 1374, 2734, 2742, 2766, 2798, 2804, 2821, 2854, 3039-3041](OS/dbc/fisker_ocean_adas_world.dbc).
- These DBC codes (C12Axxx, U32xxxx, U13xxxx) do not match the dtc.json numbering (C21Axxx, U15Bxxx) — [SP/research_notes/Fisker ADAS controller camera tap/fault_monitoring_and_procedure.md:149](SP/research_notes/Fisker%20ADAS%20controller%20camera%20tap/fault_monitoring_and_procedure.md). Blind-spot and lane-change functions have corner-radar codes C21AA09 to C21AD09 ([:270](SP/research_notes/Fisker%20ADAS%20controller%20camera%20tap/fault_monitoring_and_procedure.md)). No local note gives an MRR-specific dtc.json entry.
- `ADAS_SysFltTyp` (0x317, 51|4, 50 ms) has the value 3 = Radar — [fault_monitoring_and_procedure.md:127](SP/research_notes/Fisker%20ADAS%20controller%20camera%20tap/fault_monitoring_and_procedure.md).
- Documented comparable link monitors in the ADAS controller ([fault_monitoring_and_procedure.md:72-89](SP/research_notes/Fisker%20ADAS%20controller%20camera%20tap/fault_monitoring_and_procedure.md)):

| Code | What sets it | Monitor | Effect | Reset / age |
|---|---|---|---|---|
| U15B087 | ICC CAN messages missing 200 ms | 20 ms | "All features will be disabled" | messages back for 200 ms; 40 drive cycles |
| U15B586 | ICC_0x52A checksum/counter errors, 3 consecutive | 10 ms | features relying on ICC degraded | not stated |
| U15B788 | CAN bus off ≥ 30 ms | 10 ms | "Rear View Only … after 2000 ms no view, shutdown" | bus normal 10 ms; 40 drive cycles |
| C21A531 / C21A586 | FCM Ethernet PDU missed 500 ms / 3 consecutive E2E errors | 50 ms | LCA, LCC, LKA… critical; ACC high; AEB medium | not stated |
| U15B616 | battery < 8.5 V for 10 s | 1 s | no ADAS feature | ≥ 8.5 V for 10 s |

- Camera codes use a 500 ms persistence on a 100 ms monitor, so the "fastest a code can set is about 0.5 to 0.6 s". Healing and ageing are undocumented for those codes. "Other ADAS codes that state a rule heal when the condition clears and age out after 40 drive cycles" — [SP/reports/Fisker ADAS controller camera tap.md:88, 97](SP/reports/Fisker%20ADAS%20controller%20camera%20tap.md).
- AEB "uses the exterior cameras and radar sensors". An AEB fault lights a telltale — [fault_monitoring_and_procedure.md:263](SP/research_notes/Fisker%20ADAS%20controller%20camera%20tap/fault_monitoring_and_procedure.md).
- Connector: `C003B*6F` (560123-0200) on the ADAS controller carries "Private CAN pairs for five radars" plus DMS LED lines — [SP/reports/Fisker ADAS controller camera tap.md:24](SP/reports/Fisker%20ADAS%20controller%20camera%20tap.md); [controller_and_camera_hardware.md:68](SP/research_notes/Fisker%20ADAS%20controller%20camera%20tap/controller_and_camera_hardware.md). The controller sits in the cargo area behind the lower side trim, fuse MF07 15 A — [camera tap report:9](SP/reports/Fisker%20ADAS%20controller%20camera%20tap.md).
- UDS: ADAS at 0x7A6/0x7AE. Fiskeretta lists "four MRR entries" among discovered addresses, and clearing allows for NRC 0x78 because "radars/cameras can take seconds to wipe their fault memory". Keep the status byte: 0x01 failing now, 0x04 pending, 0x08 confirmed — [fault_monitoring_and_procedure.md:198, 201](SP/research_notes/Fisker%20ADAS%20controller%20camera%20tap/fault_monitoring_and_procedure.md); [camera tap report:127](SP/reports/Fisker%20ADAS%20controller%20camera%20tap.md).
- The SecOC DTCs (C21B000, C21B142, C21B200, C21B300) concern secured control PDUs — [fault_monitoring_and_procedure.md:93](SP/research_notes/Fisker%20ADAS%20controller%20camera%20tap/fault_monitoring_and_procedure.md).
- A precedent from this port: openpilot frames that reached the ADAS module as a second sender of its own IDs caused "ADAS_Sts_ACC_ICC going to the fault state and staying there until an ignition cycle" (2026-10-19) — [SP/opendbc_repo/opendbc/safety/modes/fisker.h:907-918](SP/opendbc_repo/opendbc/safety/modes/fisker.h).
- Camera-tap procedure to reuse ([camera tap report:141-159](SP/reports/Fisker%20ADAS%20controller%20camera%20tap.md)):
  - baseline code read without clearing, plus a CAN record;
  - a connector-handling step alone;
  - the added hardware alone;
  - the receiver;
  - low speed, then road;
  - stop on any new stored, pending or failing code, a new 0x481/0x482 bit, a telltale or a chime;
  - rollback with the car asleep, then read, save and clear (0x14), then two ignition cycles.

### Inferences
- **[Assessment]** Which monitors a radar tap can trip, from the nature of the tap:
  - A Y-tap that opens the connector, or a bad crimp, can interrupt the link. That trips ComLostFlt_MRR / PrivateCANComLostFlt_MRR (U322687 / U134B87).
  - A panda that disagrees on bit timing sends error frames, because it is not silent (§2). That can corrupt frames and trip E2E (U134B82), bus-hardware (C12A609) or bus-off-type faults.
  - A long stub or a third termination degrades signal quality at the 2 Mbit/s data phase.
  - Injection is ruled out in software: bus 1 TX is always rejected.
- **[Assessment]** Consequences are probably larger than for a surround camera. The MRR is the forward radar for ACC and AEB (the manual names radar for AEB; SysFltTyp has distinct values for Radar, AEB and ACC). So a confirmed MRR fault probably costs ACC, AEB and FCW at driving speed, not only park features. No local document lists the MRR codes' feature losses. This is inferred from the feature list and the FCM code table, not documented.
- **[Assessment]** Assume a code that sets on the MRR link holds until at least the next ignition cycle. That follows the camera-tap report's rule and the 2026-10-19 ACC fault precedent.
- **[Assessment]** The 100 ms 0x481/0x482 flags let the comma itself act as the live fault indicator on bus 2. These signals are already logged in every rlog (raw CAN), but the port's own DBC and carstate do not decode them ([camera tap report:123](SP/reports/Fisker%20ADAS%20controller%20camera%20tap.md)).

### Gaps
- The persistence, reset and ageing rules and the feature effects for U322687, U134B87, U134B82, C12A609, C12AE86, U30EA87 and C10F681 are not in any local file.
- Whether the MRR itself logs a DTC for a disturbed link (a missing 0x200 or bad SecOC on 0x200) is not documented locally. If the radar also checks the SecOC on 0x200, a tap that corrupts frames could raise a radar-side SecOC code. There is no evidence either way.
- The physical location of the radar connector and its pinout are not in the local files reviewed here.

## 5. Verification plan from logs after a tap, and rollback

### Takeaway
Everything needed for verification is already in the rlog. Raw CAN on all buses, pandaStates canState1, carParams, radarTracks and radarState together prove the link is received, clean and correctly used. The ADAS 0x481/0x482 bits prove the car did not notice. A rollback is the factory connector plus a code read and clear. **[Assessment]** The steps are my proposal, built from the code paths above.

### Cited Findings (what each check rests on)
- bus 1 health fields in pandaStates: canState1.totalRxCnt, totalErrorCnt, busOffCnt, canSpeed, canDataSpeed, canfdEnabled, brsEnabled (pre-tap baseline: rx 0, 0 errors, FD false; §3).
- Expected rates from the DBC: 0x300 and 0x310-0x32F at 65 ms (~15.4 Hz); 0x400/0x401/0x402/0x500 at 65 ms; 0x301 and 0x201 at 50 Hz; 0x200 at 100 Hz; 0x100 at 20 Hz; 0x202 and 0x600 at 1 Hz. Lengths: 48 bytes for slots, 32 for 0x300 and 0x200 — [mrr.dbc:1380-1422](SP/opendbc_repo/opendbc/dbc/fisker_ocean_mrr.dbc).
- Activation conditions:
  - `radarUnavailable = 0x300 not in fingerprint[1]` ([interface.py:58](SP/opendbc_repo/opendbc/car/fisker/interface.py));
  - radarTracks gets points with State ≥ 1 and Age ≥ 3 ([radar_interface.py:15-16, 77](SP/opendbc_repo/opendbc/car/fisker/radar_interface.py));
  - leadOne.radar is true only with the model's lead_prob > 0.5 ([radard.py:162-171, 260-263](SP/openpilot/selfdrive/controls/radard.py)).
- Existing tools: `SP/openpilot/sunnypilot/tools/rv_phase0.py` already counts bus-1 frames at 0x300-0x32F, radarTracks points and the radar-backed lead share — [rv_phase0.py:23, 205-209](SP/openpilot/sunnypilot/tools/rv_phase0.py); command in [phase0_log_measurements.md:7-9](SP/research_notes/Tesla%20Reality%20View%20equivalent%20feasibility/phase0_log_measurements.md). The web HUD replay shows the radar chip and tracks, with the "no radar" state — [OS/INTERNALS.md:152-161](OS/INTERNALS.md).

### Proposed plan **[Assessment]**
0. **Baseline, before any hardware.**
   - Pull one fresh route.
   - Record 0x481/0x482/0x487 (all MRR bits 0, as in §3), 0x317 SysFltTyp, and pandaStates for all three buses (the bus-2 error count is already high).
   - Do a UDS read of the ADAS module (0x7A6) and the MRR entries with status bytes kept. Clear nothing.
   - Confirm that the build's fisker.h still rejects bus-1 TX (src 193 present, bus-1 totalTxCnt 0).
1. **Tap fitted, ignition on, parked, comma powered (logging).** From the first segment's rlog, check:
   - (a) bus 1 src 1 frames exist from the first second. Every ID in the DBC table appears at its DBC rate (±5%) and length. Both directions are present (0x200/0x201 from the ADAS controller, 0x300+ from the radar).
   - (b) canState1: canSpeed/canDataSpeed 500/2000, canfdEnabled and brsEnabled true or false matching the frames, totalErrorCnt and busOffCnt stay 0, totalTxCnt 0.
   - (c) The 0x481/0x482/0x487 MRR bits stay 0 for the whole segment and over several ignition cycles. 0x317 never shows 3.
   - (d) Recompute the E2E CRC-8 (poly 0x1D, DataID per DBC comment) on a sample of 0x300/0x310 frames to prove they arrive intact. CANParser does not check them.
   - (e) A UDS read shows no new code against the baseline.
2. **Fingerprint and parameters.** carParams.radarUnavailable = False in that route (the bus-1 header was seen inside the ≤ 2 s fingerprint window). If it stays True while bus 1 has frames, the radar started late. Record the delay from panda start to the first 0x300.
3. **Track content, parked then low speed.**
   - radarTracks has points.
   - Header NumObjects matches the populated slots.
   - The duplicate-cycle rate is similar to the capture (~13%).
   - Stationary targets read vRel ≈ −vEgo (the DBC's 0.06 scale against wheel speed).
4. **Lead comparison on a drive (openpilot long off at first, car ACC or manual).**
   - Share of leadOne.present frames with radar = True.
   - For radar-backed frames, compare radar dRel against model `leadsV3[0].x[0] − 1.52`.
   - Compare vLead against model v, by range bin (0-20, 20-50, 50-100 m).
   - Look for leads beyond ~51 m, which the OEM list never reported.
   - Count leads that flip between radar and vision.
   - Check that radarErrors stays empty and that no canError, radarFault or radarTempUnavailable event appears.
   - Replay in the HUD to compare radar tracks against ADAS-list cars and the NPU veto at gantries.
5. **Long-term checks.**
   - Repeat 1(b) and 1(c) on every drive for the first weeks, including cold starts and a car wash or rain (radar connector at the bumper).
   - Add a check that flags any drive where canState1 errors are greater than 0, or where any MRR bit in 0x481/0x482 is set.
6. **Only then** enable openpilot longitudinal with radar leads. Remember that a mid-drive loss of radar frames now gives an IMMEDIATE_DISABLE (canError).

**Stop conditions:**
- any MRR bit set in 0x481/0x482/0x487;
- SysFltTyp = 3;
- any new ADAS or MRR DTC (pending, confirmed or failing);
- canState1 errors or bus-off;
- missing or irregular radar frames;
- a dash telltale, or a change in ACC or AEB availability;
- radarUnavailable flapping between drives.

**Rollback:**
- With the car asleep, remove the tap and restore the factory connector.
- Ignition on: confirm that 0x481/0x482 MRR bits are 0 and that bus 1 is silent again (radarUnavailable True, vision-only, which is the proven state of 89 drives).
- Read codes and check bit 0x01 first. Save, then clear with 0x14 (allow for NRC 0x78).
- Run two ignition cycles and re-read.
- No software change is needed, because the code falls back to vision-only automatically when bus 1 is silent at fingerprint.

### Gaps
- A log-only check cannot prove the absence of short glitches below the controller's persistence time. The UDS read and the 100 ms fault bits are the car's own record.
- No local document states the radar's wake or start-up timing, which decides whether the fingerprint window catches it.
