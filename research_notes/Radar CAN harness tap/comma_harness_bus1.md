# comma four harness and panda bus 1: receiving a third CAN-FD bus from a radar tap

Scope: how a comma four (built-in panda, harness box, car harness) receives a third bus on panda bus 1, as the Fisker port uses for the mid-range radar (MRR) private link. Local code is cited as `path:line` (repo root `/Users/hiremanirmss/Fisker/sunnypilot`). "Assessment" marks my own inference.

## 1. Harness box / car harness connector: where CAN0/1/2 appear, and is CAN1 passed through

### Takeaway
comma's own harness build guide says the 18-pin harness connector carries three CAN pairs. CAN0 is the car side and CAN2 is the camera side; the harness box relay connects them. CAN1 is a third pair that the box passes straight through to the device without intercepting it, and comma says "typically" it goes to the radar. So the standard path for a radar tap is to land the radar pair on the CAN1 pins of the 18-pin connector. A Y-cable, a splice or a pass-through pigtail can feed those pins. No new comma part is needed.

### Cited Findings
- 18-pin harness connector signal colours: CAN2_H orange / CAN2_L green (camera side), CAN0_H orange / CAN0_L green (car side), "CAN1_L-BLUE" and "CAN1_H-PINK", plus IGN brown, GND black and optional 12VIN red. — [commaai/hardware BUILD_HARNESS.md](https://raw.githubusercontent.com/commaai/hardware/master/harness/BUILD_HARNESS.md)
- On CAN1: "We cannot intercept this CAN bus, but we can read and write messages from this bus in the software." "Typically, this CAN bus is connected to the radar." "The harness box has two connections for CAN1, the wires are passed through." — [BUILD_HARNESS.md](https://raw.githubusercontent.com/commaai/hardware/master/harness/BUILD_HARNESS.md)
- "CAN2 and CAN0 are physically connected when the relay in the harness box is closed." Termination: a resistor loopback "between Pin 3 and Pin 5, which will terminate the CAN0 bus", and another "between Pin 13 and Pin 15". — [BUILD_HARNESS.md](https://raw.githubusercontent.com/commaai/hardware/master/harness/BUILD_HARNESS.md)
- Pass-through cables are labelled PT1, PT2, PT3 and so on. A pre-crimped development harness is available, and the crimp tool link is Molex 0638192300. — [BUILD_HARNESS.md](https://raw.githubusercontent.com/commaai/hardware/master/harness/BUILD_HARNESS.md)
- The harness directory README says: the harness is "typically connected to the ADAS camera"; "Internally, the camera's can bus is separated from the rest of the car"; the relay "opens automatically when openpilot starts"; the OBD-C cable is a fully-connected USB-C 3.1/3.2 Gen 2 cable, with its pinout only in `OBD-C.sch.pdf`; wiring diagrams for sold harnesses are in `harness/v3`. — [commaai/hardware harness/](https://github.com/commaai/hardware/tree/master/harness)
- comma shop "harness connector" ($50) contains the wires and connectors to move an existing car harness to another car, without the harness box. The "Development" option gives "all the wires and only the 18 pin molex connector". The listed platforms include Hyundai A–R, Ford Q3/Q4, Rivian A/B, Tesla A/B and "Developer". — [comma shop: harness connector](https://comma.ai/shop/harness-connector)
- From the OBD-C connector on the harness you can reach the CAN on the OBD-II port, and the design includes a fourth CAN bus for the car's main OBD CAN. The OBD-C port outputs 12 V and is not USB-PD, so do not connect normal USB-C devices to it. — search-result summary of [openpilot wiki Troubleshooting](https://github.com/commaai/openpilot/wiki/Troubleshooting) and [blog.comma.ai/our-hardware-future](https://blog.comma.ai/our-hardware-future/) (pages not fetched in full; treat as secondary)
- Panda firmware: harness orientation (flip) swaps only bus 0 and bus 2. Bus 1's lookup is fixed. — `panda/board/drivers/can_common.h:142-146`
- comma four panda board file: `panda/board/boards/cuatro.h` ("Cuatro (STM32H7) + Harness", line 6). It has 4 transceiver enables (`cuatro.h:9-25`), FDCAN3 on its own pins (`cuatro.h:66-70`), and reuses `tres_set_can_mode` (`cuatro.h:128`). That function routes FDCAN2 (bus 1) to transceiver 2 (harness) in CAN_MODE_NORMAL or to transceiver 4 (OBD) in CAN_MODE_OBD_CAN2, and XORs the choice with harness flip status (`panda/board/boards/tres.h:57-93`).
- Every safety mode except ELM327 sets `CAN_MODE_NORMAL` (`panda/board/main.c:55-86`). So in the Fisker mode, bus 1 is the harness CAN1 pair, not the OBD port.

### Inferences
- Assessment: "cuatro" is the comma four's panda (tres = comma 3X). This is based on the file name and its comma-four-like features (fan, amp, SOM bootkick). I did not confirm it from a comma doc.
- Assessment: because CAN1 is a plain pass-through on the 18-pin connector, the usual ways to add a radar tap are: (a) add CAN1_H/CAN1_L wires (pink/blue) to the car-side harness and splice or Y them onto the radar link, or (b) build a pass-through Y pigtail at the radar connector and run a pair back to the harness. The harness box needs no change.
- Assessment: `tres_set_can_mode` XORs with flip status. So if the harness is detected as flipped, NORMAL mode puts bus 1 on the transceiver that would otherwise serve OBD. The physical CAN1 pins carry bus 1 only when orientation detection works correctly. Verify with a live capture after a tap: radar IDs 0x300/0x310.. must appear on bus 1.

### Gaps
- I found no pin-number-to-signal table for the 18 pins. BUILD_HARNESS.md gives colours, not pin numbers, except that the termination pins 3/5 and 13/15 are named. The `harness/v3` wiring diagrams and `OBD-C.sch.pdf` were not read. A fetch of the DeepWiki summary returned HTTP 429.
- I found no comma product named "CAN-FD harness" or "universal harness" for 2026. The shop lists a "Developer" harness platform and the Development connector option only. I could not confirm current prices or a comma-four-specific harness box revision.

## 2. Panda (comma four) CAN-FD on bus 1: data rates, how openpilot sets speeds, silent/listen-only

### Takeaway
All three panda buses run the same STM32H7 FDCAN code. Bus 1 is CAN-FD capable with BRS, at default rates of 500 kbit/s arbitration and 2 Mbit/s data. Data rates up to 5 Mbit/s are allowed. FD and BRS for transmit are switched on automatically when the first FD/BRS frame arrives. Nothing in this tree changes bus speeds per car. The only "silent" mode is global: SAFETY_SILENT puts every FDCAN core in bus-monitoring mode. Any active safety mode, Fisker included, clears it. So the panda's bus-1 controller is a normal, ACK-capable node even though it never transmits frames there.

### Cited Findings
- Default bus config for all three buses: `can_speed = 5000U` (500 kbit/s, units of 100 bit/s), `can_data_speed = 20000U` (2 Mbit/s), canfd_auto/canfd_enabled/brs_enabled false at boot. — `panda/board/drivers/can_common.h:129-131`
- Allowed rates: arbitration `{10k,20k,50k,100k,125k,250k,500k,1M}`. Data `{… 1M, 2M, 5M}` (`speeds`/`data_speeds` arrays, in 100 bit/s). — `panda/board/stm32h7/llfdcan.h:4-5`. 5 Mbit/s has its own timing branch. — `llfdcan.h:82` (inside `llcan_set_speed`, which starts at line 47)
- FD and BRS operation are always enabled on the core: `FDCANx->CCCR |= (FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE)`. — `panda/board/stm32h7/llfdcan.h:161`. So the core receives FD frames on every bus without host setup, as long as the data-phase bit rate matches.
- On RX, the first FD frame sets `canfd_enabled` and the first BRS frame sets `brs_enabled` for that bus. — `panda/board/drivers/fdcan.h:232-238`. These flags affect only how outgoing frames are framed. — `fdcan.h:110-115`
- USB/SPI control requests: 0xde sets the arbitration bitrate (`main_comms.h:243-250`). 0xe8 sets canfd_auto (`main_comms.h:287-289`). 0xf9 sets the data bitrate, which also sets canfd_enabled when the data rate is at least the arbitration rate and brs_enabled when it is greater (`main_comms.h:323-331`). 0xfc sets non-ISO mode.
- Host side: `Panda::set_can_speed_kbps` and `Panda::set_data_speed_kbps` exist (`openpilot/selfdrive/pandad/panda.cc:141,149`). A grep of `openpilot/selfdrive`, `openpilot/sunnypilot` and `opendbc_repo/opendbc/car` (excluding tests) found no caller. pandad only reports the speeds in pandaStates (`openpilot/selfdrive/pandad/pandad.cc:167-168`). So every car, Fisker included, runs at the firmware defaults of 500k/2M.
- Silent mode: `bool can_silent = true` at boot (`can_common.h:11`). SAFETY_SILENT sets `can_silent = true`. NOOUTPUT, ELM327 and all car modes set `can_silent = false` (`panda/board/main.c:55-86`). After that, `can_init_all()` re-applies it (`main.c:88`). `can_set_speed` passes the single global `can_silent` to every bus (`panda/board/drivers/fdcan.h:5-18`). In `llcan_set_speed`, `if (silent) FDCANx->CCCR |= FDCAN_CCCR_MON;`, commented "Silent is known as bus monitoring in the docs". — `panda/board/stm32h7/llfdcan.h:105-108` (MON is cleared first at line 58)
- Panda falls back to SAFETY_SILENT when the heartbeat is lost (`main.c:111, 215-235`) and starts in SILENT at boot (`main.c:316-317`).
- Default forwarding only joins bus 0 and bus 2. `get_fwd_bus(1)` returns -1, so bus-1 traffic is never forwarded anywhere and nothing is forwarded onto bus 1. — `opendbc_repo/opendbc/safety/safety.h:256-266`

### Inferences
- Assessment: there is no per-bus listen-only setting. The only firmware path to MON on bus 1 alone would be a firmware change, for example a per-bus silent flag passed to `llcan_set_speed`.
- Assessment (from standard CAN/CAN-FD controller behaviour, no URL fetched): outside bus-monitoring mode, an FDCAN node drives the ACK slot for every frame it receives correctly. It also sends error frames when it sees errors. On a radar link that already has its real receiver (the ADAS module), one more ACK is harmless. But if panda's bit timing does not match the link, for example the link uses a data rate other than 2 Mbit/s, panda will flag errors and can disturb the radar link. The rate must be confirmed before a tap is left in the car. The `canDataSpeed`/error counters in pandaStates can confirm it.
- Assessment: when the car is off or openpilot is not running and the heartbeat is lost, panda goes to SILENT, and bus 1 is then truly listen-only. While driving, it is not.

### Gaps
- I did not read the opendbc `canfd` handling in card/car_helpers for per-car `CP.flags`. The grep shows no speed-setting caller, but upstream openpilot versions may differ.
- The ACK and error-frame behaviour of the STM32H7 FDCAN in normal mode was not checked against ST RM0433. Mark it as standard-protocol knowledge.

## 3. How the local Fisker port configures bus 1

### Takeaway
The Fisker port treats bus 1 as a receive-only CAN-FD radar link. A parser reads it, and safety neither forwards onto it nor puts frames on its wire. Speeds are not set explicitly, so they rely on the firmware defaults of 500k/2M plus FD auto-detect. Two caveats: (a) the TX allow-list does contain two bus-1 IDs (0x5D0, 0x521), but these are "candidate pool" frames that the tx hook always rejects; (b) the panda controller on bus 1 is in normal mode, not monitor mode, while the Fisker safety mode is active.

### Cited Findings
- `CANBUS.radar = 1  # MRR private link (CAN-FD), receive only: safety never forwards or sends on it`. pt = 0 (vehicle side), cam = 2 (OEM ADAS side). — `opendbc_repo/opendbc/car/fisker/values.py:48-58`
- DBC map: `Bus.radar: 'fisker_ocean_mrr'`, "Mid-range radar private link, panda bus 1". — `values.py:24-29`
- `ret.radarUnavailable = RADAR_HEADER not in fingerprint[CANBUS.radar]`: without the tap, bus 1 is silent and openpilot stays vision-only. `radarDelay = 0.08`. — `opendbc_repo/opendbc/car/fisker/interface.py:55-59`
- Radar parser on bus 1: header 0x300 plus 32 object slots 0x310..0x32F every 65 ms, `CANParser(..., CANBUS.radar)`. — `opendbc_repo/opendbc/car/fisker/radar_interface.py:6-24`
- TX allow-list entries on bus 1: `{0x5D0, 1, 8}` in FISKER_TX_MSGS and `{0x5D0, 1, 8}` plus `{0x521, 1, 8}` in FISKER_LONG_TX_MSGS. — `opendbc_repo/opendbc/safety/modes/fisker.h:763,770,772`
- In the tx hook, both candidate types are buffered for `fisker_fwd_modify` and then `tx = false;  // panda-internal: never transmit onto the radar link`. — `fisker.h:618-644` (0x5D0) and `fisker.h:647-663` (0x521). The comment says: "Bus 1 is NOT unused here: it is the mid-range radar's private link, which we only receive." (`fisker.h:622-626`). The cost is one rejected-frame echo per candidate in the host's CAN stream.
- `fisker_fwd_hook` handles only `bus_num == 2` and `bus_num == 0` (`fisker.h:882, 899`). The generic layer returns -1 for bus 1 (`safety.h:256-266`).
- No `set_can_speed_kbps`/`set_data_speed_kbps` call exists for Fisker or any car (see section 2). The Fisker car code has no `canfd` flag (grep of `opendbc/car/fisker/*.py`).

### Inferences
- Assessment: the "never transmit" guarantee depends on the tx hook's `tx = false` for 0x5D0/0x521. If someone later adds another bus-1 TX entry without that rejection, the radar link would receive frames. A safety test asserting that no frame with bus == 1 is ever allowed would lock this in. I did not check whether such a test exists in `opendbc_repo/opendbc/safety/tests`.
- Assessment: the speeds are correct only if the MRR link really is 500 kbit/s / 2 Mbit/s ISO CAN-FD. The fact that the parser decodes 0x300/0x310.. on bus 1 in logs (the radarDelay comment says it was measured on bus 1) is strong evidence that the defaults match. A wrong rate would produce no valid frames.
- Assessment: for true listen-only on the radar link, the options are a firmware change (per-bus MON) or a hardware choice (tap through a listen-only transceiver). As it stands, panda ACKs on the radar link while engaged.

### Gaps
- I did not pull a live pandaStates sample to confirm `canDataSpeed`, `canfdEnabled` or `brsEnabled` for bus 1, or the bus-1 error counters, on the car.

## 4. Other ports that bring a third bus in on panda bus 1

### Takeaway
Bus 1 as "the extra bus, often the radar" is a well-established pattern. Ford, Rivian and Hyundai's radar interfaces all parse bus 1. Ford's CanBus names bus 1 `radar`. Tesla puts the vehicle bus on bus 1. Hyundai CAN-FD uses bus 1 for A-CAN or E-CAN, depending on the harness variant. Rivian owners add the third bus with a harness mod plus a pigtail adapter at a secondary port.

### Cited Findings
- Ford: `CanBus.radar` returns `offset + 1` (main = offset, camera = offset + 2). — `opendbc_repo/opendbc/car/ford/fordcan.py:8-18`. Ford uses `Bus.radar: RADAR.DELPHI_MRR`. — `ford/values.py:86`
- Rivian radar parser: `CANParser(DBC[...][Bus.radar], messages, 1)`. — `opendbc_repo/opendbc/car/rivian/radar_interface.py:14`
- Tesla: `CANBUS.party = 0, vehicle = 1, autopilot_party = 2` (`opendbc_repo/opendbc/car/tesla/values.py:99-102`). Its radar parser also uses bus 1 (`tesla/radar_interface.py:21`).
- Hyundai CAN-FD: "the LKAS camera is on both A-CAN and E-CAN. LKA steering cars have a different harness than the LFA steering variants in order to split a different bus". `_a, _e = 1, 0` by default, swapped to `0, 1` for LKA-steering cars. — `opendbc_repo/opendbc/car/hyundai/hyundaicanfd.py:15-23`. The corner-radar POC parses bus 1 (`hyundai/radar_interface.py:13,21`).
- Rivian community: a secondary port gives "access to the blind spot monitoring radar". "Lukas' harness DO have the secondary port already built in so all that's needed is the pigtail adapter, whereas the Comma harnesses will need the harness mod AND the pigtail adapter". Both "will be able to be modified and doesn't require removal of the harness". "A third can bus" was reached with the secondary port. — [rivianforums thread (search-result summary)](https://rivianforums.com/forum/threads/comma-ai-openpilot-autopilot-progress-on-r1.37798/post-751877); [post 780559](https://www.rivianforums.com/forum/goto/post?id=780559) (forum-reported, not official)
- Rivian: power from the AXM in the original Comma/Lukas harness can cause intermittent ACC/radar/Driver+ errors. Comma recommends installing with the car asleep. — [Comma 3X Rivian Gen 1 community doc](https://docs.google.com/document/u/0/d/1clodGRssTx-dSPSUrQITq_vqumgWcpSUw83zdcAZR-A/mobilebasic) (search-result summary)
- VW J533 gateway cable: the comma relay "man-in-the-middles" the VW extended CAN bus (pins 7/17 on the J533 connector). The wiki warns that it is easy to wire the cable the wrong way round. — [openpilot wiki: VW J533 cable](https://github.com/commaai/openpilot/wiki/VW-J533-%28Gateway%29-Cable) (search-result summary)

### Inferences
- Assessment: the Rivian approach (harness mod that adds the CAN1 pair, plus a pigtail or Y adapter at a second connector) is the closest precedent for the Fisker MRR tap. The comma harness needs only the CAN1 wires populated on its 18-pin connector.
- Assessment: a Y tap onto an existing two-node radar link adds stub length and a third node. Do not add termination at the tap; the link is already terminated at both ends. Keep the stub short at FD data rates (2 Mbit/s), because long stubs cause ringing in the data phase.

### Gaps
- I did not read the actual Rivian pigtail/harness-mod part numbers, the Hyundai harness pinouts, or the Ford Q3/Q4 harness diagrams (they are in `commaai/hardware/harness/v3`, not fetched).
- I found no comma-sold Y-cable or "radar tap" accessory.
