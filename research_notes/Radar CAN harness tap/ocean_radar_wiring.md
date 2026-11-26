# Fisker Ocean mid-range radar (MRR): private CAN-FD link wiring and tap points

Scope: 2023-2024 Fisker Ocean (FM29). Research date 2026-11-26.

Source notes:
- Main source: the **MY2023 FM29 wiring set**, a community copy of Fisker design documents ([Google Drive](https://drive.google.com/file/d/1SY0NpmfMqZCx2eLu4gMEKs2z0kN_mZQT/view)). An earlier session left a local copy at `(a temporary folder, not kept)`. The scratchpad can be wiped, so keep your own copy. The pages used are PDF p.10 (sheet III.7 "ADAS BUS"), p.16 (V.2 "Cockpit fuse box part 1"), p.24 (VI.3 "Ground engine room") and p.28 (VII.4 "ADAS").
- I read the pin data with `pdftotext -bbox-layout`, matching each word to its neighbours by position. I then checked the two key regions by rendering them to images (p.10 MRR rows; p.28 `C003B*6F` block).
- The service manual and parts book are community copies too ([service manual](https://drive.google.com/file/d/1BwRtG41yIPEU5H6KvpWaj1X8OWSvLYuf/view), [parts book](https://drive.google.com/file/d/1j-PSGpu0bNz-jYVWydXP8Ga0_e9ujeEU/view)). I read them from local text copies in the same folder.
- All three documents are 2023-era design and service data. Nobody has checked them against a car in public. Treat every pin below as "documented, not yet measured".

## Where the MRR is mounted, which part it is, and its connector pinout

### Takeaway
The forward MRR is the "Mid Range Radar - Front - Center". It clips to the **front bumper assembly**, so the bumper must come off to reach it. Its harness connector is **`S015A*1F`**, a **Molex Mini50 sealed 10-way receptacle, part 34967-1001** (the drawing gives "349671001"). The pins are: **3 = Private-CAN-H, 2 = Private-CAN-L, 9 = Cl.15 power (fuse RF33, 10 A), 6 = ground (Cl.31)**. The documents show no vehicle CAN pins on the radar connector.

### Cited Findings
- The service manual has three radar procedures in the "ADAS - Far object detection - Camera, radar & controller" section (80-5): "Mid Range Radar - Front - LH", "Mid Range Radar - Front - Center" and "Mid Range Radar - Rear - LH". Note that Fisker's service text also calls the corner radars "mid range radar". — [Service manual, contents and p.80-4/80-5](https://drive.google.com/file/d/1BwRtG41yIPEU5H6KvpWaj1X8OWSvLYuf/view)
- Front-centre removal steps: (1) remove the front bumper assembly; (2) disconnect the harness connector from the radar; (3) press the two tabs and slide the radar left to release it from the 4 clips that hold it to the front bumper assembly; (4) remove the radar. After installation, programming and calibration with the Fisker Aftersales Service Tool are required. The front-LH corner radar is different: it is reached by removing the front fender liner and clips to the front fender. — [Service manual p.80-4/80-5](https://drive.google.com/file/d/1BwRtG41yIPEU5H6KvpWaj1X8OWSvLYuf/view)
- Front bumper removal ("Bumper Assembly - Front", p.60-2 to 60-5):
  - Remove the centre headlamp assembly and both front wheels.
  - Remove the fender-liner push pins (x3 per side) and M4x15 screws (x3 per side, 2.2 Nm).
  - Release the bumper from the fender cladding on both sides.
  - **Disconnect the 2 harness connectors to the front bumper harness.**
  - Remove M5x20 bolts (x4) to the undercover (2 Nm) and M6x20 bolts (x8) to the front end (3.5 Nm).
  - Release the clips (x8 per side) to the fenders, then remove the bumper with a helper.
  - On the bench, steps 35-37 remove the mid range radar from the bumper assembly.
  — [Service manual p.60-2 to 60-5](https://drive.google.com/file/d/1BwRtG41yIPEU5H6KvpWaj1X8OWSvLYuf/view)
- Wiring sheet VII.4 shows the radar as "Mid Range Radar (MRR)", connector `S015A*1F`, housing part `349671001`, with these pins: `A*03` Private-CAN-H (pin 3), `A*02` Private-CAN-L (pin 2), `A*09` Cl.15_RF33_MRR (pin 9), `A*06` Cl.31_MRR (pin 6). — [Wiring set p.28](https://drive.google.com/file/d/1SY0NpmfMqZCx2eLu4gMEKs2z0kN_mZQT/view)
- The four corner radars use the same housing part (`349671001`) and the same pin roles: 9 = Cl.15, 6 = Cl.31, 3 = CAN-H, 2 = CAN-L. Their connectors are `S011A*1F` CMRR_FL, `S013A*1F` CMRR_FR, `S012D*1F` CMRR_RL and `S014D*1F` CMRR_RR. — [Wiring set p.10, p.16, p.28](https://drive.google.com/file/d/1SY0NpmfMqZCx2eLu4gMEKs2z0kN_mZQT/view)
- Molex 34967-1001 is a "Mini50 Sealed Receptacle, Single Row, with CPA, 10 Circuits, Polarization Option A, Black". Pitch is 1.80 mm and the operating range is -40 to +125 °C. — [findchips/Molex](https://www.findchips.com/detail/349671001/Molex); [RS 34967-1001](https://uk.rs-online.com/web/p/automotive-connectors/1367814)
- Mini50 sealed receptacles take the separately sold **CTX50 sealed receptacle terminals**. 34905-2443 and 34905-2444 are named as mating terminals. One Molex datasheet gives a wire range of 0.22-0.35 mm² for series 34905, but the table is garbled. — [Molex Mini50 datasheet via Farnell](https://www.farnell.com/datasheets/2287526.pdf); [Digi-Key Mini50 sealed](https://www.digikey.com/en/product-highlight/m/molex/mini50-sealed-wire-to-device-receptacle)
- Power: fuse **RF33, 10 A mini fuse, cockpit fuse box**, circuit "Cl.15_RF33 MPC/MRR/CMRR". It feeds the MRR (wire W0763, BN/GN, 0.35) through inline `I139B*3` pin 6 (LHD) and `I136A*1` pin 18. The same fuse feeds all four corner radars and an "MPC". — [Wiring set p.16](https://drive.google.com/file/d/1SY0NpmfMqZCx2eLu4gMEKs2z0kN_mZQT/view)
- Ground: `S015A*1F` pin 6, wire `Cl.31_EB02 (W1424)`, BK/YE, 0.35, runs in the front bumper harness to ground point EB02. — [Wiring set p.24](https://drive.google.com/file/d/1SY0NpmfMqZCx2eLu4gMEKs2z0kN_mZQT/view)
- Parts book, ADAS group: `FM2980140080B MID-RANGE_RADAR` (qty 4) and `FM2980140060A FORWARD_MILLIMETER_WAVE_RADAR_BRACKET` (qty 1). No other radar part number appears. — [Parts book, 80 Electrical / 12 ADAS](https://drive.google.com/file/d/1j-PSGpu0bNz-jYVWydXP8Ga0_e9ujeEU/view)
- Supplier: Magna supplied the whole Ocean ADAS. Its "ICON" digital imaging radar, co-developed with Uhnder, was announced with the Ocean as the first production application. — [ADAS & Autonomous Vehicle International](https://www.autonomousvehicleinternational.com/news/adas/fisker-ocean-will-be-first-production-application-for-digital-radar-in-adas.html); [just-auto](https://www.just-auto.com/news/magna-supplying-digital-radar-for-fisker-ocean); [BusinessWire, 2022-01-05](https://www.businesswire.com/news/home/20220105005304/en/Fisker-Inc.-Reveals-Worlds-First-Digital-Radar-in-a-Production-Vehicle-Bringing-State-of-the-Art-Safety-to-Fisker-Ocean-All-Electric-SUV)

### Inferences
- The MRR is a 4-wire device on a 10-way housing: only pins 2, 3, 6 and 9 are drawn. The other 6 cavities are probably plugged. The drawing does not say so.
- The MRR's power is Cl.15 (ignition-switched), so the radar and its link are live only with the car on. Plan bench and car work with that in mind.
- The parts book lists 4 radars under one part number, but the car has 5. So either the forward MRR has a different part number that the parts book omits, or the qty is wrong. **Which radar is the Magna/Uhnder ICON unit is not documented.** The forward MRR is the likely one, but this is a guess.
- The radar connector is the worst tap point. It sits behind the bumper, is sealed, and is exposed to water and impact. Opening it means removing the bumper and then recalibrating the radar if the radar is disturbed.

### Gaps
- No public FCC or supplier filing tying a Magna or Uhnder model number to FM2980140080B was found in this pass. A search of FCC IDs for Magna Electronics 76-81 GHz radars is the next step.
- No photo or end view of `S015A*1F` was viewed. The cavity numbering follows Molex's drawing for 34967-1001, which I did not open.
- The exact CTX50 terminal part number for 0.35 mm² wire is not confirmed.

## Where the private link ends: ADAS controller connector, pins, wire colours and route

### Takeaway
The MRR pair ends at the ADAS controller ("Hydra3", FM2980140120B, cargo area behind the lower side trim) on connector **`C003B*6F`**: **pin 9 (`A*9B`) = Private_MRR_CAN-H** and **pin 1 (`A*1B`) = Private_MRR_CAN-L**. The harness housing is **Molex 34729-0162** (Stac64, 16-way, dual row, polarization C, brown). **This corrects the earlier camera-tap report, which gave `560123-0200` for `C003B*6F`; that number belongs to the driver-camera LED connector `V009B*1F`.**

The wires are **WH/BN (CAN-H)** and **GY/BN (CAN-L)**, wire type "2TAD", **0.35 mm²**, in three harness sections:
- body harness W2107, `C003B*6F` to inline **`I141B*1` pins 77/85**;
- engine-room harness W2105, to inline **`I136A*1` pins 10/11**;
- front-bumper harness W2123, to `S015A*1F` pins 3/2.

### Cited Findings
- `C003B*6F` block on sheet VII.4:
  - corner radar pairs: FL CAN-H/L pins 10/11, RL 12/13, FR 2/3, RR 4/5;
  - **MRR CAN-H pin 9 (`A*9B`), CAN-L pin 1 (`A*1B`)**;
  - DMS_LED1_H/L on pins 16/15;
  - housing part `34729-0162`, printed at the foot of the block. I checked the rendered image: each block's part number sits under its last pin, the same as `AMK12D-1M4Z5-D` under `C003B*5F`.
  — [Wiring set p.28](https://drive.google.com/file/d/1SY0NpmfMqZCx2eLu4gMEKs2z0kN_mZQT/view)
- `560123-0200` is printed under the driver-monitoring camera's LED connector `V009B*1F` (LED+ pin 2, LED- pin 1). It does not belong to the controller. — [Wiring set p.28](https://drive.google.com/file/d/1SY0NpmfMqZCx2eLu4gMEKs2z0kN_mZQT/view); this contradicts [local report, "Fisker ADAS controller camera tap.md", connector table](/Users/hiremanirmss/Fisker/sunnypilot/reports/Fisker%20ADAS%20controller%20camera%20tap.md)
- Molex 34729-0162 is a "Stac64 Receptacle Housing, 16 Circuits, Dual Row, Polarization C, Brown", 2.54 mm pitch. — [Molex part detail](https://www.molex.com/en-us/products/part-detail/347290161) (sibling listing); [Octopart 34729-0162](https://octopart.com/part/molex/34729-0162); [Digi-Key 34729-0161](https://www.digikey.com/en/products/detail/molex/0347290161/2520780)
- Stac64 34729 receptacles take **CTX64 34803-series female terminals**. 0348033212 is a CTX64 tin terminal "Use With 34729". Its hand-crimp tool is listed for 0.50/0.75/0.85 mm² and 20 AWG, so check the variant for 0.35 mm². The receptacles mate with 34690/34691 headers. — [Molex 0348033212 drawing via Mouser](https://www.mouser.com/datasheet/2/276/0348033212_CRIMP_TERMINALS-701701.pdf); [RS Stac64 34729](https://uk.rs-online.com/web/p/automotive-connectors/7241438)
- Route on the ADAS BUS sheet:
  - `C003B*6F` 9/1 to `CANFD_MRR_CAN-H (W2107_1)` WH/BN 2TAD-0.35 and `CANFD_MRR_CAN-L (W2107_2)` GY/BN 2TAD-0.35, zone BODY (segment marked LHD 4055 / RHD 4094);
  - then `I141B*1M/F` pins 77/77 and 85/85;
  - then W2105_1/_2 (same colours), zone ENGINE (LHD 2088 / RHD 2134);
  - then `I136A*1M/F` pins 10/10 and 11/11;
  - then W2123_1/_2 (same colours), zone BUMPER_F (field 1633.0);
  - ends at `S015A*1F` pin 3 (H) and pin 2 (L).
  - Twisted-pair symbols are drawn at both ends.
  — [Wiring set p.10](https://drive.google.com/file/d/1SY0NpmfMqZCx2eLu4gMEKs2z0kN_mZQT/view)
- The CAN pairs carry the prefix "CANFD_" on this sheet, and "Private_" at the controller. The corner radars use different colour pairs: FL GN/VT + YE/BU, FR GN/BN + GY/OG, RL WH/VT + WH/OG (from the same sheet). The ADAS vehicle CAN is BN/BU + GY/BU. — [Wiring set p.10](https://drive.google.com/file/d/1SY0NpmfMqZCx2eLu4gMEKs2z0kN_mZQT/view)
- Controller location and access: rear compartment behind the lower side trim. The steps are:
  1. Disconnect the 12 V battery.
  2. Remove the lower trim.
  3. Remove 8 M6x20 Torx screws (5 Nm) and the trim bracket.
  4. Unplug the 6 harness connectors.
  5. Remove 3 M6 nuts (9 Nm).

  The manual is unclear about the side (RH vs LH). The controller's fuse is MF07 (15 A) in the rear trunk fuse box. — [Service manual p.80-1](https://drive.google.com/file/d/1BwRtG41yIPEU5H6KvpWaj1X8OWSvLYuf/view), as summarised in [local camera-tap report](/Users/hiremanirmss/Fisker/sunnypilot/reports/Fisker%20ADAS%20controller%20camera%20tap.md)
- The ADAS vehicle CAN is on `C003B*1F` pins 20 (H) and 19 (L). Supply Cl.30_MF07_ADAS is pin 1A and ground Cl.31_ADAS is pin 14A. "A"-suffixed pins are `C003B*1F` and "B"-suffixed pins are `C003B*6F`. — [Wiring set p.28](https://drive.google.com/file/d/1SY0NpmfMqZCx2eLu4gMEKs2z0kN_mZQT/view)

### Inferences
- **The best tap point is at `C003B*6F` in the cargo area, pins 9 and 1, WH/BN and GY/BN.** The reasons:
  - No bumper removal and no radar recalibration are needed.
  - The port's camera-splice harness already sits on the same controller (ADAS CAN on `C003B*1F` pins 20/19), so a bus-1 lead runs only a short distance.
  - The colour pair WH/BN + GY/BN is unique among the five radar pairs, which makes the right pair easy to find by sight.
- The "LHD/RHD" numbers are most likely segment lengths in mm. That gives about 4.1 m (body) + 2.1 m (engine room) + 1.6 m (bumper), so roughly **7.8 m** of pair from radar to controller. This is a guess from the layout; the sheet does not label the unit.
- "2TAD" appears on power wires too (for example 2TAD-1.0, 2TAD-1.5), so it is a wire-type code, not a twisted-pair marker. The twist is shown by the drawing symbol, not by the code.
- The inline connectors `I141B*1` (body to engine room) and `I136A*1` (engine room to bumper) give two more access points. Where they sit in the car was not found. `I136A` is very probably one of the "harness connectors (x2) from front bumper harness" in the bumper removal procedure, but that is not stated.
- A passive tap is the safer method. Splice into or probe the pair at the controller without cutting it. A CAN-FD listener in parallel adds a stub; at about 7.8 m of link, keep the stub very short. The termination topology (whether both ends terminate inside the radar and the controller) is not documented.
- A safe way to confirm the pins before tapping (a proposal, not tested):
  1. With the 12 V battery disconnected and `C003B*6F` unplugged, check continuity from the harness-side cavity 9/1 to the radar end, if the bumper is off, or to `I136A*1` pins 10/11.
  2. With `C003B*6F` connected and the car on, back-probe pins 9 and 1 with fine probes, never piercing the insulation. Each line should show CAN levels: about 2.5 V recessive; H rising toward 3.5 V and L falling toward 1.5 V when dominant.
  3. On an oscilloscope, look for frames with a faster data phase.
  4. Check the resistance across H and L with the car off: about 60 Ω means two 120 Ω terminations.

### Gaps
- No connector end view or cavity map for `C003B*6F` (that is, where cavity 9 and cavity 1 are on the brown 16-way face) was found. Use the Molex 34729 drawing to locate them, then confirm by continuity.
- The physical locations of `I141B` and `I136A` are not in the extracted text. The wiring set's harness-layout pages, if any exist, were not read.
- The MY2024 wiring may differ. Only the MY2023 "SORP" set was used.

## Who captured the MRR frames, and how the link was tapped

### Takeaway
The MRR DBC and radar interface were written by **duncan999007 (``)**. Commits under that author identity appear in the **moistpope/oceanfront-view** repository (the Oceanfront View HUD). The DBC credits route `6bdf6a23860be7bd/000000b5--bfe13ac451` (61 s, low-speed parking-lot drive) as its source. The physical method is **not documented in any public source found**. The port's code only says that "the harness taps" the radar link on panda bus 1.

### Cited Findings
- DBC header comment: "Fisker Ocean mid-range radar (MRR) private CAN-FD bus (panda bus 1). Draft, reverse-engineered from route 6bdf6a23860be7bd/000000b5--bfe13ac451 (61 s, low-speed parking-lot drive). Not in the FM29 ADASBUS matrix." Most frames carry E2E CRC-8 (polynomial 0x1D). — [local `opendbc_repo/opendbc/dbc/fisker_ocean_mrr.dbc` line 1295](/Users/hiremanirmss/Fisker/sunnypilot/opendbc_repo/opendbc/dbc/fisker_ocean_mrr.dbc)
- opendbc commit `05830f17` (2026-10-02), author duncan999007 `<>`: "fisker: add mid-range radar (bus 1) radar interface … radarUnavailable now follows whether the bus-1 header is present, so harnesses without the tap stay vision-only. radarDelay 0.08 s (measured). DBC moved here from the web HUD." — local `git log` of `opendbc_repo` (upstream [benkayuk/opendbc](https://github.com/benkayuk/opendbc))
- The DBC first appeared in moistpope/oceanfront-view commit `8da8dc4` "webhud: add mid-range radar support and object tracking display" (2026-10-02 01:52 UTC). The commit metadata names the author as duncan999007 / ``. The follow-up commit `cd97803`, "fisker: integrate MRR radar", has the same author. — [GitHub API, moistpope/oceanfront-view commit 8da8dc4](https://github.com/moistpope/oceanfront-view/commit/8da8dc4971c9c084043acdba46e146d487e78874); [commit cd97803](https://github.com/moistpope/oceanfront-view/commit/cd97803fc41bd0d8fd49f0cfb35de9c6c2445229)
- Port comment: "Camera-splice harness: panda sits between the vehicle-side gateway (bus 0) and the OEM ADAS module (bus 2). Bus 1 is the mid-range radar's private link, when the harness taps it … radar = 1 # MRR private link (CAN-FD), receive only: safety never forwards or sends on it." — [local `opendbc_repo/opendbc/car/fisker/values.py` lines 49-56](/Users/hiremanirmss/Fisker/sunnypilot/opendbc_repo/opendbc/car/fisker/values.py)
- Interface comment: "Without that tap (older harnesses/logs) bus 1 is silent and openpilot stays vision-only." — [local `opendbc_repo/opendbc/car/fisker/interface.py` lines 56-59](/Users/hiremanirmss/Fisker/sunnypilot/opendbc_repo/opendbc/car/fisker/interface.py)
- Frame content and timing: the ADAS controller sends 0x100 (CanTSyn time, 10 Hz), 0x200 (ego motion, 100 Hz, SecOC tail), 0x201 (wheel speeds, 50 Hz) and 0x202 (clock, 1 Hz) to the radar. The radar sends 0x300 (header) and 0x310-0x32F (32 object slots, 48 bytes each) every 65.01 ms. — [local DBC `CM_ BO_` lines 1296-1302](/Users/hiremanirmss/Fisker/sunnypilot/opendbc_repo/opendbc/dbc/fisker_ocean_mrr.dbc)
- Press coverage of the Ocean openpilot port (July 2026) describes a model-specific harness that intercepts the steering CAN messages. It does not mention the radar. — [TechTimes, 2026-07-13](https://www.techtimes.com/articles/320358/20260713/comma-ai-delivers-hands-free-driving-fisker-ocean-via-openpilot-port.htm); [autoevolution](https://www.autoevolution.com/news/unfortunate-fisker-ocean-owners-get-an-unexpected-gift-that-could-make-their-cars-drive-autonomously-272810.html)

### Inferences
- The "moistpope" GitHub account and the "duncan999007" author identity are very probably the same person, because both carry the same email and the commit timestamps match. GitHub has no user "duncan999007" (API 404). This is an inference from metadata. Ask the author directly (oceanfront-view issues) for the harness details.
- The tap probably sits at the ADAS controller in the cargo area, because the comma's camera-splice harness already sits there (bus 0 / bus 2 split on the ADAS CAN) and panda bus 1 is the spare transceiver. This is a guess; it has not been confirmed.
- The ADAS controller sends frames to the radar (0x100-0x202). So any capture point on the pair sees both directions. One listen-only node is enough, and it must not acknowledge or transmit (the port's safety says it never sends on bus 1).

### Gaps
- No public write-up, photo or pinout of the community harness's radar lead was found. Search terms tried on the web and GitHub: Fisker Ocean harness radar tap; MRR issues in benkayuk and moistpope repos.
- It is not known whether route `6bdf6a23860be7bd` is the author's car or someone else's.

## Bit rates of the link

### Takeaway
**Not documented.** The link is definitely CAN-FD: the frames are 12-48 bytes long, and the wiring prefix is "CANFD_". The port receives it on panda bus 1 with no Fisker-specific speed setting found, and panda's default bus speeds are **500 kbit/s arbitration and 2 Mbit/s data**. That points to 500k/2M, but this is an inference until you measure it.

### Cited Findings
- The wiring sheet labels the pair "CANFD_MRR_CAN-H/L". — [Wiring set p.10](https://drive.google.com/file/d/1SY0NpmfMqZCx2eLu4gMEKs2z0kN_mZQT/view)
- The DBC frame lengths are 16, 32, 20, 12, 32, 4 and 48 bytes, which is more than classic CAN's 8-byte limit. — [local DBC `BO_` lines](/Users/hiremanirmss/Fisker/sunnypilot/opendbc_repo/opendbc/dbc/fisker_ocean_mrr.dbc)
- Panda's default bus configuration for buses 0-2 is `.can_speed = 5000U, .can_data_speed = 20000U`, in units of 100 bit/s, so 500 kbit/s and 2 Mbit/s. — [local `panda/board/drivers/can_common.h` lines 129-131](/Users/hiremanirmss/Fisker/sunnypilot/panda/board/drivers/can_common.h)
- A search of the Fisker car code and `selfdrive` for a per-car speed setting found none. Treat this as weak evidence, because the search may have missed something.

### Inferences
- To confirm without new hardware, read `pandaStates[].canState1` (`canSpeed`, `canDataSpeed`, `canfdEnabled`, `brsEnabled`, and the error counters) from an rlog in which bus 1 carries MRR frames. Zero bus-1 errors at 500/2000 confirms the rates.
- With a scope on the pair, the bit times are about 2 µs (arbitration phase) and about 0.5 µs (data phase) if the rates are 500k/2M.

### Gaps
- No Fisker or Magna document states the MRR link's nominal or data bit rate, its sample point, or whether bit-rate switching is used.
