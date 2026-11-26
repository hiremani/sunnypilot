# Fisker Ocean specifics: public knowledge and audit of the local sunnypilot port

Notes compiled 2026-11-13. Web facts carry their source date. Local findings cite file paths under `/Users/hiremanirmss/Fisker/sunnypilot` (read-only audit; nothing was changed, no device was contacted). "LOCAL" = verified by reading the checkout; "PRESS" = journalism; "ANECDOTE" = forum / owner complaint / aggregator, not independently verified.

Local source shorthand used below:
- `car/fisker/` = `opendbc_repo/opendbc/car/fisker/`
- `fisker.h` = `opendbc_repo/opendbc/safety/modes/fisker.h`
- `adas.dbc` = `opendbc_repo/opendbc/dbc/fisker_ocean_adas.dbc` (31 messages, the subset the car port parses)
- `mrr.dbc` = `opendbc_repo/opendbc/dbc/fisker_ocean_mrr.dbc` (43 messages, reverse-engineered radar link)
- `world.dbc` = `openpilot/third_party/webhud/dbc/fisker_ocean_adas_world.dbc` (about 100 messages cut from the "FM29 ADASBUS matrix V390.8"; streamed to the HUD only, NOT parsed by the car port)

---

## 1. ADAS architecture: suppliers, sensors, ECUs, buses, SecOC/E2E; shipped vs promised features; OS versions

### Takeaway
Magna supplied the whole ADAS system (camera, ultrasonic and the Uhnder-based "ICON" digital imaging radar); no source found ties Mobileye to the Ocean. The local FM29 matrix shows the concrete ECU set: a front camera module (FCM), one mid-range radar (MRR), four corner radars (CMRR FL/FR/RL/RR), 16 ultrasonic sensors, a driver-monitoring camera, surround view, an ADAS domain controller, with AUTOSAR "Short SecOC" on the actuator commands and CRC-8 E2E on most other frames. Fisker shipped far less than the hardware supports: ACC was still absent in OS 2.0 (Feb 2024), and many cars apparently never had ACC "coded" at all.

### Cited Findings
Suppliers (PRESS, all 2021, describe the planned programme):
- Magna agreed in early 2021 to develop the Ocean's full ADAS; package uses cameras, ultrasonic sensors and a digital imaging radar; Magna also built the car in Graz. — [Automotive World](https://www.automotiveworld.com/news/magna-expands-with-fisker-secures-full-adas-system-business/), [just-auto](https://www.just-auto.com/news/magna-supplying-digital-radar-for-fisker-ocean)
- The radar is Magna's "ICON Radar", co-developed with Uhnder (Austin), described as the first digital imaging single-chip radar; the Ocean was announced as its first production application. — [ADAS & Autonomous Vehicle International](https://www.autonomousvehicleinternational.com/news/adas/fisker-ocean-will-be-first-production-application-for-digital-radar-in-adas.html)
- "The core automotive software was developed by Magna and other suppliers" (reason the community focuses on infotainment/connectivity/diagnostic layers). — [Electrek, 2026-05-16](https://electrek.co/2026/05/16/fisker-ocean-open-source-ev-story-after-bankruptcy/)
- About 11,200 Oceans were built at Magna in Austria; about 3,200 went to American Lease. — [InsideEVs, 2026-07-13](https://insideevs.com/news/801422/fisker-ocean-dead-ev-hands-free-driving/)
- Fisker selected Argus (now PlaxidityX) CAN intrusion detection (IDPS) for all Oceans (this is an IDS, separate from SecOC). — [PlaxidityX press release](https://plaxidityx.com/company/newsroom/press-releases/us-based-ev-manufacturer-fisker-selects-argus-as-a-preferred-cyber-security-partner-for-its-ocean-suv-product-line/)
- A TechInsights teardown of the Fisker Ocean camera exists (would name the camera SoC), but the page returned HTTP 403 and could not be read. — [Electronics360](https://electronics360.globalspec.com/article/22672/techinsights-teardown-fisker-ocean-camera)

ECUs and sensors (LOCAL, from the FM29 matrix signal names):
- ADAS domain controller fault frames name every node it monitors: `FCM` (front camera module), `MRR`, `CMRR_FL/FR/RL/RR` (four corner radars), `ESP`, `iBooster`, `EPS`, `YRS`, `ACU`, `BCM`, `ECC`, `TBOX`, `MCU_F`, `MCU_R`, `PLGM`, `CIM`, `VCU`, `ICC`, `PEPS`, `TPMS`, `AVAP`, `MFS`; plus "PrivateCAN" E2E/comm-lost flags for FCM, MRR and each CMRR (so camera and radars sit on private links behind the ADAS ECU). — `world.dbc` messages `ADAS_0x481`, `ADAS_0x482`, `FCM_0x487`
- Version frames exist for the ADAS ECU, MRR and each corner radar (`ADAS_0x659`, `0x67E`, `0x680`, `0x687`, `0x689`, `0x68A`: SwVers/HwVers/PartVers). — `world.dbc`
- 16 ultrasonic sensors (`ADAS_USS_F0..3, B0..3, L0..3, R0..3` in `ADAS_0x352`) plus six front and six rear object-distance zones (`ADAS_0x356`, `ADAS_0x359`). — `world.dbc`
- Driver monitoring camera with IR: `ADAS_0x527` carries attention zone, alert state, engagement level, impairment level, drowsiness level, camera/IR blockage, driver detected; hands-on detection (`ADAS_HODSts`, `ADAS_HOD_HandsOnWarnReq`) is in `ADAS_0x342`. — `world.dbc`
- Surround view status (`ADAS_0x316 ADASDC_SVS_*`), park-assist slots (`ADAS_0x2C7..0x2EA`, six slots with corner coordinates), remote/trained parking status (`ADAS_Sts_RAP`, `ADAS_Sts_TP`). — `world.dbc`
- Camera perception published on ADASBUS: six lane lines with offset/heading/curvature/type/colour/confidence (`0x339`, `0x20A..0x20E`), lane topology (`0x20F`, `0x350`), eight fused objects with class, size, heading, brake-light flag but no relative velocity (`0x33B, 0x34B, 0x32D, 0x33D, 0x34D, 0x32F, 0x33F, 0x34F`), traffic light and sign data (`0x210`, `0x311`, `0x334`, `0x351`). — `world.dbc`
- MRR private link (panda bus 1, CAN-FD): 65 ms cycle, header `0x300` + 32 object slots `0x310..0x32F` with range, lateral, relative velocity and acceleration, class, size, heading, quality; the ADAS ECU feeds it ego motion (`0x200`, SecOC-tailed), wheel speeds (`0x201`), clock (`0x202`) and AUTOSAR CanTSyn time (`0x100`). — `mrr.dbc` header comment and `CM_ BO_` lines
- The MRR has no elevation: overhead lights/gantries and curbs appear as in-lane stationary cars, so the port requires model confirmation for radar leads (no radar-only low-speed lead). — `openpilot/selfdrive/controls/radard.py` lines 260-263

Buses and harness topology (LOCAL):
- The port uses a camera-splice style harness: panda bus 0 = vehicle-side ADASBUS (gateway side, 92 addresses seen), bus 2 = OEM ADAS module side (about 65 addresses, all ADAS-authored), bus 1 = MRR private link (receive only). All CAN-FD. The gateway mirrors body/HMI signals (gear, pedal, doors, belt, blinkers, wheel buttons) onto ADASBUS, so one bus suffices. — `car/fisker/values.py` lines 20-58
- Community view of the rest of the car: four classic buses (CCAN, PTCAN, Inverter CAN, BCAN) at 500 kbps, per the puddletools DBC project. — [Electrek, 2026-05-16](https://electrek.co/2026/05/16/fisker-ocean-open-source-ev-story-after-bankruptcy/)
- Head-unit buses IBUS1/IBUS2 exist; the gateway mirrors ADASBUS onto IBUS2 (used by the Oceanscape app through a "Pulse" add-on board). — [moistpope/oceanfront-view README](https://github.com/moistpope/oceanfront-view)

SecOC / E2E (LOCAL):
- AUTOSAR "Short SecOC" protects `0x1D0` (steering angle command) and `0x121` (accel command); `VCU_0x214`, MRR `0x200` and MRR object slot 31 also carry SecOC tails. Freshness comes from `GW_Syn_All` (0x20, 50 ms: trip counter, reset counter, MAC). Only a 6-bit freshness byte and 3 MAC bytes are on the wire. The gateway restarts the per-window counter about once per second. — `adas.dbc`, `car/fisker/values.py` lines 127-134, `car/fisker/carcontroller.py` lines 138-150
- Panda does not validate the MAC; the 16-byte AES key lives in userspace params (`SecOCKey`), is verified once against the gateway sync MAC, and no steering is sent without a verified key. — `fisker.h` lines 5-8, `car/fisker/carcontroller.py` lines 240-249, `car/fisker/interface.py` lines 40-42
- Key provisioning is automated in the port: commit titles "recover SecOC key from radar modules at startup" (2026-10-07) and "recover SecOC key over bounded ADAS ISO-TP callbacks" (2026-10-18). — git log, `car/fisker/secoc_key.py`
- Plain E2E is CRC-8 J1850 (poly 0x1D) seeded with a per-message DataID, with a 0..14 alive counter (15 = invalid); explicitly not standard AUTOSAR Profile 1. Verified DataIDs include 0x1C0=195, 0x1C2=0x90, 0x514=0xC5, 0x317=205. — `car/fisker/fiskercan.py` lines 14-45, `fisker.h` lines 239, 323

Shipped vs promised (PRESS and LOCAL):
- OS 2.0 (announced 2024-02-07) added 45:55 torque split, automatic vehicle hold, brake disc wiping, trailer sway mitigation, driver profiles; ACC was still not activated "even though customers paid for it". A braking fix had shipped in V1.10 (Dec 2023). — [InsideEVs, 2024-02-07](https://insideevs.com/features/707709/fisker-ocean-v2-ota-no-adaptive-cruise-control/)
- Fisker's CEO said missing ADAS such as ACC would be activated by software on April 1 (2024); whether that happened fleet-wide is not confirmed by the sources read. — [Automotive News Europe](https://www.autonews.com/automakers/fisker-ocean-get-key-active-safety-upgrade-april) (page not fetched; from search snippet)
- "Fisker fitted the electric crossover with the necessary hardware, but never shipped the software" (hands-free driving). — [InsideEVs, 2026-07-13](https://insideevs.com/news/801422/fisker-ocean-dead-ev-hands-free-driving/)
- The local port confirms the gap from the inside: the stock ADAS stays in `ACC_Off` unless the head unit's settings frame `ICC_0x52A` says `ICCACCSwt=On` and `ICCACCFuncTyp=Advanced`; the port rewrites that frame to unlock ACC. Some cars have no ACC coded at all and only offer the VCU's basic hold-speed cruise (`VCU_0x358`). — `car/fisker/values.py` lines 137-154, `car/fisker/carstate.py` lines 479-488
- The settings frames list what the ADAS ECU implements: LKA, ELKA, ESA, LCA/lane-change assist (`ICC_EnbLnChgAsst`), TJA auto-steer status, TSR, ISA, traffic-light recognition (TLR), FCTA, front/back collision mitigation (FACM/BACM), AEB/rear AEB, BSD, DOW, driver attention (DCAA), auto high beam (AHBA), APA park assist, remote automated parking (RAP), trained parking (TP), wheel/curb protection (WSPPA). — `adas.dbc` messages `ICC_0x52A`, `ICC_0x35B`, `ADAS_0x313`, `ADAS_0x314`
- The EPS only honours lateral type LKA (`ADAS_LatCtrl_Typ=1`); sending LCA/TJA (`Typ=3`) raises an ADAS DTC (confirmed on-vehicle). — `openpilot/selfdrive/ui/sunnypilot/layouts/settings/vehicle/brands/fisker.py` lines 41-51
- OS-version milestones: OS 2.1 was the last version Fisker was obliged to provide in an early sale draft; recall remedies were directed to "Version 2.2"; FOA's first "Marlin" install (June 2026) bundles FiskerOS 2.2 with VCU24, PDU400 and "ADAS 3.1.1". — [TechCrunch, 2024-10-16](https://techcrunch.com/2024/10/16/fisker-owners-get-help-with-recall-repairs-as-court-approves-liquidation-plan), [FOA timeline](https://fiskeroa.com/?p=446)

### Inferences
- The FM29 matrix proves the ADAS ECU computes far more than Fisker exposed (TLR, TSR, lane topology, DMS, HOD, parking slots). Everything on ADASBUS is readable by the comma for free; the sensor fusion the car already does is an untapped input.
- Because camera and corner radars are on private links behind the ADAS ECU, only the MRR raw objects (tapped) and the ADAS ECU's fused outputs are reachable; corner-radar raw data would need another tap.
- "ADAS 3.1.1" in FOA's Marlin bundle means ADAS ECU firmware is still changing in 2026; message behaviour the port relies on (0x52A unlock, fault transients, DataIDs) could differ per ADAS firmware version. The version frame `ADAS_0x659` could be logged to detect this.

### Gaps
- No source confirms the camera SoC/vendor (Mobileye or otherwise); the teardown page was blocked (403). Mobileye involvement is unverified.
- No authoritative list of which ADAS features each OS version (2.0 / 2.1 / 2.2) enabled per market/trim. A community Google Sheet tracks features by version but was not readable/verified: [sheet](https://docs.google.com/spreadsheets/d/1JIoWjOuvcD-AUEw3nWz2pN8v4Yd4Xz-U/htmlview).
- What "ADAS 3.1.1" changes is not documented in any source found.
- `fisker.h` and `carcontroller.py` mention `EPS_0x475` (EPS mode); it is in none of the three local DBCs.

---

## 2. Recalls, service bulletins, reported faults, and what they imply for sending accel/steer commands

### Takeaway
All 2024 recalls relevant to control are powertrain/brake-software issues (regen dropping out over bumps, loss of drive power from MCU/VCU software and from a failed water pump, failure to hold/shift gear), remedied by OS 2.1/2.2 software that many cars may not have fully received. The local port has independently hit a matching behaviour: the car faulted ACC/ESP/AEB when actual acceleration exceeded the request, so the port caps positive accel at 1.0 m/s2 and forbids every brake/hold/gear request.

### Cited Findings
Recalls (dates and counts from press; campaign numbers from aggregators, treat numbers as unverified until checked by VIN on nhtsa.gov):
- Regen braking: software can temporarily reduce regenerative braking while slowing over bumps; friction brakes remain; normal operation returns after 740 ms; one related injury known; 7,745 MY2023-24 vehicles; OTA remedy; Fisker ref TSB10062408. — [Green Car Reports, 2024-08-22](https://greencarreports.com/news/1144223_fisker-ocean-recalled-due-to-regen-brake-issue); NHTSA document for 24V623: [RCRIT-24V623](https://static.nhtsa.gov/odi/rcl/2024/RCRIT-24V623-6104.pdf) (not opened)
- Five recalls open as of 2024-09-16: regen braking weakens, electric water pump failure, door handles, incorrect warning lights, total loss of drive power. Fisker initially said it would pay parts only, owners labour. — [The Drive, 2024-09-16](https://www.thedrive.com/news/as-if-it-couldnt-get-worse-for-fisker-owners-now-they-have-to-pay-for-recalls)
- Sixth 2024 recall: vehicle may not shift into the selected gear and can roll away; 7,745 units; OTA software first released March 2024. — [CarScoops, 2024-11-22](https://www.carscoops.com/2024/11/fisker-ocean-hit-with-sixth-recall-this-year-this-time-over-rollaway-risk/)
- Water pump: communication failure with the cabin electric water pump can put the BMS in limp mode and cut drive power (aggregator lists 24V499000); door handles 24V466000; MCU/VCU loss of power 24V404000 (remedy OTA 2.1); warning lights 24V405000 (OTA 2.1); roll-away 24V865000. ANECDOTE-grade numbering; sources disagree on which number is the regen recall (24V623 vs 24V865). — [forcar.org](https://forcar.org/car-problems/fisker/ocean/), [carcomplaintdata](https://carcomplaintdata.com/fisker/ocean/2023/unknown-or-other)
- NHTSA opened a preliminary evaluation (April 2024) into 6,781 MY2023 cars over doors; two preliminary probes were later closed. — [AOL/Reuters](https://www.aol.com/nhtsa-closes-two-preliminary-probes-121543740.html)
- Recall software and bankruptcy: the sale order had Fisker take "commercially reasonable steps" to provide software updates, parts and recall remedies; an earlier draft said no obligation beyond 2.1. — [TechCrunch, 2024-10-16](https://techcrunch.com/2024/10/16/fisker-owners-get-help-with-recall-repairs-as-court-approves-liquidation-plan)
- Later, the American Lease / FOA cost dispute meant "a pending software recall was blocked" and owners lost connectivity. — [Electrek, 2026-05-16](https://electrek.co/2026/05/16/fisker-ocean-open-source-ev-story-after-bankruptcy/)
- ANECDOTE (owner complaints, 2025): "Software update 2.2 was not fully installed"; after a 2.2 update, frequent driver-assistance warnings and periodic loss of AEB and lane-keeping warnings; one owner flashed 2.2 with the FAST tool and was then pushed back OTA into an unstable state. — [vehicleflaws complaint 11642188](https://vehicleflaws.com/complaint/11642188), [r/Fisker thread](https://redlib.hbubli.cc/r/Fisker/comments/1kgivcn/any_tech_with_access_to_fast_to_reupdate_my_foo)
- From Nov 2024 cars began entering an "ICC boot loop" (centre screen restarts); FOA provided a fix without replacing the ICC in Jan 2025. — [FOA timeline](https://fiskeroa.com/?p=446)
- Cloud security: in May 2025 a researcher showed remote unlock/start with only a VIN and an exposed API key; the API accepted raw CAN messages. — [TechTimes, 2026-07-13](https://www.techtimes.com/articles/320358/20260713/comma-ai-delivers-hands-free-driving-fisker-ocean-via-openpilot-port.htm)

Matching behaviours observed in the local port (LOCAL, dated on-vehicle notes):
- Accel plausibility: in route `000000c6--51d13b87bd--1` the car faulted itself (VCU_ACCRdy dropped, ACC Failure, ESP/AEB faults) after accelerating at about 1.5 m/s2 for 0.6 s, 0.4-0.6 above the request; stock ACC never asked for more than about 0.9. Cap now 1.0 m/s2 in Python and panda (`FISKER_ACCEL_MAX`). — `car/fisker/values.py` lines 105-111, `fisker.h` line 193
- Panda refuses any long frame that carries park-gear, emergency-EPB, emergency/park standstill, HBA, brake prefill, AEB activation or ESP standstill requests; the Python side always sends 0. — `fisker.h` lines 592-609, `car/fisker/fiskercan.py` lines 211-255, opendbc commit 2026-11-13 "one sender per long frame; no gear, EPB or brake requests"
- Every ACC switch-off passes through `Failure_irreversible` for 20-40 ms; ACC sits in failure states 9/10 whenever out of a forward gear (2026-10-28: 3.4 s in R, 47 s parked); the cluster shows an ADAS fault telltale and "ACC fault" text/chime up to about 6 s later. The port debounces these and hides the cosmetic warnings in panda. — `car/fisker/carstate.py` lines 490-501, `fisker.h` lines 223-313
- EPS: aborts on any gap/duplicate/backward counter in 0x1D0 (AbortFb=8), does not reliably re-accept control after an abort (2026-10-19: a ~52 ms abort left it Off), trips its own driver-intervention at about 6.5 Nm, and a near-full-lock abort=9 gave a permanent LKAS fault (2026-10-27). — `car/fisker/carstate.py` lines 66-75, `car/fisker/values.py` lines 226-260, `car/fisker/carcontroller.py` lines 505-545
- Injected actuator frames reaching the OEM ADAS ECU (bus 0 to bus 2) were the "ADAS fault root cause" and are now blocked. — git log 2026-10-18

### Inferences
- Regen can vanish for about 0.74 s over bumps on cars without the recall software, so a deceleration request through 0x121 (which the VCU serves with regen, since the port forbids hydraulic requests) may be under-delivered exactly when following a slowing lead on rough road. Driver-brake fallback and a conservative following gap matter more than on other ports.
- Because the port deliberately never requests ESP standstill, hold, prefill or AEB, openpilot longitudinal has no verified way to hold the car at a stop or to use friction brakes. Stop-and-go and stop-sign stopping depend on regen plus the car's own hold behaviour; this needs measuring.
- The car's own ACC plausibility monitor (actual vs requested accel) is a real second safety layer but also a nuisance-fault source; grade, tailwind or regen-level settings could trip it below the 1.0 cap.
- Fleet software is heterogeneous (2.0 / 2.1 / partial 2.2 / FOA Marlin). A port tuned on one car may see different fault transients on another; logging VCU/ADAS versions per route would help triage.
- The loss-of-drive-power recalls (MCU/VCU software, water pump limp mode) mean openpilot should treat `VCU_ACCRdy` dropping or drive-ready loss as an immediate disengage; `VCU_ACCRdy` is decoded in the DBC but its use was not confirmed in this audit.

### Gaps
- Authoritative NHTSA campaign numbers per defect were not confirmed from nhtsa.gov; aggregator numbering conflicts.
- No recall, TSB or investigation specific to EPS, ADAS/ACC, AEB or the gateway was found.
- No public description of the VCU's accel-plausibility thresholds; the 1.0 m/s2 cap rests on one logged event.

---

## 3. Ownership and support ecosystem in 2026

### Takeaway
There is no manufacturer. The Fisker Owners Association (about 4,000 members) plus parts companies (Tsunami Automotive in the US, Tidal Wave in Europe) and community software (OceanLink Pro, a new phone-as-key stack, FOA's "Marlin" FiskerOS 2.2 install) carry the car; the cloud went dark in April 2025 after the American Lease cost-sharing arrangement collapsed, so official OTA is effectively gone.

### Cited Findings
- Fisker filed Chapter 11 in June 2024; FOA founded the same month; bankruptcy final October 2024. — [FOA timeline](https://fiskeroa.com/?p=446)
- Summer 2024: access to the FAST diagnostic tool was shut off; FOA works to keep tools available to independent workshops. — [FOA timeline](https://fiskeroa.com/?p=446)
- Aug 2024 Tsunami Automotive founded (California, parts); Feb 2025 Tidal Wave founded by owners (European parts). — [FOA timeline](https://fiskeroa.com/?p=446)
- Nov 2024 group-buy key fobs arrive; group buys cut fob prices from about $1,000; free global fob pairing events. — [FOA timeline](https://fiskeroa.com/?p=446), [Electrek, 2026-05-16](https://electrek.co/2026/05/16/fisker-ocean-open-source-ev-story-after-bankruptcy/)
- March 2025: OceanLink Pro reaches the app stores, "the first new features for the Ocean since Fisker stopped". April 2025: the connection to Fisker's cloud goes down. — [FOA timeline](https://fiskeroa.com/?p=446)
- Sept 2025: "PKC21" rolls out, new phone-as-key software by FOA and Chevalier. June 2026: first "Marlin" installation with FiskerOS 2.2, VCU24, PDU400, ADAS 3.1.1. March 2026: Wired long-form piece on the association. — [FOA timeline](https://fiskeroa.com/?p=446)
- American Lease bought the remaining roughly 3,200 cars (Oct 2024) with source code and cloud access, paying an extra $2.5M over five years for cloud services; FOA was to get access. The arrangement was "based on a handshake"; American Lease asked FOA to cover 58% of operating costs (LTE, Microsoft cloud, tools), refused itemised invoices, and the deal collapsed. — [InsideEVs, 2024](https://insideevs.com/news/736798/fisker-ocean-data-impossible-port/), [Electrek, 2026-05-16](https://electrek.co/2026/05/16/fisker-ocean-open-source-ev-story-after-bankruptcy/), [TechTimes, 2026-07-13](https://www.techtimes.com/articles/320358/20260713/comma-ai-delivers-hands-free-driving-fisker-ocean-via-openpilot-port.htm)
- FOA hired independent experts to reverse-engineer Fisker's patches, runs member-to-member firmware-flashing lessons, a "Flying Doctors" mobile repair programme in Europe, and persuaded several insurers to keep covering the cars. — [Electrek, 2026-05-16](https://electrek.co/2026/05/16/fisker-ocean-open-source-ev-story-after-bankruptcy/)
- ANECDOTE: an owner reports paying American Lease $100 for an OS upgrade and FOA $200 for diagnostics. — reader comment summarised in [Electrek, 2026-05-16](https://electrek.co/2026/05/16/fisker-ocean-open-source-ev-story-after-bankruptcy/)
- Community tooling: `MichaelOE/home-assistant-MyFisker` (cloud API integration, 135 commits, 20 releases); `puddletools/CAN` (one `FiskerDBC.dbc`, "use at your own risk", 6 commits); a multi-part Medium series by Majd Srour on sniffing CAN and decoding DTCs. — [Electrek, 2026-05-16](https://electrek.co/2026/05/16/fisker-ocean-open-source-ev-story-after-bankruptcy/), [puddletools/CAN](https://github.com/puddletools/CAN)
- Rideshare fleets (American Lease) would need insurer sign-off before fitting aftermarket ADAS hardware. — [TechTimes, 2026-07-13](https://www.techtimes.com/articles/320358/20260713/comma-ai-delivers-hands-free-driving-fisker-ocean-via-openpilot-port.htm)
- SecOC key knowledge: the only technical description found is in the local port, which recovers the key from the car itself at startup over ISO-TP (see section 1). No public article read mentions SecOC on the Ocean at all. — `car/fisker/secoc_key.py`; absence confirmed in [TechTimes](https://www.techtimes.com/articles/320358/20260713/comma-ai-delivers-hands-free-driving-fisker-ocean-via-openpilot-port.htm) and [InsideEVs](https://insideevs.com/news/801422/fisker-ocean-dead-ev-hands-free-driving/)

### Inferences
- The owner base is small (about 11,000 cars, about 4,000 organised), technically active and used to self-service flashing. Distribution through FOA channels and compatibility with FOA's Marlin builds are the realistic route to other owners.
- With the cloud gone and the head unit's map metadata blank on some cars (section 5), features that the stock car would have fed from connected services (navigation speed limits, ADASIS horizon) are natural things for the comma to supply instead.
- An aftermarket system that alters gateway traffic interacts with recall software status; the port should stay transparent when disengaged (it does: relays idle at init) so service tools and FOA flashing are unaffected. Unplugging before flashing is still prudent (a Ford owner reported a dealer update failing through a comma harness: [macheforum post](https://www.macheforum.com/site/goto/post?id=1015976), ANECDOTE, other brand).

### Gaps
- Fisker Owners Association pages on Marlin contents, pricing and compatibility were not read beyond the timeline.
- No information on whether FOA endorses or tracks the openpilot work.
- The Wired (March 2026) article was not read.
- No public documentation of harness availability or connector part numbers for the Ocean.

---

## 4. Public openpilot / sunnypilot / StarPilot work on the Ocean

### Takeaway
Publicly, the Ocean port is known through one July 2026 comma demo (credited to Majd Srour) and a small cluster of GitHub forks; it is not in upstream openpilot, and press accounts disagree on whether longitudinal works. The local checkout is a downstream of that cluster (`benkayuk` forked from `moistpope`, merging `jos-technion` branches and "Dave's StarPilot" fixes) and is considerably ahead of what the press describes.

### Cited Findings
- comma posted a video on 2026-07-12 of an Ocean steering itself with a comma four; the port and demo are credited to Majd Srour (security researcher, known in the Ocean community). The branch is custom and not merged upstream. — [TechTimes, 2026-07-13](https://www.techtimes.com/articles/320358/20260713/comma-ai-delivers-hands-free-driving-fisker-ocean-via-openpilot-port.htm), [InsideEVs, 2026-07-13](https://insideevs.com/news/801422/fisker-ocean-dead-ev-hands-free-driving/)
- Scope conflict: TechTimes and InsideEVs say steering only, "no automatic acceleration or braking"; Jalopnik says the Ocean joined comma's "fleet" with ACC, lane centering and lane-change assist, while describing the same video in which the driver sets speed with the wheel controls. — [Jalopnik, 2026-07-18](https://www.jalopnik.com/2216977/fisker-ocean-hands-free-driving-aftermarket-company/); contradicted by [TechTimes](https://www.techtimes.com/articles/320358/20260713/comma-ai-delivers-hands-free-driving-fisker-ocean-via-openpilot-port.htm)
- Cost cited: comma four $999 plus a model-specific harness, about $1,200-1,500 all-in (harness price not stated). — [TechTimes, 2026-07-13](https://www.techtimes.com/articles/320358/20260713/comma-ai-delivers-hands-free-driving-fisker-ocean-via-openpilot-port.htm)
- Press describes the mechanism as "torque commands" with checksum/counter matching; this is inaccurate relative to the code, which sends a steering ANGLE command with SecOC. — [TechTimes](https://www.techtimes.com/articles/320358/20260713/comma-ai-delivers-hands-free-driving-fisker-ocean-via-openpilot-port.htm) vs `fisker.h` lines 5-13
- `moistpope/oceanfront-view` (now "Oceanscape"): Tesla-style 3D driving view for the Ocean's centre screen, running as an Android app on a "Pulse" add-on board reading IBUS1/IBUS2; optional comma running sunnypilot adds the full ADAS bus, radar and openpilot, managed over SSH. Includes live car controls (locks, doors, lights, climate), music/navigation cards, comma camera window, CAN signal browser. 204 commits, 2 stars, 3 forks, 5 open issues; references `moistpope/sunnypilot` and `benkayuk/oceanscape`. — [GitHub README](https://github.com/moistpope/oceanfront-view)
- `benkayuk/sunnypilot` is a fork of `moistpope/sunnypilot`; its README is the stock sunnypilot README with no Fisker notes; 0 stars, 1 fork. — [GitHub](https://github.com/benkayuk/sunnypilot)
- Local git remotes: main repo, `opendbc_repo` and `panda` all point at `benkayuk/*`. — LOCAL `git remote -v`
- Local history shows the lineage (LOCAL git log, dates are commit dates):
  - 2026-09-11 "Add Fisker Ocean port to sunnypilot's opendbc fork"
  - 2026-10-02 MRR radar interface; "alpha release of alpha longitudinal"
  - 2026-10-13 "port jos-technion button mapping + ACC/lateral settings + EPS relay"; "merge jos-technion fisker-icc-spoof"
  - 2026-10-16/18/19 "merge jos-technion remote_control" (phone remote control, SecOC counter rework, race-free handover)
  - 2026-10-28 "port Dave's StarPilot steering-handover fixes", "basic cruise toggle"
  - 2026-10-09/13 "Initial implementation for an ICC MITM module", later "move the ICC gateway hardware project to its own repo"
- StarPilot evidence in code: on a comma 3X running StarPilot the original -1..+3 candidate window missed almost every substitution (EPS abort 8 about 100 ms after Active), so the window was widened to 11; a second race was "seen on-vehicle 2026-10-26 (in the StarPilot port, on a no-ACC-coded car)". — `car/fisker/carcontroller.py` lines 62-69, 211-222
- Two fingerprints exist ("Trim A" original capture, "Trim B" a second Ocean), so at least two physical cars have run this code family; the no-ACC-coded StarPilot car makes a likely third. — `car/fisker/fingerprints.py` lines 19, 49
- No shareable comma route exists for CI ("single-model port; no shareable comma route"); the car is listed in opendbc `docs/CARS.md` as "Fisker Ocean 2023-24, All". — `opendbc_repo/opendbc/car/tests/routes.py` line 30, `opendbc_repo/docs/CARS.md` line 61

### Inferences
- Several independent developers (at least the jos-technion, moistpope, benkayuk and StarPilot lines) are iterating in parallel with frequent cross-merges; the handover logic has been rewritten many times in six weeks (over 30 SecOC/handover commits 2026-10-10 to 2026-11-02). Regression risk across forks is high, and a shared test route would have high value.
- Whether "jos-technion" and the publicly credited Majd Srour are the same person is not established by any source read; do not assert it.
- Public perception ("steering only") lags the code (ACC unlock via 0x52A, alpha longitudinal, radar). Documentation for other owners (install, harness, key provisioning, which toggles to leave off) does not exist publicly and is itself a candidate deliverable.

### Gaps
- The jos-technion and StarPilot Fisker repos were not located by web search; their READMEs/issues are unread. GitHub code search was not available.
- comma's original X post and any comma Discord discussion were not read.
- The five open issues on `moistpope/oceanfront-view` and its `CAR_CONTROLS_PLAN.md` were not read.
- No Reddit/forum thread by owners describing day-to-day use was found; one older r/Fisker thread title ("Will the Ocean work with comma/openpilot") surfaced but was not read: [link](https://redlib.andus.dev/r/Fisker/comments/1ftgu89/will_the_ocean_work_with_commaopenpilot).

---

## 5. Local port: what is implemented, what is disabled, what is decoded but unused, and what the comments say remains

### Takeaway
The port is a mature lateral (angle + SecOC) implementation with a great deal of on-vehicle hardening, an ACC unlock, a working radar interface, and an alpha longitudinal path that is deliberately crippled (no brake/hold requests, +1.0 m/s2 cap). The clearest unfinished items are: longitudinal counter alignment and stop/start tuning, the abandoned EPS 0x1C2 spoof, the LCA/TJA lateral type (raises a DTC), unmapped buttons, first-cut vehicle parameters, and a large body of car perception data that reaches only the web HUD.

### Cited Findings

#### 5a. Implemented and active
- Angle steering via `0x1D0` (SecOC) + `0x1C0` activation at 100 Hz; limits 600 deg max, speed-dependent rate tables; `steerActuatorDelay 0.15`, `steerLimitTimer 2.0`, steer at standstill. — `car/fisker/values.py` lines 88-99, `car/fisker/interface.py` lines 27-37
- Race-free handover: openpilot sends a window of 11 fully stamped candidates as `0x5D0` addressed to bus 1; panda buffers them and substitutes the one matching the OEM ADAS's own counter and alive value, including pre-stamped frames for the next gateway reset window. — `car/fisker/carcontroller.py` lines 50-70, `fisker.h` lines 106-125
- MADS-driven lateral with per-button arming: right-pad North = ACC only, East = ACC + steering, long-press South = MADS toggle; panda rewrites East to North (cancel) when both are engaged and delays short South presses so a long press no longer changes the gap. — `car/fisker/values.py` lines 66-91, `fisker.h` lines 315-390
- Driver-override logic: proactive yield at 3.0 Nm held 0.1 s (live-settable 1.5-6.5 Nm via `FiskerOverrideTorque`), hysteresis release, 30 deg angle-divergence override, standstill pause below 1 mph, settle wait on resume, EPS fault latch and hard-abort disengage. — `car/fisker/values.py` lines 224-262, `openpilot/sunnypilot/selfdrive/car/fisker_takeover.py`
- ACC unlock: every `ICC_0x52A` frame is re-sent to the ADAS ECU with `ICCACCSwt=On`, `ICCACCFuncTyp=Advanced`, 5-unit speed step, lane trajectory on; sent unconditionally. Cruise state then comes from the ADAS ECU (`0x313`/`0x31C`). — `car/fisker/values.py` lines 137-154, `car/fisker/carcontroller.py` lines 747-779
- `ICC_0x35B` override relay is ACTIVE (the dict is non-empty): lane-change assist enabled, auto high beam on, wheel vibration on, APA activation = On, APA enabled, back-in/left park-out directions, park chime, curb warning, remote automated parking enabled, trained parking enabled. — `car/fisker/values.py` lines 169-190
- Blind spot from the car's own BSD (`0x314`/`0x315`), gated on BSD Available/Active; `enableBsm = True`. — `car/fisker/carstate.py` lines 435-450
- Radar: MRR object list to `RadarData` (dRel, yRel, vRel only), min state 1 / min age 3 cycles, `radarDelay 0.08`; falls back to vision-only when bus 1 is silent. — `car/fisker/radar_interface.py`, `car/fisker/interface.py` lines 54-59
- ADASIS META fix-up in panda (`0x362`): speed-limit unit, traffic side and country rewritten on the way to the ADAS ECU, following the GPS country by default (`FiskerAdasAuto`). Fixes UK cars where a 30 mph sign set ACC to 30 km/h. — `fisker.h` lines 392-430, `openpilot/sunnypilot/selfdrive/car/fisker_region.py`
- Cluster "ACC fault" telltale/text/chime hidden for transient, non-real failures only. — `fisker.h` lines 223-313
- On-device toggles: ACC Auto-Speed (default on), ACC Terrain Mode (off), LCA/TJA lateral type (off, "unsupported, raises DTC"), Engage From Basic Cruise (off). — `openpilot/selfdrive/ui/sunnypilot/layouts/settings/vehicle/brands/fisker.py`
- Web HUD bridge (`EnableWebHud`, default on, port 8088): 20 Hz raw CAN + service stream, camera view socket, replay, live CAN-override editing, SSH key management; writes refused while moving and from non-private addresses. — `openpilot/sunnypilot/webhud/README.md`
- Stop Sign Assist (map-based, sunnypilot-level, added 2026-11-13): params `StopSignAssist` and `StopSignAssistShadow` both default off. — `openpilot/common/params_keys.h` lines 311-312, `openpilot/sunnypilot/selfdrive/controls/lib/stop_sign/`
- Tooling: `openpilot/sunnypilot/tools/fisker_drive_review.py`, `openpilot/sunnypilot/webhud/tools/fisker_handoff_audit.py`, `car/fisker/relay_audit.py`, `scripts/debug_fisker_lateral.py`, `scripts/debug_fisker_secoc.py`.

#### 5b. Implemented but disabled, gated or crippled
- EPS `0x1C2` spoof: DISABLED. Checksum verified against 200 frames (2026-10-10), but the spoof still caused ADAS errors in MADS and Both modes. Hypothesis in the comment: the ADAS ECU cross-checks `EPS_0x1C4` (driver torque) and/or `EPS_0x475` (EPS mode). "Spoofing all three consistently would be the next step." Helper `create_eps_spoof_0x1c2`, the E2E entry and the carstate snapshot are kept; panda plumbing was removed. — `car/fisker/carcontroller.py` lines 667-688, `fisker.h` lines 52-57, 907
- Alpha longitudinal: available only behind `AlphaLongitudinalEnabled`; `ACCEL_MAX 1.0`, `ACCEL_MIN -3.5`; sends `0x121` (SecOC) + `0x117` + `0x118`; all park-gear / EPB / standstill / HBA / prefill / AEB / jerk requests forced to 0 and rejected by panda. — `car/fisker/interface.py` lines 44-52, `fisker.h` lines 583-609
- Longitudinal counters "still use the free-running counter for now; same alignment could help there too but is scoped separately." — `car/fisker/carcontroller.py` lines 334-336
- Longitudinal stop/start: "can be tuned via stopAccel / startAccel later once we have on-vehicle logs." — `car/fisker/interface.py` lines 47-51
- Comment inconsistency to resolve: `carcontroller.py` lines 723-727 says VCU-side ACC deceleration requires `ADAS_LgtCtrl_Typ = 2`, while `fiskercan.py` line 204 says on-vehicle testing with Typ=1 confirmed the VCU accepted positive requests, and `fisker.h` line 592 says openpilot "only ever asks for plain ACC". Which Typ is sent and whether deceleration is actually honoured was not verified in this audit.
- LCA/TJA lateral type (`ADAS_LatCtrl_Typ=3`): toggle exists, raises an ADAS DTC on the Ocean; "kept in the UI for future trims". — `brands/fisker.py` lines 41-51
- Phone remote-control steering: `remote_control_server` runs whenever the device is in a car (HTTP on 0.0.0.0:8899); the carcontroller substitutes the phone's angle only when `RemoteControlEnabled` is set, the last update is under 300 ms old and speed is under 10 mph; params are cleared at manager start. Commit history raised the angle cap to +/-250 deg. Steering only: no remote accel/brake. — `car/fisker/carcontroller.py` lines 23-48, 257-314, `openpilot/system/manager/process_config.py` lines 193-198
- Cluster HUD: openpilot sends nothing to the cluster; "the OEM keeps driving the cluster/HUD". `create_acc_hud` and `create_warning_hud` exist in `fiskercan.py` but are not called. — `car/fisker/carcontroller.py` lines 781-785, `car/fisker/fiskercan.py` lines 273, 283
- `ICC_0x52A` optional overrides left commented out: `ICCACCAutoSpdSts`, `ICCACCTerrainSetting`, `ICCACCSpdLimOffsTyp`, `ICCACCSpdLimOffs` (speed-limit offset as % or value, 0..31.5). — `car/fisker/values.py` lines 150-153
- `ICC_0x35B` optional overrides left commented out: `ICC_BSD_Sensitivity`, `ICC_DOW_Setting`, `ICC_DCAASetting`, `ICC_WarnTypeSetting`, `ICC_LKA_SettingWrnTyp`. Blind-spot settings are deliberately pass-through after a pinned value made the ADAS report BSD Off for a whole drive (route 00000028, 2026-11-02). — `car/fisker/values.py` lines 169-209
- Basic-cruise engagement for no-ACC-coded cars: off by default. — `brands/fisker.py` lines 53-60
- FW fingerprinting: none; `FW_QUERY_CONFIG` has zero requests, a placeholder EPS ECU entry exists, and `launch_env.sh` exports `SKIP_FW_QUERY=1` globally. — `car/fisker/values.py` lines 212-217, `car/fisker/fingerprints.py` lines 90-96, `launch_env.sh` lines 24-27
- `fisker.h` honours the LONG_CONTROL flag on release builds because "this is a personal test fork". — `fisker.h` lines 767-774

#### 5c. Decoded or available but unused by openpilot
- Radar object fields not passed on: `VrelLat`, `ArelLong`, `ArelLat`, `Class` (car / pedestrian-sized / small object), `Width`, `Length`, `Heading`, `DynProp` (stationary / moving / crossing), `Quality`, `*_Std` uncertainty codes. Only dRel/yRel/vRel are used. — `car/fisker/radar_interface.py` lines 92-97, `mrr.dbc` lines for `MRR_Obj00_*`
- Radar frames not decoded: `0x500 MRR_Env` blocks ("Not decoded", filled in 30-43% of cycles), `0x401`, `0x402`, `0x600` (probably mounting/calibration/version), `0x301` status. — `mrr.dbc` `CM_ BO_` 1025, 1026, 1280, 1536
- ADAS fused object list (8 objects with class, size, brake light, heading) is not used for leads because it has no relative velocity. — `car/fisker/interface.py` lines 54-56
- Signals present in `adas.dbc` and not used by carstate (verified by reading the carstate sections; a full signal-by-signal usage check was not done): `ADAS_TSRSts`, `ADAS_ISASts`, `ADAS_Sts_TLR`, `ADAS_TJA_AutoSteerSts`, `ADAS_LCA_Sts`, `ADAS_AHBA_Sts`, `ADAS_DOW_*`, `ADAS_DCAA_*`, AEB/FACM warning states in `0x31A`, `ADAS_DrvrTakeOvrReq`, `ADAS_TiGapSet_ACC`, `ADAS_ACCPrimTgtID`, `ESP_MstCylP` (dropped on purpose), `VCU_ACCRdy`, `VCU_RegenLvlFb`, `VCU_EPedlStsFb`, `VCU_DrvModSigFb`, `BCM_FrntHoodLidSts`, `BCM_HiBeamOutpCmd`, `BCM_FrntWiprSpd`, `BCM_VehAmbBri`, `ACU_BucSwtStFrntPass`, `ACU_Drvr_Occpt_St`, left-pad buttons and right roller of `MFS_0x514`.
- Everything that exists only in `world.dbc` reaches the web HUD and nothing else: speed-limit recognition (`0x311`), sign/traffic-light data (`0x210`, `0x334`, `0x351`), lane lines and topology, ADASIS v2 horizon from the head unit (`0x250`, `0x251`, `0x252`, `0x255`, `0x361`, `0x362`), driver monitoring (`0x527`), hands-on detection (`0x342`), ultrasonic sensors (`0x352`, `0x356`, `0x359`), parking slots, surround-view status, TBOX GPS/IMU (`0x174-0x179`, `0x525`, `0x526`), ADAS diagnostics (`0x481`, `0x482`, `0x487`), version frames. — `openpilot/third_party/webhud/README.md`, `world.dbc`

#### 5d. TODOs, "first-cut" values and things the comments say remain to be measured
- Buttons: "cancel / resume / gapAdjustCruise: TBD from bench testing". — `car/fisker/values.py` line 84
- `steerRatio=15.0 # first-cut; refine from step-response logs`; `tireStiffnessFactor=0.85` chosen as "crossover-typical"; mass 2410 kg "Extreme curb + typical equipment". — `car/fisker/values.py` lines 36-45
- `STEER_STEP = 1 ... verify cycle from logs`. — `car/fisker/values.py` line 101
- `GEAR_MAP` in values.py is a placeholder (`0: Ecu.unknown`), the real mapping lives in carstate. — `car/fisker/values.py` lines 60-63
- Analog brake force dropped: "if a future feature needs analog brake force, expose it through CarStateSP instead." — `car/fisker/carstate.py` lines 413-417
- Radar DBC is a "Draft, reverse-engineered from ... (61 s, low-speed parking-lot drive)"; open points: `Quality` meaning unknown, `ArelLong` scale 0.03 vs 1/32 "not resolved", `*_Std` scale unknown, date bits of `0x202` not decoded, pulse-counter wheel order "assumed". — `mrr.dbc` lines 1295-1301 and `CM_ SG_ 784`
- Lateral angle rate tables and `STEER_ANGLE_MAX 600` are "conservative"; torque override table entry `"FISKER_OCEAN" = [nan, 2.5, nan]`. — `car/fisker/values.py` lines 92-97, `opendbc_repo/opendbc/car/torque_data/override.toml` line 27
- Upstream sunnypilot TODOs touching shared car code (not Fisker-specific): gear-vs-speed precedence and 1 m/s hysteresis in `car_specific.py`; `HYST_GAP = 0.0 # currently disabled; might need to be brand-specific` in the intelligent cruise button controller; "test pcmCruise and pcmCruiseSpeed". — `openpilot/sunnypilot/selfdrive/car/car_specific.py` lines 36-37, `.../intelligent_cruise_button_management/controller.py` line 20
- Stale or contradictory comments worth cleaning up: values.py says `fisker_ocean_body` "remains in opendbc/dbc for reference" (no such file exists); `fisker.h` line 111 calls bus 1 "unused" while line 618 says it is "NOT unused"; interface.py says `ADAS_Obj0..7` while the matrix numbers them 1..8; `fisker.h` header still lists only bus 0.
- A comment and a test are dated "on-vehicle 2026-11-14", one day after the date of this audit (2026-11-13); probably a typo or timezone slip. — `car/fisker/carcontroller.py` line 408, `car/fisker/tests/test_steering_handover.py` line 182

#### 5e. Dated on-vehicle observations (chronological, all LOCAL comments)
- 2026-10-10: EPS 0x1C2 checksum (DataID 0x90) verified by brute force over 200 frames. — `car/fisker/fiskercan.py` line 39
- 2026-10-18: two EPS faults from dropping Sts/Typ to 0 while the EPS was Active; fixed by splitting Sts/Typ from Req. Disengage: EPS needs our stream to continue through a fade. — `car/fisker/carcontroller.py` lines 478-482, 638-643
- 2026-10-19: counter policy "snap once" confirmed; continuous mirroring produced duplicate frames the EPS rejects; a 52 ms abort left the EPS Off; first TX was happening about 15 ms early; ADAS logs a DTC if its settings frame is disturbed. — `car/fisker/carcontroller.py` lines 142-148, 491, 579-584; `car/fisker/carstate.py` lines 70-71; `fisker.h` line 895
- 2026-10-25: 0x1D0 counter restart is not on GW_Syn_All's reset counter; engage bursts of dropped frames when the panda's rate-limit baseline was stale. — `car/fisker/carstate.py` line 589, `fisker.h` lines 454-456
- 2026-10-26: EPS intervention trips at about 6.5 Nm; grab/release chatter at about 10 Hz; a hard overpower drives the EPS Off until lateral is de-asserted; about 180 ms panda-authorisation race at engage (StarPilot, no-ACC car); moving-engage abort burst from a command near 0. — `car/fisker/values.py` lines 227-231, `car/fisker/carcontroller.py` lines 211-225, 505-511, 553
- 2026-10-27: hand U-turn with EPS assisting kept torque under threshold; commanding into it gave abort=9 at near full lock and a permanent LKAS fault; a gateway SecOC reset landing at near full lock dropped frames. — `car/fisker/values.py` lines 255-260, `car/fisker/carcontroller.py` lines 539-541
- 2026-10-28: the host saw the OEM counter restart up to 5 frames late (20 ms card stall) and the EPS aborted; ACC failure blips on every cancel (all six in the drive) and for the whole time in R/P; "ACC fault" text up to 6.3 s later; UK head unit sends blank ADASIS META. — `car/fisker/carcontroller.py` line 75, `car/fisker/carstate.py` lines 81, 493; `fisker.h` lines 228-236, 394-396
- 2026-11-02: bumps spiked column torque past 3 Nm for 10-80 ms at least six times, each dropping steering 0.7-2.3 s, two leading to an LKAS fault on re-engage; first 0x1D0 of a burst dropped by the rate limit; East is ignored by the car while ACC is engaged; South steps the gap about 100 ms into a press; pinned BSD setting turned BSD Off. — `car/fisker/values.py` lines 207, 235-237; `car/fisker/carcontroller.py` line 567; `fisker.h` lines 317, 345
- 2026-11-14 (as written): in signalled low-speed turns the model's target ran more than 30 deg ahead of the wheel and was misread as a driver override. — `car/fisker/carcontroller.py` line 408

### Inferences
- The lateral path has absorbed most of the engineering; its remaining risk is timing sensitivity (host stalls, USB latency) rather than missing features. A panda-side counter/alive generator would remove the host from the timing loop but would need the SecOC key in firmware, which the design deliberately avoids.
- Longitudinal is where most candidate enhancements sit: counter alignment like lateral, verifying decel authority and the correct `LgtCtrl_Typ`, stop/hold behaviour without ESP standstill requests, stopAccel/startAccel tuning, and understanding the plausibility fault so the 1.0 m/s2 cap can be relaxed safely.
- The `ICC_0x35B` defaults currently force APA activation, remote automated parking and trained parking "enabled" on every drive. The code comment itself warns that `ICC_APAActivation` is a momentary user action that is held asserted when overridden. This looks like a leftover experiment and deserves a review before wider distribution.
- The remote-control HTTP server listens on all interfaces whenever in a car; it is param-gated and speed-capped, but it is an exposed steering input on a shared Wi-Fi and should probably be opt-in at process level.
- The global `SKIP_FW_QUERY=1` and the release-build LONG flag are fine for a personal fork but block upstreaming and would affect other brands if this fork were used on another car.

### Gaps
- `carcontroller.py` lines 70-240 and 330-620, `carstate.py` lines 1-400 and 570-695, `fiskercan.py` in full, `secoc.py`, `relay_audit.py` and the stop-sign controller were only sampled via grep, not read end to end; additional notes may exist there.
- Signal-level usage was not cross-checked mechanically; the "unused" list in 5c for `adas.dbc` is from reading the carstate sections and may include a signal used elsewhere (for example in panda).
- No `docs/` page in the main repo mentions Fisker (grep of `docs`, `README.md`, `CHANGELOG.md`, `RELEASES.md` returned only the opendbc `CARS.md` row).
- Whether openpilot longitudinal has been driven beyond the single faulting route is not stated in comments.

---

## 6. Unused CAN signals that suggest feasible new features

### Takeaway
The car already publishes, on the bus the comma is tapped into, its own speed-limit recognition, traffic-light and "forbidden sign" data, an ADASIS v2 map horizon, driver-monitoring and hands-on state, 16 ultrasonic ranges, lane topology and high-beam logic. None of it is consumed by openpilot today; all of it is already allowlisted to the web HUD, so the decode work is done and the remaining work is wiring it into carstate/CarStateSP and planners.

### Cited Findings
All signal names below are from `world.dbc` unless noted; none is parsed by `car/fisker/carstate.py` (that parser uses `adas.dbc`, which lacks these messages).

| Candidate feature | Signals available | Notes from the local evidence |
|---|---|---|
| Speed limit from the car's camera (feed sunnypilot Speed Limit Assist) | `ADAS_0x311`: `ADAS_TSRSpeedLimit`, `ADAS_TSRSpeedLimitAddOn`, `ADAS_SpeedLimitUnit`, `ADAS_ISA_SpdLmt`; `ADAS_0x210`: `ADAS_TSRSpdLimConfidenceLvl`, `ADAS_TSRSignCondition`, `ADAS_TSRPassingCondition`; `ADAS_0x313 ADAS_TSRSts` | The unit signal is wrong on UK cars unless the META fix-up is active (`fisker.h` lines 392-399). The stock ACC "Auto-Speed" already consumes this internally. |
| Stop sign / no-entry / traffic-light awareness | `ADAS_0x334 ADAS_FobdSign`; `ADAS_0x210`: `ADAS_TLR_EgoLaneSts`, `ADAS_TLR_EgoLaneColor`, `ADAS_TrafficLightShape`, `ADAS_TLRNumSpots`, `ADAS_TLR_WarnReq`; `ADAS_0x351 ADAS_TrafficLiDst`; `ADAS_0x313 ADAS_Sts_TLR`; setting `ICC_TLR_Setting` in `0x52A` | Could confirm or veto the map-based Stop Sign Assist added 2026-11-13. Whether TLR is active on shipped firmware is unknown; the setting bit can be forced through the existing 0x52A override table. |
| Map horizon (curve speed, limits, junctions) | ADASIS v2 from the head unit: `ICC_0x250` segment (functional road class, form of way, effective speed limit and type, lanes each direction, tunnel/bridge, built-up area, complex intersection, part of calculated route), `ICC_0x251` stub (turn angle, right of way), `ICC_0x252` long profile (sign type/value/location/condition), `ICC_0x255` short profile (two values per point, e.g. curvature/slope), `ICC_0x361` position (path index, offset, current lane, confidence), `ICC_0x362` meta | On the UK car observed 2026-10-28 the META frame was blank ("map provider N/A"), so the horizon may be empty on cars without working navigation. The reverse direction is also possible: the comma could SYNTHESISE an ADASIS horizon from OSM/mapd for the stock ADAS (0x362 has no E2E; E2E status of 0x250-0x255 not checked). |
| Use the car's driver-monitoring camera | `ADAS_0x527`: `ADAS_DrvrAttention_Zone`, `ADAS_DrvrAlert_State`, `ADAS_DrvrEngagement_Level_Status/Conf`, `ADAS_DrvrImpairment_Level_Stat/Conf`, `ADAS_DrvrDrowsiness_Level`, `ADAS_DMSDrvrDetn`, camera/IR blockage; `ADAS_0x315 ADAS_DCAA_WarnLvlReq`, `ADAS_DCAA_GazeOnWarnReq` | Useful as a second opinion or night/sunglasses fallback for comma's DM, or for logging agreement. Must not replace openpilot's own DM. |
| Hands-on detection | `ADAS_0x342`: `ADAS_HODSts`, `ADAS_HOD_HandsOnWarnReq`; `ADAS_0x31B ADAS_LaneCenteringHandsOnRew` | The port currently infers hands-on from column torque (0.5 Nm). A capacitive/HOD status would be a cleaner `steeringPressed` source if the hardware is fitted; fitment unknown. |
| Low-speed obstacle safety (parking, creep, remote steering) | `ADAS_0x352` 16 ultrasonic ranges; `ADAS_0x356`/`0x359` front and rear zone distances; `ADAS_0x2CD` curb-protection per wheel, `ADAS_APASpdWarn`; `ADAS_0x210 ADAS_WSP_LeftCurb/RightCurb` | Directly relevant to the phone remote-control feature, which today has no obstacle check, and to radar's weakness with curbs at low speed. |
| Car-computed lane model as a cross-check | Six lines (`0x339`, `0x20A-0x20E`), `ADAS_0x340` (line type/colour/distance, lane curvature), `ADAS_0x20F` real lane width, ego lane type, carpool/decel lane flags, `ADAS_0x350` neighbour-lane traffic direction, construction object type/distance | Could gate lane changes (do not change into an oncoming-direction lane: `ADAS_LeLn1TrffcDir`), or flag construction zones. |
| Lead-vehicle context | `ADAS_0x351`: `ADAS_LdngVhclBrkLi`, `ADAS_LdngVhclTurnSig`, `ADAS_HzdDetected`, `ADAS_HzdClassification`, `ADAS_HzdDst`; per-object `ADAS_VVP_ICC_ObjNBrkLght`; `ADAS_0x31C ADAS_ACCPrimTgtID`; radar `Class`, `DynProp`, `ArelLong` (`mrr.dbc`) | Brake-light and radar acceleration could give earlier decel; radar class could filter the stationary false leads noted in radard. |
| Blind-spot / door-open / cross-traffic extras | `ADAS_0x315 ADAS_DOW_*`, `ADAS_BSDLeftThreatID`; `ADAS_0x317` mirror warning requests; FCTA settings in `0x52A` | Door-open warning could raise an alert when stopped; BSD already used. |
| Auto high beam | `ADAS_0x314 ADAS_AHBA_Sts`, `ADAS_AHBA_BeamDecisRsn`; `ADAS_0x334 ADAS_AHBA_LiSigReq`; `BCM_HiBeamOutpCmd`; `ADAS_0x32B ADAS_AmbLi`, `ADAS_RoadIllmn`; setting `ICC_AHBA_Setting` already forced On | The car's own AHBA is already being enabled through the 0x35B override; exposing it as a user toggle is a small step. |
| openpilot status on the cluster | Cluster inputs that pass through panda: `ADAS_0x31C` (set speed, gap, icon, `ADAS_ACCFuncTyp`), `ADAS_0x317` (chime, warning text, takeover request, mirror warnings), `ADAS_0x340` (lane line display/flash), `ADAS_0x313 ADAS_TJA_AutoSteerSts`, `ADAS_0x314 ADAS_LCA_Sts/ADAS_LCA_Maneuver_Status`, `ADAS_0x342` turn-lamp and haptic requests | Panda already modifies `0x317` in flight with a re-computed CRC (`fisker.h` lines 286-313), so the same mechanism could set the takeover request/chime or the TJA/lane-centering icon for MADS state. `create_acc_hud`/`create_warning_hud` helpers exist unused. Risk: the cluster and ADAS may log DTCs on inconsistent states (seen 2026-10-19 with settings frames). |
| Haptic / indicator outputs | `ADAS_0x342`: `ADAS_LatCtrl_HapticReq`, `ADAS_LatCtrl_LeTurnLampReq/RiTurnLampReq`, `ADAS_LatCtrl_HzrdLampReq`, `ADAS_DoorLockReq`, `ADAS_AVASReq`, `ADAS_ExtLightReq`; `BCM_SteerWhlVbrnAlrmAvl` | Wheel vibration for lane-departure/takeover alerts and auto-indicator during assisted lane changes are possible in principle; these are body actuator requests and need care. |
| Speed-limit offset / ISA tuning from the comma UI | `ICC_0x52A`: `ICCACCSpdLimOffsTyp`, `ICCACCSpdLimOffs`, `ICCISASetting`, `ICC_TSRSetting`, `ICCUsrProfTiGapSet`, `ICC_AEBSensitivity`, `ICC_FACMSetting`, `ICC_LKASetting`, `ICCELKASteeringInterventionSet` (`adas.dbc`) | The override table and live editing already exist; only UI and validation are missing. Could also silence the stock LKA/ELKA while openpilot steers. |
| Events and gating | `BCM_FrntHoodLidSts`, per-door status, `ACU_BucSwtStFrntPass`, `BCM_0x364` rear belts/seat mats, `ACU_Drvr_Occpt_St`, `PLGM_0x471` tailgate, `BCM_FrntWiprSpd`, `BCM_0x321 BCM_ValueLightFrnt_RLS` (rain/light sensor), `ECC_0x373 ECC_OutdT` | Hood/tailgate open as no-entry; driver occupancy for remote control; wiper speed and outside temperature as rain/ice hints for following distance. |
| Powertrain state for longitudinal tuning | `VCU_0x102` motor torque requests and coast torque; `MCU_F_0x150`/`MCU_R_0x151` actual torque and slip-control mode; `VCU_RegenLvlFb`, `VCU_EPedlStsFb`, `VCU_AccelModFb`, `VCU_DrvModSigFb`, `VCU_ACCRdy`; `ESP_0x120 ESP_TotBrkTqReq`, `ESP_MstCylP`; `VCU_0x219 VCU_AxReqActv`, `VCU_VehSecureHold` | Lets the long tuning see regen level/one-pedal mode and whether the VCU is actually acting on the accel request; `VCU_ACCRdy` is the signal that dropped in the plausibility fault. |
| Health and fleet diagnostics | `ADAS_0x481`/`0x482` (per-ECU comm-lost, invalid-signal, E2E fault flags, calibration out of range), `FCM_0x487` (camera blind, not calibrated), `ADAS_0x334 ADAS_FrntCamBli/FrntCamFlt`, `ADAS_0x32B ADAS_CamSts`, `ADAS_0x311 ADAS_Hi/LoPrioDegradationSts`, version frames `0x659`-`0x68A`, `ICC_WorkshopMode`, `ICC_ADASSigFlt` | `ADAS_ComE2EFlt` and `ADAS_ChkInvldSig_ICC/EPS` would show immediately when the port's spoofed/relayed frames upset the ADAS ECU, which is today inferred from cluster symptoms. Version frames allow per-firmware behaviour tables. |
| Remote/parking functions | `VCU_0x219 VCU_RemCtrlDrvg*` (remote-control driving enable, brake and emergency-brake requests, exit flags), `ADAS_Sts_RAP`, `ADAS_Sts_TP`, `ADAS_RAPAvl`, APA slot geometry `0x2C7-0x2EA`, `BCM_APASwSts` | The car has a native remote-driving state machine in the VCU; the port's phone remote control bypasses it and steers only. Status of APA/RAP on shipped firmware is unknown. |
| Second position/IMU source | `TBOX_0x174-0x179` accel/velocity/angular rate/heading, `TBOX_0x525/0x526` GPS | Redundant to comma sensors; useful for HUD-only installs. |

- The bridge already allowlists and streams exactly these messages, and Oceanscape already decodes them into a world model with signs, lights and lanes, so signal scaling/enums have been exercised in practice. — `openpilot/sunnypilot/webhud/README.md` lines 10-17, [Oceanscape README](https://github.com/moistpope/oceanfront-view)
- Panda can already rewrite frames in flight with corrected E2E for `0x317`, `0x514`, `0x362` and `0x52A`/`0x35B`; the mechanism for cluster or settings changes is proven. — `fisker.h` lines 286-313, 384-430

### Inferences
- Highest value for least risk are read-only uses: TSR speed limit into Speed Limit Assist, ADAS diagnostics/version logging, ultrasonic gating for low-speed features, `VCU_ACCRdy` and regen state into longitudinal. They need only DBC additions to `adas.dbc` and carstate/CarStateSP plumbing.
- Medium risk are in-flight rewrites toward the cluster (status icon, takeover request), because the cluster/ADAS consistency checks have produced DTCs before.
- Highest risk are new actuator-type requests (haptic, turn lamps, door lock, VCU remote driving, ESP standstill/hold); each would need panda allow-listing and on-vehicle proof, and several are exactly what the 2026-11-13 safety commit chose to forbid.
- Because the stock ADAS ECU stays alive behind the panda, a hybrid approach is available that few other ports have: let the car's ACC do longitudinal (it has radar fusion, standstill, hydraulic brakes and recall-fixed software) while the comma shapes its inputs (set speed via buttons or speed-limit/ADASIS spoofing) instead of commanding accel directly. The existing META fix-up and "Auto-Speed" path are a working precedent.

### Gaps
- Which of these signals are populated on real cars is not established here: the matrix defines them, but TLR, HOD, DMS levels, ADASIS content and APA/RAP may be inactive on shipped firmware. A populated-signal survey over existing routes (the drive-review tool or HUD recordings) is the first step.
- E2E/SecOC status of the ADASIS frames `0x250-0x255`/`0x361` and of the cluster-facing frames other than `0x317` was not checked.
- Enum tables (VAL_) for `ADAS_FobdSign`, `ADAS_TSRSpeedLimit`, `ADAS_DrvrAlert_State` etc. were not extracted into these notes; they are in `world.dbc`.
- No public source describes anyone using the Ocean's native TSR/DMS/ADASIS data in openpilot; this appears to be unexplored outside the local HUD work.
