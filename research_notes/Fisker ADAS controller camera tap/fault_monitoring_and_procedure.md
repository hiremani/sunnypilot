# Fisker Ocean ADAS controller: surround-camera link monitoring, live indicators, and a staged tap trial

Research date: 2026-11-23. Read-only work: nothing was run on the car or the comma, and no repository file was changed.

**Evidence labels used below**
- **[LOCAL]**: read in a local file; path and line given.
- **[PRIMARY]**: I downloaded and read the public source itself this session (dtc.json parsed in full; DTC manual PDF sheets 305 to 309 and 2546 text-extracted, sheets 305 and 2546 also rendered and read as images; Fiskeretta repository read as text).
- **[PAGE]**: a web page read through a summarising fetch; wording is close but not guaranteed verbatim.
- **[SNIPPET]**: search-result summary only.
- **[PRIOR]**: carried from the earlier Pulse research notes, not re-verified this session.
- **[INFERENCE]**: my reasoning, not stated by any source.

**Short source names**
- dtc.json = https://fisker-ocean-archive.preetham-981.workers.dev/data/dtc.json (4,040 entries, 64 with module ADAS)
- DTC manual = https://drive.google.com/file/d/17oA-uFKGrGGDLmGYosG-R3d4jGJz1vW_/view (4,058 PDF sheets)
- world.dbc = /Users/hiremanirmss/Fisker/oceanscape/dbc/fisker_ocean_adas_world.dbc
- ibus.dbc = /Users/hiremanirmss/Fisker/oceanscape/dbc/fisker_ocean_ibus.dbc
- adas.dbc = /Users/hiremanirmss/Fisker/sunnypilot/opendbc_repo/opendbc/dbc/fisker_ocean_adas.dbc
- port = /Users/hiremanirmss/Fisker/sunnypilot/opendbc_repo/opendbc/car/fisker/

**Two corrections to the earlier notes, found this session**
1. The "features lost" text and the monitor rate for the camera codes are in dtc.json only (its `limp` and `trigger` fields; `sources` is "manual+list"). The DTC manual sheets 305 to 309 carry the failure criteria, causes and test steps, but no limp or trigger table. The earlier notes cited the PDF sheets for the feature loss.
2. The open-source Fiskeretta tool now lists a diagnostic address for the ADAS controller (0x7A6 / 0x7AE) in its known-ECU table. The earlier notes recorded ADAS as "not yet reached".

---

## 1. DTCs: exact codes, set conditions, maturation, healing, and listed effects

### Takeaway
Each surround camera has one "Component Failure" code (C21A109 rear, C21A209 front, C21A309 left, C21A409 right) that sets on any of eight criteria, most with a 500 ms persistence, checked every 100 ms. The data names the lost features (park features for all four; RAEB in addition for the rear camera), but it does **not** state the reset (healing) or aging conditions for these codes, so whether they latch is not documented.

### Cited Findings

**The four surround-camera codes (module ADAS, internal name "Hydra3")**

| DTC | UDS 3-byte value | Title in the data | Camera per the body text | Channel | Sheet (site page / PDF sheet) |
|---|---|---|---|---|---|
| C21A109 | 0x61A109 (FTB 6398217) | Rear Camera - Component Failure | Rear | 3, `CAM_CS_A`, `CAM3_PWR_EN`, pin `IN3` | 331 / 305 |
| C21A209 | 0x61A209 (FTB 6398473) | Front Camera - Component Failure | Front | 4, `CAM4_PWR_EN`, `IN4` | 332 / 306 |
| C21A309 | 0x61A309 (FTB 6398729) | "Front Camera - Component Failure" (mistitled) | Left side | 2, `CAM2_PWR_EN`, `IN2` | 333 / 307 |
| C21A409 | 0x61A409 | Right Side Camera - Component Failure | Right side | 1, `CAM1_PWR_EN`, `IN1` | 334 / 308 |

- Codes, titles, channel text and site page numbers. [PRIMARY] — [dtc.json](https://fisker-ocean-archive.preetham-981.workers.dev/data/dtc.json)
- FTB decimal values and the same failure-event text on PDF sheets 305 to 307 (308 follows the same layout). [PRIMARY] — [DTC manual](https://drive.google.com/file/d/17oA-uFKGrGGDLmGYosG-R3d4jGJz1vW_/view). The hex conversion (6398217 = 0x61A109, where 0x61 encodes "C21") is my arithmetic.

**Set conditions, identical for all four apart from the channel number.** "The Hydra3_Component shall set the DTC if any one of the following criteria is met" [PRIMARY] — [dtc.json](https://fisker-ocean-archive.preetham-981.workers.dev/data/dtc.json); [DTC manual sheet 305](https://drive.google.com/file/d/17oA-uFKGrGGDLmGYosG-R3d4jGJz1vW_/view):
1. Open circuit or short to battery on the camera power channel: `INx = L, Fault = L for 500 ms`.
2. Short to ground: `INx = H, Fault = L for 500 ms`.
3. Thermal shutdown: sensor temperature above 125 deg C for 500 ms (AR0233AT image sensor, register R0x3E9A).
4. "Lock failure on deserializer side for 500 ms."
5. "Not receiving any image, images are frozen, images are not being refreshed for 500ms."
6. "CRC Error (Read thru register over I2C)". **No time or count threshold is stated.**
7. "Calibration data is not configured/Alignment is Incorrect or not completed."
8. "Calibration data cannot be read from the flash."

- Monitor type "Continue", monitor rate "100 msec". [PRIMARY] — [dtc.json `trigger` field](https://fisker-ocean-archive.preetham-981.workers.dev/data/dtc.json)
- **Reset and aging: not stated.** The `trigger` field for all four ends with the column headings "Trigger Reset Age" and no content. [PRIMARY] — [dtc.json](https://fisker-ocean-archive.preetham-981.workers.dev/data/dtc.json). The PDF sheets have no trigger table at all; sheet 305 as rendered shows only Description, Subsystem Principle, Failure Event, Possible Causes, Reference Information, Diagnostic Test Steps and Note. [PRIMARY] — [DTC manual sheet 305](https://drive.google.com/file/d/17oA-uFKGrGGDLmGYosG-R3d4jGJz1vW_/view)
- Listed test steps for rear and front: "1. Check Harness 2. Ensure correct camera functionality 3. Restart the system". Left and right say "TBD". All carry the boilerplate "Ensure the customer has not installed any unauthorized accessories". [PRIMARY] — [dtc.json](https://fisker-ocean-archive.preetham-981.workers.dev/data/dtc.json)

**Listed effects (dtc.json `impact` and `limp` fields)** [PRIMARY] — [dtc.json](https://fisker-ocean-archive.preetham-981.workers.dev/data/dtc.json)

| DTC | Views blanked / grayed out | "Features Not available" |
|---|---|---|
| C21A109 rear | Rear view, Rear cross path view, Free floating view | **Automatic Emergency Braking - Premium Rear Automatic Emergency Braking: RAEB**; Full Automatic Park Assist System (APA); Personal Park Assist; Remote Parking Assist |
| C21A209 front | Front view, Front cross path view, Free Floating view | Full APA; Personal Park Assist; Remote Parking Assist |
| C21A309 left | Left side videos, Left side split (curb) view, Left flank 3D view, free floating view | Full APA; Personal Park Assist; Remote Parking Assist |
| C21A409 right | Right side videos, Right side split (curb) view, Right flank 3D view, free floating view | Full APA; Personal Park Assist; Remote Parking Assist |

- The text says the affected feed is "grayed out in all Views", so the display stays up with one feed blanked. [PRIMARY] — [dtc.json](https://fisker-ocean-archive.preetham-981.workers.dev/data/dtc.json)
- RAEB is also listed as lost for every ultrasonic-sensor code (C219301 to C219F09), together with APA, Personal Park Assist and Remote Parking Assist, with ACC "degraded". So RAEB loss is not unique to the rear camera. [PRIMARY] — [dtc.json](https://fisker-ocean-archive.preetham-981.workers.dev/data/dtc.json)

**Related codes**

| DTC | Module | What sets it | Timing | Listed effect | Reset / age |
|---|---|---|---|---|---|
| C21A609 | ADAS | Driver (DMS) camera: power faults on channel 5 (`CAM_CS_B`, `CAM5_PWR_EN`) for 500 ms, or OV2311 flags (status, fail_safe, temperature, frame_counter, i2C_communication) for 500 ms | 100 ms monitor | Driver drowsiness and attention warnings unavailable | Not stated |
| C21A709 | ADAS | DMS IR LED driver (MAX25614AB) flags for 500 ms | 100 ms | Same as C21A609 | Not stated |
| C21A531 | ADAS | Any listed FCM Ethernet PDU missed for 500 ms (10 x 50 ms cycle): Light_Scene, Objects, Road, TFL, TSR bundles, Core_Objects_protocol_1024/1025 | 50 ms monitor | FID list: LCA, LCC, AHB, ISA, TSR, TLR, ESA, LKA, LCW "Critical"; ACC "High"; AEB/VRU/LTAP FCW "Medium" | Not stated |
| C21A586 | ADAS | FCM PDU checksum or counter errors, 3 consecutive | — | Features relying on FCM degraded | Not stated |
| C21A500 | ADAS | FCM Ethernet time-sync error for 500 ms | — | Same | Not stated |
| C226D00 | **FCM** | Ethernet SecOC: secured PDU not verified within 0.7 ms, or MAC failure on 3 consecutive messages. "Maturity time: Immediately" | — | FCM cannot verify or generate Ethernet PDUs | Not stated |
| P1A0502 (0x1A0502, FTB 1705218) | **ICC** | "ICC ADAS Box Communication error": after Android boots, checked every 20 s; SoC BSP reports missing data, checksum error, data overflow on the LVDS link from the ADAS box | Enabled 120 s after IGN ON, 9 to 16 V, 20,000 ms cycle | "Record DTC. Camera cannot work" | Not stated |
| U15B087 | ADAS | Listed ICC CAN messages missing for 200 ms | 20 ms monitor | "All features will be disabled" | Reset: messages received again for 200 ms. Age: 40 Drive Cycle |
| U15B586 | ADAS | ICC_0x52A checksum or counter errors, 3 consecutive | 10 ms | Features relying on ICC messages degraded | Not stated |
| U15B788 | ADAS | CAN bus off for 30 ms or more | 10 ms | "Function degradation - Rear View Only - No Dynamic Overlays ..."; after 2000 ms no view, shutdown | Reset: bus normal for 10 ms. Age: 40 Drive Cycle |
| U15B616 / U15B617 | ADAS | Battery below 8.5 V for 10 s (616) / above threshold (617) | 1 s | "No ADAS feature is available"; "No output video/signal" | Reset: at or above 8.5 V for 10 s. Age: 40 Drive Cycle |
| U15B896 | ADAS | Internal error: SoC temperature out of -40 to 125 deg C for 5 s, NVM access, RAM test | 10 ms | "No ADAS feature is available"; "No output video" | Not stated |
| C21A855 / C21A856 | ADAS | Controller not coded / invalid coding | At start-up | Low-speed safety features unavailable, others degraded | Age: 40 Drive Cycle |
| U1A3D87, U315187 | ICC | ICC lost CAN with ADAS box (10 signal periods) | From 2 s after ACC ON | "Part of ICC / ADAS function fails" | U315187: reset on one message received; age 40 ignition cycles |

All rows [PRIMARY] — [dtc.json](https://fisker-ocean-archive.preetham-981.workers.dev/data/dtc.json). P1A0502 failure text also confirmed on [DTC manual sheet 2546](https://drive.google.com/file/d/17oA-uFKGrGGDLmGYosG-R3d4jGJz1vW_/view).

- The 64 ADAS codes include SecOC key codes (C21B000 default key, C21B142 key read/write failure, C21B200 secure PDU authentication fail, C21B300 trip counter). They concern secured control PDUs, not cameras. [PRIMARY] — [dtc.json](https://fisker-ocean-archive.preetham-981.workers.dev/data/dtc.json)
- "Auto park" as a phrase does not appear in the camera entries. The named items are "Full Automatic Park Assist System (APA)", "Personal Park Assist" and "Remote Parking Assist". [PRIMARY] — [dtc.json](https://fisker-ocean-archive.preetham-981.workers.dev/data/dtc.json)
- No ADAS code names its own output link to the ICC; "No output video" appears only as an effect of internal and voltage faults. [PRIOR] — /Users/hiremanirmss/code/projects/fisker/pulse/research_notes/Fisker Ocean camera options and vendors/archive_software_routes.md:82

### Inferences
- **Fastest possible set time is about 0.5 to 0.6 s** (500 ms persistence sampled every 100 ms) for a continuous loss of lock or frozen image. A disturbance shorter than 500 ms should not mature a code by criteria 4 or 5, so "no code" cannot prove that the link was error-free.
- **CRC is the criterion a marginal tap is most likely to hit first**, and it is the one with no documented threshold. It is read from a deserializer register over I2C, so it may be a cumulative counter. Treat any CRC-driven code as possible at any time, including long after the link looked stable.
- **Latching is undocumented.** Three facts point each way: (a) other ADAS codes that do state a reset rule heal when the condition clears and age out after 40 drive cycles; (b) the rear and front entries list "Restart the system" as a test step, which hints that recovery may need a power cycle; (c) the front-camera CAN signal has separate "Reversible_Failure" and "Irreversible_Failure" values (section 2), showing the controller has both kinds. Plan as if a camera code holds its feature loss until at least the next ignition cycle, and as if the stored (confirmed) code stays until cleared or aged.
- Criteria 7 and 8 mean the same code also reports **calibration** problems. A code that appears after hardware was moved or a camera re-seated may be a calibration fault, not a link fault, and no community tool is known to calibrate the surround cameras (section 3).
- C21A531 and C226D00 belong to the front main camera (FCM) Ethernet path, which a coax tap on a surround link does not touch. They matter only as "did anything else change" checks.

### Gaps
- Reset (healing) and aging conditions for C21A109 to C21A409, C21A609 and P1A0502: blank in dtc.json and absent from the PDF sheets. The original "list" source behind dtc.json was not found.
- The CRC error threshold and whether the counter is per ignition cycle.
- Whether a camera code raises the driver-display notification immediately, and its text. The owner manual says only "In the event of a camera failure, the Driver Display displays a notification." [PAGE] — [Camera System Failure](https://fisker-ocean-manual.pages.dev/manual/driving-and-operating/cameras/camera-system-failure/)
- DTC entries describe design intent from a 2023-era document. I found no report of these codes being observed with a tap fitted, and software 2.x behaviour may differ.

---

## 2. CAN signals: availability, current view, faults, RAEB, calibration, and who can see them

### Takeaway
The ADAS controller publishes surround-view availability and the current view (0x316), a system fault type that includes "Surround_view_system_fault" (0x317), degradation pop-up reasons including rear AEB and park assist (0x311), and RAEB, park-assist and remote-park status (0x313, 0x31A, 0x314, 0x342). All are ADAS-authored frames that the comma sees on bus 2 and the Pulse head unit sees on IBUS2. **There is no per-camera status signal on CAN**; the closest is a set of per-view button states in 0x316 with a "fault, not press-able" value.

### Cited Findings

**Signal table.** Bit positions are DBC `start|length` (big-endian, as written in the file). Line numbers are in world.dbc unless noted. Cycle times are from the DBC `GenMsgCycleTime` attributes (world.dbc:2987-3045) and the IBUS comments.

| ID (dec) | Message | Signal | Bits | Values | Cycle | world.dbc lines |
|---|---|---|---|---|---|---|
| 0x316 (790) | ADAS_0x316 | `ADASDC_SVS_AvailableSts` "Reasons for SVS not available" | 7\|4 | 0 SVS_available, 1 system_fault, 2 Too_high_speed, 3 to 13 "message3" to "message13" (undocumented) | 50 ms | 658, 2232, 3372 |
| 0x316 | | `ADASDC_SVS_ViewSts` current view | 3\|4 | 0 None, 1 Front_Normal, 2 Left_Flank_3D, 3 Right_Flank_3D, 4 Rear_Normal, 5 Curb, 6 Front_Cross_Path, 7 Rear_Cross_Path, 8 360_Degree_Free_Floating | 50 ms | 657, 2231, 3371 |
| 0x316 | | `ADASDC_SVS_ShowReq` ADAS asks the ICC to show SVS | 28\|1 | 0 No_Request, 1 Show_Request | 50 ms | 669, 2243, 3383 |
| 0x316 | | Ten button states: `..._RrCamBttnShw`, `WdAnglBttnShw`, `FrntWdAnglBttnShw`, `3DCamBttnShw`, `AutoVwBttnShw`, `RearMidBttnShw`, `FrntMidBttnShw`, `SplitBttnShw`, `LeftFlnk3DBttnShw`, `RightFlnk3DBttnShw` | 2 bits each, 9 to 27 | 0 do not show, 1 show, **2 show (fault, not Press-able)** | 50 ms | 659-668, 3373-3382 |
| 0x317 (791) | ADAS_0x317 | `ADAS_SysFltTyp` fault type for CID or cluster | 51\|4 | 0 No_fault, 1 Minor, 2 Front_camera_fault, 3 Radar, 4 Ultrasonic_sensor_fault, 5 AEB_fault, 6 Lane keeping, 7 ACC, **8 Surround_view_system_fault**, 9 Driver monitoring, 10 TJA_ICA, 11 Other_critical, 12 Blind spot, 13 Evasive steering | 50 ms | 684, 2256, 3394 |
| 0x317 | | `ADAS_SysFltWarnReq` cluster fault telltale request | 47\|2 | 0 ADAS_Normal, 1 ADAS_Flt, 2 ADAS_FltCrtcl | 50 ms | 683, 2255, 3393 |
| 0x317 | | `ADAS_IDS_WarnTxtReq` warning text | 29\|6 | includes 8 Surround_view_system_fault, 4 Ultrasonic_sensor_fault, 5 AEB_fault, 21 AEB_off_/_degraded | 50 ms | 677, 3387 |
| 0x311 (785) | ADAS_0x311 | `ADAS_HiPrioDegradationSts` high-priority pop-up | 21\|6 | 0 none, 1 All_ADAS_features_offline, 5 All_front_AEB_support_is_disabled, **6 All_rear_AEB_support_is_disabled**, **7 Autopark_operations_being_disabled**, **8 Park_Assist_Support_disabled**, 9 Front_Cross_Path_Braking_disabled, 10 Rear_Cross_Path_Braking_disabled, 11 ESA disabled | 20 ms | 608, 2191, 3341 |
| 0x311 | | `ADAS_LoPrioDegradationSts` low-priority pop-up | 29\|6 | 0 none, 1 Blind spot offline, 4 Front_Cross_path_warning_disabled, 5 Rear_Cross_Path_warning_disabled, **6 Park_Assist_warnings_disabled**, 7 Door opening warning, **8 Wheel_Scratch_Prevention_disabled**, 9 DCAA | 20 ms | 610, 2193, 3343 |
| 0x313 (787) | ADAS_0x313 | `AEB_ReAEB_Sts` "Status of Rear AEB function" | 27\|4 | 0 UaCMX, 1 DsblCMX, 2 StdbyCMX, 3 StdbyDgrdCMX, 4 ActvCMX, 5 ActvDgrdCMX, 6 ErrCMX, 7 ActvDgrd1CMX, 8 ActvDgrd2CMX, 9 DsblDgrdCMX | 20 ms | 621, 2202, 3347 |
| 0x313 | | `ADAS_BACM_Sts`, `ADAS_FACM_Sts` (rear / front collision mitigation, same enum) | 23\|4, 19\|4 | as above | 20 ms | 619-620, 3345-3346 |
| 0x31A (794) | ADAS_0x31A | `ADAS_RAEBAvl` "Rear AEB is ready (available)" | 16\|1 | **0 Available, 1 Not_Available** | 50 ms | 710, 2278, 3410 |
| 0x31A | | `ADAS_AEB_RAEB_WarnReq`, `ADAS_RAEB_ThreatID` | 31\|1, 39\|8 | 0 No_warning, 1 Warning | 50 ms | 717-718, 3417 |
| 0x314 (788) | ADAS_0x314 | `ADAS_APASts` Advanced Park Assist | 38\|4 | 0 Off, 1 Initialize, 2 Standby, 3 Scanning, 4 Slot_selected, 5 Park_In, 6 Park_Out, 7 Aborted, 8 Parking_complete, **9 Fault**, 10 Park_Out_ready | 20 ms | 636, 2214, 3358 |
| 0x314 | | `ADAS_WSPPASts` wheel scratch prevention | 42\|3 | 0 Off, 1 On_standby, 2 On_available, 3 On_active, **4 Off_fault** | 20 ms | 637, 3359 |
| 0x2CD (717) | ADAS_0x2CD | `ADAS_APAAvailable` | 45\|2 | 0 Not_available, 1 Park_in_available, 2 Park_out_available | 100 ms | 510, 2117, 3319 |
| 0x342 (834) | ADAS_0x342 | `ADAS_RAPAvl` remote park available; `ADAS_Sts_RAP`; `ADAS_Sts_TP` trained parking | 28\|1, 34\|3, 37\|3 | RAPAvl 0 not available, 1 available; Sts_RAP 0 off, 1 init, 2 standby, 3 active, **4 fault**; Sts_TP 0 Off to 4 Replay, **5 Fault** | 50 ms | 936, 939-940, 3549-3553 |
| 0x35B (859) | ICC_0x35B (from ICC) | `ICC_ViewReq` requested view; `ICC_ShowSts`; `ICC_SVS_AutoVwBttnReq`; `ICC_SVS_CamVwCnclReq`; `ICC_AutomaticViewReq`; `ICC_360FreeFloatingReq` | 15\|4 etc. | ViewReq 0 None, 1 Front_Normal, 2 Left_Flank_3D, 3 Right_Flank_3D, 4 Rear_Normal, 5 Curb_or_split, 6 360_or_3D_Freefloating, 7 Front_Cross_Path, 8 Rear_Cross_Path | 200 ms, change-triggered | 1116-1121, 2620-2623, 3668-3672 |

**Front main camera and driver camera (for "did anything else change" checks)**
- `ADAS_0x334` (820): `ADAS_FrntCamFlt` 0 No_Failure, 1 Reversible_Failure, 2 Irrversible_Failure; `ADAS_FrntCamBli` 0 No_Blind, 1 Blind. The comment says "Front camera fault (not handled by ICC temporarily)". [LOCAL] — world.dbc:812-814, 2363-2365, 3465-3467
- `ADAS_0x32B` (811): `ADAS_CamSts` "Camera state": 0 On_Passive, 1 On_Active, 2 On_Reduced_Sight, 3 On_Faliure, 4 Off, 14 Initating, 15 Not_Available. It sits beside `ADAS_SelfGlaring`, `ADAS_AmbLi` and `ADAS_RoadIllmn`. [LOCAL] — world.dbc:754-762, 2318, 3434
- `FCM_0x487` (1159): front camera module flags, including `FCM_CamBli`, `FCM_NotCal`, `FCM_IniCalOutOfRng`, `FCM_OnlineCalOutOfRng`, `FCM_IniCalTimeout`, `FCM_ComLostFlt_ADAS`. [LOCAL] — world.dbc:1331-1382, 3851-3899
- `ADAS_0x527` (1319): `ADAS_DMS_Camera_Blockage_Status` and `ADAS_DMS_IR_Blockage_Status`, 0 UNKNOWN, 1 OK, 2 BLOCKED, 3 FAULTY. [LOCAL] — world.dbc:1415-1417, 3922-3924

**Fault-flag frames**
- `ADAS_0x481` (1153) and `ADAS_0x482` (1154) carry one bit per internal fault, each comment naming a DTC-like code: bus hardware errors, per-ECU "signal invalid", per-ECU "communication lost", RAM/NVM/watchdog, `ADAS_EOLCalOutOfRng` "ADAS EOL calibration results overbound", `ADAS_ECUTFlt`, `ADAS_PwrModFlt`. Cycle 100 ms. **None names a surround camera, a deserializer or video.** The codes in these comments (C12Axxx, U32xxxx, U13xxxx) do not match the dtc.json numbering (C21Axxx, U15Bxxx). [LOCAL] — world.dbc:1242-1330, 2727-2811, 3039-3040

**Calibration state**
- No CAN signal reports surround-camera calibration. The only calibration signals found are yaw-rate and steering-angle sensor calibration (`YRS_YawRateCalSts`, `EPS_SteerAgSnsrCalSts`), the FCM calibration flags above, and `ADAS_EOLCalOutOfRng`. [LOCAL] — world.dbc:60, 263, 1752, 1919, 2793

**What the comma can already see**
- The harness puts the panda between the vehicle-side gateway (bus 0) and the OEM ADAS module (bus 2). ADAS-authored frames originate on bus 2 and are forwarded to bus 0; a parser must attach to bus 2. [LOCAL] — port/values.py:48-57; port/carstate.py:481-489, 783-786
- The recorded fingerprints include 0x313, 0x314, 0x315, 0x316, 0x317, 0x31A, 0x32B, 0x334, 0x481 and 0x482 (and 0x487 in the second one), so these frames are present on the car's ADAS bus. I did not check the fingerprint for 0x311, 0x342, 0x2CD or 0x527; 0x311 is covered by the next bullet. [LOCAL] — port/fingerprints.py:32-41, 66-75; port/values.py:52-54 names 0x316, 0x317 and 0x31A among bus 2 addresses
- 0x311 is parsed live at 50 Hz for the speed limit, so it is present. [LOCAL] — port/carstate.py:733, 791; adas.dbc:763
- adas.dbc (the DBC the port loads) defines 0x311, 0x313, 0x314, 0x317, 0x31A and 0x35B, with `AEB_ReAEB_Sts` (adas.dbc:272), `ADAS_APASts` (287), `ADAS_WSPPASts` (288), `ADAS_SysFltWarnReq` (319), `ADAS_SysFltTyp` (320) and `ADAS_RAEBAvl` (346). Its 0x311 is a subset without the two degradation signals (adas.dbc:260-265). It has **no** 0x316, 0x32B, 0x334, 0x342, 0x2CD, 0x481, 0x482 or 0x527. [LOCAL] — adas.dbc
- carstate.py reads from the camera-side parser only `ADAS_0x313` (ACC state), `ADAS_0x314` (`ADAS_BSDSts`), `ADAS_0x315`, `ADAS_0x311` (speed limit), `ADAS_0x31C`, and the lateral and longitudinal counter frames. It does **not** read any surround-view, RAEB, park-assist or system-fault signal. [LOCAL] — port/carstate.py:490-522, 733, 787-799
- The port already rewrites `ICC_0x35B` toward the ADAS controller: while `ICC_0x35B_OVERRIDES` is non-empty, every ICC frame is re-sent on bus 2 with settings replaced (currently lane-change assist, auto high beam, APA setting = Enabled, park direction, park chime, wheel-scratch warning, remote and trained parking enabled); other bits, including `ICC_ViewReq`, pass through. The frame has no checksum or counter. [LOCAL] — port/values.py:190-215; port/carstate.py:717, 772
- "ADAS logs a DTC if its settings frame is disturbed" is recorded from on-vehicle work on 2026-10-19. [LOCAL] — /Users/hiremanirmss/Fisker/sunnypilot/research_notes/Fisker Ocean sunnypilot enhancements/fisker_ocean_specifics.md:239

**What the Pulse head unit can already see**
- ibus.dbc is generated for "FM29 IBUS1 + IBUS2, V390.1 20230120". Every message in the table above is listed on **IBUS2**, forwarded by the gateway, at the same cycle times: 0x311 (ibus.dbc:1921, comment 4691), 0x313 (1932, 4701), 0x314 (1944, 4715), 0x316 (1972, 4740), 0x317 (1987, 4756), 0x31A (2004, 4771), 0x32B (2054, 4809), 0x334 (2111, 4862), 0x342 (2189, 4938), 0x2CD (1810, 4598), 0x527 (2744, 5431). `ICC_0x35B` is sent by the ICC on IBUS2, change-triggered every 200 ms (2357, 5098). [LOCAL] — ibus.dbc
- ibus.dbc has no 0x481 or 0x482. [LOCAL] — ibus.dbc (message search returned none)
- The web HUD already decodes `AEB_ReAEB_Sts`, `ADAS_SysFltTyp`, `ADAS_HiPrioDegradationSts` and `ADAS_LoPrioDegradationSts`. A search of the same file found no use of `ADASDC_SVS_AvailableSts` or `ADAS_RAEBAvl`. [LOCAL] — /Users/hiremanirmss/Fisker/oceanscape/static/js/world/fisker_world.js:437, 566-569
- The DBC comment on 0x313 and 0x314 reads "Messages for ICC (IBUS 2) and Event Data Recorder (Chassis Bus)". [LOCAL] — world.dbc:1732-1733

### Inferences
- **Expected CAN signature of a surround-camera fault** (not documented as a mapping anywhere; built by matching the dtc.json effects to the value tables): `ADAS_SysFltTyp` = 8 and `ADAS_IDS_WarnTxtReq` = 8; `ADASDC_SVS_AvailableSts` = 1; one or more 0x316 button states = 2; `ADAS_HiPrioDegradationSts` = 7 or 8 and `ADAS_LoPrioDegradationSts` = 6 or 8; `ADAS_APASts` = 9; `ADAS_Sts_RAP` = 4; `ADAS_Sts_TP` = 5; `ADAS_RAPAvl` = 0. For the rear camera add `ADAS_HiPrioDegradationSts` = 6, `AEB_ReAEB_Sts` = 6 or a "Dgrd" value, and `ADAS_RAEBAvl` = 1.
- **The per-view button states are the nearest thing to a per-camera indicator.** By view name, `LeftFlnk3DBttnShw` = 2 would point at the left camera, `RightFlnk3DBttnShw` at the right, `RrCamBttnShw` / `RearMidBttnShw` at the rear, and `FrntWdAnglBttnShw` / `FrntMidBttnShw` at the front. This mapping is a guess from names. The only documented per-camera identity is the DTC itself.
- `ADASDC_SVS_AvailableSts` = 2 (Too_high_speed) is normal above the view speed limits and must not be read as a fault. Values 3 to 13 are undocumented and could include door-open or mirror-folded reasons (the owner manual lists both as limitations, section 4).
- `ADAS_RAEBAvl` and `AEB_ReAEB_Sts` probably vary with gear and speed in normal use (RAEB works at 0 to 10 mph in reverse, section 4). Their fault meaning is only usable against a baseline recorded in the same gear and speed state.
- `ADAS_CamSts` in 0x32B is probably the front main camera's state for the high-beam function, given its neighbours, and not a surround-camera state.
- Openpilot records raw CAN from all buses, so 0x316, 0x32B, 0x334, 0x342, 0x481 and 0x482 should be in existing route logs and decodable offline with world.dbc even though adas.dbc lacks them. I did not open any log to confirm.
- Detection latency on CAN is one frame period (20 to 50 ms) after the controller changes state, so a live watcher would show a camera fault well within one second of it maturing.
- Because the comma already alters `ICC_0x35B` and has caused ADAS codes before, the baseline code list may not be empty. Tap results are only readable if the comma configuration is identical before and after, or the comma is unplugged for the tap sessions.

### Gaps
- No capture exists of these signals during a real camera fault, so every "expected signature" value above is unverified.
- Meaning of `ADASDC_SVS_AvailableSts` values 3 to 13.
- Whether the running software still matches the January 2023 CAN matrix for these messages.
- No per-camera status signal and no surround-camera calibration signal was found in any of the three DBC files.

---

## 3. Diagnostics access: reading and clearing DTCs

### Takeaway
Reading and clearing codes over the OBD port with plain UDS is established community practice and needs no security access in the open-source tool. Fiskeretta (free, MIT) and Freesker (paid) both use a Vgate vLinker FD+ adapter, and Fiskeretta lists the ADAS controller at request 0x7A6 / response 0x7AE and the ICC at 0x781 / 0x789. Mainstream scan tools (Autel, Topdon) are reported not to carry the model. No tool is known to calibrate the surround cameras.

### Cited Findings

**Open-source: Fiskeretta diagnostics** (all [PRIMARY], repository read as text at HEAD; last commit 2026-06-14 per the GitHub API; MIT licence) — [Fiskeretta/fiskeretta-diagnostics](https://github.com/Fiskeretta/fiskeretta-diagnostics)
- "A free, open diagnostics tool for the Fisker Ocean"; it "talks directly to a Vgate vLinker FD+ (BLE / ELM327 dongle) plugged into the OBD2 port". README.
- Transport: "UDS (ISO 14229) over ISO 15765-4 CAN, via ELM327 AT-command framing"; adapter protocol 6 = "ISO 15765-4 CAN 11-bit @ 500 kbaud". `fiskeretta/uds.py` lines 1-11.
- Built-in addresses (request, response): gateway 0x7C2/0x7CA, PKC 0x7A2/0x7AA, **ICC 0x781/0x789**, BCM 0x7C1/0x7C9, ACU 0x7A0/0x7A8, ESP 0x7D0/0x7D8, PIU 0x791/0x799, VSP 0x7C6/0x7CE, MCU_F 0x786/0x78E. `fiskeretta/uds.py` lines 25-34.
- Discovered addresses "catalogued from real cars": **ADAS 0x7A6/0x7AE, part FM2980140120H**; FCM 0x780/0x788; a second ICC entry 0x7B1/0x7B9; GW 0x7B6/0x7BE; four MRR entries; EPS, ESP, BMS, TBOX and others. `fiskeretta/knowns.py`.
- **Conflict inside the repository:** `fiskeretta/modules.py` still lists `adas` and `fcm` under `NOT_YET_REACHED_KEYS`, while `knowns.py` gives both an address. I read `knowns.py` as the newer data.
- Services used: 0x22 (VIN, DID 0xF190, to the gateway); 0x19 sub-function 0x02 (read DTCs by status mask), 0x04 (snapshot / freeze frame), 0x06 (extended data); 0x14 `FFFFFF` (clear all groups); 0x3E 00 tester present as the discovery probe across 0x700 to 0x7FF; 0x10 03 extended session, used once only as a fallback if a clear is rejected. `fiskeretta/uds.py` lines 155-397, 575-637.
- Clearing: the code handles NRC 0x78 (response pending) because "radars/cameras can take seconds to wipe their fault memory and reply 0x78 first". `fiskeretta/uds.py` lines 349-397.
- Status bits: it separates `testFailed` (0x01, "actively failing as of the most recent test") from `confirmedDTC` (0x08, "Stays set until the DTC memory is cleared"), and notes that most table entries show only "not completed / not failed since last clear" bits (0x10/0x40/0x50). `fiskeretta/uds.py` lines 36-50, 176-190.
- Conditions: "Switch the car to ON (Ready). The car must be awake"; "only use it while the car is safely parked — never while driving. Clearing codes permanently erases the car's stored diagnostic history." README lines 33, 68-69.
- No 0x27 (security access), 0x2E or 0x31 use was found in the repository. [PRIOR] — /Users/hiremanirmss/code/projects/fisker/pulse/research_notes/Fisker Ocean camera options and vendors/archive_software_routes.md:119

**Paid: Freesker** [PAGE] — [freesker.com](https://freesker.com/)
- "Read and clear Diagnostic Trouble Codes (DTCs) from ECUs or the full network"; PDF export of detected faults.
- Lite EUR 99, Pro EUR 478, one-time, licence tied to a single VIN. Android, Mac, Windows through Chrome or Edge.
- Vgate vLinker FD+ is the only adapter it calls fully tested; "other dongles are not supported officially".
- Front camera: reinitialise or overwrite calibration data for "Calibration Out of Range". Radar: role assignment. **Surround view / 360 camera calibration is not mentioned.** The page does not publish a module list and does not name Hydra or FCM.

**FAST, OV Loop, FOA, Ocean Repair Services**
- The owner manual says the diagnostic connector is under the dashboard left of the steering wheel and is used with a Fisker service diagnostic tool to read and clear fault codes. [SNIPPET] — [Vehicle Diagnostic Connector](https://fisker-ocean-manual.pages.dev/manual/maintenance/electrical/vehicle-diagnostic-connector/)
- In 2024, owners reported that the only way to check and remove error codes was the proprietary FAST (Fisker Aftersales Service Tool). [SNIPPET] — [InsideEVs](https://insideevs.com/news/725366/fast-fisker-diagnotstic-tool-release/). Later, new service locations were equipped with it. [SNIPPET] — [InsideEVs](https://insideevs.com/news/735084/fisker-ocean-service-centers-fast/)
- FAST was shut off in summer 2024 and later reopened to service providers; OV Loop sells FAST as a remote service at $150 per hour for subscribers and $250 otherwise, two-hour minimum. [PRIOR] — /Users/hiremanirmss/code/projects/fisker/pulse/research_notes/Fisker Ocean ICC camera streams/community_service_evidence.md:81; /Users/hiremanirmss/code/projects/fisker/pulse/research_notes/Fisker Ocean camera options and vendors/oem_vendors.md:157
- FAST is described as the tool for ADAS camera and radar calibration. [PRIOR] — oem_vendors.md:175 (source: [Autobody News](https://www.autobodynews.com/news/as-fisker-bankruptcy-drives-up-repair-costs-owners-spark-parts-revolution))
- Ocean Repair Services lists an "ADAS 3.1.1 Hydra and FCM" package and says SeaScan or Freesker can do secondary FCM calibration. [PRIOR] — oem_vendors.md:167 (source: [ORS shop](https://shop.oceanrepairservices.com/product/fisker-ocean-adas-311-hydra-and-fcm-configured-to-vehicle))
- OceanLink Pro reports the software version of each control unit, including "HYDRA - Advanced Driver Assist System". [PRIOR] — archive_software_routes.md:124, 135
- FOA's "SURF" software is mentioned by one technician as open for public alpha testing. [PAGE, single] — [diag.net thread](https://diag.net/discussion/szbjhkrp1cwqukksdzwiqipeoq)

**Generic and mainstream scan tools** [PAGE, single thread, undated] — [diag.net thread](https://diag.net/discussion/szbjhkrp1cwqukksdzwiqipeoq)
- "my autel and topdon both list BAIC but don't have the correct model."
- A ThinkScan 689 report "didn't properly pull DTCs from all modules".
- Launch: nothing found in this session's searches.

**Gateway and SecOC limits**
- SecOC protects control frames (steering 0x1D0, acceleration 0x121, and others), with freshness from `GW_Syn_All`. [LOCAL] — /Users/hiremanirmss/Fisker/sunnypilot/research_notes/Fisker Ocean sunnypilot enhancements/fisker_ocean_specifics.md:44-45
- I found no source saying diagnostic reads or clears need SecOC, a seed-key unlock or an authenticated gateway session. Fiskeretta performs both with plain UDS. [PRIMARY] — [Fiskeretta/fiskeretta-diagnostics](https://github.com/Fiskeretta/fiskeretta-diagnostics)

### Inferences
- The ADAS address 0x7A6 was found by an OBD-port discovery scan, so the gateway appears to route diagnostics to the ADAS controller without a secured-gateway unlock. I did not find a first-hand report of a camera DTC being read or cleared from 0x7A6 specifically.
- For a tap trial the useful read is 0x19 02 with the status byte kept: bit 0x01 (failing now) tells whether the condition has healed, bit 0x08 (confirmed) tells whether a code was stored, and bit 0x04 (pending, per ISO 14229) can show a fault that tripped this cycle without confirming. A tool that shows only "active codes" hides this.
- Read both ADAS (C21A1xx to C21A4xx) and ICC (P1A0502, U1A3D87, U315187) each time. The ICC has two candidate addresses in the Fiskeretta data (0x781 and 0x7B1); which one holds P1A0502 is not established.
- A code set by calibration criteria 7 or 8 may not be fixable with community tools, since none advertises surround-camera calibration. This is the main reason not to unbolt, re-aim or swap any camera during the trial.
- The comma could in principle send the same UDS reads on its own buses, but that means new transmit traffic through panda safety and is outside what the port does today; an OBD-port tool is the simpler and already-proven route.

### Gaps
- No confirmation that Fiskeretta or Freesker returns surround-camera codes from the ADAS controller on a car running current software.
- Whether clearing an ADAS camera code needs an extended session, and how long the controller takes.
- Whether any community tool can run surround-view calibration. None found.
- Autel and Launch coverage: one technician's remark for Autel and Topdon, nothing for Launch.
- oceanforums.com, fiskerbuzz.com, the FOA forum and Reddit were unreachable in the earlier research and were not retried; they are the likeliest places for first-hand DTC readouts.

---

## 4. Channel-to-camera map, the least consequential link, and what the views are for

### Takeaway
The DTC text fixes the channel map: channel 1 right, 2 left, 3 rear, 4 front, with the driver camera on channel 5 of a second bank. The rear link is the most consequential (RAEB and the rear view). By the documented effects the three others cost the same park features, and a side camera is the best first candidate. A practical catch: all four links meet at one quad connector on the controller, so a coupler there touches the rear link too.

### Cited Findings
- Channel map from the failure-event text: right side `x = 1`, left side `x = 2`, rear `x = 3`, front `x = 4`, all "from CAM_CS_A"; driver camera "Channel 5 from CAM_CS_B". [PRIMARY] — [dtc.json](https://fisker-ocean-archive.preetham-981.workers.dev/data/dtc.json); [DTC manual sheets 305-309](https://drive.google.com/file/d/17oA-uFKGrGGDLmGYosG-R3d4jGJz1vW_/view)
- RHD cars: the wiring sheet shows separate RHD wires crossing left and right at `D*01` and `D*02`, "so do not label channels by position alone". [PRIOR] — /Users/hiremanirmss/code/projects/fisker/pulse/research_notes/Fisker Ocean camera options and vendors/connectors_and_capture.md:87
- Connectors: surround links arrive at the controller on a quad HFM (`C003B*4F`); front and rear camera ends are sealed FAKRA rated for at most 5 mating cycles; side-camera ends appear to be plugs; the quad HFM is rated for at least 25 cycles. A quad coupler "reaches all four links, and one fault can hit all four". [PRIOR] — connectors_and_capture.md:78-88; /Users/hiremanirmss/code/projects/fisker/pulse/reports/Fisker Ocean camera options and vendors.md:55-63
- The mirror camera has its own coax to the digital mirror and is not on the ADAS channel list; no DTC names it. [PRIOR] — reports/Fisker Ocean camera options and vendors.md:30, 65
- View availability by speed. [PAGE] — [Exterior Cameras](https://fisker-ocean-manual.pages.dev/manual/driving-and-operating/cameras/exterior-cameras/)
  - Automatic: "Available in parking scenarios only. Uses obstacle detection to toggle between camera views".
  - Front Wide View: only in D below 10 mph (16 km/h).
  - Cross Traffic View: in R; also in D until speed exceeds 10 mph.
  - 3D Surround View (if equipped): in D and R below 10 mph.
  - **Rear View: in D and R between 0 and 130 mph (0 and 210 km/h).**
- Surround view limitations: "The liftgate, the hood, or a door is not closed"; "The side mirrors are folded". Views listed: 2D Front, Front Wide, Split, Rear, Rear Wide; 3D Left Flank, Right Flank, 3D Surround. [PAGE] — [2D/3D Surround View System](https://fisker-ocean-manual.pages.dev/manual/advanced-driver-assistance-systems-adas/parking-features/2d-3d-surround-view-system/)
- "Reverse Automatic Emergency Braking is available between speeds of 0-10 mph (0-16 km/h)." AEB "uses the exterior cameras and radar sensors". Rear braking is set under Driving Assistance > Rear Collision Mitigation > Brake Intervention, in Reverse. If AEB is unavailable due to a fault, a telltale lights on the Driver Display. [PAGE] — [Automatic Emergency Braking](https://fisker-ocean-manual.pages.dev/manual/advanced-driver-assistance-systems-adas/safety-features/automatic-emergency-braking/)
- "Parking Assist is only available at vehicle speeds below 12 mph (20 km/h)." [PAGE] — [Parking Assist](https://fisker-ocean-manual.pages.dev/manual/advanced-driver-assistance-systems-adas/parking-features/parking-assist/)
- ADAS fault indication: "an indicator (telltale) illuminates on the Driver Display along with an audible alert." [PAGE] — [ADAS Fault](https://fisker-ocean-manual.pages.dev/manual/advanced-driver-assistance-systems-adas/about-adas/adas-fault/)
- Owner complaints link camera failures with cold weather (below about 50 F and below about 25 F, two reports) and with partial software updates. [PRIOR] — /Users/hiremanirmss/code/projects/fisker/pulse/research_notes/Fisker Ocean ICC camera streams/community_service_evidence.md (NHTSA complaint list)

### Inferences
- **Ranking by consequence, worst first:** rear (RAEB, the rear view used up to 130 mph, rear cross-path view, and the centre-screen reversing image that FMVSS 111 concerns); then front (front wide and cross-traffic views; possibly an input to front cross-path functions, which the degradation table lists but no source ties to a camera); then left and right, which the data ties only to curb, flank and 3D views and the three park features.
- **First link to test: a side camera, left or right.** The data gives no reason to prefer one side. Nothing in the sources shows a side surround camera feeding a driving-speed function: blind-spot and lane-change functions have corner-radar codes (C21AA09 to C21AD09), and the side-camera DTC text lists only park features. That is absence of evidence, not proof.
- **At speed versus parking:** by the manual, only Rear View is available above 10 to 12 mph; everything else the surround cameras do is a parking-speed function. So a side or front link fault should not change driving-speed behaviour, apart from the fault telltale, chime and pop-up themselves, which are a distraction risk while driving.
- **"One link" is hard to achieve at the controller end.** A quad HFM coupler passes all four links, including the rear one, through the added device. A single-link tap needs a single-channel coupler at a side-camera end, or a quad breakout that leaves three channels as straight pass-through. This choice changes the risk more than which side is chosen, and it should be settled on the bench.
- For the whole trial, keep doors, hood and liftgate closed and mirrors unfolded when reading surround-view availability, or the car's own limitations will look like a fault.

### Gaps
- Which physical camera sits on which channel on this car has not been checked on the vehicle; the map is from design documents.
- Whether any driving-speed function consumes surround-camera images on current software.
- Where a single-link tap can physically go without a sealed exterior joint. Not established for the side cameras.
- Whether the car is equipped with 3D Surround View, the digital mirror and remote parking (trim dependent).

---

## 5. A staged test procedure with stop conditions and rollback

### Takeaway
No source describes a tested tap on this car, so the procedure below is my proposal built from the documented timings: prove the coupler on salvage parts first, then one side link only, stationary, with code reads and CAN logging before, during and after, hold times set by the 500 ms and 120 s + 20 s monitors, and removal on the first new code or status change. A clean result shows only that nothing matured during the test.

### Cited Findings (the facts the procedure rests on)
- Camera-link codes need 500 ms of lost lock, frozen image or power fault, on a 100 ms monitor; CRC has no stated threshold. [PRIMARY] — [dtc.json](https://fisker-ocean-archive.preetham-981.workers.dev/data/dtc.json)
- P1A0502 on the ICC starts 120 s after IGN ON and checks every 20 s. [PRIMARY] — [dtc.json](https://fisker-ocean-archive.preetham-981.workers.dev/data/dtc.json)
- Battery below 8.5 V for 10 s sets U15B616 and removes all ADAS features and video; several monitors require 9 to 16 V. [PRIMARY] — [dtc.json](https://fisker-ocean-archive.preetham-981.workers.dev/data/dtc.json)
- Where reset rules are stated for ADAS codes, stored codes age out after 40 drive cycles. [PRIMARY] — [dtc.json](https://fisker-ocean-archive.preetham-981.workers.dev/data/dtc.json)
- Codes can be read (0x19) and cleared (0x14) over the OBD port with the car ON and parked. [PRIMARY] — [Fiskeretta/fiskeretta-diagnostics](https://github.com/Fiskeretta/fiskeretta-diagnostics)
- Coupler facts: TZ couplers are passive with a receive-only forward output; a non-PoC variant on a powered line is "not suitable"; no insertion-loss figure is published; a stock camera-kit deserializer "must never touch a live coax" because it may drive the back channel and power. [PRIOR] — /Users/hiremanirmss/code/projects/fisker/pulse/reports/Fisker Ocean camera options and vendors.md:55-71; /Users/hiremanirmss/code/projects/fisker/pulse/research_notes/Fisker Ocean ICC camera streams/tapping_options.md:43
- Earlier safeguards: bench test first with insertion and return loss measured; "Remove the tap immediately if any new DTC, ADAS warning, frame drop or image artifact appears"; "the absence of a fault code does not prove margin". [PRIOR] — tapping_options.md:205-208
- Salvage prices seen earlier: controller about $254 to $350, digital mirror about $100 to $370 (undated listings); a used controller may give no output without vehicle partners. [PRIOR] — reports/Fisker Ocean camera options and vendors.md:71
- The comma rewrites `ICC_0x35B` toward the ADAS controller and has caused ADAS codes when settings frames were disturbed. [LOCAL] — port/values.py:190-215; /Users/hiremanirmss/Fisker/sunnypilot/research_notes/Fisker Ocean sunnypilot enhancements/fisker_ocean_specifics.md:239
- Regulatory context already noted: FMVSS 111 requires the rear image within 2.0 seconds of a backing event, a default view each event, and no display after backing ends ([49 CFR 571.111](https://www.law.cornell.edu/cfr/text/49/571.111)); 49 U.S.C. 30122 bars manufacturers, distributors, dealers and repair businesses from making required safety equipment inoperative and does not name owners ([statute](https://openjurist.org/49/usc/30122)), with no NHTSA letter found on owners; California permits displays that supplement the driver's view for manoeuvring ([Cal. Veh. Code 27602](https://california.public.law/codes/vehicle_code_section_27602)). [PRIOR] — reports/Fisker Ocean camera options and vendors.md:73; tapping_options.md:164-192. This is context, not legal advice.

### Inferences (the proposed procedure; every step is my proposal)

**Stage 0. Decide the scope before buying anything**
1. Target one side-camera link only. Exclude the rear link from every in-car stage until a side link has passed all stages.
2. Settle how a single link will be tapped (section 4). If the only practical device is a quad coupler at the controller, treat the trial as a four-link trial including rear, with RAEB at stake, and raise the bench bar accordingly.
3. Fix the comma state for the whole trial: either unplugged with the factory connection restored, or connected with one unchanged software build. Do not change it between baseline and test.

**Stage 1. Bench, salvage parts only**
1. Read the serializer and deserializer markings on a salvage camera and controller to settle the protocol (FPD-Link III is the best guess, unconfirmed).
2. Measure DC on the salvage coax to confirm power-over-coax and pick the PoC coupler variant.
3. Measure the coupler's main-path insertion and return loss (VNA or vendor S-parameters), with the tap output both terminated and open, and with the tap's downstream device powered and unpowered.
4. Confirm that the main path is unchanged when the downstream device loses power, is unplugged or is shorted. The trial must not go to the car if any of these disturbs the main path.
5. If a camera and controller can be made to link on the bench (not assured), run the coupler for hours across temperature, since owner reports tie camera faults to cold.
6. Never attach a normal deserializer kit directly to a live coax.

**Stage 2. In-car baseline, no hardware added**
1. Car parked, level, doors, hood and liftgate closed, mirrors unfolded, 12 V supported so it stays above 9 V.
2. Read all modules with the status bytes kept; save the full ADAS and ICC lists (both ICC addresses). Do not clear anything yet. Note any existing C21A1xx to C21A4xx, C21A6xx, P1A0502, U15Bxxx.
3. Record the CAN baseline for at least 5 minutes after ignition on: 0x316, 0x317, 0x311, 0x313, 0x31A, 0x314, 0x342, 0x2CD, 0x334, 0x32B, 0x481, 0x482. Do it in P, then R with the brake held, then D with the brake held, and cycle each camera view once.
4. Photograph each camera view on the centre screen as an image-quality reference.
5. Repeat on a second day, ideally colder, to learn normal variation.
6. Decode logs on the Mac, not on the comma.

**Stage 3. Connector handling check**
1. Ignition off and car asleep. Disconnect and reconnect the chosen factory connector once, with no coupler.
2. Repeat the Stage 2 reads and recording. This separates "handling the connector causes a code" (including calibration criteria or a not-fully-seated plug) from "the coupler causes a code".

**Stage 4. Coupler in line, tap output terminated, nothing downstream**
1. Fit with ignition off and the car asleep. Seat fully; count the mating cycle.
2. Ignition on. Watch CAN live for the first 10 seconds, since a hard link failure should mature in about 0.5 s.
3. Hold at least 10 minutes: this covers the 120 s start and many 20 s cycles of P1A0502, and gives the undocumented CRC criterion time. Then exercise every view, P, R and D with the brake held.
4. Read codes with status bytes. Compare with baseline.
5. Do at least 5 ignition cycles with a full sleep between some of them (cold starts catch lock-acquisition problems), reading codes each time.

**Stage 5. Downstream receiver connected (receive-only)**
1. Repeat Stage 4 in full with the receiver powered.
2. Then power the receiver off, and then unplug it, with the car on, watching CAN. The main path must not react.

**Stage 6. Low-speed private-ground test**
1. Only after Stages 4 and 5 are clean on several days. Private ground, below 10 mph, a second person watching the live indicators.
2. Use the park features and views normally; include bumps and full steering lock for vibration and cable movement.
3. Read codes afterwards.

**Stage 7. Road use**
1. Only for a side or front link, only after Stage 6 is clean, with a live indicator that shows `ADAS_SysFltTyp`, `ADASDC_SVS_AvailableSts` and the degradation signals.
2. Read codes after each of the first several drives, then weekly. The 40-drive-cycle aging seen on other ADAS codes means a stored code may persist for weeks, which helps here: a later read still shows an event that happened once.
3. Repeat the full check at the first cold spell and the first hot spell.

**Stop conditions (remove the tap and restore the factory connection)**
- Any new stored, pending or currently-failing code in ADAS or ICC compared with baseline; above all C21A109 to C21A409, P1A0502, U15B616, U15B896.
- `ADAS_SysFltTyp` not 0 or `ADAS_SysFltWarnReq` not 0 when the baseline was 0; `ADAS_IDS_WarnTxtReq` = 8.
- `ADASDC_SVS_AvailableSts` = 1, or any value never seen in baseline for the same gear and speed.
- Any 0x316 button state = 2 that was not 2 in baseline.
- `ADAS_HiPrioDegradationSts` or `ADAS_LoPrioDegradationSts` not 0 when baseline was 0.
- `ADAS_APASts` = 9, `ADAS_Sts_RAP` = 4, `ADAS_Sts_TP` = 5, `ADAS_WSPPASts` = 4, or `AEB_ReAEB_Sts` / `ADAS_RAEBAvl` differing from baseline in the same state.
- Any new bit in 0x481 or 0x482.
- A grayed, frozen, flickering, tearing or noisy feed in any view, or a view that opens later than in baseline, even with no code.
- Any driver-display notification, telltale or chime not present in baseline.
- The reversing image appearing late or not at all. Treat this as an immediate stop regardless of codes.
- 12 V below 9 V during a test: stop and repeat, since the result is not interpretable.
- A stop condition met twice on separate attempts ends the trial for that coupler; do not keep retrying on the car.

**Rollback**
1. Ignition off, let the car sleep, remove the coupler, reconnect the factory connector, confirm it is latched.
2. Ignition on; wait at least 3 minutes; check every view and the reversing image.
3. Read codes. Look at bit 0x01 first: if the code is no longer "failing now", the condition has healed and only the stored record remains.
4. Save the full report (including freeze-frame and extended data) before clearing; then clear ADAS and ICC with 0x14 and allow for a slow reply.
5. Run two more ignition cycles and a short private-ground check; read again and confirm the lists match the Stage 2 baseline and the CAN indicators match too.
6. If a camera code stays "failing now" with the factory connection restored, stop. The remaining causes are a damaged or unseated connector, a damaged cable or the calibration criteria, and the last may need FAST-level service.

### Gaps
- No source reports any tap trial on a Fisker Ocean; none of these steps has been tried.
- Whether a camera code and its feature loss clear on their own, on the next ignition cycle, or only after a tool clear.
- The CRC threshold, so the hold times above are a judgement and not a guarantee.
- Whether the salvage controller will run cameras on a bench without the rest of the vehicle network.
- Coupler insertion loss and the link's margin: neither is published.
- Whether the reversing image on the centre screen is the certified FMVSS 111 rear view on this car (the digital mirror is a separate system), and state rules outside California.
