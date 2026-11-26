# Fisker Ocean ADAS controller and surround camera hardware (for a passive tap at the controller connector)

Evidence labels used below:
- **[doc]** I downloaded the community-archive file and read its text myself (service manual, parts book, wiring set, fault-code data). These are community copies of Fisker documents, not checked on a car.
- **[datasheet]** I read the maker's PDF text.
- **[page]** I read the web page. **[snippet]** I saw only a search summary.
- Anything under "Inferences" is my reasoning, not a source statement.

Earlier notes already cover the TechInsights public summary, the Rosenberger code decoding and the TZ coupler catalogue. This file extends them. The main new facts are the controller's location and removal steps, its part number, the per-channel camera map, the camera power-switch behaviour, and the exact text that names the AR0233AT.

Community archive sources (all linked from the [archive index](https://fisker-ocean-archive.preetham-981.workers.dev/)):
- Service manual: [Fisker Ocean - Service Manual - EN - v1.0.pdf](https://drive.google.com/file/d/1BwRtG41yIPEU5H6KvpWaj1X8OWSvLYuf/view)
- Parts book: [Ocean Parts Book.pdf](https://drive.google.com/file/d/1j-PSGpu0bNz-jYVWydXP8Ga0_e9ujeEU/view)
- Wiring set: [Wiring Diagrams FM29 MY2023](https://drive.google.com/file/d/1SY0NpmfMqZCx2eLu4gMEKs2z0kN_mZQT/view)
- Fault-code data: [dtc.json](https://fisker-ocean-archive.preetham-981.workers.dev/data/dtc.json), built from the service DTC manual
- Fuse data: [fuses.json](https://fisker-ocean-archive.preetham-981.workers.dev/data/fuses.json)

## What do teardowns and other sources say about the controller's deserializer hub, serializer, SoC and board connectors?

### Takeaway
No public source gives a chip part number or the SoC for the controller. The only chip-level facts are the TechInsights summary's device descriptions (TI FPD-Link III deserializer hub, TI FPD-Link III serializer with CSI-2 interface). Fisker's own fault-code text adds useful structure: the controller is called "Hydra3", it has a deserializer with a lock status per camera, it reads camera registers over I2C, and it switches camera power per channel in two banks.

### Cited Findings

**TechInsights public summary (re-read on 2026-11-23, unchanged from earlier notes)**
- The summary lists NXP CAN transceivers; Texas Instruments power management, "FPD-Link III Deserializer Hub", "FPD-Link III Serializer with CSI-2 Interface" and a Gigabit Ethernet transceiver; and one Elmos DSI3 transceiver. It names no SoC, no memory, no connector and no part number. — [TechInsights](https://www.techinsights.com/blog/deep-dive-teardown-magna-electronics-adas-gp009379212-automotive) [page]
- "GP009379212" appears only in the report title. The page does not say what the number is. — same page [page]
- Two searches for a second public write-up of the controller (Electronics360 or similar) found only the front-camera article. — [Electronics360 camera teardown](https://electronics360.globalspec.com/article/22672/techinsights-teardown-fisker-ocean-camera) [snippet]

**Fisker part numbers for the unit**
- The parts book lists `FM2980140120B  ADAS_CONTROLLER` (qty 1) and `FM2980140040A  BRACKET_ADAS_CONTROLLER`, with the comment "Also supports the lift ga..." (truncated in the PDF text). — [Parts book, section 80 ELECTRICAL / 12 ADAS](https://drive.google.com/file/d/1j-PSGpu0bNz-jYVWydXP8Ga0_e9ujeEU/view) [doc]

**Internal name and architecture from the fault-code text**
- Every ADAS-module fault criterion is written as "The Hydra3_component shall set the DTC if ...". The controller is also called "ADAS DC" and "Domain Controller". — [dtc.json](https://fisker-ocean-archive.preetham-981.workers.dev/data/dtc.json), codes C21A109 to C21B200, U15B896 [doc]
- The front camera module (FCM) sends object data to the controller over Ethernet, not video. C21A531 sets if any of these PDUs "which FCM sends to Hydra3" is missing for 500 ms: `Light_Scene_Boundle_PDU`, `Objects_Boundle_PDU`, `Road_Boundle_PDU`, `TFL_Boundle_PDU`, `TSR_Boundle_PDU`, `Core_Objects_protocol_1024`, `Core_Objects_protocol_1025`. — [dtc.json](https://fisker-ocean-archive.preetham-981.workers.dev/data/dtc.json) [doc]
- FCM faults name "EyeQ" and "aurix" ("Time synchronisation not present in aurix and EyeQ", "Loss_of_SPI_Communication_from_EyeQ"). One FCM calibration fault says "when EyeQ3 reports the SPC results". — [dtc.json](https://fisker-ocean-archive.preetham-981.workers.dev/data/dtc.json), codes C224000, C220800, C223400, C21F900 [doc]
- The controller has an "SoC" with a temperature limit: U15B896 sets if "SOC temperature is not within range (-40 to 125 deg C) for 5 sec". The limp action is "No output video". — [dtc.json](https://fisker-ocean-archive.preetham-981.workers.dev/data/dtc.json) [doc]
- Supply limits: below 8.5 V for 10 s or above 16.5 V for 1 s gives "No output video/signal". — [dtc.json](https://fisker-ocean-archive.preetham-981.workers.dev/data/dtc.json), U15B616 and U15B617 [doc]

**Camera link monitoring, from the rear camera fault C21A109 (note field)**
- The text reads: "The Register Address of the temperature sensor is R0x3E9A (Refer to developer's guide for AR0233AT Image Sensor: Section - Temperature Sensor) OR 4. Lock failure on deserializer side for 500 ms. OR 5. Not receiving any image, images are frozen, images are not being refreshed for 500ms OR 6. CRC Error (Read thru register over I2C) OR 7. Calibration data is not configured/Alignment is Incorrect or not completed OR 8. Calibration data cannot be read from the flash". — [dtc.json](https://fisker-ocean-archive.preetham-981.workers.dev/data/dtc.json) [doc]
- A thermal criterion also applies: "Thermal shutdown to occur if temperature reading is more than 125 deg C for 500 ms". — same entry [doc]

**Camera power switching, per channel**
- Each camera fault uses the same pattern: "Open circuit/Short to battery failure if INx = L, Fault = L for 500 ms OR Short to ground failure if INx = H, Fault = L for 500 ms". The text says "Refer to Hydra3 schematic diagram and HSIS documenttaion for the meaning of INx and Fault pins" and "SoC is going to determine the state of the Pins using its 'Video - In' schematic block connection". — [dtc.json](https://fisker-ocean-archive.preetham-981.workers.dev/data/dtc.json), C21A109, C21A209, C21A309, C21A409, C21A609 [doc]
- Channel map stated in those entries:

| Channel | Camera | Enable net | Bank | Fault code |
|---|---|---|---|---|
| 1 | Right side | `CAM1_PWR_EN` | `CAM_CS_A` | C21A409 |
| 2 | Left side | `CAM2_PWR_EN` | `CAM_CS_A` | C21A309 (title wrongly says "Front Camera"; body says left side) |
| 3 | Rear | `CAM3_PWR_EN` | `CAM_CS_A` | C21A109 |
| 4 | Front | `CAM4_PWR_EN` | `CAM_CS_A` | C21A209 |
| 5 | Driver monitoring (OV2311) | `CAM5_PWR_EN` | `CAM_CS_B` / `CAM_BANK_B`, IN1 | C21A609 |

- The driver-monitoring entry also names the sensor "DMS camera (OV2311)" and the "DMS IR LED Driver (MAX25614AB)", both reporting status flags including `i2C_communication`. — [dtc.json](https://fisker-ocean-archive.preetham-981.workers.dev/data/dtc.json), C21A609 and C21A709 [doc]

**Harness connectors at the controller (wiring sheet VII.4, PDF p.28; also p.10, p.19, p.20, p.51)**
- The service manual says to "Disconnect harness connectors (x6) from ADAS control unit". The wiring sheet shows six connectors, `C003B*1F` to `C003B*6F`, which agrees. — [Service manual p.80-1, PDF p.237](https://drive.google.com/file/d/1BwRtG41yIPEU5H6KvpWaj1X8OWSvLYuf/view); [Wiring set p.28](https://drive.google.com/file/d/1SY0NpmfMqZCx2eLu4gMEKs2z0kN_mZQT/view) [doc]

| Connector | Part code on sheet | Contents read from the sheet |
|---|---|---|
| `C003B*1F` | `34729-0200` | Supply `Cl.30_MF07_ADAS`, ground `Cl.31_ADAS`, ADAS CAN H/L, IBUS2 CAN H/L, ultrasonic power and DSI signal pairs (pins labelled `A*xxA`) |
| `C003B*2F` | `E6K10A-1CAZ5-C` | `ETH1_P` / `ETH1_N` to the FCM (far end `2301695-1`) |
| `C003B*3F` | `AMK12A-1M4Z5-B` | `DIG_OUT_LVDS / CoAxial` to ICC head unit connector `C028B*9F` (`59Z113-000-D`) |
| `C003B*4F` | `AMK12D-1M4Z5-C` | Four surround coax links, positions `D*01` to `D*04`, each with a drain at `D*0xS` |
| `C003B*5F` | `AMK12D-1M4Z5-D` | `CAM_DMS_LVDS / CoAxial` at position `E*02` |
| `C003B*6F` | `560123-0200` | Private CAN pairs for the five radars, `DMS_LED1_H` / `DMS_LED1_L` (pins labelled `A*xxB`) |

- Position map on the quad connector `C003B*4F` for left-hand-drive cars: `D*01` = `CAM_RIGHT`, `D*02` = `CAM_LEFT`, `D*03` = `CAM_REAR`, `D*04` = `CAM_FRONT`. — [Wiring set p.28 and p.51](https://drive.google.com/file/d/1SY0NpmfMqZCx2eLu4gMEKs2z0kN_mZQT/view) [doc]
- The sheet shows separate RHD wires (W3696 to W3699) that swap `D*01` and `D*02`. — [Wiring set p.28](https://drive.google.com/file/d/1SY0NpmfMqZCx2eLu4gMEKs2z0kN_mZQT/view) [doc]
- Supply fuse: `MF07`, Micro, 15 A, usage "ADAS DC", on the "REAR TRUNK FUSE BOX" sheet, wire W0433 GN/RD 0.75. — [fuses.json](https://fisker-ocean-archive.preetham-981.workers.dev/data/fuses.json); [Wiring set p.19](https://drive.google.com/file/d/1SY0NpmfMqZCx2eLu4gMEKs2z0kN_mZQT/view) [doc]

**TI part families that match the TechInsights wording (general product facts, not Ocean findings)**
- DS90UB960-Q1 title: "Quad 4.16-Gbps FPD-Link III Deserializer Hub With Dual MIPI CSI-2". It has 2 CSI-2 output ports with 1 to 4 lanes each, 400 Mbps to 1.6 Gbps per lane (1664 Mbps maximum), a port replication mode, coax with PoC or STP, line fault detection, and a CMLOUT monitor pin. It pairs with DS90UB953-Q1, DS90UB935-Q1 and DS90UB933-Q1 serializers. — [TI DS90UB960-Q1 datasheet](https://www.ti.com/lit/ds/symlink/ds90ub960-q1.pdf) [datasheet]
- DS90UB953-Q1 title: "FPD-Link III 4.16Gbps Serializer With CSI-2 Interface". — [TI DS90UB953-Q1 datasheet](https://www.ti.com/lit/ds/symlink/ds90ub953-q1.pdf) [snippet]
- On this family the forward channel runs at about 4 Gbps and the back channel at 50 Mbps; the back channel carries clock, I2C and GPIO from the deserializer to the serializer. — [TI design guide TIDA-050035](https://www.ti.com/lit/ug/tiduev5/tiduev5.pdf) [datasheet]
- TPS4H160-Q1 is a quad-channel smart high-side switch with pins IN1 to IN4, FAULT and CS (current sense), "Open-load and short-to-battery detection", "Overcurrent and short-to-ground detection" and thermal shutdown. — [TI TPS4H160-Q1 datasheet](https://www.ti.com/lit/ds/symlink/tps4h160-q1.pdf) [datasheet]

### Inferences
- **The hub is probably a DS90UB960-class part and the output serializer a DS90UB953-class part.** The TechInsights wording matches TI's titles for those two parts almost word for word. That is a match of description, not a read marking. DS90UB954-Q1, DS90UB962-Q1 and others stay possible.
- **If the serializer is a "UB" CSI-2 camera-class part, the link to the ICC has no HDCP.** TI uses "UH" for HDCP parts, and its HDCP parts are display-class (for example DS90UH941AS-Q1), not "Serializer with CSI-2 Interface". This rests on the description match above.
- **There are five coax camera inputs, so one quad hub is not enough.** Four surround links sit on connector `*4F` and the driver camera on `*5F`, and the power switching uses two banks (A with four channels, B with at least one). A second deserializer for bank B is likely. The TechInsights summary names only one hub, but its public list is partial.
- **Power over coax is used and is switched per camera by the controller.** The wiring shows only coax and drain at each surround camera, with no separate supply wire, and the fault text describes per-channel power enables with open-circuit and short detection. The IN/Fault/CS pattern matches a TPS4H160-type quad high-side switch. The actual switch part is not named in any source.
- **The controller, not the camera, reads the image sensor.** The sensor temperature register and a CRC are read "over I2C", which on FPD-Link runs through the back channel. A tap must therefore leave the back channel and the DC path intact.
- **Deserializer hub channel numbers may follow the same 1 to 4 order** (right, left, rear, front), since the connector positions and the power channels agree. The RX port order on the chip is not stated anywhere.
- **"EyeQ3" in one FCM fault line is probably leftover template text.** It conflicts with an 8.3 MP camera. Do not use it to identify the front camera's chip.
- **GP009379212 is likely Magna's own part number for the unit** and `FM2980140120B` is Fisker's. No source links the two numbers.

### Gaps
- Exact deserializer, serializer, SoC, memory and power-switch part numbers: not public. They are in the paid TechInsights report or on a salvage board.
- No FCC filing, board photo or forum teardown of the controller was found. Salvage listing pages were not inspected in this pass (earlier notes record that eBay and PartRequest block fetches).
- Searches for "Hydra" as a Magna product name returned nothing. — [Magna compute units page](https://magna.com/products/electrical-electronics/adas-automated-driving/domain-controllers) [snippet] does not name it.
- The "Hydra3 schematic" and "HSIS" documents named in the fault text are not in the archive.
- Pin numbering inside the quad HFM housing (which physical cavity is `D*01`) is not given on the sheets I read.

## Which sensor and serializer do the surround cameras use, what do they output, and who makes them?

### Takeaway
The sensor is confirmed as onsemi AR0233AT by Fisker's own fault-code text. All four surround cameras share one Fisker part number, FM2980130004A. The serializer, the output format, the frame rate, the power-over-coax voltage and the camera maker are not stated in any source I found.

### Cited Findings
- The surround camera fault criteria refer to the "developer's guide for AR0233AT Image Sensor" and its temperature register R0x3E9A. — [dtc.json](https://fisker-ocean-archive.preetham-981.workers.dev/data/dtc.json), C21A109 note [doc]
- The parts book lists `FM2980130004A  SURROUND_VIEW_CAMERA` twice: qty 2 under exterior mirrors and qty 2 under ADAS. — [Parts book](https://drive.google.com/file/d/1j-PSGpu0bNz-jYVWydXP8Ga0_e9ujeEU/view) [doc]
- Other camera part numbers in the same book: `FM2980140002B  FRONT_CAMERA`, `FM2980160001A  DRIVER_RECOGNITION_CAMERA`, `FM2960920048A  SPOILER_ASSY_WITH_CAMERA`, mirrors `FM2955400101A` (LH) and `FM2955400105A` (RH). — [Parts book](https://drive.google.com/file/d/1j-PSGpu0bNz-jYVWydXP8Ga0_e9ujeEU/view) [doc]
- The front and rear surround cameras sit in the bumpers on a clipped bracket and need the bumper removed. Replacement needs calibration with the Fisker Aftersales Service Tool. — [Service manual p.80-4](https://drive.google.com/file/d/1BwRtG41yIPEU5H6KvpWaj1X8OWSvLYuf/view) [doc]
- Each surround camera has one coax plus drain and no separate power wire on the wiring sheet. Camera-end connector codes are `59Z163-003-B` (front, rear) and `59S14M-102A4-B` (left, right). — [Wiring set p.28, p.51](https://drive.google.com/file/d/1SY0NpmfMqZCx2eLu4gMEKs2z0kN_mZQT/view) [doc]

**AR0233AT general facts (not Ocean-specific)**
- "active array size of 2048 x 1280 with 1080p output at 60 frames per second", more than 140 dB multi-exposure HDR, full-resolution LED flicker mitigation with 120 dB, "4-lane MIPI CSI-2 Interface", ASIL-B design. "The sensor is compatible with image signal processor (ISP) companion chips from onsemi as well as third-party ISPs." — [onsemi AR0233AT product brief, Digi-Key copy](https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/7169/AR0233AT-3688810.pdf) [datasheet]
- onsemi's product page is summarised as listing 2048 x 1280 at 45 fps and 1920 x 1080 at 60 fps. — [onsemi AR0233AT](https://www.onsemi.com/products/sensors/image-sensors/ar0233at) [snippet]; the page text I fetched was truncated before the specifications.
- TI's camera reference design TIDA-050035 pairs "ON Semi AR0233" with the DS90UB953-Q1 and a TPS650330-Q1 PMIC on a single Rosenberger FAKRA coax. It describes the sensor output as "full-HD, AD 13-bit, MIPI 4-lane, RAW24, RAW20, RAW16, RAW14, RAW12". Its power-over-coax input is 4 V to 18.3 V, nominal 12 V ("a pre-regulated, 12-V supply over coax"). It was tested streaming 1080p at 60 fps into a DS90UB960-Q1. — [TI TIDA-050035 design guide](https://www.ti.com/lit/ug/tiduev5/tiduev5.pdf) [datasheet]
- A Leopard Imaging AR0233 module is listed with a GMSL2 serializer and RAW output. — [Leopard Imaging](https://leopardimaging.com/?p=27541) [snippet]

### Inferences
- **The camera output is most likely RAW Bayer (12-bit class), with the ISP in the controller.** Three things point this way: the AR0233AT has no on-chip ISP per onsemi's brief, the controller reads the bare sensor's registers directly, and TI's AR0233 plus DS90UB953 design sends RAW. A camera-side ISP (YUV output) is not excluded, because onsemi sells companion ISPs and no camera teardown exists.
- **The serializer is probably a TI DS90UB953-Q1 or DS90UB935-Q1.** The controller side is TI FPD-Link III per TechInsights, and those are the CSI-2 serializers TI pairs with its hubs. No source names it.
- **Expect 1920 x 1080 or 2048 x 1280 at 30 fps as the working assumption.** The sensor supports up to 60 fps at 1080p, but surround view systems often run 30 fps. The Ocean's actual mode is unknown, so any capture design should not fix the resolution or rate in advance.
- **RAW output raises the work after the tap.** A receive-only capture would get companded HDR Bayer data, which needs sensor-specific decompanding, demosaic, lens-shading and fisheye correction before it looks like the car's display.
- **Power-over-coax voltage is unknown.** The TI reference camera accepts 4 V to 18.3 V. The Ocean could use switched battery voltage or a regulated rail (commonly 8 V to 12 V in such systems, from my general knowledge). Measure it on a salvage unit before choosing a PoC coupler.
- **The maker is probably Magna, but this is unproven.** Magna holds the full ADAS system contract ([Magna release](https://www.magna.com/company/newsroom/releases/release/2021/01/04/news-release---magna-expands-with-fisker-secures-full-adas-system-business)) and makes surround cameras. No label photo, listing or document names the camera maker.

### Gaps
- Serializer part, output format (RAW or YUV), resolution, frame rate, HDR mode, lens field of view, PoC voltage and current: none found for the Ocean.
- Camera maker and its own part number: not found. A label photo of FM2980130004A would settle it.
- The full AR0233AT datasheet is under onsemi NDA; only the brief was read.
- Whether "Calibration data cannot be read from the flash" refers to a memory in the camera or in the controller is not stated.

## Is the surround connector a Rosenberger quad HFM, AMK12D, coding C, and what are the HFM specifications and available adapters?

### Takeaway
Yes. The wiring sheet gives `AMK12D-1M4Z5-C` for the four surround links, and Rosenberger's datasheet for that part confirms a straight jack with housing, coding C blue. The 20 GHz figure is the HFM family limit; this specific part is rated DC to 15 GHz. HFM-to-SMA and HFM-to-FAKRA assemblies exist, but I confirmed only one price and no quad-housing breakout cable.

### Cited Findings

**Part AMK12D-1M4Z5-Y (the harness-side connector)**
- "HFM STRAIGHT JACK w. HOUSING", interface per RN_108-04, assembly instruction MA_AMV003. — [Rosenberger datasheet](https://products.rosenberger.com/_ocassets/db/AMK12D-1M4Z5-Y.pdf) [datasheet]
- Electrical: 50 ohm; DC to 15 GHz; return loss at least 25 dB to 3 GHz, 20 dB to 6 GHz, 15 dB to 12 GHz, 12 dB to 15 GHz; insertion loss up to 0.1 x f(GHz) dB; working voltage up to 60 V DC; power current up to 1 A DC; test voltage 800 V rms. — same datasheet [datasheet]
- Mechanical: mating cycles at least 25; engagement force up to 45 N; retention force of the latch at least 110 N; primary and secondary locks at least 80 N each; coding efficiency at least 150 N. The secondary lock is traffic purple. Weight 3.95 g. — same datasheet [datasheet]
- Environment: -40 C to +105 C. Suitable cables: Dacar 302, Dacar 302-3, G&G 59998. Hand crimp tool AMW001-SET. — same datasheet [datasheet]
- Coding table: A black, B white, C blue, D bordeaux violet, E green, F brown, G grey, Z waterblue. — same datasheet [datasheet]

**HFM family**
- DC to 20 GHz, up to 28 Gbps, 50 ohm, mating cycles at least 25. Return loss adds "at least 10 dB, above 15 GHz to 20 GHz". Housings are single, double and quad; cable and PCB connectors, straight and right angle; waterproof and CPA versions. Hand crimp kits: 365609 (AMW001-SET, cable group M4), 428412 (AMW007-SET), 499987 (AMW021-SET). — [Rosenberger HFM page](https://rosenberger.com/en/products/automotive/hfm1.php) [page]
- Waterproof versions are limited to up to 5 mating cycles. — [Rosenberger HFM catalogue 2022, Mouser copy](https://www.mouser.com/pdfDocs/AUTO_HFM_Catalog_2022.pdf) [snippet]
- HFM is described as saving up to 80 % space against FAKRA. — [Rosenberger HFM page](https://rosenberger.com/en/products/automotive/hfm1.php) [snippet]
- Molex second-sources the interface: 25 mating cycles maximum, -40 to +105 C, 1.0 A DC. — [Molex HFM datasheet](https://supplychain.molex.com/content/dam/molex/molex-dot-com/en_us/pdf/datasheets/987651-8942.pdf) [snippet]

**Cable assemblies and adapters**
- Rosenberger `LM4-350-1000`: "HFM M / SMA M Cable Assembly 1 M", no housing, listed at INR 7,914.71 for one piece and INR 5,856.36 at volume, 23 in stock on one listing. — [Mouser India HFM cable assemblies](https://www.mouser.in/en/c/connectors/rf-interconnects/rf-cable-assemblies/?series=HFM) [snippet]
- Molex off-the-shelf HFM assemblies come as HFM-to-HFM, HFM-to-FAKRA, HFM-to-terminal and HFM-to-blunt-cut. — [Mouser, Molex HFM cable assemblies](https://eu.mouser.com/new/molex/molex-hfm-cable-assemblies/) [snippet]; the page and the Molex product page both timed out when fetched.
- TZ Electronic Systems lists a quad HFM tap coupler for PoC links (CPL0710) and one without PoC (CPL0715), with no published price. — [TZ couplers](https://stz-es.com/coupler/) [page, from earlier notes]

### Inferences
- **The controller's header is a quad HFM plug (male), coding C.** The harness part is a jack ("K" in Rosenberger's code), so the mating ECU header should be the plug. A tap that goes in line needs a C-coded quad plug facing the harness and a C-coded quad jack facing the controller. This is inferred from the harness side only.
- **"No housing" matters for the LM4-350-1000.** A bare HFM contact has no coded housing, so it will not latch into a quad header by itself. It suits bench work with a matching housing, not an in-car connection.
- **15 GHz is far more than this link needs.** A 4.16 Gbps FPD-Link III forward channel has its fundamental near 2.1 GHz, where the connector's return loss is at least 25 dB. The connector is not the limiting part; the tap and extra cable are.
- **25 mating cycles is a real budget.** Each bench trial that unplugs `C003B*4F` uses one. Plan the work so the car's connector is opened only a few times, and do repeated trials on a salvage harness tail.
- **Coding C and D differ only in keying and colour.** The driver-camera connector `*5F` (bordeaux violet) cannot be swapped with the surround one (blue) by mistake.

### Gaps
- No confirmed product for a complete quad HFM coding-C plug-to-jack breakout or a quad HFM to 4 x FAKRA or 4 x SMA cable, and no price for one. Ask Rosenberger, Molex, or a cable house, quoting AMK12D-1M4Z5-C and its mating PCB plug.
- US-dollar prices on Mouser or Digi-Key were not read; the distributor pages blocked or timed out.
- The Rosenberger HFM catalogue PDF could not be downloaded (both mirrors returned an HTML block page), so the mating PCB header part number was not found.
- Whether coding Z (waterblue) is a neutral coding that mates with C is not stated in the datasheet.

## Which other vehicles use the same or a sibling Magna controller or camera architecture?

### Takeaway
I found no source that names another vehicle using this controller, the name "Hydra3", or the same camera part. This question is unanswered.

### Cited Findings
- Magna says the Ocean ADAS uses a "scalable domain controller architecture". — [Magna release, Jan 2021](https://www.magna.com/company/newsroom/releases/release/2021/01/04/news-release---magna-expands-with-fisker-secures-full-adas-system-business) [page, from earlier notes]
- An older report says Magna and Renesas partnered on a parking and surround-view system using the R-Car V3M SoC. — [TU-Auto](https://www.tu-auto.com/renesas-magna-partner-on-adas-sensors-for-automated-parking/) [snippet]. It does not mention Fisker.
- Magna's product page describes compute units that cover "surround view to parking" in one unit, without naming SoCs or customers. — [Magna compute units](https://magna.com/products/electrical-electronics/adas-automated-driving/domain-controllers) [snippet]

### Inferences
- "Hydra3" suggests a third generation of a Magna platform, so older siblings probably exist in other makers' cars. This is a guess from the name only.
- The Renesas link is a lead for the SoC family, not evidence. A TI FPD-Link hub works with any SoC that has CSI-2 inputs.
- The better-documented comparison is at chip level, not vehicle level: any quad AR0233 plus DS90UB953 plus DS90UB960 system (TI's TIDA-050035 and its hub evaluation kits) behaves like this link and has public documentation.

### Gaps
- No sibling vehicle, no Magna part cross-reference and no FCC or regulatory record for the controller were found. I ran two searches on this and did not check Magna supplier lists or other TechInsights Magna teardown titles.

## Where is the controller, how is it mounted and reached, and what are the connector labels?

### Takeaway
The service manual places the ADAS control unit in the rear compartment (cargo area) behind the lower side trim, not behind the dash. It is held by three M6 nuts and has six harness connectors, labelled `C003B*1F` to `C003B*6F` on the wiring set. Its fuse is MF07 (15 A) in the rear trunk fuse box.

### Cited Findings

**Removal procedure, "Control Unit - ADAS", service manual page 80-1 (PDF page 237)**
1. "Disconnect 12V battery."
2. "Remove RH rear compartment lower trim. Refer to Trim - Rear Compartment - Lower - LH."
3. "Remove M6x20 Torx screws (x8) securing trim mounting bracket. Torque 5 Nm."
4. "Remove trim mounting bracket."
5. "Disconnect harness connectors (x6) from ADAS control unit."
6. "Remove M6 nuts (x3) securing ADAS control unit to body. Torque 9 Nm."
7. "Remove ADAS control unit."
- Installation: "After installation perform the programming/calibration procedure using the Fisker Aftersales Service Tool."
- Source: [Service manual](https://drive.google.com/file/d/1BwRtG41yIPEU5H6KvpWaj1X8OWSvLYuf/view) [doc]. The step illustrations were not viewed; only the text was extracted.

**Trim that must come off first, "Trim - Rear Compartment - Lower - LH", page 40-29 (PDF page 133)**
- Remove the rear compartment upper trim, the rear compartment lamp and the carpet. Remove M6x30 bolts (x3) and clips (x8) of the scuff plate, then the scuff plate. Remove the partition box. Remove M6x20 screws (x2, then x2 more) and one M6x30 screw that secure the lower trim to the body. — [Service manual](https://drive.google.com/file/d/1BwRtG41yIPEU5H6KvpWaj1X8OWSvLYuf/view) [doc]

**Mounting and supply**
- Parts: `FM2980140040A BRACKET_ADAS_CONTROLLER` ("Also supports the lift ga..."), `N000000325 NUT-M6-8` (qty 6), `N000000011 SCREW-M6X20-8.8` (qty 4). — [Parts book](https://drive.google.com/file/d/1j-PSGpu0bNz-jYVWydXP8Ga0_e9ujeEU/view) [doc]
- Fuse `MF07` Micro 15 A "ADAS DC" is on the rear trunk fuse box sheet. Ground `Cl.31_ADAS` is on "GROUND BODY SHEET 1". — [fuses.json](https://fisker-ocean-archive.preetham-981.workers.dev/data/fuses.json); [Wiring set p.19, p.20](https://drive.google.com/file/d/1SY0NpmfMqZCx2eLu4gMEKs2z0kN_mZQT/view) [doc]
- An owner fix for an ADAS fault involves the MF07 ADAS fuse in the trunk. — [Ocean Forums thread 3075](https://www.oceanforums.com/threads/2023-fisker-ocean-adas-fault-braking-failure.3075/) [snippet]
- Owners report an "ADAS fan" that runs continuously; a video and an owners-association thread describe fixes. — [YouTube: Fisker Ocean - ADAS Fan Fix?](https://www.youtube.com/watch?v=sXyRU3yCyog) [snippet]; [FOA resources page](https://fiskeroa.com/?p=547) [page]. The linked forum thread needs a login and was not read.

**Coax routes to the controller (wiring sheet VII.4)**
- Front camera `V282A`: bumper harness, inline `I136A`, engine-room harness, inline `I141B`, body harness, then `C003B*4F` position `D*04`. Wires W2172, W2171, W2170.
- Rear camera `V283D`: rear bumper harness, inline `I147D`, body harness, position `D*03`. Wires W2173, W2174.
- Left camera `V005J`: driver door harness, inline `I151J`, body harness, position `D*02` (LHD). Wires W1407, W1406.
- Right camera `V006K`: passenger door harness, inline `I152K`, body harness, position `D*01` (LHD). Wires W1408, W1409.
- Output `DIG_OUT_LVDS`: `C003B*3F` to ICC head unit `C028B*9F`, wire W1714, drain W3045.
- Source: [Wiring set p.28, p.51](https://drive.google.com/file/d/1SY0NpmfMqZCx2eLu4gMEKs2z0kN_mZQT/view) [doc]

### Inferences
- **The side is uncertain.** The step says "RH" trim but refers to the "LH" trim procedure. Check both sides, or locate the unit from the fan noise or the rear trunk fuse box position, before removing trim.
- **The "B" in `C003B` most likely means the body harness.** The sheet marks every wire into these connectors with the zone "BODY". This matches the unit being in the rear body, not the cockpit.
- **The bracket is shared with another unit, probably the power liftgate module** ("Also supports the lift ga..."). So more than one module sits behind that trim.
- **The controller has a cooling fan.** Owners' "ADAS fan" reports suggest it. Keep airflow clear when adding a tap or capture hardware behind the trim.
- **The trunk location is good for a tap.** All four surround coax links end in one quad connector there, the 12 V fuse box is close, and there is room behind the trim. The cost is distance from the dash: the tap's video must still be carried forward to wherever it is used.
- **The front camera link is the longest and has two inline joints,** so it has the least signal margin. Test a tap on the rear camera channel first, where a short run gives the most margin, and treat the front channel as the hardest case.
- **Unplugging the unit will set faults** (camera component and communication codes) and the manual calls for programming or calibration after a replacement. A reconnect of the same unit should not need calibration, but that is my assumption; the manual does not say.

### Gaps
- The service manual's illustrations (exact position, connector faces) were not viewed. They are in PDF page 237.
- No connector end views or cavity numbering for `C003B*4F` were found in the wiring set text.
- No owner photo or forum post showing the controller in place was read. The owners-association thread is login-gated.
- Whether the six-connector count and layout changed on later builds is unknown; the wiring set is MY2023 SORP.
