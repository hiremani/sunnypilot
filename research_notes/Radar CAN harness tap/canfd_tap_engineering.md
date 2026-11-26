# Electrical engineering of a passive listen-only tap on an automotive CAN-FD radar link

Scope: add a listen-only node to an existing point-to-point CAN-FD link (radar to ADAS controller), 12 V vehicle, data phase up to 5 Mbit/s. Each item is marked **[Sourced]** (cited), **[Assessment]** (my engineering inference), or **[Background]** (standard CAN knowledge I did not re-verify with a fetched source in this session).

## Q1. Stub length, ringing and SIC at 2 and 5 Mbit/s (ISO 11898-2:2016 / CiA 601)

### Takeaway
No public, citable numeric stub limit exists for CAN-FD at 2 or 5 Mbit/s. ISO 11898-2's often-quoted 0.3 m figure comes from 1 Mbit/s classic CAN. The CiA 601-3 numbers are paywalled. TI's physics rule of thumb gives about 0.7 to 1.3 m, depending on transceiver edge speed. A 0.3 to 1 m stub on a 5 Mbit/s point-to-point link is therefore marginal and must be measured. The safe design is to put the listening transceiver within a few cm to about 10 cm of the splice and run the long cable on the logic/USB side. A SIC transceiver in the *listener* does not reduce the ringing caused by its own stub.

### Cited Findings
- [Sourced] TI SLLA486: "A stub is the electrical length of cable between a node's terminal and connection to the CAN bus... Since stub-lines are unterminated, signal reflections can develop in a stub that drives signal levels back and forth across a receiver's input thresholds, creating errors." — [TI SLLA486 CAN transceiver FAQ](https://www.ti.com/document-viewer/lit/html/SLLA486A/GUID-66620DAB-AFC0-41A9-B934-92931CA2ED3D)
- [Sourced] Same TI note: "The ISO 11898-2 Standard specifies a maximum bus length of 40 meters, max speed of 1 Mbps, and maximum stub length of 0.3 meters. However with careful design, stub lengths can be longer." The 0.3 m figure is a 1 Mbit/s classic-CAN figure, not a CAN-FD data-phase figure. — [TI SLLA486](https://www.ti.com/document-viewer/lit/html/SLLA486A/GUID-66620DAB-AFC0-41A9-B934-92931CA2ED3D)
- [Sourced] TI conservative rule: `2 × propagation delay of stub ≤ (1/3) × transceiver rise or fall time`. Worked example (ISO1044, fall time 40 ns typical, 5 ns/m in twisted pair): `2·x·5 ns/m ≤ 40/3`, so x ≤ 1.33 m. TI says longer stubs "may be possible with slower data rates" and recommends "thorough testing". — [TI SLLA486 (PDF rev C, section 13)](https://www.ti.com/document-viewer/lit/html/SLLA486A/GUID-66620DAB-AFC0-41A9-B934-92931CA2ED3D)
- [Sourced] NXP SIC white paper: ISO 11898-2:2016 defines CAN FD up to 5 Mbit/s, but "quickly a new speed limit" appeared. Ringing "created by unterminated stubs (or branches)" plus impedance plateaus kept the recessive signal from settling below the 0.5 V minimum receiver threshold before the sample point ("typically around 70-80% of the bit time"). "CAN FD became effectively limited to 2 Mbit/s communication outside of point-to-point connections." — [NXP CANSIGIMP white paper](https://www.nxp.com/docs/en/white-paper/CANSIGIMP-WP.pdf)
- [Sourced] NXP: SIC "actively bring[s] the recessive signal to below the minimum recessive threshold level of 0.5 V" when that node transmits. "Should a CAN Signal Improvement transceiver be used in a legacy network, the only side effect will be an improved ringing performance when that node transmits." ISO 11898-2:2016 defined symmetry values for 2 and 5 Mbit/s. NXP SIC parts use tighter bit-rate-independent values (CiA 601-4 v2.0.0). — [NXP CANSIGIMP white paper](https://www.nxp.com/docs/en/white-paper/CANSIGIMP-WP.pdf)
- [Sourced] CiA: the SIC handling formerly in CiA 601-4 has been withdrawn from CiA and is now specified in ISO 11898-2. CiA 601-3 (system design, bit-timing spreadsheet) is the CiA system-design document. — [CiA 601 series page](https://can-cia.org/can-knowledge/cia-601-series-can-fd-guidelines-and-recommendations)
- [Sourced] SAE J2284-5 defines 500 kbit/s arbitration and a 5 Mbit/s CAN-FD data phase. CiA describes it as suitable for "point-to-point communication". J2284-4 covers 2 Mbit/s line topology. The stub clauses are paywalled. — [SAE J2284/5](https://saemobilus.sae.org/content/J2284/5_201609); [CiA seminar note](https://can-cia.org/news/cia-in-action/view/can-and-canopen-seminars-in-nuremberg/)
- [Sourced] CiA 2020 paper (Adamson/Engelhard/Zhang) simulates that CAN-SIC reduces recessive ringing and makes 5 Mbit/s networks feasible. It gives no stub figure in the retrieved excerpt. — [CiA iCC 2020](https://www2.can-cia.org/fileadmin/resources/documents/proceedings/2020_adamson_engelhard_zhang.pdf)
- [Sourced] TI E2E (forum, not normative): a 2 Mbit/s design ran a 40 m bus with stubs over 1.5 m, with a warning that "stubs will add impedance mismatch if they are too long". — [TI E2E TCAN1043-Q1](https://e2e.ti.com/support/interface-group/interface/f/interface-forum/1420726/tcan1043-q1-can-bus-length)
- [Sourced, unverified] A TI E2E thread cites a "0.5 m branch" limit from an unnamed CAN-FD network specification. The thread reads as machine-generated, so do not rely on it. — [TI E2E TCAN3403-Q1](https://e2e.ti.com/support/interface-group/interface/f/interface-forum/1690237/tcan3403-q1-achievable-data-rate-and-ringing-assessment-for-a-3-3v-can-fd-transceiver)
- [Sourced] Contrast: SAE J1939-11 allows stubs up to 1 m on a 40 m, 250/500 kbit/s backbone. J1939-15 allows 3 m. — [NI CAN physical layer guide](https://www.ni.com/en/support/documentation/supplemental/09/can-physical-layer-and-termination-guide.html); search summary of J1939 material

### Inferences
- [Assessment] Bit times: 2 Mbit/s = 500 ns, 5 Mbit/s = 200 ns. A 1 m stub has a round trip of about 10 ns (2 × 5 ns/m). Modern automotive CAN-FD transceivers have edges of about 20 to 50 ns. TI's rule then gives about 0.7 to 1.7 m. The reflection energy settles early in a 500 ns bit, but takes a larger share of a 200 ns bit whose sample point is at about 70 to 80% (140 to 160 ns).
- [Assessment] Practical numbers for this project:
  - Target: transceiver ≤ 0.1 m from the splice, at both 2 and 5 Mbit/s.
  - 0.3 m: probably acceptable at 2 Mbit/s and probably acceptable on a short point-to-point link at 5 Mbit/s. Verify with a scope.
  - 1 m: likely acceptable at 2 Mbit/s and risky at 5 Mbit/s. Do it only with a scope check at both ends.
  - Note that the OEM link was validated with zero stubs.
- [Assessment] Point-to-point links are the best case for CAN-FD (NXP's "outside of point-to-point" caveat). The OEM probably chose 5 Mbit/s or a high data rate *because* the link has no stubs. A tap turns the link into a stubbed topology, which is the exact situation that limited conventional transceivers to 2 Mbit/s.
- [Assessment] SIC matters only for nodes that transmit. A silent listener's SIC never activates, so it does nothing for the reflection from its own stub. What matters in the listener is: (a) a short stub, (b) a transceiver rated for the data rate (ISO 11898-2:2016 "5 Mbit/s" symmetry grade, or SIC grade for its tighter receiver symmetry), and (c) low pin capacitance. A SIC transceiver in the *radar or controller* (unknown for Fisker) would help tolerate the stub.
- [Assessment] The stub is measured to the transceiver pins, so adapter PCB traces and connector pigtails count as stub length.

### Gaps
- No public CiA 601-3 or ISO 11898-2:2016 text with a stub-length number at 2 or 5 Mbit/s. CiA 601-3 needs membership or purchase.
- The TI SLLA581 SIC measurement section and the J2284-5 topology clause could not be retrieved.
- Unknown: the Fisker radar link's actual data-phase rate, sample point, and whether its transceivers are SIC types. These need log measurement or a scope.

## Q2. How professionals tap vehicle CAN for logging, and the trade-offs for CAN-FD

### Takeaway
The professional order of preference is:
1. A connector-level Y or T harness with mating OEM housings and terminals.
2. An OEM test breakout.
3. A properly sealed splice.

Loggers run in silent (monitoring) mode with no termination. Contactless inductive or capacitive clamp readers do not support CAN-FD.

### Cited Findings
- [Sourced] CSS Electronics: "CAN FD is not supported (this generally applies to contactless CAN readers)." — [CSS contactless CAN reader](https://csselectronics.com/products/contactless-can-bus-reader-adapter); [CANCrocodile page](https://csselectronics.com/products/cancrocodile-contactless-can-adapter)
- [Sourced] Copperhill: CANCrocodile is reliable to 1000 kbit/s. With CAN FD (up to 8 Mbit/s) it "will not be able to capture all data accurately." One reseller (ControlTech) lists "CAN FD" compatibility, which conflicts with the manufacturer. Treat the reseller listing as wrong. — [Copperhill](https://copperhilltech.com/cancrocodile-contactless-can-bus-reader/); [ControlTech listing](https://controltech.uk.com/products/css-electronics-cancrocodile-contactless-can-bus-reader)
- [Sourced] CANedge2 datasheet: "Configurable silent mode: Restricted (acknowledge only) or monitoring (zero transmission)". The transceiver has "Ideal passive behavior when unpowered (high impedance / no load)". The CANedge install guide says to keep the stub short and use twisted data lines. — [CANedge2 datasheet](https://www.instrumentchoice.com.au/attachment/download/132253/65027539847a2327751907.pdf); [CANedge1 installation](https://canlogger.csselectronics.com/canedge-docs/ce1/hardware/installation.html)
- [Sourced] Kvaser: a silent device "does not participate in the normal CAN protocol by transmitting an Acknowledge (ACK) slot" and "does not transmit any CAN data frames, error frames, or any other frames." — [Kvaser: What is silent mode](https://kvaser.com/knowledge-base/what-is-silent-mode/)
- [Sourced] Kvaser: error frames usually mean a configuration mismatch. The interface must use the same bit rate, sample point and SJW as the other nodes. Silent mode cannot be used when there is only one other node, because nothing would ACK. — [Kvaser bus configuration lesson](https://kvaser.com/lesson/bus-configuration/); [Kvaser KB](https://kvaser.com/?p=17877)
- [Sourced] Telematics practice: use "a vehicle-specific harness when offered". T-harnesses plug between the OEM connectors. "Do not connect two T-harnesses together." — [Geotab harness cheat sheet](https://support.geotab.com/harnesses/installation/doc/harness-cheat); [Geotab T-harness guide](https://support.geotab.com/en-GB/harnesses/installation/doc/hrn-gs16k22a-16-pin)
- [Sourced] NHTSA bulletin: third-party devices often use a "T" or "Y" connector at the OEM connector, or install the module in-line. — [NHTSA MC-10207266](https://static.nhtsa.gov/odi/tsbs/2022/MC-10207266-0001.pdf)
- [Sourced] Commercial CAN Y-splitters exist "to insert a CAN device in the middle of a CAN-bus segment". — [SBD Motorsport CAN Y interface](https://sbdmotorsport.co.uk/product/can-y-interface-for-devices/)
- [Sourced] Linux-CAN discussion: some PEAK devices can transmit while in "monitor" mode, which kernel developers consider a violation of ISO 11898-1 ("does not start a transmission"). Verify the silent mode of the chosen hardware. — [linux-can thread](https://lore.kernel.org/all/20210504100746.GB27660@x1.vandijck-laurijssen.be/)

### Inferences
- [Assessment] For a CAN-FD point-to-point radar link, the best fit is a Y-harness at the radar or controller connector. The logger's transceiver should sit on a small board right at the Y, with an inner stub of ≤ 10 cm, in silent/monitoring mode with no termination. Restricted (ACK-only) mode is not needed, because both link partners already ACK each other.
- [Assessment] A contactless reader is excluded at CAN-FD rates.

### Gaps
- No Vector or PEAK application note on CAN-FD tap stub lengths was retrieved.

## Q3. Error frames, bus-off risk, ACK behaviour and link-partner reaction (DTCs)

### Takeaway
A correctly configured silent tap cannot cause protocol errors. Its only risk is physical: stub ringing, a short, or loading. A non-silent (normal-mode) controller is the real danger:
- At a wrong bit rate or sample point, or if it mis-samples ringing, it sends active error flags that destroy the radar frames.
- If it ACKs, it hides nothing, because the partner already ACKs.

A radar or ADAS ECU that misses frames usually reports "lost communication" (U0235-class) DTCs and turns off ACC/AEB.

### Cited Findings
- [Sourced] Microchip: in Listen-Only mode the module "will receive messages, but it will not transmit any bits." ISO 11898-1 calls this "bus monitoring mode". — [Microchip Listen Only Mode](https://onlinedocs.microchip.com/oxy/GUID-D6315DA4-4399-4D39-80F4-F1F66A173640-en-US-4/GUID-155CE676-0DE6-40B0-971E-3D49FBFF13DC.html); [TI C2000 TRM](https://ti.com/document-viewer/ja-jp/lit/html/SPRUHM9H/GUID-00FA2411-E2A2-42B4-878D-E8C0E402505B)
- [Sourced] Kvaser: "If no unit can acknowledge a sent frame, the sending unit will create error frames." A wrong-rate node in normal mode produces error frames. — [Kvaser](https://kvaser.com/?p=17877)
- [Sourced] CSS Electronics CANedge offers an ACK-only "restricted" mode and a "monitoring (zero transmission)" mode. — [CANedge2 datasheet](https://www.instrumentchoice.com.au/attachment/download/132253/65027539847a2327751907.pdf)
- [Sourced] U0235 = "Lost Communication With Cruise Control Front Distance Range Sensor". Causes include CAN wiring and connector faults. Symptoms: ACC is disabled and a pre-collision system malfunction warning appears. — [OBD Advisor U0235](https://obdadvisor.com/codes/code/u0235-code/); [Lexus U0235 procedure](https://manuals.autorepairshq.com/Lexus/2006/GS%20430%20V8-4.3L%20%283UZ-FE%29/Repair%20and%20Diagnosis/A%20L%20L%20%20Diagnostic%20Trouble%20Codes%20%28%20DTC%20%29/Testing%20and%20Inspection/U%20Code%20Charts/U0235)

### Inferences
- [Background] ISO 11898-1 error confinement: the transmit error counter rises by 8 for each transmit error, and receive errors raise the receive counter. At 128 the node goes error-passive. At TEC > 255 it goes bus-off. A normal-mode tap that keeps sending error flags can push the *radar* toward error-passive or bus-off. Recovery takes 128 × 11 recessive bits, and many ECUs latch a DTC.
- [Assessment] Failure modes ranked by likelihood:
  1. The tap is in normal mode with a bit-timing mismatch (a data-phase sample point or TDC mismatch is easy to get wrong at 5 Mbit/s). Result: error flags, frames destroyed, radar timeout DTC, ACC/AEB lost. Silent mode removes this risk completely.
  2. Stub ringing corrupts reception at one of the two real nodes. Result: that node sends error frames, which raise counters and can cause DTCs. Silent mode does not prevent this. Only a short stub prevents it.
  3. An unpowered or faulty tap transceiver loads the bus. Use a part that is high-impedance when unpowered (ISO 11898-2 requires low leakage when unpowered; CANedge advertises it).
  4. A CAN_H/L short to ground or 12 V, or between H and L, from a bad splice. Result: link lost.
- [Assessment] An ACK from the tap is harmless on a two-node link but serves no purpose. Monitoring mode is preferred because the tap then has zero influence on the bus (a node that ACKs also drives the bus during the ACK slot, and a node in normal mode can send error flags).
- [Assessment] The project has a related, already-known failure: a wrong SecOC key silently blocks steering. Treat any new radar DTC after the tap install as a tap problem until proven otherwise.

### Gaps
- No Fisker-specific DTC list for the radar link. Radar-to-ADAS private links may report their own timeout DTCs instead of standard U-codes.
- No source on how a specific radar (e.g. Bosch or Continental) reacts to error-passive conditions.

## Q4. Physical install: Y-adapter at a sealed connector, splices, termination, wiring, power/ground, ESD, fusing, environment, reversibility, and signal-integrity verification

### Takeaway
Use a Y-adapter built from mating housings and terminals of the OEM connector family (male side to the harness, female side to the ECU). Keep every pin straight through, including the seals. Take CAN_H/L off at the Y with ≤ 10 cm of tightly twisted pair to the listening transceiver. Add no terminator. The link stays at about 60 Ω. If splicing is unavoidable:
- Use equal-length, staggered, adhesive-lined crimp or solder-sleeve splices ≥ 100 mm from the connector.
- Untwist no more than about 30 to 50 mm.
- Do not use IDC taps.

Verify with a differential scope at both ends (recessive settled < 0.5 V before the sample point, and a data-phase eye) and with zero error counters and DTCs over a drive.

### Cited Findings
- [Sourced] Termination: the backbone is terminated "at each end with a 120 ohm resistor... total resistance... 60 ohms". Only end devices are terminated. ISO 11898 requires cable with a nominal impedance of 120 Ω. — [NI CAN physical layer and termination guide](https://www.ni.com/en/support/documentation/supplemental/09/can-physical-layer-and-termination-guide.html)
- [Sourced] Tesla wiring repair rules:
  - "Do not perform an inline repair within 100 mm of a connector" (use a new pigtail instead).
  - "Stagger the splices as much as possible."
  - Do not splice inside grommets or seals, or in areas that move.
  - "Apply tape to 50 mm of the harness on each side of the splice."
  - The single-splice method is "Not applicable for CAN repairs. CAN wires must be the same length on CAN H and CAN L."
  - Circuits marked "++" "cannot... be spliced".
  - Coax is not repairable.
  - Heat shrink is 4.0 mm for 26 to 20 AWG.
  — [Tesla Model 3 wiring harness repair](https://service.tesla.com/docs/Model3/ServiceManual/en-us/GUID-3B58243E-5655-4F56-9D8F-0CDAACAF9BA8.html)
- [Sourced] HP Academy demonstrates untwisting 30 to 40 mm of a CAN pair to splice. "The tricky part with splicing CAN bus wiring is keeping the pair twisted." — [HP Academy CAN splicing](https://www.hpacademy.com/courses/canbus-communications-decoded/practical-discussion-splicing-together-new-canbus-wires)
- [Sourced, secondary] Study-guide rule: about 25 to 50 mm per twist, ≤ 50 mm untwisted in total, splice each conductor separately, and solder joints without strain relief crack under vibration. This is not an OEM spec. — [open-exam-prep wiring repair](https://open-exam-prep.com/study-guides/ca-red-seal-automotive-service-tech/electrical-electronics-a/circuit-wiring-repair)
- [Sourced] Solder sleeves (solder ring plus heat shrink) are widely used in automotive and aerospace splicing. — [RS solder sleeve guide](https://uk.rs-online.com/web/content/discovery/ideas-and-advice/guide-to-solder-sleeves)
- [Sourced] IDC/Scotchlok: 3M markets some IDCs for low-voltage automotive use. Some are gel-free. Users report corrosion, rising resistance and vibration failures (anecdotal). No OEM prohibition was found. — [3M IDC taps](https://www.3m.com/3M/en_US/p/c/electrical/splices-accessories/idc-taps/); [Garage Journal thread](https://www.garagejournal.com/forum/threads/why-do-scotchlocks-have-such-a-bad-reputation.539967/latest); [Panbo/PMY](https://powerandmotoryacht.com/uncategorized/3m-scotchloks-my-love-so-wrong/)
- [Sourced] Eye diagrams: build separate eyes for the arbitration and data phases. Most tools build only the data-phase eye. Scope clock recovery must emulate a CAN receiver's resynchronisation. — [CiA CNLM 2024, Teledyne LeCroy](https://can-cia.org/fileadmin/cia/documents/publications/cnlm/december_2024/cnlm_24-4_p38_eye_diagrams_for_can_thomas_stueber_teledyne_lecroy.pdf); [Keysight CAN FD eye mask app note](https://www.keysight.com/kr/ko/assets/7018-04704/application-notes/5992-0437.pdf)
- [Sourced] Pass criterion: the recessive differential voltage must be stable below 0.5 V (the ISO 11898-2:2016 minimum receiver threshold) by the sample point, at 70 to 80% of the bit. — [NXP CANSIGIMP white paper](https://www.nxp.com/docs/en/white-paper/CANSIGIMP-WP.pdf)
- [Sourced] Keysight/CiA 2020: measure transceiver loop delay symmetry and recessive bit width with a scope. — [Hancock, CiA iCC 2020](https://www2.can-cia.org/fileadmin/resources/documents/proceedings/2020_hancock.pdf)

### Inferences
- [Assessment] Termination: do not add a third 120 Ω. The link drops to 40 Ω, which reduces the differential amplitude, and the tap point becomes an impedance discontinuity. The tap device's switchable terminator must be OFF. Check with the ignition off and both ECUs connected: about 60 Ω between CAN_H and CAN_L with the tap installed, the same as without it.
- [Assessment] Y-adapter build:
  - Identify the connector family from the harness (TE MCON/MQS, Molex, Yazaki, Aptiv, Rosenberger etc.).
  - Buy a genuine mating plug and receptacle, the correct terminals, wire seals and cavity plugs, and crimp with the specified applicator or tool.
  - Keep the connector position assurance (CPA) and terminal position assurance (TPA) features.
  - Pass all pins straight through, with the pair twisted through the Y body.
  - A short (≤ 15 to 20 cm) adapter adds about 2 × 15 cm to the *main line*. That length is in series, not a stub, so it matters little if it is twisted pair near 120 Ω.
  - Only the branch to the listener is a stub.
  - Fully reversible: unplug the Y and reconnect the OEM connector.
- [Assessment] Splice ranking for a permanent stub on CAN-FD:
  1. Connector-level Y (best: reversible, sealed, no harness damage).
  2. Crimp splice or solder sleeve with adhesive-lined heat shrink, equal-length legs, staggered, ≥ 100 mm from the connector, taped 50 mm each side.
  3. Inline T on a short PCB.
  4. IDC/vampire taps: avoid. They give an uncontrolled stub geometry, can cut strands of thin 0.35 mm² CAN wire, let water in, and loosen under vibration.
  A splice "T" makes the stub start at the splice. A splice also cannot be reversed without leaving repair joints in the OEM harness.
- [Assessment] Wiring: use unshielded twisted pair of 0.35 to 0.5 mm² (22 to 20 AWG) with about 120 Ω differential impedance and about 25 to 40 mm lay, matching the OEM. Most OEM CAN-FD pairs are unshielded. If the OEM pair is shielded, keep the shield continuous through the Y and grounded only where the OEM grounds it. Do not run the stub beside 12 V PWM, ignition or HV lines.
- [Assessment] Ground and power:
  - Power the listener from a fused switched/ignition feed (e.g. a 1 to 3 A fuse via an add-a-fuse in an appropriate slot) or from the comma's own supply.
  - Reference the transceiver ground to vehicle chassis/battery ground near the tapped ECU, so the CAN common-mode stays within ISO 11898-2's range (−2 to +7 V, [Background]).
  - Never leave the transceiver unpowered while its supply is backfed through I/O pins. Use a part specified as high-impedance when unpowered.
  - An isolated transceiver (e.g. TI ISO1044) gives the most protection if the logger is grounded elsewhere (laptop or comma). It adds a few tens of ns of loop delay, which does not matter for a listen-only node.
- [Assessment] ESD and transients: use an AEC-Q100 automotive CAN-FD transceiver rated for ±58 V bus-fault protection and IEC 61000-4-2 ESD (TCAN1462/TCAN1044 class, [Background]). If you add a TVS, use a low-capacitance dual CAN TVS (e.g. PESD2CAN class, a few pF), because extra capacitance on a 5 Mbit/s stub worsens ringing. Do not add a common-mode choke on a listen-only stub unless you test it. A choke can itself ring with stub inductance.
- [Assessment] Environment: radar connectors at the front bumper are exposed to water, salt and wash spray. Keep the OEM seals and use sealed mating parts. Strain-relieve the Y every 150 to 200 mm, following Tesla's practice. Mount the listener board in a sealed (IP67-class) enclosure inside the bumper or cabin, not hanging loose. The ADAS controller end, usually in the cabin or trunk, is the far better tap location for environment and service access.
- [Assessment] Verification procedure:
  1. Ignition off: measure about 60 Ω across H and L, and check there is no short to ground or 12 V.
  2. Ignition on, before the install: capture a baseline differential CAN_H − CAN_L at both the radar and controller ends. Use a ≥ 200 MHz bandwidth differential probe or two probes with math, at ≥ 1 GS/s.
  3. After the install: repeat at both ends and at the tap. Compare recessive settling below 0.5 V before the sample point (data phase 140 to 160 ns into a 200 ns bit at 5 Mbit/s), overshoot, and the data-phase eye opening.
  4. Logger in monitoring mode: record error-frame counts and the bus-state (error-active) flag. Expect zero.
  5. After a drive: scan for DTCs on the radar and ADAS controller, and confirm ACC/AEB availability and no radar dropouts in openpilot logs (e.g. radar message frequency and counter checks).
  6. Do a vibration/thermal soak drive before trusting the install.

### Gaps
- No OEM (GM/Ford/Fisker) CAN-FD-specific splice spec was retrieved. Tesla's rules stand in as a representative OEM practice.
- No Keysight CAN-FD eye mask dimensions were retrieved.
- The connector family and pinout of the Fisker radar and ADAS controller are unknown and must be identified from the vehicle.
- No source was found for terminal-level sealing and retention checks. Connector-maker application specs (e.g. TE 114-series) are where to look.
