# Passively tapping an automotive camera serial link (FPD-Link III/IV, GMSL2): engineering reference

Evidence labels: **[datasheet]** means I extracted and read the text of the maker's PDF. **[page]** means I read the web page. **[snippet]** means I saw only a search-result summary, so check the page before relying on it. Everything under "Inferences" is my reasoning or arithmetic and is not confirmed by a source. Nothing here has been tested on hardware. Research date: 2026-11-23.

These notes extend two earlier files and do not repeat them: `pulse/research_notes/Fisker Ocean ICC camera streams/tapping_options.md` and `pulse/research_notes/Fisker Ocean camera options and vendors/connectors_and_capture.md`. New material here: TI channel and PoC numbers, the DS90UB960 back-channel and status registers read from the datasheet, the GMSL2 link-up sequence read from the MAX9296A datasheet, the TZ coupler flyer (which resolves the PoC model map and "coupler mode"), and the finding that the other commercial "taps" are active in-line devices.

## Physics and signal integrity: couplers, resistive pick-offs, active buffers, margin and PoC

### Takeaway
The original link has a hard, published loss budget (TI: 19.2 dB at 2.1 GHz for the whole channel, at most 4 in-line connectors on a 6 to 15 m run), so a tap is affordable only if its through-loss is a fraction of a dB and it adds no resonance below Nyquist. No vendor publishes insertion loss or coupling factor for an automotive video coupler, so the numbers for a specific tap must be measured.

### Cited Findings

**Frequency plan the tap must respect (TI FPD-Link III)**
- DS90UB953 with DS90UB954 or DS90UB960, synchronous mode: back channel 50 Mbps (fBC-MIN 25 MHz, fBC-MAX 50 MHz), forward channel maximum 4200 Mbps, forward Nyquist 2100 MHz, frame clock 105 MHz. Cable reach 15 m on Dacar 462 coax, 10 m STQ. — [TI channel requirements, Table 1](https://e2e.ti.com/cfs-file/__key/communityserver-discussions-components-files/138/Channel_2D00_Requirements_2D00_ADAS-chipsets-FPD-III.pdf) [datasheet]. Note: the PDF is headed "TI Confidential - NDA Restriction" but is hosted publicly on TI's E2E forum. It is revision 0.8, dated 2018.
- Same pair in external-clock (non-synchronous) mode: back channel 10 Mbps (5 to 10 MHz), forward 4200 Mbps. DS90UB935/936: 25 Mbps back, 2100 Mbps forward. DS90UB933 or 913A (RAW/DVP parts): back channel 2.5 Mbps (1.25 to 2.5 MHz), forward 1870 or 1400 Mbps. — same document, Tables 1 [datasheet]
- Forward rate in synchronous mode is exactly 160 x REFCLK of the deserializer; back channel is 2 x REFCLK. With REFCLK = 25 MHz: 4.0 Gbps forward, 50 Mbps back. In non-synchronous mode the line rate is typically CLK_IN x 80, with back channel set to 10 Mbps. — [DS90UB960-Q1 datasheet SNLS589D, section 7.4.1](https://www.ti.com/lit/ds/symlink/ds90ub960-q1.pdf) [datasheet]

**Channel budget on coax, 4 or 2 Gbps (TI Table 2, total channel requirement)**
- Insertion loss S21 limits: -1.4 dB at 1 MHz, -2.3 dB at 5 MHz, -2.5 dB at 10 MHz, -3.5 dB at 50 MHz, -4.5 dB at 100 MHz, -9.5 dB at 500 MHz, -14 dB at 1 GHz, -21.6 dB at 2.1 GHz. — [TI channel requirements, Table 2](https://e2e.ti.com/cfs-file/__key/communityserver-discussions-components-files/138/Channel_2D00_Requirements_2D00_ADAS-chipsets-FPD-III.pdf) [datasheet]
- Return loss S11 limits, total channel: -16 dB from 1 to 100 MHz, -9 + 7 x log(f) dB from 0.1 to 1 GHz, -9 dB from 1 to 2.1 GHz. The PCB (with PoC) and cable-assembly budgets are tighter: -20 dB, -12 + 8 x log(f), and -12 dB. — same table [datasheet]
- PCB budget with PoC network: better than -0.35 dB up to 500 MHz, -0.6 dB at 1 GHz, -1.2 dB at 2.1 GHz. Cable-assembly budget: -12.8 dB at 1 GHz, -19.2 dB at 2.1 GHz. — same table [datasheet]
- In-line connector count: at most 1 below 4 m, at most 2 for 4 to 6 m, at most 4 for 6 to 15 m. Impedance: cable 45 to 55 ohm, connector region 40 to 62.5 ohm. Crosstalk: -30 dB far end below 2.1 GHz. — same table [datasheet]
- "Below Nyquist frequency, the transmission channel should be free of resonance that creates sharp drops in insertion loss and abrupt changes in the channel's group delay." — same document, Note 2 [datasheet]
- The public datasheet agrees: the DS90UB960 AEQ "can compensate for the transmission channel insertion loss of up to -19.2 dB at 2.1 GHz." TI's worked example budgets -1.2 dB per PCB and 0.1 dB per connector, and quotes Dacar 462 at about -1.31 dB/m at 2.1 GHz, leaving -16.6 dB for cable. — [DS90UB960-Q1 datasheet, section 7.4.7.1](https://www.ti.com/lit/ds/symlink/ds90ub960-q1.pdf) [datasheet]
- A Rosenberger HFM connector is specified at up to 0.1 x f(GHz) dB insertion loss (0.21 dB at 2.1 GHz). — earlier notes, [AMK12D datasheet](https://products.rosenberger.com/_ocassets/db/AMK12D-1M4Z5-Y.pdf)

**PoC network (what a tap must not load)**
- 4 Gbps synchronous mode: the PoC network must present high impedance over 25 to 2100 MHz, total DC resistance below 1.75 ohm, nominal inductance 10 uH. — [TI channel requirements, page 5](https://e2e.ti.com/cfs-file/__key/communityserver-discussions-components-files/138/Channel_2D00_Requirements_2D00_ADAS-chipsets-FPD-III.pdf) [datasheet]
- 2 Gbps PCLK-mode links (2.5 Mbps back channel): frequency range 1.25 to 1000 MHz, DC resistance below 1.75 ohm, nominal inductance 110 uH. — same page [datasheet]
- PoC supply disturbance limit: 500 mVpp for a pulse caused by a DC current change, at 30/60 Hz at the switcher input. — same page [datasheet]
- "A PoC network is built with several inductors in series to form a wide band inductor." — same page [datasheet]
- TI's suggested "4G" PoC parts start with a 10 uH inductor (Murata LQH3NPN100MJR / LQH3NPZ100MJR, 0.288 ohm max, 530 mA minimum, 30 MHz SRF minimum; TDK NLCV32T-100K-EFD alternative). The "2G" network uses 100 uH (Coilcraft MSS7341-104ML, 7.2 MHz SRF typical). — [DS90UB960-Q1 datasheet, Tables 8-1 and 8-2](https://www.ti.com/lit/ds/symlink/ds90ub960-q1.pdf) [datasheet]
- Coilcraft publishes PoC filter reference parts for both FPD-Link and GMSL. — [Coilcraft PoC page](https://www.coilcraft.com/en-us/products/emi/poc-page/) [snippet]

**GMSL2**
- Forward rate is fixed at 3 or 6 Gbps; reverse rate is fixed at 187.5 Mbps for camera, radar and lidar parts. Protocol overhead is about 14%, leaving about 2.6 or 5.2 Gbps forward and 162 Mbps reverse payload. — [MAX9296A datasheet](https://cdn.icstop.com/upload/pdfs/32/fd/32fdc6558965079574838d6317d194a7.pdf) [datasheet]
- ADI does not publish the channel limits in the datasheet: "Contact the factory for insertion and return loss guidelines." — same datasheet [datasheet]
- GMSL2 adapts continuously: initial equalizer adaptation happens during link lock and is repeated at about 1 Hz, using a built-in eye-opening monitor that raises a runtime alert on link degradation. — same datasheet [datasheet]
- Line-fault detection (open, short to battery, short to ground) "cannot be used when power is supplied over the link." — same datasheet [datasheet]

**What a commercial passive coupler claims**
- TZ's flyer (version 03.26) describes the couplers as passive, 50 ohm (FAKRA) or 100 ohm (H-MTD), 64 x 102.9 x 40 mm for CPL0210/0215/0610/0615. Only the legacy active models need +12 V. The PoC models route DC through the main path and give "the signal (without DC voltage component)" at the forward output. — [TZ coupler flyer](https://tz-es.com/wp-content/uploads/2023/05/TZ_Product_Flyer_Coupler.pdf) [datasheet]
- The flyer's technical-data table has no insertion loss, coupling factor, directivity, bandwidth, PoC current limit or temperature range. It says "lossless", which is a marketing word and not a measurement. — same flyer [datasheet]

### Inferences
- **Ideal through-loss of a directional coupler** is -10 x log10(1 - 10^(-C/10)) for coupling factor C: about 1.26 dB for 6 dB, 0.46 dB for 10 dB, 0.14 dB for 15 dB, 0.04 dB for 20 dB. A real part adds conductor and connector loss on top. This is textbook arithmetic, not a TZ figure.
- **Cost against the TI budget.** Inserting a coupler adds two connector interfaces plus the coupler. With TI's 0.1 dB per connector and a 10 dB coupler, expect roughly 0.7 to 1 dB at 2.1 GHz. That is equal to about 0.5 to 0.75 m of Dacar 462 at 1.31 dB/m. It also adds one or two in-line connectors against a limit of 4 on a long run. On a short in-cabin run this is small; on a link already near 19.2 dB it is not.
- **Tap signal level.** The coupled port is C dB below the main line, flat with frequency, on top of whatever cable loss precedes the tap. AEQ is designed to correct frequency-dependent cable loss, so a flat 10 to 20 dB drop competes with receiver sensitivity instead. This is the likely reason TZ's older models were "actively amplified" and why a low coupling factor (6 to 10 dB) or a wideband LNA on the tap arm may be needed. I found no source that states the minimum input amplitude for these receivers.
- **Resistive pick-off** (a series resistor from the line into a 50 ohm load). A 450 ohm resistor gives a 20 dB (10:1) probe and puts 500 ohm across the line: about 0.42 dB through-loss and about 26 dB return loss in the ideal case (my arithmetic). It is not directional, so it delivers the forward channel and the ECU's back channel superimposed. The stub length to the resistor must be millimetres at 2.1 GHz. It must also be DC-blocked, because the line carries PoC voltage.
- **Why direction matters.** The original deserializer cancels its own back-channel transmission because it knows what it sent. TI states that CMLOUT jitter "includes back channel echo". A listen-only receiver sees someone else's back channel and cannot subtract it. A directional coupler oriented toward the camera suppresses the ECU's back channel by its directivity; a resistive tap does not. On FPD-Link III the back channel sits at 25 to 50 MHz (or 1.25 to 10 MHz) and on GMSL2 at 187.5 Mbps, all inside the forward channel's band.
- **Coupler low-frequency roll-off.** Printed coupled-line couplers lose coupling at low frequency. Whether a given coupler passes the low-frequency content of the forward signal well enough is a property to measure with a VNA; I found no data.
- **Active buffer tap** (high-impedance amplifier bridging the line). This gives signal gain and reverse isolation but places an active input on the OEM line, with its own capacitance and failure modes. TZ marks its active couplers as not recommended for new designs and unusable for GMSL3 (earlier notes), which suggests the industry moved to passive.
- **PoC rules for a tap.** The through path must carry the camera current (TI network parts are rated around 450 to 530 mA) with well under an ohm, since TI allows 1.75 ohm for the whole PoC network. The tap arm must be capacitively coupled and must not have a DC path to ground or to the sniffer's own PoC supply. A coupler's coupled port is naturally DC-isolated; a resistive tap needs a series capacitor large enough to pass the lowest frequency of interest.
- **A tap that only wants video does not need the back channel**, so the tap arm can be high-pass filtered above 50 MHz for FPD-Link III. This helps reject the foreign back channel on a non-directional tap. For GMSL2 the reverse channel overlaps the forward spectrum too heavily for a simple filter.
- **Failure modes the OEM link would see** are a lower AEQ/eye margin, parity or CRC error counts rising, and intermittent loss of lock, most likely cold or hot. A camera link can degrade without any DTC.

### Gaps
- No measured S-parameters for any TZ coupler, and none for any competing coupler. Ask TZ for S21, S11, coupling and directivity to 3 GHz (FPD-Link III) or 6 GHz (GMSL2 6G), and the PoC current rating.
- ADI's GMSL2 channel specification (insertion and return loss masks, PoC filter rules) is in the "GMSL2 Hardware Design Guide" and "GMSL2 General User Guide". Both ADI URLs timed out or refused the connection, so I have no GMSL2 loss numbers. URLs to retry: [hardware design guide](https://wcm-cce.cldnet.analog.com/media/en/technical-documentation/user-guides/gmsl2-hardware-design-guide.pdf), [general user guide](https://www.analog.com/media/en/technical-documentation/user-guides/gmsl2-general-user-guide.pdf).
- FPD-Link IV (DS90UB971/9702, 7.55 Gbps class) channel numbers were not researched.
- TI's PoC application notes SNLA224 and the "PoC AppNote 97x" on E2E were located but not read: [SNLA224](https://edgeworker.ti.com/lit/pdf/snla224), [97x PoC note](https://e2e.ti.com/cfs-file/__key/communityserver-discussions-components-files/138/5140.PoC_5F00_AppNote_5F00_97x.pdf).
- Minimum receiver input amplitude for DS90UB954/960 and MAX9296A was not found.

## Listen-only deserialization: can a second deserializer lock to a link it did not negotiate?

### Takeaway
For FPD-Link III the datasheet supports the idea: the receiver recovers clock from the forward channel, the back channel can be switched off by register, and status registers report input frequency, lock, parity and line counts without any serializer access. For GMSL2 the datasheet describes link-up as a two-way handshake and documents no receive-only mode, so the only evidence that it works is TZ's statement that its own deserializers have a "coupler mode". No TI or ADI document that I found mentions sniffing at all.

### Cited Findings

**FPD-Link III (DS90UB960)**
- The DS90UB954 is described in the Linux driver patch that adds it as "mostly register compatible" with the DS90UB960, so the items below should be re-checked against the DS90UB954 datasheet. — [linux-media patch](https://www.spinics.net/lists//linux-media/msg263488.html) [snippet]
- Register 0x58 BCC_CONFIG, bit 4 BC_ALWAYS_ON, reset value 1: "1: Back channel is always enabled independent of I2C_PASS_THROUGH and I2C_PASS_THROUGH_ALL. 0: Back channel enable requires setting of either I2C_PASS_THROUGH and I2C_PASS_THROUGH_ALL. This bit may only be written via a local I2C controller." Bit 6 I2C_PASS_THROUGH resets to 0. — [DS90UB960-Q1 datasheet, Table 7-98](https://www.ti.com/lit/ds/symlink/ds90ub960-q1.pdf) [datasheet]
- Bits 2:0 BC_FREQ_SELECT: 000 = 2.5 Mbps (DS90UB913A/933), 010 = 10 Mbps, 110 = 50 Mbps (DS90UB953); other codes reserved. There is no "off" code in this field. — same table [datasheet]
- A TI E2E thread titled "DS90UB960-Q1: Back Channel Disable" exists. The search summary reports TI advising to select the port in 0x4C and then clear 0x58, and it reports two different bits in different answers (0x58[7:6] and 0x58[4]). — [E2E thread 703711](https://e2e.ti.com/support/interface-group/interface/f/interface-forum/703711/ds90ub960-q1-back-channnel-disable) [snippet; direct fetch returned 403]
- An unused RX port can be disabled per port in RX_PORT_CTL (0x0C). — same datasheet, pin table [datasheet]
- The receive mode per port is FPD3_MODE, register 0x6D[1:0], or the MODE strap. The choices are CSI-2 mode (DS90UB953/935 compatible, 40-bit forward frame) and RAW10 / RAW12-LF / RAW12-HF modes (DS90UB933/913A compatible, 28-bit forward frame). MODE strap option 0 is CSI-2 non-synchronous back channel and option 4 is CSI-2 synchronous. — same datasheet, section 7.4 [datasheet]
- Input frequency is measured without the serializer: RX_FREQ_HIGH/LOW (0x4F/0x50) give a 16-bit frequency in MHz, updated in under 1 ms, with FREQ_STABLE and NO_FPD3_CLK flags in RX_PORT_STS2. — same datasheet, section 7.4.9.3 [datasheet]
- Lock and quality are local reads: LOCK_STS and PORT_PASS in RX_PORT_STS1 (0x4D), parity error counter 0x55/0x56, FPD3_ENC_ERROR in 0x4E[5], plus line count and line length per frame. "In CSI-2 input mode, any long packet will be counted as a video line." — same datasheet, section 7.4.9 [datasheet]
- By default parity errors do not block lock: DIS_LINK_PAR resets to 1, "Parity errors will NOT prevent assertion of forward channel lock detect." — same datasheet, register table [datasheet]
- The AEQ "is continually searching through the EQ settings and could be at any setting when the serializer supplies a signal"; the range can be narrowed with AEQ_MIN_MAX (0xD5) and adaption restarted with AEQ_RESTART (0xD2). — same datasheet, section 7.4.7.4 [datasheet]
- CMLOUT gives a buffered post-equalizer copy of the selected RX port. An eye of at least 0.35 UI "suggests that the forward channel signal integrity is likely acceptable." 1 UI = 1/(160 x REFCLK) in CSI-2 synchronous mode. — same datasheet, section 7.4.8 [datasheet]
- An enhanced CRC on the FPD3 encoder sequence is optional with a DS90UB953 (FPD3_ENC_CRC_DIS 0xBA[7], FPD3_ENC_CRC_CAP 0x4A[4]). — same datasheet, section 7.4.9.2 [datasheet]
- In RAW mode the deserializer builds the CSI-2 packets itself: "the CSI-2 packet header VC-ID and Data Type are programmable." In CSI-2 mode it forwards the CSI-2 packets the serializer received. — same datasheet, sections 7.4.1 and 7.4.2 [datasheet]
- FPD-Link and GMSL are not interoperable: TI says both companies use proprietary encoding and drivers. — [TI E2E thread 913451](https://e2e.ti.com/support/interface-group/interface/f/interface-forum/913451/ds90ub960-q1-how-to-connect-a-camera-using-maxim-gmsl-serializer-like-max96705-to-a-fpd-link-iii-deserializer-like-ds90ub960) [snippet]

**GMSL2 (MAX9296A)**
- Link-up is described as mutual: the device selects a PHY "by periodically attempting to handshake", then "each enabled PHY performs link calibration, equalizer adaptation, and data-channel locking. Both chips set their LOCK pin high." About 20 ms nominal, 100 ms maximum. — [MAX9296A datasheet, Power-Up and Link Startup](https://cdn.icstop.com/upload/pdfs/32/fd/32fdc6558965079574838d6317d194a7.pdf) [datasheet]
- "Link lock indicates that the PLLs for the GMSL2 link are locked to each other and the data receive paths are locked (forward channel in serializer, reverse channel in deserializer)." — same datasheet [datasheet; the parenthesis is quoted as printed]
- The link locks from the crystal reference, "so a valid video input (pixel clock) is not needed for the GMSL2 link to lock." — same datasheet [datasheet]
- Settings that must match before a link reset: TX_RATE, RX_RATE, CXTP_A/B (coax or twisted pair), AUTO_LINK, LINK_CFG, GMSL2 (versus GMSL1). — same datasheet [datasheet]
- Side-channel data has 16-bit CRC "with retransmission upon error detection"; video-line CRC is an optional 32-bit. — same datasheet [datasheet]
- I searched the extracted MAX9296A text for a reverse-channel disable bit (DIS_REM_CC) and for any receive-only wording and found neither. — same datasheet [datasheet, negative result]

**What sniffer vendors say**
- TZ: "An additional Deserializer (DES2) can now be connected to the forward output of the coupler. DES2 cannot communicate with the serializer. Therefore, DES2 must be specially configured for compatibility with the SER. In our TZ devices, we call this special configuration 'coupler mode'." — [TZ coupler flyer](https://tz-es.com/wp-content/uploads/2023/05/TZ_Product_Flyer_Coupler.pdf) [datasheet]
- "TZ offers the option of providing every setting for its Deserializer products as a coupler mode setting." For non-TZ devices, "special adjustments must be made to this device. Where possible, we provide support with instructions on exactly what to do." — same flyer [datasheet]
- The coupler list covers GMSL1, GMSL2, GMSL3, FPD-Link III and FPD-Link IV. — same flyer [datasheet]

### Inferences
- **FPD-Link III listen-only looks feasible on paper.** Nothing in the DS90UB960 text makes forward-channel lock depend on a back-channel reply: lock, frequency, parity and line counts are all receive-side. The forward rate is set by the original deserializer's REFCLK (160 x REFCLK), and the sniffer simply recovers that clock; its own REFCLK only needs to be a valid 23 to 26 MHz. I rate this as probable but unproven, since no source shows it done.
- **The power-up hazard is real and specific.** BC_ALWAYS_ON resets to 1, so a DS90UB954/960 drives a back channel on its RX pins from power-up until firmware writes 0x58. Through a coupler that signal travels toward the camera attenuated by the coupling factor, on top of the ECU's genuine back channel. Mitigations, in order of strength: a unilateral RF buffer or isolator on the tap arm; a coupler with good directivity plus an attenuator; writing 0x58 as the first I2C transaction with the RX port disabled in 0x0C until then. The first is the only one that also covers a firmware crash.
- **Whether clearing BC_ALWAYS_ON truly silences the driver** is not proven by the datasheet wording alone. Verify on a bench with a scope on the RX pin before connecting to any vehicle. The E2E thread's conflicting bit references are a reason for this check.
- **Configuration that must be known, and how to find it without I2C access to the camera:**
  1. Chip family (FPD-Link III or IV, or GMSL1/2/3): from chip markings on a salvage unit. Cross-family lock is impossible.
  2. FPD3 mode: try CSI-2 mode, then RAW10, RAW12-HF, RAW12-LF, and read LOCK_STS and PORT_PASS. There are only four choices.
  3. Rate: read RX_FREQ (0x4F/0x50). A value near 100 MHz frame clock at 4 Gbps points to a 953-class link; 2 Gbps-class values point to 933/913A or 935.
  4. Synchronous or not: this affects only the original link's back channel rate (50 or 10 Mbps). A silent sniffer does not need it.
  5. Resolution and format: line count and line length registers, then the CSI-2 data type in the packet headers on the sniffer's CSI-2 output (in CSI-2 mode the serializer's packets pass through, so the data type and virtual channel are visible to the host receiver).
  6. In RAW (DVP) mode the data type is not carried on the link; the sniffer chooses RAW8/10/12 itself, so bit depth must be found by inspecting the image.
- **AEQ on a tap.** The tap signal is weaker and flatter than a normal cable signal. Pinning AEQ_MIN_MAX to a narrow range found on the bench should give more repeatable lock than the full search.
- **Virtual-channel filters.** If the OEM hub uses VC remapping, that happens in the OEM deserializer, after the tap point. The sniffer sees the camera's own VC, usually 0.
- **GMSL2 listen-only is the weaker case.** The datasheet's handshake and "PLLs locked to each other" wording implies the standard state machine waits for the partner. TZ's claim that coupler mode exists for GMSL parts suggests there are register or test-mode settings that force the receive path, possibly known to ADI's customers under NDA. Treat a GMSL2 tap with a third-party deserializer as unproven until ADI or TZ supplies a register sequence.
- **GMSL2 parameters to know in advance:** forward rate (3 or 6 Gbps), coax or STP, GMSL1 or GMSL2, pixel or tunnel mode, and the stream/pipe to data-type mapping. Unlike TI's part, I found no documented register that measures an unknown input rate, so the two rates would be tried in turn.
- **A hub part is a safer sniffer than it sounds** only if every unused RX port is disabled and each used port has its back channel off. PoC must not be populated on the sniffer board at all.
- **The upstream Linux drivers are not listen-only.** The mainline `ds90ub960` driver and the Maxim GMSL2/3 series are written for a normal link where the host reaches the serializer over the control channel. I expect a sniffer needs a patched or user-space configuration. I did not read the driver source for this note.

### Gaps
- No TI or ADI statement, datasheet section, application note or forum answer that I could read says a second receive-only deserializer is supported or unsupported. The one relevant E2E thread returned 403.
- No register sequence for "coupler mode" is public. TZ's sample programs and register notes are available only on request, and only for "certain deserializer models" (earlier notes).
- The DS90UB954 and DS90UB9702 (FPD-Link IV) datasheets were not read for this note. FPD-Link IV adds higher rates and its own link training; whether it can be sniffed the same way is unknown.
- MAX96712 was not read. The MAX9296A result may not carry over to quad parts.
- The ADI EngineerZone post "When GMSL Won't Talk: Your Guide to Securing Link Lock" timed out: [EZ blog](https://ez.analog.com/ez-blogs/b/engineerzone-spotlight/posts/gmsl-debugging-getting-a-lock).
- Whether a DS90UB960 asserts PORT_PASS and outputs CSI-2 with BCC CRC and sequence errors accumulating (because it never sees replies to its own control traffic) is untested.

## Commercial listen-only tools: couplers, taps, loggers and what is passive

### Takeaway
TZ is still the only vendor I can confirm sells a passive RF coupler for these links, and its flyer now settles the model map: CPL0210, CPL0610 and CPL0710 are the PoC versions. Every other "tap" I could read (b-plus MDILink TAP, Xylon tap mode, Intrepid RAD-Phobos) is an active deserializer-plus-serializer box that sits in series with the OEM link. Nothing in this class has a published price, and nothing is hobby-sized except the coupler itself.

### Cited Findings

**TZ Electronic Systems (passive)**

| Part | Connector | Channels | Flyer description |
|---|---|---|---|
| CPL0210 | FAKRA | 1 | For signal lines with power over coax |
| CPL0215 | FAKRA | 1 | For signal lines without power over coax |
| CPL0315 | H-MTD | 1 | For signal lines without power over differential lines |
| CPL0610 | HFM | 1 | For signal lines with power over coax |
| CPL0615 | HFM | 1 | For signal lines without power over coax |
| CPL0710 | HFM | 4 | For signal lines with power over coax |
| CPL0715 | HFM | 4 | For signal lines without power over coax |

— [TZ coupler flyer, version 03.26](https://tz-es.com/wp-content/uploads/2023/05/TZ_Product_Flyer_Coupler.pdf) [datasheet]
- Legacy active models, all NRFND: CPL0220 (FAKRA, two decoupled outputs, PoC), CPL0225 (same, no PoC), CPL0250 (FAKRA "Active Bi-Directional Coupler"). — same flyer [datasheet]
- Use case 3 in the flyer is live monitoring of the link with an oscilloscope on the forward output. — same flyer [datasheet]
- Flyer wording on the difference from a splitter: with bidirectional signals "the implementation of splitters is often only possible with restrictions." — same flyer [datasheet]
- No price on any TZ page or in the flyer; every model is quote-only. — [TZ couplers](https://stz-es.com/coupler/) [page]

**b-plus MDILink QX035 GMSL2 TAP (active, in-line)**
- Two GMSL2 inputs on a MAX96716 deserializer and two GMSL2 outputs on a MAX96717 serializer, FPGA SoC between them, "full transparent tap mode", up to 4.28 Gbit/s per camera, two 10 GbE downstream ports, ASAM CMP output. — [b-plus datasheet V1.1, 2026-03-03](https://www.b-plus.com/fileadmin/data_storage/Data_Storage/1_Main_Navigation/1_Portfolio/04_Datenverteilung_und_Konvertierung/MDILink/Datenblaetter/Datasheet_MDILink_GMSL2_TAP-1.pdf) [datasheet]
- Power over coax: "external supply or PoC forwarding", 6 to 15 V, maximum 300 mA. Back channel: "I2C backchannel communication and logging", host to sensor and MDILink to sensor. — same datasheet [datasheet]
- 8 to 32 V supply, typically 15 W and up to 25 W, 159.7 x 140 x 38 mm, about 0.8 kg, -25 to +70 C operating. FAKRA or H-MTD versions. No price. — same datasheet [datasheet]
- A GMSL1/2 TAP variant datasheet also exists. — [b-plus](https://www.b-plus.com/fileadmin/data_storage/Data_Storage/1_Main_Navigation/1_Portfolio/04_Datenverteilung_und_Konvertierung/MDILink/Datenblaetter/Datasheet_MDILink_GMSL1-2_TAP.pdf) [snippet, not read]

**Xylon logiRECORDER / Quattro (active)**
- One video module, logiR-FPD3C-953-954-V2 (part 61-02008-20-00001, DS90UB953 + DS90UB954), is listed with "end-point and tap-modes enabled". The page does not define tap mode. — [Xylon video modules](https://xylon-lab.com/product-category/i-o-modules/video-modules/) [page]
- Other modules pair a serializer and a deserializer for each technology: MAX9295A/9296A, MAX96717/96716A, MAX96793/96792A (GMSL3), DS90UB971/9702 (FPD-Link IV), DS90UB933/934, MAX96705/96706, Sony GVIF2. No prices; part numbers only. — same page [page]
- logiRECORDER 3.5 accepts raw FPD-Link III/IV and GMSL1/2/3 inputs. Its "TAP" in the networking options refers to 10 GbE. — [Xylon logiRECORDER 3.5](https://xylon-lab.com/product/logirecorder-3-5/) [snippet]

**Intrepid RAD-Phobos (active media converter, not a tap)**
- Converts FPD-Link III on STP (TI DS90UB954 deserializer, two input ports) to GMSL2 on coax (MAX9295 serializer, one FAKRA output). DIP switches select 6 or 3 Gbps and CSI-2 non-synchronous, RAW10 or CSI-2 synchronous mode. External 4 to 40 V supply. The page does not describe pass-through or power-loss behaviour. — [Intrepid RAD-Phobos hardware tour](https://guide.intrepidcs.com/docs/RAD-Phobos/A-Tour-of-Hardware.html) [page]

**Solectrix proFRAME (grabber and playback)**
- Captures up to 8 camera streams, GMSL and FPD-Link adapters, with a camera output path between ECU and camera on some adapters (for example "camAD3 DUAL GMSL2 (2xIN, 2xOUT)"). A Base Board 3.1 PXIe was announced in February 2026. None of the results describe a passive tap. — [embedded world listing](https://www.embedded-world.de/en/exhibitors/solectrix-gmbh-2458458/proframe-–-video-grabber-und-playback-system-2416876) [snippet]; [camAD3 reseller page](https://hongtronics.com/en/?p=63238) [snippet]

**Others seen in search only**
- AED SLA-1C-GMSL "SerDes logging adapter", with GMSL2 (up to 6 Gb/s, STP) and FPD-Link III models. — [manual](https://manuals.online/aed/adapter/sla-1c-gmsl/user-manual/o1dy) [snippet; fetch returned 403]
- Technica Engineering ran a webinar with a live SerDes capture demo on a sensor-module link. No product detail retrieved. — [mobex webinar page](https://mobex.io/webinars/serdes-technologies-in-action-enabling-smarter-sensor-validation/) [snippet]
- FLIR sells an "FPD-Link III / GMSL2 to USB Module" (SerDes box) for its own cameras. — [FLIR OEM](https://www.oem.flir.com/products/serdes-box/) [snippet]
- The Imaging Source sells GMSL2 and FPD-Link III deserializer boards for embedded platforms. — [The Imaging Source](https://www.theimagingsource.com/en-us/embedded/deserializer/gmsl2-fpd-link-iii/) [snippet]
- From the earlier notes, still current: Digiteq FG4 claims automatic input-parameter detection; TechNexion UVC boxes are $189; TI DS90UB954-Q1EVM is $399 to $595.

### Inferences
- **"Tap" means two different things in this market.** TZ's coupler is an RF tap: the OEM link stays electrically end to end. The b-plus device, and by construction any module built from a deserializer and a serializer, terminates the camera link and regenerates a new one toward the ECU. That makes the OEM link depend on the tap's power, firmware and configuration, and it must forward the back channel and PoC. For a car that is driven, that is the active in-line insertion the earlier notes ranked last.
- **Xylon's "tap mode" is almost certainly the active kind**, because the module is a 953 + 954 pair. This is my reading of the part list, not a statement on the page.
- **TZ's own deserializer or grabber is the lowest-risk receiver** for its coupler, since it is the only product documented to have the needed mode. The cost is unknown.
- **Hobby suitability.** The b-plus box is 15 to 25 W and 0.8 kg with 10 GbE output: usable in a car, not hobby-priced by any signal I can see. A TZ single coupler is passive, about 64 x 103 x 40 mm, needs no power and is the only item here small enough to hide behind a dash. Whether TZ sells single units to individuals is unknown.
- **A do-it-yourself equivalent** would be a wideband 50 ohm directional coupler covering roughly 20 MHz to 3 GHz, with FAKRA or HFM adapters and, for a PoC line, a through path rated for the camera current. General RF couplers are normally not rated to pass DC bias at hundreds of mA, so a bias-tee pair or a purpose-built board would be needed. I did not verify any specific part.

### Gaps
- Prices: none found for TZ, b-plus, Xylon, Solectrix, Digiteq, Intrepid or AED. All are quote-only.
- Not researched or not found: Vector, GOEPEL Video Dragon, Technica Engineering CaptureModule details, dSPACE, Averna. My searches returned nothing usable on these; that is a limit of the search, not evidence that they have no product.
- Whether any vendor other than TZ sells a passive RF coupler for these links. I found none.
- The b-plus datasheet does not say what happens to the OEM link when the tap loses power.

## Encryption and content protection

### Takeaway
HDCP is a feature of display-oriented parts (TI "UH" family) and is absent from the camera-oriented parts I read; no source I found shows a production camera uplink that encrypts video. The protection that does exist on camera links is integrity checking (parity, CRC) and, optionally, camera authentication over the control channel, neither of which hides the video from a tap.

### Cited Findings
- The MAX9296A feature table lists "Auto HDCP: No" for both GMSL2 and GMSL1 modes. The device does have an HDCP_CPBL capability bit in register 0x0F, alongside SPLTR_CPBL and DUAL_CPBL. — [MAX9296A datasheet](https://cdn.icstop.com/upload/pdfs/32/fd/32fdc6558965079574838d6317d194a7.pdf) [datasheet]
- The MAX9296A offers "Video Watermark Insertion and Detection", an "Optional 32-Bit Video-Line CRC" and 16-bit CRC on side-channel data. These are integrity and frozen-frame checks. — same datasheet [datasheet]
- The DS90UB960 text I extracted contains parity, an optional encoder CRC and back-channel CRC, and no cipher. — [DS90UB960-Q1 datasheet](https://www.ti.com/lit/ds/symlink/ds90ub960-q1.pdf) [datasheet; I did not search the whole text for "HDCP"]
- A TI E2E reply on protecting DS90UB953A data from falsification is summarized as: UB variants lack HDCP, UH variants include it, and HDCP is content protection for media and not a sensor-integrity mechanism. — [E2E thread 1126718](https://e2e.ti.com/support/interface-group/interface/f/interface-forum/1126718/ds90ub953a-q1-protect-data-falsification) [snippet; direct fetch returned 403]
- ADI describes authenticating a camera module by adding a DS28C40 secure authenticator whose I2C traffic is tunnelled over the GMSL link; the ECU verifies the camera by public-key signature. The stated aim is to block counterfeit or swapped cameras. — [ADI design note](https://www.analog.com/en/resources/design-notes/authenticating-remote-automotive-peripherals-using-gmsl-tunneling.html) [snippet; fetch timed out]
- GMSL video "may be unprotected or encrypted using HDCP." — [Wikipedia, GMSL](https://en.wikipedia.org/wiki/Gigabit_Multimedia_Serial_Link) [snippet]
- A TU Munich paper states that traditional automotive links, including the LVDS links used for ADAS cameras, lack integrated authentication or encryption. — [TUM document](https://mediatum.ub.tum.de/doc/1471857/document.pdf) [snippet]
- From the earlier notes: TI DS90UH941AS-Q1 has HDCP 1.4 with on-chip keys; the UB sibling does not list it.

### Inferences
- A camera with a "UB" serializer (DS90UB953, 933, 913A, 935, 971) or a MAX9295/MAX96717-class part sends video in the clear at the link layer. A passive receiver that locks will get the pixels.
- Authentication of the DS28C40 type runs over I2C on the control channel. A listen-only tap neither takes part in it nor is blocked by it, because the video is not encrypted by it.
- Watermark, CRC and frame counters matter to someone injecting video, not to someone listening.
- The ECU-to-display link is where HDCP-capable parts are more likely. Check the part marking; treat a "UH" link as unavailable. Bypassing content protection is out of scope.
- Newer camera standards (MIPI A-PHY, ASA Motion Link) define optional security layers. I did not research them, and I do not know of a production car that enables them on cameras.

### Gaps
- No primary TI text read on UB versus UH (both E2E fetches returned 403). The claim rests on a search summary plus product listings from the earlier notes.
- Whether any OEM enables MACsec-style or proprietary encryption on a camera SerDes link: no source found either way.
- The ADI design note and TUM paper were not read in full.

## What comes out: CSI-2, RAW Bayer from an AR0233AT, ISP steps, and bridging to a host

### Takeaway
A successful tap yields whatever the sensor sent, which for an ADAS or surround camera is very likely RAW Bayer in a companded HDR format meant for the ECU's ISP, so a full software ISP is part of the job. I could confirm the AR0233AT's headline specifications but not its exact output-format table, and I could not confirm prices for CSI-2-to-USB bridges.

### Cited Findings

**AR0233AT (onsemi product overview)**
- 2.6 MP, 1/2.5 inch, 3.0 um dual-conversion-gain BSI pixel, active array 2048 x 1280, 1080p output at 60 fps. — [onsemi AR0233AT overview PDF](https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/7169/AR0233AT-3688810.pdf) [datasheet]
- "MultiExposure Mode for >140 dB High Dynamic Range", "Full Resolution LED Flicker Mitigation with 120 dB High Dynamic Range", ">95 dB dynamic range from one exposure". — same overview [datasheet]
- 4-lane MIPI CSI-2 interface; colour ("RGB") parts AR0233ATSC17XUEA1; electronic rolling shutter; ASIL-B design. — same overview [datasheet]
- Works "with image signal processor (ISP) companion chips from onsemi as well as third-party ISPs", which means the sensor itself outputs unprocessed data. — same overview [datasheet]
- The sibling AR0231AT lists output formats of 12-bit linear, 20-bit uncompressed HDR, 10-bit companded linear, and 16/14/12-bit companded HDR. — [AR0231AT datasheet copy](https://datasheet.chipsmall.com/AR0231AT7C00XUEA0-DRBR/107759.pdf) [snippet; sibling part, not the AR0233AT]
- The related AR0143AT datasheet notes that its frame-rate figures assume off-chip HDR linearization. — [AR0143AT datasheet](https://mouser.com/datasheet/2/308/1/onsemi_5_27_2024_AR0143AT_D-3453378.pdf) [snippet]

**Deserializer to CSI-2**
- The MAX9296A outputs CSI-2 at 80 to 2000 Mbps per lane on one or two ports. — [MAX9296A datasheet](https://cdn.icstop.com/upload/pdfs/32/fd/32fdc6558965079574838d6317d194a7.pdf) [datasheet]
- The DS90UB960 forwards any RX port to either of two CSI-2 TX ports, with virtual-channel remapping; in RAW mode it generates RAW8/10/12 CSI-2 with a programmable data type. — [DS90UB960-Q1 datasheet](https://www.ti.com/lit/ds/symlink/ds90ub960-q1.pdf) [datasheet]
- Mainline Linux has a `ds90ub960` driver, and a patch series adding DS90UB954 support was posted in December 2025. — [linux-media patch](https://www.spinics.net/lists//linux-media/msg263488.html) [snippet]; [driver source](https://github.com/torvalds/linux/blob/master/drivers/media/i2c/ds90ub960.c) [snippet]
- A Maxim GMSL2/3 serializer/deserializer driver framework (including MAX9296A) was at patch version 16 on the kernel lists in August 2026, so it was still under review then. — [LKML v16](https://ratatoskr.run/lkml/2026/08/17481504/t) [snippet]

**Open-source ISP**
- libcamera has an upstream software ISP running on the CPU, covering debayering, auto white balance, auto gain and gamma, with a generic "simple" pipeline handler for raw Bayer cameras. — [FOSDEM 2026 libcamera softISP slides](https://fosdem.org/2026/events/attachments/TKSK3G-libcamera-softisp/slides/263009/libcamera_r0fvfbu.pdf) [snippet]
- Infinite-ISP (10x Engineers) is a permissively licensed suite with a Python algorithm platform, a bit-accurate fixed-point reference model, RTL and a tuning tool; its reference designs target 10-bit 2592 x 1536 Bayer sensors at 30 fps. — [Electronic Imaging 2025 paper](https://library.imaging.org/ei/articles/37/7/ISS-279) [snippet]
- openISP is an MIT-licensed Python pipeline that processes a raw file step by step from a config, and lists W/HDR among advanced functions. — [openISP mirror](https://gitee.com/geniusxxx/openISP) [snippet]
- Raspberry Pi's PiSP back end has stitching and tonemapping blocks "used for HDR", driven by the open libpisp library. It is a hardware ISP on the Pi 5. — [kernel docs](https://docs.kernel.org/admin-guide/media/raspberrypi-pisp-be.html) [snippet]

**CSI-2 to USB bridges**
- Infineon CX3 CYUSB3065: 4 CSI-2 lanes at 1 Gbps per lane to USB 5 Gbps. — [Infineon CX3](https://infineon.com/cms/en/product/universal-serial-bus/usb-peripheral-controllers-for-superspeed/ez-usb-cx3-mipi-csi2-to-usb-5gbps-camera-controller/cyusb3065-bzxi) [snippet]
- The plain FX3 cannot take MIPI directly; Infineon says an FPGA is needed between the sensor and FX3. — [Infineon community](https://community.infineon.com/t5/USB-superspeed-peripherals/MIPI-CSI-2-with-FX3/m-p/327345) [snippet]
- Lattice publishes a CrossLinkU-NX MIPI CSI-2 to USB UVC reference design for the LIFCL-33U-EVN board. — [Lattice](https://www.latticesemi.com/en/Products/DesignSoftwareAndIP/IntellectualProperty/ReferenceDesigns/ReferenceDesigns06/MIPI-CSI-2-to-USB-UVC-Reference-Design) [snippet]
- Infineon's EZ-USB FX20 MIPI demo kit is listed at 722.58 excluding GST on element14 Australia (so presumably AUD), with 4-lane CSI-2 receive on Raspberry Pi camera connectors. — [element14 AU](https://au.element14.com/infineon/demofx20mipi001tobo1/demo-kit-ez-usb-fx20-usb-peripheral/dp/4765074) [snippet]
- Arducam's MIPI adapter for its USB3 camera shield was $49.99 and is marked discontinued. — [Arducam](https://www.arducam.com/product/mipi-adapter-board-usb3-camera-shield/) [snippet]

### Inferences
- **Expected stream from an AR0233AT-class camera on a 953-class link:** RAW12 Bayer on CSI-2 (data type 0x2C), 1920 x 1080 or 2048 x 1280, 30 fps for ADAS, with HDR data companded from about 20 bits to 12. This is my expectation from the sibling's format list and normal automotive practice. The actual data type and size come from the deserializer's line registers and the CSI-2 headers.
- **Data rate check (my arithmetic):** 2048 x 1280 x 12 bit x 30 fps is about 0.94 Gbps of pixels, and 1.89 Gbps at 60 fps. Both fit a 4 Gbps FPD-Link III or a 3 Gbps GMSL2 link, and a 2-lane CSI-2 output at 1 Gbps per lane only at 30 fps.
- **ISP steps needed, in order:** black-level subtraction; decompanding with the sensor's piecewise-linear knee table back to linear 20-bit or more; defective-pixel correction; lens-shading correction; white balance; demosaic; colour-correction matrix; local or global tone mapping down to 8 bits; gamma; then fisheye undistortion for a surround camera.
- **Values the tap cannot read.** The knee points, the black level, the Bayer order and the per-frame exposure and gain are set over I2C on the back channel, which a silent tap does not see. Options: the sensor's embedded-data rows, if the OEM enabled them (they arrive as a separate CSI-2 data type, 0x12), carry exposure and gain per frame; knee points can be estimated from histogram discontinuities; a directional coupler turned the other way plus an I2C-over-back-channel decoder would be a second research project. Default knee tables from the sensor's developer guide are the practical starting point, and that guide is under onsemi NDA.
- **A fixed tone curve will look poor across day and night** without the exposure metadata, because the OEM's auto-exposure runs in the ECU and the tap sees only its results as pixel values. Auto white balance and tone mapping must be re-derived from the image.
- **Open ISP fit.** libcamera's software ISP does not, as far as the sources show, do HDR decompanding or lens-shading for this sensor, so a custom stage is needed first. A short custom pipeline (lookup-table decompand, OpenCV demosaic, CLAHE or Reinhard tone map, `cv2.fisheye` remap) on a GPU-capable board is realistic for one camera at 30 fps. Infinite-ISP and openISP are useful as references for block order and tuning, not as real-time code.
- **Host choice.** A board with a native CSI-2 receiver that accepts RAW12 and lets user space read raw frames is simpler than a USB bridge: Jetson-class and RK3588-class boards both have 4-lane receivers, and the CSI-2 from a DS90UB954 EVM or a MAX9296A board can be wired to them with a device-tree node that describes a "dumb" fixed-format source. A UVC bridge would have to carry RAW12 in a nonstandard format or do the ISP in an FPGA. I did not verify a specific board or its RAW12 support for this note.
- **Geometry.** Fisheye undistortion needs intrinsics. Without OEM calibration data, calibrate with a checkerboard once through the tap.

### Gaps
- The AR0233AT full datasheet (output formats, companding modes, embedded data layout, default knee points) was not obtained. The public PDF is a two-page overview. The full document and developer guide are behind onsemi's NDA portal as far as I can tell.
- Which sensor is actually in the Fisker Ocean cameras is taken from the task brief, not verified here.
- Jetson and RK3588 CSI capture were not researched in this session: no prices, no confirmation of RAW12 capture paths, no confirmation that an existing driver can run without a serializer on I2C.
- No price found for a CX3 development kit or the Lattice LIFCL-33U-EVN board.
- Whether libcamera or any open ISP has a ready decompanding (PWL) block was not confirmed.

## Documented precedents: who has tapped an OEM camera link in a production car

### Takeaway
I found no public write-up, talk, repository or paper in which someone passively tapped an OEM camera SerDes link in a production car and recovered video. The nearest public traces are a November 2026 hobbyist question on Hyundai/Kia surround cameras that went unanswered, and old Tesla forum speculation; the practice clearly exists inside OEM validation (TZ, b-plus, Xylon sell for it) but is not documented in the open.

### Cited Findings
- **Hyundai/Kia E-GMP (Ioniq 5), November 2026.** A GitHub issue asks owners to help identify the SerDes family on the four surround-view camera coax links into the parking ECU (99910-GI400), with the goal of capturing them directly, before choosing a capture board. It cites KramerTech's "gen5-RearDashcam" notes that the head unit receives the ECU's video through a MAX96706 deserializer (GMSL1), and points out that this identifies the ECU-to-head-unit link, "not the camera to ECU links". — [xwtk/hmg_nemesis issue 85](https://github.com/xwtk/hmg_nemesis/issues/85) [page]
- The repository owner replied with architecture notes only (the cameras connect to the ADAS unit, which sends one video line to the head unit and reportedly cuts the feed above a speed threshold; "Camera feed is processed via Cortex-M") and closed the issue. Nobody posted chip markings, device-tree data or a capture. — same issue [page]
- **Tesla.** A 2025 Electrek article lists the HW3 camera interface as "FPD-Link III (Likely)" and AI4 as GMSL2 or MIPI A-PHY; this is the article's inference from a table. An older Tesla Motors Club thread notes that HW2.5 has deserializers connected to the cameras and speculates about LVDS, GMSL or Ethernet. Neither describes a tap. — [Electrek](https://electrek.co/2025/12/02/tesla-new-camera-upgrade-casting-more-doubt-full-self-driving-promises/) [snippet]; [TMC thread](https://teslamotorsclub.com/tmc/threads/tesla-autopilot-hw3.139550/page-4) [snippet]
- Tesla hobby repositories found are board photographs and log tools, not camera-link work. — [djcurr/Tesla-MCU-Reverse-Engineering](https://github.com/djcurr/Tesla-MCU-Reverse-Engineering) [snippet]; [NFI teslalogs](https://github.com/NetherlandsForensicInstitute/teslalogs) [snippet]
- **Hyundai head-unit community.** Gen5 and Gen5W firmware projects exist that give root on the head unit, which is the software route to the already-processed camera view. — [rgerganov/gen5fw](https://github.com/rgerganov/gen5fw) [snippet]; [HKG_Gen5W_ReverseEngineer](https://github.com/cantcs/HKG_Gen5W_ReverseEngineer) [snippet]
- **Industry practice.** TZ describes its couplers as intended for "test drives, test series, and hardware-in-the-loop endurance tests" and for tapping "from a vehicle camera in order to forward the video signal to an additional recording system". — [TZ coupler flyer](https://tz-es.com/wp-content/uploads/2023/05/TZ_Product_Flyer_Coupler.pdf) [datasheet]
- **Fisker Ocean.** TechInsights lists a TI FPD-Link III deserializer hub and an FPD-Link III serializer with CSI-2 in the Magna ADAS controller GP009379212. — earlier notes, [TechInsights](https://www.techinsights.com/blog/deep-dive-teardown-magna-electronics-adas-gp009379212-automotive)

### Inferences
- The absence of public precedents is informative: the hobby community reaches camera video through the head unit's software (root, shared memory, the already-rendered view) or by fitting its own cameras, as comma.ai does, and not through the RF link. Anyone attempting the RF tap should expect to be first to publish and to get no community support.
- The Hyundai thread shows the usual first obstacle: identifying the SerDes family on the camera side requires opening a camera or the ECU, because the head-unit side reveals only the ECU-to-display link.
- For the Ocean, the TechInsights listing (TI FPD-Link III hub plus a CSI-2 serializer) is the favourable case: it is the family for which the datasheet evidence for listen-only operation is strongest, and the "serializer with CSI-2" in the ADAS controller matches the `DIG_OUT_LVDS` output to the ICC seen on the wiring sheets. Which TI part numbers are fitted is still unconfirmed.
- Expected failure modes, by likelihood (my judgement): no lock because the tap signal is too small or the mode is wrong; lock with garbage because of a wrong data type or bit depth; unusable colour and brightness because the companding curve is unknown; and, on the OEM side, intermittent camera faults from added loss or from a sniffer that was allowed to transmit.

### Gaps
- No academic paper on passive eavesdropping of GMSL or FPD-Link was found. My searches were few and general, so this is weak evidence of absence. Automotive security venues (escar, USENIX Security, CCS, Black Hat/DEF CON car hacking village) and camera-injection research groups would be the places to look next.
- No comma.ai or openpilot community attempt at tapping an OEM camera link was found.
- No Toyota, VW or GM precedent was found.
- KramerTech's gen5-RearDashcam notes could not be located by search; the claim about MAX96706 is second-hand from the issue.
- No published account of faults caused by a tap on a live ADAS link.
