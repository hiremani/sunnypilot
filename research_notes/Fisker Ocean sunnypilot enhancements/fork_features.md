# Fork feature inventory for a Fisker Ocean sunnypilot port (upstream openpilot, sunnypilot, FrogPilot, StarPilot, dragonpilot) — state as of 2026-11-13

Reader notes (apply to every section):

- "Local" sources are files in the checkout at `/Users/hiremanirmss/Fisker/sunnypilot` (read-only inspection; version header says openpilot 0.11.2 / sunnypilot 2026.003.000). Paths below are relative to that root.
- Web pages were read through a summarising fetch tool, so wording is paraphrased, not verbatim. Fork READMEs describe features but mostly do NOT state per-feature requirements (openpilot longitudinal, torque vs angle). Where I assign a requirement without an explicit source statement, it is in "Inferences", not "Cited Findings".
- I could not read fork source trees (no code paths confirmed for FrogPilot, StarPilot or dragonpilot). DeepWiki pages for FrogPilot and StarPilot returned HTTP 429. Code locations for those forks are a gap.
- Baseline already in the checkout (so not "missing"): MADS, ICBM framework, DEC, SCC-V/SCC-M, SLA + resolver, mapd, NNLC, auto lane change with BSM delay, road-edge lane change block (`relc.py`), Lane Turn Desire, blinker pause lateral, camera offset, live delay toggle, green-light and lead-depart alerts, torque bar, standstill timer, model manager, sunnylink, lateral + longitudinal maneuver tools, the custom stop-sign assist and web HUD. Evidence: directory listing of `openpilot/sunnypilot/selfdrive/controls/lib/` (`dec/`, `smart_cruise_control/`, `speed_limit/`, `stop_sign/`, `nnlc/`, `auto_lane_change.py`, `lane_turn_desire.py`, `relc.py`, `blinker_pause_lateral.py`), `openpilot/sunnypilot/{mads,mapd,navd,sunnylink,webhud,livedelay,models}`, `openpilot/tools/{lateral_maneuvers,longitudinal_maneuvers,replay,cabana,jotpluggler,plotjuggler,clip}`, and param keys in `openpilot/common/params_keys.h` (`AutoLaneChangeBsmDelay`, `AutoLaneChangeTimer`, `RoadEdgeLaneChangeEnabled`, `CameraOffset`, `GreenLightAlert`, `LeadDepartAlert`, `LagdToggle`, `StandstillTimer`, `TorqueBar`, `StopSignAssist`, `StopSignAssistShadow`).

## Which features in FrogPilot/StarPilot, dragonpilot, and upstream openpilot (2025-2026) are absent from sunnypilot or from this checkout, and which work WITHOUT openpilot longitudinal?

### Takeaway
The checkout already carries essentially the whole current sunnypilot feature set; what it lacks relative to other forks is mostly (a) FrogPilot/StarPilot quality-of-life and alerting items (louder blind-spot alert, blind-spot path, speed-limit-changed chime, Speed Limit Filler, weather offsets, standby mode, screen recorder, backups, phone portal with diagnostics), (b) StarPilot's Lane Centering, navigation, Sentry mode and V-ASM, and (c) dragonpilot's Position Offset and LCA Speed Changer. The most relevant upstream-sunnypilot piece that is present as a framework but not wired for Fisker is ICBM (button-spoofing speed control for stock-ACC cars).

### Cited Findings

**Upstream openpilot 2025-2026 (what the port inherits or could pick up)**
- 0.9.9 (2025-05-23) and 0.10.0 (2025-08-05) introduced online-learned steering actuation delay; 0.10.0 also replaced the longitudinal MPC with end-to-end world-model planning in Experimental mode and added opt-in dashcam audio — [commaai RELEASES.md](https://raw.githubusercontent.com/commaai/openpilot/master/RELEASES.md)
- 0.10.2 (2025-11-19) added comma four support; 0.10.3 (2025-12-17) a new temporal policy architecture and new DM model — local `RELEASES.md`; [commaai RELEASES.md](https://raw.githubusercontent.com/commaai/openpilot/master/RELEASES.md)
- 0.11.0 (2026-03-17): driving model fully trained in a learned simulator, "improved longitudinal performance in Experimental mode" — local `RELEASES.md`
- 0.11.1 (2026-05-18 in RELEASES.md; blog dated June 5, 2026): new DM model with better phone detection, driver camera uses same tone mapping/downscale as road cameras, head-pose alerts relaxed during turns (~7% fewer alerts), restructured `driverMonitoringState`, and a new Lateral Maneuver report tool — [comma blog 0.11.1](https://blog.comma.ai/0111release/)
- 0.11.2: 880M-parameter "big" driving model, big models on an external GPU, live camera streaming / dashcam clip generation / remote body control from comma connect, new alert sounds, VW ID.4 and CUPRA Born support. The local checkout's `RELEASES.md` dates this 2026-08-12 — local `RELEASES.md`; a fetch of upstream master returned 2026-10-14 for the same entry — [commaai RELEASES.md](https://raw.githubusercontent.com/commaai/openpilot/master/RELEASES.md). Dates conflict; content agrees.
- A fetch of upstream master RELEASES.md reported an unreleased 0.11.3 entry (date TBD) in which "Autospeed" becomes always-on and out of beta on supported cars: openpilot chooses speed from the scene (stops at red lights, slows for curves, matches traffic) and the cruise set speed becomes a maximum — [commaai RELEASES.md](https://raw.githubusercontent.com/commaai/openpilot/master/RELEASES.md). Single paraphrased source; this entry is not in the local checkout's RELEASES.md. Treat as unverified until read directly.

**sunnypilot (features present upstream; status in this checkout)**
- sunnypilot's documented feature set: sunnylink, SLA, DEC, SCC-Map, SCC-Vision, ICBM, Auto Lane Change, Models & AI, plus settings pages for MADS, Torque, Models, Speed Limit Source, Display, Developer — [sunnypilot docs index](https://community.sunnypilot.ai/docs)
- Latest sunnypilot release found on master's CHANGELOG is v2026.002.000 (2026-06-28), synced to openpilot 0.11.1; no lane-change, navigation or tools items in it — [sunnypilot CHANGELOG](https://raw.githubusercontent.com/sunnypilot/sunnypilot/master/CHANGELOG.md). The local checkout reports `SUNNYPILOT_VERSION "2026.003.000"` and `COMMA_VERSION "0.11.2"` — local `openpilot/sunnypilot/common/version.h`, `openpilot/common/version.h`.
- ICBM sends cruise button presses over CAN so the stock ACC still controls speed; it unlocks SLA, SCC-V, SCC-M and Custom ACC Increments on cars without openpilot longitudinal; supported brands are Hyundai/Kia/Genesis (most CAN), Honda/Acura Bosch non-CAN-FD, Chrysler/Dodge/Jeep/RAM, Mazda; mutually exclusive with Alpha Longitudinal; stock FCA/AEB remain active — [sunnypilot ICBM doc](https://community.sunnypilot.ai/t/intelligent-cruise-button-management/244)
- In the checkout, ICBM brand implementations exist only for mazda, hyundai, chrysler, honda (`opendbc_repo/opendbc/sunnypilot/car/{mazda,hyundai,chrysler,honda}/icbm.py`); no `icbm.py` under a fisker directory. The controller is at `openpilot/sunnypilot/selfdrive/car/intelligent_cruise_button_management/controller.py`; base interface at `opendbc_repo/opendbc/sunnypilot/car/intelligent_cruise_button_management_interface_base.py`. (My grep covered `opendbc/car/fisker/*.py` and `opendbc/sunnypilot/car/`; the Fisker port may implement stock-ACC speed nudging by another route, e.g. `openpilot/sunnypilot/selfdrive/car/cruise_ext.py` / `can_overrides.py`, which I did not read.)
- Fisker interface in the checkout: `enableBsm = True`, `steerControlType = angle`, `alphaLongitudinalAvailable = True`, lateral-only by default, radar gated on a fingerprint header — local `opendbc_repo/opendbc/car/fisker/interface.py` lines 24-61.
- sunnypilot 2026.001.000 (2026-05-06) added: Raylib UI rewrite, comma four support, "DM: wheel touch enforcement in MADS", "sunnylink: SDUI" (server-driven settings UI), Developer Metrics UI, Tesla "Coop Steering" (2025.002.000), "Pause Lateral Control with Blinker: Post-Blinker Delay", tools for video concatenation and memory profiling — local `CHANGELOG.md`.

**FrogPilot (README last-updated badge: October 31, 2026)** — all from [FrogPilot README](https://raw.githubusercontent.com/FrogAi/FrogPilot/FrogPilot/README.md)
- Items with no visible equivalent in the checkout's param list:
  - Louder Blind Spot Alert (louder chime replacing the existing blind-spot warning; needs Lane Changes on and BSM).
  - Blind Spot Path (adjacent lane drawn red when a vehicle is in the blind spot).
  - Speed Limit Changed Alert (chime when the limit reading changes).
  - Speed Limit Filler (on by default): learns limits from camera-read signs per road and direction and uses them until the map matches; skips conditional limits; optionally shared.
  - Speed-limit extras: Confirm New Speed Limits, higher/lower limit lookahead time (downloaded-map limits only), Fallback Speed policy, Match Speed Limit on Engage, Override Speed, Mapbox online fallback with user key, source priority.
  - Weather Condition Offsets (extra following distance, gentler accel and cornering for low visibility/rain/rainstorm/snow; uses location; optional OpenWeatherMap key).
  - Pause Steering Below a chosen speed (always, or only with signal on).
  - Distance-button and LKAS-button remapping (tap/hold/long-hold actions such as toggling Experimental Mode, Traffic Mode, pausing steering).
  - Standby Mode (screen off while driving, wakes for alerts), Screen Recorder, Capture Recent Footage (last minute of the driving screen).
  - Model Randomizer with post-drive rating and blacklist.
  - "The Pond" browser companion on the same Wi-Fi (settings, Theme Maker, issue reporting); FrogPilot Backups and Settings Backups; Use Konik Server; High-Quality Recording when uploads are off; Disable Uploads / Disable Onroad Only.
  - Tuning Level (Minimal/Standard/Advanced/Developer) gating which settings are visible.
- Items FrogPilot has that the checkout already has an equivalent of: Always On Lateral (MADS), automatic lane changes, NNFF (NNLC), Green Light Alert, Lead Departing Alert, model manager, OSM map download, speed-limit display.
- FrogPilot release cadence: `FrogPilot` branch updated the Saturday after comma's official release; `FrogPilot-Staging` and `FrogPilot-Testing` branches exist. Supports comma 3/3X and references a comma four setup guide.
- The newest FrogPilot GitHub release notes a search surfaced were November 1, 2025 (stat tracking); no 2026-dated release notes were found, although the README badge says October 31, 2026 — [FrogPilot releases](https://github.com/FrogAi/FrogPilot/releases)

**StarPilot** — from [StarPilot README](https://github.com/firestar5683/StarPilot) and [StarPilot wiki](https://wiki.firestar.link)
- StarPilot exists and is active: wiki footer shows November 5, 2026; installer value `starpilothq/StarPilot`; supports comma 3, 3X and 4; 2,438 commits.
- Feature list: model switcher; custom torque controller; custom longitudinal planner for radar and vision-only cars; "Firestar Lateral Method" auto-tuning of lateral behaviour per car; navigation that shows routes and hints the driving model about exits and turns; Always On Lateral; Lane Centering (centres between detected lane lines, falls back to model end-to-end lateral with a "safety breakout"); Force Stop 2.0; Speed Limit Controller (dashboard, map, vision); Curve Speed Controller that learns from the driver; Conditional Experimental Mode and Conditional Chill Mode; Driving Profiles and Traffic Mode; themes; alert volume control; Bluetooth keyboards/controllers/steering-wheel remotes; Sentry mode (records motion while parked); V-ASM (uses the driver-facing camera to detect vehicles in adjacent lanes / blind spots); Chestnut/eGPU support.
- Galaxy: phone portal usable "from anywhere" to download models, change settings, update software, run diagnostics and manage dashcam video.
- Developer features: native/cross compilation on Windows/Mac/Ubuntu, Raylib UI runnable on PC, "Use Precompiled Binaries" toggle, custom long-maneuver tests for regen-only vehicles.
- Lineage: an earlier search result quoted the README as saying StarPilot is built off FrogPilot and began GM-focused — [search result for StarPilot README](https://github.com/firestar5683/StarPilot); the README as fetched on 2026-11-13 did not mention FrogPilot and credits BluePilot (Ford) and sunnypilot (HKG angle steering). Conflict noted.
- Licensing flag: README notices include "a custom license from Haibin Wen and SUNNYPILOT LLC requires permission for use" for the sunnypilot-derived HKG angle-steering code.

**dragonpilot** — from [dragonpilot CHANGELOGS.md (pre-build branch)](https://raw.githubusercontent.com/dragonpilot-community/dragonpilot/pre-build/CHANGELOGS.md) unless noted
- Latest entries: 0.11.1 r1 (2026-06-22) and 0.11.0 r1 (2026-06-09). No entry based on openpilot 0.11.2 was found.
- Lateral: ALKA (always-on lane keeping; panda safety check and seatbelt-status check added in 0.10.3 r1); Road Edge Detection (since 0.9.8 r1); Auto Lane Change (since 0.9.8 r5); LCA Speed Changer (since 0.9.8 r1); Position Offset (added 0.10.3 r2, 2026-02-24).
- Longitudinal: ACM (Adaptive Coasting Mode), AEM (Adaptive Experimental Mode, marked experimental), APM (Adaptive Personality Mode, added 0.10.3 r2); DTSC (Dynamic Turn Speed Control) and DASR (Dynamic Accel Slew Rate) were removed in 0.11.0; NoGG removed in 0.10.0 r2.
- UI/tools: Display Lead & Radar Stats (lead speed), Display Radar Tracks, Torque Bar, Border Indicators, Speed Based HUD, Display Mode, Audible Alert Mode, Error Logs Button, Model Selector, "DEV/Dashy" (v3 2026-05-20) and "opview" developer tools, External Radar Support with improved filters, Delay Starting Loggerd.
- ACM lets the car coast when speed is above the cruise setting with nothing ahead — [dragonpilot Substack: ACM & NoGG](https://dragonpilot.substack.com/p/acm-nogg)
- An older README mirror describes "ALC RoadEdge Detection" as blocking lane changes when the car is too close to the road edge — [dingliangxue/dragonpilot README mirror](https://github.com/dingliangxue/dragonpilot/blob/docs/README.md) (mirror; may be outdated)
- dragonpilot's stated philosophy is "minimal changes" from upstream; hardware comma 3/3X and aftermarket O3 series — [dragonpilot repo](https://github.com/dragonpilot-community/dragonpilot)

### Inferences
- Usable today on a stock-ACC, angle-steering car (they touch only UI, alerts, lateral gating, or data sources): Louder Blind Spot Alert, Blind Spot Path, Speed Limit Changed Alert, Speed Limit Filler (as an extra SLA source), SLA policy extras (confirm, lookahead, match-on-engage, override), Pause Steering Below, button remapping, Standby Mode / screen recorder / recent-footage capture, backups, a Pond/Galaxy-style diagnostics portal (the port's web HUD is the natural host), Tuning Level gating, dragonpilot Position Offset and lead/radar stats overlay, StarPilot Sentry mode and Bluetooth remotes. None of the sources states a longitudinal requirement for these; the classification is mine from their descriptions.
- Speed-shaping features (SLA, SCC-V/M, FrogPilot SLC/CSC) can only act on a stock-ACC car through set-speed manipulation. sunnypilot's sanctioned mechanism is ICBM; wiring a Fisker `icbm.py` (if the car accepts spoofed set-speed/button messages) would be the cleanest way to align with upstream sunnypilot rather than a bespoke path.
- Upstream "Autospeed" (if the 0.11.3 entry is accurate) signals that comma is moving traffic-light, curve and traffic-matching behaviour into the default model-driven longitudinal path. That would make fork-side CEM/force-stop/curve controllers progressively redundant on cars with openpilot longitudinal, and raises the value of finishing Fisker openpilot longitudinal.
- V-ASM (driver camera for adjacent-vehicle detection) is redundant on the Ocean, which already exposes BSM data.

### Gaps
- No code paths confirmed for FrogPilot, StarPilot or dragonpilot features (DeepWiki 429; repos not browsed file-by-file).
- Whether the Fisker port's current stock-ACC speed control uses ICBM or a custom path was not determined (I did not read `cruise_ext.py`, `can_overrides.py` or `carcontroller.py`).
- sunnypilot features merged after 2026-06-28 (the 2026.003 cycle) are not documented in any changelog I could reach; the local CHANGELOG also stops at 2026.002.000.
- The 0.11.2 release date (2026-08-12 vs 2026-10-14; search summaries also gave 06-15 and 10-23) and the 0.11.3 "Autospeed" entry need direct verification against upstream.
- FrogPilot 2026 release notes not found; README badge is the only late-2026 evidence of activity.

## Which features only become available with openpilot longitudinal?

### Takeaway
Everything in the forks' "Longitudinal" groups — conditional experimental mode, force stop at lights/signs, acceleration/deceleration profiles, driving personalities and traffic mode, human-like accel/following, increased stopped distance, coasting modes, and upstream Experimental mode/Autospeed — plans gas and brake and therefore presupposes openpilot longitudinal; sunnypilot is the only project that documents a stock-ACC bridge (ICBM) and it covers only set-speed features (SLA, SCC-V, SCC-M, custom increments).

### Cited Findings
- FrogPilot groups these under "Longitudinal (Gas and Brake)": Acceleration Profile (Eco/Standard/Sport/Sport+), Deceleration Profile (Eco/Eco+ coasting/Standard), Human-Like Acceleration, Human-Like Following, Advanced Longitudinal Tuning, Driving Personalities with per-profile following distance 1.00-3.00 s plus accel/brake smoothness, safety gap bias, slowdown/speed-up response (25-200%), Traffic Mode (hold distance button 2.5 s), Conditional Experimental Mode, Curve Speed Controller, Force Stop at "Detected" Stop Lights/Signs, Increase Stopped Distance (up to 10 ft), Weather Condition Offsets — [FrogPilot README](https://raw.githubusercontent.com/FrogAi/FrogPilot/FrogPilot/README.md)
- CEM triggers: curves, low speeds, slower/stopped lead, stop signs and lights, turn signal below a chosen speed, unknown speed limit; stop signs, lights and turns on by default — [FrogPilot README](https://raw.githubusercontent.com/FrogAi/FrogPilot/FrogPilot/README.md)
- Force Stop: stops in Chill Mode when the model expects a stop and there is no lead — [FrogPilot README](https://raw.githubusercontent.com/FrogAi/FrogPilot/FrogPilot/README.md). StarPilot ships "Force Stop 2.0" and adds Conditional Chill Mode — [StarPilot README](https://github.com/firestar5683/StarPilot)
- FrogPilot Curve Speed Controller slows for curves in Chill Mode with Adaptive (learns from the driver, default), Gentle, Standard, Sport profiles; it replaced the older Vision and Map Turn Speed Controllers — [FrogPilot README](https://raw.githubusercontent.com/FrogAi/FrogPilot/FrogPilot/README.md); [FrogPilot release notes via newreleases.io](https://newreleases.io/project/github/FrogAi/FrogPilot/release/June-21st%2C2025)
- FrogPilot Speed Limit Controller "follows posted limits plus an offset, never above your set speed" — [FrogPilot README](https://raw.githubusercontent.com/FrogAi/FrogPilot/FrogPilot/README.md)
- sunnypilot: SCC-V and SCC-M send targets directly to the longitudinal planner when sunnypilot longitudinal is active, and otherwise need ICBM; ICBM and Alpha Longitudinal are mutually exclusive; ICBM control is "slower and less precise than direct longitudinal control" — [sunnypilot ICBM doc](https://community.sunnypilot.ai/t/intelligent-cruise-button-management/244)
- sunnypilot release notes: SCC "when using any form of long control (sunnypilot longitudinal control or ICBM)" controls curve entry speed from map data and vision; DEC is "intelligent longitudinal control adaptation" — local `CHANGELOG.md` (2025.001.000 entry)
- Upstream: Experimental mode uses end-to-end world-model longitudinal planning since 0.10.0; 0.11.0 improved Experimental-mode longitudinal — [commaai RELEASES.md](https://raw.githubusercontent.com/commaai/openpilot/master/RELEASES.md); local `RELEASES.md`
- dragonpilot longitudinal set: ACM (coast when above set speed and nothing ahead), AEM, APM; DTSC and DASR dropped in 0.11.0 — [dragonpilot CHANGELOGS.md](https://raw.githubusercontent.com/dragonpilot-community/dragonpilot/pre-build/CHANGELOGS.md); [Substack](https://dragonpilot.substack.com/p/acm-nogg)
- StarPilot has "custom long-maneuver tests for regen-only vehicles" — [StarPilot README](https://github.com/firestar5683/StarPilot)
- Local tool for longitudinal bring-up: `openpilot/tools/longitudinal_maneuvers/` (`maneuversd.py`, `generate_report.py`, `mpc_longitudinal_tuning_report.py`) — local checkout

### Inferences
- Requires openpilot longitudinal (gas/brake actuation; no source says otherwise): CEM / Conditional Chill Mode, DEC in its full form, Force Stop, acceleration/deceleration profiles, personalities/Traffic Mode, human-like accel/following, stopped-distance increase, weather-based following offsets, ACM/AEM/APM, Experimental mode and Autospeed, one-pedal-style stops to standstill. Neither FrogPilot nor StarPilot documents a stock-ACC fallback for any of these.
- Works in degraded form with stock ACC via set-speed manipulation: speed-limit control and curve speed control (only as set-speed reductions, with button-press latency and no braking authority beyond what the stock ACC applies). This is what the port already does with SLA and SCC.
- The port's custom map-based stop-sign assist is the stock-ACC stand-in for Force Stop; a true stop-and-hold needs openpilot longitudinal or a stock ACC that will accept a very low set speed/cancel.
- For an EV with strong regen, StarPilot's regen-only longitudinal maneuver tests and FrogPilot's Eco+/coast deceleration profile are the most transferable longitudinal ideas once Fisker longitudinal leaves alpha.
- The local longitudinal maneuvers tool plus the MPC tuning report are the upstream-sanctioned way to graduate Fisker longitudinal from alpha.

### Gaps
- No fork states explicitly "requires openpilot longitudinal" per toggle in the pages I could read; FrogPilot's in-app gating logic was not inspected.
- Details of Force Stop 2.0, Conditional Chill Mode and StarPilot's curve learning (algorithms, thresholds, file paths) were not available.
- Whether Autospeed is gated on specific cars or requires openpilot longitudinal (very likely, but unstated in what I read).

## What do these forks offer for angle-steering cars specifically?

### Takeaway
Very little in any fork is angle-specific: the big lateral investments (NNLC/NNFF, custom torque controllers, Firestar Lateral Method, torque overrides, jerk-aware torque control) are torque-only, so for an angle car the transferable items are lane-change policy (modes, BSM gating, road-edge block, minimum speed), lateral position offsets, low-speed/blinker pause options, and upstream's lateral maneuver report for measuring the angle loop.

### Cited Findings
- sunnypilot Auto Lane Change modes: Off, Nudge, Nudgeless, Timed (0.5/1/2/3 s). Requires lateral active, a minimum speed (value not given), visible lane lines in the target lane ("a lane change will not execute without visible lane lines"), and an attentive driver. BSM Delay: with BSM and a timer beyond Nudge, a vehicle in the blind spot resets the wait timer (~1 s added until clear) — [sunnypilot Auto Lane Change doc](https://community.sunnypilot.ai/t/auto-lane-change/240)
- Forum notes on the same page: users report nudgeless lane-change onset can be abrupt; a reply attributes aggressiveness to the driving model rather than tuning and suggests trying a different model — [sunnypilot Auto Lane Change doc](https://community.sunnypilot.ai/t/auto-lane-change/240)
- The checkout has these lane-change params: `AutoLaneChangeTimer`, `AutoLaneChangeBsmDelay`, `RoadEdgeLaneChangeEnabled` (read in `openpilot/sunnypilot/selfdrive/controls/lib/relc.py`), plus `LaneTurnDesire`/`LaneTurnValue`, `BlinkerLateralReengageDelay`, and `CameraOffset` (range -35..35 in `openpilot/selfdrive/ui/sunnypilot/layouts/settings/models.py`, applied in `openpilot/sunnypilot/modeld_v2/camera_offset_helper.py`) — local checkout
- The torque-specific lateral code in the checkout (`latcontrol_torque_ext*.py`, `latcontrol_torque_jerk_aware.py`, `latcontrol_torque_v0.py`, `nnlc/`, params `EnforceTorqueControl`, `LateralJerkTorqueController`, `TorqueParamsOverride*`, `LiveTorqueParams*`) does not apply to `steerControlType = angle` — local `openpilot/sunnypilot/selfdrive/controls/lib/`, `opendbc_repo/opendbc/car/fisker/interface.py:27`
- sunnypilot angle-steering work elsewhere: Tesla "Coop Steering" (2025.002.000) and HKG angle steering (StarPilot credits sunnypilot as the source of its Hyundai/Kia/Genesis angle steering) — local `CHANGELOG.md`; [StarPilot README](https://github.com/firestar5683/StarPilot)
- FrogPilot lateral list: Always On Lateral, Lane Changes / Automatic Lane Changes, Pause Steering Below (always or only with signal), NNFF / NNFF Lite, Advanced Lateral Tuning (Developer level), "more steering strength" on select vehicles — [FrogPilot README](https://raw.githubusercontent.com/FrogAi/FrogPilot/FrogPilot/README.md). A June 2024 release added a customizable minimum lane-change speed under "Lane Change Customizations" — [FrogPilot June 1 2024 release](https://newreleases.io/project/github/FrogAi/FrogPilot/release/June-1st%2C-2024)
- FrogPilot Always On Lateral sub-options include LKAS-button control and "Pause On Brake Below" a set speed — [FrogPilot wiki: Always On Lateral](https://frogpilot.wiki.gg/wiki/Always_On_Lateral) (via search snippet)
- StarPilot Lane Centering: centres between detected lane lines and falls back to the model's end-to-end lateral output with a safety breakout — [StarPilot README](https://github.com/firestar5683/StarPilot); listed as an exclusive toggle with its own guide — [StarPilot wiki](https://wiki.firestar.link)
- dragonpilot: Position Offset (0.10.3 r2), LCA Speed Changer, Road Edge Detection, ALKA with panda safety and seatbelt checks — [dragonpilot CHANGELOGS.md](https://raw.githubusercontent.com/dragonpilot-community/dragonpilot/pre-build/CHANGELOGS.md)
- Upstream lateral maneuver report (0.11.1): scripted lateral-acceleration steps at 20 and 30 mph comparing commanded vs measured response; enabled by "Lateral Maneuver Mode" in Developer settings (`LateralManeuverMode` param); report via `tools/lateral_maneuvers/generate_report.py` — [comma blog 0.11.1](https://blog.comma.ai/0111release/); local `openpilot/tools/lateral_maneuvers/README.md`
- Upstream learns steering actuation delay online (0.9.9/0.10.0); sunnypilot exposes a live-delay toggle and software delay (`LagdToggle`, `LagdToggleDelay`, `openpilot/sunnypilot/livedelay/`) — [commaai RELEASES.md](https://raw.githubusercontent.com/commaai/openpilot/master/RELEASES.md); local checkout

### Inferences
- Angle-agnostic and adoptable now: FrogPilot's minimum lane-change speed and Pause Steering Below; a Louder Blind Spot Alert tied to the existing BSM gate; dragonpilot's Position Offset (the port already has the equivalent `CameraOffset`); StarPilot-style lane-line centring with model fallback (useful if the model's path hugs one side on the Ocean, but it reintroduces laneline dependence upstream removed).
- Nudgeless lane change with blind-spot gating already exists in the checkout (`AutoLaneChangeBsmDelay`); the Ocean has `enableBsm = True`, so the missing piece relative to FrogPilot is only the audible/visual emphasis (louder alert, red blind-spot path).
- Steering smoothing/jerk limits for angle cars are not a fork feature anywhere I found; they live in the car port's `carcontroller.py`/safety (angle rate limits) and are best tuned using the lateral maneuver report and the live-delay learner rather than borrowed from forks.
- "Abrupt lane-change onset" is reported as model-driven on sunnypilot, so the model selector (already present) is the documented lever; an angle-rate limiter during the lane-change desire would be a port-specific addition.

### Gaps
- No source describing angle-specific smoothing, low-speed turn handling, or lane-change tuning in FrogPilot/StarPilot/dragonpilot. FrogPilot's lane-change sub-toggles (lane-width detection, one-change-per-signal, delay) could not be confirmed for 2026; a search found no reference to "One Lane Change Per Signal".
- StarPilot Lane Centering page (settings, breakout condition, torque/angle applicability) returned 404 at the URL I guessed; details unknown.
- dragonpilot Position Offset and LCA Speed Changer behaviour is known by name only.
- sunnypilot "Lane Change Settings" page (minimum speed value) not fetched.

## What navigation features exist as of 2026?

### Takeaway
Upstream openpilot has had no model-integrated navigation since 0.9.7 (2024) and I found no 2026 sign of its return; navigation survives in forks — FrogPilot still lists "Navigate on openpilot" with route speed limits and Mapbox fallback, and StarPilot advertises routing via its Galaxy portal that "hints the driving model about exits and turns" — while sunnypilot (and this checkout) carries only a navd helper stub plus mapd OSM data.

### Cited Findings
- openpilot 0.9.7 "temporarily removed" Navigate on openpilot from Experimental mode to focus on the driving model; the 0.9.4 feature had fed a map image and nav instructions to the model — [comma blog 0.9.7](https://blog.comma.ai/097release/) (via search summary)
- Reddit users report navigation was later removed from upstream entirely and that comma connect lost destination setting — [r/Comma_ai thread](https://axebps-redlib.hf.space/r/Comma_ai/comments/1jt4prt/comma_connect_no_ability_to_set_destination_maps) (user reports, not primary)
- No navigation mention in the 0.11.1 release blog or in 0.10.x-0.11.2 release notes — [comma blog 0.11.1](https://blog.comma.ai/0111release/); local `RELEASES.md`
- 0.10.1's world model "removed global localization inputs" — local `RELEASES.md`
- FrogPilot: "Navigate on openpilot" route-based guidance including speed limits along the route; the SLC lists the NOO route as a speed-limit source; Public Mapbox Key enables Mapbox fallback; Map Data downloads by country/state, updated monthly; navigation widgets on the driving screen — [FrogPilot README](https://raw.githubusercontent.com/FrogAi/FrogPilot/FrogPilot/README.md)
- StarPilot: "Navigation that shows routes and hints the driving model about exits and turns"; navigation is delivered through Galaxy and listed under StarPilot-exclusive tools alongside Android Auto — [StarPilot README](https://github.com/firestar5683/StarPilot); [StarPilot wiki](https://wiki.firestar.link)
- sunnypilot docs index has no navigation page — [sunnypilot docs index](https://community.sunnypilot.ai/docs)
- Checkout: `openpilot/sunnypilot/navd/` contains only `helpers.py` (Coordinate class with `from_mapbox_tuple`, direction constants `left/right/straight`, speed conversions); a grep for `navInstruction|navRoute|navDesire|nav_desire|mapbox` in `*.py`/`*.capnp` (excluding tests) matched only that file and `openpilot/cereal/log.capnp` — local checkout
- sunnypilot models "provide additional outputs like 'turn desires'"; Lane Turn Desire exists in the checkout (`lane_turn_desire.py`) — local `CHANGELOG.md` (2025.001.000), local checkout
- An SAE 2026 paper exists on navigation-integrated end-to-end path planning with vehicle testing (not read) — [SAE 2026-01-0032](https://saemobilus.sae.org/papers/navigation-integrated-end-end-path-planning-development-vehicle-testing-2026-01-0032)

### Inferences
- With current upstream models taking no navigation input, StarPilot's "hints the driving model about exits and turns" most plausibly works by raising turn/lane-change desires from route maneuvers (the same desire channel Lane Turn Desire uses), not by feeding the model a route. This is an inference; the mechanism was not confirmed.
- For the Fisker port the cheapest route-aware feature is therefore: a route source (Mapbox Directions with a user key, or OSM routing over the existing mapd tiles) -> upcoming-maneuver distance/direction -> existing Lane Turn Desire / auto-lane-change desire, with the web HUD as the destination-entry UI. The cereal nav messages still exist in `log.capnp` and `navd/helpers.py` gives the geometry helpers, so sunnypilot appears to be keeping a foothold for this.
- Route-derived speed limits (FrogPilot) add little beyond what mapd already provides offline.

### Gaps
- No primary 2026 statement from comma on navigation's status or return; removal after 0.9.7 rests on user reports.
- StarPilot navigation internals (provider, how hints reach the model, whether it needs openpilot longitudinal) not found; wiki sub-page not fetched.
- Whether FrogPilot's NOO still functions with 2026 driving models (which lack nav inputs) or is display/speed-limit only is unknown.
- Whether sunnypilot has an in-progress navd revival beyond `helpers.py` is unknown.

## What driver-monitoring, alerting, logging/replay, or tuning tools would help a new car port mature?

### Takeaway
The most valuable maturity tools are upstream's lateral and longitudinal maneuver reports (both already in the checkout), online delay learning, and the 0.11.1 DM model; from sunnypilot, sunnylink's server-driven settings/backup and MADS wheel-touch enforcement; from the forks, phone portals with diagnostics and bug reporting (Galaxy, The Pond), per-alert volumes, screen recording, radar/lead stat overlays (dragonpilot) and regen-specific longitudinal tests (StarPilot).

### Cited Findings
- Lateral Maneuver report (new in 0.11.1): scripted lateral accel profiles at 20/30 mph, commanded vs measured, HTML report from `tools/lateral_maneuvers/generate_report.py` — [comma blog 0.11.1](https://blog.comma.ai/0111release/). Present locally with `lateral_maneuversd.py`; requires setting ACC manually to the on-screen target speed, aborts/repeats if speed is out of range, steering is touched or openpilot disengages — local `openpilot/tools/lateral_maneuvers/README.md`
- Longitudinal maneuvers tool with report generator and an MPC tuning report is in the checkout — local `openpilot/tools/longitudinal_maneuvers/`
- Other upstream tools present locally: `cabana`, `replay`, `plotjuggler`, `jotpluggler`, `clip`, `camerastream`, `joystick`, `sim` — local `openpilot/tools/`
- 0.11.1 DM: better phone detection (VLM-labelled training data), fewer false positives, head-pose alerts relaxed in turns (~7% fewer alerts), standstill exemption no longer triggers audible alerts as the car starts moving, `driverMonitoringState` restructured; thermal thresholds raised for comma four — [comma blog 0.11.1](https://blog.comma.ai/0111release/)
- 0.11.2: live camera streaming and dashcam clip generation from comma connect; new alert sounds — local `RELEASES.md`
- sunnypilot: "DM: wheel touch enforcement in MADS"; "MADS safety: enable heartbeat and lateral controls mismatch checks"; dedicated `controls_allowed_lateral` in safety; sunnylink SDUI, settings backup/restore, stats integration, uploader, CarParams fallback for brand capabilities; Developer UI with error-log viewer and Developer Metrics; dynamic alert size; Quiet Mode; tools for video concatenation and memory profiling; "torque: show static override values in Dev UI" — local `CHANGELOG.md`
- sunnylink locally: `openpilot/sunnypilot/sunnylink/` (`settings_ui.json` 75 KB schema-driven settings, `capabilities.py`, `statsd.py`, `uploader.py`, `backups/`, `athena/`) — local checkout
- sunnypilot docs list "Openpilot Toolkit (OPTK)" and "Share a Route" how-tos — [sunnypilot docs index](https://community.sunnypilot.ai/docs)
- Port-specific tools already present: `openpilot/sunnypilot/tools/fisker_drive_review.py`, `pull_footage.py`, the web HUD (`openpilot/sunnypilot/webhud/` with `dbc.py`, `extract.py`, `video.py`, `maptiles.py`), and a remote-control server (`openpilot/sunnypilot/selfdrive/remote_control/`) — local checkout
- FrogPilot: per-alert Alert Volumes; Report a Bug or an Issue from device or The Pond (quick or full detail); Screen Recorder; Capture Recent Footage; High-Quality Recording; Konik server option; Share FrogPilot Data (opt-in stats, filtered logs, learned speed limits); Settings Backups; Advanced Lateral/Longitudinal Tuning at Developer level; stat tracking/sharing added in late-2025 releases — [FrogPilot README](https://raw.githubusercontent.com/FrogAi/FrogPilot/FrogPilot/README.md); [FrogPilot releases](https://github.com/FrogAi/FrogPilot/releases)
- StarPilot: Galaxy remote portal (settings, models, updates, diagnostics, dashcam video); a ticket/log-submission system (firestar.link/logs); Firestar Lateral Method auto-tuning; custom long-maneuver tests for regen-only vehicles; PC-runnable Raylib UI; cross-compilation build script — [StarPilot README](https://github.com/firestar5683/StarPilot)
- dragonpilot: Display Lead & Radar Stats, Display Radar Tracks, External Radar Support with improved filters, Error Logs Button, Audible Alert Mode, Dashy and opview developer tools — [dragonpilot CHANGELOGS.md](https://raw.githubusercontent.com/dragonpilot-community/dragonpilot/pre-build/CHANGELOGS.md)

### Inferences
- Highest-leverage, zero-porting-cost actions: run the lateral maneuver report on the Ocean's angle loop (it needs only a manually set ACC speed, so it works with stock ACC), and run the longitudinal maneuver suite when exercising alpha longitudinal.
- dragonpilot's radar-track and lead-stat overlays match the port's mid-range radar tap well; they give a visual check of radar/vision lead fusion before trusting it for longitudinal.
- Exposing the port's Fisker-specific toggles through sunnylink's `settings_ui.json` schema would give remote/phone settings without building a separate Pond/Galaxy clone; the web HUD already covers the diagnostics half.
- Per-alert volume control and a louder BSM chime are small, self-contained alerting upgrades with no control-path risk.
- Keeping DM and MADS safety checks at upstream parity matters for staying mergeable into sunnypilot (its release notes stress alignment with comma's safety policy).

### Gaps
- Internals of Firestar Lateral Method (and whether it has any meaning for angle steering) not found.
- What "Dashy" and "opview" do in dragonpilot is known by name only.
- No evidence found of on-device live-tuning UIs for angle-steering parameters in any fork.
- Konik/comma connect compatibility of an unofficial Fisker fingerprint (route upload, clip generation, live streaming) was not researched.
