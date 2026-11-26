# This car's driving stack: Fisker Ocean + comma 4 + sunnypilot fork (branch mallik-fisker), by FSD scenario

Scope and method. All facts below come from local code, local reports, local research notes, the git log of
`mallik-fisker` (HEAD 566c66815 / merge 81e2af40a, 2026-11-26), and the comma's own drive summaries on the Mac
(`~/Fisker/routes/_summaries/*.json`, 150 files, aggregated by me on 2026-11-26). No web sources were used.
Path prefix: `SP` = `/Users/hiremanirmss/Fisker/sunnypilot`. "Assessment" marks my own judgement; everything else is
from a cited file.

Important limit on "defaults". The defaults below are the code defaults in `params_keys.h`. The params actually set on
the device were NOT read (that needs SSH to the comma). The drive summaries show that some assists have run live on
some drives (see each item). Treat "default off" as "off on a fresh install", not "off in this car today".

---

## Q1. What each assist does, what it needs, and its default (the capability inventory by scenario)

### Takeaway
Steering is openpilot's end-to-end model plus about ten Fisker-specific map, profile and car-signal assists. Almost all
speed-shaping assists need openpilot longitudinal ("alpha", debug panda firmware). Every custom assist defaults to OFF
in code, and several have a shadow (log-only) mode. No code handles roundabouts, navigation routes, parking,
pedestrians or cyclists explicitly. These depend on the driving model alone, or on the driver.

### Cited Findings

**Base layers (all scenarios)**
- Steering: angle control (`steerControlType = angle`), actuator delay 0.15 s, `steerAtStandstill = True`, `minSteerSpeed = 0`. — [interface.py:27-37,53](SP/opendbc_repo/opendbc/car/fisker/interface.py)
- Lateral-only by default: `openpilotLongitudinalControl` is set only when `alpha_long` is on. Without the LONG flag, the panda TX list has no 0x121 accel frame, so only steering (0x1D0) can go out. — [interface.py:43-47,61-65](SP/opendbc_repo/opendbc/car/fisker/interface.py); param `AlphaLongitudinalEnabled` is `DEVELOPMENT_ONLY` — [params_keys.h:41](SP/openpilot/common/params_keys.h)
- openpilot longitudinal needs a debug panda firmware, because the release firmware ignores the speed-control flag. — [openpilot speed control plan.md:22](SP/reports/openpilot%20speed%20control%20plan.md)
- Without openpilot longitudinal, the car's own ACC controls speed. The ICC settings frame 0x52A is rewritten to keep ACC on with `ICCACCFuncTyp = 2` (Advanced). `FiskerACCAutoSpeed` defaults to 1 (set speed follows the speed limit). — [values.py:195-204](SP/opendbc_repo/opendbc/car/fisker/values.py); [params_keys.h:237](SP/openpilot/common/params_keys.h)
- The longitudinal planner takes the MINIMUM speed target across all sources each tick: cruise, SCC vision, SCC map, Speed Limit Assist, stop sign, red light, crosswalk, speed bump, narrow street, junction turn, driver speed, wet road. — [longitudinal_planner.py:249-351](SP/openpilot/sunnypilot/selfdrive/controls/lib/longitudinal_planner.py)
- Dynamic Experimental Control (DEC) is in the planner. Param `DynamicExperimentalControl` defaults to 0. — [longitudinal_planner.py:17,54](SP/openpilot/sunnypilot/selfdrive/controls/lib/longitudinal_planner.py); [params_keys.h:269](SP/openpilot/common/params_keys.h)
- Engagement: North button = ACC only; East = ACC + MADS (steering). South long-press = MADS toggle. Cancel/resume/gap buttons are "TBD from bench testing". — [values.py:66-90](SP/opendbc_repo/opendbc/car/fisker/values.py)

**Highway lane keeping and speed**
- Lane keeping is the stock driving model's path. Fisker-specific fix: a command trim of 0.85 deg (learned slowly within 0.3-1.5 deg) because the car sat 30 cm left of the lane centre on the freeway (route 00000106). — [values.py:346-359](SP/opendbc_repo/opendbc/car/fisker/values.py)
- Steering rate limits fall steeply with speed: wind-up 2.5 / 1.5 / 0.2 deg per 10 ms frame at 0 / 5 / 25 m/s (= 250 / 150 / 20 deg/s of wheel); unwind 5.0 / 2.0 / 0.3. — [values.py:98-102](SP/opendbc_repo/opendbc/car/fisker/values.py); [turn_planner.py:146-148](SP/openpilot/sunnypilot/selfdrive/controls/lib/turn_planner.py)
- `steerRatio = 15.0` is described as "first-cut; refine from step-response logs". — [values.py:40](SP/opendbc_repo/opendbc/car/fisker/values.py)
- Speed: with openpilot longitudinal, Speed Limit Assist (`SpeedLimitMode` default 1 = information, `SpeedLimitPolicy` default 3). The speed limit also comes from the car's own sign reader (`ADAS_0x311 ADAS_TSRSpeedLimit` → `carStateSP.speedLimit`). — [params_keys.h:309-312](SP/openpilot/common/params_keys.h); [speed_limit/common.py:26-29](SP/openpilot/sunnypilot/selfdrive/controls/lib/speed_limit/common.py); [carstate.py:733,749-755](SP/opendbc_repo/opendbc/car/fisker/carstate.py)
- Driver-profile speed cap (`DriverProfileSpeed`, default 0): posted limit + the driver's own offset. Town offsets go up to +3 mph and fast roads up to about +5. — [params_keys.h:282](SP/openpilot/common/params_keys.h); git 0d11b3080, c755ecb8d
- SignDropHold: when the car's sign reader drops the set speed 9 mph or more, to 10 mph or more under the map limit, openpilot keeps the earlier set speed (openpilot longitudinal only). — git a26ac1fcc (route 00000150, a false 25 reading on a 35 road)
- Max speed seen in logs under openpilot steering: catalog rows show drives up to 83 mph max (e.g. 000000ea). The per-drive split of steering at that speed is not in the catalog. — [CATALOG.md](~/Fisker/routes/CATALOG.md)

**Following a lead (ACC behaviour)**
- Leads are vision-only (see Q3). Under openpilot longitudinal: profile following gap `follow_gap_s` (default 1.45 s, bounds 1.25-2.5); `lead_coast_s` (no accelerating toward a slower lead); LeadBrake (an acceleration ceiling, built at 2.5 m/s^3, max 2.5 m/s^2); LeadTakeOff (moving off as the lead moves, up to 30 mph); LeadFollowCap (the set speed may rise with a lead up to the limit + 10 mph). — [profile_long.py docstring, :56](SP/openpilot/sunnypilot/selfdrive/controls/lib/profile_long.py); [driver_profile.py:112-116,184-186](SP/openpilot/sunnypilot/selfdrive/controls/lib/driver_profile.py); [longitudinal_planner.py:276-301](SP/openpilot/sunnypilot/selfdrive/controls/lib/longitudinal_planner.py); git dc66f824e, 2de5bf6fe, f9896fbba, cc0e83d31
- All need `DriverProfile` = 1 (default 0) and openpilot longitudinal. — [params_keys.h:280](SP/openpilot/common/params_keys.h); [longitudinal_planner.py:268](SP/openpilot/sunnypilot/selfdrive/controls/lib/longitudinal_planner.py)
- Without openpilot longitudinal, the car's ACC (with its own radar fusion) does the following. — [Fisker Ocean sunnypilot enhancements.md:27](SP/reports/Fisker%20Ocean%20sunnypilot%20enhancements.md)

**Lane changes**
- Upstream openpilot lane change, gated by `LANE_CHANGE_SPEED_MIN = 20 mph`. Auto lane change modes: OFF, NUDGE (default), NUDGELESS, 0.5-3 s delay; `AutoLaneChangeTimer` default 0 (= nudge, so the driver must push the wheel). — [auto_lane_change.py:14-32,101-113](SP/openpilot/sunnypilot/selfdrive/controls/lib/auto_lane_change.py); [lane_turn_desire.py](SP/openpilot/sunnypilot/selfdrive/controls/lib/lane_turn_desire.py) (`LANE_CHANGE_SPEED_MIN`); [params_keys.h:146-147](SP/openpilot/common/params_keys.h)
- Blind spot comes from the car's OEM BSM (`enableBsm = True`; `leftBlindspot` from the ADAS display state). `AutoLaneChangeBsmDelay` defaults to 0. Heavy rain disables auto lane change. A chime sounds for a blinker into an occupied blind spot. — [interface.py:23-24](SP/opendbc_repo/opendbc/car/fisker/interface.py); [carstate.py:494](SP/opendbc_repo/opendbc/car/fisker/carstate.py); [auto_lane_change.py:67,104](SP/openpilot/sunnypilot/selfdrive/controls/lib/auto_lane_change.py); git e5073f19b
- The profile can supply the driver's own blinker-to-lane-change delay (only replaces a timed delay, not nudge/off). — [auto_lane_change.py:69-85](SP/openpilot/sunnypilot/selfdrive/controls/lib/auto_lane_change.py)
- The car's own lane-change assist setting is forced on in 0x35B (`ICC_EnbLnChgAsst = 1`). — [values.py:222-223](SP/opendbc_repo/opendbc/car/fisker/values.py)

**Curves**
- Smart Cruise Control Vision (`SmartCruiseControlVision`, default 0) slows for model-predicted curvature at max lateral acceleration 2.0 m/s^2. The profile overrides it with `curve_lat_accel` (bounds 1.2-2.5), multiplied by the wet-road factor. SCC Map (`SmartCruiseControlMap`, default 0) uses map curvature. Both need openpilot longitudinal. — [vision_controller.py:30,63,92-115](SP/openpilot/sunnypilot/selfdrive/controls/lib/smart_cruise_control/vision_controller.py); [params_keys.h:316,330](SP/openpilot/common/params_keys.h); [longitudinal_planner.py:222](SP/openpilot/sunnypilot/selfdrive/controls/lib/longitudinal_planner.py); [driver_profile.py:95,175](SP/openpilot/sunnypilot/selfdrive/controls/lib/driver_profile.py)
- At highway speed, the 20 deg/s wheel rate limit at 25 m/s is the car's steering ceiling. (Assessment: this is ample for highway curves but limits fast evasive steering.) — [values.py:100](SP/opendbc_repo/opendbc/car/fisker/values.py)

**Town speed control (limits, bumps, narrow streets, crosswalks, signals)**
- Speed Bump Assist (`SpeedBumpAssist`/`Shadow`/`Alert`, all default 0): eases off before OSM `traffic_calming` nodes to 10 / 15 / 20 mph by kind, at a 0.6 m/s^2 lift-off, up to 1.0 m/s^2 when late; not above 40 mph approach; local additions in `local_speed_bumps.json`. — [speed_bump.py docstring, :52-77](SP/openpilot/sunnypilot/selfdrive/controls/lib/speed_bump.py); [params_keys.h:319-321](SP/openpilot/common/params_keys.h)
- Narrow Street Assist (`NarrowStreetAssist`, default 0): (a) CenterLineKeeper shifts right up to 0.5 m from a seen centre line, only rightward, 2-25 m/s; (b) NarrowStreetDetector: no lane lines and curbs under 8.5 m apart at under 30 mph: caps speed (openpilot longitudinal only) and shows a "Narrow street" prompt. "The car can't see oncoming or parked cars, so who yields is the driver's call." — [narrow_street.py docstring, :28-51](SP/openpilot/sunnypilot/selfdrive/controls/lib/narrow_street.py)
- Map caution (both default 0): `TrafficSignalCoast` = no accelerating toward a mapped traffic signal (never braking); `CrosswalkCaution` = speed cap at marked crosswalks on narrow streets. — [map_caution.py docstring](SP/openpilot/sunnypilot/selfdrive/controls/lib/map_caution.py); [params_keys.h:323,325](SP/openpilot/common/params_keys.h)
- Comfort acceleration cap: at most 0.6 m/s^2 below 25 mph, rising to 1.0 at 45 mph. — [openpilot speed control plan.md:71](SP/reports/openpilot%20speed%20control%20plan.md); [profile_long.py docstring](SP/openpilot/sunnypilot/selfdrive/controls/lib/profile_long.py)

**Stop signs**
- Stop Sign Assist (`StopSignAssist`, `StopSignAssistShadow`, both default 0). Source: OSM `highway=stop` nodes plus hand-added `local_stop_signs.json`, with a direction filter against mapd road tiles. — [stop_sign_map.py docstring](SP/openpilot/sunnypilot/selfdrive/controls/lib/stop_sign/stop_sign_map.py); [stop_sign_direction.py docstring](SP/openpilot/sunnypilot/selfdrive/controls/lib/stop_sign/stop_sign_direction.py); [params_keys.h:317-318](SP/openpilot/common/params_keys.h)
- Behaviour: coast from 150 ft (45.7 m); brake at A_COMFORT 1.0 m/s^2 only after the driving model's plan confirms a slowdown (planned-speed ratio ≤ 0.70, or < 1.0 m/s, or 30% endpoint shortfall; 1.5 s filter); complete stop (target 0, no rolling option, citing NHTSA 22V-037); stop 6 m before the map point (9.5 m for wider streets); soft landing at 0.5 m/s^2 from 5 mph; hold until the driver taps the accelerator. Does not take on a sign above 25 mph approach; gives up if stopping needs more than 1.2 m/s^2. — [stop_sign_controller.py docstring, :68-120](SP/openpilot/sunnypilot/selfdrive/controls/lib/stop_sign/stop_sign_controller.py); git fbbb744c0
- No all-way turn-taking or automatic creep: "It never pulls away by itself: whether the junction is clear is the driver's call." — [stop_sign_controller.py docstring](SP/openpilot/sunnypilot/selfdrive/controls/lib/stop_sign/stop_sign_controller.py)

**Traffic lights**
- Red Light Assist (`RedLightAssist`, default 0; always runs and logs): reads the car's OWN traffic-light recognition (`ADAS_TLR_EgoLaneColor`, `ADAS_0x351 ADAS_TrafficLiDst`). A red/amber held 0.6 s, up to 200 m: lift off, then the stop-sign braking profile to a full stop. Stop point = light distance − 43 m setback, replaced by the model's stop point when close. Held until green or gas. Only the ego-lane light, with no lateral position. — [red_light.py docstring, :37,48-62](SP/openpilot/sunnypilot/selfdrive/controls/lib/red_light.py); [carstate.py:742-743](SP/opendbc_repo/opendbc/car/fisker/carstate.py); [Tesla Reality View equivalent feasibility.md:70](SP/reports/Tesla%20Reality%20View%20equivalent%20feasibility.md)
- Plan source `redLight` is added only with openpilot longitudinal. — [longitudinal_planner.py:257-258](SP/openpilot/sunnypilot/selfdrive/controls/lib/longitudinal_planner.py)
- ModelStop (with profile + openpilot long): no accelerating 0.5 s after the model's plan says stop; even braking to its stop point (≤ 2.0 m/s^2), aim 2 m short. — git 74d154bb2
- Green light / lead-depart alerts exist (`GreenLightAlert`, `LeadDepartAlert`, default 0). — [params_keys.h:167,176](SP/openpilot/common/params_keys.h)

**Junction turns**
- Lane Turn Desire (`LaneTurnDesire`, default 0; `LaneTurnValue` 19 mph, capped at 20 mph): with a single blinker below that speed and no blind-spot car on that side, the model gets a turn desire. Ends after 60 deg of heading change. — [lane_turn_desire.py](SP/openpilot/sunnypilot/selfdrive/controls/lib/lane_turn_desire.py); [params_keys.h:277,283](SP/openpilot/common/params_keys.h)
- The model alone does not start a turn at low speed ("16 recorded turns: it only follows once the driver steers"). So Map Turn Assist (`MapTurnAssist`, default 0, needs LaneTurnDesire) plans a circular arc from mapd offline OSM tiles: it arms at ≤ 16 mph (arming starts while slowing from 25 mph); aborts above 20 mph; arc radius 10 m, minimum 6 m, ≤ 2.5 m/s^2 lateral; wheel ≤ 120 deg/s; blends into the model over the last 30 deg; ends in the rightmost lane (right turn) or the nearest lane (left). Releases on 0.3 s of driver torque against it, a 2 s stall, or a 20 s timeout. — [turn_planner.py:7-27,41-160](SP/openpilot/sunnypilot/selfdrive/controls/lib/turn_planner.py); [params_keys.h:278](SP/openpilot/common/params_keys.h)
- Far approach (speed only): with a blinker above 25 mph and the kerb within 4.6 m on that side, it finds the junction up to 250 m / 12 s ahead and slows at one steady rate (≤ 1.2 m/s^2) to the driver's turn-in speed. — [turn_planner.py:57-73](SP/openpilot/sunnypilot/selfdrive/controls/lib/turn_planner.py); git 0091f939a
- Turn speed: default 10 mph; profile `junction_turn_mph` (5-16) and `junction_lat_accel` (0.92 m/s^2 measured). — [turn_planner.py:55](SP/openpilot/sunnypilot/selfdrive/controls/lib/turn_planner.py); [driver_profile.py:96,176](SP/openpilot/sunnypilot/selfdrive/controls/lib/driver_profile.py); git d9d88b36d
- Junction memory (the driver's own start/finish per junction) and the driver's lane-to-lane path are built but OFF. — git 97dfa9e2c, 1124d39cf; [junction_memory.py docstring](SP/openpilot/sunnypilot/selfdrive/controls/lib/junction_memory.py)
- Car-side: a cooperative signalled turn under 20 mph lets the driver push into the turn up to 5 Nm without that counting as an override. — [values.py:369-390](SP/opendbc_repo/opendbc/car/fisker/values.py)

**Roundabouts**
- No code. No file in `controls/lib` refers to roundabouts. The turn planner only treats branches 45-135 deg off as turns (beyond 135 deg is "a U-turn or a road doubling back"). — grep of [controls/lib](SP/openpilot/sunnypilot/selfdrive/controls/lib) (no matches for roundabout/circular); [turn_planner.py:86-87](SP/openpilot/sunnypilot/selfdrive/controls/lib/turn_planner.py)

**Merges**
- No merge steering logic. Only a profile speed value: `merge_accel` (0-1.0 m/s^2, default 0) applies on a road with a limit ≥ 55 mph when the car is ≥ 10 mph under it. Gap selection uses no side/rear sensing (see Q3). — [driver_profile.py:127-129,192](SP/openpilot/sunnypilot/selfdrive/controls/lib/driver_profile.py); [profile_long.py:344-349](SP/openpilot/sunnypilot/selfdrive/controls/lib/profile_long.py)

**Parking**
- No openpilot parking feature. The 0x35B override keeps the car's own APA / remote automated parking / trained parking SETTINGS enabled. The momentary actions (APAActivation, ParkOutAct, ParkSelect) are now in a frozenset that may not be a default. — [values.py:222-252](SP/opendbc_repo/opendbc/car/fisker/values.py)
- Remote control (phone angle over Wi-Fi, < 10 mph, 300 ms staleness, ±250 deg) exists. `RemoteControlEnabled` is `CLEAR_ON_MANAGER_START`, default 0, and the process is gated by a function. It has no obstacle check. — [carcontroller.py:42-59](SP/opendbc_repo/opendbc/car/fisker/carcontroller.py); [process_config.py:71,204](SP/openpilot/system/manager/process_config.py); [params_keys.h:253-260](SP/openpilot/common/params_keys.h); git 94dd80eaa; [enhancements report:11](SP/reports/Fisker%20Ocean%20sunnypilot%20enhancements.md)
- Full-lock pause: no steering request at ≥ 400 deg wheel (resume < 360) after an EPS fault while parking. — [values.py:310-317](SP/opendbc_repo/opendbc/car/fisker/values.py)

**Navigation following**
- None. Upstream removed navigation input in 0.9.7. The only turn inputs are the blinker and the map junction on the blinker side; nothing follows a route. — [enhancements report:49](SP/reports/Fisker%20Ocean%20sunnypilot%20enhancements.md); [turn_planner.py:7-12](SP/openpilot/sunnypilot/selfdrive/controls/lib/turn_planner.py)

**Pedestrians / cyclists**
- No explicit handling. The only mention in `controls/lib` is a comment that the turn planner treats a stop under 0.5 m/s as "waiting (pedestrian, cross traffic)". Any response comes from the end-to-end driving model's plan or from a vision lead. — grep (only match [turn_planner.py:130](SP/openpilot/sunnypilot/selfdrive/controls/lib/turn_planner.py))
- The car's own AEB stays in the loop only partly: openpilot's 0x118 replaces the frame that carries AEB activation; the report recommends a Tesla-style AEB passthrough (roadmap item 14). Whether this is implemented now was not checked. — [enhancements report:29,78](SP/reports/Fisker%20Ocean%20sunnypilot%20enhancements.md)

**Rain / night**
- Wet Road Assist (`WetRoadAssist`, default 0; detection always runs and logs): wet comes from wiper activity, heavy from continuous wiping or TCS. Effects: longer following gap, gentler curves and stop braking, posted limit + offset, lower accel cap. Heavy rain also turns off auto lane change and the map turn assist. `WetRoadForce` forces wet. — [wet_road.py docstring](SP/openpilot/sunnypilot/selfdrive/controls/lib/wet_road.py); [params_keys.h:326-328](SP/openpilot/common/params_keys.h)
- Night: no night-specific code. The narrow-street problem was first logged on a night drive (000000e8). — [narrow_street.py docstring](SP/openpilot/sunnypilot/selfdrive/controls/lib/narrow_street.py)

### Inferences
- Assessment: two stacks run in this car. Stack A (lateral only, car ACC for speed) is the shipped default. Stack B (openpilot longitudinal plus all the planner assists) is what makes stop signs, red lights, turns-with-slowdown, bumps and the driver profile do anything beyond logging. Stack B needs a development flag and a debug panda firmware.
- Assessment: by FSD's scenario list, this stack covers highway keeping/following/nudge lane changes, curves, town speed, stop signs (stop and hold only), red lights (stop and hold only, ego lane), and slow signalled junction turns from a map. It has nothing for roundabouts, navigation-driven routing, unprotected-turn gap judgement, all-way-stop turn-taking, merges, parking or vulnerable road users beyond what the model implies.

### Gaps
- Which params are set on the device now: not read (needs SSH). The summaries prove only that some runs used live (non-shadow) Stop Sign Assist and openpilot longitudinal.
- Whether DEC / experimental mode is used on the device: not determined (one commit cites "00000129 ... (experimental mode)", git cc0e83d31).
- Whether the AEB passthrough was implemented after the enhancements report: not checked.

---

## Q2. Car-port limits: steering, acceleration, takeover, SecOC, openpilot vs car ACC

### Takeaway
Lateral is mature but tightly capped: 400 deg command cap, 20 deg/s wheel rate at highway speed, 3 Nm takeover. Longitudinal is a modify-and-forward of the car's own signed accel frame, capped at +1.0 / −3.5 m/s^2 with 1.0 / 2.5 m/s^3 jerk limits. Every command depends on the correct SecOC key.

### Cited Findings
- Steering angle: DBC ±780 deg, panda/controller max 600 deg, openpilot's own command cap `LAT_ANGLE_CMD_MAX = 400 deg` (the EPS aborts near full lock, ~470 deg). The turn assist's tightest arc needs ~280 deg. — [values.py:94-102,339-344](SP/opendbc_repo/opendbc/car/fisker/values.py); [fisker.h:216](SP/opendbc_repo/opendbc/safety/modes/fisker.h)
- Rate: see Q1 (250/150/20 deg/s up, 500/200/30 down by 0/5/25 m/s). Low-speed smoothing: ≤ 70 deg/s, ≤ 300 deg/s^2 below 27 mph; unwind ≤ 120 deg/s. — [values.py:373-405](SP/opendbc_repo/opendbc/car/fisker/values.py)
- Takeover: 3.0 Nm held for 0.1 s (`OVERRIDE_STEER_TORQUE`, live-tunable `FiskerOverrideTorque` default 3.0; the EPS's own intervention trips at ~6.5 Nm). Resume below 0.5 Nm for 0.25 s. Angle-divergence override at 30 deg (release 10 deg). Calm-wheel gate: no new steering burst while the wheel moves faster than 90 deg/s (every abort in 261 re-grabs followed a wheel above 280 deg/s). — [values.py:285-337,361-368](SP/opendbc_repo/opendbc/car/fisker/values.py); [params_keys.h:250](SP/openpilot/common/params_keys.h)
- Standstill: lateral pauses below 1 mph. — [values.py:306-308](SP/opendbc_repo/opendbc/car/fisker/values.py)
- Accel: `ACCEL_MAX = 1.0`, `ACCEL_MIN = −3.5 m/s^2` (panda enforces the same caps). The 1.0 cap followed a car-side plausibility fault on route 000000c6 after ~1.5 m/s^2 actual for 0.6 s. Jerk 1.0 up / 2.5 down m/s^3. Analog smoothing with an urgent-braking bypass below −1.8 m/s^2. — [values.py:105-163](SP/opendbc_repo/opendbc/car/fisker/values.py); [fisker.h:220-221](SP/opendbc_repo/opendbc/safety/modes/fisker.h)
- Low-speed brake behaviour: below ~3 mph the VCU answers any request firmer than ~−0.5 by clamping the friction brakes (−1.7 to −2.5 delivered), so a soft-landing request of −0.45 is used from 5 mph down. — [values.py:136-149](SP/opendbc_repo/opendbc/car/fisker/values.py)
- Architecture: openpilot does not send its own speed frames. Each tick it hands the panda signed copies of 0x121 (one per possible counter), and the panda swaps the matching one into the OEM frame. With no match, the OEM frame passes. — [openpilot speed control plan.md:9-23](SP/reports/openpilot%20speed%20control%20plan.md); git c34744315
- "the port is the only one in opendbc that hands a signed accel frame back and forth mid-drive". — [enhancements report:3,23-27](SP/reports/Fisker%20Ocean%20sunnypilot%20enhancements.md)
- SecOC: `secOcRequired = True`; the 16-byte AES key must be in param `SecOCKey`; a wrong or absent key means no steering. — [interface.py:39-41,64-65](SP/opendbc_repo/opendbc/car/fisker/interface.py); memory note "wrong SecOCKey silently blocks steering"
- Gas: panda now reads the accelerator (0x214 → `gas_pressed`). This closes defect #1 of the enhancements report. — [fisker.h:20,514](SP/opendbc_repo/opendbc/safety/modes/fisker.h); git 4d435de9c

### Inferences
- Assessment: the 1.0 m/s^2 accel cap and the 20 deg/s highway wheel rate make this car's stack deliberately gentle. It cannot do FSD-style assertive merges or quick evasive lane shifts, and LeadBrake (max 2.5 m/s^2) plus the MPC (to −3.5) is the braking envelope. Harder braking relies on the car's own AEB.
- Assessment: several defects listed in the enhancements report appear fixed in today's code (gas_pressed, APA activation now momentary-only, remote control opt-in, stop=all ranking, the model confirmer). The report text is older than the code.

### Gaps
- Whether the car actually delivers braking through the swapped frame at all speeds: the plan listed it as unknown on 2026-11-17. Later commits measure delivered decel (e.g. values.py:136-139 from 64 drives), which suggests braking is delivered, but no single statement confirms it.
- Standstill hold/resume: partly handled ("off the standstill hold at once when asked to go", git cc0e83d31). Full MEB-style hold logic is not confirmed.

---

## Q3. Sensor reality

### Takeaway
On every recorded drive this car drives from one forward camera pair only. The radar link is silent, so all leads are vision. The OEM camera object list never reports anything beside or behind the car and reaches about 40-51 m. Corner radars appear only as two blind-spot flags.

### Cited Findings
- Radar: 0 frames at the radar addresses on panda bus 1 in all 510 segments of 89 drives; `radarTracks` 545,362 messages with 0 points; `carParams.radarUnavailable = true`. Cause unknown (harness, connector, power or wake-up). — [phase0_log_measurements.md §2](SP/research_notes/Tesla%20Reality%20View%20equivalent%20feasibility/phase0_log_measurements.md)
- The code would use the radar if present (`radarUnavailable = RADAR_HEADER not in fingerprint[bus 1]`; 32-slot MRR parser). — [interface.py:55-59](SP/opendbc_repo/opendbc/car/fisker/interface.py); [radar_interface.py:6-24](SP/opendbc_repo/opendbc/car/fisker/radar_interface.py)
- Leads: present in 216,510 radarState frames, 0 radar-backed. — [phase0 §4](SP/research_notes/Tesla%20Reality%20View%20equivalent%20feasibility/phase0_log_measurements.md)
- Vision lead quality issue: "vision reads a stopped car as 19, 10, 5 mph in its first second"; a stopped car at a red first read as closing only 11 s out. — git dc66f824e; [red_light.py docstring](SP/openpilot/sunnypilot/selfdrive/controls/lib/red_light.py)
- OEM object list (ADAS_Obj1..8, bus 2): 0 of 602,489 object frames behind the car; "beside" objects are all 0-5 m ahead of the bumper; max range 40.6 m raw (~51 m corrected); no track that started ahead ever continued behind. With the blind-spot flag on, the list held a car on that side in 0.27% of snapshots. "The car's corner radars do not feed the list." — [phase0 §1](SP/research_notes/Tesla%20Reality%20View%20equivalent%20feasibility/phase0_log_measurements.md)
- The OEM object list is not wired into openpilot's planning: no `ADAS_Obj` reference in controls/lib (grep; only a test file matched "APA/parking"). — grep of [controls/lib](SP/openpilot/sunnypilot/selfdrive/controls/lib)
- Cameras: only the wide and narrow road cameras plus a cabin camera; the factory surround-view video is not tapped. Narrow camera ~22 px/deg on comma four vs ~48 on comma 3X. — [Reality View report:63,73](SP/reports/Tesla%20Reality%20View%20equivalent%20feasibility.md)
- Blind spot: two booleans plus a threat ID; corner radars sit on private links. — [Reality View report:64](SP/reports/Tesla%20Reality%20View%20equivalent%20feasibility.md)
- Traffic lights from the car's camera: ego-lane light only, with distance but no lateral position. The camera does not classify stop or yield signs. — [Reality View report:70](SP/reports/Tesla%20Reality%20View%20equivalent%20feasibility.md)
- The car camera reports the light colour 8 s or more before the stop in 91 of 117 red-light stops (40 drives). — [red_light.py docstring](SP/openpilot/sunnypilot/selfdrive/controls/lib/red_light.py)
- The car publishes 16 ultrasonic ranges in `ADAS_0x352`; openpilot does not use them. — [enhancements report:11](SP/reports/Fisker%20Ocean%20sunnypilot%20enhancements.md)
- Compute: model execution p50 27.5 / p99 30.6 ms of a 50 ms budget; frame drops 0 in 97.7% of frames; device CPU p50 58%, p99 77%, max 90%; memory p50 68%, max 96%. — [phase0 §3](SP/research_notes/Tesla%20Reality%20View%20equivalent%20feasibility/phase0_log_measurements.md)
- The comma four "has no spare capacity to plan on" (Snapdragon 845, 3.6 GB RAM, ~1.8 GB free parked; analysis jobs once killed the driving model mid-drive). controlsd/selfdrived/card share core 4 at 90-93% busy. — [Reality View report:81](SP/reports/Tesla%20Reality%20View%20equivalent%20feasibility.md); git 395b9d7df

### Inferences
- Assessment: everything beside and behind the car, beyond two blind-spot booleans, is unknown to the stack. Merges, unprotected turns, roundabout entry and lane-change gap choice therefore rest on the driver (nudge) and the OEM BSM.
- Assessment: fixing the silent radar link is the cheapest large gain. It would give radar-backed leads and range beyond ~51 m at no hardware cost (the code path already exists).

### Gaps
- Why the radar link is silent: unknown.
- Video-on device load: no segment ran the stream encoder.

---

## Q4. Evidence of how well it works on this car (reports, git log, drive summaries)

### Takeaway
There is a lot of evidence and it is mixed. Lateral runs for most moving time. Map junction turns complete most of the time but with frequent driver help. openpilot stops are still much harsher than the driver's own. Stop signs and speed bumps have mostly run in shadow mode. openpilot speed control has run for about 3.3 h over its history, and most runs end with a driver takeover.

### Cited Findings (aggregated by me from `~/Fisker/routes/_summaries`, 150 drive summaries, unless another source is given)
- Totals: 689.5 miles, 1,379 min moving, openpilot steering 946.5 min (≈ 69% of moving time), openpilot speed control 197.5 min (≈ 14%) in 582 runs. — [_summaries](~/Fisker/routes/_summaries)
- How openpilot speed-control runs ended (handover counts): gas 280, openpilot off 204, car ACC states 7/8/9/10/4 = 57, "vcu not ready" 3. — [_summaries](~/Fisker/routes/_summaries)
- Steering overrides counted: 5,520 over 946.5 steering min (definition per the summary tool; includes turns). — [_summaries](~/Fisker/routes/_summaries)
- LKAS faults: 15 over the 72-drive catalog (to 2026-11-20). — [CATALOG.md](~/Fisker/routes/CATALOG.md). "9 of the 14 EPS faults in 82 drives came at 444 deg or more." — [values.py:310-315](SP/opendbc_repo/opendbc/car/fisker/values.py)
- Map turn assist outcomes (165 turns): done 117 (71%), driver override 32 (19%), blinker off 5, not turning 4, too fast 4, other blinker 3. Headlines: "turn assist held the wheel" in 26 of 127 turns. — [_summaries](~/Fisker/routes/_summaries)
- Stop signs: 466 shadow passes (107 confirmed, 69 reached braking, 13 stopped); 35 live passes (15 confirmed, 13 braking, 2 stopped). Left to the driver: "taken on too close" 60, "confirmed too late" 17. — [_summaries](~/Fisker/routes/_summaries)
- Stop smoothness (final-0.5 s jerk): openpilot longitudinal stops n=60, median 8.9 m/s^3, p90 17.0, 12 rolled more than 0.5 m; driver stops n=381, median 1.4, p90 4.3. — [_summaries](~/Fisker/routes/_summaries). Target in the plan: under 1.0 m/s^3. — [speed control plan:123](SP/reports/openpilot%20speed%20control%20plan.md)
- "Every pedal-free stop on record ended above 1.5 m/s^2, and in 6 of the 11 hardest the car's ACC reported a failure at standstill." Rules replayed over 86 stops act in 72, with one false braking in 2.2 h. — git 74d154bb2
- Speed bumps: 263 shadow, 8 live events. Wet road detected 105.5 min (heavy 71 min); wet-road changes applied 0 min. — [_summaries](~/Fisker/routes/_summaries)
- Red Light Assist replay (48 drives): 159 engagements; a median 10.5 s before the driver's own stop at reds with no car ahead (33); stop point within 1 m of the driver's in 30; would have braked 6 times when the driver went on. — git fbbb744c0
- Audit of openpilot speed control, 39 drives / 181 stretches / 66 min: 97 ended on gas, 78 on brake; 22 of 35 gas takeovers had the profile speed cap holding the car back. — git c755ecb8d
- Following distance: openpilot follows further back than the driver (4.2 s at 13-25 mph vs 1.7 s; 2.3 vs 1.7 at 25-40; 2.1 vs 1.5 above). — git f9896fbba
- Moving off: of 30 stops of 2 s or more, the car moved off by itself in 3 (before LeadTakeOff); afterwards it fires a median 0.45 s after the lead moves (54 standstills). — git cc0e83d31, a26ac1fcc
- First drive with speed control + new features (00000103): "accelerated fast, overshot stop sign, missed turns, went dangerously close to cars". Causes and fixes are tabulated, and "close to cars" was not located in the log. — [speed control plan:63-73](SP/reports/openpilot%20speed%20control%20plan.md)
- Incident-driven fixes in the log include: full-lock EPS abort and surge (400 deg cap); a stop-sign overshoot on 00000136 (LAG_S 1.0→1.6, buffer 1.5 m); the planner turning into the wrong cross street (approach-skip rule); re-grabbing a spinning wheel (calm-wheel gate); bumps releasing the steering (0.1 s torque debounce); freeway lane offset (trim). — git 83df8b096, b92e1d179, a08f882ba, 98dab462b, 6b1aa89ab; [values.py](SP/opendbc_repo/opendbc/car/fisker/values.py)
- Report verdict: "Its lateral path is mature and its longitudinal path is ... architecturally unusual". — [enhancements report:93](SP/reports/Fisker%20Ocean%20sunnypilot%20enhancements.md)

### Inferences
- Assessment: lateral assistance is the dependable part (most moving minutes, faults now mostly understood). openpilot longitudinal is still alpha in quality on this car: stops are about 6× the driver's jerk, and takeovers end most runs.
- Assessment: Stop Sign Assist and Speed Bump Assist have far more shadow than live evidence. Red Light Assist evidence is replay-only, and its replay agreement is flattered because the model's plan follows the driver's braking (the commit says so itself).

### Gaps
- The summaries' "steer_overrides" and "held the wheel" definitions were not read from the summary tool's code, so read them as indicative only.
- No per-scenario disengagement rate (e.g. per highway mile) was computed.

---

## Q5. What is impossible on this hardware, what more software could add, and what needs Phase 4 hardware

### Takeaway
Assessment: forward-only perception and a saturated comma four put a hard ceiling on FSD parity. Many forward-scenario gaps can close with software, and the car's own unused signals help most. Anything that needs knowledge of beside or behind the car (merges, roundabouts, unprotected turns, automatic lane-change gap choice, parking) needs added sensors or an OEM-sensor tap.

### Cited Findings (facts that anchor the classification)
- No side or rear object source beyond flags; no side or rear cameras; the OEM list never sees beside/behind. — [phase0 §1](SP/research_notes/Tesla%20Reality%20View%20equivalent%20feasibility/phase0_log_measurements.md); [Reality View report:63-64](SP/reports/Tesla%20Reality%20View%20equivalent%20feasibility.md)
- "Phase 4: the rear needs added sensing for any real content. The OEM list will not provide it." Added surround sensing is costed at $399 to about $1,400. — [phase0, "What this changes"](SP/research_notes/Tesla%20Reality%20View%20equivalent%20feasibility/phase0_log_measurements.md); [Reality View report:26](SP/reports/Tesla%20Reality%20View%20equivalent%20feasibility.md)
- The car already publishes, unused by openpilot: speed-limit recognition, traffic-light and forbidden-sign data, an ADASIS horizon, driver monitoring, 16 ultrasonic ranges. Which are populated on shipped firmware is unknown. (Since the report, TLR colour and the TSR limit are now read: carstate.py:733,742.) — [enhancements report:3,53](SP/reports/Fisker%20Ocean%20sunnypilot%20enhancements.md)
- The model has no stop-sign, light or stop-line output. — [enhancements report:47](SP/reports/Fisker%20Ocean%20sunnypilot%20enhancements.md)
- Report advice: "No fork implements automatic creep or all-way turn-taking, and none should be attempted here." — [enhancements report:49](SP/reports/Fisker%20Ocean%20sunnypilot%20enhancements.md)

### Classification (all assessment, built on the facts above)
| Scenario | Today | More software, same hardware | Needs Phase 4 hardware (or a corner-radar/surround tap) |
|---|---|---|---|
| Highway lane keeping | Yes (model + trim) | Better steerRatio (lateral maneuver report, roadmap #18) | No |
| Highway speed / following | Car ACC, or openpilot long (vision leads) | Fix the silent radar link (free, if the cause is wiring/wake); radar Class/DynProp into leads | No |
| Lane changes | Nudge, OEM BSM blocks | Nudgeless timer exists; safer gating with BSM threat ID | Auto lane change choosing gaps in dense traffic: yes (no rear speed/range) |
| Curves | SCC vision/map (openpilot long) | Profile tuning | No |
| Town speed (limits, bumps, narrow streets) | Yes, map + camera + TSR (mostly shadow so far) | Live validation; ADASIS horizon | No |
| Stop signs | Stop and hold; driver releases | Better direction data; OEM FobdSign as a confirmer | All-way turn-taking / creep: judged not to attempt; cross-traffic awareness needs side sensing |
| Traffic lights | Ego-lane red/amber stop via the OEM TLR (replay-validated) | Map four-case signal rule; green go via driver | Lights for other lanes/turn arrows: limited by the OEM camera's ego-lane-only output |
| Junction turns | Signalled, ≤ 16 mph, map arc; ~71% done, 19% overridden | Junction memory/lane path (built, off) | Unprotected-turn gap judgement: needs side sensing |
| Roundabouts | None | Possible map-arc extension for entry/exit geometry | Yield to circulating traffic: needs side sensing |
| Merges | Speed only (merge_accel) | — | Yes (no rear/side ranges) |
| Parking | None (car's own APA settings kept on) | Use the 16 ultrasonics as a guard for remote control | Automated parking by openpilot: needs surround sensing or a tap into the car's APA |
| Navigation following | None (upstream removed nav) | Route-driven blinker/turn selection is pure software, unbuilt | No |
| Pedestrians / cyclists | Model only | Use the OEM object list's class field as an extra check | Crossing VRUs from the side and behind: needs coverage |
| Rain / night | Wet detection (logged; effects 0 min applied) | Turn on and validate | Camera-only limits in heavy rain/glare remain |

### Gaps
- Whether the corner radars or surround cameras can be tapped: "No reliable source ... treat as high-risk". — [Reality View report:112](SP/reports/Tesla%20Reality%20View%20equivalent%20feasibility.md)
- Whether compute could host extra perception: the comma is saturated. Any added sensing would need its own compute (Reality View approach D mentions a Jetson). — [Reality View report:26,81](SP/reports/Tesla%20Reality%20View%20equivalent%20feasibility.md)
