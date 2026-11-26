# Local stack inventory: what the Fisker Ocean + comma + Oceanscape system already provides toward a realistic 3D "reality view", and what it lacks

Scope and method. Local files only (two checkouts, read-only). Paths are abbreviated:

- `SP/` = `/Users/hiremanirmss/Fisker/sunnypilot/`
- `OS/` = `/Users/hiremanirmss/Fisker/oceanscape/` (branch `mallik-hud`, confirmed with `git branch --show-current`)

Reading depth, so the writer can weigh each claim:

- Read in full: `OS/README.md`, `OS/INTERNALS.md`, `OS/android/README.md`, `OS/static/js/perf.js`, `SP/openpilot/sunnypilot/webhud/README.md`, `video.py`, `state.py`, `extract.py`, `SP/opendbc_repo/opendbc/car/fisker/radar_interface.py`.
- Read in part (header comments, first 60-170 lines, or grep hits only): `world_model.js`, `models.js`, `fisker_radar.js`, `fisker_world.js`, `worker.js`, `roadcam.js`, `scene.js`, `sheet.js`, `osmtile.js`, the two DBCs, `cpu_budget.py`, `loggerd.h`, `services.py`, `values.py`.
- Not opened: `physics.js`, `mapsurface.js`, `roadnet*.js`, `furniture.js`, `mapfeatures.js`, `cutaway.js`, `groundstyle/surface.js`, `groundstyle/styles/*`, `server.py` body, `CAR_CONTROLS_PLAN.md`, `DESIGN.md` beyond its first 30 lines. Statements about these come from `INTERNALS.md`, not from the code.
- Nothing was run on the car or the comma. No frame rate, latency or message size was measured by me; every number below is one the repo states.

Items marked **[inferred]** are my reasoning, not something a file states.

## 1. Sensor and data inventory

### Takeaway
The HUD already receives a forward-looking object picture from three sources (an 8-slot OEM camera object list, a 32-slot mid-range radar, openpilot's two leads), camera lane lines and road edges, an ego-lane traffic light state, 16 ultrasonic zones, blind-spot flags, 10 Hz GPS with heading, and OSM road geometry with point features. Only one video stream reaches the HUD (the comma's road or wide camera, 1280x720 H.264, shown in a 2D window), and there is no raw side or rear perception, no depth, and no building, terrain or lane-level map data.

### Cited Findings

**Cameras and video**

- The only video path is the bridge's `/video` WebSocket: the comma livestream encoder's H.264 at 1280x720 with a keyframe every 5 frames, passed through untouched. — `SP/openpilot/sunnypilot/webhud/video.py:10-15`, `SP/openpilot/sunnypilot/webhud/README.md:22-30`
- Two cameras are selectable, `road` (narrow) and `wide`; the client switches with `{type: camera}`. — `video.py:46-51` (`CAMERAS = ("road", "wide")`)
- Encoder settings for the stream: default bitrate 5,000,000 b/s (env `STREAM_BITRATE` overrides), GOP 5; 1280x720 on the `MICI` device type; `MAIN_FPS = 20`. — `SP/openpilot/system/loggerd/loggerd.h:17,29-40,66-68`
- The livestream encode services are declared at 20 Hz. — `SP/openpilot/cereal/services.py:101-106`
- A cabin (driver camera) stream encoder exists in loggerd (`stream_cabin_encoder_info`, `livestreamCabinEncodeData`) but `video.py` does not offer it. — `loggerd.h:142`, `services.py:103,106`, `video.py:46-51`
- camerad and `stream_encoderd` run for the HUD only while a viewer is connected (`WebHudVideo` param), onroad too; the param clears 10 s after the last viewer leaves. — `video.py:11-13,60`, `SP/openpilot/system/manager/process_config.py:81-83,125,128`
- Each frame carries a 16-byte header with frame id and end-of-frame timestamp; the server also sends the camera intrinsics and the calibrated projection (`K`, `M = K . view_from_calib`) per camera. — `video.py:17-22,74-104`
- Over `/video` the model output is sent with 3D points (`[x, y, z]`): path, lane lines, lane probabilities, road edges, edge std devs, acceleration, up to two leads. — `video.py:23-24,107-135`
- In the page the video is decoded with WebCodecs `VideoDecoder`, hardware-preferred with software fallback, `optimizeForLatency: true`, and drawn to a 2D canvas in a draggable window in three styles (Road, Wide, Mirror at 536 x 240). — `OS/static/js/roadcam.js:1-7,41,306-313`, `OS/README.md:36-45`
- Back-pressure rules, in place of a measured latency: the server drops a viewer's backlog past 6 queued frames and waits for the next keyframe; the page holds at most 2 decoded frames and waits at most 40 ms for the matching model output. — `video.py:62,240-279`, `roadcam.js:16-22`
- The Android app relays `/video` to the page byte for byte. — `OS/android/README.md:36-38`

**What the comma's driving model publishes and the bridge forwards (`/ws`, 20 Hz)**

- Services forwarded: `carState`, `carParams`, `selfdriveState`, `selfdriveStateSP`, `carControl`, `radarState`, `modelV2`, `longitudinalPlan`, `longitudinalPlanSP`, `driverMonitoringState`, `deviceState`, `liveMapDataSP`, `gpsLocationExternal`, `gpsLocation`. — `SP/openpilot/sunnypilot/webhud/extract.py:15-19`
- From `modelV2` the bridge keeps: lane lines, lane-line probabilities, road edges, road-edge std devs, the path, at most two leads with probability above 0.5 (x, y, v), lane-change state and direction, and confidence. Points are 2D (x, y), every other one of 33, out to 120 m. — `extract.py:21-23,155-191`
- From `radarState`: `leadOne` and `leadTwo` (dRel, yRel, vRel, vLead, aLead, modelProb, radar flag). — `extract.py:130-152`
- No general object list, no semantic classes, no depth and no occupancy output is forwarded; the extractor has no such field. — `extract.py:169-191` (whole `model_v2` function)
- From `liveMapDataSP`: speed limit, next speed limit and distance, road name only. — `extract.py:253-259`
- The bridge also streams raw CAN frames, allowlisted by DBC message, from three buses: 0 (vehicle side), 1 (radar private link), 2 (OEM ADAS module side). — `SP/openpilot/sunnypilot/webhud/state.py:22-26,62-68`

**OEM ADAS camera object list (`OS/dbc/fisker_ocean_adas_world.dbc`)**

- Eight object slots, `ADAS_Obj1..8`, on IDs `0x33B 0x34B 0x32D 0x33D 0x34D 0x32F 0x33F 0x34F`. — `OS/INTERNALS.md:474`
- Per-object fields: ID, longitudinal and lateral distance (8 bits each at 0.2 m/bit, 0-51 m) with a sign bit each, width, length, height, a confidence bit, classification (4 bits), class confidence (2 bits); a brake-light flag is read from a separate signal. — `adas_world.dbc:867-879`, `OS/static/js/world/fisker_world.js:400-409`
- Classes: 0 Car, 1 Truck, 2 Motorcycle, 3 Bicycle, 4 Pedestrian, 5 Animal (marked "Not implemented"), 6 General object unknown size, 7 General object small, 8 General object large. — `adas_world.dbc:3443`, `fisker_world.js:23`
- The list reaches about 50 m; positions decode at 0.2 m/bit, not the matrix's 0.5; against the radar the fit is about 0.25 m/bit from about the rear axle. — `INTERNALS.md:474`
- The list carries no velocities. — `INTERNALS.md:247`, `OS/static/js/world/world_model.js:11-13`
- Measured corrections applied before fusion: range read as `x / 0.80 - 3.7`, lateral 1.35x too wide, latency about 0.23 s. — `INTERNALS.md:207-216`, `world_model.js:136-148`
- Highlights tie other functions to object IDs: ACC primary/secondary target, leading vehicle, BSD/DOW threat IDs, AEB/rear-AEB threat IDs, ELKA threat. — `INTERNALS.md:474`, `fisker_world.js:381-382,418-419`

**Mid-range radar (`OS/dbc/fisker_ocean_mrr.dbc`, `SP/opendbc_repo/opendbc/car/fisker/radar_interface.py`)**

- One cycle every 65 ms: a header (0x300) and 32 object slots (0x310..0x32F), on a private CAN-FD link on panda bus 1, receive only. — `radar_interface.py:6-11`, `SP/opendbc_repo/opendbc/car/fisker/values.py:55-57`
- The DBC is reverse-engineered from a drive and labelled a draft; the messages are not in the OEM matrix. — `fisker_ocean_mrr.dbc:1295`, `INTERNALS.md:155-157`
- Per-slot fields: ID (16 bit), measurement history, state, quality, age, longitudinal distance (13 bit at 0.05 m, field span 0-409 m), lateral distance (+/-204 m), relative velocity and acceleration (long and lat) with std devs, dynamic property, heading, width, length, class, status. — `fisker_ocean_mrr.dbc:82-117`
- Radar classes seen so far are tentative: 0 unclassified, 1 car, 6 pedestrian, 8 small. — `OS/static/js/world/fisker_radar.js:26-27`
- The radar gives no elevation. — `INTERNALS.md:225`, `world_model.js:39-41`
- Timing: measurement time is about 131 ms before the frame is received. — `fisker_ocean_mrr.dbc:1304`
- Calibration found: Doppler 1/16 m/s per bit (not the DBC's 0.06), output turned 0.6 degrees left. — `world_model.js:139-140`
- The car port forwards only `dRel`, `yRel`, `vRel` per track to openpilot, for tracks with state >= 1 and age >= 3 cycles. — `radar_interface.py:13-16,77-97`

**Blind spot, ultrasonic, surround view, other sensors**

- Blind spot reaches the HUD as left/right booleans (`carState.leftBlindspot/rightBlindspot`) and as OEM BSD display requests with a threat object ID. — `extract.py:59-60`, `fisker_world.js:381-382,418-426`
- Ultrasonic: 16 zone values (`ADAS_USS_F0..3, B0..3, L0..3, R0..3`) plus park-distance values in cm for 6 front and 6 rear sensors, and APA parking-slot corner coordinates. — `fisker_world.js:25-26,498-526`, `INTERNALS.md:478`
- Surround view: only a view-status label (`ADASDC_SVS_ViewSts`) is decoded. — `fisker_world.js:540`, `INTERNALS.md:478`
- An earlier research note (secondary source, itself citing the local DBC) lists the car's ADAS nodes as a front camera module, one mid-range radar and four corner radars (`CMRR_FL/FR/RL/RR`), and says the camera and corner radars sit on private links behind the ADAS ECU, so only the tapped MRR objects and the ADAS ECU's fused outputs are reachable. — `SP/research_notes/Fisker Ocean sunnypilot enhancements/fisker_ocean_specifics.md:17,29,60`
- Other ADAS outputs used: six lane lines (offset, heading, curvature radius, type, colour, confidence), lane width and types, adjacent lane info, oncoming lanes, landmarks, crosswalk and stop line distance, construction, curbs. — `INTERNALS.md:473`
- Signs and lights from the OEM camera: TSR speed limit, prohibition signs, traffic-light colour and shape, traffic-light distance, lead turn signal and brake. — `INTERNALS.md:477`
- The traffic light is the ego-lane light only (colour, arrow, solid or blinking, lamp count, orientation), with a distance ahead but no position across the road. The camera does not classify stop or yield signs. — `INTERNALS.md:121-132`

**GPS and pose**

- Telematics-box GPS at 10 Hz (`TBOX_0x526`), altitude (`TBOX_0x525`), fused heading with std dev (`TBOX_0x179`), TBOX IMU (`0x174..0x176`); fix quality is on IBUS1 only. — `INTERNALS.md:481`
- Fallback fixes: the comma's `gpsLocation(External)`, or the screen's own location through the Android app. — `INTERNALS.md:317-320`, `OS/static/js/world/worker.js:17-19`
- Pose is the rear axle in a local east/north frame, dead-reckoned from wheel speed and the yaw-rate gyro at CAN rate and corrected by fixes that are about 0.2 s old; wheel speed reads about 3% low. — `INTERNALS.md:317-326`
- The shown pose takes corrections with a 2 s time constant; it trailed the estimate sideways by 0.1-0.8 m typically, up to 3.5 m briefly. — `INTERNALS.md:452-460`

**Map data**

- Road tiles are sunnypilot mapd's OSM files: one packed Cap'n Proto file per 0.25 degree tile; each way has OSM id, name/ref, highway class, lanes, one-way flag, speed limits and node coordinates. Highway classes run from motorway to living_street. — `OS/static/js/world/osmtile.js:1-14`
- Point features come from Overpass per 0.05 degree cell: traffic signals, stop, give-way, crossing, mini-roundabout, level crossing, traffic calming; way tags kept include lanes, width, `turn:lanes`. — `OS/android/app/src/main/java/ai/sunnypilot/webhud/MapFeatures.kt:197-209`, `SP/openpilot/sunnypilot/webhud/maptiles.py:128-130`
- The match is "which road and roughly where along it, never which lane"; it agreed with mapd's road name 90-100% of the time on recorded drives. — `INTERNALS.md:340-341`
- "The map has no lanes and the GPS and the OSM centerline are both meter-level"; the drawn lane comes from the cameras. — `INTERNALS.md:351-356`
- Ordnance Survey (Great Britain only, needs the user's API key, served only by the APK): kerb-to-kerb road widths and surveyed outlines of carriageways, pavements and central reservations; the two maps differ by up to about 8 m, so OSM ways are snapped onto OS centrelines. — `INTERNALS.md:411-433`
- The head unit's ADASIS v2 horizon (segment, stubs, curvature and slope profile about 900 m ahead, sign profile) is forwarded and read but not decoded yet; its sign type table is not in the matrix. — `INTERNALS.md:132-134,482`
- Map data is prefetched within a set radius (default 25 km) and never dropped from disk. — `INTERNALS.md:435-441`

### Inferences
- **[inferred]** The video frame rate is 20 fps: `MAIN_FPS = 20` and the 20 Hz service declarations say so, but no file states the stream's delivered rate on the Pulse, and the HUD's own counter (`cam N fps`, `perf.js:103-105`) has no recorded value in the repo.
- **[inferred]** The ADAS object list can report objects behind and beside the car: it has a longitudinal sign bit (`fisker_world.js:400`), BSD and rear-AEB threat IDs point into it, and `scene.js:1161` handles "its front if it's behind us". No file states how far back or how reliably. This is the only rear or side object source that reaches the HUD.
- **[inferred]** The mid-range radar is forward-facing only: its origin is the front bumper (`fisker_radar.js:11-12`) and its longitudinal distance field is unsigned.
- **[inferred]** The 409 m longitudinal span is the field's capacity, not the sensor's range; no file gives the radar's real maximum range or field of view.
- **[inferred]** Surround-view camera video is not available to the HUD: only a status label is on CAN, and no code path reads such video (grep for `surround|SVS|birdseye` in `OS/static/js` and the Android sources matched only `fisker_world.js`).

### Gaps
- No measured end-to-end video latency (camera to pixels on the Pulse) is recorded anywhere I read.
- The actual radar range, field of view and track capacity in traffic are not stated; the DBC comes from a 61 s low-speed drive (`fisker_ocean_mrr.dbc:1295`).
- The ADAS object list's update rate was not read from the DBC.
- Conflict: `INTERNALS.md:474` says "the Fisker port has no radar" for openpilot's leads, while `radar_interface.py` now implements one (gated by `CP.radarUnavailable`) and `world_model.js:14-15` allows for radar-backed leads. I did not check which is live on the car.
- The corner-radar and front-camera claims come from an earlier research note, not from a file I verified against the DBC here.

## 2. World model

### Takeaway
A web worker fuses radar, the OEM camera list and openpilot leads into ground-fixed tracks, each an extended Kalman filter over a kinematic bicycle with class-based physical limits, a confidence value, and a simulated body for drawing. Its weak points are stated in its own comments: no elevation (overhead structures read as stopped cars), radar flicker, camera ranging error, and an unstable openpilot lead.

### Cited Findings
- Fusion runs in the page's worker, off the main thread: `dbc.js`, `fisker_world.js`, `fisker_radar.js`, `world_model.js`, `state.js`, `merge.js`, `worker.js`. — `INTERNALS.md:305-311`, `worker.js:1-8`
- The car's own buses are the primary source; a live comma tick is merged in. Output is 10 Hz from the car's buses alone and at the tick rate with the comma. — `worker.js:5-8,21-22,33`
- Each measurement is placed at the time it was taken (radar by its synced measurement time, the others by typical latency) with an ego pose dead-reckoned from wheel speed and the gyro; measurements are processed 0.2 s behind the present and tracks then predicted forward. — `world_model.js:24-29,56`
- Source roles: radar gives range and Doppler (bearing coarse, lateral velocity unreliable in turns); the ADAS list gives class, size, heading and "the widest coverage" with no velocity; openpilot leads only corroborate, and stand alone only when no other source exists. — `world_model.js:7-20`
- Object classes carried: car, truck, motorcycle, bicycle, pedestrian, animal, plus unknown/small/large. — `world_model.js:86`, `fisker_world.js:23`
- Track state: footprint centre, heading, speed along heading, acceleration, path curvature; limits per class (turning circle, about 0.8 g lateral, power-limited acceleration, braking, top speed). Pedestrians and animals may turn on the spot. — `INTERNALS.md:172-184`
- Drawn objects are simulated bodies steered toward the estimate, so filter noise does not show as jumps; vehicles are pushed apart so none overlap. — `INTERNALS.md:189-196`
- Snapshot fields per object: `cx`, `cy`, signed speed `v`, `accel`, `curvature`, `yawRate`, front-wheel `steer`, estimated `mass`. — `INTERNALS.md:198-199`
- Evaluation against the tracker it replaced, on the demo drive: overlapping vehicle pairs 174 to 0; 95th-percentile slip 0.9 to 0.3 degrees; steps over 6 m/s^2 16% to 5%; 95th-percentile jitter 1.6 to 1.0 cm. — `INTERNALS.md:200-205`
- Timing constants: radar tracks younger than 8 cycles are left out; a radar-only object stays hidden until its track has lasted 1.3 s; moving tracks are dropped 1.0 s after the last measurement, standing ones after 2.5 s. — `world_model.js:60,76-77`, `INTERNALS.md:233-234`
- Confidence: a camera class or an openpilot lead gives 100%; radar-only moving 85%, standing 60%; the view fades objects in between 35% and 65%. — `world_model.js:85-99`, `OS/static/js/scene.js:49,1192`

Known weaknesses the code and docs state:

- Overhead traffic lights, gantries and bridges are tracked as stopped cars in the lane until the car passes under them, because the radar has no elevation. — `INTERNALS.md:225-229`, `world_model.js:39-42`
- Young radar tracks are "mostly flicker (median life 0.13 s)". — `world_model.js:9-10`
- A radar class can flicker on for one cycle, so a class counts only after 5 cycles. — `INTERNALS.md:231-232`
- Uncorrected, a car seen by both the ADAS list and the radar splits in two at range. — `INTERNALS.md:218-219`
- The openpilot lead is "the least stable source here (in a parking lot it put a car 13 m ahead at 6 m)". — `world_model.js:16-17`
- `leadTwo` is usually the same car as `leadOne`. — `world_model.js:18-20`
- The radar sometimes sends a cycle twice. — `INTERNALS.md:219-220`, `radar_interface.py:63-66`
- The sources report the nearest face, not the centre; a radar point target with no class or size stays a point. — `INTERNALS.md:186-187`
- Camera ranging noise is modelled at about 6% of range plus 0.2 m quantisation. — `world_model.js:162-165`

### Inferences
- **[inferred]** Usable classified-object range is about 50 m (the ADAS list's limit); beyond that an object is radar plus at most two openpilot leads, so class and size beyond 50 m depend on the radar's tentative classes.
- **[inferred]** Coverage behind and beside the car rests only on the ADAS list's signed positions. The radar and openpilot leads are forward only.
- **[inferred]** At most 8 camera-classified objects exist at once (8 slots), which caps how dense a classified scene can be.

### Gaps
- I did not read `physics.js`, `merge.js` or `state.js`; the description of limits and merging comes from `INTERNALS.md`.
- No file states the model's object-count ceiling, or its accuracy for pedestrians and cyclists. The evaluation figures above are for vehicles on one demo drive.
- No evidence found of any occlusion reasoning beyond the "unseen by openpilot's model" rule (`world_model.js:98-102`).

## 3. Rendering

### Takeaway
The view is a stylised three.js r160 WebGL scene: a detailed rigged ego model, procedural low-poly shapes for other road users, lanes, a map road network with junctions and street furniture, and a route ribbon. It has no buildings, terrain, image textures, real-time shadows or video in the 3D scene, and the Pulse's Mali GPU has already forced geometry and bake reductions.

### Cited Findings

**Engine and renderer**

- three.js r160 (`REVISION` constant `"160"` in the vendored build), with `GLTFLoader`, `OrbitControls`, `BufferGeometryUtils`. — `OS/third_party/three/three.module.min.js:6`, `OS/README.md:61`
- `new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false, powerPreference: 'high-performance' })`; sRGB output. — `scene.js:313-316`
- Camera: perspective, 42 degree field of view, near 0.3 m, far 600 m. Fog from 70 to 190 m. — `scene.js:319,477`
- Lights: one hemisphere, one directional, one ambient; a PMREM-prefiltered procedural "studio" environment for the ego car's reflections. — `scene.js:103-129,322-325,361`
- Pixel ratio is the device's, capped at 1.75, times the render scale. — `scene.js:663`
- The screen is 1080 x 1920 portrait at a fixed 60 Hz. — `OS/DESIGN.md:3`, `perf.js:17`
- Plain ES modules, no build step. — `OS/README.md:14`

**What is drawn**

- Ego car: the Pulse Ocean v0.10 glTF model at true size, 23.3 MB, in factory paints and five wheel options; rolling and steering wheels; lamps following the BCM outputs; doors, windows and liftgate following the car. — `INTERNALS.md:11-19,276-280`, `OS/third_party/models/pulse_ocean_v0.10_parts.glb` (size from `ls`)
- Other road users are "procedural low-poly models in the soft, matte style of Tesla's visualization": extruded side profiles for a sedan and a truck, a two-wheeler for motorcycle and bicycle, a person for pedestrian, a block for animal, a cone for small objects. — `OS/static/js/models.js:1-2,87-102,448-455`
- Other cars' shadows are a soft blob texture drawn from a canvas, not shadow maps. — `models.js:127-143`
- Lanes: camera lane lines blended from the ADAS lines and openpilot's, shown only above a lane-confidence threshold; inferred lanes where neither reports. — `INTERNALS.md:21-44`
- Ground: a signed-distance-field ground and road, or a "ground style" (perforated is the default; pins, scanlines, point cloud) drawn from a height field baked each frame. — `INTERNALS.md:46-71`
- Map: every road within 320 m as ground-fixed strips with edges, centrelines and lane boundaries, rebuilt after 40 m of travel; junction mouths with fillets for the first 60 m of each arm. — `INTERNALS.md:360-376`
- Street furniture from the map: signal heads with stop lines, stop and give-way signs, striped crossings, level-crossing crossbucks, traffic calming, mini-roundabout rings. — `INTERNALS.md:383-390`
- The signal head ahead on our own road takes the camera's light state; other heads have a position but no state. — `INTERNALS.md:383-386`
- OS pavements and central reservations are laid over the strips where that data is loaded. — `INTERNALS.md:421-426`
- Route: the navigation route as a violet ribbon, reconstructed from the nav app's next-step notification only. — `INTERNALS.md:395-402`
- Also drawn: blind-spot warning as a red lane line or ripples, power trails, a gap bar behind the followed car, parking sensors. — `INTERNALS.md:76-85,136-146`, `OS/README.md:27-28`

**Performance work and limits on the Pulse**

- The ground sheet was cut from about 510k triangles (4 cm grid) to about 90k (8 cm, widening outward) because the dense one "held the car's GPU to under 20 fps at full resolution". The GPU is named as "a Mali". — `OS/static/js/groundstyle/sheet.js:10-12,17-18`
- Auto render resolution: when frames come below 88% of 60 Hz for two seconds with the median frame over budget, card blur goes first, then resolution steps 85%, 70%, 60%; steps come back after 8 s of keeping up. Fixed settings Full, 75%, 50% exist. — `perf.js:9-17,69-92`, `INTERNALS.md:297-303`
- The frame counter reports fps, median and worst frame, script time, draw calls, triangles and the camera window's rate. — `perf.js:101-112`
- Laying out a town's road network took 40-60 ms on the head unit's main thread, "a dropped frame or three each time", so it moved to a worker. — `INTERNALS.md:367-370`
- Commit history of the perf branch: "Ground styles: fast enough for the Pulse's GPU (coarser sheet, baked shadows, road maps halved along the road, shorter pin lattices)" (`d5cd773`); "cheaper bakes" (`d76909e`); "the road map back to every frame (every other measured no faster on the Pulse)" (`263a7dc`); "HUD: write to the page only what changed (~700 writes a second to none while driving)" (`3cad3ec`); "Auto resolution: only step down when the GPU is behind" (`cf76c18`). — `git log --oneline` in `OS/`
- A fabric bump map was prototyped and left off: "it doesn't read at the view's distance" (`85c9280`). — `git log --oneline` in `OS/`

**Evidence of absence (grep over `OS/static/js` and the Android sources, tests and two data tables excluded; a control pattern `THREE` matched 25 files)**

- `WebGPU|navigator.gpu`: no match.
- `shadowMap|castShadow`: no match.
- `TextureLoader|KTX2|.jpg|.webp`: no match.
- `InstancedMesh|THREE.LOD`: no match.
- `wasm|WebAssembly`: no match.
- `building`: one match, a mock settings entry "Buildings in 3D" marked `off: MOCK`. — `OS/static/js/carcatalog.js:512`
- The video is drawn with a 2D canvas context, in its own window. — `roadcam.js:41`

### Inferences
- **[inferred]** The renderer runs on WebGL2: three.js r160's `WebGLRenderer` asks for a WebGL2 context first and falls back to WebGL1. This is from my knowledge of the library, not from a file here, and the WebView's actual context on the Pulse was not checked.
- **[inferred]** No steady-state frame rate for the Pulse is recorded in the repo. The auto-scaler's existence and the "under 20 fps" note show the GPU is the binding limit at full resolution with heavy geometry.
- **[inferred]** There is no level-of-detail or instancing scheme, so a denser scene (many vehicles, buildings) would add draw calls and triangles linearly on a GPU where "every triangle costs" (`sheet.js:17`).
- **[inferred]** A 23.3 MB ego model is far heavier than any other asset; other-vehicle realism would need new assets, since none exist beyond the procedural shapes.

### Gaps
- No measured fps, draw-call or triangle counts for the Pulse appear in any file I read; the one timing I saw (`OS/CAR_CONTROLS_PLAN.md:110`, 4.3 ms per frame) is on a desktop Apple GPU.
- The Android WebView's version, and so its WebGL2, WebCodecs hardware-decode and WebGPU support, is not stated anywhere I read.
- I did not read `scene.js` in full, so the exact object pool size and draw distance for vehicles are not confirmed.

## 4. Compute and transport limits

### Takeaway
The bridge sends JSON ticks at 20 Hz and is deliberately kept light and niced on the comma; the comma's planner shares a core with its UI and has a CPU budget guard. The Pulse board drops a large share of CAN frames and has a weak GPU, but the Android app already ships and runs a native helper, so native code is possible; no NPU or machine-learning runtime is used anywhere.

### Cited Findings

**Transport**

- Tick rate is 20 Hz (`RATE_HZ = 20`). — `SP/openpilot/sunnypilot/webhud/server.py:68,318`
- Tick shape: `{now, reset?, brand, can: [[t, [[addr, bus, hex], ...]], ...], op: [[service, t, extract], ...]}` as JSON, CAN payloads as hex text. — `state.py:13-14,109-115`
- A new viewer first gets a snapshot of the latest frame of every message; a slow viewer's missed ticks are merged, and past 60 ticks it is started over. — `state.py:26,117-143`
- Unused radar slots are filtered out at the bridge. — `state.py:25,65-67`
- The page drops a socket silent for 6 s and reconnects; the app counts the comma lost after about 9 s. — `OS/android/README.md:43-47`
- The comma is reached over the head unit's Wi-Fi hotspot, relayed through the app's local server at loopback. — `OS/android/README.md:7-10,36-38`
- Per-source latencies used by the fusion: radar measurement to receipt about 0.10-0.13 s; ADAS list about 0.23 s; openpilot about 0.05 s; fusion delay 0.2 s. — `world_model.js:56-59,144`

**Comma**

- The bridge process is niced (`os.nice(10)`, "never compete with the driving stack") and reads nothing while no viewer is connected. — `server.py:1189`, `webhud/README.md:19-20`
- The stated design goal is that "the car's own screen carries the HUD, the comma stays light". — `INTERNALS.md:311-312`
- plannerd "has 50 ms a tick and shares core 5 with the UI"; optional extras are paused for 30 s (doubling to 600 s) if they take over 15 ms once or 3 ms on average. — `SP/openpilot/sunnypilot/selfdrive/controls/lib/cpu_budget.py:8-22`
- A report notes "card ran 45% of its core with speed control on" before a fix. — `SP/reports/openpilot speed control plan.md:160`
- A recent commit moved heavy map reads to a low-priority process: "map worker: every heavy map read in a low-priority process of its own; CPU budget for the planner's extras" (`f63ccd8ad`). — `git log --oneline` in `SP/`
- The user's standing note for work on the device: at most 1-2 analysis jobs, pinned to cores 0-1 and niced, because 10 jobs once exhausted RAM and killed the driving model mid-drive. — `/Users/hiremanirmss/.claude/projects/-Users-hiremanirmss-Fisker-sunnypilot/memory/MEMORY.md` (index entry "Comma analysis load")
- Turning the camera view on starts camerad's stream path and `stream_encoderd` onroad, which comma's own livestream does not do at ignition. — `webhud/README.md:28-30`
- The device type for the stream size is `MICI` (the comma 4). — `loggerd.h:32,40`, `OS/README.md:38`

**Pulse board and Android app**

- Pulse has two MCP251x CAN controllers on one SPI bus at 500 kbit/s; it drops about half the frames on IBUS1 and two-thirds on IBUS2 because both share the SPI bus and "every interrupt lands on CPU 0". — `OS/android/README.md:106-108,166-169`
- The app forwards CAN frames to the page in about 15 Hz batches. — `OS/android/README.md:135`
- Native code already ships: `libcanbridge.so` is a Go executable packaged as a `.so` so Android installs it into `nativeLibraryDir`, "the one directory an app may exec from"; it needs no root. — `OS/android/README.md:128-131`
- A root shell is available on the rooted head unit and is used for CAN hardware filters, the ARP table and silent installs. — `OS/android/README.md:22-23,157-162,222`
- Android build targets: `minSdk 26`, `targetSdk 36`, `compileSdk 36`. — `OS/android/app/build.gradle.kts:40-47`
- The APK is about 28 MB with the page and car model inside, served from loopback. — `OS/android/README.md:36,198`
- With the HUD off screen the app lets go of the comma so the bridge stops reading. — `OS/android/README.md:50-51`
- On Android Automotive a non-system app may be blocked while driving unless installed as a system app. — `OS/android/README.md:209-212`
- No machine-learning runtime: grep for `rknn|tflite|onnxruntime|nnapi|tensorflow|mediapipe|NPU` (whole word) over `OS/static/js`, the Android sources, the Go helper and the Gradle file returned nothing; a control grep for `VideoDecoder` matched `roadcam.js`.

### Inferences
- **[inferred]** The board is an RK3588S-class Rockchip: the task brief says so, and the only local evidence is a commit naming "Rock 5C Pulse units" (`55ad0ed`) and the "Mali" comment in `sheet.js:17`. No file I read names the SoC, its RAM, or its NPU.
- **[inferred]** JSON with hex-encoded CAN at 20 Hz is adequate for object lists but is not a path for dense data (point clouds, depth maps, per-frame meshes); those would need the binary `/video`-style socket or a new one.
- **[inferred]** The comma has little documented headroom: the planner shares a core with the UI, a budget guard exists because extras overran, and RAM ran out under analysis load. Any new perception model on the comma would compete with the driving stack; no file quantifies spare GPU or DSP capacity.
- **[inferred]** The Pulse's CPU and NPU are largely unused by the HUD today (one render thread, one world-model worker, one road-network worker, one Go helper), so on-board inference there is unexplored rather than ruled out.

### Gaps
- Tick sizes in bytes and the Wi-Fi link's measured throughput or jitter are not recorded.
- No CPU, memory, thermal or GPU utilisation figures for the Pulse appear in either repo.
- No free-capacity figures for the comma (per-core load, GPU, memory) appear in the files I read; `cpu_budget.py` gives limits, not measurements.
- Whether the WebView's H.264 decode is hardware-accelerated on the Pulse is not recorded; the code only prefers it (`roadcam.js:306-313`).

## 5. Gaps for a realistic view

### Takeaway
The missing pieces are mostly inputs, not drawing code: nothing supplies surrounding geometry (side and rear objects beyond flags, depth, buildings, terrain), state for anything but the ego-lane traffic light, or lane-level map truth. On the output side there are no realistic assets, textures, lighting or weather, and the GPU has little room.

### Cited Findings

| # | Gap | Evidence of absence or limit |
|---|-----|------------------------------|
| 1 | Side and rear perception | Radar origin is the front bumper with unsigned range (`fisker_radar.js:11-12`, `fisker_ocean_mrr.dbc:94`). Blind spot is two booleans plus a threat ID (`extract.py:59-60`, `fisker_world.js:381-382`). Corner radars are on private links, not tapped (`fisker_ocean_specifics.md:60`). Video offers only road and wide cameras (`video.py:46`). |
| 2 | Dense depth or 3D structure | The bridge forwards lanes, edges, path and two leads only (`extract.py:169-191`). No depth, occupancy or point-cloud data path exists; the only "point cloud" in the page is a ground style (`INTERNALS.md:67`). |
| 3 | Object variety and count | 8 camera slots with 9 classes, one of them "not implemented" (`adas_world.dbc:3443`); radar classes tentative (`fisker_radar.js:26-27`). No bus, van, trailer, cone-vs-barrier, or traffic-cone class beyond "small". |
| 4 | Object range with class | Camera list reaches about 50 m (`INTERNALS.md:474`); no velocity from it (`INTERNALS.md:247`). |
| 5 | Vertical information | Radar has no elevation; overhead structures become stopped cars (`INTERNALS.md:225-229`). Object height exists only in the ADAS list (`adas_world.dbc:874`). |
| 6 | Vehicle realism | Other road users are procedural low-poly extrusions (`models.js:1-2,448-455`). The only detailed asset is the ego model (`OS/third_party/models/`, one file). |
| 7 | Textures and materials | No image texture loading anywhere (grep `TextureLoader|KTX2|.jpg|.webp`: no match). A fabric bump map was dropped as not visible (`85c9280`). |
| 8 | Lighting, shadows, weather, time of day | No shadow maps (grep `shadowMap|castShadow`: no match); blob shadows (`models.js:127-143`). Fixed three-light rig and a studio environment (`scene.js:103-129,322-325`). Day/night is a theme (`INTERNALS.md:467`). No weather input was found in the forwarded services (`extract.py:15-19`). |
| 9 | Buildings and terrain | "Buildings in 3D" is a mock toggle (`carcatalog.js:512`). Road tiles hold ways only (`osmtile.js:1-7`); Overpass queries fetch road nodes and way tags only (`MapFeatures.kt:201-209`). The ground is flat by construction (signed-distance field or one height sheet, `INTERNALS.md:46-66`). Slope exists only in the undecoded ADASIS profile (`INTERNALS.md:482`). |
| 10 | Traffic lights and signs | Only the ego-lane light has a state, with distance but no lateral position (`INTERNALS.md:121-127`). Map signal heads elsewhere have no state (`INTERNALS.md:383-386`). The camera does not classify stop or yield signs (`INTERNALS.md:131-132`). sunnypilot adds map speed limits but no signs or lights (`INTERNALS.md:121-122`). |
| 11 | Pedestrians and cyclists | A class and a simple figure exist (`models.js:453`), but radar-only pedestrians needed special handling (`3ca6aa3`) and no accuracy figure exists. No pose, articulation or crossing intent. |
| 12 | Lane-level map | "Never which lane" (`INTERNALS.md:340`); lanes are parallel offsets of an OSM centreline at 3.6 m (`INTERNALS.md:348-356`); OSM and OS disagree by up to about 8 m (`INTERNALS.md:427`). Surveyed surfaces exist for Great Britain only, with a key (`INTERNALS.md:411-433`). |
| 13 | Position accuracy | Metre-level GPS; shown pose trails by 0.1-0.8 m, up to 3.5 m (`INTERNALS.md:458-460`). |
| 14 | Route | Rebuilt from the next-step notification only; Google Maps gives other apps nothing more (`INTERNALS.md:396-398`). Not yet tested on the car's head unit (`OS/android/README.md:99-101`). |
| 15 | Latency | Fusion runs 0.2 s behind and predicts forward (`world_model.js:27-28,56`); ADAS list lag 0.23 s (`world_model.js:144`); pose corrections on a 2 s time constant (`INTERNALS.md:454`). Video latency unmeasured. |
| 16 | Video in the 3D scene | Video is a 2D-canvas overlay window (`roadcam.js:41`, `OS/README.md:36`); it is not projected into the scene. Single forward viewpoint. |
| 17 | GPU headroom | Under 20 fps with a 510k-triangle sheet at full resolution (`sheet.js:11`); auto-scaler down to 60% resolution (`perf.js:12`). No WebGPU, instancing or LOD (grep: no match). |
| 18 | On-device inference | No ML runtime or NPU use on the Pulse (grep: no match); comma headroom undocumented and guarded (`cpu_budget.py:8-12`). |
| 19 | Data reliability on the Pulse alone | Half to two-thirds of CAN frames dropped (`OS/android/README.md:166-169`); car-only view posts at 10 Hz (`worker.js:33`); radar and openpilot need the comma (`OS/README.md:8-10`). |

### Inferences
- **[inferred]** The largest gap for a Tesla-style view is surround perception. Tesla's view rests on eight cameras feeding an occupancy/object network; this car exposes one forward radar, an 8-slot fused list, and two forward comma cameras. Closing it needs either new sensors, a tap on the corner radars, or a vision model run on the comma's wide camera, each outside what the local stack does today.
- **[inferred]** Static scenery (buildings, kerbs, pavements) is the cheapest realism gain available from data alone: OSM building footprints are one more Overpass query in a pipeline that already fetches and caches per-cell features, and OS surfaces already give pavements in Great Britain. The cost lands on the GPU, which is the tight resource.
- **[inferred]** Traffic-light and sign realism is capped by the OEM camera's ego-lane-only output; other heads can be placed from the map but not given a truthful state.
- **[inferred]** The existing physically constrained tracker and ground-fixed frame are a sound base for a richer view; the tracker's quality is not the limiting factor, its inputs are.

### Gaps
- Whether the ADAS list really reports vehicles alongside and behind in normal driving, and to what distance, needs a check on a recorded drive; no file answers it.
- Whether the corner radars or the surround-view cameras could be tapped is outside these repos.
- No user-facing requirement or acceptance target for "realistic" exists in either repo, so the gap list is against a general Tesla-style view, not a stated spec.
- The Pulse's real SoC, RAM, NPU and WebView version, and the comma 4's spare compute, must come from hardware documentation or measurement; the local files do not hold them.
