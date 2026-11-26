# openpilot speed control on the Fisker Ocean: plan

2026-11-17. Goal, in the driver's words: stops that are "smooth, one should not feel the jerk, head should not move",
and turns that are "slow, nice and easy". Today every stop is made by the car's own adaptive cruise, which openpilot
cannot shape. openpilot speed control (alpha) is what would let us shape them.

## Where it stands

How it works now (commit f01622a4, 2026-11-14, "modify and forward"):

- openpilot speed control rides on top of the car's own adaptive cruise. The driver engages the car's cruise as usual.
  While the cruise is active, openpilot computes an acceleration request every 10 ms.
- openpilot does not send its own speed frames. Each tick it hands the panda a set of fully signed copies of the
  car's own accel frame (0x121) with openpilot's request inside, one per possible message counter. The panda swaps the
  matching copy into the car's own frame on its way to the drivetrain. With no matching copy, the car's own frame
  passes unchanged: the car's cruise drives that frame, nothing is dropped.
- So there is always exactly one sender, with the car's own counters. The earlier designs (our own frames, blocking
  the car's) made the drivetrain withdraw "ACC ready" 50 ms into every engage (route 000000ea, 9 of 9 engages).
- Limits: +1.0 m/s^2 up, -3.5 m/s^2 down, request change 1.0 m/s^3 rising, 2.5 m/s^3 falling. The panda enforces the
  same caps.
- Gas: the panda and openpilot see the pedal; while pressed, no copies are sent and the car's own frame passes.
- It needs a debug panda firmware (the release firmware ignores the speed-control flag).

What we know works: the copies match the car's frame byte for byte (11,509 of 11,537 frames in a replay; the misses
are at signing-window restarts). Route f5 (2026-11-15) ran it, and drive fe (2026-11-16) ran it for 12.3 min in 14
runs with no faults.

What we do NOT know (each blocks something below):

| Unknown | Why it matters |
|---|---|
| Does the car actually brake when we request braking through the swapped frame? | Smooth stops need braking authority |
| How closely does the car's real acceleration follow our request, and what mismatch makes it fault? (route 000000c6: +0.4-0.6 m/s^2 above the request for 0.6 s, then cruise failure) | Sets how hard we may ask, and how fast |
| What happens at a standstill: does the car hold, roll, creep? Who resumes? | A stop must end held, without a lurch |
| What the car does during a gas override | Clean hand-back after the driver's foot |
| Does the car's own cruise mark the accel frame "valid" even in standby? (the DBC says yes) | Substitution must happen only while the cruise is really active |

## Finding from recorded drives (2026-11-17, before any new drive)

Measured with `.claude/skills/drive-review/scripts/stops.py` over 8 recorded drives (jerk = how fast the braking
changes over the last 0.5 s before stopping; the "head nod"):

| Who stopped the car | Stops | Final-0.5 s jerk |
|---|---|---|
| The driver (brake pedal) | 34 | median ~1-2 m/s^3 |
| The car's own cruise | 1 (ee 8.9 min) | 9.5 m/s^3 |
| openpilot speed control | 3 (f5 12.2 min, f6 1.2 min, 00000100) | 18.6-20.4 m/s^3 |

The f5 stop, frame by frame: openpilot asked for -1.0 easing to -0.8 m/s^2 in the last second, and its request
reached the panda every frame (11 signed copies per tick). The car's own cruise, in the same frames, asked for even
less (-0.7 easing to -0.01). The car actually decelerated at -2.2 to -2.9 m/s^2 from about 4 mph down (confirmed from
the speed trace, not only the accel signal), then let go at once at 0 mph. So below ~4 mph the car brakes on its own,
whoever asks and whatever is asked; the driver's own pedal stops do not show it. The likely cause is the car's
low-speed stop or hold function on the cruise path (or the regen-to-friction hand-over there); which one is unknown.

What this changes:
- Phase 2 cannot be only a gentler request: openpilot's request was already gentle. It has to find what the car does
  below ~4 mph on the cruise path and work with it.
- Phase 0's drive gets one more test: at 3-5 mph on an empty road, compare a few final-approach requests (the current
  -0.8, a soft -0.3, and easing to 0 just above 4 mph and letting the car stop) and measure each with stops.py.
- f6's openpilot stop also moved 1.2 m after the car first reported standstill.

## First drive with openpilot speed control and the new turn/stop features (route 00000103, 2026-11-16)

The driver: "accelerated fast, overshot stop sign, missed turns, went dangerously close to cars". What the log shows:

| What the driver felt | Cause | Fix (2026-11-17) |
|---|---|---|
| A hard surge mid left turn (4:57) | The model asked for 460 deg of wheel at 5 mph; the EPS aborted near full lock (AbortFb=9, permanent LKAS fault); openpilot let go and the car's own cruise, still on, pulled away at +2.5 m/s^2 toward the 25 mph set speed | openpilot never asks for more than 400 deg of wheel (opendbc `LAT_ANGLE_CMD_MAX`) |
| Missed turns | Nothing slowed the car before a signaled turn: 20 mph held through 4 of 5 junctions; the turn assist (starts at 16 mph) passed each. Three times it had also picked the cross street 5-18 m ahead instead of the street 60-80 m on | The turn assist's armed junction reaches the planner (`carControlSP.mapTurn`): slow to the driver's junction speed 15 m before it; junctions too close to slow for (1.5 m/s^2) are skipped; look 9 s ahead |
| "Accelerated fast" | +1.0 m/s^2 asked after every turn and stop; the car delivered up to +1.44 | Ask at most 0.6 m/s^2 below 25 mph (`profile_long.comfort_accel_cap`), opening to 1.0 at 45 mph |
| Drove through a stop sign | The driver profile's coast time is 0 s, so the coast point was the braking point and every sign approached above ~16 mph was "taken on too close"; and the 20 mph approach cap equalled the cruising speed | Coast at least 1.5 s; give up only when stopping needs more than 1.2 m/s^2 (brake a little firmer when a little late); approach cap 25 mph |
| Close to cars | Not located in the log. The car has no radar (camera leads only); no lead was seen while moving. After the 1:36 turn the model asked for a gentle left (R ~30 m) after the driver took over | Ask the driver where; watch on the next drive |

## Phases

Each phase lists what is done at the desk (offline, tests and simulation) and what needs the car. Car tests go one
change per drive, first on an empty road or car park at low speed, with the driver's foot over the brake.

### Phase 0: see what the car does (1 session offline, 1-2 short drives)

Offline:
1. A speed-control handover trace, like the steering one added today: one log line when the swap starts or stops, and
   why (cruise state, gas, no matching copy, panda permission).
2. A way to see what the drivetrain received. Candidates: the car's own feedback signals (the VCU's ACC status,
   requested vs delivered torque or brake), or a panda counter of swapped frames reported in its health message. Pick
   the cheapest that answers "did our request reach the car".
3. Extend `stops.py` (the stop smoothness measure used today) into a drive-review script: per stop, the request, the
   real deceleration, the jerk over the last 0.5 s (the "head nod"), and what happened after standstill.

On the car (empty road, 15-25 mph):
1. Engage openpilot speed control, let it stop for a stopped car or a stop sign with a lead.
2. Three or four stops and pull-aways.

Done when: we know (a) whether braking requests are delivered, (b) the car's request-to-actual tracking, (c) what
happens at standstill and on resume, (d) gas override behaviour. Everything below depends on these answers.

### Phase 1: clean up (1 session offline, no drive)

1. Remove the dead paths left from the old designs: the panda's checks for our own 0x121/0x117/0x118 frames
   (no longer sent), the relay code, the 0x118 builder used only by tests, and the stale comments about a heartbeat.
2. Gate the swap on the cruise really being active (the cruise state), not on the frame's "valid" bit alone.
3. Tests for both; run the panda mutation tests again.

### Phase 2: smooth stops (2 sessions offline, 2-3 drives)

The head nod comes from the last half second: the brakes still bite as the car reaches 0, the nose dips and springs
back. A good human stop eases off the brake just before the car stops and then holds it.

Offline:
1. A stopping profile for the final approach:
   - plan the stop so the braking is at most about 1.0 m/s^2 for most of it (softer in the wet: 0.7-0.8, already in
     the wet-road table);
   - below about 3 mph, ease the braking down toward 0.3 m/s^2 so the car arrives at 0 with almost no deceleration
     (the "chauffeur stop");
   - limit the change in braking to about 0.5 m/s^3 in the last second;
   - once stopped, step up to the hold request gently.
2. Simulate it with the real longitudinal planner on the device (the planner's solver does not run on the Mac), and
   replay the recorded stops with drive_sim to compare request profiles.
3. Tests: the profile never asks for more than the limits, the last-second jerk stays under the target, and a lead car
   stopping short still gets full braking (comfort never overrides a real need to stop).

On the car: stops behind a lead and at stop signs; measure the final-0.5 s jerk. Target: under 1.0 m/s^3, against the
car's own cruise today (measured on the same script).

Depends on Phase 0: if the car's own system takes over the last metres or the hold, the profile has to work with
that; if it rolls, Phase 3 comes first.

### Phase 3: standstill and resume (1-2 sessions, 2 drives)

Depends on what Phase 0 shows:
- If the car's cruise holds and resumes on its own: report standstill so openpilot waits, resume with the car's
  resume button, and make the pull-away ramp gentle (the 1.0 m/s^3 rise limit already helps).
- If it rolls or creeps: keep a hold request while stopped and release it with a ramp. Other cars' experience (VW
  MEB, the closest EV): the hold is easy, the release is what faults.
- Never request the parking brake or emergency standstill; the panda already blocks those.

Done when: 10 stop-and-go cycles with no roll, no lurch, no fault.

### Phase 4: how hard and how fast (1 session, 2-3 drives)

The 1.0 m/s^2 cap and the jerk limits rest on one fault (000000c6). Find the car's real tolerance, as the Tesla port
did, by stepping it up one notch per drive on an empty road: acceleration 1.0 -> 1.2 -> 1.5, and watching the
request-to-actual mismatch and the "ACC ready" signal. Stop at the first warning, back off one notch. This also
decides whether the wet-road accel cap of 1.2 means anything (today it doesn't).

### Phase 5: the features that need speed control (2-3 sessions, several drives, one feature per drive)

All already built and running in shadow (logging what they would do); each becomes live one at a time:
1. Turns, slow and easy: slow to the driver's junction speed (default 10 mph) before a signalled turn. This is
   what makes the turn assist's arc comfortable; the steering changes of 4d336fe82 assume a slow car.
2. Stop sign assist (coast from 150 ft, stop with the Phase 2 profile).
3. Speed bump assist, map caution (no speeding up toward a traffic light), narrow streets.
4. Rain mode's speed and gap changes (after the heavy-rain rule is fixed: the wiper signal is on/off, not wipes per
   minute).
5. The driver profile: following gap, speed by limit, lead coasting.

### Phase 6: everyday use

- CPU: card ran 45% of its core with speed control on (f3/f5) before the batched signing fix; measure again on a
  highway drive, the heaviest case.
- 10 mixed drives with no cruise fault, no unexpected brake or acceleration, and the driver's brake/gas takeovers
  explained.
- The debug panda firmware stays a requirement; note it in the deploy skill.

## Order and effort

| Phase | Offline | Drives | Blocks |
|---|---|---|---|
| 0 Observe | 1 session | 1-2 short | everything |
| 1 Clean up | 1 session | 0 | - |
| 2 Smooth stops | 2 sessions | 2-3 | 0 |
| 3 Standstill | 1-2 sessions | 2 | 0, 2 |
| 4 Limits | 1 session | 2-3 | 0 |
| 5 Features | 2-3 sessions | one per feature | 2, 3 |
| 6 Everyday | - | ~10 | all |

Phase 1 can start right away; Phase 0's first drive is the next step that needs the car.

## Risks

- The car's cruise may fault mid-drive when its plausibility check disagrees with our request (000000c6). Mitigation:
  conservative limits, step up one notch per drive, the "ACC ready" watch already turns a drop into a clean
  disengage.
- A stop that eases off too early could roll toward the car ahead. Mitigation: the chauffeur stop only applies when
  there is room; a lead closing in always gets full braking; tests for exactly that.
- Software differences between Oceans (2.0/2.1/2.2 builds) may change the cruise's behaviour; findings hold for this
  car's build.
