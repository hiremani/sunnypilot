# Tesla Reality View (FSD v15)

Research date: 2026-11-23. The feature exists under exactly this name. It began shipping 2026-11-21, so all coverage is 2 days old and consists of first impressions. I could not open the primary X posts directly (X does not serve them to the fetch tool); every Tesla/Musk/Elluswamy quote below is taken from secondary outlets that quote or link the posts. Primary post URLs are listed so they can be checked by hand.

Confidence labels used below: **[Tesla]** = release notes or Tesla/Musk/Elluswamy statement as quoted by an outlet; **[Reported]** = outlet or owner observation; **[Speculation]** = third-party inference.

Primary posts (not fetched, links as given by EVwire):
- Tesla announcement: https://x.com/Tesla/status/2124222447855743019
- Elluswamy on Reality View: https://x.com/aelluswamy/status/2124229978560835880
- Musk technical explanation (world model / mean-flow): https://x.com/elonmusk/status/2124458557525397780
- Musk "GTA irl": https://x.com/elonmusk/status/2124340719397781824
- Release notes parts 1 and 2: https://x.com/TeslaSI/status/2124229673056113100 , https://x.com/TeslaSI/status/2124229676055040007

## What exactly does Reality View show on the screen (content, view angles, interaction, day/night, range)?

### Takeaway
Reality View is a generated video, not a 3D scene of models: a live, near-photoreal "drone shot" of the car and its real surroundings from a virtual camera above and behind the vehicle, with planned-turn chevrons and parking pins drawn in. It replaces the old gray-car / lane-line visualization while FSD is running; camera pan/zoom, night behaviour and range are not documented anywhere I found.

### Cited Findings
- [Tesla] Release-note wording: "Tesla Reality View New: real-time generative driving visualization that continuously streams a view from an arbitrary virtual camera situated above your vehicle." — [Teslarati](https://www.teslarati.com/tesla-full-self-driving-v15-first-hand-look-at-reality-view-new-features/); same wording on the [Not a Tesla App release notes page](https://www.notateslaapp.com/software-updates/version/2026.38.6.11/release-notes)
- [Tesla] The feature "adds a live visualization of the drive that lets you see & control what FSD is planning to do." — [Teslarati](https://www.teslarati.com/tesla-full-self-driving-v15-first-hand-look-at-reality-view-new-features/)
- [Tesla] It replaces "all of the explicit model signals" with a single end-to-end visualization that uses less total compute. Teslarati notes this bullet uses the name "Tesla Augmented Reality" and does not explain the naming difference. — [Teslarati](https://www.teslarati.com/tesla-full-self-driving-v15-first-hand-look-at-reality-view-new-features/); [EVwire](https://evwire.com/p/tesla-fsd-v15-rollout-reality-view-release-notes)
- [Tesla] Layout options: fullscreen visualization, maps, split view, or a "quiet mode". Upcoming navigation turns are shown inside the visualization. Turn-signal sounds are quieter and fade out after a few clicks. — [Not a Tesla App release notes](https://www.notateslaapp.com/software-updates/version/2026.38.6.11/release-notes); [EVwire](https://evwire.com/p/tesla-fsd-v15-rollout-reality-view-release-notes)
- [Tesla] Overlays: arrows/chevrons for planned turns and pins for the spot FSD intends to park in. Musk: it "re-renders reality with chevrons in the sky for turns." — [EVwire](https://evwire.com/p/tesla-fsd-v15-rollout-reality-view-release-notes); [Basenor](https://www.basenor.com/blogs/news/tesla-fsd-v15-reality-view-what-the-rendering-actually-does)
- [Reported] Scene content: buildings, sidewalks and people, "almost like a video game". — [Not a Tesla App](https://www.notateslaapp.com/news/5065/tesla-fsd-v15-rolls-out-with-incredible-new-reality-view-and-25x-better-safety-video)
- [Reported] Teslarati's first-hand reviewer saw grass, trees, buildings, "all of nature" and signage; it was hard to tell from a drone camera at first glance, and signage was what gave away that the image is generated. — [Teslarati](https://www.teslarati.com/tesla-full-self-driving-v15-first-hand-look-at-reality-view-new-features/)
- [Reported] View angle: virtual camera above the vehicle like a drone. In Reverse (backing out of a space) the view moves to the front of the car and returns when shifted to Drive. — [Teslarati](https://www.teslarati.com/tesla-full-self-driving-v15-first-hand-look-at-reality-view-new-features/)
- [Reported] A rear-facing use: owner Roger Kappler (@roger_kappler) posted that "backup cameras are a thing of the past". — [Not a Tesla App](https://www.notateslaapp.com/news/5065/tesla-fsd-v15-rolls-out-with-incredible-new-reality-view-and-25x-better-safety-video)
- [Reported, second-hand] Interaction: tapping the visualization cycles layout modes, and tapping a point on the view while FSD is active tells the car to route toward that point. Reality View turns on automatically when FSD (Supervised) is running. Source is a third-party accessory blog summarising X posts. — [Basenor how-to](https://www.basenor.com/blogs/news/tesla-update-15-0-1); tap-to-drive also described by [Unite.AI](https://www.unite.ai/tesla-rolls-out-fsd-v15-with-reality-view-and-vision-upgrades/)
- [Reported] Wet roads: one owner reported that reflections on wet roads appear in the view. — [Yahoo Autos search summary](https://autos.yahoo.com/ev-and-future-tech/articles/tesla-rolls-reality-mode-fsd-132045563.html) (page itself returned 403; only the search snippet was seen)
- [Reported] Curb edges are easier to see (@SawyerMerritt) and road-surface details such as filled-in cracks are rendered. — [Basenor how-to](https://www.basenor.com/blogs/news/tesla-update-15-0-1)

### Inferences
- "Arbitrary virtual camera" in the release note implies the model is conditioned on a camera pose, so the pose can change (the Drive/Reverse switch confirms at least two poses). Nothing found shows a user-controlled free pan/zoom.
- Because the picture is generated from camera features and not drawn from object lists, everything the cameras see can appear (vegetation, buildings, reflections, cracks), which the old visualization never showed.
- Other vehicles and pedestrians are in the picture as generated video content, not as tracked 3D models; no source says they are highlighted or labelled.

### Gaps
- No source describes night, heavy rain, snow or glare behaviour. Basenor explicitly says low-light is "the real test" and untested.
- No source gives the visible range around the car, the camera height/pitch, or the field of view.
- No source confirms user pan/zoom/rotate. The pre-v15 setting "Visualization Camera Zoom" (2026.26.6.5) exists, but nobody says whether it applies to Reality View.
- Not documented: what the screen shows when FSD is not engaged (manual driving, parked).
- The tap-to-drive claim is not in the release-note text I could see beyond "see & control"; treat as reported, not confirmed.

## When and where was it announced or released (versions, dates, release notes, vehicles/hardware, regions)?

### Takeaway
Reality View shipped with FSD (Supervised) v15.0.1 in software 2026.38.6.11 on 2026-11-21 to Early Access owners, on HW4/AI4 only; wide release is promised by Musk for December 2026 with the Holiday Update. HW3 is excluded, and reports limit the first wave to the US and Canada.

### Cited Findings
- [Tesla] Rollout started Saturday 2026-11-21 to Early Access Program members: FSD (Supervised) v15.0.1, software update 2026.38.6.11. — [EVwire](https://evwire.com/p/tesla-fsd-v15-rollout-reality-view-release-notes); [Not a Tesla App release notes](https://www.notateslaapp.com/software-updates/version/2026.38.6.11/release-notes); [Tesla North](https://teslanorth.com/2026/11/21/tesla-fsd-v15-first-rollout-reality-view/)
- [Tesla] Musk said v15 goes to wide release "next month" (December 2026); outlets tie this to the 2026 Holiday Update. This is a Musk statement, not a formal Tesla date. — [EVwire](https://evwire.com/p/tesla-fsd-v15-rollout-reality-view-release-notes); [Teslarati](https://www.teslarati.com/tesla-full-self-driving-v15-first-hand-look-at-reality-view-new-features/)
- [Tesla] Elluswamy: "V15 debuts the Tesla Reality View." and "It's not there just because it looks cool." — [EVwire](https://evwire.com/p/tesla-fsd-v15-rollout-reality-view-release-notes)
- [Reported] Hardware: Not a Tesla App's release-notes page lists FSD v15.0.1 and Reality View as HW4, and tags Reality View for "New Y". The software build itself lists Model S, 3, X, Y, Model Y L and Cybertruck. — [Not a Tesla App release notes](https://www.notateslaapp.com/software-updates/version/2026.38.6.11/release-notes)
- [Reported] First wave appears limited to the redesigned Model Y / late-2025 and 2026+ cars in the U.S. and Canada; 0.1% of the tracked fleet at capture. — [Not a Tesla App](https://www.notateslaapp.com/news/5065/tesla-fsd-v15-rolls-out-with-incredible-new-reality-view-and-25x-better-safety-video)
- [Reported] HW3 cars are not eligible for v15 or a v15 Lite; they stay on the v14 Lite branch. EVwire words this more cautiously: v15 has only been seen on AI4 and Tesla has not said whether HW3 gets it. — [Not a Tesla App](https://www.notateslaapp.com/news/5065/tesla-fsd-v15-rolls-out-with-incredible-new-reality-view-and-25x-better-safety-video); [EVwire](https://evwire.com/p/tesla-fsd-v15-rollout-reality-view-release-notes)
- [Reported] Not available in Europe; Tesla Owners UK says there is no news of a UK launch. — [Not a Tesla Killer](https://notateslakiller.com/en/tesla-reality-view-fsd-v15-live-generated-screen-half-chip/); [Tesla Owners UK on X](https://x.com/TeslaOwnersUK/status/2124258224371077173)
- [Tesla] Musk said customer cars "will remain at AI4"; AI5 goes to Optimus. So AI5 is not a Reality View platform for cars at present. — [Electrek](https://electrek.co/2026/11/23/tesla-fsd-v15-ai4-last-shot-compute-limit/)
- [Reported] Other v15 items in the same notes: shared vision encoder trained on internet video with a Tesla-claimed 25x better critical-intervention rate, learned navigation and parking selection, speed-camera slowdown, Bike Rack Mode. The 25x baseline is unpublished. — [EVwire](https://evwire.com/p/tesla-fsd-v15-rollout-reality-view-release-notes); [Not a Tesla Killer](https://notateslakiller.com/en/tesla-reality-view-fsd-v15-live-generated-screen-half-chip/)
- [Reported] Nothing with this name appeared in v14.x. The 2026 visualization changes before v15 were: move to Unreal Engine with better lighting (2026.14), amber turn signals (2026.14.1), and a Visualization Camera Zoom setting (2026.26.6.5 with FSD 14.3.8, 2026-08-26). — [Not a Tesla App 2026.14](https://www.notateslaapp.com/software-updates/version/2026.14/release-notes); [Not a Tesla App 2026.26.6.5](https://www.notateslaapp.com/index.php/software-updates/version/2026.26.6.5/release-notes)

### Inferences
- Outlets disagree slightly on naming: Yahoo's headline says "Reality Mode", and one release-note bullet reportedly says "Tesla Augmented Reality". "Tesla Reality View" is the name used by Tesla, Elluswamy and the release-note heading.
- The eligibility picture is "AI4 only, early wave skewed to the newest Model Y"; whether older AI4 cars (2023-2024 S/X/Y) get it in the first wave is not established.

### Gaps
- I did not see the verbatim full release notes (aggregators limited quoting); the wording above is the portion outlets quoted.
- No official Tesla statement on regions, on AI4 vs AI4.5 differences, or on Cybertruck / S / X timing.

## What is known or credibly inferred about how it works (inputs, networks, compute, frame rate, latency)?

### Takeaway
Per Musk, it is real-time AI video generation with no game engine: a roughly 1-billion-parameter conditional mean-flow / diffusion-transformer model, distilled from Tesla's large "pixel-perfect" world model, conditioned on the same vision-encoder features that drive the car, producing 8-frame chunks at 36 fps on half of one AI4 TRIP accelerator. There is no confirmed Gaussian splatting or explicit 3D reconstruction; that idea comes from an owner's reverse-engineering attempt and from one outlet's description.

### Cited Findings
- [Tesla/Musk] "This is real-time AI video generation (no gaming engine!) using just half of one AI4 TRIP accelerator." — [Electrek](https://electrek.co/2026/11/23/tesla-fsd-v15-ai4-last-shot-compute-limit/); [Teslarati](https://www.teslarati.com/tesla-full-self-driving-v15-first-hand-look-at-reality-view-new-features/)
- [Tesla/Musk] "We've had a ~pixel-perfect world model since last year, which is critical for synthetic data generation. The (much) smaller version of that in the car is a conditional mean-flow model, based on the recent sequence of Tesla Universal Vision Encoder feat…" (post title truncated in search result). — [Musk on X](https://x.com/elonmusk/status/2124458557525397780)
- [Tesla/Musk] Model size about 1 billion parameters; it uses the same vision encoder features that feed "the 3B encoder model that produces the driving actions." — [Electrek](https://electrek.co/2026/11/23/tesla-fsd-v15-ai4-last-shot-compute-limit/)
- [Tesla/Musk] Output is a small video chunk: 8 images at 36 fps at the resolution of the UI screen. The work is done by half a TRIP engine on every UVE frame at 18 Hz, spread over 4 cycles. — [Not a Tesla Killer](https://notateslakiller.com/en/tesla-reality-view-fsd-v15-live-generated-screen-half-chip/); [Not a Tesla App](https://www.notateslaapp.com/news/5068/how-teslas-fsd-v15-reality-view-actually-works-video)
- [Tesla/Musk, as paraphrased] Temporal consistency: each chunk starts from the last generated frame plus noise, which is denoised into the next frames; the last frame of a chunk seeds the next chunk. — [Not a Tesla App](https://www.notateslaapp.com/news/5068/how-teslas-fsd-v15-reality-view-actually-works-video); [Not a Tesla Killer](https://notateslakiller.com/en/tesla-reality-view-fsd-v15-live-generated-screen-half-chip/)
- [Tesla/Musk] Compute placement: after moving a decoder into SRAM, "we only have half a TRIP out of three on SoC B, for this reality visualizer, so spent the compute as best we could." TRIP "basically only handles 8-bit int and has to run softmax in emulation mode in 28 steps." — [Electrek](https://electrek.co/2026/11/23/tesla-fsd-v15-ai4-last-shot-compute-limit/)
- [Tesla/Musk] Latency: "Currently, the latency is a bit high due to compute constraints"; Tesla has "ways to interleave newer UVE features along the diffusion transformer layers to cut roughly 35% of the latency." This is planned, not shipped. No absolute latency number was given. — [Teslarati](https://www.teslarati.com/tesla-full-self-driving-v15-first-hand-look-at-reality-view-new-features/); [Not a Tesla Killer](https://notateslakiller.com/en/tesla-reality-view-fsd-v15-live-generated-screen-half-chip/)
- [Tesla] Release notes say the single end-to-end visualization uses less total compute than the explicit signals it replaces; Elluswamy is reported to have said the same. — [EVwire](https://evwire.com/p/tesla-fsd-v15-rollout-reality-view-release-notes); [Not a Tesla App](https://www.notateslaapp.com/news/5065/tesla-fsd-v15-rolls-out-with-incredible-new-reality-view-and-25x-better-safety-video)
- [Tesla] The v15 vision encoder ("Tesla Universal Vision Encoder", UVE) is shared by FSD, Robotaxi and Optimus and trained on internet video. Tesla silicon engineer Dilreet Raju said it was co-designed "to squeeze out the last few percents of MFU from AI4." — [EVwire](https://evwire.com/p/tesla-fsd-v15-rollout-reality-view-release-notes); [Electrek](https://electrek.co/2026/11/23/tesla-fsd-v15-ai4-last-shot-compute-limit/)
- [Reported] Electrek: SoC B runs the decoder and Reality View rather than mirroring SoC A, and Electrek estimates AI4 at roughly 100-150 TOPS across both chips (Electrek's estimate, not Tesla's). — [Electrek](https://electrek.co/2026/11/23/tesla-fsd-v15-ai4-last-shot-compute-limit/)
- [Tesla/Musk] AI5 would be "roughly 12X more compute", but cars stay on AI4. — [Electrek](https://electrek.co/2026/11/23/tesla-fsd-v15-ai4-last-shot-compute-limit/)
- [Speculation/Reported] Not a Tesla App's first article described the method as stitching camera feeds, mapping each pixel onto a 3D stage by position, then filling gaps with generative AI. This description is the outlet's, and it does not match Musk's later statement that the model generates video directly from encoder features. — [Not a Tesla App](https://www.notateslaapp.com/news/5065/tesla-fsd-v15-rolls-out-with-incredible-new-reality-view-and-25x-better-safety-video); contradicted in emphasis by [Musk on X](https://x.com/elonmusk/status/2124458557525397780)
- [Speculation] YouTuber Ryan ("vogel"), who does not have v15, rebuilt a similar view from about 13 minutes of footage from a Model 3's six recorded cameras: calibrated camera poses from shared features, found timing offsets up to 8.7 ms, estimated depth, built Gaussian splats, refined for about 3 hours offline, removed moving cars and replaced them with clean 3D models. Pure diffusion "failed" for him. He believes Tesla uses 3D plus diffusion for "the last 10%". Open source on GitHub. This is an offline reconstruction, not evidence of Tesla's method. — [Not a Tesla App](https://www.notateslaapp.com/news/5068/how-teslas-fsd-v15-reality-view-actually-works-video)

### Inferences
- Timing arithmetic from Musk's numbers: 4 cycles at 18 Hz is about 0.22 s of compute per chunk, and 8 frames at 36 fps is about 0.22 s of video. Generation therefore just keeps pace with playback; the displayed picture must trail the world by at least one chunk (roughly 0.2 s) plus encoder and display time. This is derived, not measured (Not a Tesla Killer makes the same derivation).
- "Mean-flow" implies a one-step or few-step generative sampler rather than a many-step diffusion; that is what makes 36 fps feasible on an 8-bit integer accelerator.
- The inputs are the camera-derived encoder features only (Tesla vehicles on AI4 are camera-only), plus a virtual camera pose and the planner's intent for chevrons and pins. No source mentions radar, lidar, maps or an occupancy network as an input.
- Because the view branches off the driving model's own encoder, the marginal cost is the 1B generator only; an "equivalent" on other hardware would need both a strong multi-camera encoder and a real-time video generator.

### Gaps
- No teardown by greentheonly or any firmware-level analysis was found.
- No Tesla patent or talk specific to Reality View was found; earlier occupancy-network and world-model talks were not re-researched here.
- Screen resolution of the generated video, the exact chunk overlap, the measured end-to-end latency, and the training recipe are not public.
- Whether chevrons and pins are generated by the model or composited by the UI is not stated.
- How the virtual camera pose is supplied to the model is not stated.

## How does it differ from the previous FSD visualization and from competitors' visualizations?

### Takeaway
The old Tesla visualization drew detected objects (gray cars, lane lines, lights) as 3D assets in a game engine, so the screen was a readout of detections; Reality View is generated video, so it looks real but no longer tells you what FSD has explicitly detected. I found no sourced comparison with Rivian, Waymo, Xpeng, NIO or Mercedes.

### Cited Findings
- [Reported] Old: gray cars and lane lines directly showed what FSD detected; a red light or a car on screen meant FSD had seen it. New: a rendered view where flat features such as traffic lights and crosswalks cannot be told apart from plain video content. Tesla is "clearly moving away from showing exactly what FSD sees." — [Not a Tesla App](https://www.notateslaapp.com/news/5068/how-teslas-fsd-v15-reality-view-actually-works-video)
- [Reported] In 2026.14 Tesla moved the visualization to Unreal Engine with improved lighting (brake lights reflecting on the ground). Reality View drops the game engine entirely ("no gaming engine!"). — [Not a Tesla App 2026.14](https://www.notateslaapp.com/software-updates/version/2026.14/release-notes); [Electrek](https://electrek.co/2026/11/23/tesla-fsd-v15-ai4-last-shot-compute-limit/)
- [Tesla] Release notes: replaces "all of the explicit model signals" with one end-to-end visualization using less total compute. — [Teslarati](https://www.teslarati.com/tesla-full-self-driving-v15-first-hand-look-at-reality-view-new-features/)
- [Reported] An earlier related concept: an "augmented reality view" found in FSD beta code by @greentheonly years ago, overlaying perception on camera video. — [Teslarati](https://www.teslarati.com/tesla-fsd-augmented-reality-video)

### Inferences
- Under the old system the visualization was a trust and debugging tool for supervision. Under Reality View only the overlays (chevrons, pins) carry planner intent; the scene itself carries no detection semantics.
- For an "equivalent", the defining properties are: (1) camera-derived photoreal surround from a virtual third-person viewpoint, (2) live at video rate, (3) planner intent overlaid, (4) no hand-built 3D assets.

### Gaps
- No source compares Reality View with Rivian, Waymo's rider screen, Xpeng/NIO SR views or Mercedes. That comparison is unresearched here and must come from another researcher or from general knowledge, flagged as such.

## Reported limitations, user reception, and credible teardown/analysis

### Takeaway
Early reception is enthusiastic about the look and critical of lag, texture instability and "messiness"; the main substantive criticism is that a generated picture is not evidence of perception accuracy and says nothing about driving safety. There is no independent technical teardown yet.

### Cited Findings
- [Reported] Teslarati first-hand review: "incredibly detailed"; textures shift as the car moves, similar to early AI-generated video; input lag is noticeable but not a dealbreaker; "very, very good for a first attempt"; reviewer expects near-realistic quality in about eight months (reviewer's guess). — [Teslarati](https://www.teslarati.com/tesla-full-self-driving-v15-first-hand-look-at-reality-view-new-features/)
- [Reported] Electrek (Fred Lambert): Reality View looks good but is visibly laggy; early clips focus on UI features that say little about miles between critical interventions; v15 leaves almost no AI4 compute headroom. — [Electrek](https://electrek.co/2026/11/23/tesla-fsd-v15-ai4-last-shot-compute-limit/)
- [Reported] @wholemars: it can look "messy" compared with the older clean vector-style visualization. — [Basenor how-to](https://www.basenor.com/blogs/news/tesla-update-15-0-1)
- [Reported] Positive owner reactions: @niccruzpatane, busy downtown clip, "This is absolutely INSANE."; Musk: "FSD is GTA irl." — [Not a Tesla App](https://www.notateslaapp.com/news/5065/tesla-fsd-v15-rolls-out-with-incredible-new-reality-view-and-25x-better-safety-video); [EVwire](https://evwire.com/p/tesla-fsd-v15-rollout-reality-view-release-notes)
- [Reported] Caution that the display is "not proof of verified perception accuracy" and does not change FSD's supervised status. — [Yahoo Autos search summary](https://autos.yahoo.com/ev-and-future-tech/articles/tesla-rolls-reality-mode-fsd-132045563.html) (403 on fetch; snippet only); [Not a Tesla Killer](https://notateslakiller.com/en/tesla-reality-view-fsd-v15-live-generated-screen-half-chip/)
- [Reported] Generated signage is wrong or garbled enough to reveal the image is synthetic. — [Teslarati](https://www.teslarati.com/tesla-full-self-driving-v15-first-hand-look-at-reality-view-new-features/)
- [Reported] Reflections shown are just camera-image content and may not correspond to anything the car understands. — [Not a Tesla App](https://www.notateslaapp.com/news/5068/how-teslas-fsd-v15-reality-view-actually-works-video)

### Inferences
- Known weak points for an equivalent to match or beat: latency (about a chunk or more), temporal texture stability, legible text/signage, and the loss of an honest "what the system detected" readout.
- Hallucination risk is inherent: a generative view can draw plausible content that is not there, and nobody has yet published a systematic test of that for Reality View.

### Gaps
- No systematic owner data on hallucinated objects, night or bad-weather quality, or measured latency; the feature is 2 days old and at about 0.1% of the tracked fleet.
- No greentheonly analysis, patent, or Tesla AI talk on Reality View was found.
- Several sources are low-authority blogs (Basenor, Not a Tesla Killer, Unite.AI) relaying X posts; their technical claims match Musk's quoted posts but were not verified against the originals.
