# Tesla FSD (Supervised) v15: official driving capabilities, hardware, rules and availability (as of 26 Nov 2026)

Scope: driving behaviour only (Reality View visualization was researched elsewhere and is mentioned only where it touches driving). Every item carries its date. "Tesla claim" = stated by Tesla/Musk/Tesla staff, not independently verified. Release-note text is paraphrased from mirrors (Not a Tesla App reproduces Tesla's in-car notes).

## 1. What the v15 release notes say it does (and what v15 inherits from v14)

### Takeaway
v15.0.1 (software 2026.38.6.11, first released 21 Nov 2026 to Early Access, HW4 only) is a network upgrade with few new named manoeuvres. Its driving-relevant notes are: a new shared vision encoder (Tesla claim: 25x better "critical intervention rate"), learned navigation at trip start/end, parking chosen from personal history, better detection of small hazards (incl. potholes and deep standing water, which it now avoids), more natural speed handling (per profile, school zones, limits, weather), Speed Camera Slowdown, Bike Rack Mode, and FSD-driven Autoshift. Most of the manoeuvre list in the brief (unprotected turns, emergency vehicles, school buses, parking/unparking/reversing, speed profiles, detours, gates) was introduced in v14.x and is carried into v15. The v15 notes do not mention roundabouts, lane changes or merges, pedestrians or cyclists, night driving, or "Banish".

### Cited Findings
**v15.0.1 release notes (2026.38.6.11, 21 Nov 2026, HW4):**
- "v15 upgrades every part of the end-to-end driving network". This includes an upgraded vision encoder shared between FSD, Robotaxi and Optimus. Tesla says the encoder is trained on millions of internet videos for better generalization and data efficiency, and that it improves "the critical intervention rate by 25x". Tesla's X post words this as "25x better handling of safety critical cases". — [Not a Tesla App release notes](https://www.notateslaapp.com/software-updates/version/2026.38.6.11/release-notes); [Teslascope on X](https://x.com/teslascope/status/2124272702911594735); [Drive Tesla Canada, 21 Nov 2026](https://driveteslacanada.ca/news/tesla-fsd-v15-rollout/)
- Navigation: the start and end of trips now use an end-to-end/learned model trained on fleet route-preference data. Tesla says later v15 releases will extend this to the full route. — [Teslarati, 21 Nov 2026](https://www.teslarati.com/tesla-full-self-driving-v15-rollout-begins-full-release-notes-improvements/); [Drive Tesla Canada](https://driveteslacanada.ca/news/tesla-fsd-v15-rollout/)
- Parking: the parking location is chosen using the owner's personal parking history, and the map can show several parking options at a destination. — [Not a Tesla App](https://www.notateslaapp.com/software-updates/version/2026.38.6.11/release-notes)
- Perception: better detection of road signs, small animals, debris, chains, potholes, road details and deep standing water, which the car now avoids. — [Teslarati](https://www.teslarati.com/tesla-full-self-driving-v15-rollout-begins-full-release-notes-improvements/)
- Speed: "more natural" speed handling per driver profile, in school zones, around speed limits and in inclement weather. — [Not a Tesla App](https://www.notateslaapp.com/software-updates/version/2026.38.6.11/release-notes); [Teslarati](https://www.teslarati.com/tesla-full-self-driving-v15-rollout-begins-full-release-notes-improvements/)
- Autoshift is unified with FSD-based gear predictions, so the network chooses drive or reverse. — [Not a Tesla App](https://www.notateslaapp.com/software-updates/version/2026.38.6.11/release-notes)
- Speed Camera Slowdown (HW4): FSD slows for detected speed cameras. Pressing the accelerator or raising the speed profile cancels the slowdown. If the driver cancels twice at the same camera, the car saves that location and does not slow there again. Saved locations are managed under Controls > Service > Manage Saved Locations. — [Not a Tesla App](https://www.notateslaapp.com/software-updates/version/2026.38.6.11/release-notes)
- Bike Rack Mode (HW4): FSD and Autosteer can run with a bike rack or other hitch accessory fitted. In this mode FSD pulls over near the destination and does not reverse, and Autopark, Summon and the backup chimes and warnings are disabled. — [Not a Tesla App](https://www.notateslaapp.com/software-updates/version/2026.38.6.11/release-notes)
- Undocumented changes (found by Not a Tesla App): the disengagement feedback prompt was removed. Calm Mode and Map Mode can hide the FSD visuals. — [Not a Tesla App](https://www.notateslaapp.com/software-updates/version/2026.38.6.11/release-notes)
- Turn-signal sounds are quieter and fade after a few clicks. Upcoming turns are shown in the visualization. — [Teslarati](https://www.teslarati.com/tesla-full-self-driving-v15-rollout-begins-full-release-notes-improvements/)
- No independent measurement of the 25x figure existed as of 22 Nov 2026. Tesla has not published the baseline, miles per intervention, test conditions or its definition of "critical". The figure is not a crash rate or a comparison with human drivers. — [agentspulse summary, 22 Nov 2026](https://agentspulse.github.io/tutorials/tesla-fsd-v15/)

**Point releases:** only v15.0.1 is public as of 26 Nov 2026. TeslaFi showed it on about 82 cars (0.7% of its tracked fleet) at the time of the snapshot. — [Not a Tesla App](https://www.notateslaapp.com/software-updates/version/2026.38.6.11/release-notes)

**Pre-release promises for v15 (Tesla claims):**
- 11 Sep 2026: Ashok Elluswamy (Tesla VP of AI) promised "even earlier prediction of hazards, even faster reaction time" in v15. — [Not a Tesla App, 11 Sep 2026](https://www.notateslaapp.com/news/4679/tesla-ai-chief-talks-about-whats-coming-in-fsd-v15)
- Q2 2026 earnings call, as reported 20 Aug 2026: Elluswamy said Robotaxi was already running v15 with "seven major improvements" over v14. JPMorgan, after a meeting with Tesla investor relations, reported that about 40% of these were being tested on Robotaxi and that the step from v14 to v15 is about as large as the step from v13 to v14. — [Not a Tesla App, 20 Aug 2026](https://www.notateslaapp.com/news/4594/tesla-is-holding-back-model-y-robotaxis-reveals-new-fsd-v15-details)
- 9 Apr 2026: Musk said v15 would "far exceed human levels of safety, even in completely unsupervised and complex situations". Electrek pointed out that he made the same claim for v12 and v14. — [Electrek, 9 Apr 2026](https://electrek.co/2026/04/09/elon-musk-tesla-fsd-v15-safer-than-humans-same-claim-every-version/)

**v14 baseline carried into v15 (needed to understand v15):**
- v14.1.x (from Oct 2025): Arrival Options (parking lot, street, driveway, parking garage or curbside, saved per destination). Pulls over or yields for police cars, fire trucks and ambulances. Speed profiles SLOTH (below CHILL) and MAD MAX (above HURRY), with the profile having more effect on max speed. Navigation fed into the vision network so FSD can handle blocked roads and detours in real time. Better handling of static and dynamic gates, unprotected turns, lane changes, cut-ins and school buses. Better recovery from degraded operation. — [Not a Tesla App, 2025.32.8.16 / v14.1.4, 25 Oct 2025](https://www.notateslaapp.com/software-updates/version/2025.32.8.16/release-notes)
- v14.3.13 (2026.38.5.10, 20 Oct 2026, HW4): better reinforcement-learning training and vision encoder for rare, low-visibility scenes and traffic signs. About 20% faster reaction time from a rewritten AI compiler and runtime. Better response to emergency vehicles, school buses, right-of-way violators and other rare vehicles. Better traffic lights at complex junctions (compound lights, curved roads, stopping on yellow). Better handling of small animals and of objects leaning into the path. More decisive choice of parking spot and manoeuvring. Fewer unneeded lane biases and less minor tailgating. Recovers from temporary system degradation without the driver taking over. Summon max speed raised to 8 mph. Listed as "upcoming": reasoning across all behaviours, and pothole avoidance (pothole avoidance then shipped in v15). — [Not a Tesla App, 2026.38.5.10](https://www.notateslaapp.com/software-updates/version/2026.38.5.10/release-notes)
- v14.3 Lite (HW3) added parking, unparking and reversing, Arrival Options, speed profiles, and Start Self-Driving from Park (offered once the driver is seated and belted and the cabin camera is not blocked). From this the inference is that HW4 already had these functions in v14. — [Not a Tesla App, 2026.38.5.10](https://www.notateslaapp.com/software-updates/version/2026.38.5.10/release-notes)
- v14.3.9 (2026.27.6) added Automatic Collision Evasion (HW4). It turns FSD on to steer away from danger when the driver seems inattentive or FSD was disengaged by mistake, including during manual driving. — [Not a Tesla App, 2026.38.5.10](https://www.notateslaapp.com/software-updates/version/2026.38.5.10/release-notes); [Not a Tesla App, 11 Sep 2026](https://www.notateslaapp.com/news/4679/tesla-ai-chief-talks-about-whats-coming-in-fsd-v15)

### Inferences
- v15 is mainly a "better at everything" network release. It adds only three new driving behaviours: Speed Camera Slowdown, Bike Rack Mode (pull over instead of reversing), and avoidance of potholes and standing water. Everything else in the brief's list (unprotected lefts, emergency vehicles, school buses, park/unpark/reverse, speed profiles, detours) is inherited from v14.
- Learned navigation covers only the start and end of a trip in v15.0.1. Following navigation over the full route is not yet end to end (Tesla statement, 21 Nov 2026).

### Gaps
- No v15 note mentions roundabouts, lane changes, merges, pedestrians, cyclists, night driving or "Banish". I found no official statement on these for v15 beyond v14's general improvements.
- I found no v15.0.2 or later point release as of 26 Nov 2026.

## 2. Architecture and hardware (HW3 / HW4-AI4 / AI4+ / AI5)

### Takeaway
Tesla describes v15 as a single end-to-end network (camera input to driving output). It has a new vision encoder shared with Robotaxi and Optimus, and the driving model has about 10x more parameters than v14. v15 runs only on HW4 (AI4). HW3 stays on distilled "v14 Lite" builds. Musk says consumer cars will stay on AI4 and AI5 goes to Optimus and training. Reality View is a separate model that uses the same vision-encoder features.

### Cited Findings
- "v15 upgrades every part of the end-to-end driving network." The vision encoder is shared with Robotaxi and Optimus. — [Not a Tesla App, 21 Nov 2026](https://www.notateslaapp.com/software-updates/version/2026.38.6.11/release-notes)
- Parameter count: current FSD builds have about 1 billion parameters, and v15 has about 10 billion. Musk confirmed a "10x parameter upgrade" for v15 (exact wording not quoted). — [Not a Tesla App, 11 Sep 2026](https://www.notateslaapp.com/news/4679/tesla-ai-chief-talks-about-whats-coming-in-fsd-v15); [Not a Tesla App, 20 Aug 2026](https://www.notateslaapp.com/news/4594/tesla-is-holding-back-model-y-robotaxis-reveals-new-fsd-v15-details)
- Conflicting size figure: Teslarati says Reality View uses a diffusion model of about 1 billion parameters, separate from a driving model of about 3 billion parameters, and that both use the same vision-encoder ("UVE") features. This conflicts with the 10-billion figure above and is not reconciled. — [Teslarati, 21 Nov 2026](https://www.teslarati.com/tesla-full-self-driving-v15-rollout-begins-full-release-notes-improvements/)
- Reality View replaces the old separate model outputs with one end-to-end view that uses less compute. — [Drive Tesla Canada, 21 Nov 2026](https://driveteslacanada.ca/news/tesla-fsd-v15-rollout/)
- Hardware: v15 runs only on HW4 (AI4). HW3 cars get v14.3 Lite in software 2026.44.1 (rolled out from 17 Nov 2026). Tesla says Lite "distilled the intelligence from HW4 V14 into HW3". No HW3 build of v15 has been announced. — [agentspulse, 22 Nov 2026](https://agentspulse.github.io/tutorials/tesla-fsd-v15/); [Not a Tesla App, 11 Sep 2026](https://www.notateslaapp.com/news/4679/tesla-ai-chief-talks-about-whats-coming-in-fsd-v15)
- Musk on HW3 (Q1 2026 call): HW3 cars do not have the capacity for unsupervised driving, and Tesla will work on an upgrade plan. — [Teslarati](https://www.teslarati.com/tesla-full-self-driving-v15-rollout-begins-full-release-notes-improvements/)
- Musk, 22 Nov 2026: AI5 "is going to Optimus. Cars will remain at AI4. That's enough to solve superhuman driving." AI5 has taped out and is in pilot production, with volume production targeted for 2027. — [Teslarati](https://www.teslarati.com/tesla-full-self-driving-v15-rollout-begins-full-release-notes-improvements/)
- AI4+ (HW4.5) has about 10% more FLOPS and about 2x the memory of AI4. It is intended for larger Robotaxi models and longer context. Tesla says standard AI4 can run v15 and unsupervised FSD. It is not confirmed whether Cybercab uses AI4+. — [Not a Tesla App, 20 Aug 2026](https://www.notateslaapp.com/news/4594/tesla-is-holding-back-model-y-robotaxis-reveals-new-fsd-v15-details)
- Vehicles listed for 2026.38.6.11: Model S, 3, X, Y (Gen 1 and 2), Model Y L and Cybertruck, with the FSD v15 items marked HW4. — [Not a Tesla App](https://www.notateslaapp.com/software-updates/version/2026.38.6.11/release-notes)

### Inferences
- With v15 the HW3 and HW4 fleets split: HW4 runs a different model generation (v15), while HW3 runs v14 Lite.

### Gaps
- I found no v15 source that states which cameras are used or how many. I found nothing to suggest a change from the existing camera-only suite.
- The parameter count of the driving model is not settled: about 10B (Musk, via Not a Tesla App) against about 3B (Teslarati).

## 3. Supervision rules and unsupervised/Robotaxi variants

### Takeaway
v15 remains "Supervised". The driver must stay attentive and is watched by the cabin camera. v15 makes monitoring work better with hats and sunglasses and makes it less strict in the higher-speed profiles. The v14 strike system is unchanged as far as I found. Unsupervised driving exists only in Tesla's own Robotaxi service (8 cities in Texas and Florida without a monitor, per reports), which ran early v15 builds before customers got it.

### Cited Findings
- v15 driver monitoring: vision-based attention monitoring is "more available" with hats and sunglasses, and less strict in higher-speed profiles. — [Teslarati, 21 Nov 2026](https://www.teslarati.com/tesla-full-self-driving-v15-rollout-begins-full-release-notes-improvements/); [Not a Tesla App](https://www.notateslaapp.com/software-updates/version/2026.38.6.11/release-notes)
- v14.3.13 (20 Oct 2026): monitoring made more sensitive, with better eye-gaze tracking, eyewear handling and performance in variable lighting. — [Not a Tesla App](https://www.notateslaapp.com/software-updates/version/2026.38.5.10/release-notes)
- Strike system (v14.1.x, Oct 2025, undocumented): one strike is forgiven every 3.5 days (was 7). Five strikes on a car with a cabin camera, or three without, temporarily suspend FSD. Drowsiness alerts now suggest turning FSD on. — [Not a Tesla App, 2025.32.8.16](https://www.notateslaapp.com/software-updates/version/2025.32.8.16/release-notes)
- Start Self-Driving from Park requires the driver to be seated and belted with the cabin camera not blocked. — [Not a Tesla App, 2026.38.5.10](https://www.notateslaapp.com/software-updates/version/2026.38.5.10/release-notes)
- Europe (RDW, reported 16 Nov 2026): the European build has more restrictions: earlier warnings, stricter misuse checks, and hands on the wheel above 130 km/h. — [fsdtracker.eu](https://fsdtracker.eu/)
- Robotaxi: by Nov 2026 it runs without a monitor in Austin, Dallas, Fort Worth (from 2 Nov), Houston, San Antonio (from 9 Nov), Miami, Orlando and Tampa. In the SF Bay Area a safety supervisor rides along. Service became 24 hours a day from 12 Nov. Tesla did not confirm that Fort Worth and San Antonio run without monitors. — [Wikipedia: Tesla Robotaxi](https://en.wikipedia.org/wiki/Tesla_Robotaxi); [Not a Tesla App, Fort Worth](https://www.notateslaapp.com/news/4932/tesla-expands-robotaxi-service-launches-unsupervised-rides-in-fort-worth); [GCN, San Antonio](https://gcn.com/tesla-launches-unsupervised-robotaxi-service-san/23199)
- Robotaxi was running early v15 builds before the customer release. Tesla is holding back the addition of Model Y Robotaxis until Cybercab is ready. — [Not a Tesla App, 20 Aug 2026](https://www.notateslaapp.com/news/4594/tesla-is-holding-back-model-y-robotaxis-reveals-new-fsd-v15-details)
- A tracker lists unsupervised operation for Canadian owners as "expected Q1 2027". This is an estimate, not a Tesla date. — [CnEVPost / trackers via search](https://notanfsdtracker.com/)

### Inferences
- The release notes describe consumer v15 and the Robotaxi software as sharing a vision encoder. The differences are the supervision, the geofence and fleet operation, not a different driving brain.

### Gaps
- I did not check Tesla's own FSD page or owner's manual wording for v15. I found no official v15 statement on hands-off rules, phone use or a strike-count change.

## 4. Availability (US, Canada, China, EU/UK, Australia)

### Takeaway
As of 26 Nov 2026, v15 is US-only, on HW4, for Early Access first. Musk said wide release would be in December 2026, possibly with the Holiday Update. Other FSD (Supervised) markets (Canada, Mexico, Puerto Rico, China, Australia, NZ, South Korea, and EU countries under the Dutch RDW approval) remain on v14 builds. The UK has no approval.

### Cited Findings
- v15.0.1 began rolling out on 21 Nov 2026 to Early Access members in the US, on HW4. Musk said wide release would come "next month" (December). — [Teslarati](https://www.teslarati.com/tesla-full-self-driving-v15-rollout-begins-full-release-notes-improvements/); [agentspulse](https://agentspulse.github.io/tutorials/tesla-fsd-v15/)
- European cars remain on a v14 build approved by the regulator. v15 is US-only for now. — [Gumtree News](https://www.gumtree.com/news/cars/tesla-fsd-v15-us-only-europe-uk-approval)
- Tesla's list on 20 May 2026 of FSD Supervised markets: US, Canada, Mexico, Puerto Rico, China, Australia, New Zealand, South Korea, Netherlands, Lithuania. — [Tesla on X](https://x.com/Tesla/status/2057226337010745348); [CnEVPost, 21 May 2026](https://cnevpost.com/2026/05/21/tesla-says-fsd-supervised-available-in-china/)
- EU: on 10 Apr 2026 the RDW (Netherlands) gave the first European approval, under UN R171, based on about 1.6 million km of European testing. It covers HW4 only. — [Electrek, 10 Apr 2026](https://electrek.co/2026/04/10/tesla-fsd-supervised-approved-netherlands-rdw-europe/); [electrive, 13 Apr 2026](https://www.electrive.com/2026/04/13/tesla-secures-first-european-approval-for-fsd-supervised-driver-assistance-system/)
- As of Nov 2026, 12 EU countries have approved or recognised it: NL, LT, EE, BE, DK, SI, CZ, HR, SK, LV, FI (reported 11 Nov) and IT (20 Nov). That is 28.6% of the EU population. EU-wide approval is projected for Q2 2027 (tracker projection). The European name became "Tesla Assisted Driving" in Oct 2026 after Germany objected to the old name. — [fsdtracker.eu](https://fsdtracker.eu/); [eleport](https://eleport.com/tesla-fsd-europe/)
- UK: not approved. On 9 Oct 2026 the VCA said it had not done any type-approval work for the system. The RDW approval does not carry over to Great Britain. — [evcompared](https://www.evcompared.co.uk/news/tesla-fsd-uk-automated-vehicles-act/); [notanfsdtracker UK](https://notanfsdtracker.com/united-kingdom)
- China: Tesla listed China as of May 2026, but full approval is still pending. Reports describe a limited, localized v14 build. On the Q3 2026 call Tesla said the China service is supervised and wider approval has not been secured. — [CnEVPost](https://cnevpost.com/2026/05/21/tesla-says-fsd-supervised-available-in-china/); [Drive Tesla Canada](https://driveteslacanada.ca/news/tesla-fsd-china-not-fully-approved/)
- Australia: FSD Supervised is live on HW4, with v14 from June 2026 (tracker). — [notanfsdtracker](https://notanfsdtracker.com/)

### Inferences
- A comparison with a European or Chinese car should use v14 behaviour. v15 is the US HW4 state of the art.

### Gaps
- I found no official date for v15 in Canada or any other market outside the US.
- I did not confirm the TCMV EU-wide vote (said to be scheduled for 6 Oct 2026) or its result.
- Country lists come from trackers that disagree with each other.
