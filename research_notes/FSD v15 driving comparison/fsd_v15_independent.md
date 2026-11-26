# Tesla FSD (Supervised) v15: independent performance evidence (as of 26 Nov 2026)

Context for the report writer: FSD v15 (v15.0.1, software build 2026.38.6.11 per the Not a Tesla App release-notes link) started rolling out to early-access owners on **21 Nov 2026**, five days before these notes. Wide release is promised "next month" with the Holiday Update. **No independent v15 dataset exists yet.** Almost all hard data below is for v14.x (Oct 2025 to Nov 2026) or older, and is labelled as such. Treat v14.3.x as the best proxy baseline for v15 behaviour until v15 data accumulates.

## 1. Intervention / disengagement data (v15 vs v14 vs v13)

### Takeaway
No independent v15 intervention data exists yet. Tesla's only v15 figure is a "25x" improvement in critical interventions with no baseline. On the crowdsourced tracker, v14.x reached roughly 1,600 to 1,950 total miles (about 700 to 850 city miles) per critical disengagement by mid-2026, against 444 total / 217 city miles for v13.2.x. That is about a 4x gain. The v14.1 peak was a small-sample outlier.

### Cited Findings
- v15 rollout started 21 Nov 2026. Tesla claims an upgraded vision encoder "improves the critical intervention rate by 25x". Tesla gives only the multiplier, not the underlying rate. Electrek (Fred Lambert, 21 Nov 2026) notes Tesla still does not publish disengagement data. — [Electrek](https://electrek.co/2026/11/21/tesla-fsd-v15-rollout-25x-safety-reality-view/)
- Electrek's own extrapolation (not a measurement): v14 was at "a couple thousand miles" between critical interventions in 2026. If the 25x claim held, v15 would be at about 50,000 miles, against the roughly 10,000 miles Electrek thinks a limited unsupervised service needs. — [Electrek](https://electrek.co/2026/11/21/tesla-fsd-v15-rollout-25x-safety-reality-view/)
- v15 removed the "Interventions menu" (the prompt that asks drivers to categorise a takeover). Reviewers welcomed this. Inference for the writer: it also removes a structured source of takeover-reason data. — [EVWire, 23 Nov 2026](https://evwire.com/p/tesla-fsd-v15-first-reviews); [Electrek](https://electrek.co/2026/11/21/tesla-fsd-v15-rollout-25x-safety-reality-view/)
- Crowdsourced FSD Community Tracker data, scraped and analysed by Dominik Roth (published 1 Jun 2026; 387 testers recovered; monthly series Feb 2022 to May 2026). Miles per critical disengagement, total / city:
  - 12.3.x: 177 / 121
  - 12.5.x: 273 / 129
  - 13.2.x: 444 / 217
  - 14.1.x: 9,263 / 4,109 (early access only, tiny sample, author calls it "real but not reliable")
  - 14.2.x: 1,642 / 696
  - 14.3.x: 1,750 / 853
  - Current v14 aggregate (June 2026): about 1,950
  - Share of drives with no critical disengagement: 96.5% (v13.2) vs 99.0% (v14.3). Highway is not split out. Caveats: voluntary, enthusiast-skewed North American testers; 76% of v14 tester records have zero critical disengagements, so the metric is unstable; v13→v14 paired comparison rests on only 17 testers. — [Dominik Roth blog](https://blog.dominik-roth.eu/posts/fsd-14.html)
- Early v14 crowdsourced figure: about 732 miles between critical disengagements, from about 4,000 miles of data (Electrek, late 2025). — [Electrek via search](https://electrek.co/?p=441026); restated in [Electrek v15 article](https://electrek.co/2026/11/21/tesla-fsd-v15-rollout-25x-safety-reality-view/)
- Tracker data through 1 Nov 2025: v14 averaged 834 miles between critical interventions, against 217 for v13.2. — [Eletric-vehicles.com](https://eletric-vehicles.com/tesla/tesla-fsd-v14-data-shows-major-improvement-in-miles-between-interventions/)
- GLJ Research (Gordon Johnson, a known Tesla bear, March 2026): city miles to critical disengagement "sharply deteriorating", from 4,109 (v14.1 peak) to 809 after v14.2. This is consistent with the Roth data, but the framing exaggerates because the v14.1 sample was tiny. — [Benzinga, Mar 2026](https://www.benzinga.com/markets/tech/26/03/51137537/teslas-fsd-safety-metrics-sharply-deteriorating-says-analyst)
- From v14.3.3 (around May 2026), the in-car disengagement menu offers Navigation / Parking / Critical / Other categories. — [Not a Tesla App](https://www.notateslaapp.com/news/4157/tesla-rolls-out-fsd-v1433-update-20261466-with-new-fsd-features-and-spring-update)
- The tracker site (teslafsdtracker.com) splits city and highway, but its dashboards are Power BI and the data could not be pulled in this session. — [teslafsdtracker.com](http://teslafsdtracker.com/); [Help](https://teslafsdtracker.com/help)

### Inferences
- A realistic v14.3 baseline is about 1,700 to 1,950 total miles, or about 850 city miles, per critical disengagement (crowdsourced, enthusiast drivers). City driving is about 2x worse than the blended figure, which implies highway driving is considerably better than blended.
- If the 25x claim is even partly true, the v15 tracker should show tens of thousands of miles. Detecting that needs far more fleet miles than the tracker collects in weeks. Expect no statistically meaningful v15 number before roughly Q1 2027.
- History repeatedly shows big first-release gains followed by point-release regressions (14.1 → 14.2, 14.3.2). Early v15 impressions should be discounted accordingly.

### Gaps
- No v15 tracker numbers. No highway-only critical rate for v14.3 was found. Tesla's "FSD evidence dashboard" ([tesla.com](https://tesla.com/fsd-evidence-dashboard)) appeared in search results but was not examined.

## 2. Scenario-by-scenario performance (independent reviews and tests)

### Takeaway
The early v15 reviews (4 days, a handful of pro-Tesla influencers, mainly in LA and Texas, no stated mileage) report smoother driving, working pothole avoidance, better corner handling and faster parking. They report no critical failures, but they are not independent tests. The scenario weaknesses documented independently in 2026 (on v14.x) are railway crossings, red lights and wrong-lane entry, school buses, speed-sign reading (especially in Europe), low-visibility degradation and highway exits. None of these has yet been shown to be fixed in v15.

### Cited Findings (v15, 21 to 26 Nov 2026; tester-biased, very small sample)
- Reviewers: Zack (@BLKMDL3, LA, first 24 h of v15.0.1), Abhimanyu Yadav (@WorldlyReviewer, Texas, about 1 h), Mike P (@mikepat711), Sawyer Merritt. All are pro-Tesla X accounts. No mileage is stated. — [EVWire, 23 Nov 2026](https://evwire.com/p/tesla-fsd-v15-first-reviews)
  - **Potholes:** dodged several by steering around them, hit one minor one in Beverly Hills. Smoother slowing for dips and speed bumps.
  - **Corners:** takes higher-speed corners well.
  - **Lane changes / dense traffic:** good lane choice, "cuts through traffic". Mad Max profile "very mad now" (assertive speeding).
  - **Following distance:** too close on the highway (Zack).
  - **Pedestrians / parked cars:** cautious with pedestrians stepping out. Gives room for opening doors.
  - **Parking:** picks spots faster, often takes the first open space without offering a choice. Reversed into an angled spot once. Pulled out of a grocery spot and back in. Supercharger parking was fine. Parking garages remain a weak spot (Yadav).
  - **Rain:** Reality View held up in rain. This is a visualisation comment, not a driving test.
  - **Failures:** no disengagements or crashes reported.
- Not a Tesla App (26 Nov 2026): v15 is reaching pre-"Juniper" AI4 Model Ys without the front bumper camera. Dirty Tesla reports v15 "seems to work fine without it". Known issues: Reality View lags about 0.5 s and sometimes "turns into an oil painting". Parking chevrons do not always appear. v15 was hesitant to enter one owner's garage. A tester called it the "smoothest update" with no false pothole alarms. — [Not a Tesla App](https://www.notateslaapp.com/news/5092/tesla-fsd-v15-rolls-out-to-model-ys-without-front-bumper-cameras)
- v15 release notes (Tesla's claims, not verified):
  - better detection of road signs, small animals, debris and chains
  - pothole and deep-standing-water avoidance
  - speed handling for school zones, speed limits and bad weather
  - slowing for speed cameras
  - end-to-end learned navigation (trip start and end only)
  - parking choice from personal history
  - looser driver monitoring in higher-speed profiles
  - The models are AI4 (HW4) only. — [Electrek](https://electrek.co/2026/11/21/tesla-fsd-v15-rollout-25x-safety-reality-view/)

### Cited Findings (older versions; mark as pre-v15)
- **Railway crossings (v14.x, 2026):**
  - Mar 2026, West Covina CA: FSD drove through lowered barriers and did not brake until after impact.
  - Apr 2026, Texas: crashed through a gate seconds before a train.
  - 26 Aug 2026: v14.3.7 at 9 mph tried to cross as a train arrived, with lights flashing. The driver braked.
  - Electrek counts at least 3 such incidents in 2026. NBC News documented more than 40 accounts. Senators Markey and Blumenthal asked NHTSA to investigate (Mar 2026). — [Electrek, 27 Aug 2026](https://electrek.co/2026/08/27/tesla-fsd-likes-trains-too-much/); [Electrek, 9 Mar 2026](https://electrek.co/2026/03/09/tesla-fsd-drives-through-railroad-crossing-barriers-viral-video/); [Electrek, 15 Apr 2026](https://electrek.co/2026/04/15/tesla-fsd-railroad-gate-oncoming-train-texas-owner/); [NBC News, 2025](https://www.nbcnews.com/tech/elon-musk/tesla-full-self-driving-fails-train-crossings-drivers-warn-railroad-rcna225558)
- **Red lights / opposing lanes (v13/v14):** NHTSA PE25012 cites red-light running and opposing-lane entry. Details are in Section 3. — [TechCrunch, 5 Dec 2025](https://techcrunch.com/2025/12/05/feds-find-more-complaints-of-teslas-fsd-running-red-lights-and-crossing-lanes)
- **School buses (date and version unclear, likely 2025):**
  - A Forbes-reported LA test found FSD ignoring a flashing school-bus stop sign, with 7 major safety lapses.
  - Dawn Project tests: the car failed the child-dummy school-bus test 6 of 6 times. The Dawn Project is an anti-Tesla advocacy group, so note the bias.
  - These findings come from search summaries only. The primary sources were not fetched. — [search result summary; Jalopnik/Forbes coverage](https://jalopnik.com/2119268/tesla-full-self-driving-drives-through-railroad-crossing)
- **School zones (Robotaxi):** NHTSA is reviewing a petition alleging that Tesla Robotaxis speed through active school zones (reported Nov 2026). — [Electrek](https://electrek.co/2026/11/21/tesla-fsd-v15-rollout-25x-safety-reality-view/)
- **Speed-sign reading / Europe (v14, Sept 2026):** the Brussels nonprofit Johanna.be found FSD over the limit in 55% of 30 km/h zones, averaging 44 km/h. The screen usually showed 50 km/h, so the car did not read the sign. — [Electrek](https://electrek.co/2026/11/21/tesla-fsd-v15-rollout-25x-safety-reality-view/)
- **Speed-limit offset in Europe:** the Dutch RDW approved "FSD Supervised" on 10 Apr 2026. In Nov 2026 the RDW signed off an update that caps the speed offset at 10% (it was 50%), resets the offset to 0% at each start, and renames the system "Tesla Assisted Driving". Sweden recommended voting against EU-wide approval unless over-limit driving is disabled. — [Electrek, 3 Nov 2026](https://electrek.co/2026/11/03/tesla-assisted-driving-rdw-speed-cap-10-percent-europe/); [Heise](https://heise.de/-11253269); [Asharq Al-Awsat/Reuters](https://english.aawsat.com/node/5285748)
- **Highway exits (v14.3.6, 2026):** Fred Lambert had a critical disengagement when the car tried to take an exit ramp it had already passed and began to veer off the road. — [Basenor summary](https://www.basenor.com/blogs/news/fsd-v14-3-6-highway-exit-bug-what-owners-need-to-do-now)
- **Unprotected lefts (v14.1.2, Oct 2025):** Chuck Cook's Jacksonville test was positive. A forum user reported v14 hesitating halfway through an unprotected left, with jerky steering, where v13 had been fine. — [Tesla Oracle, 18 Oct 2025](https://www.teslaoracle.com/2025/10/18/tesla-fsd-v14-1-2-shows-impressive-performance-at-unprotected-left-turns-video-review/); [TMC forum](https://teslamotorsclub.com/tmc/threads/fsd-v14-x.347764/page-141)
- **v14 general (Oct 2025 onward):** hallucinations, brake stabbing, excessive speeding. v14.2 point releases regressed. — [Electrek](https://electrek.co/2026/11/21/tesla-fsd-v15-rollout-25x-safety-reality-view/)
- **Emergency vehicles / roundabouts / construction (v14):** only low-quality blog anecdotes were found. Examples: over-cautious pull-overs for sirens on a different street; roundabouts "notoriously difficult"; construction "moderate improvement" in 14.3.3. — [Tesery blog](https://www.tesery.com/blogs/news/tesla-fsd-v14-3-3-update-technical-analysis-real-world-experience); [teslaacessories blog](https://www.teslaacessories.com/blogs/news/the-dawn-of-unsupervised-autonomy-%E2%80%94-inside-tesla's-fsd-v14-rollout-in-austin-and-palo-alto)
- **Vulnerable road users (v14.3, Apr 2026):** faster reaction time reported to help with animals and pedestrians (pro-Tesla outlet). — [Tesla Oracle, 10 Apr 2026](https://www.teslaoracle.com/2026/04/10/tesla-fsd-v14-3-faster-reaction-time-helps-save-animals-and-vulnerable-road-users/)

### Inferences
- **Probable v15 strengths** (Tesla notes plus early reviewers): road-surface hazards (potholes, debris, water), corner speed, decisive lane choice, faster parking.
- **Biggest unknowns for v15:** whether the v14 rule-compliance failures are fixed. These are railway crossings, red-light creep, school buses and sign reading. None of the early reviewers tested them.
- **For the openpilot comparison:** stock openpilot/sunnypilot does not do city turns, parking, junction negotiation or pothole avoidance at all. FSD's failure modes there are failures of a capability openpilot lacks. On highway lane-keeping and following, the useful comparison is the v14/v15 behaviour: close highway following distances, assertive speeding profiles and sign misreads.

### Gaps
- No structured independent v15 scenario tests yet (AI DRIVR, Chuck Cook, Consumer Reports, AMCI): none were found as of 26 Nov 2026.
- Found nothing reliable on v15 roundabouts, emergency vehicles, construction zones, night or glare driving, cyclists, or railway crossings.
- The school-bus test dates and FSD versions were not confirmed.
- No AMCI or Consumer Reports FSD-specific 2026 evaluation was found. CR pages are paywalled.

## 3. Safety and regulatory record relevant to v15

### Takeaway
Two active NHTSA defect probes cover FSD's known weak scenarios:
- **EA26002 (low visibility):** an Engineering Analysis since March 2026, 3.2M vehicles, pre-recall stage.
- **PE25012 (red lights / opposing lanes):** still a Preliminary Evaluation as of mid-2026, about 2.88M vehicles, 80 incidents.

Robotaxi crash reports continue to grow. No FSD-specific recall was found for 2026.

### Cited Findings
- **EA26002** (upgraded from PE24031 around 18 to 19 Mar 2026, about 3.2M vehicles):
  - NHTSA says FSD's degradation detection may fail to "detect and/or warn the driver appropriately" in glare and "airborne obscurants".
  - In the crashes reviewed, FSD lost track of, or never detected, the lead vehicle. Alerts came only just before the crash, or never.
  - These include a pedestrian fatality. About 9 crashes are confirmed and 6 more are under review. — [CleanTechnica, 20 Mar 2026](https://cleantechnica.com/2026/03/20/nhtsa-elevates-tesla-fsd-probe-to-engineering-analysis/); [Tesorb tracker](https://tesorb.com/nhtsa-tesla-fsd-investigation-tracker/)
- **PE25012** (opened 7 Oct 2025, 2,882,566 vehicles):
  - Covers red-light running and entry into opposing lanes.
  - Incident count grew from 58 to 80 (62 complaints, 14 Tesla field reports, 4 media reports).
  - 6 SGO crashes at red lights, 4 with injuries.
  - Scope may extend to driving next to opposing traffic and to railway crossings.
  - Information request dated 3 Dec 2025. Tesla's extension deadline was 4 Mar 2026.
  - Secondary sites conflict on whether PE25012 was upgraded. The most credible sources say it is still a PE (an injury-law site cites the docket as of 26 Jul 2026). — [NHTSA ODI PE25012 open](https://static.nhtsa.gov/odi/inv/2025/INOA-PE25012-19171.pdf); [NHTSA info request](https://static.nhtsa.gov/odi/inv/2025/INIM-PE25012-30897.pdf); [TechCrunch, 9 Oct 2025](https://techcrunch.com/2025/10/09/teslas-full-self-driving-software-under-investigation-for-traffic-safety-violations); [Yahoo Autos](https://autos.yahoo.com/policy-and-environment/articles/tesla-fsd-red-light-probe-120800698.html); [Aguiar Injury Lawyers](https://aguiarinjurylawyers.com/tesla-fsd-investigation-2026/)
- **PE24033 (Actually Smart Summon):** closed 3 Apr 2026 with no recall. 159 incidents and 97 low-speed parking-lot crashes, no injuries. Tesla shipped six OTA updates in 2025. — [Tesorb tracker](https://tesorb.com/nhtsa-tesla-fsd-investigation-tracker/)
- **AQ25002 (Aug 2025):** audit of Tesla's late crash reporting under the SGO. — [Tesorb tracker](https://tesorb.com/nhtsa-tesla-fsd-investigation-tracker/)
- **Robotaxi (Austin, launched Jun 2025):**
  - 14 crashes in the first 8 months (Feb 2026). Narratives were later un-redacted for 17 crashes; 2 of them happened under teleoperator control.
  - Electrek counted 36 unique Robotaxi incident reports by the Sept 2026 NHTSA release (22 in July). By Electrek's own calculation, the Robotaxi fleet crashes about 4x more often than humans. That is Electrek's estimate, not NHTSA's, and NHTSA cautions against comparing manufacturers with SGO data.
  - Tesla told JPMorgan that about 40% of v15's "core technologies" already run in the Austin Robotaxi fleet. — [CBS News](https://www.cbsnews.com/news/tesla-robotaxi-austin-14-crashes-nhtsa); [TechCrunch, 15 May 2026](https://www.techcrunch.com/2026/05/15/tesla-reveals-two-robotaxi-crashes-involving-teleoperators/); [Electrek, 17 Feb 2026](https://electrek.co/2026/02/17/tesla-robotaxi-adds-5-more-crashes-austin-month-4x-worse-than-humans/); [Electrek v15](https://electrek.co/2026/11/21/tesla-fsd-v15-rollout-25x-safety-reality-view/)
- **IIHS:** the 2026-27 Model Y earned Top Safety Pick+ (announced 3 Sep 2026), but this rated the standard crash-avoidance systems, not FSD. In 2024, IIHS rated Tesla's partial-automation safeguards "poor" (older). — [IIHS](https://iihs.org/news/detail/two-additional-models-garner-top-safety-pick-awards); [Fortune, Mar 2024](https://fortune.com/2024/03/12/iihs-test-tesla-rated-poor-lexus-acceptable/)
- **Driver monitoring:** v15 loosens driver monitoring in higher-speed profiles, while Tesla tightens it in Europe at regulators' request. There are reports of drivers sleeping on FSD. — [Electrek](https://electrek.co/2026/11/21/tesla-fsd-v15-rollout-25x-safety-reality-view/)

### Inferences
- Low-visibility lead-vehicle loss (EA26002) and traffic-control compliance (PE25012) are the regulator-documented failure classes. These are the scenarios v15 must be shown to have fixed. Both apply to camera-only perception, which openpilot shares. The red-light and railway issues relate to FSD's city capability, which openpilot does not attempt.

### Gaps
- No NHTSA document from May to November 2026 was retrieved. The current status of PE25012 and EA26002, and any v15-linked recall, is unconfirmed.
- No crash or incident involving v15 yet (5 days in).

## 4. What testers say FSD still cannot do or does badly

### Takeaway
As of late 2026, the recurring complaints are these:
- unreliable speed-limit and sign reading
- railway-crossing failures
- red-light and rolling-stop compliance
- parking garages and choice of parking spot
- following too close on highways and aggressive speed profiles
- navigation and exit errors
- regressions in point releases

v15's early reviewers mostly add parking quirks and close following. Tesla's own v15 notes target signs, potholes and speed handling, which implicitly confirms these were weak in v14.

### Cited Findings
- Electrek: FSD still drove through potholes before v15 (Musk first promised pothole avoidance in 2019). "Improved detection" claims give no size. Slowing only for speed cameras implies FSD may still speed elsewhere. History is "two steps forward, one step back". — [Electrek, 21 Nov 2026](https://electrek.co/2026/11/21/tesla-fsd-v15-rollout-25x-safety-reality-view/)
- Electrek (Nov 2026): Tesla's "biggest issue with FSD right now is that it doesn't properly detect the speed limits". — [Electrek, 3 Nov 2026](https://electrek.co/2026/11/03/tesla-assisted-driving-rdw-speed-cap-10-percent-europe/)
- v15 early reviewers:
  - parking garages remain weak
  - the car takes the first open spot without offering a choice
  - odd reverse-park and pull-out moves
  - highway following too close
  - hesitant garage entry
  - parking chevrons missing
  - Reality View lag — [EVWire](https://evwire.com/p/tesla-fsd-v15-first-reviews); [Not a Tesla App](https://www.notateslaapp.com/news/5092/tesla-fsd-v15-rolls-out-to-model-ys-without-front-bumper-cameras)
- Hargreaves Lansdown (Nov 2026) framed the 25x claim as "a claim to test rather than bank" (search snippet, not fetched). — [HL](https://www.hl.co.uk/shares/share-research/tesla-quick-take-is-fsd-v15-the-real-deal)
- v14.3.7 attempted to cross in front of a train even though the v14.3 notes claimed better handling of "rare and unusual objects". — [Electrek, 27 Aug 2026](https://electrek.co/2026/08/27/tesla-fsd-likes-trains-too-much/)

### Inferences
- **Bias balance:**
  - The v15 evidence comes entirely from pro-Tesla influencers in their first hours of use.
  - The negative evidence comes mostly from Electrek (Lambert is now openly critical), advocacy groups (Dawn Project), short sellers (GLJ) and regulators.
  - The crowdsourced tracker is the most neutral source, but it is enthusiast-skewed and not yet available for v15.

### Gaps
- No long-run v15 tester reports (more than 1,000 miles), no v15 city/highway split, and no structured scenario matrix from a neutral tester. Revisit in Dec 2026 to Jan 2027, after the wide release.
