# OSM tagging of stop signs, give-way signs and traffic signals: which direction of travel a mapped node applies to

Research date: 2026-11-13/14. Taginfo database snapshot: 2026-11-13 00:59 UTC. Overpass snapshot: 2026-11-14 07:31 UTC.

Method notes for the report writer:
- Wiki quotes were extracted through a page-summarising fetch tool. They are faithful to the pages but should be treated as near-verbatim; re-check a quote against the URL before printing it as an exact citation.
- Source-code findings were read directly from the raw files on the default branches (`master` / `develop` / `main`) on 2026-11-14; they are not pinned to a commit.
- Two findings are my own measurements (Overpass queries plus a script), marked "own measurement". They are not from a published source. Scripts and raw data are in `/private/tmp/claude-501/-Users-hiremanirmss-Fisker-sunnypilot/5822bfb0-0e5a-46d5-8088-6fd59cd1d7a1/scratchpad/` (`analyze.py`, `control.py`, `compass.json`, `sj.json`, `stopdir.csv`); that directory is temporary.

## 1. What the wiki pages say (highway=stop, highway=give_way, Key:direction, Key:traffic_signals:direction, Key:stop, Key:traffic_sign:direction)

### Takeaway
On a node that is part of a highway way, `direction=forward|backward` is documented as relative to the way's drawing (node) order, and it names the direction of *travel that is affected*: `forward` = traffic travelling in the way's direction must stop. The current implementation's reading of forward/backward is correct.

### Cited Findings
- highway=stop: "Insert a node with tag highway=stop on the approach way(s) that must stop, at the stopping point." — [Tag:highway=stop](https://wiki.openstreetmap.org/wiki/Tag:highway%3Dstop)
- highway=stop: "Since the stop line on the approach applies to only one travel direction" it is indicated with `direction` or `stop:direction`; "add direction=* to signify to which direction of travel the stop sign applies." — [Tag:highway=stop](https://wiki.openstreetmap.org/wiki/Tag:highway%3Dstop)
- highway=stop worked example: "First thing to know is the direction of the path where you want to place your highway=stop." "The stop on the left is mapped with direction=forward"; "The stop on the right is mapped with direction=backward". "Alternatively, the key stop:direction can be used with the same values." — [Tag:highway=stop](https://wiki.openstreetmap.org/wiki/Tag:highway%3Dstop)
- highway=give_way: "Follow the black arrows, add direction=forward if the driver can see the give way sign and must give way, else add direction=backward if the driver can not see the sign and must not give way." "Since the yield sign or marking on the approach applies to only one travel direction, this can be indicated using the key direction or give_way:direction with value forward or backward." — [Tag:highway=give_way](https://wiki.openstreetmap.org/wiki/Tag:highway%3Dgive_way)
- Key:direction, relative values: `forward` = "The direction is forward, relative to how the associated way was drawn in the editor."; `backward` = "The direction is backward, relative to how the associated way was drawn in the editor."; `both` = "The direction is both forward and backward." — [Key:direction](https://wiki.openstreetmap.org/wiki/Key:direction)
- Key:traffic_signals:direction: "Indicates the direction of travel affected by the traffic signals." `forward` = "The traffic signal affects the forward direction, relative to how the associated way was drawn in the editor."; `backward` likewise; `both` = "The traffic signal affects both the forward and the backward direction." "This tag only applies for nodes that are part of a way." Status "in use". — [Key:traffic_signals:direction](https://wiki.openstreetmap.org/wiki/Key:traffic_signals:direction)
- Key:stop: `stop=all` = "All approaches to this highway=stop node must stop (all-way stop)"; `stop=minor` = "Only minor road approaches to this highway=stop node must stop (minor-road stop)". Status "in use". The page records older proposals that put `stop=*` on ways with values `yes`, `both`, `-1` and cardinal lists ("a listing of cardinal directions separated by semicolons"), says cardinal directions were favoured by North Americans, and calls `direction=forward/backward` "a more widely accepted approach." — [Key:stop](https://wiki.openstreetmap.org/wiki/Key:stop)
- Key:traffic_sign:direction: "The direction affected by a traffic_sign relative to the highway way (equivalent to direction)." `forward` = "The traffic sign affects vehicles travelling in the same direction as the direction of the highway=* way."; `backward` = "...in the opposite direction as the direction of the highway=* way." — [Key:traffic_sign:direction](https://wiki.openstreetmap.org/wiki/Key:traffic_sign:direction)
- Key:traffic_sign, for nodes on a way: "`forward` means the same direction as the `highway=*` way and `backward` the opposite direction." — [Key:traffic_sign](https://wiki.openstreetmap.org/wiki/Key:traffic_sign)
- A separate wiki page defines `stop:direction` (forward/backward) and advises using it only on nodes that are part of exactly one highway, because otherwise it is ambiguous (reported by a search-result summary; page not fetched) — [Key:stop:direction](https://wiki.openstreetmap.org/Key:stop:direction)

### Inferences
- Four independent wiki pages agree: on a way node, forward/backward is the *affected direction of travel*, relative to node order. This part of the implementation needs no change.
- The Key:direction page defines forward/backward only as "relative to how the way was drawn"; the "affected direction of travel" meaning for stop/give-way comes from the tag pages and from Key:traffic_sign:direction, not from Key:direction itself.
- `stop:direction` and `give_way:direction` are documented synonyms; an implementation should read them as a fallback when `direction` is absent (usage is tiny, see section 4).

### Gaps
- Key:direction has no section specific to highway=stop or give_way (the fetch found none), so the claim in the current code docstring that Key:direction carries a "direction of travel affected" reading for stop signs is not supported by that page.
- Key:stop:direction was not fetched directly.

## 2. Compass / degree values of direction=* on a stop node: facing or travel?

### Takeaway
The wiki defines compass and degree values as the direction the sign *faces*, which is opposite to the travel it holds, and it documents them only for sign nodes placed beside the road, not for `highway=stop` nodes on a way. Real data is split: degree values mostly follow the facing convention, cardinal letters mostly follow the travel convention, so neither reading is safe to apply blindly.

### Cited Findings
- Key:direction, degrees: "The given number specifies the degrees from 0 to 359 in which the feature faces, where 0 is equal to northward". Example: "Bench facing east: direction=90 (or direction=E)." Cardinal values "are **not** allowed to specify directions on a way." — [Key:direction](https://wiki.openstreetmap.org/wiki/Key:direction)
- Key:direction, traffic signs: they "are tagged the direction the sign faces, i.e., against the direction of travel." "But if the node of the sign is connected to the highway, then the relative direction tagging scheme comes into effect". "If these nodes are instead placed at their real position next to the road, the cardinal direction should be used." — [Key:direction](https://wiki.openstreetmap.org/wiki/Key:direction)
- Key:traffic_sign, separate nodes beside the road: "You can use the `direction=*` tag to describe the **facing** orientation of the sign by using an angle or cardinal direction." "Note that the sign is facing against the direction of travel." "So if you encounter a traffic sign when traveling north, then the sign is facing south. So you can add direction=180 or direction=S." — [Key:traffic_sign](https://wiki.openstreetmap.org/wiki/Key:traffic_sign)
- Key:traffic_signals:direction: "For traffic signal nodes placed at their real position next to the road, use `direction=*` with cardinal directions or angles as the value." — [Key:traffic_signals:direction](https://wiki.openstreetmap.org/wiki/Key:traffic_signals:direction)
- The Tag:highway=stop and Tag:highway=give_way pages do not mention compass or degree values at all. — [Tag:highway=stop](https://wiki.openstreetmap.org/wiki/Tag:highway%3Dstop), [Tag:highway=give_way](https://wiki.openstreetmap.org/wiki/Tag:highway%3Dgive_way)
- Historical origin of cardinal values on stops: the talk page records a 2009 proposal `stop=N;E;S;W` on the intersection node (RussNelson: "using stop=N;E;S;W is because when you fetch the way, you also fetch the intersection"), and a 2014 objection that cardinal values do "not make sense for junctions with roads approaching from non-cardinal directions". — [Talk:Tag:highway=stop](https://wiki.openstreetmap.org/wiki/Talk:Tag:highway%3Dstop)
- iD renders a numeric or cardinal `direction` on a vertex as the cone angle directly, and renders `direction=forward` as a cone pointing from the node toward the *previous* node of the way (`lookForward` adds `nodes[i - 1]`). So in iD a numeric value is drawn the same way as the facing direction of a forward-affecting sign. — [iD modules/osm/node.ts](https://github.com/openstreetmap/iD/blob/develop/modules/osm/node.ts)
- Own measurement, global Overpass query of `highway=stop` nodes whose `direction` is a plain number or 1-3 cardinal letters (6,516 nodes). For the 3,843 nodes that sit mid-way on exactly one highway way with one way-end clearly nearer (within 40 m, other end at least 2.5 times farther), I compared the tag with the bearing of travel toward the near end:
  - Degree values: 2,034 facing (differ by 135 degrees or more from travel), 164 travel (within 45 degrees), 86 neither. That is 89% facing.
  - Cardinal letters: 1,069 travel, 358 facing, 132 neither. That is 69% travel.
  - Excluded: 1,641 on a node shared by more than one way, 922 mid-way with no clearly nearer end, 104 not on a highway way, 6 at a way end.
  - Source: [Overpass API](https://overpass-api.de/api/interpreter), query `node[highway=stop][direction~"^([0-9]+|[NSEWnsew]{1,3})$"]; way(bn)[highway]; out geom;`
- Positive control for that heuristic (own measurement): on 9,871 `highway=stop` nodes in a San Jose / South Bay bounding box (37.20,-122.10,37.45,-121.75), "travel is toward the nearer way end" agreed with the mapped `direction=forward/backward` on 5,810 of 5,969 testable nodes (97.3%); 2,934 were not testable. — [Overpass API](https://overpass-api.de/api/interpreter)

### Inferences
- Documented convention: compass/degree = facing = opposite of affected travel. The current code's planned reading ("compass bearing of the direction of travel the sign applies to", attributed to Key:direction and to iD) is the reverse of what both the wiki and iD's renderer do. Keeping `COMPASS_DIRECTIONS_TRUSTED = False` was the right call.
- De-facto usage contradicts itself by value type. A likely explanation is two populations: degree values written by tools or mappers following the sign-facing convention, and cardinal letters inherited from the North American `stop=N;S` travel-direction proposals. This explanation is my inference; no source states it.
- Safest handling for a driving-assist feature: do not interpret a compass value on its own. Either leave it UNKNOWN, or use it only to choose between the two bearings of the way at the node and accept it only when it agrees with the geometric "toward nearest junction" answer. Multi-value forms (`N;S`, `0;180`, about 450 nodes) describe junction nodes and carry no usable sign: both readings give the same axis.
- Compass values are rare enough (section 4: 0.29% of stop nodes globally, about 0.08% in California) that leaving them UNKNOWN costs almost nothing.

### Gaps
- No wiki page explicitly says "do not use compass values on highway=stop"; it is undocumented rather than expressly discouraged. The only explicit prohibition found is Key:direction's rule that cardinal values are not allowed on ways.
- The facing/travel measurement relies on the way-end heuristic (97.3% accurate in the control sample, in one region). I did not break the result down by country or by editor/changeset source.

## 3. Where the stop node belongs, and what junction-node or untagged nodes mean

### Takeaway
The documented position is on the approach way at the stop line, not on the junction node; the junction node is used only for the simple all-way form `highway=stop` + `stop=all`. For an untagged node the wiki gives no inference rule; the long-standing community rule of thumb, which OSRM implements, is that the sign applies to traffic heading toward the nearest intersection.

### Cited Findings
- "Insert a node with tag highway=stop on the approach way(s) that must stop, at the stopping point." "Do NOT tag the conflict points" because "not all the approaches will stop." "As this minor road highway=stop node is NOT at the intersection point, we know it just shows the stop line." Without a painted line the point "may be at the nearer edge of a crosswalk." — [Tag:highway=stop](https://wiki.openstreetmap.org/wiki/Tag:highway%3Dstop)
- All-way stops: "All-way stops can be mapped in different ways". "For a simple intersection, all-way stops can be mapped using highway=stop stop=all at the intersection node." For split carriageways or more detail: "add separate nodes at the respective stop positions" and "Add stop=all to all of the highway=stop nodes." — [Tag:highway=stop](https://wiki.openstreetmap.org/wiki/Tag:highway%3Dstop)
- "In all-way stop countries you could also add stop=minor to distinguish 2-way from 4-way stop signs." Priority junctions: "tag the intersection node(s) where traffic conflicts can occur with highway=priority". — [Tag:highway=stop](https://wiki.openstreetmap.org/wiki/Tag:highway%3Dstop)
- Give way: "Add highway=give_way near to the dotted lines in the middle of the road". "Give way should not be placed directly on junction=roundabout at a conflict point because it is not possible to understand which road gives way to the other. Instead, you should extract the give way from the roundabout and place it on the road that approaches the roundabout, at the marked stop line (if applicable)." — [Tag:highway=give_way](https://wiki.openstreetmap.org/wiki/Tag:highway%3Dgive_way)
- Nearest-junction rule, talk page only: Alv (2008) "almost always the nearest intersection node is the correct one."; Pieren (2009) "the stop sign applies to the nearest intersection point"; Ceyockey (2011) objected that without tags "There is no indication of which way the sign is facing or which direction of travel is impacted." — [Talk:Tag:highway=stop](https://wiki.openstreetmap.org/wiki/Talk:Tag:highway%3Dstop)
- The Tag:highway=stop page itself contains no "nearest junction" rule and no explicit routing rule. — [Tag:highway=stop](https://wiki.openstreetmap.org/wiki/Tag:highway%3Dstop)
- Own measurement, San Jose sample (9,871 stop nodes): 9,231 mid-way on a single way (8,908 with a direction tag, 323 without); 637 on a node shared by more than one way (269 with direction, 368 without). Of 764 `stop=all` nodes, 298 had no direction, 175 had `direction=both`, and 291 had `direction=forward` or `backward`. — [Overpass API](https://overpass-api.de/api/interpreter)

### Inferences
- `stop=all` does not imply "junction node". The wiki tells mappers to put `stop=all` on every per-approach node of a detailed all-way stop, and 38% of the `stop=all` nodes in the San Jose sample carry forward/backward. The current rule "stop=all: always YES" therefore answers YES for a car that has just left the junction and passes the opposing approach's stop node on the same undivided way. An explicit forward/backward on a single-way node should take precedence over `stop=all`.
- `stop=minor` on a mid-way node with forward/backward is the ordinary two-way-stop form (2,794 of the sample); it adds no directional information there. On a junction node, `stop=minor` means only the minor approaches stop, which matches the implementation's rank-based inference.
- For an untagged mid-way node, "applies to travel toward the nearer junction" is a well-founded default: it matched explicit tags 97.3% of the time in the control sample, and OSRM ships a 20 m version of it (section 5). The current implementation returns UNKNOWN for an untagged mid-way node; this rule could resolve most of those.
- Because the node is at the stop line, the car reaches the node before the junction. A stop node encountered just *after* a junction (travelling away from it) is for the opposing approach.

### Gaps
- The wiki does not define the meaning of a bare `highway=stop` on a junction node with no `stop=*` tag. Consumers differ (section 5).
- "Shared by more than one way" in my sample also counts nodes where a way was merely split at the stop node, so the 637 figure overstates true junction-node placement.

## 4. Taginfo statistics (global, US, California)

### Takeaway
About 87% of `highway=stop` nodes carry `direction=*`, and over 99.6% of those values are forward, backward or both; compass and degree values are 0.29% of stop nodes worldwide. Traffic signals are far less complete: only 41% globally have `traffic_signals:direction`.

### Cited Findings
All counts from the taginfo API, data as of 2026-11-13. Global: [taginfo.openstreetmap.org](https://taginfo.openstreetmap.org/tags/highway=stop#combinations). US: [taginfo.geofabrik.de/north-america:us](https://taginfo.geofabrik.de/north-america:us/tags/highway=stop#combinations). California: [taginfo.geofabrik.de/north-america:us:california](https://taginfo.geofabrik.de/north-america:us:california/tags/highway=stop#combinations).

highway=stop nodes:

| | Global | US | California |
|---|---|---|---|
| Nodes | 2,549,758 | 1,070,912 | 239,312 |
| with `direction=*` | 2,225,501 (87.3%) | 933,231 (87.1%) | 210,855 (88.1%) |
| `direction=forward` | 1,150,899 (45.1%) | 460,211 (43.0%) | 101,028 (42.2%) |
| `direction=backward` | 1,035,691 (40.6%) | 442,902 (41.4%) | 99,406 (41.5%) |
| `direction=both` | 31,496 (1.2%) | 26,572 (2.5%) | 10,224 (4.3%) |
| other `direction` values (derived by subtraction) | 7,415 (0.29%) | 3,546 (0.33%) | 197 (0.08%) |
| with `stop=*` | 425,374 (16.7%) | 245,074 (22.9%) | 60,540 (25.3%) |
| `stop=minor` | 285,154 (11.2%) | 169,281 (15.8%) | 34,705 (14.5%) |
| `stop=all` | 126,275 (5.0%) | 74,016 (6.9%) | 24,549 (10.3%) |

- The US holds 42% of all `highway=stop` nodes (1,070,912 of 2,549,758). — [taginfo US](https://taginfo.geofabrik.de/north-america:us/tags/highway=stop), [taginfo global](https://taginfo.openstreetmap.org/tags/highway=stop)
- Breakdown of the 7,415 non-relative `direction` values on stop nodes worldwide (own Overpass count; total matches taginfo exactly): 4,120 purely numeric, 2,309 cardinal letters, 986 other (lists such as `N;S`, `0;180`, words such as `north`, `all`, `anticlockwise`). Most common: N 600, S 563, E 497, W 464, 180 336, 90 331, 270 315, 0 290, N;S 149, E;W 118, 0;180 108, 90;270 74. — [Overpass API](https://overpass-api.de/api/interpreter)
- In the US, the top degree values on stop nodes are 180 (306), 90 (283), 270 (265), 0 (259). — [taginfo US](https://taginfo.geofabrik.de/north-america:us/tags/highway=stop#combinations)
- `stop=*` key values, all objects, global: minor 286,467; all 126,928; yes 13,739; no 958; N 290; S 288; W 243; E 232; both 196; -1 111; N;S 85; forward 69; E;W 64. — [taginfo key stop](https://taginfo.openstreetmap.org/keys/stop#values)
- `traffic_sign:direction` on stop nodes: 1,090 globally; `traffic_sign:forward` 11,613 and `traffic_sign:backward` 12,374. — [taginfo global](https://taginfo.openstreetmap.org/tags/highway=stop#combinations)

highway=give_way nodes:

| | Global | US | California |
|---|---|---|---|
| Nodes | 1,717,955 | 59,772 | 6,337 |
| with `direction=*` | 1,434,550 (83.5%) | 40,531 (67.8%) | 5,187 (81.9%) |
| forward | 838,906 (48.8%) | 29,804 (49.9%) | 3,531 (55.7%) |
| backward | 583,032 (33.9%) | 10,330 (17.3%) | 1,641 (25.9%) |

Source: [taginfo give_way global](https://taginfo.openstreetmap.org/tags/highway=give_way#combinations), [US](https://taginfo.geofabrik.de/north-america:us/tags/highway=give_way#combinations), [California](https://taginfo.geofabrik.de/north-america:us:california/tags/highway=give_way#combinations)

highway=traffic_signals nodes:

| | Global | US | California |
|---|---|---|---|
| Nodes | 2,033,487 | 549,319 | 89,246 |
| with `traffic_signals:direction=*` | 826,738 (40.7%) | 200,082 (36.4%) | 55,486 (62.2%) |
| forward | 546,295 (26.9%) | 122,876 (22.4%) | 40,193 (45.0%) |
| backward | 219,875 (10.8%) | 42,012 (7.6%) | 10,785 (12.1%) |
| both | 59,694 (2.9%) | 34,874 (6.3%) | 4,447 (5.0%) |
| with plain `direction=*` | 23,157 (1.1%) | 5,355 (1.0%) | 1,171 (1.3%) |

Source: [taginfo traffic_signals global](https://taginfo.openstreetmap.org/tags/highway=traffic_signals#combinations), [US](https://taginfo.geofabrik.de/north-america:us/tags/highway=traffic_signals#combinations), [California](https://taginfo.geofabrik.de/north-america:us:california/tags/highway=traffic_signals#combinations)

- `traffic_signals:direction` values, all objects, global: forward 550,795; backward 222,409; both 61,088; all 31; then compass forms in the tens (N;E 25, S;W 22, N 17, S 16). — [taginfo key traffic_signals:direction](https://taginfo.openstreetmap.org/keys/traffic_signals:direction#values)

### Inferences
- Forward/backward handling covers about 86% of stop nodes globally and 84% in California; the remaining cases are mainly "no direction" (12-13%) and `both` (1-4%), not compass values.
- `direction=both` is unusually common in California (4.3%); in the San Jose sample it mostly accompanies `stop=all`, which is the junction-node all-way form. Treating `both` as "applies to all approaches through this node" is consistent with Key:direction, Valhalla and iD (section 5); the current implementation returns UNKNOWN for it.
- A plain `direction=forward/backward` on a traffic-signal node (1.1%) should be read as a fallback for `traffic_signals:direction`, as OSRM does.

### Gaps
- Taginfo does not report how many stop nodes sit on junction nodes versus mid-way; the only placement figures are from my single San Jose sample.
- I did not get taginfo counts for `stop:direction` and `give_way:direction`; they did not appear among the top combinations, so each is below roughly 0.04% of nodes.

## 5. How established software interprets these tags

### Takeaway
Every consumer examined reads only forward, backward and both, relative to way node order, as the affected travel direction. None interprets compass or degree values for stop signs; Valhalla discards such nodes and OSRM treats them as undirected.

### Cited Findings
- OSRM, node parsing: for `highway` in stop / give_way / traffic_signals it reads `direction`; for signals "traffic_signals:direction trumps direction"; `stop=minor` becomes a separate `stop_minor` type; the direction is `obstacle_direction[direction] or obstacle_direction.none`, so any value outside the enum becomes "none". — [osrm-backend profiles/lib/obstacles.lua](https://github.com/Project-OSRM/osrm-backend/blob/master/profiles/lib/obstacles.lua)
- OSRM direction enum is None, Forward, Backward, Both; "The obstacle may be present in both directions or in one direction only"; for unidirectional obstacles it "stores the node leading to it." — [osrm-backend include/extractor/obstacles.hpp](https://github.com/Project-OSRM/osrm-backend/blob/master/include/extractor/obstacles.hpp)
- OSRM turn processing, junction-node `stop=minor`: "disregard a minor stop if entering by the major road / rationale: if a stop sign is tagged at the center of the intersection with stop=minor it should only penalize the minor roads entering the intersection". Minor versus major is decided by comparing road speeds ("comparing road speeds (anybody has a better idea?)"). — [osrm-backend profiles/car.lua](https://github.com/Project-OSRM/osrm-backend/blob/master/profiles/car.lua), [obstacles.lua](https://github.com/Project-OSRM/osrm-backend/blob/master/profiles/lib/obstacles.lua)
- OSRM untagged-stop heuristic: "heuristic to infer the direction of a stop without an explicit direction tag / rationale: a stop sign should not be placed farther than 20m from the intersection". On a two-road node with an undirected stop, the penalty is skipped when the source road is shorter than 20 m and the target road longer than 20 m, meaning the vehicle has just left the junction. — [osrm-backend profiles/car.lua](https://github.com/Project-OSRM/osrm-backend/blob/master/profiles/car.lua)
- OSRM legacy signal helper reads only `traffic_signals:direction` forward / backward, otherwise all directions. — [osrm-backend profiles/lib/traffic_signal.lua](https://github.com/Project-OSRM/osrm-backend/blob/master/profiles/lib/traffic_signal.lua)
- Valhalla, stop nodes: `direction=both` sets forward_stop and backward_stop; `forward` sets forward_stop; `backward` or `reverse` sets backward_stop; otherwise `elseif kv["direction"] ~= nil and kv["stop"] == nil then kv["highway"] = nil`, so a stop whose direction is any other value (a compass value, for example) and that has no `stop=*` is dropped. The same logic is applied to `highway=give_way` with `give_way=*`. — [valhalla lua/graph.lua](https://github.com/valhalla/valhalla/blob/master/lua/graph.lua)
- Valhalla, signals: only `traffic_signals:direction` forward / backward is read. — [valhalla lua/graph.lua](https://github.com/valhalla/valhalla/blob/master/lua/graph.lua)
- Valhalla graph build: a stop or yield with no direction attribute applies to both directions of the edge (`edge.attributes.direction ? edge.attributes.forward_stop : true`); on a one-way with neither flag set it applies; a `minor` flag is carried for stop/yield nodes. — [valhalla src/mjolnir/graphbuilder.cc](https://github.com/valhalla/valhalla/blob/master/src/mjolnir/graphbuilder.cc)
- GraphHopper: repository code search for `traffic_signals` returned only the pedestrian-crossing enum (`Crossing.TRAFFIC_SIGNALS`, `OSMCrossingParser`), and `give_way` returned only turn-restriction parsing. No handling of `highway=stop` direction was found. — [graphhopper OSMCrossingParser.java](https://github.com/graphhopper/graphhopper/blob/master/core/src/main/java/com/graphhopper/routing/util/parsers/OSMCrossingParser.java)
- iD presets: Stop Sign has fields `stop` (radio: "All Ways", "Minor Road") and `direction_vertex` (key `direction`, label "Direction Affected", options Forward / Backward / "Both / All"). Yield Sign uses `direction_vertex_dual`. Traffic Signals uses `traffic_signals:direction` with the same label and options. No compass input is offered for these presets; the degree field ("Direction (Degrees Clockwise)") is a different field not attached to them. — [id-tagging-schema highway/stop.json](https://github.com/openstreetmap/id-tagging-schema/blob/main/data/presets/highway/stop.json), [fields/direction_vertex.json](https://github.com/openstreetmap/id-tagging-schema/blob/main/data/fields/direction_vertex.json), [fields/traffic_signals/direction.json](https://github.com/openstreetmap/id-tagging-schema/blob/main/data/fields/traffic_signals/direction.json), [fields/direction.json](https://github.com/openstreetmap/id-tagging-schema/blob/main/data/fields/direction.json)
- iD rendering: a node on a highway intersection with `stop=all` is drawn as direction "all" regardless of `direction`; any key ending in `:direction` is honoured like `direction`; `both` and `all` look both ways along every parent way. — [iD modules/osm/node.ts](https://github.com/openstreetmap/iD/blob/develop/modules/osm/node.ts)
- iD reverses `traffic_signals:direction` on vertices when a way is reversed (test: forward becomes backward). — [iD test/spec/actions/reverse.js](https://github.com/openstreetmap/iD/blob/develop/test/spec/actions/reverse.js)
- JOSM's reverse-way corrector swaps forward/backward for keys that start or end with "direction" and "Also tests the nodes, e.g. a highway=stop with direction, see #20013." JOSM's validator flags `highway=stop` on a way and on an unconnected node. — [JOSM ReverseWayTagCorrector.java](https://github.com/JOSM/josm/blob/master/src/org/openstreetmap/josm/actions/corrector/ReverseWayTagCorrector.java), [JOSM geometry.mapcss](https://github.com/JOSM/josm/blob/master/resources/data/validator/geometry.mapcss)
- StreetComplete: the quest directory has no stop-sign, give-way or signal-direction quest; its signal quests are `traffic_signals_button`, `traffic_signals_sound`, `traffic_signals_vibrate` (pedestrian features). — [StreetComplete quests directory](https://github.com/streetcomplete/StreetComplete/tree/master/app/src/commonMain/kotlin/de/westnordost/streetcomplete/quests)
- pfeiferj/mapd: its roadmap lists "Additional map files for stop sign/stop light locations, possibly some other node based things" as an unchecked item; code search found direction handling only for speed limits (`MaxSpeedForDirection`). It does not process stop or signal nodes. — [pfeiferj/mapd Roadmap.md](https://github.com/pfeiferj/mapd/blob/main/Roadmap.md)
- OsmAnd (2017 report on the tagging list, secondary): it announced `highway=stop` but ignored `direction=*`. — [tagging list, March 2017](https://lists.openstreetmap.org/pipermail/tagging/2017-March/031779.html)

### Inferences
- The two routers form a consistent reference behaviour: (a) forward/backward/both relative to node order; (b) no direction = applies both ways, with OSRM's 20 m rule to suppress the "just left the junction" case; (c) junction-node `stop=minor` = only the minor approach; (d) unknown direction values are never interpreted as bearings.
- Both editors keep forward/backward correct when a way is reversed, which is why the relative scheme is robust in practice. The remaining failure mode is a snapshot mismatch between the node tags and the way geometry, which the current implementation already treats as UNKNOWN.
- The current implementation's rank-based inference for a bare junction-node stop is more conservative than Valhalla (applies to all) and in line with OSRM's speed-based `stop=minor` logic.

### Gaps
- GraphHopper and StreetComplete conclusions rest on code search and a directory listing; GitHub code search was rate-limited midway, so a targeted StreetComplete search for `highway = stop` did not complete. I found no evidence of stop-direction handling in either, but absence is not fully confirmed.
- sunnypilot's own mapd and the older "move-fast" / stop-sign openpilot forks could not be searched (rate limit; the old `commaai/openpilot` `selfdrive/mapd/mapd_helpers.py` path returned 404 at the tags tried). No finding either way.
- No iD or JOSM validator rule was found that warns about a missing direction or a compass value on a stop vertex; not exhaustively checked.

## 6. US and California practice, imports and MapRoulette campaigns

### Takeaway
US stop signs are overwhelmingly mapped per approach, mid-way, with forward/backward: 87% of US and 88% of California stop nodes have a direction. Junction-node mapping exists mainly as the `stop=all` all-way form.

### Cited Findings
- US: 1,070,912 stop nodes, 87.1% with `direction`; California: 239,312, 88.1% with `direction`, 10.3% `stop=all`, 14.5% `stop=minor`. — [taginfo US](https://taginfo.geofabrik.de/north-america:us/tags/highway=stop#combinations), [taginfo California](https://taginfo.geofabrik.de/north-america:us:california/tags/highway=stop#combinations)
- California stop-node density is the highest of the three scopes: taginfo's node `count_fraction` for `highway=stop` is 0.0501 in California, 0.0222 in the US and 0.0085 globally. I did not confirm the denominator taginfo uses for this fraction (it appears to be tagged nodes), so use it only as a relative comparison. — [taginfo California](https://taginfo.geofabrik.de/north-america:us:california/tags/highway=stop), [taginfo US](https://taginfo.geofabrik.de/north-america:us/tags/highway=stop), [taginfo global](https://taginfo.openstreetmap.org/tags/highway=stop)
- Own measurement, San Jose / South Bay sample: 93.5% of 9,871 stop nodes were mid-way on a single way, and 90.2% were mid-way with a direction tag. — [Overpass API](https://overpass-api.de/api/interpreter)
- 2017: Telenav staff (Horea Meleg) announced two MapRoulette challenges, for traffic signals and for stop signs, built from open data for Wayne County, Michigan. In reply, Paul Johnson objected to node direction tags: "So we're ignoring that nodes don't inherit the directionality of the underlying way?" and argued a relation was needed. — [talk-us, June 2017](https://lists.openstreetmap.org/pipermail/talk-us/2017-June/017411.html)
- 2020: a MapRoulette challenge by community member Binnette added `direction=forward/backward` to stop signs across Ontario, Canada ("So you have 12k tasks to solve."), after a suggestion by Matthew Darwin. — [MapRoulette blog, 2020-12-03](https://blog.maproulette.org/2020/12/03/adding-directionality-to-stop-signs-in-maproulette/)
- A 2021 help-site answer said `direction=both` on a stop is rarely needed and is usually better mapped as two nodes, each with its own direction (search-result summary; page not fetched). — [help.openstreetmap.org question 81548](https://help.openstreetmap.org/questions/81548/looking-for-best-practices-for-all-way-stop-intersections)
- Key:stop states cardinal directions were favoured by North Americans in early proposals. — [Key:stop](https://wiki.openstreetmap.org/wiki/Key:stop)

### Inferences
- The high density and near-uniform tagging in California suggest organised or tool-assisted mapping (corporate mapping teams or machine-detected signs), but I found no source naming a California import or campaign, so this is unconfirmed.
- For a California-focused feature, forward/backward on mid-way nodes is the dominant case by a wide margin; correctness there matters far more than compass handling.

### Gaps
- No documented California- or US-wide stop-sign import, and no currently active US MapRoulette challenge for stop direction, was found. The two campaigns found are Michigan (2017) and Ontario (2020).
- community.openstreetmap.org threads on stop direction were not retrieved; the forum evidence here is from the older mailing lists and the wiki talk page.

## 7. Traffic signals: junction node versus per-stop-line nodes

### Takeaway
The wiki accepts both schemes and says there is no settled convention. A signal node on a junction node applies to every approach; a mid-way node needs `traffic_signals:direction` to say which travel direction it controls, and about 59% of signal nodes worldwide lack it.

### Cited Findings
- "For simple intersections, just tag the intersection node." Alternative: "put a traffic signal on all incoming ways of a junction on the place of the stop line." "As of now, there is no well established convention on how this problem should be ideally solved". "Many junctions are tagged this way due to simplicity." — [Tag:highway=traffic_signals](https://wiki.openstreetmap.org/wiki/Tag:highway%3Dtraffic_signals)
- "Traffic signals added to dual-ways need to be tagged with either" `traffic_signals:direction=forward` or `backward`. The tag table lists `direction` or `traffic_signals:direction` (forward / backward / both): "Indicates which direction of travel (w.r.t. the orientation of the OSM highway way) sees the traffic signal." — [Tag:highway=traffic_signals](https://wiki.openstreetmap.org/wiki/Tag:highway%3Dtraffic_signals)
- "because traffic signals can affect routing decisions, it is important that they are attached to the ways" they apply to; "It is up to the routing software to count nearby signals as one for timing purposes." — [Tag:highway=traffic_signals](https://wiki.openstreetmap.org/wiki/Tag:highway%3Dtraffic_signals)
- Key:traffic_signals:direction motivation: at complex junctions with several signal nodes, routers can count several signals where traffic meets only one. — [Key:traffic_signals:direction](https://wiki.openstreetmap.org/wiki/Key:traffic_signals:direction)
- Usage: 40.7% of signal nodes globally, 36.4% in the US and 62.2% in California have `traffic_signals:direction`; `both` is 2.9% / 6.3% / 5.0%. — [taginfo global](https://taginfo.openstreetmap.org/tags/highway=traffic_signals#combinations), [US](https://taginfo.geofabrik.de/north-america:us/tags/highway=traffic_signals#combinations), [California](https://taginfo.geofabrik.de/north-america:us:california/tags/highway=traffic_signals#combinations)
- Consumers: OSRM reads `traffic_signals:direction`, falling back to `direction`; Valhalla reads only `traffic_signals:direction`; on a one-way, Valhalla applies an undirected signal. — [osrm obstacles.lua](https://github.com/Project-OSRM/osrm-backend/blob/master/profiles/lib/obstacles.lua), [valhalla graph.lua](https://github.com/valhalla/valhalla/blob/master/lua/graph.lua), [valhalla graphbuilder.cc](https://github.com/valhalla/valhalla/blob/master/src/mjolnir/graphbuilder.cc)

### Inferences
- Rule set for "does this signal apply to my approach": (1) node on a junction node (shared by two or more highway ways): applies to all approaches; (2) mid-way node with `traffic_signals:direction` (or `direction`) forward/backward: applies to that travel direction by node order; `both`: applies both ways; (3) mid-way node on a one-way: applies; (4) mid-way node on a two-way road with no direction: ambiguous, and the nearest-junction rule is the reasonable default.
- The undirected signal nodes are likely to be mostly junction-node signals (the simple scheme needs no direction tag), so the 59% without a direction is not a 59% ambiguity rate. I did not measure the split.
- With per-stop-line mapping, one physical intersection yields several signal nodes, and a vehicle crossing it passes its own stop-line node and then, on an undivided road, the opposing approach's node. Direction filtering is what prevents a second, false signal just past the junction.

### Gaps
- No measurement of how many undirected signal nodes are junction nodes versus mid-way nodes.
- Signals mapped beside the road with a compass `direction` (the documented form for off-way nodes) were not quantified; plain `direction` on signal nodes is 1.1% in total and is mostly forward/backward.
