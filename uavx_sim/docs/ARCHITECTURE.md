# Architecture: UAV-X Stage 1 PoC (v3, BVLOS scale)

## 1. Overview

```
                    +---------------------------------------------------+
                    |                 Webots R2025a                     |
                    |  worlds/uavx_stage1.wbt                           |
                    |   GCS Robot (supervisor) -- swarm_supervisor.py   |
                    |     | mirrors state into scene each 32 ms step:   |
                    |     |  UAV poses, status lights, PoI colours,     |
                    |     |  link/route tubes, survey cones, HUD        |
                    +-----|---------------------------------------------+
                          | SwarmSim.step(dt)      (same core runs headless:
                          v                          tools/run_headless.py)
 +-----------------------------------------------------------------------+
 |  uavx/sim.py  SwarmSim: step loop, fault injection, metrics            |
 |                                                                         |
 |   +-------------+   heartbeat (pos, battery, status, survey data)      |
 |   |  agent.py   | ---------------------------+                         |
 |   |  Uav x5     |                            v                         |
 |   |  onboard    |                   +-----------------+                |
 |   |  autonomy   |   comms.py        |    gcs.py       |                |
 |   |             |  Network:         |  GroundStation: |                |
 |   |             |  link graph,      |  fleet picture, |                |
 |   |             |  min-ETX routing, |  loss detection,|                |
 |   |             |  per-hop loss     |  relay/task     |                |
 |   |             | <-----------------|  planner        |                |
 |   +-------------+   command (task + delivery acks)    +-----------------+
 +-----------------------------------------------------------------------+
```

Every packet between a UAV and the GCS, in either direction, goes through
`comms.Network`. It is routed hop by hop over the current link graph, and
each hop can drop it. The GCS never reads UAV state directly. Everything it
knows came in a heartbeat that survived the mesh. This is the core premise
of BVLOS (Beyond Visual Line of Sight): situational awareness only exists
where the network does.

### Why one supervisor instead of one controller per UAV

All UAV logic sits in per-UAV `Uav` objects. Each one acts only on its own
state and on the packets it received. They are stepped from a single
process, for three reasons:

- **Deterministic:** the same seed gives the same run, both in Webots and
  headless.
- **Testable** without Webots (`tests/`).
- **Robust:** no inter-process timing issues in the demo.

The logical separation, and so the information each agent has, is the same
as with separate controllers. Moving each `Uav` into its own Webots/ROS 2
node is a change to the transport layer, not a redesign.

## 2. Who knows what (information flow)

The review's key question: *when every surviving UAV loses its route to
the GCS, what information can each decision-maker still access?*

| Decision-maker | Can use | Cannot use |
|---|---|---|
| **Simulator** (`sim.py`) | Full truth: every UAV's state, the radio graph. Uses it for physics, the link model and scoring. | (it makes no mission decisions) |
| **GCS** (`gcs.py`) | Only heartbeats and imagery chunks that crossed the network (`on_heartbeat`, `on_chunk`). Every value it holds carries the time it was last heard. | UAV objects, positions, batteries or the radio graph directly. `gcs.py` does not import the simulator or the network. |
| **Each UAV** (`agent.py`) | Its own position and battery; command packets that crossed the network; neighbour beacons from UAVs in radio reach (their task kind and whether their mesh fragment has a route; the position of a UAV in a failsafe descent). | The GCS's state, or UAVs it cannot hear. |
| **New GCS commands** | Reach a UAV only as a command packet: routed, lossy, delayed 20 ms per hop. | No shared variables. |

During a partition the GCS keeps planning on its stale picture. It marks
the UAVs LOST after 3 s, and its tasks cannot reach them. The UAVs fall
back on onboard rules (section 5).

**Evidence** (`tests/test_information_flow.py`, run on every change):

- `gcs.py` imports only `agent` (constants and pure energy helpers) and `config`.
- With every radio down, the GCS's knowledge is frozen: no heartbeat
  arrives, and positions, batteries and last-heard times don't change.
  Meanwhile the true state keeps moving.
- A new high-priority site injected during the partition changes the GCS
  plan but reaches no UAV.
- Commands to a UAV h hops away arrive h × 20 ms after sending.
- Over a full mission, GCS heartbeats received equal heartbeats that
  crossed the network, and UAV commands received equal commands that
  crossed it. `results.information_flow.bypass` is `false`, and it is
  `false` in all 90 evaluation runs.

The dashboard follows the same rule. Its fleet table, map, charts, feeds
and site tiles are drawn from GCS knowledge only, with data ages, so
during a partition they freeze and say how old they are. Only the 3D view
shows simulator truth, and it is labelled as such.

## 3. Step loop (`sim.py`, every 64 ms)

1. Apply any scripted or keyboard faults. Restore radios whose outage ended.
2. Rebuild the link graph and minimum-ETX routing tree from current
   positions (`comms.Network.update`).
3. **Uplink**, only for UAVs whose radio works:
   - a heartbeat every 0.5 s (position, battery, status, task, hop count,
     next hop, and which sites it holds data for);
   - an imagery chunk every 0.25 s while it holds unsent data;
   - a camera thumbnail every 1 s while airborne.

   Each packet either survives every hop, in which case it is queued for
   delivery after the path latency, or it is lost.
4. **Deliver** packets whose latency has elapsed, then run the GCS update:
   - declare a UAV lost after 3 s of silence;
   - re-plan every 1 s, and at once after a loss or EMERGENCY report.
5. **Downlink:** each UAV's task plus delivery acknowledgements every 0.5 s,
   subject to the same loss and latency.
6. **Neighbour beacons, separation assurance**, then each UAV's onboard step.
7. **Record metrics** (section 8).

## 4. Communication model (`comms.py`)

| Aspect | Model |
|---|---|
| Link existence | 3D distance ≤ `COMM_RANGE` = 600 m (conservative for a low-power 2.4 GHz mesh radio), both radios up |
| Per-hop packet loss | `p(d) = 0.01 + 0.20·(d/600)^4`: about 1% up close, 21% at the edge |
| Latency | 20 ms per hop (MAC access and forwarding) |
| Capacity | Shared half-duplex channel: 4 Mbit/s per hop, left over for imagery after telemetry. A path of h hops carries 4/h Mbit/s. |
| Traffic | Heartbeat 200 B every 0.5 s; command 300 B every 0.5 s; thumbnail about 20 kB every 1 s; site imagery 1.5 MB (about 8 JPEGs) sent in 0.25 s chunks |
| Reliability | Imagery chunks use stop-and-wait with an end-to-end ack and are retried until delivered. Heartbeats, commands and thumbnails are never retried (the next one supersedes them). |
| Routing | Dijkstra from the GCS with ETX = 1/(1−p) as the link cost, which prefers several reliable hops over one marginal one |
| Relay role | Any UAV that is an intermediate node on another UAV's route. Recomputed every step. |

**Assumptions** (for the limitations section):

- Free-space distance threshold only.
- No obstruction by terrain or rubble, no multipath, no interference.
- Links are symmetric, and capacity is not shared between parallel flows.
- The GCS antenna is at 8 m.

Stage 2 can swap in a statistical channel model or ns-3 behind the same
`Network` interface.

## 5. Onboard autonomy (`agent.py`)

States: `READY` (on the pad, charged), `ACTIVE`, `RTH` (return to home),
`CHARGING`, `EMERGENCY` (failsafe descent), `FAILED`.

| Behaviour | Rule |
|---|---|
| Flight | Kinematic, 15 m/s horizontal, 5 m/s vertical. Climb to its own layer, transit, then descend only over a pad or target. |
| Task execution | `SURVEY`: hover 10 s at a standoff 0.8 × cruise altitude back from the site, then send the imagery. `RELAY`: hold the assigned point. `HOME`: land and swap the battery. |
| Perception | YOLOv8n runs **onboard** on the UAV's own gimbal camera while surveying, whether or not it has a link. The person count is stored with the site's data and travels to the GCS with the imagery. |
| Battery | Drains 0.055%/s in transit and 0.05%/s hovering (about 30 min endurance). **Onboard RTH** when battery ≤ energy home + 15% reserve + 5%. The battery swap at the pad is modelled as a 0.8%/s charge (about 2 min), which is a time-compressed abstraction. |
| Critical fault | Failsafe: stop the task and descend vertically, level, at 2.5 m/s. **Failsafe** variant: the radio stays up, so EMERGENCY goes out in the next heartbeat. **Hard** variant: the fault also takes the radio down, so the GCS can only learn of it by timeout. |
| Lost link: holding unsent data | Silent for more than 3 s → carry the data back toward the GCS (data mule). |
| Lost link: orphaned relay and its fragment | A relay silent for more than 3 s falls back toward the GCS. **Any UAV that can hear an orphaned relay** (beacons show it's in the same mesh fragment and the fragment has no route) falls back with it. Everyone moving toward one point only shortens the links between them, so the fragment contracts as one piece and reconnects in one go instead of UAV by UAV. |
| Lost link: task done | Survey done and imagery acked, but out of contact → come back. |
| Lost link: any UAV | Silent for more than 120 s → come back. |
| Emergency keep-out | Never aim into the descent column of a UAV in a failsafe landing: hold 20 m clear of it. |
| Geofence | Targets are clamped 30 m inside the fence; the position is hard-clamped to the fence. |

## 6. GCS planner (`gcs.py`)

Runs every 1 s on GCS knowledge only.

1. **Order pending sites** by priority, then by distance from the GCS.
2. **Chain building:** from the nearest anchor (the GCS or a planned relay
   slot), `n = ceil(d / 480 m) − 1` relays plus a surveyor. Chains branch
   off earlier relays, so the layout is a tree rooted at the GCS.
3. **Data-mule fallback** for the most urgent site that can't get a full
   chain. Spare UAVs extend the network toward it.
4. **Proactive redundancy (backup relay):** the GCS works out, from the
   next hops UAVs report, which relay carries the most other UAVs. It puts
   a `BACKUP` relay 110 m to the side of that relay, in range of both its
   upstream and downstream neighbours, so a second disjoint path exists.
   Two policies (`BACKUP_POLICY`):
   - `spare` (default): only when a UAV is left over.
   - `reserve`: always hold one UAV back for this when a relay carries at
     least 2 others.

   Section 9 measures both.
5. **Energy-constrained assignment:** permutation search (at most 5! = 120)
   minimising flight distance, with stickiness. A UAV is only eligible for
   a new slot if

   `E_remaining > E_transit + E_on_station(120 s) + E_return + E_reserve(15%)`

   If no feasible assignment fills every slot, the least urgent slot is
   dropped.
6. **Energy-driven relay handover:** once a relay's margin
   (battery − (return + reserve)) falls below 12%:
   - the GCS dispatches a replacement to that slot;
   - the outgoing relay keeps forwarding until the replacement is within
     60 m of the slot and linked;
   - then the outgoing relay is released home.

   Each handover gets an outcome:
   - relieved on station;
   - released because the slot is no longer needed (its site finished
     first);
   - recalled at mission end;
   - ended early by the onboard RTH or a fault.

   The onboard RTH (margin < 5%) stays as the backstop.
7. **Fault handling:**
   - a UAV silent for 3 s is declared lost;
   - an EMERGENCY report triggers an immediate re-plan;
   - silent surveyors keep their slot for 240 s;
   - silent relays are replaced at once.
8. **New sites** can arrive at any time. When every site's imagery is at
   the GCS, all UAVs are recalled.

`FixedRelayStation` is the **baseline** for the evaluation. It lays two
relays toward the area centre once and never re-plans them. The other UAVs
survey one site each and ferry the data back. Everything else (UAVs,
onboard rules, network) is shared.

## 7. Separation assurance (`sim.py` + `agent.py`)

- **Altitude layers:** UAV i cruises at 60 + 8(i−1) m (60–92 m, below the
  120 m ceiling), so transits never conflict.
- **Predictive vertical yield:** a UAV climbing or descending toward
  another UAV's altitude stops short of it while they are within 20 m
  horizontally and 12 m vertically. It never pauses inside another layer.
  If both are changing altitude, the higher id yields.
- **Emergency keep-out:** a failsafe descent can't manoeuvre, so the
  others keep 15 m of horizontal clearance from its column. They don't aim
  into it (onboard) and are pushed out of it (fallback).
- **Fallback:** horizontal repulsion below 5 m, with a fixed per-UAV
  direction if two UAVs are exactly stacked.

The randomised evaluation found a collision (two UAVs at 0.0 m in one of
30 scenarios): a replacement relay was sent to the exact spot where the
failed relay was descending. The keep-out and predictive yield were added
for that case, and `test_emergency_descent_column_is_kept_clear` guards it.

## 8. Metrics (`sim.results()`, our definitions)

| Metric | Definition |
|---|---|
| Site complete | The site's **full imagery bundle** (1.5 MB) has reached the GCS. Visited, imagery delivered, people detected and detection correctness are reported separately per site (`mission.sites`). |
| `all_delivered_at_s` / `fleet_landed_at_s` | Last site's imagery at the GCS / every surviving UAV on its pad. These are two separate milestones. |
| `survey_to_gcs_latency_s`, `sites_within_deadline` | Survey finished → imagery complete at the GCS; "timely" means within 60 s |
| `connectivity_pct` | Time average of (airborne UAVs with a route / airborne UAVs) |
| `disconnected_uav_s`, `partition_time_s`, `partition_episodes` | Airborne UAV-seconds without a route; seconds with at least 1 UAV cut off; how often that started |
| `single_point_of_failure_time_pct` | Share of airborne time in which some UAV's loss alone would cut another UAV off (articulation points) |
| **PDR** per class (`hb`, `cmd`, `data`, `thumb`) | **Unique packets received at the endpoint / unique packets sent, end to end.** A packet sent with no route counts as lost. Data-chunk retries count as transmissions, not new packets. Heartbeats, commands and thumbnails are never retried. `delivered_per_routed_attempt_pct` separates link quality from reachability. |
| Recent PDR | The same ratio over the last 10 s; **N/A** if nothing was sent |
| Fault `detected` | Fault → the GCS flags it (EMERGENCY heartbeat or 3 s timeout) |
| Fault `orphaned` | Surviving UAVs that had a route before the fault and lost it |
| Fault `first_reconnect` / `all_reconnected` | → the first / all orphaned UAVs have a route (possibly briefly) |
| Fault `stable_recovery` | → start of the first **5 s** window in which every survivor keeps a route (our definition) |
| Fault `flaps` | Times the fleet split again after being whole, before stable recovery |
| Fault `disconnected_uav_s` | Survivor-seconds without a route, from the fault to stable recovery |
| Fault `telemetry_restored` / `next_delivery` | → the GCS has heard every orphaned survivor again / → the next site's imagery completes |
| `energy_handovers[]` | Start time and margin, replacement, on-station and release times, battery at release and at landing, outcome |
| Safety | Minimum 3D separation; near-miss episodes (< 5 m) and collision episodes (< 1.5 m), counted per pair per episode; minimum terrain clearance while cruising; geofence violations; depletions; battery at every landing. Per-second log in `safety_*.csv`. |

### Results: scripted scenario (seed 7, `python tools/run_headless.py`)

Events:

| Time | Event |
|---|---|
| 150 s | Hard fault (motor + radio) on the busiest relay |
| 220 s | New high-priority site H1 |
| 300 s | 45 s radio outage on the deepest UAV |
| 360 s | Battery cell fault (−52%) on the GCS-side relay |

| | No faults | Scripted scenario |
|---|---|---|
| Sites with full imagery at GCS | 7/7 at 306 s | 8/8 at 664 s |
| Surviving fleet landed | 447 s | 808 s |
| Survey → GCS latency, mean / max | 17 / 47 s | 24 / 69 s (7/8 within 60 s) |
| Connectivity | 95.3% | 91.8% |
| Disconnected UAV-s / partition episodes | 105 / 5 | 239 / 16 |
| Heartbeat PDR end to end / per routed attempt | 84.5% / 89.5% | 82.1% / 87.7% |
| Imagery chunks delivered | 100% | 100% (276 chunks, 417 transmissions) |
| Time with a single point of failure | 70% | 80% |
| Min separation / near misses / collisions | 6.0 m / 0 / 0 | 6.0 m / 0 / 0 |
| Geofence violations / depletions | 0 / 0 | 0 / 0 |

**Hard fault on UAV3, which was relaying for 4 UAVs:**

| Measure | Value |
|---|---|
| Detected (timeout) | 2.9 s |
| Orphaned | 4 |
| First reconnect / all reconnected | 14.9 s / 14.9 s |
| Stable recovery | 14.9 s |
| UAV-seconds disconnected | 60 |
| Re-splits | 0 |
| Next site delivered | 75 s after the fault |

**Radio outage (45 s):** the GCS flags it after 2.8 s and re-acquires the
UAV 0.12 s after the radio returns.

**Energy handover (UAV2, cell fault 81% → 29%):**

| Measure | Value |
|---|---|
| Handover starts | 4.2 s after the fault (margin 12.0%) |
| Replacement | UAV4, on station and linked 21.5 s later |
| UAV2 released at | 28.2% (needs 17%) |
| UAV2 landed with | 25.9% |

The Webots run (`results/metrics_webots.json`) uses the same core. It
differs slightly because it adds terrain height and real onboard detection
counts.

## 9. Generalisation (`python tools/evaluate.py`)

30 random scenarios per planner. Each draws:

- 7 site positions (0.5–2.4 km from the GCS);
- initial batteries of 45–100%;
- a critical fault at 100–260 s: hard or failsafe, on the busiest relay or
  a random UAV;
- a random high-priority site;
- a 15–60 s radio outage;
- a 35–55% cell fault on the GCS-side relay.

Values are mean (worst case). Full table: `results/evaluation.md`.

| | adaptive-spare (default) | adaptive-reserve | fixed-relay baseline |
|---|---|---|---|
| Success (all sites, no collision, depletion or fence breach) | 30/30 | 30/30 | 30/30 |
| All imagery delivered (s) | 528 (771) | 697 (1164) | 692 (1713) |
| Connectivity % | 92.1 (82.4) | 87.9 (81.0) | 62.2 (29.9) |
| Heartbeat PDR end to end % | 82.6 (72.6) | 78.9 (71.1) | 55.3 (27.3) |
| UAVs orphaned by the fault | 1.2 (4) | 0.1 (2) | 0.7 (4) |
| Stable recovery after the fault (s) | 10.2 (135.7) | 6.4 (101.9) | 105.0 (566.6) |
| UAV-s disconnected by the fault | 25.7 (173.6) | 7.5 (85.6) | 270.6 (1939.4) |
| Time with a single point of failure % | 70.3 | 53.5 | 46.6 |
| Min separation (m) | 10.8 (4.3) | 10.0 (4.3) | 6.3 (3.0) |
| Collisions / near misses < 5 m | 0 / 1 | 0 / 1 | 0 / 9 |
| Energy handovers | 15 | 22 | 0 |
| Information-flow bypasses | 0 | 0 | 0 |

Reading the table:

- The adaptive planner beats fixed relays on every network and recovery
  measure.
- **Holding a backup relay in reserve nearly removes fault-induced
  disconnection** (0.1 UAVs orphaned on average), but it costs about 170 s
  of mission time and 4 points of connectivity, because one UAV fewer
  serves the sites.
- The default (`spare`) is the better all-round policy. The trade-off is
  a configuration choice (`BACKUP_POLICY`) that can be made per mission.
- Worst cases are reported, not hidden. The single-point-of-failure share
  stays high for the adaptive planners because long chains are what reach
  2.4 km with five UAVs.

## 10. Visualisation and perception layer

This layer only reads the core's state, apart from the onboard detection
count it writes into the UAV (which the UAV then transmits). The headless
metrics are what the 3D run produces, minus the detections.

| Part | What it does |
|---|---|
| `tools/make_world.py` | Builds the 4.8 km scene from `config.py`: terrain, ten villages, roads, landslide, and survivors at each site (counts from `config.SURVIVORS_GT`, which are also used to score perception). |
| `Scene` (controller) | Drone poses, props, role LEDs, altitude lines, survey cone, routing-tree and radio-link tubes (thin, sized on screen), site flags. **Simulator truth.** |
| `Cameras` + `Feeds` + `vision.py` | One gimbal camera per UAV. YOLOv8n runs **onboard**, for surveying UAVs first, whether or not they have a link. A GCS feed changes **only when a thumbnail packet crosses the mesh**; otherwise it shows "NO NEW FRAME · last N s ago". |
| `dashboard.py` | 1920×1080 GCS view built from GCS knowledge: fleet as last reported with "heard N s ago", 10 s and total heartbeat PDR (N/A when nothing was sent), oldest-data age, GCS map, and site tiles showing "full imagery at GCS" separately from "people detected / ground truth". |
| `narrator.py` | Video captions per disturbance: WHAT FAILED / GCS KNOWS / DECISION / RESULT, filled from the event log and GCS knowledge. |
| `director.py` | Recording camera. It holds a **stable wide overview of the whole fleet** from 7 s after the critical fault until 8 s after stable recovery, and a locked wide shot for the relay handover. |
| `cards.py`, `tools/compose_video.py` | Intro card (what changed); results cards (this run, and the 30-scenario evaluation). ffmpeg composites the 4× time-lapse 3D view, the GCS panels, the caption band (keyed out when empty) and the cards. |

## 11. Tests

`python -m unittest discover -s tests -v` runs 19 tests.

| Test | Covers |
|---|---|
| `test_information_flow.py` (6) | Section 2: GCS isolation; GCS knowledge frozen and commands blocked during a partition; latency per hop; no bypass over a full mission; PDR counts unrouted packets and reports N/A |
| `test_core.py` link model (3) | Loss grows with distance; multi-hop routing; out-of-range |
| `test_core.py` agent (4) | Geofence clamp; RTH before depletion; battery never negative; failsafe landing |
| `test_core.py` scenario (6) | Failsafe fault detection and recovery; hard fault detected only by timeout; energy-driven handover; emergency keep-out regression; new site tasked; full scripted mission (8/8, no collision, handover relieved on station, delivery before landing) |

## 12. Limitations and Stage 2 path

| Limitation | Stage 2 path |
|---|---|
| Kinematic flight; the emergency landing is vertical and in place | PX4 SITL + Gazebo, or the physics-based Webots Mavic controller |
| Distance-threshold radio without terrain shadowing; capacity per path, not per shared neighbourhood | Obstruction-aware statistical model, or ns-3, behind the same `Network` interface |
| Fault times are scripted in the demo | The evaluation randomises them; Stage 2 adds random fault processes |
| Centralised planner (with onboard fallbacks) | Distributed auction or consensus |
| Battery swap time-compressed (0.8 %/s) | A realistic swap turnaround |
| Perception scored against 8 scripted sites with a COCO model | Train on aerial SAR imagery; score precision and recall on randomised crowds |
| The fixed-relay baseline is one simple design | Add stronger baselines (e.g. a static k-connected backbone) |
