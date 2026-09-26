# Response to the technical review of the v2 demo

Each review point below says what changed in `webots_3/` and where the
evidence is. Numbers come from `results/` and can be regenerated:

```bash
python tools/run_headless.py      # scripted scenario  -> results/metrics_scenario.json
python tools/evaluate.py          # 30 random scenarios x 3 planners -> results/evaluation.md
python -m unittest discover -s tests -v   # 19 tests
```

Video: `media/uav_x_3.mp4` (copied to the Desktop as `uav_x 3.mp4`).

---

### 1. "Does communication loss actually constrain the controller?" (highest priority)

**Audited, and it does.** There is no hidden bypass.

**Information flow:**
- The GCS (`gcs.py`) has exactly two inputs: heartbeats and imagery chunks
  that crossed the simulated network. `gcs.py` does not import the
  simulator or the network.
- Commands reach a UAV only as packets on the same lossy, multi-hop route,
  now with 20 ms per hop of latency.

**Tests** (`tests/test_information_flow.py`) prove it:
- with every radio down, the GCS's knowledge freezes while the truth keeps
  changing;
- a new site injected during the partition changes the GCS plan but
  reaches no UAV;
- commands arrive h × 20 ms after sending;
- over a full mission, messages received equal messages that crossed the
  network (`information_flow.bypass = false` in all 90 evaluation runs).

**What each decision-maker knows during an outage:** see the table in
`ARCHITECTURE.md` §2.

**The dashboard was part of the problem the review saw.** In v2 it showed
simulator truth (batteries and roles kept changing during the partition).
It is now built only from GCS knowledge:
- every UAV row shows "heard N s ago" and freezes as LINK LOST;
- the "oldest data" KPI goes red;
- camera feeds update only when a thumbnail packet gets through, and
  otherwise show "NO NEW FRAME · last N s ago".

The 3D view is labelled "SIMULATOR TRUTH (observer)".

### 2. "Your recovery claim looks too generous" (transient vs stable)

**New metrics** for each fault (`faults[]` in the metrics JSON; definitions
in `ARCHITECTURE.md` §8):
- `first_reconnect`;
- `all_reconnected`;
- `stable_recovery`: every survivor keeps a route for 5 s. This is our
  definition, labelled as such.
- `flaps`: re-splits before stable recovery;
- `disconnected_uav_s`;
- `telemetry_restored`;
- `next_delivery`.

A new `RECOVERY: mesh split again` event is logged and captioned.

**Single point of failure:**
- It is now measured: `single_point_of_failure_time_pct` (articulation
  points of the live mesh).
- It is made worse on purpose. The scripted fault is now a **hard** fault
  (motor + radio) on the relay carrying 4 UAVs, so the GCS only learns of
  it by timeout.

**Recovery policy (measured):**
- **Onboard fragment contraction.** An orphaned relay, and every UAV that
  can hear it, falls back toward the GCS together, so the fragment
  reconnects as one piece. This cut stable recovery for that fault from
  47.6 s to **14.9 s**, and disconnection from 156 to **60 UAV-s**, with
  **0 re-splits**.
- **Proactive backup relay.** A `BACKUP` UAV sits beside the busiest
  relay. It is a policy (`BACKUP_POLICY`); `reserve` always keeps one.
  Over 30 random scenarios, `reserve` cuts fault-orphaned UAVs from 1.2 to
  0.1 on average, but costs about 170 s of mission time, because one UAV
  fewer serves the sites. Both results are reported (§4 below).

### 3. "The PDR display needs an audit"

**PDR = unique packets received / unique packets sent, end to end:**
- per traffic class (heartbeat, command, imagery, thumbnail);
- a packet sent with no route counts as **lost**;
- imagery retries count as transmissions, not new packets;
- heartbeats and commands are never retried;
- measured at the endpoint (GCS for uplink, UAV for downlink).

The old display showed delivered/routed, which ignored packets with no
route. That is why it read about 92% during a total outage. It is still
reported, but separately, as `delivered_per_routed_attempt_pct` (link
quality).

**Dashboard KPIs:**
- **heartbeat PDR over the last 10 s** (N/A when nothing is sent; it drops
  to 0% during the partition);
- **cumulative PDR**;
- **GCS link count**;
- **oldest data age**.

**Timeliness of imagery:**
- `survey_to_gcs_latency_s` (mean 24 s, max 69 s);
- `sites_within_deadline` (7/8 within 60 s).

Eventual delivery and timely delivery are reported separately.

### 4. "The emergency-site result exposes a perception weakness"

- **Cause of "0 ppl":** in v2, detection ran only on frames that reached
  the GCS live. A surveyor working out of range (data mule) was never
  scored.
- **Detection now runs onboard** on the surveyor's own camera, link or no
  link. The count travels with the imagery.
- Each site tile and `mission.sites` keep these separate:
  - surveyed (time, UAV);
  - full imagery at GCS (time);
  - people detected (onboard);
  - **ground truth** (the survivors placed in the world,
    `config.SURVIVORS_GT`).
- Completion is defined explicitly as full imagery at the GCS, and that is
  written on the dashboard and in the results.
- Detection is scored **per site**, so over- and under-counts can't cancel
  out. In the recorded Webots run, 5/8 sites were counted exactly (P1, P2,
  P5, P7, H1: 3/3, 2/2, 2/2, 4/4, 3/3). There was 1 person missed (P3:
  3/4) and 2 extra detections (P4 and P6: 4/3 each). This is
  `perception.sites_exact`, `persons_missed` and `persons_extra` in
  `results/metrics_webots.json`. The summed total (25 vs 24) would hide
  this, so it is not the headline.

### 5. "Battery management is shown, but the difficult case isn't"

**New energy-driven relay handover** (`gcs.py`, `ARCHITECTURE.md` §6):
- assignment requires
  `E_remaining > E_transit + E_on_station + E_return + E_reserve`;
- a relay whose margin (battery − return − reserve) drops below 12%
  triggers a handover;
- the replacement is dispatched; the outgoing relay keeps forwarding until
  the replacement is on station and linked, then is released.

**Scripted demo:** a cell fault (81% → 29%) at 360 s on the GCS-side relay
carrying 3 UAVs, while remote surveys are still running.

| Measure | Value |
|---|---|
| Handover starts | 4.2 s after the fault |
| UAV4 on station | 21.5 s later |
| UAV2 released at | 28.2% (needs 17% incl. reserve) |
| UAV2 landed with | **25.9%** |

Tested by `test_energy_driven_relay_handover`, and by the full-scenario
test (outcome "relieved on station", landed ≥ reserve).

**Time scaling is disclosed:** the pad charge is a battery swap modelled
at 0.8 %/s (about 2 min). This appears in a video caption, on the results
card and in the docs.

### 6. "Several important claims remain unproven"

| Area | Now |
|---|---|
| Communication model | Packet sizes, 4 Mbit/s shared channel (rate/hops), 20 ms/hop latency, per-hop loss, stop-and-wait retries, min-ETX routing: `ARCHITECTURE.md` §4 |
| Camera transmission | Imagery is 1.5 MB per site in chunks that consume path capacity. Feeds are 20 kB thumbnails at 1 Hz, each a packet that can be lost. |
| Relay selection | Planner rules §6; adaptivity measured on 30 random layouts against a fixed-relay baseline |
| Safety | Min separation, near-miss (< 5 m) and collision (< 1.5 m) episodes, terrain clearance, geofence, depletions, landing batteries; per-second `results/safety_*.csv` |
| Generalisation | `tools/evaluate.py`: random layouts, fault type/target/time, outage, cell fault, initial batteries. Mean **and worst case**. |
| Reproducibility | Seeded; `README.md` has install and run instructions; every run writes metrics, timeline, safety and event logs |

**The evaluation found a real bug.** A replacement relay was sent to the
exact spot where a failed relay was descending: a collision at 0.0 m, in 1
of 30 scenarios. Fixes:
- an emergency keep-out column;
- a predictive vertical yield;
- a regression test.

Result: 0 collisions in 90 runs. Near misses (< 5 m) went from 5 to 1 per
adaptive planner.

### 7. "Scheduled failures vs general autonomy"

The demo keeps scripted times so the video is watchable. The evaluation
randomises all of the following, and nothing in the planner or onboard
logic knows the script:
- fault time, type (hard or failsafe) and target (busiest relay or random
  UAV);
- outage time and length;
- cell-fault size;
- site layout.

### 8. Presentation

**Captions** for every disturbance:
- WHAT FAILED / GCS KNOWS / DECISION / RESULT;
- a stable wide overview of the whole fleet during recovery and during
  the handover;
- thinner link tubes;
- larger event log.

**End cards:**
- **this run's results**, with "imagery complete at 664 s" and "surviving
  fleet landed at 808 s" as separate milestones;
- **the 30-scenario comparison**, with the baseline and worst cases.

The time-lapse is 4× (was 5×).

### Still open (stated in `ARCHITECTURE.md` §12)

- kinematic flight;
- no terrain shadowing in the radio model;
- centralised planner with onboard fallbacks;
- perception scored on 8 fixed sites with a COCO model;
- one simple baseline.
