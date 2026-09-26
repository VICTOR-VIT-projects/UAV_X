# Metric specification (v3.1)

Every number in the video, `results/metrics_*.json`, the result cards and
`results/evaluation.md` is defined here. Units are simulation seconds (s),
metres (m), percent (%) and UAV-seconds (UAV-s = number of UAVs × seconds).
All definitions are **ours** (internal test definitions). UAV-X has not
published Stage 1 metric definitions.

Key metrics are recomputed from the raw logs by `tools/recompute_metrics.py`,
which does not import the simulator. It runs after every Webots recording,
and the results card states how many checks passed.

## 1. Who measures what

| Actor | Sees | Used for |
|---|---|---|
| Evaluator (`uavx/metrics.py`) | Simulator truth: positions, radio graph, statuses | Scoring only. `gcs.py` and `agent.py` cannot import it (`tests/test_information_flow.py`). |
| GCS (`uavx/gcs.py`) | Packets delivered by the channel | Decisions. Also the "GCS-confirmed" milestones. |

## 2. Populations (per fault event, frozen at the fault)

| Population | Definition |
|---|---|
| `faulted` | The aircraft the fault was injected into. |
| `before_fault_routed` | Other UAVs that were ACTIVE, had their radio up and had a route to the GCS at the step before the fault. |
| `orphaned` | Members of `before_fault_routed` without a route at the first observation after the fault. |
| `unaffected` | `before_fault_routed` minus `orphaned`. |
| `required` | `orphaned`, plus the `faulted` aircraft if the fault is **temporary** (radio outage). A critical fault is an **irrecoverable aircraft loss**: that aircraft is reported separately and is never in `required`. |

**Membership changes** are logged as `(t, uav, reason)` and happen only
when a required UAV:

- **lands**, which means it has left the mission; or
- suffers **a later critical fault**, in which case it has left the event.

A UAV is **never** removed because it is still disconnected.

## 3. Recovery timestamps

Each timestamp is measured in seconds after the fault time `t0`. `None`
means it did not happen, and it is never reported as 0.

| Field | Definition |
|---|---|
| `detected` | The GCS flags the faulted UAV: first `declared_lost` or `emergency_reported` event for it at or after `t0` (from `gcs_events_*.csv`). Timeout detection counts from the last heartbeat received, which can precede `t0` by up to one heartbeat period (0.5 s). |
| `first_route` | First observation at which at least one `required` UAV has a route again. |
| `all_routes` | First observation at which every `required` UAV has a route. This may be brief. |
| `stable_start` | Start of the first interval in which every `required` UAV kept a route for `STABLE_WINDOW` = 5 s. Any disconnection of a required UAV resets the hold. |
| `stable_confirmed` | `stable_start + 5 s`. This is the earliest time stability can be known. |
| `first_app_delivery` / `all_app_delivery` | The GCS receives a heartbeat from the first / every required UAV after its route loss (application-level service restored). |
| `flaps` | Number of times the required population was whole and then split again before `stable_start`. |
| `required_disconnected_uav_s` | Σ over steps of (required UAVs without a route) × 64 ms, from `t0` until the event closes. |
| `unaffected_continuity_pct` | Share of the event window in which every unaffected UAV kept its route. This describes the unaffected network only, never whole-fleet recovery. |

**Outcomes:**

- `recovered: required population stable`
- `unaffected (0 of N other UAVs lost their route)` (with the count N)
- `not stable within 180s attribution window`
- `unrecovered at end of run`
- `undetermined: every required UAV left the mission before recovery`

Overlapping faults are listed in `overlapping_faults`.

**Radio outage.** The outage duration (`duration`) is the injected
radio-off time. Reacquisition after restoration is
`all_app_delivery − duration`, which is typically a few tenths of a second.
These are reported separately and never merged.

## 4. Connectivity, partitions and critical relays

**Mission UAV:** status ACTIVE or RTH, meaning airborne and not in a
failsafe descent.

| Metric | Formula |
|---|---|
| `connectivity_pct` | mean over steps with ≥ 1 mission UAV of (mission UAVs with a route / mission UAVs) |
| `disconnected_uav_s` | Σ over steps of (mission UAVs without a route) × 64 ms |
| `network_state_pct` | Share of time with ≥ 1 mission UAV spent in each of three exclusive states: **partitioned** (some mission UAV has no route), **connected_single_point** (all routed, but some UAV is a critical relay), **connected_redundant** (all routed, no critical relay). A disconnected network is counted as *partitioned*, never as "safe". |
| critical relay | A UAV whose removal alone would cut at least one currently routed mission UAV off from the GCS (an articulation point of the undirected link graph, with drained relays unable to forward). `worst_critical_relay_dependants` is the largest such count. |

## 5. Outage episodes (ledger)

**Per-UAV route episode.** A mission UAV's continuous time without a route.
Gaps with a route shorter than `EPISODE_MERGE_GAP` = 1 s are merged into
one episode. The episode closes when:

- the route is regained;
- the UAV leaves the mission (lands or has a critical fault); or
- the run ends.

**Mission partition episode.** The union over UAVs: it starts when the
first UAV loses its route and ends when every mission UAV has a route again.

**GCS silence episode.** From the GCS declaring a UAV lost to the next
heartbeat it receives from it. Classification by the fraction of the
silence during which the modelled graph had a path:

| Path present during the silence | Kind |
|---|---|
| ≥ 90% | `heartbeat timeout with a path present (loss/queue)` |
| ≤ 10% | `graph disconnection` |
| otherwise | `mixed` |

The cause rules are in `docs/OUTAGE_LEDGER.md`. **Fault-attributed**
outage time covers episodes whose cause is `fault-attributed: upstream
relay lost` or `injected radio outage`. Everything else is reported
separately as not fault-attributed. Later outages are never hidden behind a
successful initial recovery.

## 6. Traffic, PDR and latency

Everything below is measured per class: `hb` (heartbeat), `cmd`, `ack`,
`data` (imagery chunk) and `thumb`. Counts are **unique packets** unless
stated otherwise.

| Field | Definition |
|---|---|
| `generated` | Unique packets created by the application. |
| `delivered` | Unique packets that reached their endpoint. A data chunk counts once even if several copies arrive; copies go to `duplicate_deliveries`. |
| `expired` | Control packets (hb, cmd, thumb, ack) that waited past their deadline (hb, cmd and thumb 2 s; ack 5 s) and were discarded. |
| `dropped` | Tail drop at a full queue (64 packets per node). |
| `dropped_radio_down` | Generated while the sender's radio was off. |
| `lost` | A hop failed `1 + MAC_RETRIES` = 4 times. |
| `pending_end` | Still queued or in flight when the run ended. |
| `hop_tx` | Every transmission attempt on every hop, retries included (airtime). |
| `e2e_retransmissions` | Imagery chunks resent after a 4 s end-to-end ack timeout. These count as transmissions, never as new unique packets. |

For every class except data:
`generated = delivered + expired + dropped + dropped_radio_down + lost + pending_end`.
This is tested in `tests/test_channel.py::Reconciliation`.

**PDR (cohort).**

```
PDR = delivered within the class deadline / generated
```

This is computed over unique packets whose deadline has already passed at
evaluation time ("matured"), grouped by *send time*, so a late receipt is
never divided by unrelated sends. The **10 s window** covers packets sent
during the 10 s before maturity. An empty cohort is reported as **N/A**,
never 100%.

A packet generated with no route is counted in the denominator (as expired
or dropped). Data chunks have no deadline, so they have no PDR. Their
timeliness is measured per site instead (section 7).

**Latency.**

- Per class, delivery time minus generation time: p50, p95 and max.
- **Command application latency** (`command_ack_latency_s`): from the GCS
  sending command *s* to the GCS receiving a heartbeat that reports
  `cmd_seq = s`. Commands overtaken by a newer one before delivery are
  counted in `commands_superseded_before_delivery`, not in the latency.

## 7. Mission, milestones and constraints

| Metric | Definition |
|---|---|
| Site complete | All `N_CHUNKS` = 46 imagery chunks (46 × 32 KiB ≈ 1.5 MB) have reached the GCS. Survey coverage, imagery delivery and person-count correctness are separate fields in `mission.sites`. |
| `all_delivered_at_s` | Last site complete (GCS knowledge). |
| `fleet_landed_at_s_observer` | Every surviving UAV is on its pad (simulator truth). |
| `fleet_landed_confirmed_by_gcs_s` | Every UAV the GCS can still hear reports it has landed. `unaccounted_by_gcs` lists UAVs the GCS never heard from again. The observer can know a landing before the GCS does. |
| `survey_to_gcs_s` | Survey complete → imagery complete. **Internal** timeliness benchmark: 60 s. |
| `high_priority_response_s` | High-priority report → its imagery complete. |

**Predeclared internal constraints** (`config.CONSTRAINTS`, fixed on
2026-09-25 before the benchmark was re-run):

| Constraint | Threshold |
|---|---|
| All imagery at the GCS | ≤ 900 s |
| Fleet landed (observer) | ≤ 1200 s |
| High-priority report → imagery | ≤ 240 s |
| Sites within 60 s | ≥ 75% |
| Controller-caused separation below 5 m | none |
| Collision proxy (< 1.5 m) | none |
| Geofence violations | none |
| Battery depletions | none |
| Every landing battery | ≥ 15% |

A run can be `eventually_complete` and still fail
`operational_success`. Every run stays in the denominator. Runs that hit
`MAX_TIME` are counted as `time_limited`.

## 8. Safety

These are geometric proxies on kinematic trajectories. There is no
physics collision detection.

- **Separation:** minimum 3D distance between every pair of airborne UAVs,
  checked **continuously** between 64 ms samples (closest approach of two
  linearly moving points, `metrics.seg_min_dist`).
- **Near-miss episode:** one pair closer than 5 m. Consecutive steps form
  one episode, which reports duration, minimum distance, flight phases
  (takeoff, landing, cruise, return, relay replacement, climb/descent,
  failsafe descent) and cause:
  - **fault consequence:** one UAV is in a failsafe descent;
  - **controller:** otherwise.

  Both kinds stay in the raw log.
- **Collision proxy:** an episode whose minimum distance is below 1.5 m.
- **Also reported:** violation time (Σ episode durations), geofence
  violations (steps outside the fence), depletions (battery reached 0%),
  landing batteries, and minimum terrain clearance while cruising.
