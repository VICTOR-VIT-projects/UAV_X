# Operator completion report: UAV-X v3 fix brief (24 Sep 2026)

**Scope:** `webots_3/` (now v3.1).

**Preserved baseline:**

- `../_baseline_v3_recorded/`: the exact recorded v3 code and results.
- `../_backup_webots_3_pre_review/`: the earlier v2-review state.

**Nothing was submitted, published or sent to organisers.**

**New video:** `media/uav_x_4.mp4` (also `Desktop/uav_x 4.mp4`), 3 min 50 s,
seed 7, recorded in Webots R2025a. Every number on its result cards comes
from files written by that run. 16/16 key metrics were recomputed from the
run's raw logs by an independent script (`results/validation_webots.json`).

## 1. What changed, why, what was measured, what remains open

| | |
|---|---|
| **Changed** | Recovery metrics with explicit populations. Per-episode outage ledger. A hop-by-hop shared-airtime channel with sequence numbers, expiry, acks and retries. Make-before-break relay handover and retirement. Three planner defects found by the ledger and benchmark and fixed. Safety episodes with continuous closest-approach checks, and a take-off separation fix. A fair benchmark with predeclared constraints, a held-out set and a fresh set. Negative isolation tests. Perception counting rule chosen on a split benchmark. Presentation labels. Raw logs and an independent recomputation tool. |
| **Why** | Every item in the brief. Four defects were *found* while doing the work and would otherwise have stayed hidden: data-mule range-edge dithering, high-priority starvation, break-before-make relay retirement, and near misses at take-off. |
| **Measured** | Seed-7 regression. 120 saved scenarios × 4 planners × 3 experiments (960 runs incl. variants and the fresh set). A fresh 30-scenario set. Before/after on 60 scenarios with the unmodified recorded code. 72-view perception benchmark. **65 unit, scenario and negative tests, all passing.** |
| **Open** | Degraded handovers when the replacement is too late (4/28 events; worst 248 s dependant outage). Break-before-make during ordinary re-plans (about 5 UAV-s/run). Tightly packed survivors are undercounted (about 10% of persons). Compounding-fault capacity limit on high-priority response (1/30 fresh, 1/30 each E2 set). Degraded-radio sensitivity (13/30). Kinematic flight, uncalibrated radio. See §5. |

## 2. Finding-by-finding disposition

Classification:

| Code | Meaning |
|---|---|
| CD | confirmed defect |
| AR | ambiguous reporting |
| ML | model limitation |
| AL | already resolved |
| UN | unresolved |

### Item 1 · P0 · Recovery populations and timestamps — CD, fixed

**Reproduced.** The recorded code, re-run, reports the radio outage's
`stable_recovery = 0.0` s with `orphaned = 0`. The faulted aircraft was
excluded from its own event (`_baseline_v3_recorded/.../sim.py::_snapshot_connected(exclude=uid)`).
`stable_recovery` was the start of the stable interval, not its
confirmation.

**Change.** `uavx/metrics.py::FaultTracker`:

- populations `faulted / orphaned / unaffected / required`, frozen at the
  fault, with logged membership changes;
- temporary outages keep the faulted aircraft in `required`;
- aircraft loss is reported as irrecoverable;
- the hold resets on any required disconnection;
- `first_route`, `all_routes`, `stable_start`, `stable_confirmed`,
  `first_app_delivery` and `all_app_delivery` are separate timestamps;
- "unaffected" is reported with a count, not "0 s";
- explicit outcomes, and overlaps are recorded.

Specification: `docs/METRICS.md` §2–3.

**Evidence (seed 7):**

| Event | Result |
|---|---|
| Radio outage, UAV1, 45 s | `all_routes` 45.1 s · `stable_start` 45.1 s · `stable_confirmed` 50.1 s · GCS hears it again 45.6 s. Reacquisition after restore ≈ 0.6 s; the outage duration is reported separately. |
| Hard fault | Orphaned 4 · first/all routes 15.1 s · stable 15.1 s → confirmed 20.2 s. |

**Tests:** `tests/test_recovery_metrics.py` (7):

- a temporary outage is not recovered while the radio is off;
- a 2 s reconnection fails the 5 s hold;
- irrecoverable loss of a leaf is "unaffected (0 of N)";
- ordered timestamps;
- overlaps;
- unrecovered outcome;
- outage duration vs reacquisition.

Timestamps are recomputed from raw logs by `tools/recompute_metrics.py`
(all pass).

### Item 2 · P0 · Diagnose each partition — CD (one cause), plus ledger built

**Ledger:** `results/ledger_<tag>.json`, rules in `docs/OUTAGE_LEDGER.md`.

**Reproduced.** The recorded run's post-handover losses (UAV5 LOST and
re-acquired 12 times, then a 51 s gap). A per-second trace shows
**range-boundary dither of a data mule**:

1. the UAV is carrying imagery back;
2. it hears one command at the range edge, so its silence timer resets;
3. it flies back out, and the cycle repeats.

**Fixed** with data-mule hysteresis (`agent.py`). The fix was **not** made
by raising timeouts: `LOST_TIMEOUT` is still 3 s.

**A/B, same definitions and channel** (`run_headless.py --mule-fix off`,
seed 7):

| Measure | Fix off | Fix on |
|---|---|---|
| GCS silences | 18 | 7 |
| Worst survey → GCS | 93.9 s | 52.2 s |
| Sites within 60 s | 7/8 | 8/8 |

**Reconciliation** of 16 episodes / 239.4 UAV-s → 4 mission (7 per-UAV)
episodes / 227.1 UAV-s: the merge definition, the channel model and the
fix, separated in `docs/OUTAGE_LEDGER.md`.

Also separated:

- **graph disconnection vs heartbeat timeout with a path present:** 0 of
  the 7 seed-7 silences had a path present; the case is tested under
  extreme loss;
- **fault-attributed (105 UAV-s) vs other (122 UAV-s, planned survey
  legs).**

**Open:** "relay repositioning / cascade / upstream moving" is a confirmed
break-before-make during ordinary re-plans, at about 5–10 UAV-s/run. Not
fixed. "Unknown" is 1.8–4.1 UAV-s/run and is kept as unknown.

### Item 3 · P1 · Relay handover preserves service — CD, fixed

**Before:** release happened on "on station" alone.

**Now** (`gcs.py::_handover`):

1. dispatched;
2. on station (≤ 60 m);
3. **bidirectional** (the replacement's heartbeat reports the `cmd_seq` of
   the command that carried the slot);
4. outgoing **drained** (told to stop forwarding; its heartbeat confirms
   `forwarding=false`);
5. **downstream verified** (every dependant that was being heard before
   the drain sends fresh heartbeats over a route that avoids the outgoing
   relay; silent-before-drain dependants are exempted *and logged*);
6. **5 s hold**;
7. release.

Only GCS evidence is used.

Failure handling:

- **Rollback:** if a dependant goes silent while draining, forwarding is
  re-enabled.
- **Degraded modes:**
  - energy critical (margin < 7%): leave anyway, never forced toward
    depletion;
  - no affordable replacement: hold, logged;
  - replacement failed: another is dispatched;
  - timeout: logged.

Every release records its reason and which checks passed.

**Defect found during testing:** relays whose slot vanished were released
immediately while still carrying others (break-before-make). They are now
**retired** through the same drain → verify → hold path
(`gcs.py::_retire`).

**Tests:** `tests/test_handover.py`, covering normal, delayed replacement,
packet loss during handover, replacement failure and insufficient energy
for overlap. Each asserts that the outgoing relay lands ≥ 15% without
depletion, and that every non-degraded release had verified downstream
service.

**Seed 7:**

| Step | Value |
|---|---|
| Start | 422.5 s, margin 12.0% |
| On station | 453.3 s |
| Drained / downstream verified | 454.3 s |
| Released | 459.5 s |
| Battery at release / landing | 27.3% / 24.9% |
| Interruption attributable to the handover | **0 s** |

The 26.5 s of dependant outage in the window is attributed by the ledger
to a planned survey leg.

**Benchmark (E1 dev + held-out, spare), 28 handover events:**

| Outcome | Events |
|---|---|
| relieved on station (verified) | 12 |
| retired, slot no longer needed (verified) | 7 |
| **degraded: released on energy before verification** | **4** |
| recalled at mission end | 5 |

Handover-attributable dependant interruption:

- median **0 s** (dev and held-out);
- p90 0 s (dev) / 39.8 s (held-out);
- **worst 59.6 s (dev) / 247.8 s (held-out)**.

**The worst case (held-out seed_114) is unresolved.** The replacement never
reached the slot within 100 s. The relay was correctly released at 7%
margin rather than being run toward depletion, and its 3 dependants then
lost service: 248 s in the window, 116 s of it labelled "unknown" by the
ledger. In two verified retirements (seeds 120 and 128), dependants still
lost service for 40–75 s in the 30 s after release. So a verified release
does not guarantee continued service once the plan changes again.

### Item 4 · P1 · Critical relays — AR (metric), measured, redundancy evaluated

**Metric.** The old single-point-of-failure % counted partitioned time as
if nothing could break. The new metric uses three exclusive states with an
explicit denominator (time with ≥ 1 airborne mission UAV):
partitioned / connected with a critical relay / connected and redundant.
Seed 7: 24.8 / 53.2 / 22.0 %.

**Policies.** Compared on the same scenarios:

- `spare` (default);
- `reserve` (always a backup);
- `conditional` (a backup only for relays carrying ≥ 3).

**Infeasible redundancy** is logged (`relay.redundancy_infeasible`; seed 7
from t = 52 s: "no spare UAV for a backup relay (6 sites unserved)"),
tested in `test_safety_ledger.py::test_infeasible_redundancy_is_logged`.

**Result (E1):**

- `reserve` orphans 0 UAVs (vs 1.2 / 0.9 for spare, dev / held-out), but
  its median completion is 50–100 s later and its held-out success is
  27/30 vs 30/30.
- The ledger explains why: holding a UAV back roughly doubles planned
  survey-leg outage time per run (more data-mule legs).

**Decision:** keep `spare`. `conditional` is available.

### Item 5 · P1 · Fair planner comparison — AR, fixed

**Setup.**

- `tools/evaluate.py`: saved scenario files (`scenarios/dev`, `heldout`,
  `fresh`).
- **Common random numbers:** channel loss is a hash of
  (seed, link, 64 ms slot, attempt), independent of policy-dependent
  traffic.
- **Experiments labelled separately:**
  - E1: busiest relay of each planner;
  - E2: the *same* aircraft id;
  - E3: radio stress.
- Identical traffic, energy and safety rules for all planners.
- The evaluator uses the demo's code: `SwarmSim` is shared by the
  headless tools and Webots, and the seed-7 headless and Webots metrics
  match (validated).
- Quantiles, worst cases and failed runs are in `results/evaluation.md`.

**Held-out discipline.**

- The silent-surveyor *acknowledgement* fix was found on E2 dev.
- An optional "unexplained-silence re-queue" variant was designed after a
  held-out failure had been inspected. On a **fresh** set (seeds 200–229,
  created afterwards), variant and default are identical (59/60 each), so
  the variant was **not adopted** (`results/evaluation/fresh_variant_check.json`).

### Item 6 · P1 · Success with declared constraints — AR, fixed

`config.CONSTRAINTS`, predeclared on 2026-09-25 and **internal** (UAV-X
publishes no Stage 1 thresholds; NIDAR rules not imported):

| Constraint | Threshold |
|---|---|
| Imagery complete | ≤ 900 s |
| Fleet landed | ≤ 1200 s |
| High-priority response | ≤ 240 s |
| Sites within 60 s | ≥ 75% |
| Controller-caused separation below 5 m | none |
| Collisions | 0 |
| Geofence violations | 0 |
| Depletions | 0 |
| Landing battery | ≥ 15% |

Eventual completion, operational success and time-limited runs are
reported separately. Every run stays in the denominator.

**Fixed baseline:** eventually complete 22/30 (dev) but operational 4/30.

**Adaptive-spare:**

| Set | Operational success |
|---|---|
| E1 dev | 30/30 |
| E1 held-out | 30/30 |
| E1 fresh | 29/30 |
| E2 dev | 29/30 |
| E2 held-out | 29/30 |
| E2 fresh | 30/30 |
| E3 shadowed | 30/30 |
| E3 degraded | **13/30** |

### Item 7 · P0 · Information isolation — AL (no bypass existed), strengthened with negative tests

**Test file:** `tests/test_information_flow.py` (11 tests).

**Positive checks:**

- `gcs.py` and `agent.py` import neither the simulator, the channel nor
  the evaluator;
- GCS knowledge freezes during a partition while the truth changes;
- a command issued during a partition is not applied until delivered, and
  is applied only after reconnection;
- a disconnected survey leaves the GCS's site state unchanged until its
  imagery arrives;
- stale heartbeats are discarded (sequence numbers);
- expired or older commands are ignored;
- duplicate chunks are deduplicated and re-acked;
- fault injection does not inform the GCS (it learns only by timeout).

**Negative checks:**

- a `LeakyGCS` fixture that reads a silent UAV's true position and battery
  is **caught** by the hidden-truth invariance test;
- a heartbeat delivered around the channel is **caught** by the audit
  counters.

**Limits (stated in the file):** the tests cover imports, runtime inputs
and decision invariance. They cannot prove the absence of every future
leak.

**Result:** 0 bypass runs in all 960 benchmark runs (incl. the fresh set).

### Item 8 · P1 · Application delivery and shared-channel accounting — ML, fixed

**Model** (`uavx/comms.py::Channel`):

- per-hop airtime at 4 Mbit/s plus MAC overhead;
- carrier-sense blocking of radio neighbours;
- up to 3 MAC retries;
- priority queues (64 packets, tail drop);
- per-class expiry;
- imagery in 32 KiB chunks with a sliding window of 4, end-to-end acks and
  a 4 s retransmit timeout;
- thumbnails (8 kB / 2 s) now use the channel.

**Accounting.** Per-class generated / delivered / expired / dropped /
radio-down / lost / pending, plus `hop_tx`, `e2e_retransmissions` and
bytes. Cohort PDR by send time, with N/A for empty cohorts.

**Old "276/276 chunks, 417 transmissions".** Those were end-to-end
attempts on a model with no per-hop airtime. The new seed 7 reports 368
unique chunks (8 × 46), 368 delivered and 1,200 hop transmissions.

**Tests:** `tests/test_channel.py`:

- no instant multi-hop delivery;
- partition → expiry;
- N/A cohort;
- late receipt not counted as on time;
- retransmissions don't inflate unique delivery;
- **causality:**
  - lower capacity → slower imagery;
  - more load → higher control latency;
  - more loss → more transmissions per delivery;
  - missing acks → retransmissions and duplicates, not double counting;
- accounting balances exactly over a full mission.

### Item 9 · P2 · Radio and fault-model claims — ML, labelled and stress-tested

**Radio profiles:**

- `simple` (baseline, kept);
- `degraded` (480 m range, steeper edge loss);
- `shadowed` (terrain line-of-sight blocking plus clearance loss).

The stress profiles are **uncalibrated**. Sensitivity for spare:

| Profile | Operational success | Connectivity (median) |
|---|---|---|
| Shadowed | 30/30 | 93.8% |
| Degraded | **13/30** | 70.8% |

**Labels** in code, captions and cards:

- the motor fault is an "abstracted controlled descent";
- the 81 → 29 % drop is an "energy-availability fault".

**Battery service** is now a **swap** (`SERVICE` for 60 s, then 100%; no
continuous ramp), disclosed as time-compressed.

**Not done:** no dynamics or cell model, no hardware validation (out of
Stage 1 scope).

### Item 10 · P1 · Safety evaluation — CD (take-off near misses), fixed

**Model:** thresholds declared as geometric proxies (5 m near miss,
1.5 m collision proxy; no physics collision detection), a continuous
segment closest-approach check between 64 ms samples, and merged episodes
with duration, severity, phases and cause (controller vs fault
consequence).

**Found:** with the new timing, simultaneous launches produced
controller-caused near misses down to 2.1 m. A cruising UAV flew into the
column of a climbing one.

**Fix:**

- predictive vertical yield (60 m look-ahead);
- vertical back-off inside 8 m;
- horizontal yield to off-layer UAVs within 7.5 m;
- earlier: the emergency keep-out column.

**Tests:** crossing between samples detected, episode merging, collision
proxy with fault attribution, phase labels.

**Result:** 0 near misses and 0 collision proxies in all 960 benchmark runs (incl. the fresh set)
and the Webots run. Worst separation 6.0 m (the recorded v3 benchmark
reported 4.3 m).

### Item 11 · P2 · Perception — AR (semantics), partly fixed, UN (recall)

**Semantics** (`docs/PERCEPTION.md`): coverage, imagery delivery, count and
count correctness are separate. The count rule is defined (static ground
tracks). Live runs report count error, not precision or recall.

**Webots benchmark:** 72 views, 216 survivor instances, matched to truth.

- **0 false-positive boxes** in all 360 frames.
- The rule was chosen on the 76 m views and validated on the 60 m and
  92 m views: precision **0.992** / recall **0.903** (old rule 1.000 /
  0.889).

**New Webots run:**

| Measure | Recorded v3 | New run |
|---|---|---|
| Exact sites | 5/8 | **6/8** |
| Overcounts | 2 | **0** |
| Undercounts | 1 | 3 (P3 2/4, P4 2/3) |

The recorded overcounts are consistent with the settled-frame hypothesis
and are gone. The undercounts are **unresolved**: people standing less than
about 0.5 m apart are merged into one box by the stock detector
(`results/perception/P4_alt76_az0.jpg`). Scene truth is never used by the
detector.

### Item 12 · P2 · Operational metrics and presentation — AR, fixed

**Dashboard.**

- "ACTIVE DATA AGE" counts only UAVs the GCS still hears. Lost aircraft
  are listed separately ("MISSING: UAV5 (307 s)").
- Populations are labelled: roster / GCS-believed airborne / linked.
- Every event is tagged GCS or OBS (observer).

**Milestones.** Imagery complete 634.8 s; fleet landed (observer) 734.7 s;
**GCS-confirmed** 736.2 s, with UAV5 unaccounted.

**Captions.** GCS lines come from GCS knowledge; "RESULT (observer)" lines
come from the evaluator.

**Cards.** Generated from the metrics JSON and the raw-log validation.

**Cosmetic flaw:** the "RESULT (observer)" label slightly overlaps its
caption text.

### Also found and fixed (not in the brief)

| Defect | Found by | Result |
|---|---|---|
| **High-priority starvation:** a far high-priority site waited for leftover UAVs | benchmark dev seed_023 | Worst E1 dev high-priority response 375 → 146 s. |
| **Silent-surveyor hold:** the GCS held a site for a UAV that never received the task | E2 dev seed_007 | Now uses command acknowledgement: the high-priority response for that scenario dropped from 314 s to 24 s. |
| **Expected-silence estimate** replaces the fixed 240 s hold | same analysis | Capped at 240 s. |

## 3. Before/after on the same 60 scenarios (`results/before_after.md`)

BEFORE is the recorded code, unmodified. Like-for-like rows, adaptive-spare:

| Metric | Before | After |
|---|---|---|
| Median all imagery | 562.3 s | 542.8 s |
| Connectivity (median) | 91.3% | 93.5% |
| Disconnected UAV-s (median) | 235.0 | 155.2 |
| Orphaned by the fault (mean) | 1.4 | 1.1 |
| Worst separation | 6.0 m | 6.0 m |
| Near misses | 0 | 0 |

**Fixed baseline:** eventually complete 60 → 49. The new channel model is
more demanding for long data-mule legs. This is reported, not hidden.

Rows whose definitions changed (stable recovery, PDR) are marked as not
like-for-like.

## 4. Reproduction

**Versions:**

| Component | Version |
|---|---|
| Python | 3.10.11 |
| numpy | 2.2.6 |
| Pillow | 12.3.0 |
| ultralytics | 8.4.123 |
| torch | 2.13.0+cpu |
| ffmpeg | 9.0 |
| Webots | R2025a (Windows 11) |

**Configuration:** `controllers/swarm_supervisor/uavx/config.py` (seed 7,
`BACKUP_POLICY="spare"`, `RADIO_PROFILE="simple"`,
`HP_UNEXPLAINED_SILENCE_REQUEUE=False`).

```bash
python -m unittest discover -s tests -v            # 65 tests
python tools/run_headless.py                       # seed 7 -> results/*_scenario.*
python tools/run_headless.py --mule-fix off        # A/B for item 2
python tools/recompute_metrics.py results scenario # independent recomputation
python tools/evaluate.py --make-scenarios          # (re)writes scenarios/ deterministically
python tools/evaluate.py                           # E1, E2, E3 -> results/evaluation*
python tools/evaluate.py --variants                # optional variant
python tools/check_variant_fresh.py                # fresh-set check
python tools/compare_before_after.py               # needs ../_baseline_v3_recorded
python tools/perception_rules.py                   # rule selection from the bench
```

**Webots:**

```powershell
$env:UAVX_RECORD_MOVIE="$PWD\media\uav_x_4.mp4"
webots --batch --mode=fast worlds/uavx_stage3.wbt
$env:UAVX_PERCEPTION_BENCH="$PWD\results\perception"
webots --batch --mode=fast worlds/uavx_stage3.wbt
```

Recording note: on this machine a *minimised* Webots window now records a
black 3D view, so record with the window visible.

## 5. Remaining limitations, unresolved failures, unrun tests

**Unresolved failures:**

- High-priority response over 240 s in 1/30 E1-fresh, 1/30 E2-dev and
  1/30 E2-held-out (272–309 s). Cause: compounding faults (a hard fault on
  the busiest relay or on the high-priority surveyor, plus a radio outage)
  leave no free UAV near the far site.
- Degraded radio: 13/30 operational.

**Open behaviours:**

- degraded handovers (replacement too late): up to 248 s of dependant
  outage (held-out seed_114);
- break-before-make during ordinary re-plans;
- undercounting of adjacent people;
- "unknown" outage cause at about 1–2% of outage time.

**Model limits:**

- kinematic flight;
- abstracted faults;
- uncalibrated radio (no hidden-terminal or interference model;
  single-channel CSMA abstraction);
- time-compressed battery swap;
- perception from one scene with COCO weights.

**Not run:**

- ns-3 or calibrated propagation;
- PX4 or Gazebo dynamics;
- hardware;
- multiple simultaneous critical faults in the random benchmark (overlap is
  only unit-tested);
- per-person perception matching during live runs (only in the offline
  bench, by design);
- E3 for the optional variant;
- long-endurance (> 2,400 s) missions.

**Not claimed:** competition readiness. All thresholds are internal.
