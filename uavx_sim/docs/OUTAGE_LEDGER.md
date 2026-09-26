# Outage ledger: every partition diagnosed (brief item 2)

The evaluator writes one ledger per run, in `results/ledger_<tag>.json`. It
contains:

- **per-UAV route episodes**, each with:
  - start and end;
  - positions;
  - nearest radio node and its distance;
  - whether the UAV was moving away from it;
  - last route parent, and whether that parent was moving or had just been
    re-tasked;
  - the UAV's heartbeat PDR over the 10 s before;
  - the GCS heartbeat age;
  - queue length;
  - concurrent faults and role transitions;
  - end reason;
  - an evidence-based **cause**;
- **mission partition episodes** (union over UAVs);
- **GCS silence episodes**, each with the fraction of the silence during
  which a modelled path existed;
- **safety episodes**.

Merge and split policy and metric formulas are in `docs/METRICS.md` §5.

## Cause rules (first match wins, `uavx/metrics.py::_diagnose`)

| # | Cause label | Evidence required |
|---|---|---|
| 1 | injected radio outage (own radio off) | A comm-outage fault on this UAV is active at the episode start. |
| 2 | own radio off | The UAV's radio is off for another reason. |
| 3 | fault-attributed: upstream relay lost | Starts less than 2 s after a critical fault, and the UAV is in that fault's orphaned population. |
| 4 | data mule carrying imagery back (planned out-of-range leg) | The UAV is in data-mule mode at the start. |
| 5 | return transit | Status RTH or task HOME. |
| 6 | survey beyond the network (planned by GCS) | Task SURVEY, and the UAV or its survey hover point is more than one radio range from every other node. |
| 7 | cascade: upstream relay cut off at the same moment | The last route parent lost its own route in the same 0.2 s. |
| 8 | range-boundary crossing: upstream relay moving (re-plan) / own motion out of range of its upstream | Distance to the last parent is within ±40 m of radio range; the label depends on which side moved. |
| 9 | range-boundary crossing: own motion outward | The nearest node is within ±40 m of radio range and the UAV is moving away from it. |
| 10 | relay repositioning after re-plan | The last parent was moving, or had been re-tasked in the last 20 s. |
| 11 | survey beyond the network (planned by GCS) | Task SURVEY (fallback). |
| 12 | **unknown** | None of the above. **Kept and reported, never dropped.** |

## Reconciling the recorded totals (seed 7)

**Recorded v3 code, re-run unmodified** from `_baseline_v3_recorded`:

| Measure | Value |
|---|---|
| Disconnected UAV-s | **239.4** |
| Partition episodes | **16** |
| Connectivity | 91.8% |
| All imagery at the GCS | 664.4 s |
| Fleet landed | 808.4 s |
| Radio outage "stable recovery" | "0.0 s" |

This reproduces every reference number in the brief.

**What changed, and why the new totals differ:**

| Run (seed 7, same script) | Per-UAV episodes | Mission episodes | Disconnected UAV-s | GCS silences | All imagery | Max survey → GCS |
|---|---|---|---|---|---|---|
| Recorded v3 code, old definitions | n/a | 16 (old: every cut/uncut transition, no merge) | 239.4 | 25 (LOST events in the log) | 664.4 s | 69.4 s |
| v3.1, **mule fix OFF** (`--mule-fix off`) | 9 | 4 | 222.6 | 18 | 674.6 s | 93.9 s |
| **v3.1** (`python tools/run_headless.py`) | 7 | 4 | 227.1 | 7 | 634.8 s | 52.2 s |

Reading the table:

- **Definitions (16 → 4).** The old count registered every step at which
  "some UAV is cut off" flipped. During UAV5's range-edge dithering (next
  section), that state flipped many times in a few seconds. The new mission
  count merges route gaps under 1 s per UAV before taking the union
  (`docs/METRICS.md` §5).
- **Channel model (239.4 → 222.6 with the fix off).** v3.1 routes every
  packet hop by hop over a shared channel with queues, expiry and per-hop
  retries (`uavx/comms.py`). That changes packet timing and therefore UAV
  behaviour slightly. Disconnected UAV-s is defined the same way in both
  runs.
- **The data-mule fix (18 → 7 GCS silences).** With definitions and channel
  held constant, the fix:
  - removes 11 false-alarm silences;
  - cuts the worst survey-to-GCS latency from 93.9 s to 52.2 s;
  - brings the internal 60 s threshold from 7/8 sites to 8/8;
  - finishes all imagery 40 s earlier.

  Disconnected UAV-s barely moves (222.6 → 227.1), because most of it is
  planned survey time beyond the network, which the fix does not (and
  should not) remove.

## Diagnosed causes and what was done

| Cause | Evidence | Status |
|---|---|---|
| **Range-boundary dither of a data mule** (recorded run t = 387–519 s: UAV5 LOST/re-acquired 12 times, then a 51 s gap) | The per-second trace shows the cycle: UAV5 is 600–650 m from its only neighbour; it hears one command at the edge, the silence timer resets, it flies back out and the link breaks. | **Confirmed, fixed.** Data-mule hysteresis: once carrying unacked imagery back, keep going until every chunk is acked (`agent.py::_choose_target`). Test: `test_core.py::test_data_mule_hysteresis`. A/B in the table above. |
| **High-priority starvation (priority inversion)** | Benchmark dev seed_023: H1 reported at 243 s, not tasked until 534 s. Too far for a full chain, so it waited for leftover UAVs after lower-priority chains. | **Confirmed, fixed.** A high-priority site with no full chain gets a data-mule leg at once (`gcs.py::_plan`). High-priority worst case, dev: 375 s → 164 s. |
| **Break-before-make relay retirement** | Handover test (delayed replacement): the relay carrying 4 dependants was released the moment its slot vanished from a new plan, before the new chain existed. | **Confirmed, fixed.** Retirement is now drained, verified and held like a handover (`gcs.py::_retire`). |
| **Upstream relay lost** (hard fault) | Fault-attributed, rule 3; 4 UAVs × 15 s at seed 7. | Expected consequence of the injected fault; measured by the fault tracker. The recovery policy is the onboard fragment contraction. |
| **Survey beyond the network** | Rule 6: the planner deliberately sends a surveyor out as a data mule when the fleet can't build a full chain. | Planned, reported separately from faults. It is the largest share of outage time (~74–93 UAV-s per run in the benchmark). |
| **Relay repositioning after re-plan / range-boundary (upstream moving) / cascade** | Rules 7, 8 and 10. The benchmark median is about 5–10 UAV-s per run for the adaptive planner (up to ~1,000 UAV-s per run for the fixed baseline). | **Confirmed, not fixed.** When the planner moves a relay, its downstream can briefly lose the link (break-before-make during re-plans). This is small for the adaptive planner. The proposed fix is to move a relay only after its dependants report a route that avoids it (the same verification as the handover). Left open. |
| **Heartbeat timeout with a path present** | Silence episodes with a routed fraction ≥ 90%. | **None** in the scripted run: all 7 GCS silences are graph disconnections. Detected by `test_safety_ledger.py::test_heartbeat_timeout_with_a_path_present_is_distinguished` under extreme loss. The timeout was **not** changed. |
| **unknown** | No rule matched. | Kept. Adaptive-spare: 1.8 UAV-s per run (dev) and 4.1 (held-out), about 1–2% of outage time. |

## Seed-7 ledger, v3.1 (`results/ledger_scenario.json`)

| UAV | Start → end (s) | Duration | Cause | Task at start |
|---|---|---|---|---|
| 1, 2, 3, 4 | 150.0 → 165.1 | 15.0 s each | fault-attributed: upstream relay lost (UAV5 hard fault) | survey P5 / relay P4 / relay P4 / survey P4 |
| 1 | 300.0 → 345.0 | 45.0 s | injected radio outage (own radio off) | survey H1 |
| 2 | 463.0 → 512.6 | 49.7 s | survey beyond the network (planned by GCS) | survey P6 |
| 2 | 546.1 → 617.9 | 71.8 s | survey beyond the network (planned by GCS) | survey P7 |

**GCS silences:** 7, all `graph disconnection` (routed fraction 0.00–0.02),
one for each episode above.

The Webots run's ledger is in `results/ledger_webots.json`.
