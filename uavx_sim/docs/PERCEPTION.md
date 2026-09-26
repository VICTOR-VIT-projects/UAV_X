# Perception: semantics, benchmark and count errors (brief item 11)

## What is measured, separately

| Quantity | Where | Meaning |
|---|---|---|
| Survey coverage | `mission.sites.*.surveyed_s` | The surveyor hovered at the site's standoff point for 10 s. |
| Imagery delivery | `mission.sites.*.imagery_delivered_s` | All 46 chunks reached the GCS. **This is what "site complete" means.** |
| Person count | `mission.sites.*.persons_detected` | The onboard count, carried to the GCS with the imagery. |
| Count correctness | `persons_ground_truth`, `perception.*` | Compared with the survivors placed in the scene. The scene truth is evaluator-only and is never given to the detector. |

Live runs have no per-person matching, so they report **count error**
(exact-count sites, undercount and overcount). An undercount and an
overcount at the same site can cancel out, so these are **not** precision
or recall. Precision and recall come only from the benchmark below, where
detections are matched to true positions.

## Counting rule (onboard, `controllers/swarm_supervisor/perception.py`)

1. **Detect:** YOLOv8n (COCO weights), person boxes with confidence ≥ 0.30
   and NMS IoU 0.70, on the UAV's own gimbal frame.
2. **Settled frames only:** a frame counts only when the gimbal has settled
   on the site. Yaw and pitch must be within 0.03 rad of the target and the
   zoom within 5%. Frames taken while the gimbal is still slewing or
   zooming are ignored.
3. **Ground projection:** each box's bottom-centre is projected through the
   known camera pose onto the site's ground height.
4. **Association:** only points within 8 m of the site centre count.
5. **Static tracks:** points within `TRACK_R` of a track join it.
6. **Site count:** the number of tracks seen in at least `TRACK_SUPPORT`
   of the settled frames ("unique static tracks").

The **previous rule** was the maximum number of person boxes in any single
frame during the hover. It had no association and included slewing frames.

## Benchmark (`UAVX_PERCEPTION_BENCH`, Webots)

**Views.** 8 sites × 3 hover altitudes (60, 76, 92 m) × 3 viewing azimuths
(−30°, 0°, +30°) × 5 jittered frames = 360 frames, 72 view sequences and
216 survivor instances.

**Scoring.** Detections are projected and matched to the survivors' true
positions (read from the world file) within 1.5 m.

**Outputs.**

- Raw per-frame points: `results/perception/perception_bench.json`.
- Annotated frames: `results/perception/*_alt76_az0.jpg`.

**Projection accuracy** (matched tracks): median error 0.06 m, worst 0.12 m.

**Rule selection** (`python tools/perception_rules.py`). Parameters were
chosen **on the 76 m views only** and reported on the **60 m and 92 m
views**, which were not used for the choice. Grid:

| Parameter | Values tried |
|---|---|
| NMS IoU | 0.70, 0.90 |
| `TRACK_R` | 0.3, 0.5, 1.0 m |
| `TRACK_SUPPORT` | 0.2, 0.4, 0.6 |

Validation results (48 view sequences):

| Rule | Precision | Recall | Exact-count views |
|---|---|---|---|
| Old: max boxes in one frame (associated) | 1.000 | 0.889 | 36 / 48 |
| Tracks with the first defaults (IoU 0.7, 1.0 m, 40%) | 1.000 | 0.882 | 35 / 48 |
| **Chosen: tracks, IoU 0.70, 0.3 m, 20%** | **0.992** | **0.903** | 36 / 48 |

The improvement is **small**. A higher NMS threshold (0.90) did not help.

## Explaining the recorded count discrepancies

The recorded run reported P3 3/4, P4 4/3, P6 4/3 and H1 3/3.

| Site | Recorded | Evidence | Explanation |
|---|---|---|---|
| P3 | 3/4 (under) | Benchmark: P3 exact in 5/9 views; 8 of 36 persons missed. | 4 survivors within 2.6 m. Two partly overlap and the detector returns one box. **Detector limitation, not fixed.** |
| P4 | 4/3 (over) | Benchmark, steady views: 0 false positives at P4 across 45 frames. Undercounts only: P4 exact in 2/9 views because two survivors stand 0.2 m apart (see `P4_alt76_az0.jpg`: one merged box, plus the flag mast occluding). | The overcount could not be reproduced in steady views. The most likely source is the old rule counting boxes in unsettled, wide-angle frames (while the gimbal zoomed onto the site) with no site association. **Hypothesis, not proven.** The new rule excludes both. |
| P6 | 4/3 (over) | Benchmark: P6 exact in 9/9 views, 0 false positives. | Same hypothesis as P4. |
| H1 | 3/3 | Exact in 9/9 views. | n/a |

**New Webots run with the new rule** (`results/metrics_webots.json`,
45–50 settled frames per site):

| Site | Count / truth |
|---|---|
| P1 | 3/3 |
| P2 | 2/2 |
| P3 | 2/4 |
| P4 | 2/3 |
| P5 | 2/2 |
| P6 | 3/3 |
| P7 | 4/4 |
| H1 | 3/3 |

That is **6/8 exact, 0 overcounts, 3 persons undercounted**. The recorded
run had 5/8 exact, 2 overcounts and 1 undercount. The overcounts are gone,
which is consistent with the settled-frame hypothesis. The undercounts at
P3 and P4 are the known merged-box limitation.

## Limits (not solved)

- About 10% of survivors are missed when people stand closer than about
  0.5 m, because the stock COCO detector merges them into one box. A model
  trained on aerial search-and-rescue imagery would be the Stage 2 fix.
- The benchmark uses the 8 sites of one scene, with its rubble and
  lighting. It is broader than the single demo pass but still one
  environment.
- Live runs report count error only. No per-person truth matching is done
  during the mission, which is correct, since the GCS must not see the
  truth.
