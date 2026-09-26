# Before (recorded v3) vs after, same 60 saved scenarios (E1: busiest-relay fault)

Command: `python tools/compare_before_after.py`. BEFORE runs the unmodified recorded code from `_baseline_v3_recorded`. Rows marked DEFINITION CHANGED are not like-for-like.

## adaptive-spare (60 runs)

| metric | before | after |
|---|---|---|
| complete (runs) | 60 | 60 |
| all imagery at GCS, median s | 562.3 | 542.8 |
| all imagery at GCS, worst s | 866.4 | 871.5 |
| connectivity %, median | 91.3 | 93.5 |
| disconnected UAV-s, median | 235.0 | 155.2 |
| UAVs orphaned by fault, mean | 1.4 | 1.1 |
| min separation m, worst | 6.0 | 6.0 |
| near misses < 5 m, total | 0 | 0 |
| depletions, total | 0 | 0 |
| DEFINITION CHANGED: fault 'stable' s, median (before: start; after: start / confirmed) | 0.0 | 16.5 / 21.6 |
| DEFINITION CHANGED: heartbeat PDR %, median (before: no channel model; after: cohort) | 81.2 | 94.3 |
| after only: operational success (predeclared constraints) | n/a | 60 |

## fixed-baseline (60 runs)

| metric | before | after |
|---|---|---|
| complete (runs) | 60 | 49 |
| all imagery at GCS, median s | 757.2 | 813.8 |
| all imagery at GCS, worst s | 2250.0 | 2356.3 |
| connectivity %, median | 44.7 | 44.4 |
| disconnected UAV-s, median | 1800.3 | 1815.0 |
| UAVs orphaned by fault, mean | 1.2 | 0.9 |
| min separation m, worst | 2.6 | 6.0 |
| near misses < 5 m, total | 14 | 0 |
| depletions, total | 0 | 0 |
| DEFINITION CHANGED: fault 'stable' s, median (before: start; after: start / confirmed) | 0.0 | 47.0 / 52.1 |
| DEFINITION CHANGED: heartbeat PDR %, median (before: no channel model; after: cohort) | 40.2 | 57.5 |
| after only: operational success (predeclared constraints) | n/a | 8 |
