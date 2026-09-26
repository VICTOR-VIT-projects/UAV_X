# UAV-X Resilient BVLOS Swarm: Stage 1 submission

| Requirement | Where it is |
|---|---|
| **Working proof-of-concept simulation** on a standard open-source framework | `uavx_sim/`, a Webots R2025a world (Apache-2.0) plus a Python controller. Open `uavx_sim/worlds/uavx_stage3.wbt`. |
| **Source code** that reproduces the reported results | `uavx_sim/`: controllers, tools, tests (65 tests), saved scenarios, and raw logs. `python tools/run_headless.py` and `python tools/evaluate.py` regenerate every reported number; `tools/recompute_metrics.py` re-derives them from the raw logs. |
| **Installation instructions**: exact, ordered steps | `INSTALL.md` |
| **Demonstration video** | `video/UAV-X_demo.mp4` (3 min 50 s, 1080p). Not published online. Upload it and put the link here: `VIDEO LINK: ____________` |

## Supporting documents

| File | Contents |
|---|---|
| `reports/UAV-X_60_simulations_report.pdf` | Results of the 60 simulations and every formula in the model |
| `reports/UAV-X_60_simulations.xlsx` | All 60 runs, with live pass/fail formulas |
| `uavx_sim/docs/OPERATOR_REPORT.md` | Findings, evidence, and open limitations |
| `uavx_sim/docs/METRICS.md` | Metric definitions |
| `uavx_sim/docs/ARCHITECTURE.md` | System architecture |

## Headline result

The default planner meets every predeclared internal constraint in 60/60
saved scenarios, with 0 near misses and 0 collisions. This is a simulation
only: kinematic flight and an abstracted radio model.
