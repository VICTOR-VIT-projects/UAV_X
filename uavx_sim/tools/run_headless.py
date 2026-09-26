"""Run the scripted scenario (or a seed sweep) without Webots.

    python tools/run_headless.py              # scripted scenario -> results/*_scenario.*
    python tools/run_headless.py --no-faults  # no disturbances    -> results/*_nofault.*
    python tools/run_headless.py --seeds 20   # channel-seed sweep of the scripted scenario
    python tools/run_headless.py --mule-fix off   # A/B: the recorded v3 data-mule behaviour

Every run writes the summary (metrics_<tag>.json) and the raw logs
(packets, routes, state, timeline, ledger, gcs_events, events) that
tools/recompute_metrics.py uses to re-derive the summary independently.
"""

import argparse
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "controllers", "swarm_supervisor"))

from uavx import config as C          # noqa: E402
from uavx.sim import SwarmSim          # noqa: E402


def run(seed, scenario, quiet):
    sim = SwarmSim(scenario=scenario, seed=seed, log=(lambda m: None) if quiet else print)
    while not sim.done:
        sim.step()
    return sim


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-faults", action="store_true")
    ap.add_argument("--seed", type=int, default=C.SEED)
    ap.add_argument("--seeds", type=int, default=0, help="run this many seeds and summarise")
    ap.add_argument("--mule-fix", choices=("on", "off"), default="on")
    args = ap.parse_args()
    C.MULE_HYSTERESIS = args.mule_fix == "on"
    out = os.path.join(ROOT, "results")
    os.makedirs(out, exist_ok=True)

    if args.seeds:
        rows = []
        for s in range(args.seeds):
            r = run(s, not args.no_faults, quiet=True).results()
            rows.append({"seed": s, "constraints": r["constraints"],
                         "all_delivered_s": r["mission"]["all_delivered_at_s"],
                         "connectivity_pct": r["communication"]["connectivity_pct"],
                         "hb_pdr_pct": r["communication"]["classes"]["hb"]["pdr_cohort_pct"],
                         "min_separation_m": r["safety"]["min_separation_m"]})
            print(f"seed {s:2d}: done {rows[-1]['all_delivered_s']}s, conn "
                  f"{rows[-1]['connectivity_pct']}%, hb PDR {rows[-1]['hb_pdr_pct']}%, "
                  f"min sep {rows[-1]['min_separation_m']} m, operational "
                  f"{r['constraints']['operational_success']}")
        ok = sum(1 for r in rows if r["constraints"]["operational_success"])
        print(f"\n{ok}/{len(rows)} seeds meet every predeclared constraint")
        with open(os.path.join(out, "seed_sweep.json"), "w") as f:
            json.dump(rows, f, indent=2)
        return

    sim = run(args.seed, not args.no_faults, quiet=False)
    res = sim.results()
    tag = ("nofault" if args.no_faults else "scenario") + ("" if args.mule_fix == "on" else "_mulefix_off")
    with open(os.path.join(out, f"metrics_{tag}.json"), "w") as f:
        json.dump(res, f, indent=2, default=list)
    sim.write_logs(out, tag)
    print(json.dumps({k: res[k] for k in ("end_reason", "constraints")}, indent=2))
    print(f"results + raw logs written to {out} (tag {tag})")


if __name__ == "__main__":
    main()
