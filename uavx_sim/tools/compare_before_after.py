"""Before/after on the SAME saved scenarios (brief items 2 and 5).

    python tools/compare_before_after.py

BEFORE = the recorded v3 code, run unmodified from ../_baseline_v3_recorded
(in a separate process, because both packages are called `uavx`).
AFTER  = this tree. Both run the E1 (busiest-relay fault) scenarios of the
dev and held-out sets with the default planner (adaptive-spare) and with
the fixed-relay baseline.

Only quantities whose definition did NOT change are compared directly:
all-imagery time, connectivity (time-average of airborne UAVs with a route),
disconnected UAV-seconds, minimum separation, near-miss count (< 5 m),
depletions. Quantities whose definition changed are listed side by side
with the reason (see docs/METRICS.md): partition episodes (now merged with a
1 s gap), fault "stable recovery" (was the start of the stable interval and
excluded a radio-outage aircraft from its own recovery; now start AND
confirmation, faulted temporary aircraft included), heartbeat PDR (before:
one-shot packets without a channel model; now: cohort PDR over a shared
channel with queues and expiry).
Writes results/before_after.json and results/before_after.md.
"""

import json
import os
import statistics
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BASE = os.path.join(os.path.dirname(ROOT), "_baseline_v3_recorded")
sys.path.insert(0, os.path.join(ROOT, "tools"))

WORKER = r'''
import json, sys
sys.path.insert(0, sys.argv[1])
from uavx import config as C
from uavx.gcs import GroundStation, FixedRelayStation
from uavx.sim import SwarmSim
jobs = json.load(sys.stdin)
out = []
for planner, sc in jobs:
    C.BACKUP_POLICY = "spare"
    f = sc["fault"]
    script = [tuple(e) for e in sc["script"]] + [(f["t"], "fail", f["kind"] + "-busiest")]
    script = [(t, k, tuple(a) if isinstance(a, list) else a) for t, k, a in script]
    cls = GroundStation if planner == "adaptive-spare" else FixedRelayStation
    sim = SwarmSim(seed=sc["seed"], log=lambda m: None, pois=[tuple(p) for p in sc["pois"]],
                   script=script, batteries=sc["batteries"], gcs_cls=cls)
    while not sim.done:
        sim.step()
    r = sim.results()
    fail = next((x for x in r["faults"] if x["type"] == "uav_failure"), {})
    out.append({"planner": planner, "scenario": sc["id"], "end": r["end_reason"],
                "all_delivered_s": r["mission"]["all_delivered_at_s"],
                "complete": r["mission"]["completion_pct"] == 100.0,
                "connectivity_pct": r["communication"]["connectivity_pct"],
                "disconnected_uav_s": r["communication"]["disconnected_uav_s"],
                "partition_episodes": r["communication"]["partition_episodes"],
                "hb_pdr_pct": r["communication"]["pdr"]["hb"]["pdr_end_to_end_pct"],
                "fault_orphaned": fail.get("orphaned"),
                "fault_stable_old_def_s": fail.get("stable_recovery"),
                "min_separation_m": r["safety"]["min_separation_m"],
                "near_misses": r["safety"]["near_miss_episodes_lt_5m"],
                "depletions": r["safety"]["battery_depletions"]})
print(json.dumps(out))
'''


def before(jobs):
    p = subprocess.run([sys.executable, "-c", WORKER, os.path.join(BASE, "controllers",
                                                                    "swarm_supervisor")],
                       input=json.dumps(jobs), capture_output=True, text=True)
    if p.returncode:
        raise RuntimeError(p.stderr[-2000:])
    return json.loads(p.stdout)


def after(jobs):
    from evaluate import run_one
    rows = []
    for planner, sc in jobs:
        r = run_one(("E1-busiest", planner, sc, None))
        rows.append({"planner": planner, "scenario": sc["id"],
                     "all_delivered_s": r["all_delivered_s"], "complete": r["eventually_complete"],
                     "connectivity_pct": r["connectivity_pct"],
                     "disconnected_uav_s": r["disconnected_uav_s"],
                     "fault_orphaned": r["fault_orphaned"],
                     "fault_stable_start_s": r["fault_stable_start_s"],
                     "fault_stable_confirmed_s": r["fault_stable_confirmed_s"],
                     "hb_pdr_pct": r["hb_pdr_pct"], "min_separation_m": r["min_separation_m"],
                     "near_misses": r["near_misses"], "depletions": r["depletions"],
                     "operational_success": r["operational_success"],
                     "gcs_silences": r["gcs_silences"]})
    return rows


def med(rows, k, fn=statistics.median):
    v = [r[k] for r in rows if r.get(k) is not None]
    return round(fn(v), 1) if v else None


def main():
    from evaluate import load
    jobs = [(p, sc) for s in ("dev", "heldout") for sc in load(s)
            for p in ("adaptive-spare", "fixed-baseline")]
    b = before(jobs)
    a = after(jobs)
    table = {}
    for p in ("adaptive-spare", "fixed-baseline"):
        bb = [r for r in b if r["planner"] == p]
        aa = [r for r in a if r["planner"] == p]
        table[p] = {
            "runs": len(aa),
            "complete (runs)": [sum(r["complete"] for r in bb), sum(r["complete"] for r in aa)],
            "all imagery at GCS, median s": [med(bb, "all_delivered_s"), med(aa, "all_delivered_s")],
            "all imagery at GCS, worst s": [med(bb, "all_delivered_s", max), med(aa, "all_delivered_s", max)],
            "connectivity %, median": [med(bb, "connectivity_pct"), med(aa, "connectivity_pct")],
            "disconnected UAV-s, median": [med(bb, "disconnected_uav_s"), med(aa, "disconnected_uav_s")],
            "UAVs orphaned by fault, mean": [med(bb, "fault_orphaned", statistics.mean),
                                             med(aa, "fault_orphaned", statistics.mean)],
            "min separation m, worst": [med(bb, "min_separation_m", min), med(aa, "min_separation_m", min)],
            "near misses < 5 m, total": [sum(r["near_misses"] for r in bb), sum(r["near_misses"] for r in aa)],
            "depletions, total": [sum(r["depletions"] for r in bb), sum(r["depletions"] for r in aa)],
            "DEFINITION CHANGED: fault 'stable' s, median (before: start; after: start / confirmed)":
                [med(bb, "fault_stable_old_def_s"),
                 f"{med(aa, 'fault_stable_start_s')} / {med(aa, 'fault_stable_confirmed_s')}"],
            "DEFINITION CHANGED: heartbeat PDR %, median (before: no channel model; after: cohort)":
                [med(bb, "hb_pdr_pct"), med(aa, "hb_pdr_pct")],
            "after only: operational success (predeclared constraints)":
                ["n/a", sum(r["operational_success"] for r in aa)],
        }
    out = os.path.join(ROOT, "results")
    json.dump({"table": table, "before_runs": b, "after_runs": a},
              open(os.path.join(out, "before_after.json"), "w"), indent=1)
    L = ["# Before (recorded v3) vs after, same 60 saved scenarios (E1: busiest-relay fault)", "",
         "Command: `python tools/compare_before_after.py`. BEFORE runs the unmodified recorded "
         "code from `_baseline_v3_recorded`. Rows marked DEFINITION CHANGED are not like-for-like.", ""]
    for p, t in table.items():
        L += [f"## {p} ({t['runs']} runs)", "", "| metric | before | after |", "|---|---|---|"]
        for k, v in t.items():
            if k != "runs":
                L.append(f"| {k} | {v[0]} | {v[1]} |")
        L.append("")
    open(os.path.join(out, "before_after.md"), "w", encoding="utf-8").write("\n".join(L))
    print("\n".join(L))


if __name__ == "__main__":
    main()
