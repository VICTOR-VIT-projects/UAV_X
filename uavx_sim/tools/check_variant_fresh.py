"""Judge the optional silent-surveyor variant on scenarios nobody has seen.

    python tools/check_variant_fresh.py

The variant (config.HP_UNEXPLAINED_SILENCE_REQUEUE) was designed after
inspecting a held-out failure (heldout seed_123), so the held-out set is no
longer clean evidence for it. This runs the default planner and the variant
on the 'fresh' set (seeds 200-229, created afterwards), experiments E1 and E2.
Writes results/evaluation/fresh_variant_check.json.
"""

import json
import multiprocessing as mp
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))

from evaluate import _safe_run, load, summarise   # noqa: E402

PLANNERS = ("adaptive-spare", "adaptive-spare+hp-requeue")


def main():
    jobs = [(f"{e}/fresh", p, sc, None) for e in ("E1-busiest", "E2-fixed")
            for p in PLANNERS for sc in load("fresh")]
    with mp.Pool(max(1, (os.cpu_count() or 2) - 4)) as pool:
        rows = pool.map(_safe_run, jobs)
    out = {}
    for r in rows:
        out.setdefault(r["experiment"], {}).setdefault(r["planner"], []).append(r)
    res = {e: {p: summarise(v) for p, v in d.items()} for e, d in out.items()}
    with open(os.path.join(ROOT, "results", "evaluation", "fresh_variant_check.json"), "w") as f:
        json.dump({"summary": res, "runs": rows}, f, indent=1)
    for e, d in res.items():
        for p, s in d.items():
            print(e, p, "operational", f"{s['operational_success']}/{s['runs']}",
                  "failures", s["constraint_failures (runs)"],
                  "hp worst", s["hp_response_s"].get("worst"),
                  "imagery p50/worst", s["all_delivered_s"].get("p50"), s["all_delivered_s"].get("worst"),
                  "near misses", s["near_misses (total events)"])


if __name__ == "__main__":
    main()
