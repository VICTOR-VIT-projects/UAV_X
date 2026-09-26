"""Fair benchmark: saved scenarios, common random numbers, several planners.

    python tools/evaluate.py --make-scenarios          # (re)write scenarios/*.json
    python tools/evaluate.py                           # full benchmark (all experiments)
    python tools/evaluate.py --only E1-busiest --n 10  # a subset

Scenario files (scenarios/<set>/<id>.json) hold everything a run needs:
site layout, initial batteries, the disturbance script and the RNG seed.
Every planner runs the SAME files. Channel loss draws are keyed by (seed,
link, 64 ms slot, attempt), not drawn from a shared stream, so planners that
use the same link at the same time see the same loss realisation.

Experiments (labelled separately because they answer different questions):
  E1-busiest  critical fault on EACH PLANNER'S busiest relay at the fault time
              (worst case for that planner's own topology)
  E2-fixed    critical fault on the SAME aircraft id in every planner
  E3-radio    E1 on the "degraded" and "shadowed" radio profiles
              (uncalibrated sensitivity cases)
Sets: dev = seeds 0-29 (the set the v3 policies were first compared on);
heldout = seeds 100-129 (never used while choosing any policy parameter).

Planners: adaptive-spare (default), adaptive-reserve, adaptive-conditional,
fixed-baseline (gcs.FixedRelayStation). Same UAVs, onboard rules, network,
energy rules and safety logic for all.

Success is reported three ways, never collapsed:
  eventually_complete   all site imagery reached the GCS before MAX_TIME
  operational_success   ... AND every predeclared internal constraint held
                        (config.CONSTRAINTS; failures are listed per check)
  time_limited          the run hit MAX_TIME
Outputs: results/evaluation/<experiment>.json (every run) and
results/evaluation.md (tables with n, mean, p10/p50/p90, worst, failures).
"""

import argparse
import json
import math
import multiprocessing as mp
import os
import random
import statistics
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "controllers", "swarm_supervisor"))
sys.path.insert(0, os.path.join(ROOT, "tools"))

from uavx import config as C                             # noqa: E402
from uavx.gcs import FixedRelayStation, GroundStation    # noqa: E402
from uavx.sim import SwarmSim                            # noqa: E402

SCEN = os.path.join(ROOT, "scenarios")
OUT = os.path.join(ROOT, "results", "evaluation")
SETS = {"dev": range(0, 30), "heldout": range(100, 130),
        # Created after the silent-surveyor variant was designed (held-out seed 123 had
        # been inspected), so the variant is judged on scenarios nobody has seen.
        "fresh": range(200, 230)}
PLANNERS = {
    "adaptive-spare": (GroundStation, "spare"),
    "adaptive-reserve": (GroundStation, "reserve"),
    "adaptive-conditional": (GroundStation, "conditional"),
    "fixed-baseline": (FixedRelayStation, "spare"),
}
# Optional variants, run with --variants (not part of the default 4-planner table).
VARIANTS = {"adaptive-spare+hp-requeue": (GroundStation, "spare", {"HP_UNEXPLAINED_SILENCE_REQUEUE": True})}


# ------------------------------------------------------------- scenarios
def draw_scenario(seed):
    rng = random.Random(1000 + seed)
    gx, gy, _ = C.GCS_POS
    lo, hi = C.GEOFENCE[0] + 60, C.GEOFENCE[1] - 60

    def site():
        while True:
            x, y = rng.uniform(lo, hi), rng.uniform(lo, hi)
            if 500 <= math.hypot(x - gx, y - gy) <= 2400:
                return round(x, 1), round(y, 1)
    pois = [[f"P{i}", *site(), 1] for i in range(1, 8)]
    hx, hy = site()
    t_fail = round(rng.uniform(100, 260), 1)
    hard = rng.choice(["hard", "failsafe"])
    fixed_id = rng.randint(1, C.NUM_UAVS)
    return {
        "id": f"seed_{seed:03d}", "seed": seed, "pois": pois,
        "batteries": [round(rng.uniform(45, 100), 1) for _ in range(C.NUM_UAVS)],
        "fault": {"t": t_fail, "kind": hard, "fixed_uav": fixed_id},
        "script": [
            [round(rng.uniform(120, 320), 1), "new_poi", ["H1", hx, hy, 2]],
            [round(rng.uniform(150, 400), 1), "radio", round(rng.uniform(15, 60), 1)],
            [round(rng.uniform(250, 500), 1), "battery_sag", round(rng.uniform(35, 55), 1)],
        ],
    }


def make_scenarios():
    for name, seeds in SETS.items():
        d = os.path.join(SCEN, name)
        os.makedirs(d, exist_ok=True)
        for s in seeds:
            with open(os.path.join(d, f"seed_{s:03d}.json"), "w") as f:
                json.dump(draw_scenario(s), f, indent=1)
    with open(os.path.join(SCEN, "demo_seed7.json"), "w") as f:
        json.dump({"id": "demo_seed7", "seed": C.SEED, "pois": [list(p) for p in C.POIS],
                   "batteries": C.INITIAL_BATTERY,
                   "script": [list(e) for e in C.SCENARIO]}, f, indent=1)


def load(set_name):
    d = os.path.join(SCEN, set_name)
    return [json.load(open(os.path.join(d, fn))) for fn in sorted(os.listdir(d))]


# ------------------------------------------------------------------ runs
def _ground():
    try:
        from make_world import ground_h
        return ground_h
    except Exception:                                   # noqa: BLE001
        return None


def run_one(job):
    exp, planner, sc, radio = job
    if planner in VARIANTS:
        cls, policy, flags = VARIANTS[planner]
    else:
        (cls, policy), flags = PLANNERS[planner], {}
    C.BACKUP_POLICY = policy
    C.HP_UNEXPLAINED_SILENCE_REQUEUE = flags.get("HP_UNEXPLAINED_SILENCE_REQUEUE", False)
    f = sc["fault"]
    target = f["kind"] + (f"-uav{f['fixed_uav']}" if exp.startswith("E2") else "-busiest")
    script = [tuple(e) for e in sc["script"]] + [(f["t"], "fail", target)]
    script = [(t, k, tuple(a) if isinstance(a, list) else a) for t, k, a in script]
    sim = SwarmSim(seed=sc["seed"], log=lambda m: None, pois=[tuple(p) for p in sc["pois"]],
                   script=script, batteries=sc["batteries"], gcs_cls=cls, radio=radio,
                   ground=_ground() if radio == "shadowed" else None)
    while not sim.done:
        sim.step()
    r = sim.results()
    fail = next((x for x in r["faults"] if x["type"] == "uav_failure"), {})
    ho = r["relay"]["energy_handovers"]
    m, c, s = r["mission"], r["communication"], r["safety"]
    return {
        "experiment": exp, "planner": planner, "scenario": sc["id"], "radio": radio or "simple",
        "fault_target": target, "fault_uav": fail.get("uav"),
        "time_limited": r["end_reason"] == "time limit",
        "eventually_complete": r["constraints"]["eventually_complete"],
        "operational_success": r["constraints"]["operational_success"],
        "failed_constraints": [k for k, v in r["constraints"].items() if not v],
        "all_delivered_s": m["all_delivered_at_s"],
        "fleet_landed_s": m["fleet_landed_at_s_observer"],
        "timely_sites_frac": round(m["sites_within_internal_deadline"] / m["pois_total"], 3),
        "hp_response_s": (m["high_priority_response_s"] or [None])[0],
        "connectivity_pct": c["connectivity_pct"],
        "partitioned_pct": c["network_state_pct"].get("partitioned", 0.0),
        "critical_relay_pct": c["network_state_pct"].get("connected_single_point", 0.0),
        "redundant_pct": c["network_state_pct"].get("connected_redundant", 0.0),
        "disconnected_uav_s": c["disconnected_uav_s"],
        "outage_by_cause_uav_s": {k: v["uav_s"] for k, v in c["outage_episodes"]["by_cause"].items()},
        "gcs_silences": c["outage_episodes"]["gcs_silences"],
        "hb_pdr_pct": c["classes"]["hb"].get("pdr_cohort_pct"),
        "cmd_ack_p95_s": (c["command_ack_latency_s"] or {}).get("p95"),
        "fault_orphaned": len(fail.get("population", {}).get("orphaned", [])) if fail else None,
        "fault_outcome": fail.get("outcome"),
        "fault_stable_start_s": fail.get("stable_start"),
        "fault_stable_confirmed_s": fail.get("stable_confirmed"),
        "fault_required_disconnected_uav_s": fail.get("required_disconnected_uav_s"),
        "handovers": len(ho),
        "handover_outcomes": [h["outcome"] for h in ho],
        "handover_interruption_s": [h.get("handover_attributable_interruption_s") for h in ho],
        "min_separation_m": s["min_separation_m"],
        "near_misses": s["near_miss_episodes"],
        "controller_near_misses": sum(1 for e in s["episodes"] if e["cause"] == "controller"),
        "collisions": s["collision_proxy_episodes"],
        "violation_time_s": s["violation_time_s"],
        "fence_violations": s["geofence_violations"],
        "depletions": s["battery_depletions"],
        "min_landing_battery_pct": min((x["battery_pct"] for x in s["landings"]), default=None),
        "redundancy_infeasible_events": len(r["relay"]["redundancy_infeasible"]),
        "bypass": r["information_flow"]["bypass"],
    }


# --------------------------------------------------------------- summary
def q(vals, f):
    v = sorted(vals)
    return v[min(len(v) - 1, max(0, int(round(f * (len(v) - 1)))))]


def dist(rows, key, worst=max):
    v = [r[key] for r in rows if r[key] is not None]
    if not v:
        return {"n": 0}
    return {"n": len(v), "mean": round(statistics.mean(v), 1), "p10": q(v, 0.1),
            "p50": q(v, 0.5), "p90": q(v, 0.9), "worst": worst(v)}


def summarise(rows):
    fails = {}
    for r in rows:
        for k in r["failed_constraints"]:
            if k != "operational_success":
                fails[k] = fails.get(k, 0) + 1
    causes = {}
    for r in rows:
        for k, v in r["outage_by_cause_uav_s"].items():
            causes[k] = causes.get(k, 0.0) + v
    return {
        "runs": len(rows),
        "eventually_complete": sum(r["eventually_complete"] for r in rows),
        "operational_success": sum(r["operational_success"] for r in rows),
        "time_limited": sum(r["time_limited"] for r in rows),
        "constraint_failures (runs)": fails,
        "all_delivered_s": dist(rows, "all_delivered_s"),
        "fleet_landed_s": dist(rows, "fleet_landed_s"),
        "hp_response_s": dist(rows, "hp_response_s"),
        "timely_sites_frac": dist(rows, "timely_sites_frac", min),
        "connectivity_pct": dist(rows, "connectivity_pct", min),
        "partitioned_pct": dist(rows, "partitioned_pct"),
        "critical_relay_pct": dist(rows, "critical_relay_pct"),
        "redundant_pct": dist(rows, "redundant_pct", min),
        "disconnected_uav_s": dist(rows, "disconnected_uav_s"),
        "mean_outage_uav_s_by_cause (per run)": {k: round(v / len(rows), 1)
                                                 for k, v in sorted(causes.items(),
                                                                    key=lambda kv: -kv[1])},
        "hb_pdr_pct": dist(rows, "hb_pdr_pct", min),
        "fault_orphaned": dist(rows, "fault_orphaned"),
        "fault_stable_confirmed_s": dist(rows, "fault_stable_confirmed_s"),
        "fault_required_disconnected_uav_s": dist(rows, "fault_required_disconnected_uav_s"),
        "fault_not_recovered (runs)": sum(1 for r in rows if r["fault_outcome"] and
                                          not r["fault_outcome"].startswith(("recovered", "unaffected"))),
        "handovers (total events)": sum(r["handovers"] for r in rows),
        "handover_outcomes (total events)": _count(o for r in rows for o in r["handover_outcomes"]),
        "handover_interruption_s": dist([{"x": x} for r in rows
                                         for x in r["handover_interruption_s"]], "x"),
        "min_separation_m": dist(rows, "min_separation_m", min),
        "near_misses (total events)": sum(r["near_misses"] for r in rows),
        "controller_near_misses (total events)": sum(r["controller_near_misses"] for r in rows),
        "collisions (total events)": sum(r["collisions"] for r in rows),
        "depletions (total)": sum(r["depletions"] for r in rows),
        "fence_violations (total)": sum(r["fence_violations"] for r in rows),
        "min_landing_battery_pct": dist(rows, "min_landing_battery_pct", min),
        "bypass_runs": sum(r["bypass"] for r in rows),
        "failed_runs": [f"{r['scenario']}: {', '.join(k for k in r['failed_constraints'] if k != 'operational_success')}"
                        for r in rows if not r["operational_success"]],
    }


def _count(it):
    out = {}
    for x in it:
        out[x] = out.get(x, 0) + 1
    return out


def jobs_for(only, n, variants=False):
    if variants:
        out = []
        for set_name in ("dev", "heldout"):
            scs = load(set_name)[:n] if n else load(set_name)
            for exp in ("E1-busiest", "E2-fixed"):
                out += [(f"V-{exp}/{set_name}", v, sc, None) for v in VARIANTS for sc in scs]
        return out
    out = []
    for set_name in ("dev", "heldout"):
        scs = load(set_name)[:n] if n else load(set_name)
        for exp in ("E1-busiest", "E2-fixed"):
            if only and exp not in only:
                continue
            for p in PLANNERS:
                out += [(f"{exp}/{set_name}", p, sc, None) for sc in scs]
        if (not only or "E3-radio" in only) and set_name == "dev":
            for radio in ("degraded", "shadowed"):
                for p in PLANNERS:
                    out += [(f"E3-radio-{radio}/dev", p, sc, radio) for sc in scs]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--make-scenarios", action="store_true")
    ap.add_argument("--only", nargs="*")
    ap.add_argument("--n", type=int, default=0, help="first n scenarios of each set")
    ap.add_argument("--procs", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument("--variants", action="store_true", help="run the optional policy variants")
    args = ap.parse_args()
    if args.make_scenarios or not os.path.isdir(os.path.join(SCEN, "dev")):
        make_scenarios()
        print(f"scenarios written to {SCEN}")
        if args.make_scenarios:
            return
    jobs = jobs_for(args.only, args.n, args.variants)
    print(f"{len(jobs)} runs on {args.procs} processes", flush=True)
    with mp.Pool(args.procs) as pool:
        rows = []
        for i, r in enumerate(pool.imap_unordered(_safe_run, jobs, chunksize=2)):
            rows.append(r)
            if i % 50 == 0:
                print(f"  {i + 1}/{len(jobs)}", flush=True)
    os.makedirs(OUT, exist_ok=True)
    groups = {}
    for r in rows:
        groups.setdefault(r["experiment"], {}).setdefault(r["planner"], []).append(r)
    summary = {}
    for exp, by_p in sorted(groups.items()):
        summary[exp] = {p: summarise(sorted(v, key=lambda r: r["scenario"]))
                        for p, v in sorted(by_p.items(), key=lambda kv: (list(PLANNERS) + list(VARIANTS)).index(kv[0]))}
        with open(os.path.join(OUT, exp.replace("/", "__") + ".json"), "w") as f:
            json.dump({"summary": summary[exp],
                       "runs": sorted((r for v in by_p.values() for r in v),
                                      key=lambda r: (r["planner"], r["scenario"]))}, f, indent=1)
    # Rebuild the report from every saved experiment (not just this invocation).
    full = {}
    for fn in sorted(os.listdir(OUT)):
        if fn.endswith(".json"):
            full[fn[:-5].replace("__", "/")] = json.load(open(os.path.join(OUT, fn)))["summary"]
    write_md(full)
    print(open(os.path.join(ROOT, "results", "evaluation.md"), encoding="utf-8").read()[:3000])


def _safe_run(job):
    try:
        return run_one(job)
    except Exception as e:                              # noqa: BLE001
        import traceback
        return {"experiment": job[0], "planner": job[1], "scenario": job[2]["id"],
                "error": traceback.format_exc(), "failed_constraints": ["crash"],
                **{k: None for k in ("all_delivered_s",)}, "operational_success": False,
                "eventually_complete": False, "time_limited": False}


ROWS = [("eventually complete", "eventually_complete", "count"),
        ("operational success (all predeclared constraints)", "operational_success", "count"),
        ("hit time limit", "time_limited", "count"),
        ("all imagery at GCS (s)", "all_delivered_s", "dist"),
        ("surviving fleet landed (s)", "fleet_landed_s", "dist"),
        ("high-priority report -> imagery (s)", "hp_response_s", "dist"),
        ("sites within 60 s (fraction)", "timely_sites_frac", "dist"),
        ("connectivity %", "connectivity_pct", "dist"),
        ("time partitioned %", "partitioned_pct", "dist"),
        ("time with a critical relay %", "critical_relay_pct", "dist"),
        ("time redundant %", "redundant_pct", "dist"),
        ("heartbeat PDR (cohort) %", "hb_pdr_pct", "dist"),
        ("UAVs orphaned by the fault", "fault_orphaned", "dist"),
        ("fault: stable recovery confirmed (s)", "fault_stable_confirmed_s", "dist"),
        ("fault: required UAV-s disconnected", "fault_required_disconnected_uav_s", "dist"),
        ("fault not recovered (runs)", "fault_not_recovered (runs)", "raw"),
        ("handover interruption (s)", "handover_interruption_s", "dist"),
        ("min separation (m)", "min_separation_m", "dist"),
        ("near misses < 5 m (total events)", "near_misses (total events)", "raw"),
        ("... controller-caused (total events)", "controller_near_misses (total events)", "raw"),
        ("collision proxy < 1.5 m (total events)", "collisions (total events)", "raw"),
        ("depletions / fence violations (total)", None, "safety"),
        ("min landing battery %", "min_landing_battery_pct", "dist"),
        ("information-flow bypass (runs)", "bypass_runs", "raw")]


def _cell(s, key, kind):
    if kind == "count":
        return f"{s[key]}/{s['runs']}"
    if kind == "raw":
        return str(s[key])
    if kind == "safety":
        return f"{s['depletions (total)']} / {s['fence_violations (total)']}"
    d = s[key]
    if not d.get("n"):
        return "n/a"
    return f"{d['p50']} [{d['p10']}–{d['p90']}] worst {d['worst']} (n={d['n']})"


def write_md(summary):
    L = ["# Benchmark results", "",
         "Generated by `python tools/evaluate.py` from the saved scenario files in "
         "`scenarios/`. Cells: median [p10–p90] worst (n). Counts are runs out of runs; "
         "'total events' are summed over all runs of that planner. Constraints are the "
         "INTERNAL ones predeclared in `config.CONSTRAINTS` (not organiser rules).", ""]
    for exp, by_p in summary.items():
        L += [f"## {exp}", "", "| | " + " | ".join(by_p) + " |", "|---|" + "---|" * len(by_p)]
        for label, key, kind in ROWS:
            L.append(f"| {label} | " + " | ".join(_cell(s, key, kind) for s in by_p.values()) + " |")
        L.append("")
        for p, s in by_p.items():
            L.append(f"- **{p}** constraint failures (runs): {s['constraint_failures (runs)'] or 'none'}; "
                     f"handover outcomes: {s['handover_outcomes (total events)']}; "
                     f"outage UAV-s per run by cause: {s['mean_outage_uav_s_by_cause (per run)']}")
        L.append("")
    with open(os.path.join(ROOT, "results", "evaluation.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(L))


if __name__ == "__main__":
    main()
