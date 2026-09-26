"""Independently recompute key metrics from a run's RAW logs and compare
them with the summary the simulator wrote.

    python tools/recompute_metrics.py [results_dir] [tag]     # default: results webots

Inputs (written by SwarmSim.write_logs):
  routes_<tag>.csv      every change of (routed, hops, parent, status) per UAV
  packets_<tag>.csv     every packet: class, src, dst, t_gen, t_delivered, outcome
  gcs_events_<tag>.csv  when the GCS declared a UAV lost / received EMERGENCY
  metrics_<tag>.json    the summary under test
This script does NOT import the simulator; it re-derives each number from
the raw rows with the formulas in docs/METRICS.md. Writes
<results_dir>/validation_<tag>.json and exits non-zero on any mismatch.
"""

import csv
import json
import os
import sys

DT = 0.064
MISSION = ("ACTIVE", "RTH")
STABLE = 5.0
MERGE = 1.0
HB_DEADLINE = 2.0


def load_routes(path):
    rows = list(csv.DictReader(open(path)))
    by = {}
    for r in rows:
        by.setdefault(int(r["uav"]), []).append((float(r["t"]), r["routed"] == "1", r["status"]))
    return by


def state_at(series, t):
    """(routed, status) of one UAV at time t from its change log."""
    cur = (False, "READY")
    for tt, routed, status in series:
        if tt > t + 1e-9:
            break
        cur = (routed, status)
    return cur


def main():
    rdir = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results")
    tag = sys.argv[2] if len(sys.argv) > 2 else "webots"
    m = json.load(open(os.path.join(rdir, f"metrics_{tag}.json")))
    routes = load_routes(os.path.join(rdir, f"routes_{tag}.csv"))
    end = m["sim_time_s"]
    steps = [round(k * DT, 3) for k in range(1, int(end / DT) + 1)]
    # Per-step states, rebuilt from the change log (step grid = the sim's).
    idx = {u: 0 for u in routes}
    cur = {u: (False, "READY") for u in routes}
    grid = []
    for t in steps:
        for u, ser in routes.items():
            while idx[u] < len(ser) and ser[idx[u]][0] <= t + 1e-6:
                cur[u] = ser[idx[u]][1:]
                idx[u] += 1
        grid.append((t, dict(cur)))
    checks = []

    def check(name, recomputed, reported, tol):
        ok = (recomputed is None and reported is None) or (
            recomputed is not None and reported is not None and abs(recomputed - reported) <= tol)
        checks.append({"check": name, "recomputed": recomputed, "reported": reported,
                       "tolerance": tol, "pass": ok})

    # 1. disconnected UAV-seconds and connectivity
    disc = 0.0
    conn_sum, n = 0.0, 0
    for t, st in grid:
        air = [u for u, (r, s) in st.items() if s in MISSION]
        if air:
            cut = [u for u in air if not st[u][0]]
            disc += len(cut) * DT
            conn_sum += (len(air) - len(cut)) / len(air)
            n += 1
    check("disconnected_uav_s", round(disc, 1), m["communication"]["disconnected_uav_s"], 2.0)
    check("connectivity_pct", round(100 * conn_sum / max(1, n), 1),
          m["communication"]["connectivity_pct"], 0.3)

    # 2. per-UAV outage episodes (merge gaps < 1 s)
    eps = 0
    for u in routes:
        open_, last_cut = False, None
        for t, st in grid:
            r, s = st[u]
            if s in MISSION and not r:
                if not open_:
                    open_ = True
                    eps += 1
                last_cut = t
            elif open_ and (s not in MISSION or t - last_cut > MERGE):
                open_ = False
    check("per_uav_outage_episodes", eps,
          m["communication"]["outage_episodes"]["per_uav_episodes"], 1)

    # 3. fault events: population frozen at the fault, 5 s hold, reset on any gap
    lost = [r for r in csv.DictReader(open(os.path.join(rdir, f"gcs_events_{tag}.csv")))]
    for f in m["faults"]:
        if f["type"] not in ("uav_failure", "comm_outage"):
            continue
        req = set(f["population"]["required"])
        for ch in f["population"]["membership_changes"]:
            pass                                              # applied below by time
        left = {int(u): float(t) for t, u, why in f["population"]["membership_changes"]}
        t0 = f["t"]
        first = all_ = sstart = sconf = None
        hold = None
        for t, st in grid:
            if t <= t0 or not req:
                continue
            active = [u for u in req if not (u in left and t >= left[u])]
            if not active:
                break
            cut = [u for u in active if not st[u][0]]
            if first is None and len(cut) < len(active):
                first = t - t0
            if not cut:
                if first is None:
                    first = t - t0
                if all_ is None:
                    all_ = t - t0
                hold = hold if hold is not None else t
                if t - hold >= STABLE - 1e-9:
                    sstart, sconf = hold - t0, t - t0
                    break
            else:
                hold = None
        label = f"{f['type']}@{f['t']:.0f}s"
        if req:
            check(f"{label}.first_route", _r(first), f["first_route"], 0.1)
            check(f"{label}.all_routes", _r(all_), f["all_routes"], 0.1)
            check(f"{label}.stable_start", _r(sstart), f["stable_start"], 0.1)
            check(f"{label}.stable_confirmed", _r(sconf), f["stable_confirmed"], 0.1)
        det = next((float(r["t"]) - t0 for r in lost if int(r["uav"]) == f["uav"]
                    and float(r["t"]) >= t0), None)
        check(f"{label}.detected", _r(det), f["detected"], 0.1)

    # 4. heartbeat cohort PDR and unique imagery chunks from the packet log
    hb_n = hb_ok = 0
    keys_gen, keys_ok = set(), set()
    for r in csv.DictReader(open(os.path.join(rdir, f"packets_{tag}.csv"))):
        if r["class"] == "hb":
            tg = float(r["t_gen"])
            if tg <= end + 100.0 - HB_DEADLINE:
                hb_n += 1
                if r["t_delivered"] and float(r["t_delivered"]) - tg <= HB_DEADLINE + 1e-6:
                    hb_ok += 1
        elif r["class"] == "data":
            keys_gen.add(r["chunk_key"])
            if r["outcome"] == "delivered":
                keys_ok.add(r["chunk_key"])
    cls = m["communication"]["classes"]
    check("hb_pdr_cohort_pct", round(100 * hb_ok / max(1, hb_n), 1), cls["hb"]["pdr_cohort_pct"], 0.1)
    check("data_unique_generated", len(keys_gen), cls["data"].get("generated", 0), 0)
    check("data_unique_delivered", len(keys_ok), cls["data"].get("delivered", 0), 0)

    out = {"tag": tag, "all_pass": all(c["pass"] for c in checks), "checks": checks}
    with open(os.path.join(rdir, f"validation_{tag}.json"), "w") as f:
        json.dump(out, f, indent=1)
    for c in checks:
        print(("PASS " if c["pass"] else "FAIL ") + f"{c['check']}: recomputed {c['recomputed']} "
              f"vs reported {c['reported']}")
    print("ALL PASS" if out["all_pass"] else "MISMATCH")
    sys.exit(0 if out["all_pass"] else 1)


def _r(v):
    return None if v is None else round(v, 2)


if __name__ == "__main__":
    main()
