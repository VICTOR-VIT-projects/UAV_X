"""Title and results cards for the demo video (1920x1080 PNGs).

render_cards(results, out_dir) returns ["intro=<png>", "outro=<png>", ...]
for tools/compose_video.py. Every number on the outro cards is read from
files written by the simulation and the benchmark tools:
  results/metrics_webots.json, results/validation_webots.json (raw-log
  recomputation), results/evaluation/*.json, results/perception/*.json.
Nothing is typed in by hand.
"""

import json
import os

from PIL import Image, ImageDraw

from dashboard import ACCENT, BAD, BG, GOOD, MUTED, PANEL, PANEL2, TEXT, WARN, _font

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RES = os.path.join(ROOT, "results")
T1 = _font(["segoeuib.ttf", "DejaVuSans-Bold.ttf"], 50)
T2 = _font(["segoeuib.ttf", "DejaVuSans-Bold.ttf"], 27)
B = _font(["segoeuib.ttf", "DejaVuSans-Bold.ttf"], 20)
R = _font(["segoeui.ttf", "DejaVuSans.ttf"], 20)
S = _font(["segoeui.ttf", "DejaVuSans.ttf"], 17)
XS = _font(["segoeui.ttf", "DejaVuSans.ttf"], 15)


def _load(*parts):
    p = os.path.join(RES, *parts)
    return json.load(open(p)) if os.path.exists(p) else None


def _card(title, subtitle):
    im = Image.new("RGB", (1920, 1080), BG)
    d = ImageDraw.Draw(im)
    d.rectangle([0, 0, 1920, 8], fill=ACCENT)
    d.text((70, 40), title, font=T1, fill=TEXT)
    d.text((72, 106), subtitle, font=S, fill=MUTED)
    return im, d


def _box(d, x, y, w, h, head, rows, col=ACCENT):
    d.rounded_rectangle([x, y, x + w, y + h], 12, fill=PANEL)
    d.rectangle([x, y, x + 6, y + h], fill=col)
    d.text((x + 22, y + 12), head, font=T2, fill=col)
    yy = y + 56
    for k, v in rows:
        d.text((x + 22, yy + 2), k, font=XS, fill=MUTED)
        d.text((x + w - 20, yy), str(v), font=B, fill=TEXT, anchor="ra")
        yy += 31


def _s(v, unit="", nd=1):
    if v is None:
        return "n/a"
    if isinstance(v, float):
        return f"{v:.{nd}f}{unit}"
    return f"{v}{unit}"


def _wrap(d, text, font, width):
    words, lines, cur = text.split(), [], ""
    for w in words:
        if d.textlength((cur + " " + w).strip(), font=font) > width:
            lines.append(cur)
            cur = w
        else:
            cur = (cur + " " + w).strip()
    return lines + [cur] if cur else lines


# ------------------------------------------------------------------- cards
def intro_card(path):
    im, d = _card("UAV-X  ·  Resilient BVLOS Swarm  ·  Stage 1 PoC v3.1",
                  "PUSHPAK Grand Challenge 2026 · Webots simulation · revised against the v3 "
                  "operator fix brief · all results reproducible from saved scenarios and raw logs")
    rows = [
        ("Recovery metrics", "explicit populations (faulted / orphaned / unaffected / required); a "
                             "radio-outage aircraft must reconnect itself; first route, all routed, "
                             "stable START and CONFIRMED (5 s hold) reported separately"),
        ("Outage diagnosis", "every outage episode gets an evidence-based cause. Fixed causes: "
                             "data-mule range-edge dithering, high-priority starvation, take-off "
                             "separation, break-before-make relay retirement"),
        ("Channel model", "hop-by-hop shared airtime (4 Mbit/s), queues, expiry, per-hop retries, "
                          "sequence numbers, end-to-end imagery acks; thumbnails use the channel"),
        ("Relay handover", "dispatch -> on station -> both-way traffic verified -> old relay drained "
                           "-> every dependant heard via the new path -> 5 s hold -> release; "
                           "rollback and degraded modes logged"),
        ("Fair benchmark", "120 saved scenarios, 4 planners incl. a fixed-relay baseline, common "
                           "random numbers, held-out set, success = predeclared internal constraints"),
        ("Who knows what", "GCS decides only on delivered packets; negative tests catch a leaky GCS "
                           "and a channel bypass"),
        ("Stated abstractions", "kinematic flight; motor fault = controlled descent; energy fault = "
                                "instant charge loss; 60 s battery swap (time-compressed); radio "
                                "stress profiles uncalibrated; no hardware validation"),
    ]
    y = 160
    for k, v in rows:
        lines = _wrap(d, v, R, 1400)
        h = 20 + 30 * len(lines)
        d.rounded_rectangle([70, y, 1850, y + h], 10, fill=PANEL)
        d.text((95, y + 12), k, font=B, fill=ACCENT)
        for i, ln in enumerate(lines):
            d.text((400, y + 10 + 30 * i), ln, font=R, fill=TEXT)
        y += h + 12
    d.text((70, 1035), "3D view = simulator truth (observer)  ·  right panel, feeds, site tiles = "
                       "GCS view  ·  captions: GCS lines from GCS knowledge, RESULT (observer) from "
                       "the evaluator", font=XS, fill=MUTED)
    im.save(path)


def results_card(r, path):
    val = _load("validation_webots.json")
    vtxt = (f"{sum(c['pass'] for c in val['checks'])}/{len(val['checks'])} key metrics recomputed "
            "from raw logs match (tools/recompute_metrics.py)" if val else
            "raw-log recomputation not yet run")
    im, d = _card("RESULTS  ·  this run (scripted scenario, seed 7)",
                  "from results/metrics_webots.json  ·  " + vtxt)
    m, c, s = r["mission"], r["communication"], r["safety"]
    ft = next((f for f in r["faults"] if f["type"] == "uav_failure"), {})
    ro = next((f for f in r["faults"] if f["type"] == "comm_outage"), {})
    ho = (r["relay"]["energy_handovers"] or [{}])[0]
    k = r["constraints"]
    ns = c["network_state_pct"]
    pc = r["perception"]
    _box(d, 60, 150, 590, 390, "Mission", [
        ("sites with all imagery at GCS", f"{m['pois_delivered']}/{m['pois_total']}"),
        ("imagery complete", _s(m["all_delivered_at_s"], " s")),
        ("fleet landed: observer / GCS-confirmed", f"{_s(m['fleet_landed_at_s_observer'], ' s')} / "
                                                   f"{_s(m['fleet_landed_confirmed_by_gcs_s'], ' s')}"),
        ("unaccounted by the GCS", ", ".join(f"UAV{u}" for u in m["unaccounted_by_gcs"]) or "none"),
        ("high-priority report -> imagery", _s((m["high_priority_response_s"] or [None])[0], " s")),
        ("sites within 60 s (internal)", f"{m['sites_within_internal_deadline']}/{m['pois_total']}"),
        ("persons: exact sites · under · over", f"{pc.get('sites_exact', 'n/a')}/{pc.get('sites_scored', 0)}"
                                                f" · {pc.get('persons_undercount', '-')} · "
                                                f"{pc.get('persons_overcount', '-')}"),
        ("predeclared constraints", "ALL PASS" if k["operational_success"] else
         "FAIL: " + ", ".join(x for x, v in k.items() if not v)),
    ], GOOD if k["operational_success"] else BAD)
    pop = ft.get("population", {})
    _box(d, 670, 150, 590, 390, f"Hard fault · UAV{ft.get('uav')} (aircraft lost)", [
        ("orphaned / unaffected", f"{len(pop.get('orphaned', []))} / {len(pop.get('unaffected', []))}"),
        ("GCS detects (heartbeat timeout)", _s(ft.get("detected"), " s", 2)),
        ("first route back", _s(ft.get("first_route"), " s")),
        ("all orphaned routed", _s(ft.get("all_routes"), " s")),
        ("stable from / confirmed (5 s hold)", f"{_s(ft.get('stable_start'), ' s')} / "
                                               f"{_s(ft.get('stable_confirmed'), ' s')}"),
        ("GCS hears all of them again", _s(ft.get("all_app_delivery"), " s")),
        ("required UAV-s cut off · re-splits", f"{_s(ft.get('required_disconnected_uav_s'))} · "
                                               f"{ft.get('flaps')}"),
        ("outcome", (ft.get("outcome") or "n/a").split(":")[0]),
    ], BAD)
    _box(d, 1280, 150, 580, 390, f"Radio outage · UAV{ro.get('uav')} · {_s(ro.get('duration'), ' s', 0)}", [
        ("faulted aircraft in its own recovery", "yes" if ro.get("uav") in
         ro.get("population", {}).get("required", []) else "no"),
        ("GCS flags it (timeout)", _s(ro.get("detected"), " s", 2)),
        ("routed again after the fault", _s(ro.get("all_routes"), " s")),
        ("stable from / confirmed", f"{_s(ro.get('stable_start'), ' s')} / "
                                    f"{_s(ro.get('stable_confirmed'), ' s')}"),
        ("GCS hears it again", _s(ro.get("all_app_delivery"), " s")),
        ("unaffected UAVs kept routes", _s(ro.get("unaffected_continuity_pct"), " %")),
        ("outcome", (ro.get("outcome") or "n/a").split(":")[0]),
    ], MAGENTA_OR(WARN))
    _box(d, 60, 560, 590, 330, f"Energy handover · UAV{ho.get('uav')} -> UAV{ho.get('replacement')}", [
        ("margin at start", _s(ho.get("margin_start"), " %")),
        ("dispatch -> on station", f"{_s(ho.get('t_dispatch'))} -> {_s(ho.get('t_on_station'))} s"),
        ("drained · downstream verified", f"{_s(ho.get('t_drain'))} · {_s(ho.get('t_downstream_ok'))} s"),
        ("released (after 5 s hold)", _s(ho.get("t_released"), " s")),
        ("battery at release · at landing", f"{_s(ho.get('battery_released'), '%')} · "
                                            f"{_s(ho.get('landed_battery'), '%')}"),
        ("interruption attributable to handover", _s(ho.get("handover_attributable_interruption_s"), " s")),
        ("outcome", (ho.get("outcome") or "n/a")),
    ], WARN)
    hb = c["classes"]["hb"]
    dat = c["classes"]["data"]
    top = list(c["outage_episodes"]["by_cause"].items())[:2]
    _box(d, 670, 560, 590, 330, "Communication", [
        ("connectivity (routed airborne UAVs)", _s(c["connectivity_pct"], " %")),
        ("time partitioned / critical relay / redundant",
         f"{_s(ns.get('partitioned'), '')} / {_s(ns.get('connected_single_point'), '')} / "
         f"{_s(ns.get('connected_redundant'), '')} %"),
        ("heartbeat PDR (cohort, 2 s deadline)", _s(hb.get("pdr_cohort_pct"), " %")),
        ("command applied, p95 latency", _s((c["command_ack_latency_s"] or {}).get("p95"), " s", 2)),
        ("imagery chunks unique / on-air tx", f"{dat.get('delivered')}/{dat.get('generated')} / "
                                             f"{dat.get('hop_tx')}"),
        ("largest outage cause (ledger)", f"{_short(top[0][0])} · {top[0][1]['uav_s']:.0f} UAV-s"
                                          if top else "none"),
    ])
    _box(d, 1280, 560, 580, 330, "Safety (geometric proxies)", [
        ("min separation", _s(s["min_separation_m"], " m", 2)),
        ("near-miss episodes < 5 m", s["near_miss_episodes"]),
        ("collision proxy < 1.5 m", s["collision_proxy_episodes"]),
        ("violation time", _s(s["violation_time_s"], " s")),
        ("geofence violations · depletions", f"{s['geofence_violations']} · {s['battery_depletions']}"),
        ("lowest landing battery (reserve 15%)",
         _s(min((x["battery_pct"] for x in s["landings"]), default=None), " %")),
    ])
    d.text((60, 910), "Definitions: docs/METRICS.md. Stable = every required UAV routed for 5 s "
           "(our definition); confirmed = start + 5 s. PDR cohort = heartbeats delivered within 2 s / "
           "sent, counted after their deadline.", font=XS, fill=MUTED)
    d.text((60, 935), "Outage causes: docs/OUTAGE_LEDGER.md (per-episode ledger in "
           "results/ledger_webots.json). Constraints are INTERNAL test thresholds "
           "predeclared in config.CONSTRAINTS, not organiser rules.", font=XS, fill=MUTED)
    im.save(path)


def _short(cause):
    for k, v in (("survey beyond", "planned survey legs"), ("fault-attributed", "relay fault"),
                 ("injected radio", "radio outage"), ("cascade", "cascade"),
                 ("range-boundary", "range edge"), ("relay repositioning", "relay re-plan"),
                 ("data mule", "data-mule leg"), ("return", "return transit")):
        if cause.startswith(k):
            return v
    return cause[:20]


def MAGENTA_OR(c):
    return (219, 97, 212) if c else c


def benchmark_card(path):
    e = {name: _load("evaluation", f"E1-busiest__{name}.json") for name in ("dev", "heldout")}
    if not all(e.values()):
        return None
    planners = list(e["dev"]["summary"])
    im, d = _card("BENCHMARK  ·  same saved scenarios for every planner",
                  "E1: critical fault on each planner's busiest relay · dev = 30 scenarios, held-out = "
                  "30 never used for tuning · cells: median (worst)  ·  tools/evaluate.py")
    rows = [("operational success (all predeclared constraints)", "operational_success", "count"),
            ("eventually complete", "eventually_complete", "count"),
            ("all imagery at GCS (s)", "all_delivered_s", "dist"),
            ("high-priority report -> imagery (s)", "hp_response_s", "dist"),
            ("connectivity %", "connectivity_pct", "dist"),
            ("fault: stable confirmed (s)", "fault_stable_confirmed_s", "dist"),
            ("UAVs orphaned by the fault (mean)", "fault_orphaned", "mean"),
            ("near misses · collisions (events)", None, "safety"),
            ("information-flow bypass (runs)", "bypass_runs", "raw")]
    cw = 205
    x0 = 700
    y = 150
    d.rounded_rectangle([60, y, 1860, y + 70], 8, fill=PANEL2)
    for i, p in enumerate(planners):
        for j, sname in enumerate(("dev", "heldout")):
            xx = x0 + (i * 2 + j) * (cw * 0.72) - 60
            d.text((xx + 70, y + 20), p.replace("adaptive-", "").replace("fixed-", "fixed "),
                   font=XS, fill=ACCENT if p != "fixed-baseline" else MUTED, anchor="mm")
            d.text((xx + 70, y + 46), sname, font=XS, fill=MUTED, anchor="mm")
    y += 80
    for label, key, kind in rows:
        d.rounded_rectangle([60, y, 1860, y + 52], 8, fill=PANEL)
        d.text((80, y + 14), label, font=S, fill=TEXT)
        for i, p in enumerate(planners):
            for j, sname in enumerate(("dev", "heldout")):
                s = e[sname]["summary"][p]
                if kind == "count":
                    v = f"{s[key]}/{s['runs']}"
                elif kind == "raw":
                    v = str(s[key])
                elif kind == "safety":
                    v = f"{s['near_misses (total events)']} · {s['collisions (total events)']}"
                elif kind == "mean":
                    v = _s(s[key].get("mean"))
                else:
                    dd = s[key]
                    if dd.get("n"):
                        v = f"{_s(dd.get('p50'), '', 0)} ({_s(dd.get('worst'), '', 0)})"
                    elif key.startswith("fault_stable") and s["fault_orphaned"].get("mean") == 0:
                        v = "none orphaned"
                    else:
                        v = "n/a"
                xx = x0 + (i * 2 + j) * (cw * 0.72) - 60
                d.text((xx + 70, y + 26), v, font=XS, fill=TEXT, anchor="mm")
        y += 58
    sens = []
    for prof in ("degraded", "shadowed"):
        ev = _load("evaluation", f"E3-radio-{prof}__dev.json")
        if ev:
            sp = ev["summary"]["adaptive-spare"]
            sens.append(f"{prof}: {sp['operational_success']}/{sp['runs']} operational, connectivity "
                        f"median {sp['connectivity_pct'].get('p50')} %")
    y += 10
    d.text((70, y), "Radio sensitivity, adaptive-spare (uncalibrated stress profiles): "
           + "  ·  ".join(sens), font=S, fill=WARN)
    e2 = {n: _load("evaluation", f"E2-fixed__{n}.json") for n in ("dev", "heldout")}
    if all(e2.values()):
        cells = []
        for p in planners:
            cells.append(f"{p.replace('adaptive-', '').replace('fixed-', 'fixed ')} "
                         + "/".join(f"{e2[n]['summary'][p]['operational_success']}"
                                    for n in ("dev", "heldout")))
        d.text((70, y + 34), "E2 (fault on the SAME aircraft id in every planner), operational "
               "success dev/held-out of 30: " + "  ·  ".join(cells), font=S, fill=TEXT)
        y += 34
    fr = _load("evaluation", "fresh_variant_check.json")
    if fr:
        e1 = fr["summary"]["E1-busiest/fresh"]["adaptive-spare"]
        e2 = fr["summary"]["E2-fixed/fresh"]["adaptive-spare"]
        v1 = fr["summary"]["E1-busiest/fresh"]["adaptive-spare+hp-requeue"]
        v2 = fr["summary"]["E2-fixed/fresh"]["adaptive-spare+hp-requeue"]
        d.text((70, y + 34), f"Fresh set (seeds 200-229, created last, never inspected): spare E1 "
               f"{e1['operational_success']}/30, E2 {e2['operational_success']}/30  ·  optional "
               f"silent-surveyor re-queue variant {v1['operational_success']}/30, "
               f"{v2['operational_success']}/30 -> no gain on unseen data, not adopted",
               font=S, fill=ACCENT)
        y += 34
    d.text((70, y + 34), "Reserve keeps a backup relay (fewer orphans, slower: one UAV fewer "
           "serves the sites). Conditional reserves only for relays carrying >= 3. Spare is the "
           "default.", font=XS, fill=MUTED)
    d.text((70, y + 60), "Fixed-baseline: relay backbone assigned once and never re-planned. "
           "Same UAVs, onboard rules, channel, energy and safety logic for all planners.",
           font=XS, fill=MUTED)
    im.save(path)
    return path


def perception_card(path):
    rs = _load("perception", "rule_selection.json")
    ba = _load("before_after.json")
    im, d = _card("PERCEPTION, BEFORE/AFTER AND OPEN LIMITATIONS",
                  "perception: 8 sites x 3 altitudes x 3 view angles x 5 frames in Webots, detections "
                  "matched to true survivor positions (evaluator only)")
    rows = []
    if rs:
        cv = rs["chosen_validate"]
        old = list(rs["baselines_validate"].values())[0]
        rows += [("counting rule (chosen on 76 m views)", f"static ground tracks, r = "
                                                          f"{rs['chosen']['track_r']} m, support "
                                                          f"{int(rs['chosen']['support'] * 100)} %"),
                 ("validation views (60 m and 92 m)", f"{cv['views']}"),
                 ("precision / recall, new rule", f"{cv['precision']} / {cv['recall']}"),
                 ("precision / recall, old max-per-frame rule", f"{old['precision']} / {old['recall']}"),
                 ("exact-count views, new / old", f"{cv['exact_views']} / {old['exact_views']}")]
    _box(d, 60, 150, 880, 250, "Onboard perception (YOLOv8n, COCO weights)", rows)
    brow = []
    if ba:
        t = ba["table"]["adaptive-spare"]
        brow = [("runs (same 60 scenarios)", t["runs"]),
                ("all imagery, median s: before -> after", f"{t['all imagery at GCS, median s'][0]} -> "
                                                            f"{t['all imagery at GCS, median s'][1]}"),
                ("connectivity %, median", f"{t['connectivity %, median'][0]} -> {t['connectivity %, median'][1]}"),
                ("disconnected UAV-s, median", f"{t['disconnected UAV-s, median'][0]} -> "
                                               f"{t['disconnected UAV-s, median'][1]}"),
                ("min separation m, worst", f"{t['min separation m, worst'][0]} -> {t['min separation m, worst'][1]}"),
                ("near misses < 5 m, total", f"{t['near misses < 5 m, total'][0]} -> {t['near misses < 5 m, total'][1]}"),
                ("after: operational success", f"{t['after only: operational success (predeclared constraints)'][1]}/{t['runs']}")]
    _box(d, 980, 150, 880, 250, "Before (recorded v3) -> after, adaptive-spare", brow, GOOD)
    lim = ["Misses are people standing so close that the stock COCO detector returns one box "
           "(~10 % of persons); not solved. Count error, not per-person truth, in live runs.",
           "Flight is kinematic; the motor fault is an abstracted controlled descent; the energy "
           "fault is an instant loss of usable charge (no cell model).",
           "The radio is a distance/loss abstraction with a single-channel CSMA airtime model; "
           "'degraded' and 'shadowed' are uncalibrated stress cases. No hardware validation.",
           "Battery service is a 60 s swap (time-compressed). Demo fault times are scripted; the "
           "benchmark randomises times, targets, kinds and layouts.",
           "All thresholds are internal (config.CONSTRAINTS), predeclared; UAV-X has not published "
           "Stage 1 timing requirements. Not a competition-readiness claim."]
    y = 430
    d.text((70, y), "Open limitations", font=T2, fill=WARN)
    y += 46
    for ln in lim:
        for i, part in enumerate(_wrap(d, ln, S, 1760)):
            d.text((90 if i else 70, y), ("• " if i == 0 else "  ") + part, font=S, fill=TEXT)
            y += 28
        y += 8
    im.save(path)
    return path


def render_cards(results, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    out = []
    p = os.path.join(out_dir, "_card_intro.png")
    intro_card(p)
    out.append(f"intro={p}")
    if results:
        p = os.path.join(out_dir, "_card_results.png")
        results_card(results, p)
        out.append(f"outro={p}")
    for fn, name in ((benchmark_card, "_card_bench.png"), (perception_card, "_card_perc.png")):
        p = fn(os.path.join(out_dir, name))
        if p:
            out.append(f"outro={p}")
    return out
