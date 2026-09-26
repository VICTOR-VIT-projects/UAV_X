"""Build results/UAV-X_60_simulations.xlsx from the saved benchmark results.

    python tools/make_report_xlsx.py

The 60 simulations are the E1 experiment (critical fault on the busiest relay)
of the default planner (adaptive-spare) on the 30 development + 30 held-out
saved scenarios (scenarios/dev, scenarios/heldout).
"""

import json
import os

from openpyxl import Workbook
from openpyxl.comments import Comment
from openpyxl.formatting.rule import CellIsRule
from openpyxl.styles import Alignment, Border, Font, PatternFill, Side
from openpyxl.utils import get_column_letter

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EV = os.path.join(ROOT, "results", "evaluation")
OUT = os.path.join(ROOT, "results", "UAV-X_60_simulations.xlsx")

F = "Arial"
HEAD = PatternFill("solid", fgColor="1F3864")
SUB = PatternFill("solid", fgColor="D9E1F2")
INPUT = PatternFill("solid", fgColor="FFFF00")
thin = Side(style="thin", color="B7B7B7")
BOX = Border(left=thin, right=thin, top=thin, bottom=thin)


def runs(set_name, planner="adaptive-spare"):
    d = json.load(open(os.path.join(EV, f"E1-busiest__{set_name}.json")))
    return [r for r in d["runs"] if r["planner"] == planner]


def head(ws, row, labels, widths=None):
    for j, lab in enumerate(labels, 1):
        c = ws.cell(row=row, column=j, value=lab)
        c.font = Font(name=F, bold=True, color="FFFFFF", size=9)
        c.fill = HEAD
        c.alignment = Alignment(wrap_text=True, horizontal="center", vertical="center")
        c.border = BOX
        if widths:
            ws.column_dimensions[get_column_letter(j)].width = widths[j - 1]
    ws.row_dimensions[row].height = 42


def main():
    wb = Workbook()

    # ------------------------------------------------------------ Constraints
    cs = wb.active
    cs.title = "Constraints"
    cs["A1"] = "Predeclared INTERNAL constraints (config.CONSTRAINTS, fixed 2026-09-25)"
    cs["A1"].font = Font(name=F, bold=True, size=12)
    head(cs, 3, ["Constraint", "Threshold", "Unit", "Meaning"], [38, 12, 10, 70])
    rows = [("Imagery complete by", 900, "s", "All site imagery must reach the GCS by this time"),
            ("Fleet landed by", 1200, "s", "Every surviving UAV on its pad (observer)"),
            ("High-priority response", 240, "s", "New high-priority report -> its imagery at GCS"),
            ("Timely sites, minimum fraction", 0.75, "fraction", "Sites whose imagery arrives <= 60 s after survey"),
            ("Minimum separation", 5.0, "m", "No controller-caused separation below this"),
            ("Collision proxy", 1.5, "m", "Geometric contact proxy; must never occur"),
            ("Minimum landing battery", 15.0, "%", "The energy reserve")]
    for i, (a, b, u, m) in enumerate(rows, 4):
        cs.cell(row=i, column=1, value=a).font = Font(name=F)
        c = cs.cell(row=i, column=2, value=b)
        c.font = Font(name=F, color="0000FF")
        c.fill = INPUT
        cs.cell(row=i, column=3, value=u).font = Font(name=F)
        cs.cell(row=i, column=4, value=m).font = Font(name=F)
    cs["A12"] = ("Blue text on yellow = inputs. Changing a threshold here re-evaluates every "
                 "check column on the 'All 60 Runs' sheet and the pass counts on 'Summary'. "
                 "Source: user-approved internal test thresholds (UAV-X publishes none for Stage 1).")
    cs["A12"].font = Font(name=F, italic=True, size=9)
    # named-like fixed refs
    IMG, LAND, HP, TIMELY, SEP, COLL, BATT = ("Constraints!$B$4", "Constraints!$B$5", "Constraints!$B$6",
                                              "Constraints!$B$7", "Constraints!$B$8", "Constraints!$B$9",
                                              "Constraints!$B$10")

    # ------------------------------------------------------------- All runs
    ws = wb.create_sheet("All 60 Runs")
    cols = [("#", 5), ("Set", 9), ("Scenario", 11), ("Fault", 15), ("Faulted UAV", 7),
            ("Imagery complete (s)", 10), ("Fleet landed (s)", 10), ("High-priority response (s)", 11),
            ("Sites within 60 s (fraction)", 10), ("Connectivity (%)", 10), ("Partitioned (%)", 10),
            ("Critical relay (%)", 10), ("Redundant (%)", 10), ("Disconnected UAV-s", 11),
            ("Heartbeat PDR (%)", 10), ("Command ack p95 (s)", 10), ("UAVs orphaned by fault", 9),
            ("Stable recovery confirmed (s)", 11), ("Fault UAV-s disconnected", 10), ("Handovers", 9),
            ("Min separation (m)", 10), ("Near misses", 8), ("Collisions", 8), ("Depletions", 8),
            ("Min landing battery (%)", 10),
            # formula check columns
            ("CHECK imagery <= limit", 10), ("CHECK HP <= limit", 10), ("CHECK timely >= min", 10),
            ("CHECK separation", 10), ("CHECK landing >= reserve", 10), ("CHECK no collision / depletion", 11),
            ("ALL CHECKS (formula)", 10), ("Simulator verdict", 10)]
    ws["A1"] = ("UAV-X v3.1 - all 60 simulations (E1: critical fault on the busiest relay, "
                "default planner adaptive-spare, 30 dev + 30 held-out scenarios)")
    ws["A1"].font = Font(name=F, bold=True, size=12)
    head(ws, 3, [c for c, _ in cols], [w for _, w in cols])
    r0 = 4
    all_rows = [("dev", r) for r in runs("dev")] + [("held-out", r) for r in runs("heldout")]
    for i, (s, r) in enumerate(all_rows):
        row = r0 + i
        vals = [i + 1, s, r["scenario"], r["fault_target"], r["fault_uav"], r["all_delivered_s"],
                r["fleet_landed_s"], r["hp_response_s"], r["timely_sites_frac"], r["connectivity_pct"],
                r["partitioned_pct"], r["critical_relay_pct"], r["redundant_pct"], r["disconnected_uav_s"],
                r["hb_pdr_pct"], r["cmd_ack_p95_s"], r["fault_orphaned"], r["fault_stable_confirmed_s"],
                r["fault_required_disconnected_uav_s"], r["handovers"], r["min_separation_m"],
                r["near_misses"], r["collisions"], r["depletions"], r["min_landing_battery_pct"]]
        for j, v in enumerate(vals, 1):
            c = ws.cell(row=row, column=j, value=v)
            c.font = Font(name=F, size=9)
            c.border = BOX
            c.alignment = Alignment(horizontal="center")
        # formula checks (reference the Constraints sheet)
        f = {26: f'=IF(F{row}<={IMG},"PASS","FAIL")',
             27: f'=IF(H{row}="","PASS",IF(H{row}<={HP},"PASS","FAIL"))',
             28: f'=IF(I{row}>={TIMELY},"PASS","FAIL")',
             29: f'=IF(AND(U{row}>={SEP},V{row}=0),"PASS","FAIL")',
             30: f'=IF(Y{row}>={BATT}-0.000001,"PASS","FAIL")',
             31: f'=IF(AND(W{row}=0,X{row}=0,G{row}<={LAND}),"PASS","FAIL")',
             32: f'=IF(COUNTIF(Z{row}:AE{row},"FAIL")=0,"PASS","FAIL")'}
        for j, formula in f.items():
            c = ws.cell(row=row, column=j, value=formula)
            c.font = Font(name=F, size=9, bold=(j == 32))
            c.border = BOX
            c.alignment = Alignment(horizontal="center")
        c = ws.cell(row=row, column=33, value="PASS" if r["operational_success"] else "FAIL")
        c.font = Font(name=F, size=9, color="008000")
        c.border = BOX
        c.alignment = Alignment(horizontal="center")
        for j, fmt in ((9, "0.0%"), (10, "0.0"), (11, "0.0"), (12, "0.0"), (13, "0.0"), (14, "0.0"),
                       (15, "0.0"), (16, "0.000"), (18, "0.00"), (19, "0.0"), (21, "0.00"), (25, "0.0")):
            ws.cell(row=row, column=j).number_format = fmt
    last = r0 + len(all_rows) - 1
    green = PatternFill("solid", fgColor="C6EFCE")
    red = PatternFill("solid", fgColor="FFC7CE")
    rng = f"Z{r0}:AG{last}"
    ws.conditional_formatting.add(rng, CellIsRule(operator="equal", formula=['"PASS"'], fill=green))
    ws.conditional_formatting.add(rng, CellIsRule(operator="equal", formula=['"FAIL"'], fill=red))
    ws.freeze_panes = "D4"
    ws.cell(row=last + 2, column=1, value=(
        "Columns A-Y: values written by the simulator (results/evaluation/E1-busiest__dev.json and "
        "__heldout.json). Blank 'Stable recovery confirmed' = no UAV was orphaned, so there was "
        "nothing to recover. Columns Z-AF: live Excel formulas against the Constraints sheet; "
        "AG: the simulator's own verdict, for cross-checking.")).font = Font(name=F, italic=True, size=9)
    ws["Z2"] = "Live formula checks ->"
    ws["Z2"].font = Font(name=F, bold=True, color="1F3864")
    ws.cell(row=3, column=32).comment = Comment("PASS only if every check in Z:AF passes.", "report")

    # ---------------------------------------------------------------- Summary
    sm = wb.create_sheet("Summary", 0)
    sm["A1"] = "UAV-X v3.1 - summary of the 60 simulations (all values are live formulas)"
    sm["A1"].font = Font(name=F, bold=True, size=13)
    sm["A2"] = ("Default planner adaptive-spare · E1 (critical fault on the busiest relay) · "
                "30 dev + 30 held-out saved scenarios")
    sm["A2"].font = Font(name=F, italic=True, size=10)
    head(sm, 4, ["Metric", "Column", "All 60: mean", "All 60: median", "All 60: best",
                 "All 60: worst", "Dev: mean", "Held-out: mean", "Runs with value"],
         [34, 8, 13, 13, 12, 12, 12, 13, 12])
    metrics = [("Imagery complete (s)", "F", "min", "max"), ("Fleet landed (s)", "G", "min", "max"),
               ("High-priority response (s)", "H", "min", "max"),
               ("Sites within 60 s (fraction)", "I", "max", "min"), ("Connectivity (%)", "J", "max", "min"),
               ("Time partitioned (%)", "K", "min", "max"), ("Time with a critical relay (%)", "L", "min", "max"),
               ("Time redundant (%)", "M", "max", "min"), ("Disconnected UAV-s", "N", "min", "max"),
               ("Heartbeat PDR (%)", "O", "max", "min"), ("Command ack p95 (s)", "P", "min", "max"),
               ("UAVs orphaned by fault", "Q", "min", "max"),
               ("Stable recovery confirmed (s)", "R", "min", "max"),
               ("Fault UAV-s disconnected", "S", "min", "max"), ("Min separation (m)", "U", "max", "min"),
               ("Min landing battery (%)", "Y", "max", "min")]
    R = f"'All 60 Runs'!"
    for i, (name, col, best, worst) in enumerate(metrics, 5):
        rng = f"{R}{col}{r0}:{col}{last}"
        sets = f"{R}B{r0}:B{last}"
        cells = [name, col,
                 f"=AVERAGE({rng})", f"=MEDIAN({rng})",
                 f"={best.upper()}({rng})", f"={worst.upper()}({rng})",
                 f'=AVERAGEIFS({rng},{sets},"dev")', f'=AVERAGEIFS({rng},{sets},"held-out")',
                 f"=COUNT({rng})"]
        for j, v in enumerate(cells, 1):
            c = sm.cell(row=i, column=j, value=v)
            c.font = Font(name=F, size=10)
            c.border = BOX
            if j >= 3:
                c.number_format = "0.00" if col in ("I", "P") else "0.0"
                if col == "I":
                    c.number_format = "0.0%"
    k = 5 + len(metrics) + 1
    sm.cell(row=k, column=1, value="Pass counts (formulas over the check columns)").font = Font(name=F, bold=True, size=11)
    head(sm, k + 1, ["Check", "Column", "All 60", "Dev (of 30)", "Held-out (of 30)", "", "", "", ""])
    checks = [("Imagery complete within limit", "Z"), ("High-priority response within limit", "AA"),
              ("Timely sites >= minimum", "AB"), ("Separation >= 5 m, no near miss", "AC"),
              ("Landing battery >= reserve", "AD"), ("No collision / depletion; fleet landed in time", "AE"),
              ("ALL CHECKS (operational success)", "AF")]
    for i, (name, col) in enumerate(checks, k + 2):
        rng = f"{R}{col}{r0}:{col}{last}"
        sets = f"{R}B{r0}:B{last}"
        vals = [name, col, f'=COUNTIF({rng},"PASS")', f'=COUNTIFS({rng},"PASS",{sets},"dev")',
                f'=COUNTIFS({rng},"PASS",{sets},"held-out")']
        for j, v in enumerate(vals, 1):
            c = sm.cell(row=i, column=j, value=v)
            c.font = Font(name=F, size=10, bold=(col == "AF"))
            c.border = BOX
    t = k + 2 + len(checks) + 1
    tot = [("Total near misses (< 5 m)", "V"), ("Total collisions (< 1.5 m proxy)", "W"),
           ("Total battery depletions", "X"), ("Total energy handovers", "T")]
    for i, (name, col) in enumerate(tot, t):
        sm.cell(row=i, column=1, value=name).font = Font(name=F, size=10)
        c = sm.cell(row=i, column=3, value=f"=SUM({R}{col}{r0}:{col}{last})")
        c.font = Font(name=F, size=10, bold=True)
        sm.cell(row=i, column=2, value=col).font = Font(name=F, size=10)
    sm.cell(row=t + len(tot) + 1, column=1, value=(
        "Best/worst: for 'lower is better' metrics best = MIN, worst = MAX, and the reverse for "
        "'higher is better'. Blank cells (e.g. no orphaned UAVs, so no recovery to time) are "
        "ignored by AVERAGE/MEDIAN; 'Runs with value' shows the count.")).font = Font(name=F, italic=True, size=9)

    # ------------------------------------------------------ Planner comparison
    pc = wb.create_sheet("Planner Comparison")
    pc["A1"] = "Same 60 scenarios, four planners (values from results/evaluation/*.json; median / worst)"
    pc["A1"].font = Font(name=F, bold=True, size=12)
    planners = ["adaptive-spare", "adaptive-reserve", "adaptive-conditional", "fixed-baseline"]
    head(pc, 3, ["Metric", "Set"] + planners, [40, 10, 18, 18, 20, 18])
    items = [("Operational success (runs of 30)", "operational_success", "count"),
             ("Eventually complete (runs of 30)", "eventually_complete", "count"),
             ("Imagery complete, median (s)", "all_delivered_s", "p50"),
             ("Imagery complete, worst (s)", "all_delivered_s", "worst"),
             ("High-priority response, worst (s)", "hp_response_s", "worst"),
             ("Connectivity, median (%)", "connectivity_pct", "p50"),
             ("Connectivity, worst (%)", "connectivity_pct", "worst"),
             ("UAVs orphaned by fault, mean", "fault_orphaned", "mean"),
             ("Stable recovery confirmed, median (s)", "fault_stable_confirmed_s", "p50"),
             ("Near misses (total events)", "near_misses (total events)", "raw"),
             ("Collisions (total events)", "collisions (total events)", "raw")]
    row = 4
    for set_name, lab in (("dev", "dev"), ("heldout", "held-out")):
        summ = json.load(open(os.path.join(EV, f"E1-busiest__{set_name}.json")))["summary"]
        for name, key, kind in items:
            pc.cell(row=row, column=1, value=name).font = Font(name=F, size=10)
            pc.cell(row=row, column=2, value=lab).font = Font(name=F, size=10)
            for j, p in enumerate(planners, 3):
                s = summ[p]
                if kind in ("count", "raw"):
                    v = s[key]
                else:
                    v = s[key].get(kind)
                c = pc.cell(row=row, column=j, value=v if v is not None else "none orphaned")
                c.font = Font(name=F, size=10)
                c.border = BOX
            row += 1
        row += 1
    pc.cell(row=row, column=1, value=(
        "Hardcoded values: copied from the benchmark summaries written by tools/evaluate.py "
        "(results/evaluation/E1-busiest__dev.json, __heldout.json).")).font = Font(name=F, italic=True, size=9)

    # ------------------------------------------------------------------ Notes
    nt = wb.create_sheet("Notes")
    notes = [
        "How these results were produced",
        "Command: python tools/evaluate.py  (saved scenarios in scenarios/dev and scenarios/heldout).",
        "Each scenario: 7 random sites 0.5-2.4 km from the GCS, random initial batteries (45-100 %),",
        "a critical fault (hard or failsafe) on the busiest relay at 100-260 s, a new high-priority site,",
        "a 15-60 s radio outage and a 35-55 % energy-availability fault on the GCS-side relay.",
        "Channel loss uses common random numbers, so every planner sees the same channel realisation.",
        "",
        "Definitions (full list in docs/METRICS.md and in the PDF report)",
        "Connectivity % = mean over time of (airborne UAVs with a route / airborne UAVs).",
        "Disconnected UAV-s = sum over 64 ms steps of (airborne UAVs without a route) x 0.064 s.",
        "Heartbeat PDR % = heartbeats delivered within 2 s / heartbeats generated (per send cohort).",
        "Stable recovery confirmed = start of the first 5 s interval with every orphaned UAV routed, + 5 s.",
        "Min separation = closest approach of any UAV pair, checked continuously between samples.",
        "",
        "All thresholds are internal test constraints, not organiser rules. Simulation only: kinematic",
        "flight, abstracted faults, uncalibrated radio model, time-compressed 60 s battery swap.",
    ]
    for i, n in enumerate(notes, 1):
        c = nt.cell(row=i, column=1, value=n)
        c.font = Font(name=F, bold=(i in (1, 8)), size=10)
    nt.column_dimensions["A"].width = 110

    wb.save(OUT)
    print(OUT)


if __name__ == "__main__":
    main()
