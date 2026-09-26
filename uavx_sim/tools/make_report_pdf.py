"""Build results/UAV-X_60_simulations_report.pdf (about 11 pages).

    python tools/make_report_pdf.py

Contents: what was simulated; every mathematical model and formula used in
the Webots/headless simulation (values taken from uavx/config.py); the
results of all 60 simulations (E1, default planner, 30 dev + 30 held-out
saved scenarios); summary statistics (read from the recalculated
results/UAV-X_60_simulations.xlsx); planner comparison; limitations.
"""

import json
import math
import os
import sys

from openpyxl import load_workbook
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (KeepTogether, PageBreak, Paragraph, SimpleDocTemplate, Spacer,
                                Table, TableStyle)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "controllers", "swarm_supervisor"))
from uavx import agent as A    # noqa: E402
from uavx import config as C   # noqa: E402

EV = os.path.join(ROOT, "results", "evaluation")
OUT = os.path.join(ROOT, "results", "UAV-X_60_simulations_report.pdf")
XLSX = os.path.join(ROOT, "results", "UAV-X_60_simulations.xlsx")

FD = "C:/Windows/Fonts/"
for name, fn in (("DV", "DejaVuSans.ttf"), ("DVB", "DejaVuSans-Bold.ttf"),
                 ("DVI", "DejaVuSans-Oblique.ttf"), ("DVS", "DejaVuSerif.ttf"),
                 ("DVSI", "DejaVuSerif-Italic.ttf"), ("DVM", "DejaVuSansMono.ttf"),
                 ("DVC", "DejaVuSansCondensed.ttf"), ("DVCB", "DejaVuSansCondensed-Bold.ttf"),
                 ("DVSBI", "DejaVuSerif-BoldItalic.ttf")):
    pdfmetrics.registerFont(TTFont(name, FD + fn))
pdfmetrics.registerFontFamily("DV", normal="DV", bold="DVB", italic="DVI", boldItalic="DVB")
pdfmetrics.registerFontFamily("DVSI", normal="DVSI", bold="DVSBI", italic="DVSI", boldItalic="DVSBI")

NAVY = colors.HexColor("#1F3864")
LIGHT = colors.HexColor("#EEF2F8")
GRID = colors.HexColor("#B7C3D6")
MUTE = colors.HexColor("#555555")

S = {
    "title": ParagraphStyle("t", fontName="DVB", fontSize=22, leading=27, textColor=NAVY),
    "sub": ParagraphStyle("s", fontName="DV", fontSize=11, leading=15, textColor=MUTE),
    "h1": ParagraphStyle("h1", fontName="DVB", fontSize=14, leading=18, textColor=NAVY,
                         spaceBefore=6, spaceAfter=6),
    "h2": ParagraphStyle("h2", fontName="DVB", fontSize=10.5, leading=14, textColor=NAVY,
                         spaceBefore=6, spaceAfter=3),
    "p": ParagraphStyle("p", fontName="DV", fontSize=9, leading=12.5, spaceAfter=4),
    "small": ParagraphStyle("sm", fontName="DV", fontSize=7.5, leading=10, textColor=MUTE),
    "eq": ParagraphStyle("eq", fontName="DVSI", fontSize=9.5, leading=13, alignment=TA_CENTER),
    "cell": ParagraphStyle("c", fontName="DVC", fontSize=6.6, leading=8, alignment=TA_CENTER),
    "cellL": ParagraphStyle("cl", fontName="DVC", fontSize=7.5, leading=9.5),
    "hcell": ParagraphStyle("hc", fontName="DVCB", fontSize=6.6, leading=8, alignment=TA_CENTER,
                            textColor=colors.white),
}


def P(t, st="p"):
    return Paragraph(t, S[st])


def eq(formula, note=None):
    """A numbered-looking equation box: formula (serif italic) + a plain note."""
    rows = [[Paragraph(formula, S["eq"])]]
    if note:
        rows.append([Paragraph(note, S["small"])])
    t = Table(rows, colWidths=[170 * mm])
    t.setStyle(TableStyle([("BACKGROUND", (0, 0), (-1, -1), LIGHT),
                           ("BOX", (0, 0), (-1, -1), 0.4, GRID),
                           ("TOPPADDING", (0, 0), (-1, -1), 2), ("BOTTOMPADDING", (0, 0), (-1, -1), 2)]))
    return KeepTogether([t, Spacer(1, 3)])


def table(data, widths, head_rows=1, font=7.5, zebra=True, left_col=False):
    rows = []
    for i, r in enumerate(data):
        st = S["hcell"] if i < head_rows else S["cell"]
        out = []
        for j, v in enumerate(r):
            s = st
            if left_col and j == 0 and i >= head_rows:
                s = S["cellL"]
            out.append(Paragraph("" if v is None else str(v), s))
        rows.append(out)
    t = Table(rows, colWidths=widths, repeatRows=head_rows)
    style = [("BACKGROUND", (0, 0), (-1, head_rows - 1), NAVY),
             ("GRID", (0, 0), (-1, -1), 0.3, GRID),
             ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
             ("TOPPADDING", (0, 0), (-1, -1), 1.5), ("BOTTOMPADDING", (0, 0), (-1, -1), 1.5),
             ("LEFTPADDING", (0, 0), (-1, -1), 1.5), ("RIGHTPADDING", (0, 0), (-1, -1), 1.5)]
    if zebra:
        for i in range(head_rows, len(rows)):
            if (i - head_rows) % 2:
                style.append(("BACKGROUND", (0, i), (-1, i), LIGHT))
    t.setStyle(TableStyle(style))
    return t


def fmt(v, nd=1):
    if v is None:
        return "–"
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, float):
        return f"{v:.{nd}f}"
    return str(v)


def load_runs():
    out = []
    for set_name, lab in (("dev", "dev"), ("heldout", "held-out")):
        d = json.load(open(os.path.join(EV, f"E1-busiest__{set_name}.json")))
        out += [(lab, r) for r in d["runs"] if r["planner"] == "adaptive-spare"]
    return out


def footer(canvas, doc):
    canvas.saveState()
    canvas.setFont("DV", 7.5)
    canvas.setFillColor(MUTE)
    canvas.drawString(20 * mm, 10 * mm, "UAV-X Stage 1 PoC v3.1 · 60-simulation results and mathematical model")
    canvas.drawRightString(190 * mm, 10 * mm, f"Page {doc.page}")
    canvas.restoreState()


def main():
    runs = load_runs()
    wb = load_workbook(XLSX, data_only=True)
    sm = wb["Summary"]
    summ = {}
    for r in sm.iter_rows(min_row=5, max_row=20, values_only=True):
        if r[0]:
            summ[r[0]] = r
    passes = {}
    for r in sm.iter_rows(min_row=22, max_row=45, values_only=True):
        if r[0] and r[2] is not None:
            passes[r[0]] = r
    story = []

    # ------------------------------------------------------------ page 1
    n_ok = sum(1 for _, r in runs if r["operational_success"])
    story += [Spacer(1, 18 * mm),
              P("UAV-X Resilient BVLOS Swarm", "title"),
              P("Results of all 60 simulations and the complete mathematical model", "sub"),
              Spacer(1, 3 * mm),
              P("Stage 1 proof of concept v3.1 · Webots R2025a + identical headless core · "
                "PUSHPAK Grand Challenge 2026", "sub"),
              Spacer(1, 10 * mm)]
    kpi = [["Simulations", "Passed every constraint", "Median imagery complete", "Median connectivity",
            "Near misses / collisions"],
           ["60", f"{n_ok} / 60", f"{summ['Imagery complete (s)'][3]:.0f} s",
            f"{summ['Connectivity (%)'][3]:.1f} %", "0 / 0"]]
    t = Table([[Paragraph(c, ParagraphStyle("k", fontName="DV", fontSize=8, alignment=TA_CENTER,
                                            textColor=MUTE)) for c in kpi[0]],
               [Paragraph(c, ParagraphStyle("k2", fontName="DVB", fontSize=17, leading=21,
                                            alignment=TA_CENTER, textColor=NAVY)) for c in kpi[1]]],
              colWidths=[34 * mm] * 5)
    t.setStyle(TableStyle([("BOX", (0, 0), (-1, -1), 0.5, GRID), ("INNERGRID", (0, 0), (-1, -1), 0.3, GRID),
                           ("BACKGROUND", (0, 0), (-1, -1), LIGHT), ("TOPPADDING", (0, 0), (-1, -1), 6),
                           ("BOTTOMPADDING", (0, 0), (-1, -1), 6)]))
    story += [t, Spacer(1, 8 * mm),
              P("What the 60 simulations are", "h1"),
              P("Five DJI Mavic-class UAVs operate from a forward ground control station (GCS) over a "
                "2 km × 2 km disaster area. Every search site lies beyond the 600 m radio range of the "
                "GCS, so imagery must travel UAV-to-UAV over a multi-hop mesh. Each simulation is one "
                "saved scenario file (<i>scenarios/dev</i> seeds 0–29 and <i>scenarios/heldout</i> seeds "
                "100–129) with:"),
              P("• 7 random survey sites 0.5–2.4 km from the GCS and random initial batteries (45–100 %);<br/>"
                "• a critical fault (hard: motor + radio, or failsafe: radio survives) on the busiest relay "
                "at a random time between 100 and 260 s (experiment <b>E1</b>);<br/>"
                "• a new high-priority site reported mid-mission;<br/>"
                "• a 15–60 s radio outage on another UAV;<br/>"
                "• a 35–55 % energy-availability fault on the relay next to the GCS, forcing a relay handover."),
              P("All 60 use the default planner (<b>adaptive-spare</b>). The held-out 30 were never used to "
                "choose any parameter. The same files were also run with three other planners for comparison "
                "(section 5). A run <b>passes</b> only if it meets every predeclared internal constraint "
                "(section 3.8); completing the mission eventually is not enough."),
              P("Companion workbook: <b>results/UAV-X_60_simulations.xlsx</b>, with all 60 rows, live "
                "Excel formula checks against the constraint sheet, and the summary statistics that this "
                "report quotes. The pass/fail formulas in Excel agree with the simulator's own verdict "
                "in all 60 runs.", "p"),
              P("Scope and honesty", "h2"),
              P("Simulation only. Flight is kinematic; the motor fault is an abstracted controlled "
                "descent; the energy fault is an instant loss of usable charge; the radio is a distance/"
                "loss abstraction with a single-channel airtime model; battery service is a 60 s swap "
                "(time-compressed). All thresholds are internal test constraints, not organiser rules.",
                "p"),
              PageBreak()]

    # ------------------------------------------------------ pages 2-6: math
    R, LB, LE = C.COMM_RANGE, C.LOSS_BASE, C.LOSS_EDGE
    story += [P("1. System parameters (uavx/config.py)", "h1")]
    params = [["Symbol", "Meaning", "Value"],
              ["Δt", "simulation step (= Webots basicTimeStep)", f"{C.DT*1000:.0f} ms"],
              ["N", "UAVs in the fleet", C.NUM_UAVS],
              ["v<sub>h</sub>, v<sub>z</sub>", "cruise speed, climb/descent speed", f"{C.CRUISE_SPEED} m/s, {C.CLIMB_SPEED} m/s"],
              ["h<sub>i</sub>", "cruise layer of UAV i", f"{C.BASE_ALT:.0f} + {C.ALT_STEP:.0f}(i − 1) m"],
              ["R", "radio range (link cut-off)", f"{R:.0f} m"],
              ["p<sub>0</sub>, p<sub>e</sub>", "per-hop loss at 0 m, extra loss at the edge", f"{LB}, {LE}"],
              ["s<sub>r</sub>", "planned relay spacing", f"{C.RELAY_SPACING:.0f} m (0.8 R)"],
              ["c<sub>m</sub>, c<sub>h</sub>", "battery drain moving / hovering", f"{C.DRAIN_CRUISE} %/s, {C.DRAIN_HOVER} %/s"],
              ["E<sub>res</sub>", "landing reserve", f"{C.RESERVE:.0f} %"],
              ["C<sub>ch</sub>", "shared channel rate", f"{C.LINK_RATE_MBPS} Mbit/s"],
              ["t<sub>mac</sub>, t<sub>hop</sub>", "MAC overhead per transmission, forwarding latency per hop", f"{C.MAC_OVERHEAD*1000:.1f} ms, {C.HOP_LATENCY*1000:.0f} ms"],
              ["k<sub>max</sub>", "per-hop MAC retries", C.MAC_RETRIES],
              ["B<sub>hb</sub>, B<sub>cmd</sub>, B<sub>thumb</sub>, B<sub>chunk</sub>", "packet sizes",
               f"{C.HB_BYTES} B, {C.CMD_BYTES} B, {C.THUMB_BYTES} B, {C.CHUNK_BYTES} B"],
              ["D", "imagery per site", f"{C.SURVEY_DATA_MB} MB = {A.N_CHUNKS} chunks"],
              ["T<sub>hb</sub>, T<sub>cmd</sub>, T<sub>lost</sub>", "heartbeat and command period, GCS lost timeout",
               f"{C.HEARTBEAT_PERIOD} s, {C.COMMAND_PERIOD} s, {C.LOST_TIMEOUT} s"],
              ["W", "stability hold for 'recovered'", f"{C.STABLE_WINDOW} s"],
              ["m<sub>ho</sub>, m<sub>crit</sub>", "handover margin, critical margin", f"{C.HANDOVER_MARGIN} %, {C.HANDOVER_CRITICAL} %"],
              ["d<sub>nm</sub>, d<sub>col</sub>", "near-miss and collision-proxy distances", f"{C.NEAR_MISS_M} m, {C.COLLISION_M} m"]]
    story += [table(params, [34 * mm, 90 * mm, 46 * mm], left_col=True), Spacer(1, 4 * mm)]

    story += [P("2. Motion, energy and onboard autonomy", "h1"),
              P("2.1 Kinematic flight", "h2"),
              P("Each UAV first climbs to its own layer, then moves horizontally, then descends only over "
                "its pad or target, so transits never cross another layer."),
              eq("<b>x</b><sub>t+Δt</sub> = <b>x</b><sub>t</sub> + min(v<sub>h</sub>Δt, ‖<b>g</b> − <b>x</b><sub>t</sub>‖) · "
                 "(<b>g</b> − <b>x</b><sub>t</sub>) / ‖<b>g</b> − <b>x</b><sub>t</sub>‖ + <b>a</b>Δt,"
                 "   z<sub>t+Δt</sub> = z<sub>t</sub> + clip(z* − z<sub>t</sub>, −v<sub>z</sub>Δt, v<sub>z</sub>Δt)",
                 "g = horizontal goal, a = separation push (section 2.4), z* = own layer h<sub>i</sub> in transit, "
                 "ground or pad height when descending; position hard-clamped to the geofence."),
              P("2.2 Battery and energy decisions", "h2"),
              eq("E<sub>t+Δt</sub> = max(0, E<sub>t</sub> − c Δt),   c = c<sub>m</sub> if moving, c<sub>h</sub> if hovering"),
              eq("E<sub>home</sub>(<b>x</b>) = ( ‖<b>x</b><sub>xy</sub> − <b>pad</b>‖ / v<sub>h</sub> + z / v<sub>z</sub> ) · c<sub>m</sub> + E<sub>res</sub>",
                 "energy (in %) needed to fly home and land from x, plus the 15 % reserve"),
              eq("Onboard return-to-home:   E ≤ E<sub>home</sub>(<b>x</b>) + 5 %",
                 "taken onboard, overrides any GCS task"),
              eq("Slot feasibility (GCS):   E &gt; E<sub>transit</sub> + T<sub>station</sub>·c<sub>h</sub> + E<sub>home</sub>(slot)",
                 f"E_transit = (climb/v_z + distance/v_h)·c_m, T_station = {C.ON_STATION_BUDGET:.0f} s"),
              eq("Handover margin:   m = E − E<sub>home</sub>(<b>x</b>);   start handover if m &lt; m<sub>ho</sub>;"
                 "   leave regardless if m &lt; m<sub>crit</sub>"),
              eq(f"Battery swap:   on landing, status = SERVICE for {C.SWAP_TIME:.0f} s, then E = 100 %"),
              P("2.3 Failsafe fault and lost-link rules", "h2"),
              eq("z<sub>t+Δt</sub> = max(z<sub>ground</sub>, z<sub>t</sub> − v<sub>fs</sub>Δt),   "
                 f"v<sub>fs</sub> = {C.EMERGENCY_DESCENT} m/s",
                 "critical fault: level controlled descent in place (abstracted); hard variant also switches the radio off"),
              P("A UAV silent for more than 3 s while holding unacknowledged imagery becomes a <i>data mule</i> "
                "and flies toward the GCS <b>until every chunk is acknowledged</b> (hysteresis). An orphaned relay, "
                "and every UAV in its radio fragment, falls back toward the GCS after 3 s of silence."),
              P("2.4 Separation assurance", "h2"),
              eq("vertical yield if  |Δx<sub>h</sub>| &lt; 60 m,  |Δz| &lt; 12 m  and  v<sub>z</sub>·(z<sub>other</sub> − z) &gt; 0;"
                 "   back off if |Δz| &lt; 8 m and |Δx<sub>h</sub>| &lt; 25 m"),
              eq("<b>a</b> = 6 · (d<sub>min</sub> − d)/d<sub>min</sub> · <b>u</b>  for d &lt; d<sub>min</sub> = 5 m;"
                 "   keep-out radius 15 m around a failsafe descent column",
                 "u = unit vector away from the other UAV (a fixed per-UAV direction if exactly stacked)")]

    story += [P("3. Radio, routing and the shared channel", "h1"),
              P("3.1 Link model", "h2"),
              eq(f"p(d) = p<sub>0</sub> + p<sub>e</sub> (d / R)<super>4</super>  for d ≤ R,   p(d) = 1  for d &gt; R"
                 f"   ( p<sub>0</sub> = {LB}, p<sub>e</sub> = {LE}, R = {R:.0f} m )",
                 "d = 3D distance between the two radios; p = probability one transmission attempt is lost"),
              eq(f"degraded profile: R = {C.DEGRADED_RANGE:.0f} m, p<sub>e</sub> = {C.DEGRADED_LOSS_EDGE};"
                 f"   shadowed: p ← min(1, p + {C.SHADOW_LOSS}(1 − c/{C.SHADOW_CLEARANCE:.0f})) if terrain clearance c &lt; {C.SHADOW_CLEARANCE:.0f} m, blocked if c &lt; 0",
                 "uncalibrated stress cases used only for sensitivity runs"),
              P("3.2 Minimum-ETX routing", "h2"),
              eq("ETX(a,b) = 1 / (1 − p(d<sub>ab</sub>)),     cost(n) = min<sub>paths</sub> Σ<sub>hops</sub> ETX",
                 "Dijkstra from the GCS every step; a relay being drained (handover) may not forward"),
              eq("P<sub>path</sub> = Π<sub>hops</sub> (1 − p<sub>i</sub>)",
                 "probability one packet survives every hop without retries"),
              P("3.3 Airtime, latency and retries", "h2"),
              eq(f"T<sub>air</sub> = 8B / C<sub>ch</sub> + t<sub>mac</sub>      e.g. a 32 KiB chunk: "
                 f"{8*C.CHUNK_BYTES/(C.LINK_RATE_MBPS*1e6)*1000+C.MAC_OVERHEAD*1000:.1f} ms per hop",
                 "the sender and every radio neighbour are busy for T_air (carrier sense); distant nodes can transmit in parallel"),
              eq("t<sub>arrive</sub> = t<sub>start</sub> + T<sub>air</sub> + t<sub>hop</sub>,     "
                 "t<sub>start</sub> = max(t<sub>ready</sub>, medium free)"),
              eq(f"hop success after at most k<sub>max</sub> + 1 = {C.MAC_RETRIES + 1} attempts:"
                 f"   P = 1 − p<super>{C.MAC_RETRIES + 1}</super>",
                 "each attempt costs airtime and is counted as a transmission"),
              eq("loss draw u = splitmix64(seed, link, ⌊t/Δt⌋, attempt) / 2<super>53</super>;  lost if u &lt; p",
                 "common random numbers: every planner sees the same channel realisation on the same link and slot"),
              P("3.4 Imagery transfer", "h2"),
              eq(f"N<sub>chunks</sub> = ⌈ D / B<sub>chunk</sub> ⌉ = ⌈ 1.5·10<super>6</super> / {C.CHUNK_BYTES} ⌉ = {A.N_CHUNKS}",
                 f"sliding window of {C.DATA_WINDOW} chunks, end-to-end acknowledgement, retransmit after {C.CHUNK_RTO:.0f} s; "
                 "a site is complete when all chunks are at the GCS"),
              P("3.5 Delivery ratio (PDR)", "h2"),
              eq("PDR = |{ packets delivered within their deadline }| / |{ packets generated }|",
                 "per class (heartbeat, command, ack, thumbnail); only packets whose 2 s deadline has passed; "
                 "a packet generated with no route counts as lost; empty set → N/A, never 100 %"),
              eq("generated = delivered + expired + dropped + dropped<sub>radio</sub> + lost + pending",
                 "exact accounting identity, checked by the test suite"),
              P("3.6 GCS planner", "h2"),
              eq("n<sub>relays</sub> = max(0, ⌈ d<sub>anchor→site</sub> / s<sub>r</sub> ⌉ − 1),"
                 "   relay k at  <b>a</b> + k/(n+1) · (<b>site</b> − <b>a</b>)",
                 "chains branch from the nearest existing anchor (GCS or planned relay), so relays form a tree"),
              eq("assignment = argmin<sub>perm</sub> Σ ‖<b>x</b><sub>uav</sub> − <b>slot</b>‖ − 30 m · [same slot as before]",
                 "exhaustive over ≤ 5! = 120 permutations, subject to the energy feasibility of section 2.2"),
              eq("silent surveyor kept for  min( 240 s,  2·d<sub>last→site</sub>/v<sub>h</sub> + 10 s + 60 s )",
                 "only if its heartbeat had acknowledged that survey task; otherwise the site is re-queued at once")]

    story += [P("3.7 Recovery after a fault (evaluator)", "h1"),
              P("Populations are frozen at the fault time t<sub>0</sub>: <b>orphaned</b> = UAVs that had a route before "
                "the fault and lost it; <b>required</b> = orphaned (+ the faulted UAV itself if the fault is a temporary "
                "radio outage). An aircraft loss is irrecoverable and never counted as recovered."),
              eq("t<sub>first</sub> = min{ t &gt; t<sub>0</sub> : ∃ u ∈ required with a route }   "
                 "t<sub>all</sub> = min{ t : ∀ u ∈ required have a route }"),
              eq("t<sub>stable</sub> = min{ t : ∀ τ ∈ [t, t + W], ∀ u ∈ required have a route },     "
                 "t<sub>confirmed</sub> = t<sub>stable</sub> + W,   W = 5 s",
                 "any disconnection resets the hold; all times reported relative to t0; 'none orphaned' → N/A, not 0"),
              eq("disconnected<sub>fault</sub> = Σ<sub>steps</sub> |{ u ∈ required without a route }| · Δt   (UAV·s)"),
              P("3.8 Network and mission metrics", "h2"),
              eq("Connectivity = mean<sub>t</sub> [ |airborne UAVs with a route| / |airborne UAVs| ] × 100 %"),
              eq("Disconnected UAV·s = Σ<sub>t</sub> |airborne UAVs without a route| · Δt"),
              eq("state(t) ∈ { partitioned,  connected with a critical relay,  connected & redundant }",
                 "critical relay = a UAV whose removal alone disconnects another routed UAV (articulation point); "
                 "shares are of time with ≥ 1 airborne UAV"),
              eq("survey→GCS latency = t<sub>all chunks at GCS</sub> − t<sub>survey done</sub>;   "
                 "high-priority response = t<sub>imagery at GCS</sub> − t<sub>report</sub>"),
              eq("operational success = [T<sub>img</sub> ≤ 900 s] ∧ [T<sub>land</sub> ≤ 1200 s] ∧ [T<sub>HP</sub> ≤ 240 s] ∧ "
                 "[f<sub>60s</sub> ≥ 0.75] ∧ [d<sub>min</sub> ≥ 5 m] ∧ [no collision] ∧ [no geofence breach] ∧ "
                 "[no depletion] ∧ [E<sub>landing</sub> ≥ 15 %]",
                 "predeclared internal constraints (config.CONSTRAINTS); implemented identically in the Excel check columns"),
              P("3.9 Safety: continuous closest approach", "h2"),
              P("Between two samples both UAVs move linearly. With relative position <b>r</b><sub>0</sub> and relative "
                "displacement <b>Δr</b> over the step:"),
              eq("s* = clip( −(<b>r</b><sub>0</sub>·<b>Δr</b>) / ‖<b>Δr</b>‖<super>2</super>, 0, 1 ),     "
                 "d<sub>min</sub> = ‖ <b>r</b><sub>0</sub> + s* <b>Δr</b> ‖",
                 "catches crossings that happen between 64 ms samples; consecutive steps below 5 m form one near-miss episode"),
              P("3.10 Onboard person counting (perception)", "h2"),
              P("For a person box with bottom-centre pixel (u, v) in a W × H image, camera yaw ψ, downward pitch θ "
                "and horizontal field of view φ:"),
              eq("f<sub>x</sub> = (W/2) / tan(φ/2),   a = −(u − W/2)/f<sub>x</sub>,   b = −(v − H/2)/f<sub>x</sub>"),
              eq("<b>ray</b> = <b>f</b> + a <b>l</b> + b <b>u</b>,   <b>f</b> = (cosθcosψ, cosθsinψ, −sinθ),  "
                 "<b>l</b> = (−sinψ, cosψ, 0),  <b>u</b> = (sinθcosψ, sinθsinψ, cosθ)"),
              eq("<b>p</b><sub>ground</sub> = <b>c</b> + λ<b>ray</b>,   λ = (z<sub>ground</sub> − c<sub>z</sub>) / ray<sub>z</sub>",
                 "c = camera position; measured projection error: median 0.06 m, worst 0.12 m"),
              eq("count = |{ tracks within 8 m of the site, seen in ≥ 20 % of settled frames }|,   "
                 "a detection joins a track if within 0.3 m",
                 "benchmark (72 views, matched to true positions): precision = TP/(TP+FP) = 0.992, recall = TP/(TP+FN) = 0.903"),
              PageBreak()]

    # ------------------------------------------ worked examples + code index
    from uavx.comms import loss_prob
    d1 = C.RELAY_SPACING
    p1 = loss_prob(d1)
    etx1 = 1 / (1 - p1)
    p600 = loss_prob(599.9)
    air_hb = 8 * C.HB_BYTES / (C.LINK_RATE_MBPS * 1e6) + C.MAC_OVERHEAD
    air_ch = 8 * C.CHUNK_BYTES / (C.LINK_RATE_MBPS * 1e6) + C.MAC_OVERHEAD
    far = (900.0, 900.0, C.BASE_ALT)
    e_home = A.energy_to_home_from(1, *far)
    dist_far = math.hypot(far[0] - A.pad_position(1)[0], far[1] - A.pad_position(1)[1])
    n_rel = max(0, math.ceil(2400 / C.RELAY_SPACING) - 1)
    hops = n_rel + 1
    t_chunk_path = hops * (air_ch + C.HOP_LATENCY)
    t_site = A.N_CHUNKS * hops * air_ch / min(C.DATA_WINDOW, hops)
    rows = [["Quantity", "Formula with numbers", "Result"],
            ["Loss on one 480 m relay hop", f"0.01 + 0.2·(480/600)<super>4</super>", f"{p1:.3f} ({p1*100:.1f} %)"],
            ["ETX of that hop", f"1 / (1 − {p1:.3f})", f"{etx1:.3f} transmissions"],
            ["Loss at the range edge (600 m)", "0.01 + 0.2·1<super>4</super>", f"{p600:.3f}"],
            ["Hop failure after all MAC retries (480 m)", f"{p1:.3f}<super>{C.MAC_RETRIES+1}</super>", f"{p1**(C.MAC_RETRIES+1):.2e}"],
            ["Relays to reach a site 2.4 km away", f"⌈2400/{C.RELAY_SPACING:.0f}⌉ − 1", f"{n_rel} relays ({hops} hops)"],
            ["Heartbeat airtime per hop", f"8·{C.HB_BYTES}/4·10<super>6</super> + 0.5 ms", f"{air_hb*1000:.2f} ms"],
            ["Imagery chunk airtime per hop", f"8·{C.CHUNK_BYTES}/4·10<super>6</super> + 0.5 ms", f"{air_ch*1000:.1f} ms"],
            ["One chunk over the 5-hop chain", f"{hops}·({air_ch*1000:.1f} + {C.HOP_LATENCY*1000:.0f}) ms", f"{t_chunk_path:.2f} s"],
            ["Whole site (46 chunks), shared channel, 5 hops", f"46·{hops}·{air_ch*1000:.1f} ms / {min(C.DATA_WINDOW, hops)} in parallel (lower bound)", f"≈ {t_site:.1f} s"],
            ["Energy to return from (900, 900) at 60 m", f"({dist_far:.0f}/15 + 60/5)·0.055 + 15", f"{e_home:.1f} %"],
            ["Endurance at cruise from 100 % to the reserve", "(100 − 15) / 0.055", f"{85/0.055/60:.1f} min"],
            ["Stability confirmation", "t<sub>stable</sub> + 5 s", "e.g. 15.1 → 20.2 s (demo run)"]]
    story += [P("3.11 Worked examples (numbers from the actual code)", "h1"),
              table(rows, [58 * mm, 72 * mm, 40 * mm], left_col=True), Spacer(1, 4 * mm),
              P("3.12 Where each formula lives (controllers/swarm_supervisor/)", "h2"),
              table([["Model", "Source", "Function"],
                     ["Kinematic flight, layers, yielding", "uavx/agent.py", "Uav._fly_to, _choose_target"],
                     ["Battery, return-home, swap", "uavx/agent.py", "energy_to_home_from, step, _on_pad"],
                     ["Slot energy feasibility", "uavx/agent.py", "energy_for_slot"],
                     ["Link loss, ETX routing, critical relays", "uavx/comms.py", "loss_prob, Network.update, critical_relays"],
                     ["Airtime, queues, retries, PDR", "uavx/comms.py", "Channel.step, cohort_pdr"],
                     ["Relay chains, assignment, handover", "uavx/gcs.py", "_plan, _assign, _handover, _retire"],
                     ["Silent-surveyor rule", "uavx/gcs.py", "_out_of_contact, _expected_silence"],
                     ["Separation assurance", "uavx/sim.py", "_collision_avoidance"],
                     ["Recovery, connectivity, safety metrics", "uavx/metrics.py", "FaultTracker, Metrics, seg_min_dist"],
                     ["Constraints", "uavx/sim.py", "evaluate_constraints"],
                     ["Ground projection, person tracks", "perception.py", "project, SiteCounter"]],
                    [62 * mm, 40 * mm, 68 * mm], left_col=True),
              PageBreak()]

    # --------------------------------------------------- pages 7-9: results
    story += [P("4. Results of all 60 simulations", "h1"),
              P("E1 (critical fault on the busiest relay), planner adaptive-spare. Columns: Img = all imagery at GCS; "
                "Land = surviving fleet landed; HP = high-priority report→imagery; 60s = share of sites whose imagery "
                "arrived ≤ 60 s after survey; Conn = connectivity; Part / Crit = % time partitioned / with a critical "
                "relay; Disc = disconnected UAV·s; PDR = heartbeat PDR; Orph = UAVs orphaned by the fault; Stab = stable "
                "recovery confirmed (– = none orphaned); Sep = minimum separation; Bat = lowest landing battery; "
                "HO = energy handovers; OK = passed every constraint.", "small"),
              Spacer(1, 2 * mm)]
    hdr = ["#", "Scenario", "Fault", "Img<br/>s", "Land<br/>s", "HP<br/>s", "60s<br/>%", "Conn<br/>%",
           "Part<br/>%", "Crit<br/>%", "Disc<br/>UAV·s", "PDR<br/>%", "Orph", "Stab<br/>s", "Sep<br/>m",
           "Bat<br/>%", "HO", "OK"]
    widths = [6, 12, 13, 9, 9, 8, 8, 9, 9, 9, 10, 9, 7, 9, 8, 8, 6, 11]
    widths = [w * mm for w in widths]
    for part, (a, b) in enumerate(((0, 30), (30, 60))):
        data = [hdr]
        for i, (s, r) in enumerate(runs[a:b], a + 1):
            data.append([i, r["scenario"].replace("seed_", "seed "), r["fault_target"].replace("-busiest", ""),
                         fmt(r["all_delivered_s"], 0), fmt(r["fleet_landed_s"], 0), fmt(r["hp_response_s"], 0),
                         fmt(100 * r["timely_sites_frac"], 0), fmt(r["connectivity_pct"]),
                         fmt(r["partitioned_pct"]), fmt(r["critical_relay_pct"]), fmt(r["disconnected_uav_s"], 0),
                         fmt(r["hb_pdr_pct"]), r["fault_orphaned"], fmt(r["fault_stable_confirmed_s"]),
                         fmt(r["min_separation_m"]), fmt(r["min_landing_battery_pct"], 0), r["handovers"],
                         "PASS" if r["operational_success"] else "FAIL"])
        story += [P(f"4.{part + 1} {'Development' if part == 0 else 'Held-out'} scenarios ({a + 1}–{b})", "h2"),
                  table(data, widths), Spacer(1, 3 * mm)]
        if part == 0:
            story.append(PageBreak())
    story += [P("All 60 runs: 0 near misses (&lt; 5 m), 0 collision proxies (&lt; 1.5 m), 0 geofence violations, "
                "0 battery depletions and 0 information-flow bypasses. Faults marked <i>hard</i> also switch the "
                "faulted UAV's radio off, so the GCS can only detect them by the 3 s heartbeat timeout.", "small"),
              PageBreak()]

    # ------------------------------------------------- page 10: statistics
    story += [P("4.3 Summary statistics (computed by formulas in the workbook)", "h1")]
    order = ["Imagery complete (s)", "Fleet landed (s)", "High-priority response (s)",
             "Sites within 60 s (fraction)", "Connectivity (%)", "Time partitioned (%)",
             "Time with a critical relay (%)", "Time redundant (%)", "Disconnected UAV-s", "Heartbeat PDR (%)",
             "Command ack p95 (s)", "UAVs orphaned by fault", "Stable recovery confirmed (s)",
             "Fault UAV-s disconnected", "Min separation (m)", "Min landing battery (%)"]
    data = [["Metric", "Mean", "Median", "Best", "Worst", "Dev mean", "Held-out mean", "n"]]
    for k in order:
        r = summ[k]
        nd = 3 if "fraction" in k or "p95" in k else 1
        data.append([k] + [fmt(float(x), nd) for x in r[2:8]] + [r[8]])
    story += [table(data, [58 * mm, 17 * mm, 17 * mm, 17 * mm, 17 * mm, 18 * mm, 20 * mm, 10 * mm], left_col=True),
              Spacer(1, 5 * mm)]
    data = [["Constraint check (Excel formula)", "All 60", "Dev (of 30)", "Held-out (of 30)"]]
    for k, r in passes.items():
        if k.startswith("Total") or k == "Check":
            continue
        data.append([k, r[2], r[3], r[4]])
    story += [table(data, [90 * mm, 25 * mm, 25 * mm, 30 * mm], left_col=True), Spacer(1, 3 * mm)]
    tot = [(k, r[2]) for k, r in passes.items() if k.startswith("Total")]
    story += [P(" · ".join(f"{k}: <b>{int(v)}</b>" for k, v in tot), "p"),
              P("Best/worst: for lower-is-better metrics best = minimum and worst = maximum, and the reverse for "
                "higher-is-better ones. The stable-recovery row covers only the runs in which the fault orphaned "
                "at least one UAV (n shown); in the other runs nothing had to recover.", "small"),
              PageBreak()]

    # ------------------------------------------ page 11: planner comparison
    story += [P("5. Same 60 scenarios, four planners", "h1"),
              P("adaptive-spare (default): relay chains re-planned every second, a backup relay only when a UAV is "
                "spare. adaptive-reserve: always holds one UAV as a backup relay. adaptive-conditional: holds a backup "
                "only for a relay carrying ≥ 3 UAVs. fixed-baseline: relay backbone assigned once and never re-planned. "
                "Identical UAVs, onboard rules, channel, energy and safety logic. Cells: median (worst).", "p")]
    planners = ["adaptive-spare", "adaptive-reserve", "adaptive-conditional", "fixed-baseline"]
    rows = [("Passed every constraint", "operational_success", "count"),
            ("Eventually completed", "eventually_complete", "count"),
            ("All imagery at GCS (s)", "all_delivered_s", "dist"),
            ("High-priority response (s)", "hp_response_s", "dist"),
            ("Connectivity (%)", "connectivity_pct", "dist"),
            ("Disconnected UAV·s", "disconnected_uav_s", "dist"),
            ("UAVs orphaned by fault (mean)", "fault_orphaned", "mean"),
            ("Stable recovery confirmed (s)", "fault_stable_confirmed_s", "dist"),
            ("Near misses · collisions", None, "safety")]
    for set_name, lab in (("dev", "Development (30)"), ("heldout", "Held-out (30)")):
        sd = json.load(open(os.path.join(EV, f"E1-busiest__{set_name}.json")))["summary"]
        data = [[lab] + [p.replace("adaptive-", "adaptive-<br/>") for p in planners]]
        for name, key, kind in rows:
            line = [name]
            for p in planners:
                s = sd[p]
                if kind == "count":
                    v = f"{s[key]}/{s['runs']}"
                elif kind == "safety":
                    v = f"{s['near_misses (total events)']} · {s['collisions (total events)']}"
                elif kind == "mean":
                    v = fmt(s[key].get("mean"))
                else:
                    dd = s[key]
                    v = (f"{fmt(dd['p50'])} ({fmt(dd['worst'])})" if dd.get("n") else "none orphaned")
                line.append(v)
            data.append(line)
        story += [table(data, [56 * mm, 29 * mm, 29 * mm, 29 * mm, 29 * mm], left_col=True), Spacer(1, 4 * mm)]
    ba = json.load(open(os.path.join(ROOT, "results", "before_after.json")))["table"]["adaptive-spare"]
    story += [P("Before (recorded v3 code) → after (v3.1), same 60 scenarios, adaptive-spare", "h2"),
              table([["Metric", "Before", "After"],
                     ["All imagery at GCS, median (s)"] + ba["all imagery at GCS, median s"],
                     ["Connectivity, median (%)"] + ba["connectivity %, median"],
                     ["Disconnected UAV·s, median"] + ba["disconnected UAV-s, median"],
                     ["UAVs orphaned by fault, mean"] + ba["UAVs orphaned by fault, mean"],
                     ["Minimum separation, worst (m)"] + ba["min separation m, worst"]],
                    [90 * mm, 35 * mm, 35 * mm], left_col=True),
              PageBreak()]

    # ------------------------------------------- page 12: sensitivity, limits
    sens = []
    for prof in ("shadowed", "degraded"):
        sd = json.load(open(os.path.join(EV, f"E3-radio-{prof}__dev.json")))["summary"]["adaptive-spare"]
        sens.append([prof, f"{sd['operational_success']}/30", fmt(sd["connectivity_pct"]["p50"]),
                     fmt(sd["all_delivered_s"]["p50"], 0), fmt(sd["all_delivered_s"]["worst"], 0)])
    e2 = {n: json.load(open(os.path.join(EV, f"E2-fixed__{n}.json")))["summary"]["adaptive-spare"]
          for n in ("dev", "heldout")}
    story += [P("6. Robustness beyond the 60 runs", "h1"),
              P("Radio stress profiles (development scenarios, adaptive-spare):", "p"),
              table([["Radio profile", "Passed", "Connectivity median (%)", "Imagery median (s)", "Imagery worst (s)"]]
                    + [["simple (the 60 runs, dev half)",
                        f"{sum(1 for s, r in runs[:30] if r['operational_success'])}/30",
                        fmt(sorted(r['connectivity_pct'] for s, r in runs[:30])[14]),
                        fmt(sorted(r['all_delivered_s'] for s, r in runs[:30])[14], 0),
                        fmt(max(r['all_delivered_s'] for s, r in runs[:30]), 0)]] + sens,
                    [40 * mm, 22 * mm, 38 * mm, 34 * mm, 34 * mm], left_col=True),
              Spacer(1, 3 * mm),
              P(f"Fault on the same aircraft id in every planner (E2): adaptive-spare passes "
                f"{e2['dev']['operational_success']}/30 (dev) and {e2['heldout']['operational_success']}/30 (held-out). "
                "The failures are high-priority responses of 272–276 s when a hard fault hits the UAV assigned to the "
                "high-priority site while it is still in radio range: the GCS cannot tell this from a normal survey "
                "leg and waits for the expected out-and-back time.", "p"),
              P("7. Limitations", "h1"),
              P("• Kinematic flight: no rotor or aerodynamic dynamics; the motor fault is an abstracted controlled descent.<br/>"
                "• Energy fault = instant loss of usable charge; no cell voltage/current model. Battery swap is time-compressed (60 s).<br/>"
                "• Radio: distance/loss model with a single-channel carrier-sense airtime abstraction; no hidden-terminal or "
                "interference model; the stress profiles are uncalibrated.<br/>"
                "• Perception: stock COCO YOLOv8n; people standing closer than about 0.5 m are merged into one box "
                "(recall 0.90 in the benchmark).<br/>"
                "• All thresholds are internal. This is a simulation study, not hardware validation or a competition-"
                "readiness claim.", "p"),
              P("8. Reproduce", "h1"),
              P("<font name='DVM' size='7.5'>python tools/evaluate.py            # the 60 runs (and the other planners)<br/>"
                "python tools/make_report_xlsx.py    # the workbook<br/>"
                "python tools/make_report_pdf.py     # this report<br/>"
                "python -m unittest discover -s tests   # 65 tests</font>", "p"),
              P("Sources: results/evaluation/E1-busiest__dev.json, E1-busiest__heldout.json, E2-fixed__*.json, "
                "E3-radio-*__dev.json, results/before_after.json, results/UAV-X_60_simulations.xlsx; model: "
                "controllers/swarm_supervisor/uavx/*.py and perception.py.", "small")]

    doc = SimpleDocTemplate(OUT, pagesize=A4, leftMargin=18 * mm, rightMargin=18 * mm,
                            topMargin=16 * mm, bottomMargin=18 * mm,
                            title="UAV-X 60-simulation results and mathematical model",
                            author="UAV-X team")
    doc.build(story, onFirstPage=footer, onLaterPages=footer)
    print(OUT)


if __name__ == "__main__":
    main()
