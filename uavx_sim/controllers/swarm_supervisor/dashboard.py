"""GCS dashboard: what the ground station knows, plus the video captions.

Renders a 1920x1080 frame with Pillow:

  +-----------------------------------+---------------------+
  |  GCS tactical map (live window)   |  header + KPIs      |
  |  -- or the Webots 3D view when    |  fleet as the GCS   |
  |     composited into the video --  |  knows it           |
  |  [caption band, bottom 136 px]    |  battery graph      |
  +-----------------------------------+  network + PDR graph|
  |  camera feeds as received at GCS  |  event log          |
  |  site tiles (GCS knowledge)       |      640 x 1080     |
  +-----------------------------------+---------------------+

Everything except the 3D view is built from GCS knowledge: telemetry and
imagery that crossed the mesh, each value with the age of its last update.
When there is no caption the caption band is filled with KEY colour, which
tools/compose_video.py keys out so the 3D view shows through.
"""

import math
import os

from PIL import Image, ImageDraw, ImageFont

W, H = 1920, 1080
MAP_W, MAP_H = 1280, 720
CAP_H = 136
TOP_H = 46                      # header strip over the 3D view (video only)
KEY = (255, 0, 255)

BG = (13, 17, 23)
PANEL = (22, 27, 34)
PANEL2 = (30, 36, 45)
GRID = (48, 56, 68)
TEXT = (230, 237, 243)
MUTED = (139, 148, 158)
ACCENT = (88, 166, 255)
GOOD = (63, 185, 80)
WARN = (210, 153, 34)
BAD = (248, 81, 73)
MAGENTA = (219, 97, 212)

UAV_RGB = [(230, 51, 51), (51, 140, 242), (242, 191, 26), (153, 77, 217), (26, 191, 140), (242, 115, 26)]
ROLE_RGB = {
    "SURVEY": (46, 204, 96), "RELAY": (60, 150, 255), "BACKUP": (120, 200, 255),
    "HANDOVER": (255, 190, 60), "HOLD": (150, 150, 150),
    "RECALLED": (190, 190, 190), "RTH": (255, 150, 30), "SERVICE": (240, 210, 40),
    "READY": (225, 225, 225), "LINK LOST": (255, 60, 190),
    "FAILED": (120, 30, 30), "EMERGENCY": (255, 40, 40),
}
POI_RGB = {"pending": (230, 60, 50), "high": MAGENTA, "surveying": (255, 150, 20),
           "held": (240, 220, 40), "delivered": (63, 200, 90)}


def _font(names, size):
    for n in names:
        for d in ("C:/Windows/Fonts", "/usr/share/fonts/truetype/dejavu", "/Library/Fonts"):
            p = os.path.join(d, n)
            if os.path.exists(p):
                return ImageFont.truetype(p, size)
    try:
        return ImageFont.load_default(size)
    except TypeError:
        return ImageFont.load_default()


SANS = ["segoeui.ttf", "DejaVuSans.ttf", "Arial.ttf"]
BOLD = ["segoeuib.ttf", "DejaVuSans-Bold.ttf", "Arial Bold.ttf"]
MONO = ["consola.ttf", "DejaVuSansMono.ttf", "Menlo.ttc"]
F = {
    "h1": _font(BOLD, 26), "h2": _font(BOLD, 18), "b": _font(BOLD, 16),
    "t": _font(SANS, 16), "s": _font(SANS, 14), "xs": _font(SANS, 12),
    "m": _font(MONO, 15), "big": _font(BOLD, 34), "kpi": _font(BOLD, 28),
    "cap_t": _font(BOLD, 24), "cap_l": _font(BOLD, 15), "cap": _font(SANS, 17),
}


def _mmss(t):
    return f"{int(t // 60):02d}:{t % 60:04.1f}"


def _fit(d, text, font, width):
    if d.textlength(text, font=font) <= width:
        return text
    while text and d.textlength(text + "…", font=font) > width:
        text = text[:-1]
    return text + "…"


class Dashboard:
    def __init__(self, geofence, gcs_pos, comm_range, map_features=None):
        self.fence = geofence
        self.gcs = gcs_pos
        self.range = comm_range
        self.features = map_features or {}
        self.hist = {"t": [], "batt": {}, "air": [], "linked": [], "pdr": []}
        self.next_sample = 0.0
        self.marks = []                       # (t, kind)
        self._seen_faults = 0
        self._seen_pois = None

    # --------------------------------------------------------------- data
    def sample(self, st):
        if st["t"] < self.next_sample:
            return
        self.next_sample = st["t"] + 0.5
        h = self.hist
        h["t"].append(st["t"])
        for u in st["uavs"]:
            if u["known"]:
                h["batt"].setdefault(u["id"], []).append((st["t"], u["battery"]))
        h["air"].append(st["airborne"])
        h["linked"].append(st["linked"])
        h["pdr"].append(st["pdr10"])
        for f in st["faults"][self._seen_faults:]:
            self.marks.append((f["t"], {"uav_failure": "fail", "comm_outage": "radio",
                                        "battery_sag": "batt"}[f["type"]]))
        self._seen_faults = len(st["faults"])
        n = st["n_sites"]
        if self._seen_pois is not None and n > self._seen_pois:
            self.marks.append((st["t"], "poi"))
        self._seen_pois = n

    # ------------------------------------------------------------- render
    def render(self, st, with_map=True):
        self.sample(st)
        im = Image.new("RGB", (W, H), BG)
        d = ImageDraw.Draw(im)
        if with_map:
            self._map(d, st)
        self._caption(d, st, with_map)
        self._right(d, st)
        self._feeds(im, d, st)
        return im

    # ----- caption band ---------------------------------------------------
    def _topbar(self, d):
        d.rectangle([0, 0, MAP_W - 1, TOP_H - 1], fill=(10, 13, 18))
        d.text((16, 6), "3D VIEW = SIMULATOR TRUTH", font=F["b"], fill=(160, 210, 255))
        d.text((16, 26), "observer's view, not available to the GCS", font=F["xs"], fill=MUTED)
        x = 330
        for col, txt in (((255, 210, 40), "active route"), ((80, 220, 255), "radio link"),
                         ((46, 204, 96), "survey"), ((60, 150, 255), "relay"),
                         ((255, 150, 30), "RTH"), ((190, 90, 255), "data mule"),
                         ((255, 40, 40), "emergency")):
            if x == 330 or txt == "survey":
                if txt == "survey":
                    d.text((x, 14), "drone LEDs:", font=F["s"], fill=MUTED)
                    x += 86
            d.rectangle([x, 18, x + 18, 28], fill=col)
            d.text((x + 24, 13), txt, font=F["s"], fill=TEXT)
            x += 34 + d.textlength(txt, font=F["s"])

    def _caption(self, d, st, with_map):
        if not with_map:
            self._topbar(d)
        y0 = MAP_H - CAP_H
        cap = st.get("caption")
        if not cap:
            if not with_map:
                d.rectangle([0, y0, MAP_W - 1, MAP_H - 1], fill=KEY)
            return
        title, lines, col = cap
        d.rectangle([0, y0, MAP_W - 1, MAP_H - 1], fill=(10, 13, 18))
        d.rectangle([0, y0, 7, MAP_H - 1], fill=col)
        d.text((22, y0 + 8), title, font=F["cap_t"], fill=col)
        y = y0 + 42
        for label, text in lines[:4]:
            d.text((22, y + 2), label, font=F["cap_l"], fill=MUTED)
            d.text((150, y), _fit(d, text, F["cap"], MAP_W - 170), font=F["cap"], fill=TEXT)
            y += 23

    # ----- tactical map (GCS knowledge) ------------------------------------
    def _map(self, d, st):
        d.rectangle([0, 0, MAP_W - 1, MAP_H - 1], fill=(16, 24, 20))
        xmin, xmax, ymin, ymax = self.fence
        span = (xmax - xmin) * 1.2
        sz = MAP_H - 40
        ox, oy = 30, 20

        def P(x, y):
            return (ox + (x + span / 2) / span * sz, oy + (span / 2 - y) / span * sz)

        for poly, col in self.features.get("areas", []):
            d.polygon([P(*p) for p in poly], fill=col)
        for (x1, y1, x2, y2, w) in self.features.get("roads", []):
            d.line([P(x1, y1), P(x2, y2)], fill=(70, 72, 76), width=w)
        d.rectangle([P(xmin, ymax), P(xmax, ymin)], outline=(255, 110, 60), width=2)
        gx, gy = self.gcs[0], self.gcs[1]
        c = P(gx, gy)
        r = self.range / span * sz
        d.ellipse([c[0] - r, c[1] - r, c[0] + r, c[1] + r], outline=(70, 130, 220), width=1)
        pos = st["positions"]
        for (a, b) in st["tree"]:
            if a in pos and b in pos:
                d.line([P(*pos[a][:2]), P(*pos[b][:2])], fill=(255, 210, 40), width=3)
        for (x, y) in st["relay_slots"]:
            q = P(x, y)
            d.ellipse([q[0] - 7, q[1] - 7, q[0] + 7, q[1] + 7], outline=(60, 150, 255), width=1)
        for p in st["sites"]:
            q = P(p["x"], p["y"])
            d.rectangle([q[0] - 6, q[1] - 6, q[0] + 6, q[1] + 6], fill=POI_RGB[p["state"]])
            d.text((q[0] + 9, q[1] - 8), p["id"], font=F["s"], fill=TEXT)
        d.rectangle([c[0] - 8, c[1] - 8, c[0] + 8, c[1] + 8], fill=(240, 240, 240))
        d.text((c[0] + 11, c[1] - 6), "GCS", font=F["b"], fill=TEXT)
        for u in st["uavs"]:
            if not u["known"]:
                continue
            q = P(u["x"], u["y"])
            col = UAV_RGB[(u["id"] - 1) % len(UAV_RGB)]
            if u["lost"]:
                d.ellipse([q[0] - 9, q[1] - 9, q[0] + 9, q[1] + 9], outline=col, width=2)
                d.text((q[0] + 11, q[1] + 2), f"U{u['id']} ? {u['age']:.0f}s", font=F["s"], fill=col)
            else:
                d.ellipse([q[0] - 7, q[1] - 7, q[0] + 7, q[1] + 7], fill=col, outline=(0, 0, 0))
                d.text((q[0] + 10, q[1] + 2), f"U{u['id']}", font=F["s"], fill=col)
        lx = ox + sz + 30
        d.text((lx, 30), "GCS TACTICAL MAP", font=F["h2"], fill=TEXT)
        d.text((lx, 56), "positions as last reported", font=F["s"], fill=MUTED)
        items = [((255, 210, 40), "route reported by each UAV"),
                 ((60, 150, 255), "planned relay slot"), ((70, 130, 220), "GCS radio range"),
                 ((255, 110, 60), "geofence")]
        for k, (col, txt) in enumerate(items):
            y = 90 + k * 26
            d.line([(lx, y + 9), (lx + 30, y + 9)], fill=col, width=3)
            d.text((lx + 40, y), txt, font=F["t"], fill=MUTED)
        d.text((lx, 200), "hollow marker + age = LINK LOST", font=F["t"], fill=MUTED)
        d.text((lx, 360), "Close this window to hide it; the", font=F["s"], fill=MUTED)
        d.text((lx, 378), "simulation keeps running in Webots.", font=F["s"], fill=MUTED)

    # ----- right column ---------------------------------------------------
    def _right(self, d, st):
        x0 = MAP_W
        d.rectangle([x0, 0, W, H], fill=PANEL)
        d.text((x0 + 18, 8), "UAV-X  GROUND CONTROL", font=F["h1"], fill=TEXT)
        d.text((x0 + 18, 40), "GCS VIEW: only what has reached the GCS over the mesh",
               font=F["s"], fill=ACCENT)
        d.text((W - 18, 6), f"T+{_mmss(st['t'])}", font=F["big"], fill=ACCENT, anchor="ra")
        n, k = st["n_sites"], st["delivered"]
        y = 66
        d.rounded_rectangle([x0 + 18, y, W - 18, y + 22], 6, fill=PANEL2)
        if n:
            d.rounded_rectangle([x0 + 18, y, x0 + 18 + (W - x0 - 36) * k / n, y + 22], 6, fill=GOOD)
        label = f"{k}/{n} sites: full imagery at GCS"
        if st["complete"]:
            label += f"  ·  complete t={st['complete_t']:.0f}s"
            if st.get("fleet_confirmed_t") is not None:
                label += f"  ·  fleet landed (GCS-confirmed) t={st['fleet_confirmed_t']:.0f}s"
        d.text(((x0 + W) / 2, y + 11), label, font=F["b"],
               fill=(10, 14, 18) if k == n and n else TEXT, anchor="mm")

        def pct(v):
            return "N/A" if v is None else f"{v:.0f}%"
        age = st["max_age"]
        kpis = [("GCS LINK", f"{st['linked']}/{st['airborne']}",
                 GOOD if st["linked"] == st["airborne"] else BAD),
                ("HB PDR, LAST 10 s", pct(st["pdr10"]),
                 MUTED if st["pdr10"] is None else (GOOD if st["pdr10"] > 80 else
                                                    WARN if st["pdr10"] > 50 else BAD)),
                ("HB PDR TOTAL", pct(st["pdr_all"]), ACCENT),
                ("ACTIVE DATA AGE", "N/A" if age is None else f"{age:.1f} s",
                 MUTED if age is None else (GOOD if age < 2 else WARN if age < 10 else BAD))]
        kw = (W - x0 - 36 - 3 * 10) / 4
        for i, (lab, val, col) in enumerate(kpis):
            bx = x0 + 18 + i * (kw + 10)
            d.rounded_rectangle([bx, 98, bx + kw, 160], 8, fill=PANEL2)
            d.text((bx + 10, 102), lab, font=F["xs"], fill=MUTED)
            d.text((bx + 10, 118), val, font=F["kpi"], fill=col)
        miss = st.get("missing") or []
        pop = (f"roster {st.get('roster', 5)} · GCS-believed airborne {st['airborne']} · linked "
               f"{st['linked']}")
        if miss:
            pop += "  ·  MISSING: " + ", ".join(f"UAV{u} ({a:.0f} s)" for u, a in miss)
        d.text((x0 + 18, 163), pop, font=F["xs"], fill=BAD if miss else MUTED)
        d.text((x0 + 18, 177), "PDR = heartbeats delivered within 2 s / sent, per send cohort; "
               "no route = lost; N/A = none sent", font=F["xs"], fill=MUTED)
        self._fleet(d, st, x0 + 18, 194, W - 18)
        self._chart_batt(d, x0 + 18, 516, W - 18, 600, st)
        self._chart_net(d, x0 + 18, 634, W - 18, 724, st)
        self._events(d, st, x0 + 18, 738, W - 18, H - 8)

    def _fleet(self, d, st, x0, y0, x1):
        d.text((x0, y0), "FLEET (as last reported)", font=F["h2"], fill=TEXT)
        cols = [("UAV", 0), ("ROLE", 62), ("TASK", 196), ("BATTERY", 262), ("HEARD", 410),
                ("ROUTE", 500)]
        yh = y0 + 26
        for name, dx in cols:
            d.text((x0 + dx, yh), name, font=F["xs"], fill=MUTED)
        for k, u in enumerate(st["uavs"]):
            y = yh + 18 + k * 51
            d.rounded_rectangle([x0, y, x1, y + 46], 6, fill=PANEL2)
            col = UAV_RGB[(u["id"] - 1) % len(UAV_RGB)]
            d.rectangle([x0, y, x0 + 5, y + 46], fill=col)
            d.text((x0 + 12, y + 12), f"UAV{u['id']}", font=F["b"], fill=TEXT)
            if not u["known"]:
                d.text((x0 + 62, y + 12), "no telemetry yet", font=F["t"], fill=MUTED)
                continue
            stale = u["lost"]
            role = u["role"]
            rc = ROLE_RGB.get(role, (150, 150, 150))
            d.rounded_rectangle([x0 + 62, y + 10, x0 + 188, y + 36], 10, fill=rc)
            d.text((x0 + 125, y + 23), role, font=F["xs"], fill=(10, 12, 14), anchor="mm")
            vcol = MUTED if stale else TEXT
            d.text((x0 + 196, y + 12), u["task"] or "-", font=F["t"], fill=vcol)
            b = max(0.0, min(100.0, u["battery"]))
            bc = MUTED if stale else (GOOD if b > 50 else (WARN if b > 25 else BAD))
            d.rounded_rectangle([x0 + 262, y + 16, x0 + 352, y + 30], 4, fill=GRID)
            d.rounded_rectangle([x0 + 262, y + 16, x0 + 262 + 90 * b / 100, y + 30], 4, fill=bc)
            d.text((x0 + 358, y + 12), f"{b:.0f}%", font=F["t"], fill=vcol)
            age = u["age"]
            ac = GOOD if age < 1.5 else (WARN if age < C_LOST else BAD)
            d.text((x0 + 410, y + 12), f"{age:.1f} s ago" if age < 100 else f"{age:.0f} s ago",
                   font=F["t"], fill=ac)
            if stale:
                d.text((x0 + 500, y + 12), "LINK LOST", font=F["b"], fill=BAD)
            elif u["hops"]:
                d.text((x0 + 500, y + 12), f"{u['hops']} hop{'s' if u['hops'] != 1 else ''}",
                       font=F["t"], fill=TEXT)
            else:
                d.text((x0 + 500, y + 12), "-", font=F["t"], fill=MUTED)

    def _axes(self, d, x0, y0, x1, y1, title, ymax, t_now, legend=()):
        d.text((x0, y0 - 24), title, font=F["h2"], fill=TEXT)
        lx = x1
        for txt, col in reversed(legend):
            w = d.textlength(txt, font=F["xs"])
            lx -= w + 22
            d.rectangle([lx, y0 - 16, lx + 10, y0 - 8], fill=col)
            d.text((lx + 14, y0 - 20), txt, font=F["xs"], fill=MUTED)
        d.rectangle([x0, y0, x1, y1], fill=PANEL2)
        for g in range(1, 4):
            yy = y0 + (y1 - y0) * g / 4
            d.line([(x0, yy), (x1, yy)], fill=GRID, width=1)
        tmax = max(60.0, t_now * 1.05)
        tick = 30 if tmax <= 200 else (60 if tmax <= 500 else 120)
        for tt in range(0, int(tmax) + 1, tick):
            xx = x0 + (x1 - x0) * tt / tmax
            d.line([(xx, y0), (xx, y1)], fill=GRID, width=1)
            d.text((xx + 2, y1 - 14), f"{tt}s", font=F["xs"], fill=MUTED)
        for (t, kind) in self.marks:
            xx = x0 + (x1 - x0) * t / tmax
            col = {"fail": BAD, "radio": MAGENTA, "poi": WARN, "batt": (255, 190, 60)}[kind]
            d.line([(xx, y0), (xx, y1)], fill=col, width=2)

        def P(t, v):
            return (x0 + (x1 - x0) * t / tmax, y1 - (y1 - y0) * min(v, ymax) / ymax)
        return P

    def _chart_batt(self, d, x0, y0, x1, y1, st):
        leg = [(f"U{i}", UAV_RGB[(i - 1) % 6]) for i in sorted(self.hist["batt"])]
        P = self._axes(d, x0, y0, x1, y1, "BATTERY % (reported)", 100, st["t"], leg)
        for uid, pts in self.hist["batt"].items():
            if len(pts) > 1:
                d.line([P(t, v) for t, v in pts], fill=UAV_RGB[(uid - 1) % 6], width=2)

    def _chart_net(self, d, x0, y0, x1, y1, st):
        top = 5
        P = self._axes(d, x0, y0, x1, y1, "NETWORK (GCS view)", top, st["t"],
                       [("airborne", GRID), ("linked", GOOD), ("HB PDR 10 s", ACCENT)])
        ts = self.hist["t"]
        if len(ts) > 1:
            poly = [P(ts[0], 0)] + [P(t, v) for t, v in zip(ts, self.hist["air"])] + [P(ts[-1], 0)]
            d.polygon(poly, fill=(55, 64, 78))
            poly = [P(ts[0], 0)] + [P(t, v) for t, v in zip(ts, self.hist["linked"])] + [P(ts[-1], 0)]
            d.polygon(poly, fill=(40, 110, 60))
            seg = []
            for t, v in zip(ts, self.hist["pdr"]):
                if v is None:
                    if len(seg) > 1:
                        d.line(seg, fill=ACCENT, width=2)
                    seg = []
                else:
                    seg.append(P(t, v / 100.0 * top))
            if len(seg) > 1:
                d.line(seg, fill=ACCENT, width=2)

    def _events(self, d, st, x0, y0, x1, y1):
        d.text((x0, y0), "EVENT LOG", font=F["h2"], fill=TEXT)
        d.text((x1, y0 + 4), "GCS = known to ground station · OBS = observer only", font=F["xs"],
               fill=MUTED, anchor="ra")
        skip = ("GCS: task", "swapped, ready", "launching for")
        evs = [(t, m) for t, m in st["events"] if not any(s in m for s in skip)]
        y = y0 + 28
        n = int((y1 - y) // 24)
        for t, msg in evs[-n:]:
            col = MUTED
            if msg.startswith("FAULT") or "declared LOST" in msg or "split again" in msg:
                col = BAD
            elif msg.startswith("RECOVERY") or "re-acquired" in msg or "complete" in msg \
                    or "released" in msg:
                col = GOOD
            elif "HIGH-PRIORITY" in msg or "handover" in msg or "relieve" in msg:
                col = WARN
            tag = "GCS" if msg.startswith("GCS") else "OBS"
            body = msg[5:] if tag == "GCS" else msg
            d.text((x0, y), _mmss(t), font=F["m"], fill=MUTED)
            d.text((x0 + 70, y), tag, font=F["m"], fill=ACCENT if tag == "GCS" else MUTED)
            d.text((x0 + 108, y), _fit(d, body, F["m"], x1 - x0 - 108), font=F["m"], fill=col)
            y += 24

    # ----- camera strip + site tiles ----------------------------------------
    def _feeds(self, im, d, st):
        y0 = MAP_H
        d.rectangle([0, y0, MAP_W, H], fill=BG)
        det = "boxes = onboard YOLOv8n" if st["detector"] else "detector not installed"
        d.text((12, y0 + 5), "CAMERA FEEDS AS RECEIVED AT GCS  ·  1 thumbnail/s per UAV over the "
               f"mesh  ·  {det}", font=F["b"], fill=TEXT)
        fw, fh = 248, 186
        gap = (MAP_W - 5 * fw) / 6
        for k, u in enumerate(st["uavs"][:5]):
            x = int(gap + k * (fw + gap))
            y = y0 + 30
            feed = st["feeds"].get(u["id"])
            col = UAV_RGB[(u["id"] - 1) % 6]
            d.rectangle([x - 2, y - 2, x + fw + 1, y + fh + 1], outline=col, width=2)
            if feed and feed.get("img") is not None:
                img = feed["img"].resize((fw, fh))
                sx, sy = fw / feed["img"].width, fh / feed["img"].height
                if not feed["live"]:
                    img = Image.blend(img, Image.new("RGB", img.size, (0, 0, 0)), 0.45)
                im.paste(img, (x, y))
                for (lab, conf, x1, y1, x2, y2) in feed.get("dets", []):
                    c = (60, 255, 120) if lab == "person" else (255, 200, 60)
                    d.rectangle([x + x1 * sx, y + y1 * sy, x + x2 * sx, y + y2 * sy], outline=c, width=2)
            else:
                d.rectangle([x, y, x + fw, y + fh], fill=(8, 8, 8))
                d.text((x + fw / 2, y + fh / 2), "no frame received yet", font=F["s"],
                       fill=MUTED, anchor="mm")
            d.rectangle([x, y, x + fw, y + 20], fill=(0, 0, 0))
            role = u.get("role", "")
            d.text((x + 4, y + 2), f"UAV{u['id']}  {role}  {u.get('task') or ''}", font=F["xs"], fill=col)
            if u.get("known"):
                d.text((x + fw - 4, y + 2), f"{u['alt']:.0f} m", font=F["xs"], fill=TEXT, anchor="ra")
            if feed and feed.get("img") is not None and not feed["live"]:
                d.rectangle([x, y + fh - 24, x + fw, y + fh], fill=(110, 70, 0))
                d.text((x + fw / 2, y + fh - 12), f"NO NEW FRAME · last {feed['age']:.0f} s ago",
                       font=F["s"], fill=TEXT, anchor="mm")
            elif feed and feed.get("dets"):
                npers = sum(1 for dd in feed["dets"] if dd[0] == "person")
                if npers:
                    d.rectangle([x, y + fh - 22, x + fw, y + fh], fill=(10, 90, 40))
                    d.text((x + fw / 2, y + fh - 11), f"{npers} PERSON(S) · onboard",
                           font=F["s"], fill=TEXT, anchor="mm")
        # Site tiles (GCS knowledge)
        y = y0 + 30 + fh + 8
        d.text((12, y), "SEARCH SITES  (complete = full imagery at GCS; people = onboard detector"
               " vs simulator ground truth)", font=F["b"], fill=TEXT)
        sites = st["sites"][:10]
        tw = (MAP_W - 24 - (len(sites) - 1) * 8) / max(1, len(sites))
        for k, p in enumerate(sites):
            x = 12 + k * (tw + 8)
            yy = y + 24
            col = POI_RGB[p["state"]]
            d.rounded_rectangle([x, yy, x + tw, yy + 104], 6, fill=PANEL)
            d.rectangle([x, yy, x + tw, yy + 5], fill=col)
            d.text((x + 8, yy + 9), p["id"] + ("  HIGH" if p["prio"] > 1 else ""), font=F["b"],
                   fill=MAGENTA if p["prio"] > 1 else TEXT)
            d.text((x + 8, yy + 32), _fit(d, p["text"], F["s"], tw - 12), font=F["s"], fill=col)
            if p["delivered"] is not None:
                if p["persons"] is None:
                    ppl, pc = "people: no detector", MUTED
                else:
                    ppl = f"people {p['persons']} / truth {p['truth']}"
                    pc = GOOD if p["persons"] == p["truth"] else WARN
                d.text((x + 8, yy + 56), _fit(d, ppl, F["s"], tw - 12), font=F["s"], fill=pc)
            else:
                d.text((x + 8, yy + 56), f"truth: {p['truth']} people", font=F["s"], fill=MUTED)


C_LOST = 3.0


class LiveWindow:
    """Tk window that shows the dashboard frames while the simulation runs."""

    def __init__(self, title="UAV-X GCS Dashboard", scale=None, log=print):
        self.log = log
        import tkinter as tk
        from PIL import ImageTk
        self._ImageTk = ImageTk
        self.root = tk.Tk()
        self.root.title(title)
        sw, sh = self.root.winfo_screenwidth(), self.root.winfo_screenheight()
        self.scale = scale or min(1.0, (sw * 0.9) / W, (sh * 0.85) / H)
        self.size = (int(W * self.scale), int(H * self.scale))
        self.label = tk.Label(self.root, bg="#0d1117")
        self.label.pack()
        self.alive = True
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self._photo = None

    def close(self):
        self.alive = False
        try:
            self.root.destroy()
        except Exception:                           # noqa: BLE001
            pass

    def show(self, im):
        if not self.alive:
            return
        try:
            frame = im.resize(self.size, Image.BILINEAR) if self.scale != 1.0 else im
            self._photo = self._ImageTk.PhotoImage(frame)
            self.label.configure(image=self._photo)
            self.root.update()
        except Exception as e:                      # noqa: BLE001
            if self.alive:
                self.log(f"dashboard window closed ({type(e).__name__}: {e})")
            self.alive = False
