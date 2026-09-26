"""Webots supervisor for the UAV-X Stage 1 PoC (v3: BVLOS scale).

Runs the swarm core (uavx/) once per simulation step and drives everything
you see:
  * the 3D scene: Mavic 2 Pro poses (heading, forward tilt, spinning props,
    level failsafe descent after a critical fault), role-coloured nav LEDs,
    altitude lines (so UAVs stay visible at 2 km scale), PoI flags, radio links
    (thin cyan) and the routing tree (thick yellow), survey cones;
  * one gimbal camera per UAV (Webots Camera devices that follow each drone
    and aim at the site being surveyed), with real YOLOv8n detection;
  * the GCS dashboard window (fleet status, graphs, camera feeds). A feed
    only updates while that UAV has a route to the GCS; otherwise it freezes
    with a LINK LOST banner, which is what a BVLOS operator would really see.

Keys (click the 3D view first):  F = hard fault on the worst-case relay,
C = 15 s radio outage, N = new high-priority PoI, B = battery cell fault.

What you see: the 3D view is the simulator's truth (an observer's view).
The dashboard is the GCS: fleet table, map, charts, feeds and site tiles
are built only from packets that reached the GCS over the mesh.

Environment variables:
  UAVX_SCENARIO=0          disable the scripted fault timeline
  UAVX_DASHBOARD=0         don't open the dashboard window
  UAVX_CINEMATIC=1         let the camera director fly the 3D view
  UAVX_RECORD_MOVIE=x.mp4  record 3D view + dashboard, composite to x.mp4, quit
  UAVX_VIDEO_SPEED=n       time-lapse factor of the recording (default 4)
  UAVX_QUIT=1              quit Webots when the mission ends
  UAVX_SNAPSHOTS=dir       save 3D view + dashboard frames periodically
  UAVX_SNAPSHOT_PERIOD=s   ... every s seconds (default 15)
  UAVX_STOP_AFTER=s        quit after s simulated seconds (layout checks)
"""

import csv
import json
import math
import os
import shutil
import subprocess
import sys

import numpy as np
from PIL import Image

from controller import Supervisor

from uavx import agent as A
from uavx import config as C
from uavx.comms import GCS_ID
from uavx.sim import SwarmSim

from cards import render_cards
from dashboard import Dashboard, LiveWindow
from director import Director
from narrator import Narrator
from perception import DET_CONF, SiteCounter, project
from vision import Detector

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "tools"))
from make_world import (ROADS, VILLAGES, axis_angle_from_yaw_pitch,  # noqa: E402
                        axis_angle_look, ground_h, in_landslide, poi_marker)

GCS_ANTENNA = (C.GCS_POS[0] - 2.0, C.GCS_POS[1] - 0.3, C.GCS_POS[2] + 9.6)
DRONE_Z_OFFSET = 0.45
FRAME_STEPS = 3                       # dashboard / camera / detection cadence

LED_RGB = {
    "SURVEY": (0.1, 1.0, 0.3), "RELAY": (0.2, 0.55, 1.0), "HOLD": (0.8, 0.8, 0.8),
    "RECALLED": (0.9, 0.9, 0.9), "RTH": (1.0, 0.55, 0.05), "CHARGING": (1.0, 0.85, 0.1),
    "READY": (1.0, 1.0, 1.0), "DATA MULE": (0.75, 0.25, 1.0), "RADIO DOWN": (1.0, 0.1, 0.75),
    "FAILED": (0.2, 0.0, 0.0), "EMERGENCY": (1.0, 0.05, 0.05),
}
POI_RGB = {"pending": (0.9, 0.15, 0.15), "high": (1.0, 0.1, 0.8), "surveying": (1.0, 0.6, 0.05),
           "held": (1.0, 0.95, 0.1), "delivered": (0.1, 0.9, 0.25)}


_LOG = open(os.environ["UAVX_LOG"], "w", buffering=1) if os.environ.get("UAVX_LOG") else None


def say(msg):
    print(msg, flush=True)
    if _LOG:
        _LOG.write(msg + "\n")


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def screen_size(eye, a, b, frac, lo, hi):
    """Radius that looks roughly constant on screen: proportional to the
    distance from the camera to the closest point of segment a-b."""
    v = [b[i] - a[i] for i in range(3)]
    w = [eye[i] - a[i] for i in range(3)]
    vv = sum(c * c for c in v) or 1e-9
    t = max(0.0, min(1.0, sum(v[i] * w[i] for i in range(3)) / vv))
    d = math.sqrt(sum((a[i] + v[i] * t - eye[i]) ** 2 for i in range(3)))
    return max(lo, min(hi, frac * d))


# =========================================================== 3D scene
class Scene:
    def __init__(self, sup):
        self.sup = sup
        self.n = {}
        for i in range(1, C.NUM_UAVS + 1):
            node = sup.getFromDef(f"UAV_{i}")
            self.n[i] = {
                "tr": node.getField("translation"),
                "rot": node.getField("rotation"),
                "props": [sup.getFromDef(f"UAV_{i}_P{k}").getField("rotation") for k in range(1, 5)],
                "led": sup.getFromDef(f"UAV_{i}_LED"),
                "beam_tr": sup.getFromDef(f"UAV_{i}_BEAM").getField("translation"),
                "beam_h": sup.getFromDef(f"UAV_{i}_BEAM_GEO").getField("height"),
                "beam_tp": sup.getFromDef(f"UAV_{i}_BEAM_APP").getField("transparency"),
                "mast_tr": sup.getFromDef(f"UAV_{i}_MAST").getField("translation"),
                "mast_h": sup.getFromDef(f"UAV_{i}_MAST_GEO").getField("height"),
            }
        self.yaw = {i: math.pi / 2 for i in self.n}
        self.prev = {}
        self.spin = 0.0
        self.led_role = {}
        self.links = {}
        for a in range(0, C.NUM_UAVS + 1):
            for b in range(a + 1, C.NUM_UAVS + 1):
                self.links[(a, b)] = {
                    "tr": sup.getFromDef(f"LINK_{a}_{b}").getField("translation"),
                    "rot": sup.getFromDef(f"LINK_{a}_{b}").getField("rotation"),
                    "h": sup.getFromDef(f"LINK_{a}_{b}_GEO").getField("height"),
                    "r": sup.getFromDef(f"LINK_{a}_{b}_GEO").getField("radius"),
                    "app": sup.getFromDef(f"LINK_{a}_{b}_APP"),
                    "state": None,
                }
        self.poi_app = {}
        self.poi_col = {}
        self.root_children = sup.getRoot().getField("children")
        self.view = sup.getFromDef("VIEW").getField("position")
        self.mast_r = {i: sup.getFromDef(f"UAV_{i}_MAST_GEO").getField("radius") for i in self.n}

    def eye(self):
        return self.view.getSFVec3f()

    @staticmethod
    def color(app, rgb, glow=0.8):
        app.getField("baseColor").setSFColor(list(rgb))
        app.getField("emissiveColor").setSFColor([c * glow for c in rgb])

    def update_uavs(self, sim, roles, dt):
        self.spin = (self.spin + 50.0 * dt) % (2 * math.pi)
        eye = self.eye()
        for u in sim.uavs:
            n = self.n[u.id]
            p = u.pos
            q = self.prev.get(u.id, p)
            vx, vy = (p[0] - q[0]) / dt, (p[1] - q[1]) / dt
            self.prev[u.id] = list(p)
            speed = math.hypot(vx, vy)
            if speed > 0.4 and u.status != A.FAILED:
                err = wrap(math.atan2(vy, vx) - self.yaw[u.id])
                self.yaw[u.id] = wrap(self.yaw[u.id] + max(-2.5 * dt, min(2.5 * dt, err)))
            # Forward tilt with speed; a failsafe descent stays level.
            pitch = min(0.2, speed / C.CRUISE_SPEED * 0.18)
            n["rot"].setSFRotation(list(axis_angle_from_yaw_pitch(self.yaw[u.id], pitch)))
            n["tr"].setSFVec3f([p[0], p[1], p[2] + DRONE_Z_OFFSET])
            if u.airborne:
                for k, f in enumerate(n["props"]):
                    f.setSFRotation([0, 0, 1, self.spin if k % 2 else -self.spin])
            g = ground_h(p[0], p[1])
            h = max(0.5, p[2] - g)
            if u.airborne:
                n["mast_tr"].setSFVec3f([p[0], p[1], g + h / 2])
                n["mast_h"].setSFFloat(h)
                self.mast_r[u.id].setSFFloat(
                    screen_size(eye, (p[0], p[1], g), (p[0], p[1], p[2] - 3.0), 0.0014, 0.04, 2.5))
            else:
                n["mast_tr"].setSFVec3f([0, 0, -200])
            role = roles[u.id].replace("+FWD", "")
            if role == "EMERGENCY":                 # strobing red while descending
                role = "EMERGENCY" if int(sim.t * 3) % 2 else "FAILED"
            if self.led_role.get(u.id) != role:
                self.led_role[u.id] = role
                self.color(n["led"], LED_RGB.get(role, (0.7, 0.7, 0.7)), 1.0)
            scanning = (u.status == A.ACTIVE and u.task[0] == A.SURVEY
                        and u.survey_timer > 0 and u.task[2] not in u.surveyed)
            if scanning:
                alt = max(1.0, p[2] - ground_h(p[0], p[1]))
                n["beam_h"].setSFFloat(alt)
                n["beam_tr"].setSFVec3f([0, 0, -alt / 2 - DRONE_Z_OFFSET])
                n["beam_tp"].setSFFloat(0.8)
            else:
                n["beam_tp"].setSFFloat(1.0)

    def update_links(self, sim):
        pos = sim.positions()
        pos[GCS_ID] = GCS_ANTENNA
        tree, edges = sim.net.tree_edges(), sim.net.all_edges()
        eye = self.eye()
        for key, L in self.links.items():
            state = "tree" if key in tree else ("link" if key in edges else None)
            if state is None:
                if L["state"] is not None:
                    L["tr"].setSFVec3f([0, 0, -100])
                    L["state"] = None
                continue
            a, b = pos[key[0]], pos[key[1]]
            v = [b[i] - a[i] for i in range(3)]
            n = math.sqrt(sum(c * c for c in v)) or 1e-6
            axis = (-v[1], v[0], 0.0)
            an = math.hypot(axis[0], axis[1])
            rot = [axis[0] / an, axis[1] / an, 0.0, math.acos(max(-1, min(1, v[2] / n)))] \
                if an > 1e-9 else [1, 0, 0, 0]
            L["tr"].setSFVec3f([(a[i] + b[i]) / 2 for i in range(3)])
            L["rot"].setSFRotation(rot)
            L["h"].setSFFloat(n)
            # Thin up close, thick far away: constant-ish width on screen, so
            # the network reads in the 2 km overview without hiding drones
            # in close-ups.
            L["r"].setSFFloat(screen_size(eye, a, b, 0.0012 if state == "tree" else 0.0005,
                                          0.03, 2.2))
            if state != L["state"]:
                L["state"] = state
                if state == "tree":
                    self.color(L["app"], (1.0, 0.82, 0.1))
                    L["app"].getField("transparency").setSFFloat(0.15)
                else:
                    self.color(L["app"], (0.3, 0.9, 1.0))
                    L["app"].getField("transparency").setSFFloat(0.6)

    def update_pois(self, pois):
        for p in pois:
            pid = p["id"]
            if pid not in self.poi_app:
                if self.sup.getFromDef(f"POI_{pid}") is None:
                    self.root_children.importMFNodeFromString(-1, poi_marker(pid, p["x"], p["y"]))
                self.poi_app[pid] = (self.sup.getFromDef(f"POI_{pid}_APP"),
                                     self.sup.getFromDef(f"POI_{pid}_RING"))
            if self.poi_col.get(pid) != p["state"]:
                self.poi_col[pid] = p["state"]
                flag, ring = self.poi_app[pid]
                self.color(flag, POI_RGB[p["state"]], 0.7)
                self.color(ring, POI_RGB[p["state"]], 0.5)


# ============================================================ cameras
class Cameras:
    """One gimbal camera per UAV. The Camera devices live on the GCS robot
    (a UAV here is a kinematic Solid, not a Robot) and are moved to each
    drone's gimbal every frame, so they render exactly what the drone sees."""

    def __init__(self, sup, period_ms):
        self.dev, self.f = {}, {}
        self.aim = {}
        for i in range(1, C.NUM_UAVS + 1):
            cam = sup.getDevice(f"cam_{i}")
            cam.enable(period_ms)
            node = sup.getFromDef(f"CAM_{i}")
            self.dev[i] = cam
            self.f[i] = (node.getField("translation"), node.getField("rotation"),
                         node.getField("fieldOfView"))
            self.aim[i] = [math.pi / 2, 0.1, 1.0]
        self.pose = {}                   # uid -> (cam_pos, yaw, pitch, fov) last applied
        self.settled = {}                # uid -> gimbal on the survey target (aim converged)

    def update(self, sim, yaw, dt):
        gx, gy, _ = C.GCS_POS
        a = 1.0 - math.exp(-dt / 0.45)
        for u in sim.uavs:
            p = u.pos
            cam_pos = (p[0], p[1], p[2] + DRONE_Z_OFFSET - 0.9)
            want = None
            kind, xy, poi = u.task
            if u.status == A.ACTIVE and kind == A.SURVEY and xy is not None and poi not in u.surveyed:
                hx, hy = u.survey_point(xy)
                if math.hypot(p[0] - hx, p[1] - hy) < 30:
                    tgt = (xy[0], xy[1], ground_h(*xy) + 1.0)
                    dx, dy, dz = (tgt[i] - cam_pos[i] for i in range(3))
                    rng = math.sqrt(dx * dx + dy * dy + dz * dz)
                    want = [math.atan2(dy, dx), math.atan2(-dz, math.hypot(dx, dy)),
                            max(0.05, min(0.9, 2 * math.atan(6.0 / rng)))]
            if want is None:
                if u.status == A.EMERGENCY:
                    want = [yaw[u.id], 1.1, 1.05]
                elif u.airborne:
                    want = [yaw[u.id], 0.55, 1.05]
                else:
                    want = [yaw[u.id], 0.12, 1.05]
            cur = self.aim[u.id]
            cur[0] = wrap(cur[0] + wrap(want[0] - cur[0]) * a)
            cur[1] += (want[1] - cur[1]) * a
            cur[2] += (want[2] - cur[2]) * a
            self.settled[u.id] = bool(
                kind == A.SURVEY and u.survey_timer > 0 and want[2] < 1.0
                and abs(wrap(want[0] - cur[0])) < 0.03 and abs(want[1] - cur[1]) < 0.03
                and abs(want[2] - cur[2]) < 0.05 * want[2])
            self.pose[u.id] = (cam_pos, cur[0], cur[1], cur[2])
            tr, rot, fov = self.f[u.id]
            tr.setSFVec3f([cam_pos[0] - gx, cam_pos[1] - gy, cam_pos[2]])
            rot.setSFRotation(list(axis_angle_from_yaw_pitch(cur[0], cur[1])))
            fov.setSFFloat(cur[2])

    def frame(self, uid):
        cam = self.dev[uid]
        buf = cam.getImage()
        if not buf:
            return None
        arr = np.frombuffer(buf, np.uint8).reshape(cam.getHeight(), cam.getWidth(), 4)
        return arr[:, :, [2, 1, 0]].copy()


# ========================================================= state/feeds
class Feeds:
    """Onboard perception and the GCS's view of the onboard cameras.

    Perception runs ONBOARD: a surveying UAV runs YOLOv8n on its own gimbal
    camera whether or not it has a link, and the person count travels to the
    GCS with the site's imagery (u.detections -> data chunks).

    The GCS feed of a UAV only changes when one of that UAV's thumbnail
    packets has crossed the mesh (sim.thumb_rx), so a feed freezes, and says
    how old it is, whenever the network doesn't deliver.
    """

    LIVE_AGE = C.THUMB_PERIOD + 2.5   # s: older than this and the feed reads NO NEW FRAME

    def __init__(self, detector):
        self.det = detector
        self.data = {}                  # uid -> {img, t, dets, age, live}
        self.onboard = {}               # uid -> (dets, t) latest onboard detections
        self.last_det = {}
        self.counters = {}              # (uid, pid) -> perception.SiteCounter
        self.frames_used = {}           # pid -> settled frames that went into its count

    def update(self, sim, cams):
        now = sim.t
        if self.det.ok:
            self._perceive(sim, cams, now)
        for u in sim.uavs:
            f = self.data.setdefault(u.id, {"img": None, "t": None, "dets": []})
            rx = sim.thumb_rx.get(u.id)
            if rx is not None and (f["t"] is None or rx > f["t"]):
                rgb = cams.frame(u.id)
                if rgb is not None:
                    f["img"] = Image.fromarray(rgb)
                    f["t"] = rx
                    od = self.onboard.get(u.id)
                    f["dets"] = od[0] if od and now - od[1] < 1.5 else []
            f["age"] = None if f["t"] is None else now - f["t"]
            f["live"] = f["age"] is not None and f["age"] < self.LIVE_AGE

    def _perceive(self, sim, cams, now):
        """Onboard counting (perception.py): only frames taken while the
        gimbal is settled on the site count; person boxes are projected to
        the ground, associated with the site and tracked; the site count is
        the number of confirmed static tracks."""
        surveying = [u for u in sim.uavs if u.status == A.ACTIVE and u.task[0] == A.SURVEY
                     and u.survey_timer > 0 and u.task[2] not in u.surveyed
                     and cams.settled.get(u.id)]
        pool = surveying or [u for u in sim.uavs if u.airborne and u.status != A.EMERGENCY
                             and now - self.last_det.get(u.id, -99.0) > 2.0]
        if not pool:
            return
        u = min(pool, key=lambda v: self.last_det.get(v.id, -99.0))
        self.last_det[u.id] = now
        rgb = cams.frame(u.id)
        if rgb is None:
            return
        dets = self.det.detect(rgb, conf=DET_CONF)
        self.onboard[u.id] = (dets, now)
        if u in surveying:
            pid = u.task[2]
            cam_pos, yaw, pitch, fov = cams.pose[u.id]
            xy = u.task[1]
            zg = ground_h(*xy)
            pts = [project(d[2:6], cam_pos, yaw, pitch, fov, zg) for d in dets if d[0] == "person"]
            sc = self.counters.setdefault((u.id, pid), SiteCounter(xy))
            sc.add_frame(pts)
            u.detections[pid] = sc.count()
            self.frames_used[pid] = sc.frames


def poi_states(sim):
    """3D flag colours: simulator truth (observer view)."""
    held = set().union(*(u.data for u in sim.uavs))
    scanning = {u.task[2] for u in sim.uavs if u.task[0] == A.SURVEY and u.survey_timer > 0
                and u.task[2] not in u.surveyed and u.status == A.ACTIVE}
    out = []
    for pid, p in sim.gcs.pois.items():
        if p["delivered"] is not None:
            s = "delivered"
        elif pid in held:
            s = "held"
        elif pid in scanning:
            s = "surveying"
        else:
            s = "high" if p["prio"] > 1 else "pending"
        out.append({"id": pid, "x": p["x"], "y": p["y"], "state": s})
    return out


def gcs_sites(sim):
    """Site tiles for the dashboard: GCS knowledge only."""
    g = sim.gcs
    tasked = {t[2]: uid for uid, t in g.tasks.items() if t and t[0] == A.SURVEY}
    out = []
    for pid, p in g.pois.items():
        if p["delivered"] is not None:
            s, txt = "delivered", f"imagery at GCS · t={p['delivered']:.0f}s"
        elif p["rx"] > 0:
            s, txt = "held", f"receiving {100 * p['rx'] / (C.SURVEY_DATA_MB * 8):.0f}%"
        elif p["held_known"] is not None:
            s, txt = "held", f"data on UAV{p['held_by']}"
        elif pid in tasked:
            s, txt = "surveying", f"UAV{tasked[pid]} tasked"
        else:
            s, txt = ("high" if p["prio"] > 1 else "pending"), "pending"
        out.append({"id": pid, "x": p["x"], "y": p["y"], "prio": p["prio"], "state": s,
                    "text": txt, "persons": p["persons"], "truth": C.SURVIVORS_GT.get(pid),
                    "delivered": p["delivered"]})
    return out


def gcs_role(g, uid):
    k = g.known[uid]
    if k["lost"]:
        return "LINK LOST"
    st = k["status"]
    if st in (A.EMERGENCY, A.FAILED, A.RTH, A.SERVICE, A.READY):
        return st
    kind, _, poi = k["task"]
    if uid in g.outgoing:
        return "HANDOVER"
    if kind == A.HOME:
        return "RECALLED"
    if kind == A.RELAY and str(poi).startswith("BKP"):
        return "BACKUP"
    return kind


def dashboard_state(sim, feeds, caption, detector_ok):
    """Everything the dashboard shows. Fleet, sites, map and charts come from
    GCS knowledge (sim.gcs); PDR is what the GCS measures on the packets it
    receives. The simulator's truth is only used by the 3D view."""
    now = sim.t
    g = sim.gcs
    uavs = []
    for uid in range(1, C.NUM_UAVS + 1):
        k = g.known.get(uid)
        if k is None:
            uavs.append({"id": uid, "known": False})
            continue
        kind, _, poi = k["task"]
        f = feeds.data.get(uid, {})
        uavs.append({
            "id": uid, "known": True, "x": k["pos"][0], "y": k["pos"][1],
            "alt": max(0.0, k["pos"][2] - ground_h(k["pos"][0], k["pos"][1])),
            "role": gcs_role(g, uid), "status": k["status"], "lost": k["lost"],
            "task": poi if kind in (A.SURVEY, A.RELAY) and poi else "",
            "battery": k["battery"], "age": now - k["last_seen"], "hops": k["hops"],
            "parent": k["parent"],
            "det": sum(1 for d in f.get("dets", []) if d[0] == "person") if f.get("live") else 0,
        })
    air = [u for u in uavs if u["known"] and u["status"] in (A.ACTIVE, A.RTH, A.EMERGENCY)]
    linked = [u for u in air if not u["lost"]]
    missing = [u for u in air if u["lost"]]
    pos = {GCS_ID: C.GCS_POS}
    for u in uavs:
        if u["known"]:
            pos[u["id"]] = (u["x"], u["y"])
    tree = {tuple(sorted((u["id"], u["parent"]))) for u in linked if u["parent"] is not None}
    return {
        "t": now, "complete": g.complete_time is not None,
        "complete_t": g.complete_time,
        "sites": gcs_sites(sim), "uavs": uavs, "airborne": len(air), "linked": len(linked),
        "roster": C.NUM_UAVS, "missing": [(u["id"], u["age"]) for u in missing],
        "delivered": len(g.delivered_ids()), "n_sites": len(g.pois),
        # Cohort PDR: heartbeats whose 2 s deadline has passed, delivered in time.
        "pdr10": sim.channel.cohort_pdr("hb", now, C.PDR_WINDOW),
        "pdr_all": sim.channel.cohort_pdr("hb", now),
        # Freshness of ACTIVE participants only; lost aircraft are listed separately.
        "max_age": max((u["age"] for u in linked), default=None),
        "fleet_confirmed_t": g.fleet_confirmed_time,
        "faults": sim.faults, "events": sim.events,
        "positions": pos, "tree": tree,
        "relay_slots": [t[1] for t in g.tasks.values() if t and t[0] == A.RELAY],
        "feeds": feeds.data, "caption": caption, "detector": detector_ok,
    }


def map_features():
    areas = []
    for _, vx, vy, vr, _, _ in VILLAGES:
        r = vr * 1.25
        areas.append(([(vx + r * math.cos(a / 24 * 2 * math.pi), vy + r * math.sin(a / 24 * 2 * math.pi))
                       for a in range(24)], (58, 52, 40)))
    left, right = [], []
    for k in range(47):
        y = 600 + k * 10
        xs = [x for x in range(400, 960, 5) if in_landslide(x, y)]
        if xs:
            left.append((min(xs), y))
            right.append((max(xs), y))
    areas.append((left + right[::-1], (92, 64, 40)))
    return {"areas": areas, "roads": [(x1, y1, x2, y2, 3) for x1, y1, x2, y2 in ROADS]}


# ============================================================== main
def perception_table(sim, feeds):
    return {pid: sim.gcs.pois[pid]["persons"] for pid in sim.gcs.pois}


def write_results(sim, tag, feeds):
    out = os.path.join(ROOT, "results")
    os.makedirs(out, exist_ok=True)
    res = sim.results()
    res["perception"]["detector"] = ("YOLOv8n (COCO), onboard, conf >= %.2f; counting rule: "
                                     "static ground tracks (perception.py)" % DET_CONF
                                     if feeds.det.ok else f"disabled: {feeds.det.error}")
    res["perception"]["settled_frames_per_site"] = feeds.frames_used
    with open(os.path.join(out, f"metrics_{tag}.json"), "w") as f:
        json.dump(res, f, indent=2, default=list)
    sim.write_logs(out, tag)
    say(f"results and raw logs written to {out}")
    return res


def hud(sup, sim, recording):
    """Webots text labels, only for interactive use: in the recording the
    dashboard layer draws a readable header strip over the 3D view instead."""
    if recording:
        return
    sup.setLabel(0, f"UAV-X  ·  3D VIEW = SIMULATOR TRUTH  ·  t = {sim.t:5.1f} s",
                 0.012, 0.012, 0.075, 0xFFFFFF, 0.0, "Arial")
    sup.setLabel(1, "yellow = active route   cyan = radio link   LEDs: green survey · blue relay"
                    " · orange RTH · purple data mule · red emergency",
                 0.012, 0.05, 0.052, 0xFFFFFF, 0.0, "Arial")


def main():
    sup = Supervisor()
    step_ms = int(sup.getBasicTimeStep())
    dt = step_ms / 1000.0
    scripted = os.environ.get("UAVX_SCENARIO", "1") != "0"
    movie = os.environ.get("UAVX_RECORD_MOVIE")
    quit_at_end = bool(movie) or os.environ.get("UAVX_QUIT") == "1"
    snap_dir = os.environ.get("UAVX_SNAPSHOTS")
    snap_every = float(os.environ.get("UAVX_SNAPSHOT_PERIOD", "15"))
    stop_after = float(os.environ.get("UAVX_STOP_AFTER", "0"))
    cinematic = bool(movie) or os.environ.get("UAVX_CINEMATIC") == "1"
    want_window = os.environ.get("UAVX_DASHBOARD", "1") != "0" and not movie
    speed = max(1, int(os.environ.get("UAVX_VIDEO_SPEED", "4")))

    sim = SwarmSim(scenario=scripted, log=say, ground=ground_h)
    scene = Scene(sup)
    cams = Cameras(sup, step_ms * FRAME_STEPS)
    feeds = Feeds(Detector(ROOT, log=say))
    bench_dir = os.environ.get("UAVX_PERCEPTION_BENCH")
    if bench_dir:
        import bench_perception
        bench_perception.run(sup, step_ms, FRAME_STEPS, cams, feeds.det, bench_dir, ROOT, C,
                             ground_h, axis_angle_from_yaw_pitch, DRONE_Z_OFFSET, say)
        sup.simulationQuit(0)
        return
    narrator = Narrator()
    dash = Dashboard(C.GEOFENCE, C.GCS_POS, C.COMM_RANGE, map_features())
    director = Director(sup, axis_angle_look, ground_h) if cinematic else None
    window = None
    if want_window:
        try:
            window = LiveWindow(log=say)
            say(f"dashboard window open ({window.size[0]}x{window.size[1]})")
        except Exception as e:                      # noqa: BLE001
            say(f"dashboard window unavailable ({e}); continuing without it")
    kb = sup.getKeyboard()
    kb.enable(step_ms)

    frames_dir = movie3d = None
    if movie:
        base = os.path.splitext(movie)[0]
        movie3d = base + "_3d.mp4"
        frames_dir = base + "_frames"
        shutil.rmtree(frames_dir, ignore_errors=True)
        os.makedirs(frames_dir, exist_ok=True)
        sup.movieStartRecording(movie3d, 1280, 720, 0, 95, speed, False)
        say(f"recording 3D view to {movie3d}, dashboard frames to {frames_dir}")

    say(f"UAV-X Stage 1 PoC v3: {C.NUM_UAVS} UAVs, {len(C.POIS)} PoIs, "
        f"radio range {C.COMM_RANGE:.0f} m, scripted faults {'ON' if scripted else 'OFF'}")

    n = 0
    frame_no = 0
    end_hold = None
    next_snap = 0.0
    written = None
    while sup.step(step_ms) != -1:
        key = kb.getKey()
        while key != -1:
            k = key & 0xFF
            if k in (ord("F"), ord("f")):
                sim.inject_failure(hard=True)
            elif k in (ord("C"), ord("c")):
                sim.inject_radio_outage(15.0)
            elif k in (ord("N"), ord("n")):
                sim.inject_new_poi()
            elif k in (ord("B"), ord("b")):
                sim.inject_battery_sag(45.0)
            key = kb.getKey()

        sim.step(dt)
        roles = sim.roles()
        scene.update_uavs(sim, roles, dt)
        cams.update(sim, scene.yaw, dt)
        if director:
            director.update(sim, dt, scene.yaw)
        if n % FRAME_STEPS == 0:
            scene.update_links(sim)
            feeds.update(sim, cams)
            scene.update_pois(poi_states(sim))
            hud(sup, sim, bool(movie))
            st = dashboard_state(sim, feeds, narrator.caption(sim), feeds.det.ok)
            if frames_dir or (window and window.alive) or (snap_dir and sim.t >= next_snap):
                im = dash.render(st, with_map=not frames_dir)
                if frames_dir:
                    im.save(os.path.join(frames_dir, f"{frame_no:06d}.jpg"), quality=90)
                    frame_no += 1
                if window and window.alive:
                    window.show(im)
                if snap_dir and sim.t >= next_snap:
                    next_snap += snap_every
                    os.makedirs(snap_dir, exist_ok=True)
                    im.save(os.path.join(snap_dir, f"dash_t{int(sim.t):03d}.jpg"), quality=90)
                    sup.exportImage(os.path.join(snap_dir, f"view_t{int(sim.t):03d}.jpg"), 90)
            else:
                dash.sample(st)
        n += 1

        if stop_after and sim.t >= stop_after and end_hold is None:
            if window:
                say(f"dashboard window alive at stop: {window.alive}")
            if not movie:
                sup.simulationQuit(0)
                break
            written, end_hold = False, sim.t      # short test clip: compose what we have
        if sim.done and written is None:
            written = write_results(sim, "webots" + ("" if scripted else "_nofault"), feeds)
            end_hold = sim.t
        if end_hold is not None and quit_at_end and sup.getTime() >= end_hold + 4.0:
            if movie:
                sup.movieStopRecording()
                while not sup.movieIsReady():
                    if sup.step(step_ms) == -1:
                        break
                say("3D movie saved" if not sup.movieFailed() else "3D movie FAILED")
                if written:
                    v = subprocess.run([sys.executable, os.path.join(ROOT, "tools", "recompute_metrics.py"),
                                        os.path.join(ROOT, "results"), "webots"],
                                       capture_output=True, text=True)
                    say("raw-log validation: " + (v.stdout.strip().splitlines() or ["?"])[-1])
                cards = render_cards(written or None, os.path.dirname(os.path.abspath(movie)))
                compose = os.path.join(ROOT, "tools", "compose_video.py")
                rate = f"{1000 * speed}/{step_ms * FRAME_STEPS}"
                r = subprocess.run([sys.executable, compose, movie3d, frames_dir, movie, rate]
                                   + cards, capture_output=True, text=True)
                say(r.stdout.strip() or r.stderr.strip()[-800:])
            sup.simulationQuit(0)
            break


try:
    main()
except Exception:
    import traceback
    say(traceback.format_exc())
    raise
