"""Cinematic camera for recordings: moves the Webots Viewpoint through a
shot list that follows the scripted scenario (launch, the relay that is
about to fail, a STABLE overview of the whole fleet while it recovers, the
new high-priority site, the radio-outage UAV, the relay handover, landing).
Recovery and handover use locked wide framings so the viewer can see the
mesh re-form instead of chasing one drone.

Only active when recording or with UAVX_CINEMATIC=1, so it never fights a
user steering the 3D view by hand.
"""

import math

from uavx import agent as A
from uavx import config as C
from uavx.agent import pad_position

TAU = 2.0                     # s, smoothing time constant for camera moves


def _event_time(kind, default):
    for t, k, _ in C.SCENARIO:
        if k == kind:
            return t
    return default


T_FAIL = _event_time("fail", 150.0)
T_POI = _event_time("new_poi", 220.0)
T_RADIO = _event_time("radio", 300.0)
T_SAG = _event_time("battery_sag", 360.0)
H1 = next((a for t, k, a in C.SCENARIO if k == "new_poi" and a), None)


class Director:
    def __init__(self, sup, look_fn, ground_fn):
        self.view = sup.getFromDef("VIEW")
        self.f_pos = self.view.getField("position")
        self.f_ori = self.view.getField("orientation")
        self.look = look_fn
        self.ground = ground_fn
        self.overview_pos = tuple(self.f_pos.getSFVec3f())
        pos, tgt, _ = self._pads()
        self.pos, self.target = list(pos), list(tgt)
        self.overview_target = (60.0, 60.0, 0.0)
        self.locked = {}
        self.caption = ""
        self.heading = {}

    # --------------------------------------------------------------- shots
    def _pads(self):
        xs = [pad_position(i) for i in range(1, C.NUM_UAVS + 1)]
        cx = sum(p[0] for p in xs) / len(xs)
        cy = sum(p[1] for p in xs) / len(xs)
        return (cx - 22, cy - 40, 18), (cx + 2, cy + 4, 8), "LAUNCH - forward GCS, 2.4 km from the furthest site"

    def _chase(self, sim, uid, dist=24.0, up=8.0):
        u = sim.uav(uid)
        yaw = self.heading.get(uid, 0.0)
        hx, hy = math.cos(yaw), math.sin(yaw)
        p = u.pos
        pos = (p[0] - hx * dist - hy * 9, p[1] - hy * dist + hx * 9, p[2] + up)
        tgt = (p[0] + hx * 10, p[1] + hy * 10, p[2] - 6)
        return pos, tgt

    def _site(self, x, y, back=70.0, up=45.0):
        gx, gy, _ = C.GCS_POS
        d = math.hypot(x - gx, y - gy) or 1
        ux, uy = (x - gx) / d, (y - gy) / d
        z = self.ground(x, y)
        return (x - ux * back - uy * 20, y - uy * back + ux * 20, z + up), (x, y, z + 3)

    def _busiest(self, sim):
        live = [u for u in sim.uavs if u.status == A.ACTIVE]
        if not live:
            return None
        return max(live, key=lambda u: (sim._descendants(u.id), sim.net.hops.get(u.id, 0))).id

    def _lock(self, key, fn):
        if key not in self.locked:
            self.locked[key] = fn()
        return self.locked[key]

    def _surveyor(self, sim):
        best = None
        for u in sim.uavs:
            if u.status == A.ACTIVE and u.task[0] == A.SURVEY and u.task[2] not in u.surveyed:
                hx, hy = u.survey_point(u.task[1])
                d = math.hypot(u.pos[0] - hx, u.pos[1] - hy)
                if best is None or d < best[0]:
                    best = (d, u.id)
        return best[1] if best else None

    def _frame(self, pts, margin=1.15):
        """Wide, fixed framing of a set of ground points (and the GCS)."""
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
        e = max(250.0, max(xs) - min(xs), max(ys) - min(ys)) * margin
        z = self.ground(cx, cy)
        # From the south-west (behind the GCS), looking across the mesh.
        return (cx - 0.5 * e, cy - 0.72 * e, z + 0.62 * e), (cx + 0.05 * e, cy + 0.08 * e, z)

    def _fleet_frame(self, sim):
        pts = [C.GCS_POS] + [u.pos for u in sim.uavs if u.status in (A.ACTIVE, A.EMERGENCY)]
        return self._frame(pts)

    def _stable_until(self, sim):
        f = next((f for f in sim.metrics.faults if f.kind == "uav_failure"), None)
        if f and f.ts["stable_confirmed"] is not None:
            return f.t + f.ts["stable_confirmed"] + 8.0
        if f and f.outcome is not None:
            return f.t + 20.0
        return T_FAIL + 60.0

    def _handover_active(self, sim):
        """From the energy fault until 8 s after the GCS releases the relay."""
        for h in sim.gcs.handovers:
            if h["t_released"] is None or sim.t < h["t_released"] + 8.0:
                return h
        return None

    def shot(self, sim):
        t = sim.t
        done_t = sim.gcs.complete_time
        if t < 30.0:
            return self._pads()
        if done_t is not None and t > done_t + 10.0:
            pos, tgt, _ = self._pads()
            return pos, tgt, "RECOVERY - fleet returning to base"
        if T_FAIL - 30.0 <= t < T_FAIL:
            uid = self._lock("relay", lambda: self._busiest(sim))
            if uid:
                pos, tgt = self._chase(sim, uid)
                return pos, tgt, f"CHASE CAM - UAV{uid}, backbone relay"
        if T_FAIL <= t < T_FAIL + 7.0:
            uid = self.locked.get("relay")
            if uid:
                u = sim.uav(uid)
                pos, tgt = self._site(u.pos[0], u.pos[1], back=55, up=u.pos[2] * 0.6 + 12)
                return pos, (u.pos[0], u.pos[1], u.pos[2] * 0.7), f"UAV{uid} FAULT"
        if T_FAIL + 7.0 <= t < min(T_FAIL + 60.0, self._stable_until(sim)):
            pos, tgt = self._lock("fleet", lambda: self._fleet_frame(sim))
            return pos, tgt, "RECOVERY OVERVIEW"
        h = self._handover_active(sim) if t >= T_SAG else None
        if h is not None and h.get("t_dispatch") is not None:
            def handover_frame():
                u = sim.uav(h["uav"])
                pts = [C.GCS_POS, u.pos, h["xy"]]
                if h.get("replacement"):
                    pts.append(sim.uav(h["replacement"]).pos)
                return self._frame([(p[0], p[1]) for p in pts], margin=1.5)
            pos, tgt = self._lock(("handover", h["replacement"]), handover_frame)
            return pos, tgt, "HANDOVER OVERVIEW"
        if H1 and T_POI - 2.0 <= t < T_POI + 22.0:
            pos, tgt = self._site(H1[1], H1[2])
            return pos, tgt, f"NEW HIGH-PRIORITY SITE {H1[0]} - survivors reported"
        if T_RADIO <= t < T_RADIO + 30.0:
            victim = next((f["uav"] for f in sim.faults if f["type"] == "comm_outage"), None)
            if victim and sim.uav(victim).airborne:
                pos, tgt = self._chase(sim, victim, dist=26, up=9)
                return pos, tgt, f"UAV{victim} RADIO OUTAGE - flying its last task autonomously"
        # Otherwise alternate the wide shot with a chase of whoever is about
        # to survey a site (camera cone, survivors, YOLO feed in the dashboard).
        if int(t // 45) % 2 == 1:
            uid = self._surveyor(sim)
            if uid:
                u = sim.uav(uid)
                return (*self._chase(sim, uid), f"SURVEY - UAV{uid} approaching {u.task[2]}")
        return self.overview_pos, self.overview_target, ""

    # -------------------------------------------------------------- update
    def update(self, sim, dt, headings):
        self.heading = headings
        pos, tgt, cap = self.shot(sim)
        if cap != self.caption:
            self.shot_start = sim.t
        self.caption = cap
        # Glide between shots, then lock on tightly: a chase cam that lags
        # 2 s behind a 15 m/s drone would lose it off the edge of the frame.
        settled = sim.t - getattr(self, "shot_start", 0.0) > 4.0
        tau = 0.25 if (settled and cap.startswith(("CHASE", "SURVEY", "UAV"))) else TAU
        if cap.endswith("OVERVIEW"):
            tau = 1.5                         # glide in, then hold still
        a = 1.0 - math.exp(-dt / tau)
        for i in range(3):
            self.pos[i] += (pos[i] - self.pos[i]) * a
            self.target[i] += (tgt[i] - self.target[i]) * a
        self.f_pos.setSFVec3f(self.pos)
        self.f_ori.setSFRotation(list(self.look(self.pos, self.target)))


__all__ = ["Director"]
