"""Evaluator: measures a run from simulator truth. Never read by the GCS or
the UAVs (tests/test_information_flow.py checks that gcs.py and agent.py do
not import it). Definitions, units and formulas: docs/METRICS.md.

Populations used by every fault event (frozen at the fault, changes logged):
  faulted     the aircraft the fault was injected into
  orphaned    other mission UAVs (ACTIVE, radio up) that had a route just
              before the fault and lost it in the first observation after it
  unaffected  other mission UAVs that had a route before and kept it
  required    orphaned, plus the faulted aircraft if the fault is temporary
              (radio outage): it must be routed again for recovery.
              An aircraft loss (critical fault) is irrecoverable: it is
              reported separately and is never part of `required`.
Timestamps (seconds after the fault, None = did not happen):
  detected            GCS flags the faulted UAV (EMERGENCY heartbeat or timeout)
  first_route         first required UAV has a route again
  all_routes          every required UAV has a route (may be brief)
  stable_start        start of the first STABLE_WINDOW-long interval in which
                      every required UAV kept a route (any gap resets the hold)
  stable_confirmed    stable_start + STABLE_WINDOW (when it can be known)
  first_app_delivery  GCS receives a heartbeat from a required UAV (after its
                      route loss); all_app_delivery: from every required UAV
"""

import collections
import math

from . import agent as A
from . import config as C
from .comms import GCS_ID

MISSION = (A.ACTIVE, A.RTH)


def _r(v, nd=2):
    return None if v is None else round(v, nd)


def seg_min_dist(p0, p1, q0, q1):
    """Minimum distance between two points moving linearly p0->p1, q0->q1
    over the same interval (closest approach, not just the endpoints)."""
    r0 = [p0[i] - q0[i] for i in range(3)]
    dr = [(p1[i] - p0[i]) - (q1[i] - q0[i]) for i in range(3)]
    a = sum(c * c for c in dr)
    s = 0.0 if a < 1e-12 else max(0.0, min(1.0, -sum(r0[i] * dr[i] for i in range(3)) / a))
    return math.sqrt(sum((r0[i] + s * dr[i]) ** 2 for i in range(3)))


class FaultTracker:
    def __init__(self, sim, kind, uid, temporary, extra):
        self.kind, self.uid, self.temporary = kind, uid, temporary
        self.t = sim.t
        self.before = {u.id for u in sim.uavs if u.id != uid and u.status in MISSION
                       and u.radio_ok and sim.net.connected(u.id)}
        self.extra = dict(extra)
        self.orphaned = None
        self.unaffected = None
        self.required = None
        self.members_log = []            # (t, uav, change)
        self.ts = dict(detected=None, first_route=None, all_routes=None, stable_start=None,
                       stable_confirmed=None, first_app_delivery=None, all_app_delivery=None)
        self.flaps = 0
        self.disc_uav_s = 0.0            # required-population UAV-seconds without a route
        self.unaffected_ok_s = 0.0
        self.window_s = 0.0
        self.outcome = None
        self._hold = None
        self._whole_before = False
        self._heard = set()
        self._overlaps = []

    # -------------------------------------------------------------- update
    def observe(self, sim, dt):
        now = sim.t
        if self.outcome is not None:
            return
        rel = now - self.t
        if self.orphaned is None:        # first observation after the fault
            self.orphaned = {u for u in self.before if not sim.net.connected(u)}
            self.unaffected = self.before - self.orphaned
            self.required = set(self.orphaned)
            if self.temporary:
                self.required.add(self.uid)
            self._lost_at = {u: now for u in self.required}
        if self.ts["detected"] is None:
            for lt, luid in sorted(sim.gcs.lost_events + sim.gcs.emergency_events):
                if luid == self.uid and lt >= self.t:
                    self.ts["detected"] = lt - self.t
                    break
        for u in sorted(self.required):   # membership changes (explicit rules)
            s = sim.uav(u).status
            if s in (A.SERVICE, A.READY):
                self.required.discard(u)
                self.members_log.append((round(now, 2), u, "left: landed (no longer a mission UAV)"))
            elif s in (A.EMERGENCY, A.FAILED) and not (self.temporary and u == self.uid):
                self.required.discard(u)
                self.members_log.append((round(now, 2), u, "left: later critical fault"))
        if not self.required:
            if not self.orphaned and not self.temporary:
                self.outcome = f"unaffected (0 of {len(self.before)} other UAVs lost their route)"
            elif self.ts["stable_start"] is None:
                self.outcome = "undetermined: every required UAV left the mission before recovery"
            return
        self.window_s += dt
        cut = [u for u in self.required if not sim.net.connected(u)]
        self.disc_uav_s += len(cut) * dt
        ua = [u for u in (self.unaffected or ()) if sim.uav(u).status in MISSION]
        if ua and all(sim.net.connected(u) for u in ua):
            self.unaffected_ok_s += dt
        n_req = len(self.required)
        if cut and len(cut) < n_req and self.ts["first_route"] is None:
            self.ts["first_route"] = rel
        whole = not cut
        if whole:
            if self.ts["first_route"] is None:
                self.ts["first_route"] = rel
            if self.ts["all_routes"] is None:
                self.ts["all_routes"] = rel
            if self._hold is None:
                self._hold = now
            if now - self._hold >= C.STABLE_WINDOW - 1e-9:
                self.ts["stable_start"] = self._hold - self.t
                self.ts["stable_confirmed"] = rel
                self.outcome = "recovered: required population stable"
        else:
            if self._whole_before:
                self.flaps += 1
            self._hold = None
        self._whole_before = whole
        for u in self.required:
            k = sim.gcs.known.get(u)
            if k and k["last_seen"] > self.t + 0.1 and u not in self._heard \
                    and k["last_seen"] > self._lost_at.get(u, self.t):
                self._heard.add(u)
                if self.ts["first_app_delivery"] is None:
                    self.ts["first_app_delivery"] = k["last_seen"] - self.t
        if self.required and self.required <= self._heard and self.ts["all_app_delivery"] is None:
            self.ts["all_app_delivery"] = max(sim.gcs.known[u]["last_seen"]
                                              for u in self.required) - self.t
        if rel > C.FAULT_WINDOW and self.outcome is None:
            self.outcome = f"not stable within {C.FAULT_WINDOW:.0f}s attribution window"

    def close(self, sim):
        if self.outcome is None:
            self.outcome = "unrecovered at end of run"

    def report(self):
        faulted = ("temporary outage: included in its own recovery" if self.temporary
                   else "aircraft lost: irrecoverable, excluded from recovery")
        return {
            "type": self.kind, "uav": self.uid, "t": round(self.t, 2),
            "faulted_aircraft": faulted,
            "population": {"before_fault_routed": sorted(self.before),
                           "orphaned": sorted(self.orphaned or ()),
                           "unaffected": sorted(self.unaffected or ()),
                           "required": sorted((self.orphaned or set())
                                              | ({self.uid} if self.temporary else set())),
                           "membership_changes": self.members_log},
            **{k: _r(v) for k, v in self.ts.items()},
            "flaps": self.flaps,
            "required_disconnected_uav_s": _r(self.disc_uav_s, 1),
            "unaffected_continuity_pct": (_r(100.0 * self.unaffected_ok_s / self.window_s, 1)
                                          if self.window_s and self.unaffected else None),
            "overlapping_faults": self._overlaps,
            "outcome": self.outcome,
            **self.extra,
        }


class Metrics:
    def __init__(self, sim):
        self.sim = sim
        self.faults = []
        self.samples = 0
        self.conn_sum = 0.0
        self.air_s = 0.0
        self.state_s = collections.Counter()    # connected-redundant / connected-critical / partitioned
        self.disc_uav_s = 0.0
        self.max_crit = 0
        self.route_log = []                     # (t, uav, routed, hops, parent) on change
        self._last_route = {}
        self.state_log = []
        self._next_state = 0.0
        self.episodes = []                      # closed per-UAV route episodes
        self._open_ep = {}
        self.mission_eps = []                   # mission-level partition episodes
        self._mission_open = None
        self.silences = []                      # GCS declared-lost -> re-acquired
        self._silence_open = {}
        self.safety_eps = []
        self._pair_open = {}
        self.min_sep = math.inf
        self.fence_violations = 0
        self.min_agl = math.inf
        self.min_battery = 100.0
        self.hops_sum = self.hops_n = self.max_hops = 0
        self._prev_pos = {u.id: tuple(u.pos) for u in sim.uavs}
        self._task_changed = {}
        self._last_task = {u.id: u.task for u in sim.uavs}

    # ---------------------------------------------------------------- faults
    def add_fault(self, kind, uid, temporary, **extra):
        for f in self.faults:
            if f.outcome is None:
                f._overlaps.append({"t": round(self.sim.t, 2), "type": kind, "uav": uid})
        f = FaultTracker(self.sim, kind, uid, temporary, extra)
        self.faults.append(f)
        return f

    # --------------------------------------------------------------- observe
    def observe(self, dt):
        sim = self.sim
        now = sim.t
        for u in sim.uavs:
            if u.task != self._last_task[u.id]:
                self._task_changed[u.id] = now
                self._last_task[u.id] = u.task
        air = [u for u in sim.uavs if u.status in MISSION]
        cut = [u for u in air if not sim.net.connected(u.id)]
        for u in sim.uavs:
            key = (sim.net.connected(u.id), sim.net.hops.get(u.id), sim.net.parent.get(u.id),
                   u.status)
            if self._last_route.get(u.id) != key:
                self._last_route[u.id] = key
                self.route_log.append((round(now, 3), u.id, int(key[0]), key[1], key[2],
                                       u.status))
        if air:
            self.samples += 1
            self.air_s += dt
            self.conn_sum += (len(air) - len(cut)) / len(air)
            self.disc_uav_s += len(cut) * dt
            crit = sim.net.critical_relays({u.id for u in air})
            if cut:
                self.state_s["partitioned"] += dt
            elif crit:
                self.state_s["connected_single_point"] += dt
                self.max_crit = max(self.max_crit, max(crit.values()))
            else:
                self.state_s["connected_redundant"] += dt
            for u in air:
                if sim.net.connected(u.id):
                    h = sim.net.hops[u.id]
                    self.hops_sum += h
                    self.hops_n += 1
                    self.max_hops = max(self.max_hops, h)
                self.min_battery = min(self.min_battery, u.battery)
                if abs(u.pos[2] - u.cruise_alt) < 0.5:
                    self.min_agl = min(self.min_agl, u.pos[2] - u.ground(u.pos[0], u.pos[1]))
        for f in self.faults:
            f.observe(sim, dt)
        self._episodes(air, now)
        self._silence(now)
        self._safety(now, dt)
        xmin, xmax, ymin, ymax = C.GEOFENCE
        for u in sim.uavs:
            if not (xmin <= u.pos[0] <= xmax and ymin <= u.pos[1] <= ymax):
                self.fence_violations += 1
        if now >= self._next_state:
            self._next_state += 1.0
            self._state_row(now)
        self._prev_pos = {u.id: tuple(u.pos) for u in sim.uavs}

    # -------------------------------------------------------- outage ledger
    def _nearest(self, u):
        best = (None, math.inf)
        pos = self.sim.positions()
        for n, p in pos.items():
            if n == u.id or n != GCS_ID and (not self.sim.uav(n).radio_ok
                                             or self.sim.uav(n).status == A.FAILED):
                continue
            d = math.dist(p, u.pos)
            if d < best[1]:
                best = (n, d)
        return best

    def _episodes(self, air, now):
        sim = self.sim
        ids = {u.id for u in air}
        for u in air:
            routed = sim.net.connected(u.id)
            ep = self._open_ep.get(u.id)
            if not routed:
                if ep is None:
                    self._open_ep[u.id] = self._new_ep(u, now)
                else:
                    ep["_last_cut"] = now
                    n, d = self._nearest(u)
                    ep["min_nearest_m"] = min(ep["min_nearest_m"], d)
                    if u.task != ep["_task"]:
                        ep["role_transitions"].append((round(now, 1), u.task[0], u.task[2]))
                        ep["_task"] = u.task
            elif ep is not None and now - ep["_last_cut"] > C.EPISODE_MERGE_GAP:
                self._close_ep(u.id, ep, ep["_last_cut"], "route regained")
        for uid, ep in list(self._open_ep.items()):
            if uid not in ids:
                st = sim.uav(uid).status
                self._close_ep(uid, ep, now, f"left mission ({st})")
        cut = bool(self._open_ep)
        if cut and self._mission_open is None:
            self._mission_open = {"start": round(now, 2), "uavs": set(self._open_ep)}
        elif cut:
            self._mission_open["uavs"] |= set(self._open_ep)
        elif self._mission_open is not None:
            m = self._mission_open
            m.update(end=round(now, 2), duration_s=round(now - m["start"], 2),
                     uavs=sorted(m["uavs"]))
            self.mission_eps.append(m)
            self._mission_open = None

    def _new_ep(self, u, now):
        sim = self.sim
        n, d = self._nearest(u)
        route = [e for e in self.route_log if e[1] == u.id and e[2] == 1]
        last = route[-1] if route else None
        k = sim.gcs.known.get(u.id)
        pdr = sim.channel.cohort_pdr("hb", now, window=10.0, src=u.id)
        prev = self._prev_pos.get(u.id, tuple(u.pos))
        nn = sim.positions().get(n) if n is not None else None
        outward = None
        if nn is not None:
            outward = math.dist(u.pos, nn) > math.dist(prev, nn) + 1e-6
        parent = last[4] if last else None
        d_parent = (round(math.dist(u.pos, sim.positions()[parent]), 1)
                    if parent is not None else None)
        parent_cut_same_time = bool(parent and parent != GCS_ID and parent in self._open_ep
                                    and abs(self._open_ep[parent]["start"] - now) < 0.2)
        parent_moving = False
        if parent and parent != GCS_ID:
            pu = sim.uav(parent)
            parent_moving = math.dist(pu.pos, self._prev_pos.get(parent, tuple(pu.pos))) > 0.1
        conc = [{"type": f.kind, "uav": f.uid, "t": round(f.t, 1)} for f in self.faults
                if f.t <= now <= f.t + C.FAULT_WINDOW]
        target_beyond = None
        if u.task[0] == A.SURVEY and u.task[1] is not None:
            sp = u.survey_point(u.task[1])
            others = [p for nid, p in sim.positions().items() if nid != u.id and (
                nid == GCS_ID or sim.uav(nid).status in MISSION and sim.uav(nid).radio_ok)]
            target_beyond = round(min(math.dist((sp[0], sp[1], u.cruise_alt), p)
                                      for p in others), 1) if others else None
        return {"uav": u.id, "start": round(now, 2), "status": u.status, "task": u.task[0],
                "task_target": u.task[2], "data_mule": u.mule, "radio_ok": u.radio_ok,
                "survey_point_to_nearest_node_m": target_beyond,
                "pos_start": [round(c) for c in u.pos], "nearest_node": n,
                "nearest_m": round(d, 1), "min_nearest_m": d, "moving_away": outward,
                "last_route_parent": parent, "parent_moving": parent_moving,
                "dist_to_last_parent_m": d_parent, "parent_cut_same_time": parent_cut_same_time,
                "parent_task_changed_s_ago": (round(now - self._task_changed[parent], 1)
                                              if parent in self._task_changed else None),
                "hb_pdr_prev_10s": _r(pdr, 1),
                "gcs_heartbeat_age_s": _r(now - k["last_seen"], 2) if k else None,
                "queue_len": sim.channel.queue_len(u.id), "concurrent_faults": conc,
                "role_transitions": [], "_task": u.task, "_last_cut": now}

    def _close_ep(self, uid, ep, t_end, why):
        sim = self.sim
        u = sim.uav(uid)
        ep["end"] = round(t_end, 2)
        ep["duration_s"] = round(t_end - ep["start"], 2)
        ep["end_reason"] = why
        ep["pos_end"] = [round(c) for c in u.pos]
        ep["min_nearest_m"] = round(ep["min_nearest_m"], 1)
        ep["route_after"] = sim.net.route(uid)
        ep["cause"] = self._diagnose(ep)
        for k in ("_task", "_last_cut"):
            ep.pop(k, None)
        self.episodes.append(ep)
        self._open_ep.pop(uid, None)

    def _diagnose(self, ep):
        """Evidence-based cause label (first matching rule; docs/OUTAGE_LEDGER.md)."""
        rng = C.COMM_RANGE if C.RADIO_PROFILE != "degraded" else C.DEGRADED_RANGE
        for f in ep["concurrent_faults"]:
            if f["type"] == "comm_outage" and f["uav"] == ep["uav"]:
                return "injected radio outage (own radio off)"
        if not ep["radio_ok"]:
            return "own radio off"
        for f in self.faults:
            if f.kind == "uav_failure" and 0 <= ep["start"] - f.t < 2.0 \
                    and ep["uav"] in (f.orphaned or ()):
                return "fault-attributed: upstream relay lost"
        if ep["data_mule"]:
            return "data mule carrying imagery back (planned out-of-range leg)"
        if ep["status"] == A.RTH or ep["task"] == A.HOME:
            return "return transit"
        if ep["task"] == A.SURVEY and (ep["nearest_m"] > rng + 40 or (
                ep["survey_point_to_nearest_node_m"] or 0) > rng):
            return "survey beyond the network (planned by GCS)"
        if ep.get("parent_cut_same_time"):
            return "cascade: upstream relay cut off at the same moment"
        dpar = ep.get("dist_to_last_parent_m")
        if dpar is not None and abs(dpar - rng) <= 40:
            if ep["parent_moving"]:
                return "range-boundary crossing: upstream relay moving (re-plan)"
            return "range-boundary crossing: own motion out of range of its upstream"
        if abs(ep["nearest_m"] - rng) <= 40 and ep["moving_away"]:
            if ep["parent_moving"]:
                return "range-boundary crossing: upstream relay moving (re-plan)"
            return "range-boundary crossing: own motion outward"
        if ep["parent_moving"] or (ep["parent_task_changed_s_ago"] is not None
                                   and ep["parent_task_changed_s_ago"] < 20):
            return "relay repositioning after re-plan"
        if ep["task"] == A.SURVEY:
            return "survey beyond the network (planned by GCS)"
        return "unknown"

    def _silence(self, now):
        """GCS silence episodes: declared LOST -> heard again, with how much
        of the silence had a modelled route (timeout with a path present)."""
        gk = self.sim.gcs.known
        for uid, k in gk.items():
            s = self._silence_open.get(uid)
            if k["lost"] and s is None:
                self._silence_open[uid] = {"uav": uid, "declared": round(now, 2),
                                           "silent_since": round(k["last_seen"], 2),
                                           "_routed": 0, "_n": 0}
            elif k["lost"]:
                s["_n"] += 1
                s["_routed"] += int(self.sim.net.connected(uid))
            elif s is not None:
                frac = s["_routed"] / s["_n"] if s["_n"] else 0.0
                s.update(reacquired=round(now, 2), duration_s=round(now - s["silent_since"], 2),
                         routed_fraction=round(frac, 2),
                         kind=("heartbeat timeout with a path present (loss/queue)" if frac >= 0.9
                               else "graph disconnection" if frac <= 0.1 else "mixed"))
                del s["_routed"], s["_n"]
                self.silences.append(s)
                del self._silence_open[uid]

    # -------------------------------------------------------------- safety
    def _phase(self, u):
        if u.status == A.EMERGENCY:
            return "failsafe descent"
        pad = A.pad_position(u.id)
        near_pad = math.hypot(u.pos[0] - pad[0], u.pos[1] - pad[1]) < 3.0
        if near_pad and u.pos[2] < u.cruise_alt - 0.5:
            return "landing" if u.status == A.RTH or u.task[0] == A.HOME else "takeoff"
        if u.status == A.RTH or u.task[0] == A.HOME:
            return "return"
        if u.id in self.sim.gcs.outgoing or any(
                h.get("replacement") == u.id and h.get("t_released") is None
                for h in self.sim.gcs.handovers):
            return "relay replacement"
        if abs(u.pos[2] - u.cruise_alt) > 0.5:
            return "climb/descent at target"
        return "cruise"

    def _safety(self, now, dt):
        fly = [u for u in self.sim.uavs if u.airborne]
        open_now = set()
        for i, a in enumerate(fly):
            for b in fly[i + 1:]:
                d = seg_min_dist(self._prev_pos[a.id], a.pos, self._prev_pos[b.id], b.pos)
                self.min_sep = min(self.min_sep, d)
                key = (a.id, b.id)
                if d < C.NEAR_MISS_M:
                    open_now.add(key)
                    ep = self._pair_open.get(key)
                    if ep is None:
                        ep = self._pair_open[key] = {
                            "uavs": list(key), "start": round(now, 2), "min_m": d,
                            "phases": sorted({self._phase(a), self._phase(b)}),
                            "cause": ("fault consequence" if A.EMERGENCY in (a.status, b.status)
                                      else "controller"),
                            "duration_s": 0.0, "below_collision_s": 0.0}
                    ep["min_m"] = min(ep["min_m"], d)
                    ep["duration_s"] += dt
                    if d < C.COLLISION_M:
                        ep["below_collision_s"] += dt
                    ep["phases"] = sorted(set(ep["phases"]) | {self._phase(a), self._phase(b)})
        for key in list(self._pair_open):
            if key not in open_now:
                ep = self._pair_open.pop(key)
                ep.update(end=round(now, 2), min_m=round(ep["min_m"], 2),
                          duration_s=round(ep["duration_s"], 2),
                          below_collision_s=round(ep["below_collision_s"], 2),
                          severity=("collision (proxy)" if ep["min_m"] < C.COLLISION_M
                                    else "near miss"))
                self.safety_eps.append(ep)

    # ----------------------------------------------------------- raw logs
    def _state_row(self, now):
        sim = self.sim
        for u in sim.uavs:
            k = sim.gcs.known.get(u.id)
            self.state_log.append({
                "t": round(now, 1), "uav": u.id, "x": round(u.pos[0], 1), "y": round(u.pos[1], 1),
                "z": round(u.pos[2], 1), "battery": round(u.battery, 2), "status": u.status,
                "task": u.task[0], "target": u.task[2], "radio": int(u.radio_ok),
                "forwarding": int(u.forwarding), "routed": int(sim.net.connected(u.id)),
                "hops": sim.net.hops.get(u.id), "parent": sim.net.parent.get(u.id),
                "queue": sim.channel.queue_len(u.id),
                "gcs_age_s": _r(now - k["last_seen"], 2) if k else None,
                "gcs_lost": int(k["lost"]) if k else None,
            })

    def close(self):
        now = self.sim.t
        for uid, ep in list(self._open_ep.items()):
            self._close_ep(uid, ep, now, "end of run")
        if self._mission_open is not None:
            m = self._mission_open
            m.update(end=round(now, 2), duration_s=round(now - m["start"], 2), uavs=sorted(m["uavs"]))
            self.mission_eps.append(m)
            self._mission_open = None
        for key in list(self._pair_open):
            ep = self._pair_open.pop(key)
            ep.update(end=round(now, 2), min_m=round(ep["min_m"], 2),
                      duration_s=round(ep["duration_s"], 2),
                      below_collision_s=round(ep["below_collision_s"], 2),
                      severity=("collision (proxy)" if ep["min_m"] < C.COLLISION_M
                                else "near miss"))
            self.safety_eps.append(ep)
        for f in self.faults:
            f.close(self.sim)

    # -------------------------------------------------------- aggregation
    def episode_summary(self):
        by_cause = collections.defaultdict(lambda: {"episodes": 0, "uav_s": 0.0})
        for e in self.episodes:
            c = by_cause[e["cause"]]
            c["episodes"] += 1
            c["uav_s"] = round(c["uav_s"] + e["duration_s"], 1)
        att = sum(e["duration_s"] for e in self.episodes if e["cause"].startswith(
            ("fault-attributed", "injected radio")))
        return {"per_uav_episodes": len(self.episodes),
                "mission_partition_episodes": len(self.mission_eps),
                "merge_policy": f"per UAV, route gaps shorter than {C.EPISODE_MERGE_GAP}s are "
                                "one episode; mission episodes = union over UAVs",
                "by_cause": dict(sorted(by_cause.items(), key=lambda kv: -kv[1]["uav_s"])),
                "fault_attributed_uav_s": round(att, 1),
                "not_fault_attributed_uav_s": round(sum(e["duration_s"] for e in self.episodes)
                                                    - att, 1),
                "gcs_silences": len(self.silences),
                "gcs_silences_with_path_present": sum(1 for s in self.silences
                                                      if s["kind"].startswith("heartbeat timeout"))}
