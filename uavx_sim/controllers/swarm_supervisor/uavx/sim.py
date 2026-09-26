"""Swarm simulation core: runs identically inside Webots and headless.

The Webots controller calls SwarmSim.step() once per basicTimeStep and only
mirrors the resulting state into the 3D scene. tools/run_headless.py and
tools/evaluate.py drive the same class without Webots.

Who knows what. This class is the *simulator*: it holds the true state of
every UAV and the radio graph and uses it for physics, the channel and (via
metrics.Metrics, the evaluator) for scoring. The decision-makers never see
that truth:
  * the GCS (gcs.py) only receives heartbeats and imagery chunks that
    crossed the channel (comms.Channel: routed hop by hop, queued, lossy,
    airtime-limited, expiring);
  * each UAV (agent.py) only uses its own sensors, command packets and
    chunk acks that crossed the channel the other way, and neighbour beacons.
All GCS <-> UAV traffic goes through self.channel; tests/test_information_flow.py
checks this, including negative tests that a deliberate bypass is caught.
Fault injection only changes the faulted UAV (and the observer's event
log); it never informs the GCS or other UAVs directly.
"""

import csv
import json
import math
import os
import random

from . import agent as A
from . import config as C
from .comms import GCS_ID, Channel, Network
from .gcs import GroundStation
from .metrics import Metrics


class SwarmSim:
    def __init__(self, scenario=True, seed=C.SEED, log=print, ground=None,
                 pois=None, script=None, batteries=None, gcs_cls=None, radio=None):
        self.t = 0.0
        self.seed = seed
        self._print = log
        self.events = []                     # (t, message): observer + GCS log
        self.rng = random.Random(seed)
        self.net = Network(seed=seed + 1, ground=ground, profile=radio)
        self.channel = Channel(self.net, seed=seed + 1)
        self.gcs = (gcs_cls or GroundStation)(pois or C.POIS, self.log)
        self.uavs = []
        for i in range(1, C.NUM_UAVS + 1):
            u = A.Uav(i, (batteries or C.INITIAL_BATTERY)[i - 1])
            u.log = self.log
            if ground:
                u.ground = ground
            self.uavs.append(u)
        self.scenario = sorted(script if script is not None else C.SCENARIO,
                               key=lambda e: e[0]) if scenario else []
        self.radio_restore = {}              # uav_id -> time radio comes back
        # Stagger periodic traffic so packets aren't all in the same step.
        self.next_hb = {u.id: 0.1 * u.id for u in self.uavs}
        self.next_cmd = {u.id: 0.05 + 0.1 * u.id for u in self.uavs}
        self.next_thumb = {u.id: 0.37 * u.id for u in self.uavs}
        self.thumb_rx = {}                   # uav -> generation time of the newest thumbnail at GCS
        self.via_net = {"hb": 0, "cmd": 0, "chunk": 0, "ack": 0, "thumb": 0}
        self.done = False
        self.end_reason = None
        self.landed_time = None
        self.survey_done = {}                # pid -> (t, uav) when survey completed (truth)
        self.faults = []                     # public fault list (dicts) for UI / narrator
        self.metrics = Metrics(self)
        self.timeline = []
        self._next_timeline = 0.0

    # ------------------------------------------------------------------ utils
    def log(self, msg):
        self.events.append((self.t, msg))
        self._print(f"[t={self.t:6.1f}s] {msg}")

    def uav(self, uid):
        return self.uavs[uid - 1]

    def positions(self):
        pos = {GCS_ID: C.GCS_POS}
        for u in self.uavs:
            pos[u.id] = tuple(u.pos)
        return pos

    def roles(self):
        """Display role per UAV (simulator truth), for the 3D status lights."""
        fwd = self.net.forwarders()
        out = {}
        for u in self.uavs:
            if u.status == A.FAILED:
                r = "FAILED"
            elif u.status == A.EMERGENCY:
                r = "EMERGENCY"
            elif not u.radio_ok:
                r = "RADIO DOWN"
            elif u.status in (A.RTH, A.SERVICE, A.READY):
                r = u.status
            elif u.task[0] == A.HOME:
                r = "RECALLED"
            elif u.mule:
                r = "DATA MULE"
            elif u.task[0] == A.RELAY and str(u.task[2]).startswith("BKP"):
                r = "BACKUP"
            else:
                r = u.task[0]
            if u.id in fwd and r not in ("FAILED", "RADIO DOWN", "EMERGENCY"):
                r += "+FWD"
            out[u.id] = r
        return out

    # ------------------------------------------------------ fault injection
    def _descendants(self, uid):
        count = 0
        for n in self.net.parent:
            p = self.net.parent.get(n)
            while p is not None and p != GCS_ID:
                if p == uid:
                    count += 1
                    break
                p = self.net.parent.get(p)
        return count

    def inject_failure(self, uid=None, hard=False):
        alive = [u for u in self.uavs if u.status == A.ACTIVE]
        if not alive:
            return None
        if uid is None:
            # Worst case: the relay carrying the most downstream traffic.
            uid = max(alive, key=lambda u: (self._descendants(u.id),
                                            self.net.hops.get(u.id, 0))).id
        deps = self._descendants(uid)
        self.metrics.add_fault("uav_failure", uid, temporary=False, hard=hard, dependants=deps)
        self.faults.append({"type": "uav_failure", "uav": uid, "t": self.t, "hard": hard,
                            "dependants": deps})
        self.uav(uid).fail(hard=hard)
        self.log(f"FAULT: UAV{uid} critical fault{' + radio loss' if hard else ''} "
                 f"(was relaying for {deps} UAV(s))")
        return uid

    def inject_radio_outage(self, duration=15.0, uid=None):
        cands = [u for u in self.uavs if u.status == A.ACTIVE and u.radio_ok]
        if not cands:
            return None
        if uid is None:
            uid = max(cands, key=lambda u: (self.net.hops.get(u.id, 0),
                                            self._descendants(u.id))).id
        self.metrics.add_fault("comm_outage", uid, temporary=True, duration=duration,
                               dependants=self._descendants(uid))
        self.faults.append({"type": "comm_outage", "uav": uid, "t": self.t, "duration": duration})
        self.uav(uid).radio_ok = False
        self.radio_restore[uid] = self.t + duration
        self.log(f"FAULT: UAV{uid} radio outage for {duration:.0f}s")
        return uid

    def inject_battery_sag(self, amount=45.0, uid=None):
        """Energy-availability fault: the relay nearest the GCS that carries
        the most traffic suddenly loses `amount` % of usable charge."""
        cands = [u for u in self.uavs if u.status == A.ACTIVE and u.task[0] == A.RELAY]
        if not cands:
            cands = [u for u in self.uavs if u.status == A.ACTIVE]
        if not cands:
            return None
        if uid is None:
            uid = max(cands, key=lambda u: (self.net.hops.get(u.id) == 1,
                                            self._descendants(u.id))).id
        u = self.uav(uid)
        before = u.battery
        u.battery = max(1.0, u.battery - amount)
        self.faults.append({"type": "battery_sag", "uav": uid, "t": self.t,
                            "battery_before": round(before, 1), "battery_after": round(u.battery, 1),
                            "dependants": self._descendants(uid)})
        self.log(f"FAULT: UAV{uid} energy-availability fault {before:.0f}% -> {u.battery:.0f}% "
                 f"(relaying for {self._descendants(uid)} UAV(s))")
        return uid

    def inject_new_poi(self, poi=None):
        if poi is None:
            n = sum(1 for p in self.gcs.pois if p.startswith("H")) + 1
            poi = (f"H{n}", self.rng.uniform(-800, 800), self.rng.uniform(-800, 800), 2)
        pid, x, y, prio = poi
        self.gcs.add_poi(pid, x, y, prio, self.t)
        return pid

    def clear_faults(self):
        for u in self.uavs:
            if not u.radio_ok and u.status not in (A.EMERGENCY, A.FAILED):
                u.radio_ok = True
                self.log(f"UAV{u.id} radio restored")
        self.radio_restore.clear()

    # --------------------------------------------------------------- traffic
    def _generate(self, radios, now):
        ch = self.channel
        for u in self.uavs:
            if u.status == A.FAILED:
                continue
            up = u.id in radios
            if now >= self.next_hb[u.id]:
                self.next_hb[u.id] += C.HEARTBEAT_PERIOD
                hb = u.heartbeat(self.net.hops.get(u.id), self.net.parent.get(u.id))
                uid = ch.submit("hb", u.id, GCS_ID, C.HB_BYTES, now, hb)
                if not up:
                    self._drop_radio_down(u.id, uid, now)
            if up:
                for key, retx in u.chunks_to_send(now):
                    payload = (u.id, key, u.detections.get(key[1]), None)
                    if retx:
                        ch.retransmit("data", u.id, GCS_ID, C.CHUNK_BYTES, now, payload, key)
                    else:
                        ch.submit("data", u.id, GCS_ID, C.CHUNK_BYTES, now, payload, key=key)
            if now >= self.next_thumb[u.id]:
                self.next_thumb[u.id] += C.THUMB_PERIOD
                if u.airborne:
                    uid = ch.submit("thumb", u.id, GCS_ID, C.THUMB_BYTES, now, (u.id, now))
                    if not up:
                        self._drop_radio_down(u.id, uid, now)
        for u in self.uavs:
            if now >= self.next_cmd[u.id]:
                self.next_cmd[u.id] += C.COMMAND_PERIOD
                ch.submit("cmd", GCS_ID, u.id, C.CMD_BYTES, now, self.gcs.make_command(u.id, now))

    def _drop_radio_down(self, node, uid, now):
        q = self.channel.queues[node]
        for i, p in enumerate(q):
            if p.uid == uid:
                q.pop(i)
                self.channel._finish(p, "dropped_radio_down", now)
                break

    def _dispatch(self):
        for t, p in self.channel.delivered:
            self.via_net[{"data": "chunk"}.get(p.cls, p.cls)] += 1
            if p.cls == "hb":
                self.gcs.on_heartbeat(p.payload, t)
            elif p.cls == "cmd":
                u = self.uav(p.dst)
                if u.radio_ok and u.status != A.FAILED:
                    u.receive_command(p.payload, t)
            elif p.cls == "data":
                uid, key, persons, _ = p.payload
                if self.gcs.on_chunk(uid, key, persons, p.hops_done, t):
                    self.channel.submit("ack", GCS_ID, uid, C.ACK_BYTES, t, key)
            elif p.cls == "ack":
                u = self.uav(p.dst)
                if u.radio_ok:
                    u.on_chunk_ack(p.payload, t)
            elif p.cls == "thumb":
                uid, t_gen = p.payload
                self.thumb_rx[uid] = max(self.thumb_rx.get(uid, -1.0), t_gen)
        self.channel.delivered = []

    # ------------------------------------------------------------------ step
    def step(self, dt=C.DT):
        if self.done:
            return
        t0 = self.t
        self.t += dt
        now = self.t

        while self.scenario and self.scenario[0][0] <= now:
            _, kind, arg = self.scenario.pop(0)
            if kind == "fail":
                arg = arg or ""
                uid = None
                if "random" in arg:
                    alive = [u.id for u in self.uavs if u.status == A.ACTIVE]
                    uid = self.rng.choice(alive) if alive else None
                elif "uav" in arg:                 # e.g. "hard-uav3": a fixed aircraft id
                    uid = int(arg.split("uav")[1])
                    if self.uav(uid).status != A.ACTIVE:
                        uid = None
                self.inject_failure(uid, hard="hard" in arg)
            elif kind == "radio":
                self.inject_radio_outage(arg or 15.0)
            elif kind == "new_poi":
                self.inject_new_poi(arg)
            elif kind == "battery_sag":
                self.inject_battery_sag(arg or 45.0)

        for uid, when in list(self.radio_restore.items()):
            if now >= when:
                self.uav(uid).radio_ok = True
                del self.radio_restore[uid]
                self.log(f"UAV{uid} radio restored")

        radios = {GCS_ID} | {u.id for u in self.uavs
                             if u.radio_ok and u.status != A.FAILED}
        no_fwd = {u.id for u in self.uavs if not u.forwarding}
        self.net.update(self.positions(), radios, no_fwd)
        self.channel.set_radios(radios)

        self._generate(radios, now)
        self.channel.step(t0, now)
        self._dispatch()
        self.gcs.update(now)

        self._beacons()
        self._collision_avoidance()
        for u in self.uavs:
            before = set(u.surveyed)
            u.step(now, dt)
            for pid in u.surveyed - before:
                self.survey_done.setdefault(pid, (now, u.id))

        self.metrics.observe(dt)
        self._timeline(now)
        self._check_end()

    def _beacons(self):
        """Neighbour beacons: each UAV learns the task kind of every UAV in its
        own mesh fragment (reachable over air-to-air links, GCS excluded), and
        whether that fragment has a route (routing table). Local radio
        information, not GCS knowledge."""
        comp = {}
        for u in self.uavs:
            if u.id not in self.net.links:
                u.mesh_tasks = []
                continue
            if u.id not in comp:
                seen, stack = {u.id}, [u.id]
                while stack:
                    n = stack.pop()
                    for m in self.net.links[n]:
                        if m != GCS_ID and m not in seen:
                            seen.add(m)
                            stack.append(m)
                for n in seen:
                    comp[n] = seen
            frag = comp[u.id]
            if any(self.net.connected(n) for n in frag):
                u.mesh_tasks = []
            else:
                u.mesh_tasks = [self.uav(n).task[0] for n in frag if n != u.id]

    def _collision_avoidance(self):
        """Separation assurance.

        Primary: every UAV cruises on its own altitude layer, so transits
        never conflict. A UAV climbing or descending toward another UAV's
        altitude stops short of it while they are close horizontally.
        Emergency keep-out: a UAV in a failsafe descent can't manoeuvre, so
        every other UAV below it keeps EMERGENCY_KEEPOUT metres of horizontal
        clearance from its descent column.
        Fallback: a horizontal repulsive push below MIN_SEPARATION.
        """
        falling = [e for e in self.uavs if e.status == A.EMERGENCY]
        for u in self.uavs:
            u.avoid = (0.0, 0.0)
            u.hold_vertical = False
            u.back_off = False
            u.clear_of = []
            u.keepout = [(e.pos[0], e.pos[1]) for e in falling if e is not u and (
                e.radio_ok and u.id in self.net.links.get(e.id, {})
                or math.dist(e.pos, u.pos) < 60.0)]
        air = [u for u in self.uavs if u.airborne and u.status != A.FAILED]
        for i, a in enumerate(air):
            for b in air[i + 1:]:
                dx, dy = a.pos[0] - b.pos[0], a.pos[1] - b.pos[1]
                h = math.hypot(dx, dy)
                dz = abs(a.pos[2] - b.pos[2])
                if h < 60.0 and dz < 12.0:
                    # Vertical yield: a UAV climbing/descending TOWARD another
                    # one's altitude stops 12 m short while it is within 60 m
                    # horizontally (~4 s warning at cruise speed), and backs
                    # off vertically if it is already inside 8 m and within
                    # 25 m. If both move toward each other, the higher id yields.
                    def toward(u, o):
                        return (u.status != A.EMERGENCY and abs(u.vdir) > 0.5
                                and u.vdir * (o.pos[2] - u.pos[2]) > 0)
                    a_moving, b_moving = toward(a, b), toward(b, a)
                    y = a if a_moving and (not b_moving or a.id > b.id) else (b if b_moving else None)
                    if y is not None:
                        y.hold_vertical = True
                        if dz < 8.0 and h < 25.0:
                            y.back_off = True
                if h < 25.0 and dz < 7.5:
                    # Horizontal yield: a levelled UAV doesn't close in on a
                    # UAV that is off its layer within 7.5 m of our altitude.
                    # No deadlock: that UAV either moves away vertically or
                    # backs off (above) until the gap is >= 8 m.
                    for lv, ot in ((a, b), (b, a)):
                        if (abs(lv.pos[2] - lv.cruise_alt) < 0.5
                                and abs(ot.pos[2] - ot.cruise_alt) >= 0.5
                                and ot.status != A.EMERGENCY):
                            lv.clear_of.append((ot.pos[0], ot.pos[1]))
                for e, o, sx, sy in ((a, b, -dx, -dy), (b, a, dx, dy)):
                    if (e.status == A.EMERGENCY and o.status != A.EMERGENCY
                            and h < C.EMERGENCY_KEEPOUT and o.pos[2] < e.pos[2] + 5.0):
                        ux, uy = _away(sx, sy, h, o.id)
                        push = 10.0 * (C.EMERGENCY_KEEPOUT - h) / C.EMERGENCY_KEEPOUT + 2.0
                        o.avoid = (o.avoid[0] + ux * push, o.avoid[1] + uy * push)
                d = math.dist(a.pos, b.pos)
                if d < C.MIN_SEPARATION:
                    ux, uy = _away(dx, dy, h, a.id)
                    push = 6.0 * (C.MIN_SEPARATION - d) / C.MIN_SEPARATION
                    a.avoid = (a.avoid[0] + ux * push, a.avoid[1] + uy * push)
                    b.avoid = (b.avoid[0] - ux * push, b.avoid[1] - uy * push)

    # --------------------------------------------------------------- metrics
    def _timeline(self, now):
        if now < self._next_timeline:
            return
        self._next_timeline += 1.0
        air = [u for u in self.uavs if u.status in (A.ACTIVE, A.RTH)]
        gk = self.gcs.known
        active_ages = [now - k["last_seen"] for k in gk.values()
                       if not k["lost"] and k["status"] in (A.ACTIVE, A.RTH, A.EMERGENCY)]
        self.timeline.append({
            "t": round(now, 1),
            "delivered": len(self.gcs.delivered_ids()),
            "pois": len(self.gcs.pois),
            "airborne": len(air),
            "routed": sum(1 for u in air if self.net.connected(u.id)),
            "gcs_believed_airborne": sum(1 for k in gk.values()
                                         if k["status"] in (A.ACTIVE, A.RTH, A.EMERGENCY)),
            "gcs_lost": sum(1 for k in gk.values() if k["lost"]),
            "forwarders": len(self.net.forwarders()),
            "max_hops": max((self.net.hops.get(u.id, 0) for u in air), default=0),
            "min_battery": round(min((u.battery for u in air), default=100.0), 1),
            "hb_pdr_10s": _r(self.channel.cohort_pdr("hb", now, C.PDR_WINDOW), 1),
            "cmd_pdr_10s": _r(self.channel.cohort_pdr("cmd", now, C.PDR_WINDOW), 1),
            "gcs_active_data_age_s": _r(max(active_ages), 2) if active_ages else None,
            "queue_total": sum(len(q) for q in self.channel.queues.values()),
        })

    def _check_end(self):
        landed = self.gcs.complete_time is not None and all(
            u.status == A.FAILED or (u.status in (A.READY, A.SERVICE)
                                     and u.pos[2] <= C.PAD_Z + 0.05)
            for u in self.uavs)
        if landed and self.landed_time is None:
            self.landed_time = self.t
            self.log(f"OBSERVER: surviving fleet landed (t={self.t:.1f}s); imagery for every "
                     f"site at GCS since t={self.gcs.complete_time:.1f}s")
        if self.landed_time is not None and (self.gcs.fleet_confirmed_time is not None
                                             or self.t > self.landed_time + 15.0):
            self.done = True
            self.end_reason = "mission complete, fleet landed"
            self.log("MISSION COMPLETE: imagery at GCS "
                     f"t={self.gcs.complete_time:.1f}s · fleet landed (observer) "
                     f"t={self.landed_time:.1f}s · GCS-confirmed "
                     + (f"t={self.gcs.fleet_confirmed_time:.1f}s"
                        if self.gcs.fleet_confirmed_time is not None else "not yet"))
        elif self.t >= C.MAX_TIME:
            self.done = True
            self.end_reason = "time limit"
            self.log("time limit reached")
        if self.done:
            self.metrics.close()
            self.channel.finish(self.t)

    # --------------------------------------------------------------- results
    def results(self):
        m = self.metrics
        pois = self.gcs.pois
        delivered = [p for p in pois.values() if p["delivered"] is not None]
        sites = {}
        for pid, p in pois.items():
            sv = self.survey_done.get(pid)
            sites[pid] = {
                "priority": p["prio"],
                "added_s": _r(p["added"]),
                "surveyed_s": _r(sv[0]) if sv else None,
                "surveyed_by": sv[1] if sv else None,
                "imagery_delivered_s": _r(p["delivered"]),
                "survey_to_gcs_s": _r(p["delivered"] - sv[0]) if sv and p["delivered"] else None,
                "report_to_gcs_s": _r(p["delivered"] - p["added"]) if p["delivered"] else None,
                "within_internal_deadline": bool(sv and p["delivered"]
                                                 and p["delivered"] - sv[0] <= C.DATA_DEADLINE),
                "persons_detected": p["persons"],
                "persons_ground_truth": C.SURVIVORS_GT.get(pid),
            }
        lat = [s["survey_to_gcs_s"] for s in sites.values() if s["survey_to_gcs_s"] is not None]
        scored = [s for s in sites.values() if s["persons_detected"] is not None
                  and s["persons_ground_truth"] is not None]
        ch = self.channel

        def cls(c):
            st = ch.stats[c]
            out = {k: st[k] for k in ("generated", "delivered", "expired", "dropped",
                                      "dropped_radio_down", "lost", "lost_attempts", "hop_tx",
                                      "e2e_retransmissions", "pending_end") if st[k]}
            out["bytes_generated"] = ch.bytes[c]["generated"]
            out["bytes_on_air"] = ch.bytes[c]["hop_tx"]
            if c != "data":
                out["pdr_cohort_pct"] = _r(ch.cohort_pdr(c, self.t + 100.0), 1)
            out["latency"] = ch.latency_stats(c)
            return out
        lat_cmd = sorted(self.gcs.cmd_ack_latency)
        hi = [s for s in sites.values() if s["priority"] > 1]
        landings = [(u.id, t, b) for u in self.uavs for t, b in u.landings]
        faults = [f.report() for f in m.faults]
        faults += [dict(pub) for pub in self.faults if pub["type"] == "battery_sag"]
        res = {
            "end_reason": self.end_reason,
            "sim_time_s": round(self.t, 1),
            "config": {"seed": self.seed, "radio_profile": self.net.profile,
                       "backup_policy": C.BACKUP_POLICY, "planner": type(self.gcs).__name__,
                       "link_rate_mbps": C.LINK_RATE_MBPS, "mac_retries": C.MAC_RETRIES},
            "mission": {
                "pois_total": len(pois),
                "pois_delivered": len(delivered),
                "completion_pct": round(100.0 * len(delivered) / len(pois), 1),
                "completion_definition": "a site is complete when all its imagery chunks "
                                         f"({A.N_CHUNKS} x {C.CHUNK_BYTES} B) are at the GCS",
                "all_delivered_at_s": _r(self.gcs.complete_time),
                "fleet_landed_at_s_observer": _r(self.landed_time),
                "fleet_landed_confirmed_by_gcs_s": _r(self.gcs.fleet_confirmed_time),
                "unaccounted_by_gcs": sorted(u for u, k in self.gcs.known.items() if k["lost"]),
                "sites": sites,
                "survey_to_gcs_latency_s": {"mean": _r(sum(lat) / len(lat)) if lat else None,
                                            "max": _r(max(lat)) if lat else None},
                "sites_within_internal_deadline": sum(s["within_internal_deadline"]
                                                      for s in sites.values()),
                "internal_deadline_s": C.DATA_DEADLINE,
                "high_priority_response_s": [s["report_to_gcs_s"] for s in hi],
            },
            "perception": {
                "count_rule": "per site: number of static ground tracks (person boxes projected "
                              "to the ground, within 8 m of the site, seen in >= 20% of the "
                              "settled survey frames); see docs/PERCEPTION.md",
                "sites_scored": len(scored),
                "sites_exact": sum(1 for s in scored
                                   if s["persons_detected"] == s["persons_ground_truth"]),
                "persons_undercount": sum(max(0, s["persons_ground_truth"] - s["persons_detected"])
                                          for s in scored),
                "persons_overcount": sum(max(0, s["persons_detected"] - s["persons_ground_truth"])
                                         for s in scored),
                "note": "count error only: no per-person matching, so misses and false "
                        "detections at one site can cancel; empty = no detector (headless)",
            },
            "communication": {
                "connectivity_pct": round(100.0 * m.conn_sum / max(1, m.samples), 1),
                "disconnected_uav_s": round(m.disc_uav_s, 1),
                "network_state_pct": {k: round(100.0 * v / max(1e-9, m.air_s), 1)
                                      for k, v in sorted(m.state_s.items())},
                "network_state_definition": "share of time with >= 1 airborne mission UAV: "
                                            "partitioned (some UAV has no route) / connected with "
                                            "a critical relay / connected and redundant",
                "worst_critical_relay_dependants": m.max_crit,
                "outage_episodes": m.episode_summary(),
                "classes": {c: cls(c) for c in ("hb", "cmd", "ack", "data", "thumb")},
                "command_ack_latency_s": ({"n": len(lat_cmd),
                                           "p50": _r(lat_cmd[len(lat_cmd) // 2], 3),
                                           "p95": _r(lat_cmd[int(0.95 * (len(lat_cmd) - 1))], 3),
                                           "max": _r(lat_cmd[-1], 3)} if lat_cmd else None),
                "commands_superseded_before_delivery": self.gcs.cmd_superseded,
                "commands_discarded_stale": sum(u.cmd_stale for u in self.uavs),
                "commands_discarded_expired": sum(u.cmd_expired for u in self.uavs),
                "heartbeats_discarded_stale_at_gcs": self.gcs.hb_stale,
                "imagery_chunks_duplicate_at_gcs": self.gcs.chunk_dup,
                "airtime_s": {str(k): round(v, 1) for k, v in sorted(self.channel.airtime.items())},
                "mean_hops": round(m.hops_sum / max(1, m.hops_n), 2),
                "max_hops": m.max_hops,
            },
            "relay": {
                "energy_handovers": [self._handover_report(h, landings) for h in self.gcs.handovers],
                "redundancy_infeasible": self.gcs.redundancy_infeasible,
            },
            "faults": faults,
            "safety": {
                "thresholds": {"near_miss_m": C.NEAR_MISS_M, "collision_proxy_m": C.COLLISION_M,
                               "note": "geometric proxies on kinematic trajectories; closest "
                                       "approach checked continuously between 64 ms samples"},
                "min_separation_m": _r(m.min_sep if m.min_sep < math.inf else None, 2),
                "near_miss_episodes": sum(1 for e in m.safety_eps if e["severity"] == "near miss"),
                "collision_proxy_episodes": sum(1 for e in m.safety_eps
                                                if e["severity"] != "near miss"),
                "controller_caused_episodes": sum(1 for e in m.safety_eps
                                                  if e["cause"] == "controller"),
                "violation_time_s": round(sum(e["duration_s"] for e in m.safety_eps), 2),
                "episodes": m.safety_eps,
                "min_terrain_clearance_cruise_m": _r(m.min_agl if m.min_agl < math.inf else None),
                "geofence_violations": m.fence_violations,
                "min_airborne_battery_pct": round(m.min_battery, 1),
                "battery_depletions": sum(1 for u in self.uavs if u.depleted),
                "landings": [{"uav": u, "t": _r(t), "battery_pct": _r(b)} for u, t, b in landings],
            },
            "information_flow": self.information_flow(),
        }
        res["constraints"] = evaluate_constraints(res)
        return res

    def _handover_report(self, h, landings):
        """GCS handover record + observer-measured service to its dependants."""
        out = {k: v for k, v in h.items() if not k.startswith("_")}
        out["checks"] = {k: v for k, v in h.get("checks", {}).items() if k != "served"}
        out["landed_battery"] = next((round(b, 1) for u, t, b in landings
                                      if u == h["uav"] and t >= (h["t_start"] or 0)), None)
        t0 = h.get("t_dispatch") or h["t_start"]
        t1 = h.get("t_released")
        deps = set(h.get("dependants", []))
        rows = [r for r in self.metrics.state_log if r["uav"] in deps and r["status"] == A.ACTIVE]
        during = [r for r in rows if t0 <= r["t"] <= (t1 or self.t)]
        after = [r for r in rows if t1 is not None and t1 < r["t"] <= t1 + 30.0]
        out["observer_dependant_unrouted_s_during"] = sum(1 for r in during if not r["routed"])
        out["observer_dependant_unrouted_s_30s_after_release"] = sum(1 for r in after
                                                                     if not r["routed"])
        # Attribute every dependant outage in [dispatch, release + 30 s] to its
        # ledger cause, so a planned survey leg is not blamed on the handover.
        hi = (t1 if t1 is not None else self.t) + 30.0
        causes = {}
        for e in self.metrics.episodes:
            if e["uav"] in deps and e["start"] < hi and e["end"] > t0:
                d = min(e["end"], hi) - max(e["start"], t0)
                causes[e["cause"]] = round(causes.get(e["cause"], 0.0) + d, 1)
        planned = ("survey beyond the network", "data mule", "injected radio", "return transit")
        out["dependant_outage_by_cause_s"] = causes
        out["handover_attributable_interruption_s"] = round(sum(
            v for k, v in causes.items() if not k.startswith(planned)), 1)
        return out

    def information_flow(self):
        cmd_applied = sum(u.cmd_rx for u in self.uavs)
        return {
            "gcs_heartbeats_applied": self.gcs.hb_rx,
            "gcs_heartbeats_stale_discarded": self.gcs.hb_stale,
            "heartbeats_that_crossed_the_channel": self.via_net["hb"],
            "uav_commands_applied": cmd_applied,
            "commands_that_crossed_the_channel": self.via_net["cmd"],
            "gcs_chunks_received": self.gcs.chunk_rx,
            "chunks_that_crossed_the_channel": self.via_net["chunk"],
            "bypass": (self.gcs.hb_rx + self.gcs.hb_stale != self.via_net["hb"]
                       or cmd_applied > self.via_net["cmd"]
                       or self.gcs.chunk_rx != self.via_net["chunk"]),
        }

    # ---------------------------------------------------------- raw logs
    def write_logs(self, out_dir, tag):
        """Raw per-run logs for independent recomputation (tools/recompute_metrics.py)."""
        os.makedirs(out_dir, exist_ok=True)
        m = self.metrics

        def w(name, rows, fields):
            with open(os.path.join(out_dir, f"{name}_{tag}.csv"), "w", newline="") as f:
                wr = csv.writer(f)
                wr.writerow(fields)
                wr.writerows(rows)
        w("packets", [[uid] + r[:7] + ["" if r[7] is None else "|".join(map(str, r[7]))]
                      for uid, r in self.channel.records.items()],
          ["uid", "class", "src", "dst", "t_gen", "t_delivered", "outcome", "hops", "chunk_key"])
        w("routes", m.route_log, ["t", "uav", "routed", "hops", "parent", "status"])
        for name, rows in (("state", m.state_log), ("timeline", self.timeline)):
            if rows:
                with open(os.path.join(out_dir, f"{name}_{tag}.csv"), "w", newline="") as f:
                    wr = csv.DictWriter(f, fieldnames=list(rows[0]))
                    wr.writeheader()
                    wr.writerows(rows)
        with open(os.path.join(out_dir, f"ledger_{tag}.json"), "w") as f:
            json.dump({"per_uav_route_episodes": m.episodes,
                       "mission_partition_episodes": m.mission_eps,
                       "gcs_silence_episodes": m.silences,
                       "safety_episodes": m.safety_eps}, f, indent=1, default=list)
        with open(os.path.join(out_dir, f"gcs_events_{tag}.csv"), "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(["t", "uav", "event"])
            for t, uid in self.gcs.lost_events:
                wr.writerow([round(t, 3), uid, "declared_lost"])
            for t, uid in self.gcs.emergency_events:
                wr.writerow([round(t, 3), uid, "emergency_reported"])
        with open(os.path.join(out_dir, f"events_{tag}.log"), "w") as f:
            for t, msg in self.events:
                src = "GCS" if msg.startswith("GCS:") else "OBSERVER"
                f.write(f"[t={t:7.2f}s] [{src}] {msg}\n")


def evaluate_constraints(res):
    """Predeclared internal constraints (config.CONSTRAINTS) -> pass/fail."""
    K = C.CONSTRAINTS
    m, s = res["mission"], res["safety"]
    n = max(1, m["pois_total"])
    hp = m["high_priority_response_s"]
    checks = {
        "eventually_complete": m["pois_delivered"] == m["pois_total"],
        "imagery_complete_by": (m["all_delivered_at_s"] is not None
                                and m["all_delivered_at_s"] <= K["imagery_complete_by_s"]),
        "fleet_landed_by": (m["fleet_landed_at_s_observer"] is not None
                            and m["fleet_landed_at_s_observer"] <= K["fleet_landed_by_s"]),
        "high_priority_response": all(v is not None and v <= K["high_priority_response_s"]
                                      for v in hp),
        "timely_sites": m["sites_within_internal_deadline"] / n >= K["timely_sites_min_frac"],
        "separation": not any(e["cause"] == "controller" and e["min_m"] < K["min_separation_m"]
                              for e in s["episodes"]),
        "no_collision": s["collision_proxy_episodes"] <= K["collisions"],
        "geofence": s["geofence_violations"] <= K["geofence_violations"],
        "no_depletion": s["battery_depletions"] <= K["battery_depletions"],
        "landing_reserve": all(x["battery_pct"] >= K["min_landing_battery_pct"] - 1e-6
                               for x in s["landings"]),
    }
    checks["operational_success"] = all(checks.values())
    return checks


def _away(dx, dy, h, uid):
    """Unit vector along (dx, dy); a fixed per-UAV direction if they are
    exactly stacked (so the push is never zero)."""
    if h > 1e-3:
        return dx / h, dy / h
    a = uid * 2.399963                      # golden angle: distinct per UAV
    return math.cos(a), math.sin(a)


def _r(v, nd=1):
    return None if v is None else round(v, nd)
