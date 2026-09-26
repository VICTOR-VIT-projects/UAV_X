"""Ground Control Station: fleet awareness, failure detection, task planning.

Information flow. The GCS never reads UAV state directly. It has exactly two
inputs, both of which arrive only through packets that survived the
multi-hop network (loss, per-hop latency, capacity):
  * on_heartbeat(): telemetry a UAV chose to send (position, battery, status,
    route, which PoIs it holds data for);
  * on_chunk(): a piece of a site's imagery bundle.
Its picture of the fleet is therefore stale or incomplete during a partition,
and every field it acts on carries the time it was last heard. Its only
output, the task for each UAV, likewise reaches a UAV only through a command
packet that survives the network (see SwarmSim.step).

Planner (runs every PLAN_PERIOD seconds on the GCS's current knowledge):
  1. Order pending PoIs by priority (high first), then distance from the GCS.
  2. For each PoI, find the nearest existing network anchor (the GCS or a
     relay slot already planned) and compute how many relays are needed to
     bridge the gap at RELAY_SPACING. If the fleet can afford the chain plus
     a surveyor, reserve those slots. Chains branch off each other, so the
     relay layout grows as a tree rooted at the GCS.
     Surveyors that went silent keep their slot for REQUEUE_AFTER_LOST
     seconds (they are most likely still working); silent relays are
     replaced at once.
  3. If the most urgent remaining PoI can't get a full chain, one "data
     mule" surveys it out of range and flies back until its data gets
     through, while any spare UAVs extend the network toward it.
  4. Proactive redundancy: if a UAV is still spare, it becomes a BACKUP relay
     beside the relay that currently carries the most other UAVs' traffic,
     so that relay stops being a single point of failure.
  5. Assign UAVs to slots by exhaustive search over permutations (at most
     5! = 120 for this fleet), minimising total flight distance with a bonus
     for keeping a UAV on its current slot so the plan doesn't thrash.
     A UAV is only eligible for a new slot if
        E_remaining > E_transit + E_on_station + E_return + E_reserve.
  6. Energy-driven relay handover: once a relay's margin above
     (return + reserve) drops below HANDOVER_MARGIN, a replacement is sent to
     its slot; the outgoing relay keeps forwarding until the replacement is on
     station and linked, and only then is released home.
  7. Spare UAVs are sent home to recharge. When every PoI is delivered, all
     UAVs are recalled.
"""

import itertools
import math

from . import agent as A
from . import config as C

STICKY_BONUS = 30.0       # metres of "free" distance for keeping a slot
DATA_MBIT = C.SURVEY_DATA_MB * 8.0
N_CHUNKS = A.N_CHUNKS


class GroundStation:
    def __init__(self, pois, log):
        self.log = log
        self.pois = {}
        for pid, x, y, prio in pois:
            self.add_poi(pid, x, y, prio, now=0.0, announce=False)
        self.known = {}          # uav_id -> last heartbeat contents + last_seen
        self.tasks = {}          # uav_id -> (kind, (x, y), poi)
        self.task_keys = {}      # uav_id -> slot identity used for stickiness
        self.next_plan = 0.0
        self.complete_time = None
        self.lost_events = []    # (time, uav_id) when a UAV was declared lost
        self.emergency_events = []  # (time, uav_id) when a UAV reported EMERGENCY
        self.handovers = []      # energy-driven relay handovers, see _handover()
        self.outgoing = {}       # uav_id -> handover record while it is being relieved
        self.backup_of = None    # uav id the current BACKUP slot shadows
        self.hb_rx = 0           # heartbeats accepted (the only telemetry input)
        self.hb_stale = 0        # heartbeats discarded: older than one already applied
        self.chunk_rx = 0        # imagery chunks received (incl. duplicates)
        self.chunk_dup = 0       # ... of which duplicates (retransmission after a lost ack)
        self.cmd_seq = {}        # uav -> last command seq issued
        self.cmd_sent = {}       # uav -> {seq: t_sent} awaiting acknowledgement in a heartbeat
        self.cmd_ack_latency = []  # s from sending a command to seeing it applied (heartbeat)
        self.cmd_superseded = 0    # commands overtaken by a newer one before being applied
        self.slot_cmd_seq = {}   # uav -> seq of the first command carrying its current task
        self.drain = set()       # relays told to stop forwarding (handover verification)
        self.redundancy_infeasible = []    # (t, reason) redundancy wanted but not feasible
        self.fleet_confirmed_time = None   # GCS evidence that every reachable UAV is on its pad
        self.inputs = []         # (t, kind, payload) every input, for replay tests (off by default)
        self.record_inputs = False

    # ---------------------------------------------------------------- inputs
    def add_poi(self, pid, x, y, prio, now, announce=True):
        self.pois[pid] = {"x": x, "y": y, "prio": prio, "delivered": None,
                          "added": now, "rx": 0.0, "held_by": None,
                          "held_known": None, "persons": None, "from": None}
        self.complete_time = None
        if announce:
            tag = "HIGH-PRIORITY " if prio > 1 else ""
            self.log(f"GCS: new {tag}PoI {pid} reported at ({x:.0f}, {y:.0f})")

    def on_heartbeat(self, hb, now):
        uid = hb["id"]
        prev = self.known.get(uid)
        if prev and hb["seq"] <= prev["seq"]:
            self.hb_stale += 1                  # reordered/duplicate: never roll state back
            return
        if self.record_inputs:
            self.inputs.append((now, "hb", hb))
        self.hb_rx += 1
        pend = self.cmd_sent.get(uid, {})
        applied = hb.get("cmd_seq", 0)
        if applied in pend:                    # the command the UAV reports it applied
            self.cmd_ack_latency.append(now - pend.pop(applied))
        for seq in [q for q in pend if q < applied]:
            del pend[seq]                      # superseded before delivery (never applied)
            self.cmd_superseded += 1
        if prev and prev["lost"]:
            self.log(f"GCS: UAV{uid} link re-acquired after "
                     f"{now - prev['last_seen']:.1f}s")
        if hb["status"] == A.EMERGENCY and (prev is None or prev["status"] != A.EMERGENCY):
            self.emergency_events.append((now, uid))
            self.log(f"GCS: UAV{uid} reports EMERGENCY landing -> replacing it")
            self.next_plan = now
        self.known[uid] = dict(hb, last_seen=now, lost=False)
        for pid in hb["data"]:
            p = self.pois.get(pid)
            if p and p["delivered"] is None and p["held_known"] is None:
                p["held_known"] = now
                p["held_by"] = uid

    def on_chunk(self, uid, key, persons, hops, now):
        """Imagery chunk `key` = (uav, poi, index) arrived. Deduplicated by
        key. Returns True: the GCS always acks (the sender may have missed an
        earlier ack)."""
        if self.record_inputs:
            self.inputs.append((now, "chunk", (uid, key, persons, hops)))
        self.chunk_rx += 1
        pid, idx = key[1], key[2]
        p = self.pois.get(pid)
        if p is None:
            return True
        got = p.setdefault("chunks", set())
        if idx in got:
            self.chunk_dup += 1
            return True
        got.add(idx)
        if p["delivered"] is not None:
            return True
        p["rx"] = DATA_MBIT * len(got) / N_CHUNKS
        if p["held_known"] is None:
            p["held_known"], p["held_by"] = now, uid
        if len(got) >= N_CHUNKS:
            p["delivered"] = now
            p["persons"] = persons
            p["from"] = uid
            ppl = "" if persons is None else f", onboard detector flagged {persons} person(s)"
            self.log(f"GCS: imagery for {pid} complete from UAV{uid} "
                     f"({hops} hop{'s' if hops != 1 else ''}){ppl}")
        return True

    def make_command(self, uid, now):
        """The command packet for `uid`: newest task, delivery acks, drain
        flag, a sequence number and an expiry."""
        seq = self.cmd_seq.get(uid, 0) + 1
        self.cmd_seq[uid] = seq
        pend = self.cmd_sent.setdefault(uid, {})
        pend[seq] = now
        for q in [q for q in pend if now - pend[q] > 30.0]:
            del pend[q]                       # never acknowledged: give up tracking
        task = self.tasks.get(uid)
        if task is not None and self.slot_cmd_seq.get(uid, (None, None))[0] != task:
            self.slot_cmd_seq[uid] = (task, seq)
        return {"seq": seq, "t_sent": now, "expiry": now + C.COMMAND_EXPIRY, "task": task,
                "delivered": self.delivered_ids(), "drain": uid in self.drain}

    def task_acknowledged(self, uid):
        """True if a heartbeat shows `uid` applied a command carrying its
        current task (downlink verified by uplink)."""
        k = self.known.get(uid)
        rec = self.slot_cmd_seq.get(uid)
        return bool(k and rec and rec[0] == self.tasks.get(uid)
                    and k.get("cmd_seq", 0) >= rec[1] and k.get("task") == rec[0])

    def delivered_ids(self):
        return {pid for pid, p in self.pois.items() if p["delivered"] is not None}

    def task_for(self, uid):
        return self.tasks.get(uid)

    def data_age(self, uid, now):
        k = self.known.get(uid)
        return None if k is None else now - k["last_seen"]

    # ---------------------------------------------------------------- update
    def update(self, now):
        for uid, k in self.known.items():
            if not k["lost"] and now - k["last_seen"] > C.LOST_TIMEOUT:
                k["lost"] = True
                self.lost_events.append((now, uid))
                self.log(f"GCS: UAV{uid} declared LOST (no heartbeat for "
                         f"{C.LOST_TIMEOUT:.0f}s) -> replanning")
                self.next_plan = now   # react immediately
        if now >= self.next_plan:
            self.next_plan = now + C.PLAN_PERIOD
            self._plan(now)

    # ------------------------------------------------------------- planning
    def _out_of_contact(self, now):
        """Surveyors that went silent recently. Surveyors are expected to
        fly beyond the network (ahead of their chain, or as data mules), so
        their slot is kept while they are most likely still working. A
        silent relay, by contrast, is replaced immediately: its whole job is
        to be in contact."""
        out = []
        for uid, k in sorted(self.known.items()):
            task = self.tasks.get(uid, (None,))
            if not (k["lost"] and k["status"] == A.ACTIVE and task[0] == A.SURVEY):
                continue
            # Only a surveyor whose heartbeat showed it had applied THIS task
            # can be working on it out of range; otherwise the task never
            # reached it and the site is re-queued at once (evaluate.py E2
            # dev seed_007: H1 was held for a UAV that never received it).
            if not self.task_acknowledged(uid):
                continue
            if C.HP_UNEXPLAINED_SILENCE_REQUEUE and self._unexplained_silence(uid, task):
                continue
            if now - k["last_seen"] < self._expected_silence(k, task):
                out.append(uid)
        return out

    def _unexplained_silence(self, uid, task):
        """Optional variant (config.HP_UNEXPLAINED_SILENCE_REQUEUE): for a
        HIGH-PRIORITY site, a surveyor last heard well inside coverage (its
        last position within COMM_RANGE - 150 m of the GCS or of another UAV
        the GCS still hears) went silent without a distance explanation, so
        the site is re-queued at once. Cost: a possible duplicate survey."""
        p = self.pois.get(task[2])
        if p is None or p["prio"] <= 1:
            return False
        pos = self.known[uid]["pos"]
        others = [C.GCS_POS] + [k["pos"] for u, k in self.known.items()
                                if u != uid and not k["lost"]
                                and k["status"] in (A.ACTIVE, A.RTH)]
        near = min(math.dist(pos, o) for o in others)
        return near < C.COMM_RANGE - 150.0

    @staticmethod
    def _expected_silence(k, task):
        """How long a silent surveyor can plausibly still be working, from
        GCS knowledge only: fly from where it was last heard to the site,
        survey it, fly back to where it was last heard, plus slack. Capped at
        REQUEUE_AFTER_LOST. Beyond this, its site is re-queued (a duplicate
        survey is possible, never unsafe). (Before: always 240 s, which held
        a high-priority site for a UAV that had failed: evaluate.py E2 dev
        seed_007.)"""
        xy = task[1]
        if xy is None:
            return C.REQUEUE_AFTER_LOST
        leg = math.hypot(xy[0] - k["pos"][0], xy[1] - k["pos"][1]) / C.CRUISE_SPEED
        return min(C.REQUEUE_AFTER_LOST, 2 * leg + C.SURVEY_TIME + C.SILENT_SURVEY_SLACK)

    def margin(self, uid):
        """Battery % above what the UAV needs to get home with its reserve,
        from the last heartbeat."""
        k = self.known[uid]
        return k["battery"] - A.energy_to_home_from(uid, *k["pos"])

    def _dependants(self):
        """uav -> how many other linked UAVs route through it (known parents)."""
        live = {u for u, k in self.known.items() if not k["lost"] and k["status"] == A.ACTIVE}
        out = {}
        for u in live:
            p, seen = self.known[u].get("parent"), set()
            while p and p in self.known and p not in seen:
                seen.add(p)
                out[p] = out.get(p, 0) + 1
                p = self.known[p].get("parent")
        return out

    def _plan(self, now):
        pending = [pid for pid, p in self.pois.items() if p["delivered"] is None]
        if not pending:
            if self.complete_time is None:
                self.complete_time = now
                self.log(f"GCS: all {len(self.pois)} sites' imagery delivered at "
                         f"t={now:.1f}s -> recalling fleet")
            for uid in self.known:
                self._set_task(uid, (A.HOME, None, None), ("HOME",), now)
            reachable = [k for k in self.known.values() if not k["lost"]]
            if (self.fleet_confirmed_time is None and reachable
                    and all(k["status"] in (A.SERVICE, A.READY) for k in reachable)):
                self.fleet_confirmed_time = now
                gone = sorted(u for u, k in self.known.items() if k["lost"])
                self.log(f"GCS: every reachable UAV reports landed at t={now:.1f}s"
                         + (f"; unaccounted: {', '.join(f'UAV{u}' for u in gone)}" if gone else ""))
            for uid, rec in self.outgoing.items():
                rec.update(outcome="recalled: mission complete", t_released=round(now, 1),
                           release_reason="mission complete",
                           battery_released=round(self.known[uid]["battery"], 1))
            self.outgoing.clear()
            self.drain.clear()
            return

        usable = [uid for uid, k in sorted(self.known.items())
                  if not k["lost"] and k["status"] in (A.READY, A.ACTIVE)]
        # Relays whose energy margin is too thin start a handover; any other
        # low-margin UAV is simply recalled.
        low = [uid for uid in usable if self.known[uid]["status"] == A.ACTIVE
               and self.margin(uid) < C.HANDOVER_MARGIN]
        for uid in low:
            if uid not in self.outgoing and self.tasks.get(uid, (None,))[0] == A.RELAY:
                rec = {"uav": uid, "t_start": round(now, 1), "slot": self.task_keys.get(uid),
                       "xy": self.tasks[uid][1], "margin_start": round(self.margin(uid), 1),
                       "battery_start": round(self.known[uid]["battery"], 1),
                       "dependants": sorted(self._dependants_of(uid)),
                       "replacement": None, "replacements_tried": [], "t_dispatch": None,
                       "t_on_station": None, "t_bidirectional": None, "t_drain": None,
                       "t_downstream_ok": None, "t_hold_passed": None, "t_released": None,
                       "rollbacks": 0, "battery_released": None, "landed_battery": None,
                       "outcome": None, "release_reason": None, "checks": {},
                       "_hold_since": None}
                self.outgoing[uid] = rec
                self.handovers.append(rec)
                self.log(f"GCS: UAV{uid} relay energy margin {rec['margin_start']:.1f}% "
                         f"< {C.HANDOVER_MARGIN:.0f}% -> starting relay handover")
        pool_ok = [u for u in usable if u not in low]
        pinned = self._out_of_contact(now)
        gx, gy, _ = C.GCS_POS
        pending.sort(key=lambda pid: (-self.pois[pid]["prio"],
                                      math.hypot(self.pois[pid]["x"] - gx,
                                                 self.pois[pid]["y"] - gy)))

        slots = []                   # (key, task)
        anchors = [(gx, gy)]
        budget = len(pool_ok) + len(pinned)
        unplaced = []
        # Redundancy policy (see config.BACKUP_POLICY):
        #   reserve     : always hold one UAV as a backup beside a relay that
        #                 carries >= BACKUP_MIN_DEPENDANTS others;
        #   conditional : only when that relay carries >= CONDITIONAL_DEPENDANTS
        #                 (a large blast radius); otherwise like "spare";
        #   spare       : only with a UAV left over after every site is served.
        need = {"reserve": C.BACKUP_MIN_DEPENDANTS,
                "conditional": C.CONDITIONAL_DEPENDANTS}.get(C.BACKUP_POLICY)
        if need is not None:
            backup = self._backup_slot(need)
            if backup and budget > 1:
                slots.append(backup)
                budget -= 1
            elif backup:
                self._infeasible(now, "backup wanted but only one UAV is available")
        for pid in pending:
            p = self.pois[pid]
            ax, ay = min(anchors, key=lambda a: math.hypot(a[0] - p["x"], a[1] - p["y"]))
            d = math.hypot(p["x"] - ax, p["y"] - ay)
            n_relays = max(0, math.ceil(d / C.RELAY_SPACING) - 1)
            if n_relays + 1 > budget:
                if p["prio"] > 1 and budget > 0:
                    # A high-priority site that can't get a full chain is
                    # served NOW as a data-mule leg (surveyor + as many
                    # extension relays as the budget allows), before any
                    # lower-priority site can use up the fleet. (Before this,
                    # it waited for leftovers: priority inversion, found by
                    # tools/evaluate.py dev seed_023.)
                    for i in range(1, budget):
                        f = i * C.RELAY_SPACING / d
                        if f >= 1.0:
                            break
                        rxy = (ax + (p["x"] - ax) * f, ay + (p["y"] - ay) * f)
                        slots.append((("RELAY", pid, i), (A.RELAY, rxy, pid)))
                        anchors.append(rxy)
                        budget -= 1
                    slots.append((("SURVEY", pid), (A.SURVEY, (p["x"], p["y"]), pid)))
                    budget -= 1
                    continue
                unplaced.append(pid)
                continue
            for i in range(1, n_relays + 1):
                f = i / (n_relays + 1)
                rxy = (ax + (p["x"] - ax) * f, ay + (p["y"] - ay) * f)
                slots.append((("RELAY", pid, i), (A.RELAY, rxy, pid)))
                anchors.append(rxy)
            slots.append((("SURVEY", pid), (A.SURVEY, (p["x"], p["y"]), pid)))
            budget -= n_relays + 1
        if unplaced and budget:
            # Data-mule fallback for the most urgent unreachable PoI: one
            # surveyor goes out of range, and any spare UAVs extend the
            # network toward it at full spacing to shorten its trip back.
            pid = unplaced[0]
            p = self.pois[pid]
            ax, ay = min(anchors, key=lambda a: math.hypot(a[0] - p["x"], a[1] - p["y"]))
            d = math.hypot(p["x"] - ax, p["y"] - ay)
            for i in range(1, budget):
                f = i * C.RELAY_SPACING / d
                if f >= 1.0:
                    break
                slots.append((("RELAY", pid, i),
                              (A.RELAY, (ax + (p["x"] - ax) * f, ay + (p["y"] - ay) * f), pid)))
                budget -= 1
            slots.append((("SURVEY", pid), (A.SURVEY, (p["x"], p["y"]), pid)))
            budget -= 1
        if C.BACKUP_POLICY in ("spare", "conditional") and not any(
                k[0] == "BACKUP" for k, _ in slots):
            backup = self._backup_slot(1)
            if backup and budget > 0 and not unplaced:
                slots.append(backup)
            elif backup:
                self._infeasible(now, "no spare UAV for a backup relay "
                                      f"({len(unplaced)} site(s) still unserved)")

        best = self._assign(slots, pool_ok, pinned)
        while slots and best is None:
            # No energy-feasible assignment for every slot: drop the least
            # urgent slot and try again.
            slots = slots[:-1]
            best = self._assign(slots, pool_ok, [])
        best = best or ()

        # UAVs on their way home or charging get HOME so they don't relaunch
        # on a stale task the moment they're charged.
        for uid, k in self.known.items():
            if not k["lost"] and k["status"] in (A.RTH, A.CHARGING):
                self._set_task(uid, (A.HOME, None, None), ("HOME",), now)

        assigned = set()
        slot_of = {}
        for uid, (key, task) in zip(best, slots):
            if uid not in pinned:
                self._set_task(uid, task, key, now)
            assigned.add(uid)
            slot_of[key] = (uid, task)
        for uid in usable:
            if uid not in assigned and uid not in self.outgoing:
                if uid in low and self.tasks.get(uid, (None,))[0] != A.HOME:
                    self.log(f"GCS: UAV{uid} energy margin {self.margin(uid):.0f}% -> recalled")
                self._set_task(uid, (A.HOME, None, None), ("HOME",), now)
        self._handover(now, slot_of, {key for key, _ in slots})

    def _infeasible(self, now, why):
        """Record (once per change) that redundancy was wanted but not
        feasible: the mission runs degraded with a single point of failure."""
        if getattr(self, "_last_infeasible", None) != why:
            self._last_infeasible = why
            self.redundancy_infeasible.append((round(now, 1), why))

    def _backup_slot(self, min_dependants):
        """A BACKUP relay slot beside the busiest relay (the worst single
        point of failure the GCS can see), or None if nobody depends on one."""
        dep = self._dependants()
        if not dep:
            return None
        v = max(dep, key=lambda u: (dep[u], -u))
        if dep[v] < min_dependants or self.known[v].get("status") != A.ACTIVE:
            return None
        k = self.known[v]
        up = self.known.get(k.get("parent"), {}).get("pos", C.GCS_POS)
        vx, vy = k["pos"][0] - up[0], k["pos"][1] - up[1]
        n = math.hypot(vx, vy) or 1.0
        xy = A.clamp_to_fence(k["pos"][0] - vy / n * C.BACKUP_OFFSET,
                              k["pos"][1] + vx / n * C.BACKUP_OFFSET)
        self.backup_of = v
        return (("BACKUP", v), (A.RELAY, xy, f"BKP-U{v}"))

    def _assign(self, slots, usable, pinned):
        """Best UAV-to-slot assignment, or None if no energy-feasible one
        fills every slot. A pinned UAV can't be re-tasked (we can't reach
        it), so it may only take the slot it already has."""
        pool = usable + pinned
        if len(slots) > len(pool):
            return None
        best, best_cost = None, math.inf
        for perm in itertools.permutations(pool, len(slots)):
            cost = 0.0
            for uid, (key, task) in zip(perm, slots):
                if uid in pinned:
                    if self.task_keys.get(uid) != key:
                        break
                    continue
                k = self.known[uid]
                if self.task_keys.get(uid) == key:
                    cost -= STICKY_BONUS
                else:
                    alt = C.BASE_ALT + (uid - 1) * C.ALT_STEP
                    if k["battery"] < A.energy_for_slot(uid, k["pos"], task[1], alt):
                        break
                cost += math.hypot(k["pos"][0] - task[1][0], k["pos"][1] - task[1][1])
            else:
                if cost < best_cost:
                    best, best_cost = perm, cost
        return best

    def _dependants_of(self, v):
        """UAVs whose reported route (parent chain) passes through v."""
        out = set()
        for u, k in self.known.items():
            if u == v or k["lost"]:
                continue
            p, seen = k.get("parent"), set()
            while p and p in self.known and p not in seen:
                if p == v:
                    out.add(u)
                    break
                seen.add(p)
                p = self.known[p].get("parent")
        return out

    def _served(self, u, avoid, now):
        """GCS evidence that `u` is served without going through `avoid`: a
        fresh heartbeat whose reported route does not contain `avoid`."""
        k = self.known.get(u)
        if k is None or k["lost"] or now - k["last_seen"] > 1.5:
            return False
        p, seen = k.get("parent"), set()
        while p and p in self.known and p not in seen:
            if p == avoid:
                return False
            seen.add(p)
            p = self.known[p].get("parent")
        return True

    def _release(self, uid, rec, now, reason, outcome):
        k = self.known[uid]
        rec.update(t_released=round(now, 1), battery_released=round(k["battery"], 1),
                   release_reason=reason, outcome=outcome)
        self.log(f"GCS: UAV{uid} released home ({reason}; {k['battery']:.0f}%, needs "
                 f"{A.energy_to_home_from(uid, *k['pos']):.0f}% incl. reserve)")
        self._set_task(uid, (A.HOME, None, None), ("HOME",), now)
        self.drain.discard(uid)
        del self.outgoing[uid]

    def _handover(self, now, slot_of, planned):
        """Make-before-break relay handover, on GCS evidence only:

          dispatched -> on station -> bidirectional verified (the replacement's
          heartbeat shows it applied the command with this slot) -> outgoing
          drained (told to stop forwarding; its heartbeat confirms) ->
          downstream verified (every dependant sends fresh heartbeats over a
          route that avoids the outgoing relay) -> HANDOVER_HOLD s with all
          of that true -> released.

        Rollback: a dependant losing service while draining re-enables
        forwarding and restarts verification. Degraded modes: the outgoing
        relay's margin falls below HANDOVER_CRITICAL (it leaves anyway, never
        forced towards depletion); no replacement can afford the slot; the
        replacement fails (another is tried next plan); the slot disappears.
        Every release logs its reason and which checks had passed."""
        for uid in list(self.outgoing):
            rec = self.outgoing[uid]
            k = self.known.get(uid)
            if k is None or k["lost"] or k["status"] != A.ACTIVE:
                rec["outcome"] = ("ended early: " + ("lost contact" if k and k["lost"]
                                                     else str(k and k["status"])))
                rec["release_reason"] = "onboard decision or fault (no GCS release)"
                self.drain.discard(uid)
                del self.outgoing[uid]
                continue
            margin = self.margin(uid)
            if margin < C.HANDOVER_CRITICAL:
                rec["checks"] = self._checks(rec, now)
                self._release(uid, rec, now,
                              f"energy critical (margin {margin:.1f}% < {C.HANDOVER_CRITICAL:.0f}%)",
                              "degraded: released on energy before verification")
                continue
            if rec["slot"] not in planned:
                # The slot is gone from the plan (its site is done), but the
                # relay may still carry others: retire it make-before-break
                # (drain, verify every dependant is served without it, hold).
                self._retire(uid, rec, now)
                continue
            repl = slot_of.get(rec["slot"])
            if repl is None:
                if rec["checks"].get("degraded") is None:
                    rec["checks"]["degraded"] = "no UAV can afford the slot; holding"
                    self.log(f"GCS: no replacement can afford UAV{uid}'s slot -> "
                             f"UAV{uid} holds until margin < {C.HANDOVER_CRITICAL:.0f}%")
                self.drain.discard(uid)
                continue
            ruid, task = repl
            if rec["replacement"] != ruid:
                if rec["replacement"] is not None:
                    self.log(f"GCS: replacement UAV{rec['replacement']} unavailable -> "
                             f"UAV{ruid} dispatched instead")
                rec["replacement"] = ruid
                rec["replacements_tried"].append(ruid)
                rec["t_dispatch"] = round(now, 1)
                for f in ("t_on_station", "t_bidirectional", "t_drain", "t_downstream_ok"):
                    rec[f] = None
                self.drain.discard(uid)
                self.log(f"GCS: UAV{ruid} dispatched to relieve UAV{uid} "
                         f"({self.known[ruid]['battery']:.0f}% battery)")
            if now - (rec["t_dispatch"] or now) > C.HANDOVER_TIMEOUT \
                    and rec["checks"].get("timeout") is None:
                rec["checks"]["timeout"] = round(now, 1)
                self.log(f"GCS: handover of UAV{uid} not verified after "
                         f"{C.HANDOVER_TIMEOUT:.0f}s -> degraded, holding")
            ch = self._checks(rec, now)
            rec["checks"].update(ch)
            if ch["on_station"] and rec["t_on_station"] is None:
                rec["t_on_station"] = round(now, 1)
            if ch["bidirectional"] and rec["t_bidirectional"] is None:
                rec["t_bidirectional"] = round(now, 1)
            if ch["on_station"] and ch["bidirectional"]:
                if uid not in self.drain:
                    self.drain.add(uid)
                    rec["_drain_start"] = now
                    rec["_must_serve"] = [d for d in rec["dependants"]
                                          if self._served(d, -1, now)]
                    rec["checks"]["exempt_silent_before_drain"] = sorted(
                        set(rec["dependants"]) - set(rec["_must_serve"]))
                    self.log(f"GCS: UAV{ruid} on station, traffic verified -> UAV{uid} "
                             "told to stop forwarding (verifying downstream)")
                if ch["drained"] and rec["t_drain"] is None:
                    rec["t_drain"] = round(now, 1)
            ok = ch["on_station"] and ch["bidirectional"] and ch["drained"] and ch["downstream"]
            if ok:
                if rec["t_downstream_ok"] is None:
                    rec["t_downstream_ok"] = round(now, 1)
                if rec["_hold_since"] is None:
                    rec["_hold_since"] = now
                if now - rec["_hold_since"] >= C.HANDOVER_HOLD:
                    rec["t_hold_passed"] = round(now, 1)
                    self._release(uid, rec, now, "verified: replacement on station, traffic both "
                                  f"ways, {len(ch['served'])} dependant(s) served, "
                                  f"{C.HANDOVER_HOLD:.0f}s hold", "relieved on station (verified)")
            else:
                if rec["_hold_since"] is not None or (
                        uid in self.drain and ch["drained"] and not ch["downstream"]
                        and now - rec.get("_drain_start", now) > 4.0):
                    # A dependant lost service while the relay was drained: roll back.
                    rec["rollbacks"] += 1
                    self.drain.discard(uid)
                    rec["_hold_since"] = None
                    rec["t_downstream_ok"] = None
                    self.log(f"GCS: downstream not served without UAV{uid} -> rollback "
                             f"(forwarding re-enabled, #{rec['rollbacks']})")

    def _retire(self, uid, rec, now):
        if rec["checks"].get("retiring") is None:
            rec["checks"]["retiring"] = round(now, 1)
            rec["_hold_since"] = None
            self.log(f"GCS: UAV{uid}'s slot is no longer needed -> draining it, verifying its "
                     "dependants are served without it")
        self.drain.add(uid)
        k = self.known[uid]
        if "_must_serve" not in rec:
            rec["_must_serve"] = [d for d in rec["dependants"] if self._served(d, -1, now)]
            rec["checks"]["exempt_silent_before_drain"] = sorted(
                set(rec["dependants"]) - set(rec["_must_serve"]))
        deps = [d for d in rec["_must_serve"] if d in self.known
                and self.known[d]["status"] == A.ACTIVE]
        served = [d for d in deps if self._served(d, uid, now)]
        drained = k.get("forwarding") is False and now - k["last_seen"] < 1.5
        rec["checks"].update(drained=drained, downstream=len(served) == len(deps),
                             served=served, dependants_active=deps)
        if drained and len(served) == len(deps):
            if rec["_hold_since"] is None:
                rec["_hold_since"] = now
                rec["t_drain"] = rec["t_drain"] or round(now, 1)
                rec["t_downstream_ok"] = round(now, 1)
            if now - rec["_hold_since"] >= C.HANDOVER_HOLD:
                rec["t_hold_passed"] = round(now, 1)
                self._release(uid, rec, now, f"slot no longer needed; {len(served)} dependant(s) "
                              f"served without it for {C.HANDOVER_HOLD:.0f}s",
                              "retired: slot no longer needed (verified)")
        else:
            rec["_hold_since"] = None

    def _checks(self, rec, now):
        uid, ruid = rec["uav"], rec["replacement"]
        k = self.known.get(uid)
        rk = self.known.get(ruid) if ruid else None
        xy = rec["xy"]
        on_station = bool(rk and not rk["lost"] and rk["status"] == A.ACTIVE
                          and math.hypot(rk["pos"][0] - xy[0], rk["pos"][1] - xy[1])
                          < C.HANDOVER_RANGE)
        bidir = bool(rk and now - rk["last_seen"] < 1.5 and self.task_acknowledged(ruid))
        drained = bool(k and k.get("forwarding") is False and now - k["last_seen"] < 1.5)
        must = rec.get("_must_serve", rec["dependants"])
        deps = [d for d in must if d in self.known
                and self.known[d]["status"] == A.ACTIVE and d != ruid]
        served = [d for d in deps if self._served(d, uid, now)]
        return {"on_station": on_station, "bidirectional": bidir, "drained": drained,
                "downstream": len(served) == len(deps), "served": served,
                "dependants_active": deps}

    def _set_task(self, uid, task, key, now=None):
        if self.tasks.get(uid) == task:
            return
        old = self.tasks.get(uid)
        self.tasks[uid] = task
        self.task_keys[uid] = key
        if old is None or (old[0], old[2]) != (task[0], task[2]):
            what = {A.RELAY: "RELAY", A.SURVEY: "SURVEY", A.HOME: "HOME"}.get(task[0], task[0])
            tgt = f" {task[2]}" if task[2] else ""
            self.log(f"GCS: task UAV{uid} -> {what}{tgt}")


class FixedRelayStation(GroundStation):
    """Baseline for tools/evaluate.py: fixed relay assignment.

    At the first plan, a static backbone of relays is laid from the GCS
    toward the centre of the area at RELAY_SPACING and each slot is given to
    one UAV for the whole mission. Relays are never re-assigned: a failed
    relay leaves a hole, and a relay that goes home to recharge returns to
    the same slot. Every other UAV surveys pending sites one at a time (most
    urgent first) and carries its data back into backbone range. Same UAVs,
    same onboard safety, same network: only the GCS planning differs.
    """

    N_BACKBONE = 2

    def _plan(self, now):
        pending = [pid for pid, p in self.pois.items() if p["delivered"] is None]
        if not pending:
            return GroundStation._plan(self, now)
        usable = [uid for uid, k in sorted(self.known.items())
                  if not k["lost"] and k["status"] in (A.READY, A.ACTIVE)]
        if not hasattr(self, "backbone"):
            if len(usable) < self.N_BACKBONE + 1:
                return
            gx, gy, _ = C.GCS_POS
            d = math.hypot(gx, gy)
            slots = [(gx - gx / d * C.RELAY_SPACING * i, gy - gy / d * C.RELAY_SPACING * i)
                     for i in range(1, self.N_BACKBONE + 1)]
            self.backbone = dict(zip(usable[:self.N_BACKBONE], slots))
        for uid, xy in self.backbone.items():
            k = self.known.get(uid)
            if k and not k["lost"] and k["status"] in (A.READY, A.ACTIVE):
                self._set_task(uid, (A.RELAY, xy, "BB"), ("BB", uid), now)
        for uid, k in self.known.items():
            if not k["lost"] and k["status"] in (A.RTH, A.CHARGING) and uid not in self.backbone:
                self._set_task(uid, (A.HOME, None, None), ("HOME",), now)
        gx, gy, _ = C.GCS_POS
        pending.sort(key=lambda pid: (-self.pois[pid]["prio"],
                                      math.hypot(self.pois[pid]["x"] - gx, self.pois[pid]["y"] - gy)))
        taken = {self.tasks[u][2] for u in usable
                 if u in self.tasks and self.tasks[u][0] == A.SURVEY and self.tasks[u][2] in pending}
        for uid in usable:
            if uid in self.backbone:
                continue
            cur = self.tasks.get(uid)
            if cur and cur[0] == A.SURVEY and cur[2] in pending:
                continue
            free = [pid for pid in pending if pid not in taken]
            if not free:
                self._set_task(uid, (A.HOME, None, None), ("HOME",), now)
                continue
            k = self.known[uid]
            pid = free[0]
            p = self.pois[pid]
            alt = C.BASE_ALT + (uid - 1) * C.ALT_STEP
            if k["battery"] < A.energy_for_slot(uid, k["pos"], (p["x"], p["y"]), alt):
                self._set_task(uid, (A.HOME, None, None), ("HOME",), now)
                continue
            taken.add(pid)
            self._set_task(uid, (A.SURVEY, (p["x"], p["y"]), pid), ("SURVEY", pid), now)
