"""Onboard autonomy for a single UAV.

A UAV only acts on what it knows locally: its own position and battery, the
last task it received from the GCS, and how long ago it last heard the GCS.
Safety decisions (return-to-home on low battery, lost-link recovery,
geofence clamping, failsafe emergency landing) are taken onboard and
override any GCS task.
"""

import math

from . import config as C

# Flight status
READY = "READY"          # on the pad, fresh pack, waiting for a task
ACTIVE = "ACTIVE"        # airborne, executing a task
RTH = "RTH"              # returning home on low battery (onboard decision)
SERVICE = "SERVICE"      # on the pad, battery swap in progress (SWAP_TIME)
CHARGING = SERVICE       # legacy name
EMERGENCY = "EMERGENCY"  # critical fault: failsafe controlled descent
FAILED = "FAILED"        # landed after a critical fault, powered down

# Task kinds sent by the GCS
SURVEY = "SURVEY"
RELAY = "RELAY"
HOLD = "HOLD"
HOME = "HOME"

N_CHUNKS = math.ceil(C.SURVEY_DATA_MB * 1e6 / C.CHUNK_BYTES)


def clamp_to_fence(x, y, margin=C.FENCE_MARGIN):
    xmin, xmax, ymin, ymax = C.GEOFENCE
    return (min(max(x, xmin + margin), xmax - margin),
            min(max(y, ymin + margin), ymax - margin))


def pad_position(uav_id):
    gx, gy, _ = C.GCS_POS
    # Row of pads just north-east of the GCS building.
    return (gx + 10.0 + (uav_id - 1) * C.PAD_SPACING, gy + 12.0, C.PAD_Z)


def energy_to_home_from(uav_id, x, y, z):
    """Battery % to fly from (x, y, z) to the pad and land, plus the reserve.
    Used onboard and by the GCS (with the last position it heard)."""
    px, py, _ = pad_position(uav_id)
    t = math.hypot(x - px, y - py) / C.CRUISE_SPEED + z / C.CLIMB_SPEED
    return t * C.DRAIN_CRUISE + C.RESERVE


def energy_for_slot(uav_id, pos, slot_xy, cruise_alt):
    """Battery % a UAV at `pos` needs to take a slot: transit + on-station
    budget + return from the slot + reserve (the GCS assignment constraint)."""
    climb = max(0.0, cruise_alt - pos[2]) / C.CLIMB_SPEED
    transit = math.hypot(slot_xy[0] - pos[0], slot_xy[1] - pos[1]) / C.CRUISE_SPEED
    e = (climb + transit) * C.DRAIN_CRUISE + C.ON_STATION_BUDGET * C.DRAIN_HOVER
    return e + energy_to_home_from(uav_id, slot_xy[0], slot_xy[1], cruise_alt)


class Uav:
    def __init__(self, uav_id, battery=100.0):
        self.id = uav_id
        self.pos = list(pad_position(uav_id))
        self.cruise_alt = C.BASE_ALT + (uav_id - 1) * C.ALT_STEP
        self.battery = battery
        self.status = SERVICE if battery < C.READY_LEVEL else READY
        self.service_left = C.SWAP_TIME if self.status == SERVICE else 0.0
        self.radio_ok = True
        self.task = (HOLD, None, None)       # (kind, (x, y), poi_id)
        self.forwarding = True               # False while the GCS drains this relay
        self.last_heard_gcs = 0.0            # sim time of the last packet FROM the GCS
        self.survey_timer = 0.0
        self.data = set()                    # PoIs surveyed, not yet confirmed by a GCS command
        self.acked = {}                      # PoI -> set of chunk indices acked end to end
        self.outstanding = {}                # chunk key -> sim time sent (awaiting ack)
        self.detections = {}                 # PoI -> persons found onboard (set by perception)
        self.surveyed = set()                # PoIs this UAV has ever surveyed
        self.mule = False                    # carrying imagery back until it is acked
        self.hb_seq = 0
        self.cmd_seq = 0                     # last command applied
        self.cmd_stale = 0                   # commands discarded: older seq / duplicate
        self.cmd_expired = 0                 # commands discarded: past their expiry
        self.avoid = (0.0, 0.0)              # repulsion from nearby UAVs, set by the sim
        self.hold_vertical = False           # pause climb/descent, set by the sim
        self.clear_of = []                   # (x, y) of nearby climbing/descending UAVs, set by the sim
        self.back_off = False                # reverse vertical motion to restore separation, set by the sim
        self.vdir = 0.0
        self.moving = False
        self.depleted = False
        self.keepout = []                    # (x, y) of failsafe descents heard nearby, set by the sim
        self.mesh_tasks = []                 # task kinds heard from UAVs in radio reach (beacons), set by the sim
        self.cmd_rx = 0                      # command packets applied (all via the network)
        self.landings = []                   # (t, battery %) at each pad landing
        self.log = None                      # callable(msg), injected by the sim
        self.ground = lambda x, y: 0.0       # terrain height, injected by the sim

    # ------------------------------------------------------------------ helpers
    @property
    def airborne(self):
        return self.status in (ACTIVE, RTH, EMERGENCY)

    def _say(self, msg):
        if self.log:
            self.log(f"UAV{self.id}: {msg}")

    def energy_to_home(self):
        """Battery % needed to fly home and land from here, plus reserve."""
        return energy_to_home_from(self.id, *self.pos)

    def heartbeat(self, hops, parent):
        """Telemetry packet: only what this UAV itself knows."""
        self.hb_seq += 1
        return {"id": self.id, "seq": self.hb_seq, "pos": tuple(self.pos),
                "battery": self.battery, "status": self.status, "data": set(self.data),
                "task": self.task, "hops": hops, "parent": parent,
                "cmd_seq": self.cmd_seq, "forwarding": self.forwarding}

    # ------------------------------------------------------------ imagery
    def unacked(self, pid):
        return N_CHUNKS - len(self.acked.get(pid, ()))

    def next_chunk(self):
        """(PoI, chunks not yet acked) of the imagery still to deliver, or None."""
        for pid in sorted(self.data):
            left = self.unacked(pid)
            if left > 0:
                return pid, left
        return None

    def chunks_to_send(self, now):
        """Chunks to put on the air now: RTO expiries first (retransmissions),
        then new chunks while the window has room. -> [(key, retransmit)]"""
        out = []
        for key, t in list(self.outstanding.items()):
            if now - t >= C.CHUNK_RTO:
                self.outstanding[key] = now
                out.append((key, True))
        for pid in sorted(self.data):
            acked = self.acked.setdefault(pid, set())
            for idx in range(N_CHUNKS):
                if len(self.outstanding) >= C.DATA_WINDOW:
                    return out
                key = (self.id, pid, idx)
                if idx in acked or key in self.outstanding:
                    continue
                self.outstanding[key] = now
                out.append((key, False))
        return out

    def on_chunk_ack(self, key, now):
        """End-to-end ack from the GCS: proves contact, frees the window."""
        self.last_heard_gcs = now
        self.outstanding.pop(key, None)
        self.acked.setdefault(key[1], set()).add(key[2])

    # ----------------------------------------------------------- commands
    def receive_command(self, cmd, now):
        """Apply a command packet if it is newer than the last one applied
        and not past its expiry. Returns True if applied."""
        if now > cmd["expiry"]:
            self.cmd_expired += 1
            return False
        if cmd["seq"] <= self.cmd_seq:
            self.cmd_stale += 1
            return False
        self.cmd_seq = cmd["seq"]
        self.cmd_rx += 1
        self.last_heard_gcs = now
        self.data -= cmd["delivered"]
        self.forwarding = not cmd.get("drain", False)
        task = cmd["task"]
        if task is not None and task != self.task:
            self.survey_timer = 0.0
            self.task = task
        return True

    def receive_task(self, task, delivered, now):
        """Convenience for unit tests: apply an always-fresh command."""
        return self.receive_command({"seq": self.cmd_seq + 1, "expiry": now + 1.0,
                                     "task": task, "delivered": delivered}, now)

    def fail(self, hard=False):
        """Critical fault (abstracted, e.g. a motor/ESC failure). The flight
        controller's failsafe takes over: the UAV stops its task, levels off
        and makes a controlled vertical descent where it is. No rotor-out
        dynamics are modelled.

        failsafe (hard=False): the radio keeps working, so the next heartbeat
        tells the GCS. hard=True: the fault also takes the radio down (power
        bus), so the GCS can only find out by heartbeat timeout."""
        if self.status in (ACTIVE, RTH):
            self.status = EMERGENCY
            if hard:
                self.radio_ok = False
            self._say("CRITICAL FAULT -> failsafe emergency landing"
                      + (", radio lost" if hard else ""))

    # --------------------------------------------------------------------- step
    def step(self, now, dt):
        if self.status == FAILED:
            self.moving = False
            return
        if self.status == EMERGENCY:
            floor = self.ground(self.pos[0], self.pos[1]) + C.PAD_Z
            self.pos[2] = max(floor, self.pos[2] - C.EMERGENCY_DESCENT * dt)
            self.battery = max(0.0, self.battery - C.DRAIN_HOVER * dt)
            self.moving = False
            if self.pos[2] <= floor + 1e-6:
                self.status = FAILED
                self._say("emergency landing complete, powered down")
            return

        if self.status in (READY, SERVICE):
            self._on_pad(dt)
            return

        # Airborne: onboard safety checks first.
        if self.status == ACTIVE and self.battery <= self.energy_to_home() + 5.0:
            self.status = RTH
            self._say(f"battery {self.battery:.0f}% -> returning home")

        target = self._keep_clear(self._choose_target(now, dt))
        self._fly_to(target, dt)

        drain = C.DRAIN_CRUISE if self.moving else C.DRAIN_HOVER
        self.battery = max(0.0, self.battery - drain * dt)
        if self.battery <= 0.0 and not self.depleted:
            self.depleted = True
            self._say("BATTERY DEPLETED")

        # Landed?
        pad = pad_position(self.id)
        if self.status == RTH and self._at(pad, 0.3) and self.pos[2] <= C.PAD_Z + 0.05:
            self.status = SERVICE
            self.service_left = C.SWAP_TIME
            self.forwarding = True
            self.landings.append((now, self.battery))
            self._say(f"landed with {self.battery:.0f}%, battery swap started")

    def _on_pad(self, dt):
        self.moving = False
        if self.status == SERVICE:
            self.service_left -= dt
            if self.service_left <= 0.0:
                self.battery = 100.0
                self.status = READY
                self._say("battery swapped, ready")
        launch = (self.status == READY and self.task[0] in (SURVEY, RELAY)
                  and self.battery > self.energy_to_home() + 20.0)
        if launch:
            self.status = ACTIVE
            self._say(f"launching for {self.task[0]} {self.task[2] or ''}".rstrip())

    def _choose_target(self, now, dt):
        """Pick where to fly this step. Returns (x, y, z)."""
        pad = pad_position(self.id)
        if self.status == RTH:
            return pad

        silent = now - self.last_heard_gcs
        kind, xy, poi = self.task
        toward_gcs = (C.GCS_POS[0] + 8.0, C.GCS_POS[1] + 8.0, self.cruise_alt)

        # Lost-link recovery (onboard, from local knowledge only):
        #  * DATA MULE, with hysteresis: holding imagery the GCS hasn't acked
        #    and silent for LOST_LINK_WITH_DATA -> carry it back, and keep
        #    carrying it back until every chunk is acked. (Before: hearing a
        #    single command reset the silence timer and sent the UAV straight
        #    back out of range, so it dithered on the range edge; see
        #    docs/OUTAGE_LEDGER.md, cause "range-boundary dither".)
        #  * a relay that can't hear the GCS is useless where it is, so it
        #    falls back toward the GCS to rejoin the mesh;
        #  * any UAV that can still hear an orphaned relay (neighbour beacons,
        #    same fragment, no route) falls back with it, so the fragment
        #    contracts toward the GCS as one piece;
        #  * task done (surveyed, all imagery acked): out of contact -> come
        #    back; in contact -> hold where it is until re-tasked;
        #  * silent for too long -> come back regardless.
        pending = self.next_chunk() is not None
        if pending and silent > C.LOST_LINK_WITH_DATA and not self.mule:
            self.mule = True
            self._say("carrying imagery back (data mule)")
        if self.mule and not pending:
            self.mule = False
            self._say("imagery acked, resuming task")
        if self.mule and not C.MULE_HYSTERESIS and silent <= C.LOST_LINK_WITH_DATA:
            self.mule = False            # pre-fix behaviour, kept only for before/after runs
        done = kind == SURVEY and poi in self.surveyed and not pending
        fragment_falls_back = silent > C.LOST_LINK_RELAY and any(
            k == RELAY for k in self.mesh_tasks)
        if (self.mule
                or (kind == RELAY and silent > C.LOST_LINK_RELAY)
                or fragment_falls_back
                or (done and silent > C.LOST_LINK_WITH_DATA)
                or silent > C.LOST_LINK_ABORT):
            return toward_gcs
        if done:
            return (self.pos[0], self.pos[1], self.cruise_alt)

        if kind == HOME:
            if self._at(pad, 0.5):
                self.status = RTH       # reuse the landing logic
            return pad
        if kind == SURVEY and poi not in self.surveyed:
            x, y = clamp_to_fence(*self.survey_point(xy))
            if self._at((x, y), C.SURVEY_RADIUS):
                self.survey_timer += dt
                if self.survey_timer >= C.SURVEY_TIME:
                    self.surveyed.add(poi)
                    self.data.add(poi)
                    self._say(f"surveyed {poi}")
            return (x, y, self.cruise_alt)
        if kind == SURVEY and xy is not None:
            x, y = clamp_to_fence(*self.survey_point(xy))
            return (x, y, self.cruise_alt)
        if kind == RELAY and xy is not None:
            x, y = clamp_to_fence(*xy)
            return (x, y, self.cruise_alt)
        # HOLD: stay where we are (still useful as an opportunistic relay).
        return (self.pos[0], self.pos[1], self.cruise_alt)

    def _keep_clear(self, target):
        """Never aim into the descent column of a UAV in a failsafe landing
        (heard in its beacon): hold at the edge of the keep-out instead."""
        tx, ty, tz = target
        for ex, ey in self.keepout:
            d = math.hypot(tx - ex, ty - ey)
            if d < C.EMERGENCY_KEEPOUT + 5.0:
                if d < 1e-3:
                    ux, uy = self.pos[0] - ex, self.pos[1] - ey
                    n = math.hypot(ux, uy) or 1.0
                    ux, uy = (ux / n, uy / n) if n > 1e-3 else (1.0, 0.0)
                else:
                    ux, uy = (tx - ex) / d, (ty - ey) / d
                r = C.EMERGENCY_KEEPOUT + 5.0
                tx, ty = ex + ux * r, ey + uy * r
        return (tx, ty, tz)

    def survey_point(self, poi_xy):
        """Hover point for surveying a PoI: set back from it toward the GCS so
        the gimbal camera sees the site obliquely (people are far easier to
        detect from 50-60 deg than straight down) and the hover sits a little
        closer to the relay network."""
        px, py = poi_xy
        gx, gy, _ = C.GCS_POS
        d = math.hypot(gx - px, gy - py) or 1.0
        s = C.SURVEY_STANDOFF * self.cruise_alt
        return (px + (gx - px) / d * s, py + (gy - py) / d * s)

    def _at(self, xy, tol):
        return math.hypot(self.pos[0] - xy[0], self.pos[1] - xy[1]) <= tol

    def _fly_to(self, target, dt):
        """Kinematic flight: climb to the cruise layer, transit, then descend.

        Horizontal and vertical legs are separated so UAVs only change
        altitude directly above their own pad or target, which keeps the
        altitude layers clean.
        """
        tx, ty, tz = target
        dx, dy = tx - self.pos[0], ty - self.pos[1]
        dh = math.hypot(dx, dy)
        start = tuple(self.pos)

        if dh > 0.5 and abs(self.pos[2] - self.cruise_alt) > 0.5:
            # Get to our layer before moving sideways.
            tz_now = self.cruise_alt
            dh_step = 0.0
        else:
            tz_now = tz if dh <= 0.5 else self.cruise_alt
            dh_step = min(dh, C.CRUISE_SPEED * dt)

        if dh > 1e-6 and dh_step > 0:
            nx = self.pos[0] + dx / dh * dh_step
            ny = self.pos[1] + dy / dh * dh_step
            # Don't close in horizontally on a UAV that is changing altitude
            # through our layer (moving away from it is fine).
            closing = any(math.hypot(nx - cx, ny - cy) < math.hypot(self.pos[0] - cx,
                                                                      self.pos[1] - cy)
                          for cx, cy in self.clear_of)
            if not closing:
                self.pos[0], self.pos[1] = nx, ny
        # Collision-avoidance nudge (normally zero thanks to altitude layers).
        self.pos[0] += self.avoid[0] * dt
        self.pos[1] += self.avoid[1] * dt

        self.vdir = tz_now - self.pos[2]     # intended vertical direction (for yielding)
        dz = 0.0 if self.hold_vertical else self.vdir
        if self.back_off and abs(self.vdir) > 0.5:
            dz = -math.copysign(C.CLIMB_SPEED * dt, self.vdir)
        self.pos[2] += max(-C.CLIMB_SPEED * dt, min(C.CLIMB_SPEED * dt, dz))
        self.pos[2] = min(self.pos[2], C.ALT_CEILING)

        # Hard geofence guard (margin 0: the fence itself).
        self.pos[0], self.pos[1] = clamp_to_fence(self.pos[0], self.pos[1], 0.0)

        moved = math.dist(start, self.pos)
        self.moving = moved > 0.2 * C.CRUISE_SPEED * dt
