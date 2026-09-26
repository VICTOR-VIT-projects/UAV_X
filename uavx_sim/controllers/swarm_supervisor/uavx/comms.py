"""Radio links, multi-hop routing and a hop-by-hop shared-channel model.

Links (radio profile, config.RADIO_PROFILE):
  * "simple" (baseline): a link exists iff 3D distance <= COMM_RANGE; each
    hop attempt is lost with p(d) = LOSS_BASE + LOSS_EDGE * (d / COMM_RANGE)^4.
  * "degraded": the same law with a shorter range and a steeper edge.
  * "shadowed": "simple" plus a terrain line-of-sight check: a link whose
    straight line passes below terrain is blocked, and one with less than
    SHADOW_CLEARANCE of clearance gets SHADOW_LOSS extra loss. This is an
    uncalibrated stress case, not a propagation model.

Routing: Dijkstra from the GCS with ETX = 1/(1-p) per hop (minimum-ETX
route). A node listed in `no_forward` (a relay being drained before release)
can still send and receive its own traffic but is never used as an
intermediate hop.

Channel (class Channel), an abstraction of a single-channel CSMA mesh:
  * Store-and-forward: every packet waits in the queue of the node that
    holds it and is forwarded to the next hop of the routing table at the
    moment it is sent (routes can change while it travels).
  * Shared airtime: a transmission of B bytes occupies the medium for
    B*8/LINK_RATE + MAC_OVERHEAD seconds at the sender AND at every node in
    radio range of it (carrier sense). Nodes out of each other's range can
    transmit at the same time (spatial reuse). No hidden-terminal model.
  * Per-hop MAC retries: a failed hop attempt is retried up to MAC_RETRIES
    times; each attempt costs airtime and counts as a transmission.
  * Queues: strict priority (control > thumbnail > imagery), QUEUE_LIMIT
    packets per node (tail drop), and per-class expiry: a heartbeat, command
    or thumbnail older than its deadline is discarded unsent.
  * Latency = queueing + airtime + HOP_LATENCY per hop, all emergent.
Loss draws come from a counter-based hash of (seed, link, 64 ms slot,
attempt), not from a shared RNG stream, so two planners that use the same
link at the same time see the same channel realisation.

Accounting (per class, unique packets vs transmissions):
  generated -> delivered | expired | dropped (queue full) | lost (MAC retries
  exhausted) | pending (still queued at the end). hop_tx counts every
  transmission attempt on every hop, retries included.
  PDR (cohort) = delivered within the class deadline / generated, over
  packets whose deadline has passed ("matured"), so a late receipt is never
  divided by unrelated sends. Empty cohort -> None (N/A).
"""

import bisect
import collections
import heapq
import itertools
import math

from . import config as C

GCS_ID = 0
CLASSES = ("hb", "cmd", "ack", "data", "thumb")
PRIORITY = {"cmd": 0, "hb": 0, "ack": 0, "thumb": 1, "data": 2}
MASK64 = (1 << 64) - 1


def dist(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def radio_params(profile=None):
    p = profile or C.RADIO_PROFILE
    if p == "degraded":
        return C.DEGRADED_RANGE, C.LOSS_BASE, C.DEGRADED_LOSS_EDGE
    return C.COMM_RANGE, C.LOSS_BASE, C.LOSS_EDGE


def loss_prob(d, profile=None):
    rng, base, edge = radio_params(profile)
    if d > rng:
        return 1.0
    return min(0.99, base + edge * (d / rng) ** 4)


def _mix(x):
    """splitmix64 finaliser -> float in [0, 1)."""
    x = (x + 0x9E3779B97F4A7C15) & MASK64
    x = ((x ^ (x >> 30)) * 0xBF58476D1CE4E5B9) & MASK64
    x = ((x ^ (x >> 27)) * 0x94D049BB133111EB) & MASK64
    x ^= x >> 31
    return (x >> 11) / float(1 << 53)


class Network:
    """Link graph + minimum-ETX routing tree toward the GCS."""

    def __init__(self, seed=0, ground=None, profile=None):
        self.seed = seed
        self.ground = ground
        self.profile = profile or C.RADIO_PROFILE
        self.links = {}    # node -> {neighbour: loss probability}
        self.parent = {}   # node -> next hop toward the GCS
        self.hops = {}     # node -> hop count to the GCS
        self.no_forward = set()

    def _shadow(self, a, b):
        """Extra loss (or 1.0 = blocked) from terrain on the a-b line."""
        if self.profile != "shadowed" or self.ground is None:
            return 0.0
        worst = math.inf
        for k in range(1, 12):
            f = k / 12.0
            x = a[0] + (b[0] - a[0]) * f
            y = a[1] + (b[1] - a[1]) * f
            z = a[2] + (b[2] - a[2]) * f
            worst = min(worst, z - self.ground(x, y))
        if worst < 0.0:
            return 1.0
        if worst < C.SHADOW_CLEARANCE:
            return C.SHADOW_LOSS * (1.0 - worst / C.SHADOW_CLEARANCE)
        return 0.0

    def update(self, positions, active, no_forward=()):
        """Rebuild the link graph and routing tree.

        positions: {node_id: (x, y, z)}; active: node ids whose radio is up.
        """
        self.no_forward = set(no_forward)
        rng, _, _ = radio_params(self.profile)
        ids = sorted(n for n in active if n in positions)
        self.links = {n: {} for n in ids}
        for i, a in enumerate(ids):
            for b in ids[i + 1:]:
                d = dist(positions[a], positions[b])
                if d <= rng:
                    p = loss_prob(d, self.profile)
                    s = self._shadow(positions[a], positions[b])
                    p = min(1.0, p + s)
                    if p < 0.99:
                        self.links[a][b] = p
                        self.links[b][a] = p

        self.parent, self.hops = {}, {}
        if GCS_ID not in self.links:
            return
        cost = {GCS_ID: 0.0}
        self.hops[GCS_ID] = 0
        heap = [(0.0, GCS_ID)]
        while heap:
            c, n = heapq.heappop(heap)
            if c > cost[n]:
                continue
            if n != GCS_ID and n in self.no_forward:
                continue                         # reachable, but carries no one else
            for m, p in self.links[n].items():
                nc = c + 1.0 / (1.0 - p)
                if nc < cost.get(m, math.inf):
                    cost[m] = nc
                    self.parent[m] = n
                    self.hops[m] = self.hops[n] + 1
                    heapq.heappush(heap, (nc, m))

    def connected(self, n):
        return n in self.hops

    def route(self, n):
        """Node list from n to the GCS, or None if n is cut off."""
        if n not in self.hops:
            return None
        path = [n]
        while path[-1] != GCS_ID:
            path.append(self.parent[path[-1]])
        return path

    def next_hop(self, at, dst):
        """Next node from `at` toward `dst` (GCS or a UAV), or None."""
        if at == dst:
            return None
        if dst == GCS_ID:
            return self.parent.get(at)
        path = self.route(dst)                   # dst ... GCS
        if path is None:
            return None
        down = path[::-1]                        # GCS ... dst
        if at not in down:
            return None
        return down[down.index(at) + 1]

    def path_success(self, n):
        path = self.route(n)
        if path is None:
            return 0.0
        q = 1.0
        for a, b in zip(path, path[1:]):
            q *= 1.0 - self.links[a][b]
        return q

    def forwarders(self):
        """UAVs currently relaying traffic for at least one other UAV."""
        return {p for n, p in self.parent.items() if p != GCS_ID}

    def tree_edges(self):
        return {tuple(sorted((n, p))) for n, p in self.parent.items()}

    def all_edges(self):
        return {tuple(sorted((a, b))) for a in self.links for b in self.links[a]}

    def critical_relays(self, among):
        """{node: dependants} for every UAV whose loss alone would cut at
        least one node of `among` (that currently has a route) off from the
        GCS. Uses the undirected link graph with the same no-forward rule as
        routing (articulation points restricted to mission UAVs)."""
        out = {}
        reach = set(self.hops)
        for v in list(reach):
            if v == GCS_ID:
                continue
            seen = {GCS_ID}
            stack = [GCS_ID]
            while stack:
                n = stack.pop()
                if n != GCS_ID and n in self.no_forward:
                    continue
                for m in self.links.get(n, ()):
                    if m != v and m not in seen:
                        seen.add(m)
                        stack.append(m)
            cut = [n for n in among if n in reach and n != v and n not in seen]
            if cut:
                out[v] = len(cut)
        return out

    # Backwards-compatible name.
    single_points_of_failure = critical_relays


class Packet:
    __slots__ = ("uid", "cls", "src", "dst", "size", "t_gen", "deadline", "payload",
                 "at", "attempt", "hops_done", "key", "ready")

    def __init__(self, uid, cls, src, dst, size, t_gen, deadline, payload, key=None):
        self.uid, self.cls, self.src, self.dst = uid, cls, src, dst
        self.size, self.t_gen, self.deadline, self.payload = size, t_gen, deadline, payload
        self.at = src
        self.ready = t_gen        # earliest time this node may transmit it
        self.attempt = 0
        self.hops_done = 0
        self.key = key            # stable identity for retransmitted data chunks


class Channel:
    """Hop-by-hop, shared-airtime packet transport (see module docstring)."""

    def __init__(self, net, seed=0, log_packets=False):
        self.net = net
        self.seed = seed
        self.queues = collections.defaultdict(list)     # node -> [Packet]
        self.busy_until = collections.defaultdict(float)  # node -> medium busy until
        self.inflight = []                               # heap (t_arrive, n, packet, ok, a, b)
        self._n = itertools.count()
        self._uid = itertools.count(1)
        self._slot_ctr = collections.Counter()
        self.radio_up = set()
        self.stats = {c: collections.Counter() for c in CLASSES}
        self.bytes = {c: collections.Counter() for c in CLASSES}
        self.records = {}          # uid -> [cls, src, dst, t_gen, t_deliver, outcome, hops, key]
        self.by_cls = {c: ([], []) for c in CLASSES}     # cls -> (t_gen list, record list), unique only
        self.log_packets = log_packets
        self.airtime = collections.Counter()           # node -> seconds transmitting
        self.delivered = []        # (t, packet) delivered this step (drained by the sim)
        self.keys_delivered = set()  # data chunk keys delivered at least once

    # ------------------------------------------------------------ interface
    def submit(self, cls, src, dst, size, now, payload, key=None, deadline=None):
        """Generate a new unique packet at `src`. Returns its uid."""
        if deadline is None:
            deadline = now + C.DEADLINE.get(cls, 1e9)
        uid = next(self._uid)
        p = Packet(uid, cls, src, dst, size, now, deadline, payload, key)
        self.stats[cls]["generated"] += 1
        self.bytes[cls]["generated"] += size
        rec = [cls, src, dst, round(now, 3), None, "pending", 0, key]
        self.records[uid] = rec
        ts, rs = self.by_cls[cls]
        ts.append(now)
        rs.append(rec)
        q = self.queues[src]
        if len(q) >= C.QUEUE_LIMIT:
            self._finish(p, "dropped", now)
            return uid
        q.append(p)
        return uid

    def retransmit(self, cls, src, dst, size, now, payload, key):
        """End-to-end retransmission of an already-generated data chunk: a
        new packet on the air, not a new unique message."""
        uid = next(self._uid)
        p = Packet(uid, cls, src, dst, size, now, now + C.DEADLINE.get(cls, 1e9), payload, key)
        self.stats[cls]["e2e_retransmissions"] += 1
        self.records[uid] = [cls, src, dst, round(now, 3), None, "pending", 0, key]
        self.queues[src].append(p)
        return uid

    def set_radios(self, up):
        self.radio_up = set(up)

    def queue_len(self, node):
        return len(self.queues.get(node, ()))

    # ---------------------------------------------------------------- core
    def _loss_draw(self, a, b, now, attempt):
        slot = int(now / C.DT)
        k = (min(a, b), max(a, b), slot)
        i = self._slot_ctr[k]
        self._slot_ctr[k] += 1
        h = (self.seed * 1_000_003 + k[0] * 97_003 + k[1] * 7919 + slot * 131 + i * 17
             + attempt) & MASK64
        return _mix(h)

    def _finish(self, p, outcome, t):
        rec = self.records[p.uid]
        rec[5] = outcome
        rec[6] = p.hops_done
        if outcome == "delivered":
            rec[4] = round(t, 3)
        if p.key is None:
            self.stats[p.cls][outcome] += 1
            self.bytes[p.cls][outcome] += p.size
        elif outcome == "delivered":
            if p.key in self.keys_delivered:
                self.stats[p.cls]["duplicate_deliveries"] += 1
            else:
                self.keys_delivered.add(p.key)
                self.stats[p.cls]["delivered"] += 1
                self.bytes[p.cls]["delivered"] += p.size
        else:
            self.stats[p.cls][outcome + "_attempts"] += 1

    def step(self, t0, t1):
        """Advance the channel from t0 to t1. Delivered packets are appended
        to self.delivered as (t_arrival, packet)."""
        self._slot_ctr.clear()
        for node, q in self.queues.items():            # expire stale control traffic
            keep = []
            for p in q:
                if t0 > p.deadline and p.cls in ("hb", "cmd", "thumb", "ack"):
                    self._finish(p, "expired", t0)
                else:
                    keep.append(p)
            q[:] = keep
        progress = True
        while progress:
            progress = False
            while self.inflight and self.inflight[0][0] <= t1:
                ta, _, p, ok, a, b = heapq.heappop(self.inflight)
                if ok and b in self.radio_up:
                    p.hops_done += 1
                    p.at = b
                    p.ready = ta
                    p.attempt = 0
                    if b == p.dst:
                        self._finish(p, "delivered", ta)
                        self.delivered.append((ta, p))
                    else:
                        self.queues[b].append(p)
                    progress = True
                else:
                    p.attempt += 1
                    if p.attempt > C.MAC_RETRIES:
                        self._finish(p, "lost", ta)
                    else:
                        p.ready = ta
                        self.queues[a].insert(0, p)     # retry from the head of its queue
                    progress = True
            for node in sorted(self.queues):
                q = self.queues[node]
                if not q or node not in self.radio_up:
                    continue
                medium = max(t0, self.busy_until[node])
                if medium >= t1:
                    continue
                q.sort(key=lambda p: PRIORITY[p.cls])
                sent = False
                for i, p in enumerate(q):
                    start = max(medium, p.ready)
                    if start >= t1:
                        continue                          # not yet available at this node
                    nh = self.net.next_hop(node, p.dst)
                    if nh is None or nh not in self.net.links.get(node, {}):
                        continue                          # no route yet: keep waiting
                    q.pop(i)
                    air = p.size * 8.0 / (C.LINK_RATE_MBPS * 1e6) + C.MAC_OVERHEAD
                    end = start + air
                    self.busy_until[node] = end
                    for m in self.net.links.get(node, {}):
                        self.busy_until[m] = max(self.busy_until[m], end)
                    self.airtime[node] += air
                    self.stats[p.cls]["hop_tx"] += 1
                    self.bytes[p.cls]["hop_tx"] += p.size
                    ok = self._loss_draw(node, nh, start, p.attempt) >= self.net.links[node][nh]
                    heapq.heappush(self.inflight, (end + C.HOP_LATENCY, next(self._n), p, ok,
                                                   node, nh))
                    sent = progress = True
                    break
                if not sent:
                    continue

    def finish(self, t):
        """Mark everything still queued or in flight as pending at time t."""
        for q in self.queues.values():
            for p in q:
                self.stats[p.cls]["pending_end"] += 1
        for item in self.inflight:
            self.stats[item[2].cls]["pending_end"] += 1

    # ------------------------------------------------------------- metrics
    def cohort_pdr(self, cls, now, window=None, src=None, dst=None):
        """Delivered-within-deadline / generated, over unique packets of
        `cls` whose deadline has passed (matured) and that were generated in
        the `window` seconds before maturity (all matured packets if window
        is None). None (N/A) if the cohort is empty. Not defined for data
        chunks, which have no deadline (see site-level timeliness instead)."""
        dl = C.DEADLINE[cls]
        ts, rs = self.by_cls[cls]
        hi = bisect.bisect_right(ts, now - dl)
        lo = 0 if window is None else bisect.bisect_right(ts, now - dl - window)
        n = ok = 0
        for rec in rs[lo:hi]:
            if src is not None and rec[1] != src or dst is not None and rec[2] != dst:
                continue
            n += 1
            if rec[4] is not None and rec[4] - rec[3] <= dl + 1e-6:
                ok += 1
        return None if n == 0 else 100.0 * ok / n

    def latency_stats(self, cls):
        lat = sorted(r[4] - r[3] for r in self.by_cls[cls][1] if r[4] is not None)
        if not lat:
            return None
        q = lambda f: lat[min(len(lat) - 1, int(f * len(lat)))]
        return {"n": len(lat), "p50_s": round(q(0.5), 3), "p95_s": round(q(0.95), 3),
                "max_s": round(lat[-1], 3)}
