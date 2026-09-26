"""Captions for the demo video: what failed, what the GCS knows, what was
decided, what recovered.

Each disturbance opens a "story" whose lines fill in as events arrive.
  * WHAT FAILED / NOTE come from the observer's event log.
  * GCS KNOWS and DECISION come from GCS log lines and GCS knowledge only.
  * RESULT (observer) lines for faults come from the evaluator's fault
    trackers (uavx/metrics.py), with the populations and timestamps defined
    in docs/METRICS.md, and are labelled as observer measurements.
Scheduling: a story stays until resolved, then HOLD s; a new disturbance
replaces a story that is only being held and queues behind one that is still
developing; a story with no news for HOLD s is parked and returns with its
result.
"""

import re

from uavx import agent as A
from uavx import config as C

HOLD = 16.0          # s of sim time (~4 s of video at 4x)


class Story:
    def __init__(self, kind, title, t, color, key=None):
        self.kind, self.title, self.color, self.key = kind, title, color, key
        self.t0 = t
        self.lines = {}
        self.done_t = None
        self.shown_t = None
        self.updated_t = t
        self.tracker = None
        self.parked_at = 0.0

    def set(self, label, text, t):
        if self.lines.get(label) != text:
            self.updated_t = t
        self.lines[label] = text


class Narrator:
    ORDER = ("WHAT FAILED", "GCS KNOWS", "DECISION", "RESULT (observer)", "RESULT", "NOTE")

    def __init__(self):
        self.seen = 0
        self.story = None
        self.queue = []
        self.parked = []
        self.history = []

    # ------------------------------------------------------------- output
    def caption(self, sim):
        """(title, [(label, text)], rgb) for the current frame, or None."""
        self._consume(sim)
        t = sim.t
        for s in self.history:
            if s.tracker is not None and s.done_t is None:
                self._tracker_lines(s, t)
        for p in list(self.parked):               # news for a parked story: bring it back
            if p.done_t is not None or p.updated_t > p.parked_at:
                self.parked.remove(p)
                if p.done_t is not None:
                    p.done_t = t
                p.shown_t = None
                self.queue.insert(0, p)
        s = self.story
        if s is None and self.queue:
            s = self.story = self.queue.pop(0)
        if s is None:
            return self._idle(sim)
        if s.shown_t is None:
            s.shown_t = t
        self._live(sim, s)
        if s.done_t is not None and t > max(s.done_t, s.shown_t) + HOLD:
            self.story = None
            return self.caption(sim)
        if (s.done_t is None and self.queue and t - s.shown_t > HOLD
                and t - s.updated_t > HOLD):
            s.parked_at = t
            self.parked.append(s)
            self.story = None
            return self.caption(sim)
        return s.title, [(k, s.lines[k]) for k in self.ORDER if s.lines.get(k)], s.color

    def _idle(self, sim):
        if sim.t < 40:
            return ("MISSION  ·  5 UAVs, 7 sites up to 2.4 km from the GCS, 600 m radios",
                    [("SETUP", "every site is beyond radio range of the GCS: imagery must hop "
                               "UAV to UAV over a shared 4 Mbit/s channel"),
                     ("VIEWS", "3D = simulator truth (observer)  ·  right panel and feeds = only "
                               "what has reached the GCS")], (120, 200, 255))
        if sim.gcs.complete_time is not None:
            return (f"ALL SITE IMAGERY AT GCS  ·  t = {sim.gcs.complete_time:.0f} s",
                    [("NEXT", "fleet recalled; the GCS confirms recovery when every UAV it can "
                              "hear reports it has landed")], (80, 230, 120))
        return None

    # ------------------------------------------------------------ helpers
    def _open(self, story):
        self.history.append(story)
        cur = self.story
        if cur is None or cur.done_t is not None:
            self.story = story
        else:
            self.queue.append(story)

    def _find(self, kind, key=None):
        for s in [self.story] + self.queue + self.parked:
            if s is not None and s.kind == kind and (key is None or s.key == key):
                return s
        return None

    def _consume(self, sim):
        ev = sim.events[self.seen:]
        self.seen = len(sim.events)
        for t, m in ev:
            self._event(sim, t, m)

    def _tracker_lines(self, s, t):
        f = s.tracker
        ts = f.ts
        if f.orphaned is None:
            return
        if s.kind == "fail" and not f.orphaned and f.outcome:
            s.set("RESULT (observer)", f"no other UAV lost its route ({len(f.before)} unaffected); "
                                       "aircraft loss is irrecoverable", t)
            s.done_t = t
            return
        parts = []
        if ts["first_route"] is not None:
            parts.append(f"first route back +{ts['first_route']:.1f}s")
        if ts["all_routes"] is not None:
            parts.append(f"all {len(f.required)} routed +{ts['all_routes']:.1f}s")
        if f.flaps:
            parts.append(f"{f.flaps} re-split(s)")
        if ts["stable_confirmed"] is not None:
            parts.append(f"stable from +{ts['stable_start']:.1f}s, confirmed +"
                         f"{ts['stable_confirmed']:.1f}s ({C.STABLE_WINDOW:.0f}s hold)")
            parts.append(f"{f.disc_uav_s:.0f} UAV-s cut off")
        elif parts:
            parts.append("holding for stability")
        if parts:
            s.set("RESULT (observer)", "  ·  ".join(parts), t)
        if f.outcome is not None and f.outcome.startswith(("recovered", "not stable", "unrecovered")):
            if f.outcome.startswith(("not stable", "unrecovered")):
                s.set("RESULT (observer)", f.outcome, t)
            s.done_t = t

    def _event(self, sim, t, m):
        g = re.match(r"FAULT: UAV(\d) critical fault( \+ radio loss)? \(was relaying for (\d+)", m)
        if g:
            uid, hard, deps = int(g[1]), bool(g[2]), int(g[3])
            s = Story("fail", f"CRITICAL FAULT  ·  UAV{uid}  ·  t = {t:.0f} s", t, (255, 90, 80), uid)
            s.tracker = next((f for f in reversed(sim.metrics.faults)
                              if f.kind == "uav_failure" and f.uid == uid), None)
            s.set("WHAT FAILED", f"UAV{uid} motor fault{' + radio loss' if hard else ''} "
                                 f"(abstracted): controlled descent. It was relaying for {deps} UAV(s).", t)
            s.set("GCS KNOWS", "nothing yet: no EMERGENCY heartbeat can get out" if hard
                  else f"EMERGENCY arrives in UAV{uid}'s next heartbeat", t)
            self._open(s)
            return
        g = re.match(r"GCS: UAV(\d) declared LOST", m)
        if g:
            s = self._find("fail")
            if s and s.done_t is None and "DECISION" not in s.lines:
                s.set("DECISION", "onboard: cut-off relays and their mesh fragment fall back toward "
                                  "the GCS  ·  GCS: re-plans on what it last heard", t)
            return
        g = re.match(r"GCS: UAV(\d) reports EMERGENCY", m)
        if g:
            s = self._find("fail", int(g[1]))
            if s:
                s.set("GCS KNOWS", f"EMERGENCY heartbeat received {t - s.t0:.1f} s after the "
                                   "fault", t)
                s.set("DECISION", "GCS re-plans at once; the descending UAV keeps forwarding "
                                  "meanwhile", t)
            return
        g = re.match(r"GCS: new HIGH-PRIORITY PoI (\w+)", m)
        if g:
            pid = g[1]
            p = sim.gcs.pois[pid]
            km = ((p["x"] - C.GCS_POS[0]) ** 2 + (p["y"] - C.GCS_POS[1]) ** 2) ** 0.5 / 1000
            s = Story("poi", f"NEW HIGH-PRIORITY SITE {pid}  ·  t = {t:.0f} s", t,
                      (255, 120, 230), pid)
            s.set("GCS KNOWS", f"report of survivors at {pid}, {km:.1f} km out, beyond every "
                               "radio", t)
            s.set("DECISION", "served before any normal site: relay chain or data-mule leg "
                              "assigned at once", t)
            self._open(s)
            return
        g = re.match(r"GCS: imagery for (\w+) complete from UAV(\d) \((\d) hops?\)(.*)", m)
        if g:
            s = self._find("poi", g[1])
            if s:
                p = sim.gcs.pois[g[1]]
                s.set("RESULT", f"all imagery at GCS {t - p['added']:.0f} s after the report "
                                f"({g[3]} hops){self._ppl(g[1], g[4])}", t)
                s.done_t = t
            return
        g = re.match(r"FAULT: UAV(\d) radio outage for (\d+)s", m)
        if g:
            uid = int(g[1])
            s = Story("radio", f"RADIO OUTAGE  ·  UAV{uid}  ·  {g[2]} s  ·  t = {t:.0f} s", t,
                      (255, 90, 200), uid)
            s.tracker = next((f for f in reversed(sim.metrics.faults)
                              if f.kind == "comm_outage" and f.uid == uid), None)
            s.set("WHAT FAILED", f"UAV{uid}'s radio goes silent for {g[2]} s; UAV{uid} itself must "
                                 "reconnect before this counts as recovered", t)
            s.set("DECISION", f"UAV{uid} (onboard) keeps flying its task; imagery it can't send is "
                              "carried back until acked (data mule)", t)
            self._open(s)
            return
        g = re.match(r"GCS: UAV(\d) link re-acquired after ([\d.]+)s", m)
        if g:
            s = self._find("radio", int(g[1]))
            if s and float(g[2]) > 5:
                s.set("GCS KNOWS", f"UAV{g[1]} heard again after {g[2]} s of silence", t)
            return
        g = re.match(r"FAULT: UAV(\d) energy-availability fault (\d+)% -> (\d+)%", m)
        if g:
            uid = int(g[1])
            s = Story("sag", f"ENERGY FAULT  ·  UAV{uid} (GCS-side relay)  ·  t = {t:.0f} s",
                      t, (255, 190, 60), uid)
            s.set("WHAT FAILED", f"UAV{uid} loses usable charge {g[2]}% -> {g[3]}% at once "
                                 "(abstracted cell fault) while remote surveys continue", t)
            self._open(s)
            return
        g = re.match(r"GCS: UAV(\d) relay energy margin ([\d.]+)% < (\d+)%", m)
        if g:
            s = self._find("sag", int(g[1]))
            if s:
                s.set("GCS KNOWS", f"from telemetry: battery - (return + {C.RESERVE:.0f}% reserve) "
                                   f"= {g[2]}% < {g[3]}% handover margin", t)
            return
        g = re.match(r"GCS: UAV(\d) dispatched to relieve UAV(\d) \((\d+)%", m)
        if g:
            s = self._find("sag", int(g[2]))
            if s:
                s.set("DECISION", f"UAV{g[1]} ({g[3]}%) sent to the slot; UAV{g[2]} keeps relaying "
                                  "until the handover is verified", t)
            return
        g = re.match(r"GCS: UAV(\d) on station, traffic verified -> UAV(\d) told to stop forwarding", m)
        if g:
            s = self._find("sag", int(g[2]))
            if s:
                s.set("DECISION", f"UAV{g[1]} on station, commands acked both ways -> UAV{g[2]} "
                                  "stops forwarding; GCS checks every dependant is still heard", t)
            return
        g = re.match(r"GCS: downstream not served without UAV(\d) -> rollback", m)
        if g:
            s = self._find("sag", int(g[1]))
            if s:
                s.set("DECISION", f"a dependant went silent -> rollback: UAV{g[1]} forwards again", t)
            return
        g = re.match(r"GCS: UAV(\d) released home \((.+?); (\d+)%, needs (\d+)% incl\. reserve\)", m)
        if g:
            s = self._find("sag", int(g[1]))
            if s:
                s.set("RESULT", f"released: {g[2]}  ·  {g[3]}% left, needs {g[4]}% to land", t)
                s.done_t = t
            return
        g = re.match(r"UAV(\d): landed with (\d+)%", m)
        if g:
            s = next((h for h in self.history if h.kind == "sag" and h.key == int(g[1])), None)
            if s is not None and s.done_t is not None and not s.lines.get("NOTE"):
                s.lines["NOTE"] = " "
                n = Story("land", f"UAV{g[1]} LANDED AFTER THE HANDOVER  ·  t = {t:.0f} s", t,
                          (255, 190, 60))
                n.set("RESULT", f"touched down with {g[2]}% (reserve {C.RESERVE:.0f}%)", t)
                n.set("NOTE", f"pad service = battery SWAP in {C.SWAP_TIME:.0f} s (time-compressed; "
                              "real swaps take minutes)", t)
                n.done_t = t
                self._open(n)
            return
        g = re.match(r"GCS: every reachable UAV reports landed at t=([\d.]+)s(; unaccounted: (.*))?", m)
        if g:
            s = Story("end", "MISSION COMPLETE", t, (80, 230, 120))
            s.set("RESULT", f"all site imagery at GCS t = {sim.gcs.complete_time:.0f} s  ·  GCS-confirmed "
                            f"fleet landed t = {float(g[1]):.0f} s", t)
            if g[3]:
                s.set("NOTE", f"unaccounted by the GCS: {g[3]} (never heard again; the observer "
                              "knows it made a controlled landing)", t)
            s.done_t = t
            self._open(s)

    @staticmethod
    def _ppl(pid, tail):
        g = re.search(r"flagged (\d+) person", tail)
        if not g:
            return ""
        return (f"  ·  onboard count {g[1]}, scene truth {C.SURVIVORS_GT.get(pid)} "
                "(truth shown for evaluation only)")

    def _live(self, sim, s):
        if s.kind == "radio" and s.done_t is None:
            k = sim.gcs.known.get(s.key)
            if k and k["lost"]:
                s.lines["GCS KNOWS"] = (f"UAV{s.key} LOST · last heard {sim.t - k['last_seen']:.0f} s "
                                        f"ago at {k['battery']:.0f}% · feed frozen, state as last heard")
        if s.kind == "fail" and s.done_t is None:
            lost = [u for u, k in sim.gcs.known.items() if k["lost"] and u != s.key
                    and k["status"] == A.ACTIVE]
            if lost:
                s.lines["GCS KNOWS"] = (f"heartbeats stopped -> LOST after {C.LOST_TIMEOUT:.0f} s: "
                                        + ", ".join(f"UAV{u}" for u in lost)
                                        + "  ·  it holds their last known state")
