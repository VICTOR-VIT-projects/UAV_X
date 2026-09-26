"""Who knows what (brief item 7): positive AND negative tests.

Coverage: the GCS and UAV decision paths, the channel as the only transport,
sequence/expiry/dedup handling, and fault injection. Negative fixtures
introduce a deliberate bypass (a GCS that peeks at simulator truth; a
heartbeat delivered around the channel) and the same checks must catch it.

Limitations: these tests cannot prove the absence of every conceivable
leak (e.g. through Python globals a future change might add); they check
the two decision classes' imports, their runtime inputs, and that their
decisions are invariant to hidden truth for identical delivered inputs.
"""

import ast
import copy
import os
import unittest

from helpers import PKG, A, C, SwarmSim, quiet_sim, run_for, run_until
from uavx.comms import GCS_ID
from uavx.gcs import GroundStation


def _imports(fn):
    with open(os.path.join(PKG, "uavx", fn), encoding="utf-8") as f:
        tree = ast.parse(f.read())
    out = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.ImportFrom, ast.Import)):
            out |= {a.name for a in node.names}
            if isinstance(node, ast.ImportFrom) and node.module:
                out.add(node.module)
    return out


SIM_REF = {}


class LeakyGCS(GroundStation):
    """NEGATIVE FIXTURE: a GCS that peeks at simulator truth for UAVs it
    cannot hear (treats a silent UAV as alive, at its true position). It
    must be caught by the hidden-truth invariance test."""

    def _plan(self, now):
        sim = SIM_REF.get("sim")
        if sim is not None:
            for uid, k in self.known.items():
                if k["lost"]:                                # the prohibited bypass
                    k["pos"] = tuple(sim.uav(uid).pos)
                    k["battery"] = sim.uav(uid).battery
                    k["lost"] = False
        return GroundStation._plan(self, now)


def _cut_all(sim):
    for u in sim.uavs:
        u.radio_ok = False


def decisions_invariant_to_hidden_truth(gcs_cls):
    """Run twice with identical delivered inputs but different hidden truth:
    one airborne UAV loses its radio (the GCS declares it lost), then, in the
    second run only, that unreachable UAV is moved. No packet can reveal the
    move. Returns True if the GCS's decisions are identical in both runs."""
    plans = []
    for perturb in (False, True):
        sim = SwarmSim(log=lambda m: None, scenario=False, gcs_cls=gcs_cls)
        SIM_REF["sim"] = sim
        run_until(sim, lambda: sim.t > 120, 121)
        victim = next(u for u in sim.uavs if u.status == A.ACTIVE)
        victim.radio_ok = False
        run_for(sim, 4.0)                        # GCS declares it lost
        saved = [list(u.pos) for u in sim.uavs]
        if perturb:                              # move the unreachable UAV
            victim.pos[0], victim.pos[1] = 900.0, 900.0   # far corner of the area
            victim.battery = 20.0                          # and nearly empty
        log = []
        for _ in range(int(6.0 / C.DT)):
            sim.gcs.next_plan = sim.t            # force a plan every step
            sim.gcs.update(sim.t)
            log.append({u: t for u, t in sim.gcs.tasks.items()})
            sim.t += C.DT
        for u, p in zip(sim.uavs, saved):
            u.pos = p
        victim.battery = 100.0
        plans.append(log)
    SIM_REF.clear()
    return plans[0] == plans[1]


class Isolation(unittest.TestCase):
    def test_decision_modules_cannot_import_truth(self):
        forbidden = {"sim", "metrics", "comms", "Channel", "Network", "SwarmSim"}
        self.assertFalse(_imports("gcs.py") & forbidden, _imports("gcs.py"))
        self.assertFalse(_imports("agent.py") & forbidden, _imports("agent.py"))

    def test_production_gcs_ignores_hidden_truth(self):
        self.assertTrue(decisions_invariant_to_hidden_truth(GroundStation))

    def test_negative_leaky_gcs_is_caught(self):
        self.assertFalse(decisions_invariant_to_hidden_truth(LeakyGCS),
                         "the invariance test failed to detect a GCS reading simulator truth")

    def test_negative_channel_bypass_is_caught(self):
        sim = quiet_sim(scenario=False)
        run_for(sim, 30.0)
        self.assertFalse(sim.information_flow()["bypass"])
        u = sim.uav(1)                            # deliver a heartbeat around the channel
        sim.gcs.on_heartbeat(u.heartbeat(1, GCS_ID), sim.t)
        self.assertTrue(sim.information_flow()["bypass"])


class Partition(unittest.TestCase):
    def setUp(self):
        self.sim = quiet_sim(scenario=False)
        run_until(self.sim, lambda: self.sim.t > 120, 121)

    def test_gcs_knowledge_freezes(self):
        sim = self.sim
        _cut_all(sim)
        run_for(sim, 2.5)                         # let packets in flight land or expire
        snap = {u: (k["pos"], k["battery"], k["last_seen"]) for u, k in sim.gcs.known.items()}
        hb = sim.gcs.hb_rx
        truth = [tuple(u.pos) for u in sim.uavs]
        run_for(sim, 20.0)
        self.assertEqual(sim.gcs.hb_rx, hb, "GCS received telemetry with no radio path")
        for u, k in sim.gcs.known.items():
            self.assertEqual((k["pos"], k["battery"], k["last_seen"]), snap[u])
            self.assertTrue(k["lost"])
        self.assertNotEqual(truth, [tuple(u.pos) for u in sim.uavs])

    def test_new_command_not_executed_before_delivery(self):
        sim = self.sim
        _cut_all(sim)
        run_for(sim, 2.5)
        rx = {u.id: u.cmd_rx for u in sim.uavs}
        sim.inject_new_poi(("H9", -700.0, -700.0, 2))
        run_for(sim, 10.0)
        self.assertTrue(any(t and t[2] == "H9" for t in sim.gcs.tasks.values()),
                        "GCS should plan H9 on its own knowledge")
        for u in sim.uavs:
            self.assertEqual(u.cmd_rx, rx[u.id], f"UAV{u.id} applied a command with no radio")
            self.assertNotEqual(u.task[2], "H9")
        for u in sim.uavs:                        # reconnect: only now can it arrive
            u.radio_ok = True
        t_back = sim.t
        run_until(sim, lambda: any(u.task[2] == "H9" for u in sim.uavs), sim.t + 20)
        got = [u for u in sim.uavs if u.task[2] == "H9"]
        self.assertTrue(got)
        self.assertGreater(got[0].last_heard_gcs, t_back)

    def test_disconnected_survey_is_not_known_until_its_data_arrives(self):
        sim = quiet_sim(scenario=False)
        u = sim.uav(1)
        run_until(sim, lambda: u.task[0] == A.SURVEY and u.status == A.ACTIVE, 60)
        pid = u.task[2]
        u.radio_ok = False
        run_until(sim, lambda: pid in u.surveyed, sim.t + 200)
        self.assertIn(pid, u.surveyed)
        run_for(sim, 5.0)
        p = sim.gcs.pois[pid]
        self.assertIsNone(p["delivered"])
        self.assertIsNone(p["held_known"])
        u.radio_ok = True
        run_until(sim, lambda: sim.gcs.pois[pid]["delivered"] is not None, sim.t + 200)
        self.assertIsNotNone(sim.gcs.pois[pid]["delivered"])


class Protocol(unittest.TestCase):
    def test_stale_heartbeat_never_rolls_state_back(self):
        g = GroundStation(C.POIS, lambda m: None)
        hb = {"id": 1, "seq": 10, "pos": (0, 0, 60), "battery": 50.0, "status": A.ACTIVE,
              "data": set(), "task": (A.HOLD, None, None), "hops": 1, "parent": 0,
              "cmd_seq": 0, "forwarding": True}
        g.on_heartbeat(hb, 1.0)
        old = dict(hb, seq=9, battery=90.0)
        g.on_heartbeat(old, 1.1)
        self.assertEqual(g.known[1]["battery"], 50.0)
        self.assertEqual(g.hb_stale, 1)

    def test_duplicate_chunk_is_deduplicated_but_reacked(self):
        g = GroundStation(C.POIS, lambda m: None)
        key = (1, "P1", 0)
        self.assertTrue(g.on_chunk(1, key, None, 1, 1.0))
        self.assertTrue(g.on_chunk(1, key, None, 1, 1.5))
        self.assertEqual(g.chunk_dup, 1)
        self.assertEqual(len(g.pois["P1"]["chunks"]), 1)

    def test_fault_injection_does_not_inform_the_gcs(self):
        sim = quiet_sim(scenario=False)
        run_until(sim, lambda: len(sim.net.forwarders()) > 0 and sim.t > 60, 400)
        victim = sim.inject_failure(hard=True)
        k0 = copy.deepcopy(sim.gcs.known[victim])
        sim.step()
        k1 = sim.gcs.known[victim]
        self.assertEqual((k0["status"], k0["lost"]), (k1["status"], k1["lost"]))
        self.assertNotIn(victim, [u for _, u in sim.gcs.emergency_events])
        run_until(sim, lambda: sim.gcs.known[victim]["lost"], sim.t + 10)
        lost_at = next(t for t, u in sim.gcs.lost_events if u == victim)
        self.assertGreaterEqual(lost_at - sim.faults[-1]["t"],
                                C.LOST_TIMEOUT - C.HEARTBEAT_PERIOD - C.DT)

    def test_no_bypass_over_a_full_mission(self):
        sim = quiet_sim(scenario=True)
        while not sim.done:
            sim.step()
        flow = sim.information_flow()
        self.assertFalse(flow["bypass"], flow)
        self.assertGreater(flow["gcs_heartbeats_applied"], 0)


if __name__ == "__main__":
    unittest.main()
