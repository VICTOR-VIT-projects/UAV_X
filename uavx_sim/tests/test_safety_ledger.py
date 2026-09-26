"""Safety episodes (item 10), outage ledger (item 2) and critical-relay
exposure / infeasible redundancy (item 4)."""

import unittest

from helpers import A, C, override, quiet_sim, run_for, run_until
from uavx.comms import GCS_ID, Network
from uavx.metrics import Metrics, seg_min_dist


def place(sim, uid, pos, status=A.ACTIVE):
    u = sim.uav(uid)
    u.status = status
    u.pos = list(pos)
    sim.metrics._prev_pos[uid] = tuple(pos)
    return u


class Safety(unittest.TestCase):
    def test_crossing_between_samples_is_detected(self):
        # Endpoints are 10 m apart at both samples, but the paths cross mid-step.
        d = seg_min_dist((0, 0, 60), (10, 0, 60), (10, 0, 60), (0, 0, 60))
        self.assertLess(d, 1e-6)
        self.assertLess(seg_min_dist((0, -5, 60), (0, 5, 60), (-5, 0, 60.5), (5, 0, 60.5)), 1.0)

    def _pair_run(self, path_a, path_b, status_b=A.ACTIVE):
        sim = quiet_sim(scenario=False)
        m = Metrics(sim)
        sim.metrics = m
        for pa, pb in zip(path_a, path_b):
            m._prev_pos = {u.id: tuple(u.pos) for u in sim.uavs}
            place(sim, 1, pa)
            place(sim, 2, pb, status_b)
            for k in (1, 2):
                m._prev_pos.setdefault(k, tuple(sim.uav(k).pos))
            m._safety(sim.t, C.DT)
            sim.t += C.DT
        m.close()
        return m

    def test_near_miss_episode_is_merged_and_measured(self):
        steps = 20
        a = [(0.0, 0.0, 60.0)] * steps
        b = [(3.0, 0.0, 60.0)] * steps
        m = self._pair_run(a, b)
        self.assertEqual(len(m.safety_eps), 1, "one continuous event, not one per step")
        e = m.safety_eps[0]
        self.assertEqual(e["severity"], "near miss")
        self.assertAlmostEqual(e["duration_s"], steps * C.DT, delta=2 * C.DT)
        self.assertAlmostEqual(e["min_m"], 3.0, delta=0.01)
        self.assertEqual(e["cause"], "controller")

    def test_collision_proxy_and_fault_attribution(self):
        a = [(0.0, 0.0, 60.0), (0.0, 0.0, 60.0)]
        b = [(0.5, 0.0, 60.0), (0.5, 0.0, 60.0)]
        m = self._pair_run(a, b, status_b=A.EMERGENCY)
        e = m.safety_eps[0]
        self.assertEqual(e["severity"], "collision (proxy)")
        self.assertEqual(e["cause"], "fault consequence")

    def test_takeoff_phase_is_labelled(self):
        sim = quiet_sim(scenario=False)
        u = place(sim, 1, list(A.pad_position(1))[:2] + [20.0])
        u.task = (A.RELAY, (0.0, 0.0), "P1")
        self.assertEqual(sim.metrics._phase(u), "takeoff")


class Ledger(unittest.TestCase):
    def test_range_boundary_motion_is_labelled(self):
        sim = quiet_sim(scenario=False)
        gx, gy, gz = C.GCS_POS
        u = place(sim, 1, (gx + 590.0, gy, 60.0))
        u.task = (A.RELAY, (gx + 800.0, gy), "PX")
        for other in sim.uavs[1:]:
            other.status = A.READY
        sim.net.update(sim.positions(), {GCS_ID, 1})
        sim.metrics.observe(C.DT)
        for _ in range(20):                          # fly outward across the edge
            sim.metrics._prev_pos = {v.id: tuple(v.pos) for v in sim.uavs}
            u.pos[0] += 1.0
            sim.t += C.DT
            sim.net.update(sim.positions(), {GCS_ID, 1})
            sim.metrics.observe(C.DT)
        sim.metrics.close()
        ep = sim.metrics.episodes[0]
        self.assertTrue(ep["cause"].startswith("range-boundary crossing: own motion"), ep["cause"])

    def test_heartbeat_timeout_with_a_path_present_is_distinguished(self):
        with override(LOSS_EDGE=0.97, MAC_RETRIES=0):
            sim = quiet_sim(scenario=False)
            run_for(sim, 250.0)
        kinds = {s["kind"] for s in sim.metrics.silences}
        self.assertIn("heartbeat timeout with a path present (loss/queue)", kinds)

    def test_every_episode_has_a_cause(self):
        sim = quiet_sim(scenario=True)
        while not sim.done:
            sim.step()
        for e in sim.metrics.episodes:
            self.assertTrue(e["cause"], e)
        summ = sim.results()["communication"]["outage_episodes"]
        total = summ["fault_attributed_uav_s"] + summ["not_fault_attributed_uav_s"]
        self.assertAlmostEqual(total, sum(e["duration_s"] for e in sim.metrics.episodes), delta=0.2)


class Redundancy(unittest.TestCase):
    def test_partitioned_time_is_not_counted_as_safe(self):
        net = Network()
        gx, gy, _ = C.GCS_POS
        pos = {GCS_ID: C.GCS_POS, 1: (gx + 450, gy, 60), 2: (gx + 3000, gy, 68)}
        net.update(pos, set(pos))
        self.assertFalse(net.connected(2))
        self.assertEqual(net.critical_relays({1, 2}), {})   # nothing to break ...
        sim = quiet_sim(scenario=False)
        sim.net = net
        place(sim, 1, pos[1])
        place(sim, 2, pos[2])
        for u in sim.uavs[2:]:
            u.status = A.READY
        sim.metrics.observe(1.0)
        self.assertEqual(dict(sim.metrics.state_s), {"partitioned": 1.0})  # ... but not "safe"

    def test_critical_relay_counts_its_dependants(self):
        net = Network()
        gx, gy, _ = C.GCS_POS
        pos = {GCS_ID: C.GCS_POS, 1: (gx + 450, gy, 60), 2: (gx + 900, gy, 68),
               3: (gx + 900, gy + 100, 76)}
        net.update(pos, set(pos))
        self.assertEqual(net.critical_relays({1, 2, 3}), {1: 2})

    def test_infeasible_redundancy_is_logged(self):
        sim = quiet_sim(scenario=True)
        run_for(sim, 200.0)
        self.assertTrue(sim.gcs.redundancy_infeasible)
        self.assertIn("no spare UAV", sim.gcs.redundancy_infeasible[0][1])

    def test_conditional_policy_places_a_backup_when_feasible(self):
        with override(BACKUP_POLICY="conditional"):
            sim = quiet_sim(scenario=False)
            run_for(sim, 400.0)
        slots = [k for k in sim.gcs.task_keys.values() if k and k[0] == "BACKUP"]
        placed = slots or any("BKP" in str(t[2]) for t in sim.gcs.tasks.values() if t)
        infeasible = sim.gcs.redundancy_infeasible
        self.assertTrue(placed or infeasible)


if __name__ == "__main__":
    unittest.main()
