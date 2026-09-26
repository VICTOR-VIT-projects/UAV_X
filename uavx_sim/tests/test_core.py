"""Link model, onboard agent, and the recorded scripted scenario as a
regression test.  Run:  python -m unittest discover -s tests -v"""

import unittest

from helpers import A, C, quiet_sim
from uavx.comms import GCS_ID, Network, loss_prob


class LinkModel(unittest.TestCase):
    def test_loss_grows_with_distance_and_cuts_off(self):
        self.assertLess(loss_prob(10), loss_prob(400))
        self.assertLess(loss_prob(400), loss_prob(C.COMM_RANGE))
        self.assertEqual(loss_prob(C.COMM_RANGE + 0.1), 1.0)

    def test_multi_hop_route_through_chain(self):
        net = Network()
        gx, gy, _ = C.GCS_POS
        pos = {GCS_ID: C.GCS_POS}
        step = 0.75 * C.COMM_RANGE                  # 3 UAVs in a line, each in range
        for i in range(1, 4):                       # of the next but not of the GCS
            pos[i] = (gx + step * i, gy, C.BASE_ALT)
        net.update(pos, set(pos))
        self.assertEqual(net.hops[3], 3)
        self.assertEqual(net.route(3), [3, 2, 1, GCS_ID])
        self.assertEqual(net.forwarders(), {1, 2})
        self.assertEqual(net.next_hop(GCS_ID, 3), 1)
        self.assertEqual(net.next_hop(2, GCS_ID), 1)

    def test_node_beyond_range_is_cut_off(self):
        net = Network()
        pos = {GCS_ID: C.GCS_POS, 1: (C.GCS_POS[0] + 3 * C.COMM_RANGE, C.GCS_POS[1], C.BASE_ALT)}
        net.update(pos, set(pos))
        self.assertIsNone(net.route(1))

    def test_drained_relay_forwards_nothing(self):
        net = Network()
        gx, gy, _ = C.GCS_POS
        pos = {GCS_ID: C.GCS_POS, 1: (gx + 450, gy, 60), 2: (gx + 900, gy, 68)}
        net.update(pos, set(pos), no_forward={1})
        self.assertTrue(net.connected(1))            # still reachable itself
        self.assertFalse(net.connected(2))           # but carries no one

    def test_shadowed_profile_blocks_links_through_terrain(self):
        ridge = lambda x, y: 200.0 if abs(x) < 50 else 0.0   # noqa: E731
        pos = {GCS_ID: (-270.0, 0.0, 8.0), 1: (270.0, 0.0, 60.0)}
        plain = Network(ground=ridge, profile="simple")
        plain.update(pos, set(pos))
        shadow = Network(ground=ridge, profile="shadowed")
        shadow.update(pos, set(pos))
        self.assertTrue(plain.connected(1))
        self.assertFalse(shadow.connected(1))


class Agent(unittest.TestCase):
    def test_geofence_clamp(self):
        x, y = A.clamp_to_fence(5 * C.GEOFENCE[1], 5 * C.GEOFENCE[2])
        xmin, xmax, ymin, ymax = C.GEOFENCE
        self.assertEqual((x, y), (xmax - C.FENCE_MARGIN, ymin + C.FENCE_MARGIN))

    def test_low_battery_triggers_rth_before_depletion(self):
        u = A.Uav(1, battery=60.0)
        u.status = A.ACTIVE
        far = 0.8 * C.GEOFENCE[1]
        u.pos = [far, far, u.cruise_alt]
        u.task = (A.RELAY, (far, far), "P7")
        t = 0.0
        while u.status != A.SERVICE and t < C.MAX_TIME:
            t += C.DT
            u.last_heard_gcs = t
            u.step(t, C.DT)
        self.assertEqual(u.status, A.SERVICE)
        self.assertFalse(u.depleted)
        self.assertGreaterEqual(u.battery, C.RESERVE - 1.0)

    def test_battery_swap_is_a_swap(self):
        u = A.Uav(1, battery=40.0)
        self.assertEqual(u.status, A.SERVICE)
        t = 0.0
        while u.status == A.SERVICE:
            t += C.DT
            self.assertEqual(u.battery, 40.0)       # no ramp: the pack is exchanged
            u.step(t, C.DT)
        self.assertAlmostEqual(t, C.SWAP_TIME, delta=C.DT)
        self.assertEqual(u.battery, 100.0)

    def test_battery_never_negative(self):
        u = A.Uav(1, battery=0.5)
        u.status = A.ACTIVE
        u.pos = [0.0, 0.0, u.cruise_alt]
        for _ in range(500):
            u.step(0.0, C.DT)
        self.assertGreaterEqual(u.battery, 0.0)

    def test_critical_fault_is_a_controlled_landing(self):
        u = A.Uav(1)
        u.status = A.ACTIVE
        u.pos = [0.0, 0.0, u.cruise_alt]
        u.fail()
        self.assertEqual(u.status, A.EMERGENCY)
        z, t = u.pos[2], 0.0
        while u.status == A.EMERGENCY and t < 300:
            u.step(t, C.DT)
            t += C.DT
            self.assertLessEqual(z - u.pos[2], C.EMERGENCY_DESCENT * C.DT + 1e-9)
            z = u.pos[2]
        self.assertEqual(u.status, A.FAILED)
        self.assertAlmostEqual(u.pos[2], C.PAD_Z)

    def test_data_mule_hysteresis(self):
        """Regression for the recorded run's range-edge dithering: a UAV
        carrying unacked imagery keeps heading home after it hears a
        command, until every chunk is acked."""
        u = A.Uav(2)
        u.status = A.ACTIVE
        u.pos = [0.0, 0.0, u.cruise_alt]
        u.task = (A.SURVEY, (100.0, 100.0), "P9")
        u.surveyed.add("P9")
        u.data.add("P9")
        u.last_heard_gcs = 0.0
        tgt = u._choose_target(10.0, C.DT)          # silent 10 s -> mule
        self.assertTrue(u.mule)
        self.assertAlmostEqual(tgt[0], C.GCS_POS[0] + 8.0)
        u.receive_task(u.task, set(), 10.1)         # hears one command
        tgt = u._choose_target(10.2, C.DT)
        self.assertTrue(u.mule, "a single command must not end the data-mule leg")
        self.assertAlmostEqual(tgt[0], C.GCS_POS[0] + 8.0)
        for i in range(A.N_CHUNKS):
            u.on_chunk_ack((2, "P9", i), 10.3)
        u._choose_target(10.4, C.DT)
        self.assertFalse(u.mule)

    def test_stale_and_expired_commands_are_ignored(self):
        u = A.Uav(1)
        task = (A.RELAY, (0.0, 0.0), "P1")
        ok = u.receive_command({"seq": 5, "expiry": 10.0, "task": task, "delivered": set()}, 9.0)
        self.assertTrue(ok)
        old = u.receive_command({"seq": 4, "expiry": 10.0, "task": (A.HOME, None, None),
                                 "delivered": set()}, 9.1)
        late = u.receive_command({"seq": 6, "expiry": 9.5, "task": (A.HOME, None, None),
                                  "delivered": set()}, 9.6)
        self.assertFalse(old)
        self.assertFalse(late)
        self.assertEqual(u.task, task)
        self.assertEqual((u.cmd_stale, u.cmd_expired), (1, 1))


class Planner(unittest.TestCase):
    def test_silent_surveyor_releases_its_site_after_its_expected_time(self):
        """Regression (evaluate.py E2 dev seed_007): a surveyor that went
        silent near its site must not hold a high-priority site for the full
        240 s; it is re-queued after its own out-and-back estimate."""
        from uavx.gcs import GroundStation
        g = GroundStation([("H1", 300.0, 0.0, 2)], lambda m: None)
        task = (A.SURVEY, (300.0, 0.0), "H1")
        g.known[1] = {"pos": (0.0, 0.0, 60.0), "status": A.ACTIVE, "lost": True,
                      "last_seen": 100.0, "battery": 90.0, "task": task, "cmd_seq": 5}
        g.tasks[1] = task
        g.slot_cmd_seq[1] = (task, 5)          # its heartbeat showed it applied this task
        expect = 2 * 300.0 / C.CRUISE_SPEED + C.SURVEY_TIME + C.SILENT_SURVEY_SLACK
        self.assertEqual(g._out_of_contact(100.0 + expect - 1.0), [1])     # still plausible
        self.assertEqual(g._out_of_contact(100.0 + expect + 1.0), [])      # re-queued
        self.assertLess(expect, C.REQUEUE_AFTER_LOST)

    def test_unacknowledged_survey_is_requeued_at_once(self):
        """A silent UAV that never showed it had applied its survey task
        cannot be working on it: its site is re-queued immediately."""
        from uavx.gcs import GroundStation
        g = GroundStation([("H1", 300.0, 0.0, 2)], lambda m: None)
        task = (A.SURVEY, (300.0, 0.0), "H1")
        g.known[1] = {"pos": (0.0, 0.0, 60.0), "status": A.ACTIVE, "lost": True,
                      "last_seen": 100.0, "battery": 90.0, "task": (A.HOLD, None, None),
                      "cmd_seq": 4}
        g.tasks[1] = task
        g.slot_cmd_seq[1] = (task, 5)          # sent as command 5, never acknowledged
        self.assertEqual(g._out_of_contact(101.0), [])

    def test_high_priority_site_is_served_before_normal_sites(self):
        """Regression (dev seed_023): a far high-priority site with no full
        chain still gets a surveyor in the same plan."""
        from uavx.gcs import GroundStation
        g = GroundStation([("P1", -800.0, -500.0, 1), ("P2", -500.0, -800.0, 1),
                           ("H1", 900.0, 900.0, 2)], lambda m: None)
        for i in (1, 2):
            g.known[i] = {"pos": A.pad_position(i), "status": A.READY, "lost": False,
                          "last_seen": 0.0, "battery": 100.0, "task": (A.HOLD, None, None),
                          "parent": 0, "hops": 1, "seq": 1, "cmd_seq": 0}
        g._plan(1.0)
        self.assertTrue(any(t and t[0] == A.SURVEY and t[2] == "H1" for t in g.tasks.values()),
                        g.tasks)


class ScriptedScenario(unittest.TestCase):
    """The recorded demo scenario (seed 7) as a regression test."""

    @classmethod
    def setUpClass(cls):
        cls.sim = quiet_sim(scenario=True)
        while not cls.sim.done:
            cls.sim.step()
        cls.r = cls.sim.results()

    def test_all_constraints(self):
        self.assertTrue(self.r["constraints"]["operational_success"], self.r["constraints"])

    def test_milestones_are_separate(self):
        m = self.r["mission"]
        self.assertEqual(m["pois_delivered"], m["pois_total"])
        self.assertLess(m["all_delivered_at_s"], m["fleet_landed_at_s_observer"])
        self.assertGreaterEqual(m["fleet_landed_confirmed_by_gcs_s"], m["fleet_landed_at_s_observer"])

    def test_hard_fault_event(self):
        f = next(f for f in self.r["faults"] if f["type"] == "uav_failure")
        self.assertIn("irrecoverable", f["faulted_aircraft"])
        self.assertNotIn(f["uav"], f["population"]["required"])
        # Timeout detection counts from the last heartbeat received, which
        # can precede the fault by up to one heartbeat period.
        self.assertGreaterEqual(f["detected"], C.LOST_TIMEOUT - C.HEARTBEAT_PERIOD - C.DT)
        self.assertLessEqual(f["detected"], C.LOST_TIMEOUT + C.DT)
        self.assertAlmostEqual(f["stable_confirmed"] - f["stable_start"], C.STABLE_WINDOW, delta=0.1)

    def test_radio_outage_includes_its_own_aircraft(self):
        f = next(f for f in self.r["faults"] if f["type"] == "comm_outage")
        self.assertIn(f["uav"], f["population"]["required"])
        self.assertGreaterEqual(f["stable_start"], f["duration"] - C.DT)

    def test_energy_handover_is_verified(self):
        ho = self.r["relay"]["energy_handovers"]
        self.assertEqual(len(ho), 1)
        h = ho[0]
        self.assertEqual(h["outcome"], "relieved on station (verified)")
        self.assertLessEqual(h["t_on_station"], h["t_drain"])
        self.assertLessEqual(h["t_drain"], h["t_downstream_ok"])
        self.assertGreaterEqual(h["t_released"] - h["t_downstream_ok"], C.HANDOVER_HOLD - 0.1)
        self.assertGreaterEqual(h["landed_battery"], C.RESERVE)
        self.assertEqual(h["handover_attributable_interruption_s"], 0)

    def test_no_bypass_and_no_collision(self):
        self.assertFalse(self.r["information_flow"]["bypass"])
        self.assertEqual(self.r["safety"]["collision_proxy_episodes"], 0)


if __name__ == "__main__":
    unittest.main()
