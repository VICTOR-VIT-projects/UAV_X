"""Recovery populations and timestamps (brief item 1)."""

import unittest

from helpers import A, C, override, quiet_sim, run_for, run_until


def _sim_with_relays():
    sim = quiet_sim(scenario=False)
    run_until(sim, lambda: len(sim.net.forwarders()) > 0 and sim.t > 100, 400)
    return sim


def _fault(sim, kind):
    return next(f for f in sim.metrics.faults if f.kind == kind)


class TemporaryOutage(unittest.TestCase):
    def test_not_recovered_while_its_radio_is_off(self):
        sim = _sim_with_relays()
        sim.inject_radio_outage(30.0)
        f = _fault(sim, "comm_outage")
        run_for(sim, 29.0)
        self.assertIn(f.uid, f.required, "the faulted aircraft must be in its own recovery")
        self.assertIsNone(f.ts["stable_start"])
        self.assertIsNone(f.ts["all_routes"])
        self.assertIsNone(f.outcome)
        run_until(sim, lambda: f.outcome is not None, sim.t + 60)
        self.assertGreaterEqual(f.ts["all_routes"], 30.0 - C.DT)
        self.assertAlmostEqual(f.ts["stable_confirmed"] - f.ts["stable_start"], C.STABLE_WINDOW,
                               delta=C.DT)

    def test_brief_reconnection_does_not_pass_the_hold(self):
        sim = _sim_with_relays()
        victim = sim.inject_radio_outage(8.0)
        f = _fault(sim, "comm_outage")
        run_for(sim, 8.0 + 2.0)                     # radio back for 2 s ...
        sim.uav(victim).radio_ok = False            # ... then off again for 10 s
        run_for(sim, 10.0)
        self.assertIsNone(f.ts["stable_start"], "a 2 s reconnection passed a 5 s hold")
        sim.uav(victim).radio_ok = True
        run_until(sim, lambda: f.outcome is not None, sim.t + 60)
        self.assertGreaterEqual(f.flaps, 1)
        self.assertGreaterEqual(f.ts["stable_start"], 20.0 - C.DT)
        self.assertLess(f.ts["all_routes"], f.ts["stable_start"])   # first, brief reconnection kept

    def test_outage_duration_vs_reacquisition_are_separate(self):
        sim = _sim_with_relays()
        victim = sim.inject_radio_outage(20.0)
        run_until(sim, lambda: _fault(sim, "comm_outage").outcome is not None, sim.t + 80)
        rep = _fault(sim, "comm_outage").report()
        self.assertEqual(rep["duration"], 20.0)
        k = sim.gcs.known[victim]
        self.assertFalse(k["lost"])
        self.assertLess(rep["all_app_delivery"] - rep["duration"], 2.0)


class AircraftLoss(unittest.TestCase):
    def test_irrecoverable_loss_of_a_leaf_is_unaffected_not_zero(self):
        sim = _sim_with_relays()
        leaves = [u for u in sim.uavs if u.status == A.ACTIVE
                  and u.id not in sim.net.forwarders()]
        sim.inject_failure(leaves[0].id, hard=True)
        f = _fault(sim, "uav_failure")
        run_for(sim, 1.0)
        rep = f.report()
        self.assertEqual(rep["population"]["orphaned"], [])     # a leaf carries no one
        self.assertTrue(rep["outcome"].startswith("unaffected"), rep["outcome"])
        self.assertIn(f"of {len(rep['population']['before_fault_routed'])} other UAVs",
                      rep["outcome"])
        self.assertIsNone(rep["stable_start"])                  # N/A, never "0.0 s"
        self.assertIn("irrecoverable", rep["faulted_aircraft"])

    def test_busiest_relay_hard_fault_has_ordered_timestamps(self):
        sim = _sim_with_relays()
        sim.inject_failure(hard=True)
        f = _fault(sim, "uav_failure")
        run_until(sim, lambda: f.outcome is not None, sim.t + C.FAULT_WINDOW + 5)
        ts = f.ts
        if f.orphaned:
            self.assertLessEqual(ts["first_route"], ts["all_routes"])
            self.assertLessEqual(ts["all_routes"], ts["stable_start"])
            self.assertAlmostEqual(ts["stable_confirmed"], ts["stable_start"] + C.STABLE_WINDOW,
                                   delta=C.DT)
            self.assertNotIn(f.uid, f.required)


class Bookkeeping(unittest.TestCase):
    def test_overlapping_faults_are_recorded(self):
        sim = _sim_with_relays()
        sim.inject_radio_outage(40.0)
        run_for(sim, 5.0)
        sim.inject_failure(hard=True)
        first = sim.metrics.faults[0]
        self.assertTrue(first._overlaps, "overlap not recorded")

    def test_unrecovered_event_has_an_explicit_outcome(self):
        with override(FAULT_WINDOW=20.0):
            sim = _sim_with_relays()
            sim.inject_radio_outage(60.0)
            f = _fault(sim, "comm_outage")
            run_for(sim, 25.0)
            self.assertTrue(f.outcome and f.outcome.startswith("not stable within"), f.outcome)


if __name__ == "__main__":
    unittest.main()
