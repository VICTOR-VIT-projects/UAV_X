"""Energy-driven relay handover (brief item 3): normal, delayed replacement,
packet loss during handover, replacement failure, insufficient energy."""

import unittest

from helpers import A, C, override, quiet_sim, run_for, run_until


def setup_handover(t=200.0, prepare=None):
    """Run to `t`, then drop the GCS-side relay with the most dependants to
    10 % above what it needs to get home (below the 12 % handover margin,
    above the 7 % critical margin and the onboard RTH)."""
    sim = quiet_sim(scenario=False)
    run_until(sim, lambda: sim.t > t, t + 1)
    relays = [u for u in sim.uavs if u.status == A.ACTIVE and u.task[0] == A.RELAY]
    relay = max(relays, key=lambda u: (sim.net.hops.get(u.id) == 1, sim._descendants(u.id)))
    if prepare:
        prepare(sim, relay)
    sim.inject_battery_sag(relay.battery - relay.energy_to_home() - 10.0, relay.id)
    run_until(sim, lambda: sim.gcs.handovers, sim.t + 20)
    return sim, relay


def finish(sim, relay, limit=400):
    run_until(sim, lambda: relay.landings, sim.t + limit)
    return sim.gcs.handovers[0]


class Handover(unittest.TestCase):
    def assert_safe(self, sim, relay, h):
        self.assertTrue(relay.landings, "outgoing relay never landed")
        self.assertGreaterEqual(relay.landings[0][1], C.RESERVE - 0.5)
        self.assertFalse(relay.depleted)
        self.assertIsNotNone(h["release_reason"] or h["outcome"])
        # Make-before-break: unless energy forced it (or the mission ended),
        # a release requires verified service to every dependant.
        if h["outcome"] in ("relieved on station (verified)",
                            "retired: slot no longer needed (verified)"):
            self.assertTrue(h["checks"]["drained"] and h["checks"]["downstream"], h["checks"])
            self.assertIsNotNone(h["t_hold_passed"])
        else:
            self.assertTrue(h["outcome"].startswith(("degraded", "ended early", "recalled")),
                            h["outcome"])

    VERIFIED = ("relieved on station (verified)", "retired: slot no longer needed (verified)")

    def test_normal_replacement(self):
        sim, relay = setup_handover()
        h = finish(sim, relay)
        self.assert_safe(sim, relay, h)
        if h["outcome"] == "relieved on station (verified)":
            for f in ("t_dispatch", "t_on_station", "t_bidirectional", "t_drain",
                      "t_downstream_ok", "t_hold_passed", "t_released"):
                self.assertIsNotNone(h[f], f)
            self.assertLessEqual(h["t_dispatch"], h["t_on_station"])
            self.assertLessEqual(h["t_drain"], h["t_downstream_ok"])
            self.assertGreaterEqual(h["t_released"] - h["t_downstream_ok"], C.HANDOVER_HOLD - 0.1)
        else:
            self.assertIn(h["outcome"], self.VERIFIED)

    def test_delayed_replacement_is_waited_for(self):
        sim, relay = setup_handover()
        h = sim.gcs.handovers[0]
        run_until(sim, lambda: h["replacement"] is not None, sim.t + 20)
        rep = sim.uav(h["replacement"])
        rep.radio_ok = False                        # replacement can't hear its tasking
        run_for(sim, 25.0)
        rep.radio_ok = True
        h = finish(sim, relay)
        self.assert_safe(sim, relay, h)          # never released on an unverified replacement
        self.assertIn(h["outcome"], self.VERIFIED + ("degraded: released on energy before "
                                                     "verification",))

    def test_packet_loss_during_handover(self):
        with override(LOSS_EDGE=0.55):
            sim, relay = setup_handover()
            h = finish(sim, relay)
        self.assert_safe(sim, relay, h)
        if h["outcome"] == "relieved on station (verified)":
            self.assertIsNotNone(h["t_hold_passed"])

    def test_replacement_failure(self):
        sim, relay = setup_handover()
        h = sim.gcs.handovers[0]
        run_until(sim, lambda: h["replacement"] is not None, sim.t + 20)
        first = h["replacement"]
        sim.uav(first).status = A.ACTIVE
        sim.inject_failure(first, hard=True)
        h = finish(sim, relay)
        self.assert_safe(sim, relay, h)
        self.assertTrue(len(h["replacements_tried"]) >= 2 or h["outcome"].startswith(
            ("degraded", "retired", "ended early")), h)

    def test_insufficient_energy_for_overlap(self):
        def drain_others(sim, relay):
            for u in sim.uavs:
                if u.id != relay.id:
                    u.battery = min(u.battery, u.energy_to_home() + 6.0)
        sim, relay = setup_handover(prepare=drain_others)
        h = finish(sim, relay)
        self.assert_safe(sim, relay, h)
        self.assertNotEqual(h["outcome"], "relieved on station (verified)")
        self.assertTrue(h["checks"].get("degraded") or h["outcome"].startswith(
            ("degraded", "retired", "ended early")), h)


if __name__ == "__main__":
    unittest.main()
