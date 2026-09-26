"""Shared-channel accounting and causal behaviour (brief item 8)."""

import unittest

from helpers import C, override, quiet_sim, run_for
from uavx.comms import GCS_ID, Channel, Network


def chain(n_hops=3):
    """GCS + a straight chain of UAVs, each 450 m apart."""
    net = Network()
    gx, gy, _ = C.GCS_POS
    pos = {GCS_ID: C.GCS_POS}
    for i in range(1, n_hops + 1):
        pos[i] = (gx + 450.0 * i, gy, 60.0 + i)
    net.update(pos, set(pos))
    ch = Channel(net, seed=1)
    ch.set_radios(set(pos))
    return net, ch, pos


def drive(ch, t0, t1, dt=C.DT):
    t = t0
    out = []
    while t < t1:
        ch.step(t, t + dt)
        out += ch.delivered
        ch.delivered = []
        t += dt
    return out


class Delivery(unittest.TestCase):
    def test_no_instant_delivery_over_a_multi_hop_path(self):
        net, ch, _ = chain(3)
        ch.submit("hb", 3, GCS_ID, C.HB_BYTES, 0.0, {})
        got = drive(ch, 0.0, 2.0)
        self.assertEqual(len(got), 1)
        t, p = got[0]
        air = C.HB_BYTES * 8 / (C.LINK_RATE_MBPS * 1e6) + C.MAC_OVERHEAD
        self.assertGreaterEqual(t, 3 * (air + C.HOP_LATENCY) - 1e-9)
        self.assertEqual(p.hops_done, 3)

    def test_partition_blocks_and_expires(self):
        net = Network()
        pos = {GCS_ID: C.GCS_POS, 1: (C.GCS_POS[0] + 2000, C.GCS_POS[1], 60)}
        net.update(pos, set(pos))
        ch = Channel(net)
        ch.set_radios(set(pos))
        ch.submit("hb", 1, GCS_ID, C.HB_BYTES, 0.0, {})
        got = drive(ch, 0.0, 5.0)
        self.assertEqual(got, [])
        self.assertEqual(ch.stats["hb"]["expired"], 1)
        self.assertEqual(ch.cohort_pdr("hb", 10.0), 0.0)

    def test_empty_cohort_is_na(self):
        _, ch, _ = chain(1)
        self.assertIsNone(ch.cohort_pdr("hb", 100.0))
        self.assertIsNone(ch.cohort_pdr("hb", 100.0, window=10.0))

    def test_late_receipt_is_not_counted_as_on_time(self):
        _, ch, _ = chain(1)
        uid = ch.submit("hb", 1, GCS_ID, C.HB_BYTES, 0.0, {})
        rec = ch.records[uid]
        rec[4], rec[5] = C.DEADLINE["hb"] + 0.5, "delivered"     # arrived after its deadline
        self.assertEqual(ch.cohort_pdr("hb", 10.0), 0.0)

    def test_retransmissions_do_not_inflate_unique_delivery(self):
        _, ch, _ = chain(2)
        key = (2, "P1", 0)
        ch.submit("data", 2, GCS_ID, C.CHUNK_BYTES, 0.0, None, key=key)
        ch.retransmit("data", 2, GCS_ID, C.CHUNK_BYTES, 0.1, None, key)
        drive(ch, 0.0, 3.0)
        st = ch.stats["data"]
        self.assertEqual(st["generated"], 1)
        self.assertEqual(st["delivered"], 1)
        self.assertEqual(st["duplicate_deliveries"], 1)
        self.assertEqual(st["e2e_retransmissions"], 1)
        self.assertGreaterEqual(st["hop_tx"], 4)          # 2 packets x 2 hops, each counted


class Causality(unittest.TestCase):
    """Whole-mission knobs must move the outcome in the right direction."""

    @staticmethod
    def mission(seconds=260.0, **kv):
        with override(**kv):
            sim = quiet_sim(scenario=False)
            run_for(sim, seconds)
            return sim

    def test_lower_capacity_slows_imagery(self):
        base = self.mission()
        slow = self.mission(LINK_RATE_MBPS=0.5)
        lb = base.channel.latency_stats("data")["p50_s"]
        ls = slow.channel.latency_stats("data")["p50_s"]
        self.assertGreater(ls, lb * 1.5)

    def test_more_load_raises_control_latency(self):
        base = self.mission()
        heavy = self.mission(THUMB_BYTES=120000, THUMB_PERIOD=0.5)
        self.assertGreater(heavy.channel.latency_stats("hb")["p95_s"],
                           base.channel.latency_stats("hb")["p95_s"])

    def test_more_loss_costs_more_transmissions(self):
        base = self.mission()
        lossy = self.mission(LOSS_EDGE=0.6)
        rb = base.channel.stats["hb"]["hop_tx"] / max(1, base.channel.stats["hb"]["delivered"])
        rl = lossy.channel.stats["hb"]["hop_tx"] / max(1, lossy.channel.stats["hb"]["delivered"])
        self.assertGreater(rl, rb)

    def test_missing_acks_cause_retransmission_not_double_counting(self):
        sim = quiet_sim(scenario=False)
        orig = sim.channel.submit

        def drop_acks(cls, src, dst, size, now, payload, key=None, deadline=None):
            if cls == "ack" and 60.0 < now < 120.0:
                uid = orig(cls, src, dst, size, now, payload, key, deadline)
                p = sim.channel.queues[src].pop()
                sim.channel._finish(p, "dropped", now)
                return uid
            return orig(cls, src, dst, size, now, payload, key, deadline)
        sim.channel.submit = drop_acks
        run_for(sim, 300.0)
        st = sim.channel.stats["data"]
        self.assertGreater(st["e2e_retransmissions"], 0)
        self.assertGreater(sim.gcs.chunk_dup, 0)
        self.assertLessEqual(st["delivered"], st["generated"])
        done = [p for p in sim.gcs.pois.values() if p["delivered"] is not None]
        for p in done:
            self.assertEqual(len(p["chunks"]), __import__("uavx.agent").agent.N_CHUNKS)


class Reconciliation(unittest.TestCase):
    def test_accounting_balances(self):
        sim = quiet_sim(scenario=True)
        while not sim.done:
            sim.step()
        for cls in ("hb", "cmd", "thumb", "ack"):
            st = sim.channel.stats[cls]
            ends = sum(st[k] for k in ("delivered", "expired", "dropped", "dropped_radio_down",
                                       "lost", "pending_end"))
            self.assertEqual(st["generated"], ends, (cls, dict(st)))
        # Heartbeats applied by the GCS = heartbeats delivered by the channel.
        self.assertEqual(sim.gcs.hb_rx + sim.gcs.hb_stale, sim.channel.stats["hb"]["delivered"])


if __name__ == "__main__":
    unittest.main()
