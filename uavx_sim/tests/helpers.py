"""Shared test helpers."""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PKG = os.path.join(ROOT, "controllers", "swarm_supervisor")
if PKG not in sys.path:
    sys.path.insert(0, PKG)
if os.path.join(ROOT, "tools") not in sys.path:
    sys.path.insert(0, os.path.join(ROOT, "tools"))

from uavx import agent as A            # noqa: E402,F401
from uavx import config as C           # noqa: E402,F401
from uavx.sim import SwarmSim          # noqa: E402


def quiet_sim(**kw):
    return SwarmSim(log=lambda m: None, **kw)


def run_until(sim, cond, limit):
    while sim.t < limit and not sim.done and not cond():
        sim.step()


def run_for(sim, seconds):
    end = sim.t + seconds
    while sim.t < end and not sim.done:
        sim.step()


class override:
    """Temporarily change config values: with override(LINK_RATE_MBPS=2.0): ..."""

    def __init__(self, **kv):
        self.kv = kv
        self.old = {}

    def __enter__(self):
        for k, v in self.kv.items():
            self.old[k] = getattr(C, k)
            setattr(C, k, v)
        return self

    def __exit__(self, *exc):
        for k, v in self.old.items():
            setattr(C, k, v)
