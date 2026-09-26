"""Scenario and model parameters for the UAV-X Stage 1 PoC (v3, BVLOS scale).

All distances are metres, times are seconds, battery is percent.
Coordinates are ENU (x east, y north, z up), matching Webots' default.

Scale rationale:
  * Visual line of sight for a small multirotor ends around 400-500 m, so a
    2 km x 2 km operating area with sites up to ~2.4 km from the GCS is
    genuinely beyond visual line of sight.
  * 600 m air-to-air range is a conservative figure for a low-power 2.4 GHz
    mesh radio with omni antennas (longer-range links exist but cost weight
    and power). It keeps multi-hop relaying necessary at this scale.
  * 15 m/s cruise, 5 m/s climb, ~30 min endurance and 60-92 m altitude
    layers (below the 120 m ceiling) are typical of a Mavic-class airframe.
"""

# --- Simulation ------------------------------------------------------------
DT = 0.064                  # step, matches Webots basicTimeStep = 64 ms
MAX_TIME = 2400.0           # hard stop for a run
SEED = 7                    # packet-loss RNG seed (runs are reproducible)

# --- Operating area --------------------------------------------------------
GEOFENCE = (-1000.0, 1000.0, -1000.0, 1000.0)   # xmin, xmax, ymin, ymax
FENCE_MARGIN = 30.0         # targets are kept this far inside the fence
ALT_CEILING = 120.0         # regulatory ceiling

GCS_POS = (-850.0, -850.0, 8.0)   # ground control station antenna

# --- Fleet -----------------------------------------------------------------
NUM_UAVS = 5
CRUISE_SPEED = 15.0         # horizontal m/s
CLIMB_SPEED = 5.0           # vertical m/s
BASE_ALT = 60.0             # UAV i cruises at BASE_ALT + (i-1)*ALT_STEP
ALT_STEP = 8.0              # -> altitude layering gives vertical separation
PAD_SPACING = 6.0           # each UAV has its own landing pad next to the GCS
PAD_Z = 0.3
MIN_SEPARATION = 5.0        # repulsive fallback kicks in below this 3D range
EMERGENCY_DESCENT = 2.5     # m/s controlled descent after a critical fault
EMERGENCY_KEEPOUT = 15.0    # m horizontal clearance others keep from a failsafe descent column

# Starting charge per UAV (a real fleet is never uniformly full).
INITIAL_BATTERY = [100.0, 100.0, 48.0, 100.0, 85.0]

# --- Battery model ---------------------------------------------------------
DRAIN_CRUISE = 0.055        # %/s while translating  (~30 min endurance)
DRAIN_HOVER = 0.050         # %/s while hovering
RESERVE = 15.0              # % that must remain on landing
# Pad service is a battery SWAP, not charging: after landing the UAV is in
# SERVICE for SWAP_TIME seconds (pack exchange + checks), then has a full
# pack. 60 s is time-compressed (a real field swap is ~2-5 min); disclosed.
SWAP_TIME = 60.0
# Legacy names kept for readers of older results.
RECHARGE_RATE = None
READY_LEVEL = 95.0          # a UAV that starts below this is sent for a swap first

# --- Communication model ---------------------------------------------------
# Radio profile: "simple" (baseline, distance-threshold), "degraded" (shorter
# range, steeper edge loss) or "shadowed" (simple + terrain line of sight).
# The two stress profiles are uncalibrated sensitivity cases.
RADIO_PROFILE = "simple"
DEGRADED_RANGE = 480.0
DEGRADED_LOSS_EDGE = 0.35
SHADOW_CLEARANCE = 25.0     # m of terrain clearance below which extra loss applies
SHADOW_LOSS = 0.30
COMM_RANGE = 600.0          # hard link cutoff
RELAY_SPACING = 0.8 * COMM_RANGE  # planner spaces relays this far apart
LOSS_BASE = 0.01            # per-hop packet loss at zero distance
LOSS_EDGE = 0.20            # extra loss approaching COMM_RANGE, ~(d/R)^4
HEARTBEAT_PERIOD = 0.5      # UAV -> GCS telemetry
COMMAND_PERIOD = 0.5        # GCS -> UAV tasking
LOST_TIMEOUT = 3.0          # GCS declares a UAV lost after this silence
REQUEUE_AFTER_LOST = 240.0  # upper bound: re-plan a silent surveyor's PoI after this long
SILENT_SURVEY_SLACK = 60.0  # s added to a silent surveyor's expected out-and-back time
HP_UNEXPLAINED_SILENCE_REQUEUE = False   # optional variant, benchmarked separately (off in the demo)
LOST_LINK_WITH_DATA = 3.0   # UAV holding data flies back to the network after this
LOST_LINK_RELAY = 3.0       # an orphaned relay (and its fragment) falls back toward the GCS after this
LOST_LINK_ABORT = 120.0     # any UAV returns toward the GCS after this silence
MULE_HYSTERESIS = True      # False = the recorded v3 behaviour (range-edge dithering), for A/B only

# Shared channel (comms.Channel): every hop of every packet costs airtime at
# the sender and silences its radio neighbours; see comms.py.
LINK_RATE_MBPS = 4.0        # PHY rate of the shared channel
MAC_OVERHEAD = 0.0005       # s per transmission (preamble, MAC ack, backoff)
MAC_RETRIES = 3             # per-hop retries after a failed attempt (802.11-style)
HOP_LATENCY = 0.02          # s per hop of processing/forwarding
QUEUE_LIMIT = 64            # packets per node, tail drop
HB_BYTES = 200              # heartbeat: pos, battery, status, task, route, data held, cmd ack
CMD_BYTES = 300             # command: task + delivery acks + seq + expiry
ACK_BYTES = 40              # end-to-end imagery chunk acknowledgement (GCS -> UAV)
THUMB_BYTES = 8000          # live-feed thumbnail (160x120 JPEG)
THUMB_PERIOD = 2.0          # s per thumbnail per airborne UAV
SURVEY_DATA_MB = 1.5        # imagery bundle per site (~8 geotagged JPEGs)
CHUNK_BYTES = 32768         # imagery chunk
DATA_WINDOW = 4             # imagery chunks in flight per UAV (sliding window)
CHUNK_RTO = 4.0             # s without an end-to-end ack -> retransmit the chunk
# Per-class deadline: a packet not delivered within this is expired (control
# traffic) and counts as not delivered in the cohort PDR.
DEADLINE = {"hb": 2.0, "cmd": 2.0, "ack": 5.0, "thumb": 2.0, "data": 1e9}
COMMAND_EXPIRY = 2.0        # a UAV ignores a command older than this

# Metric definitions (ours, not organiser rules; see docs/METRICS.md).
PDR_WINDOW = 10.0           # s, "recent" PDR cohort window
STABLE_WINDOW = 5.0         # s, required population must stay routed this long
FAULT_WINDOW = 180.0        # s, attribution window of a fault's recovery metrics
EPISODE_MERGE_GAP = 1.0     # s, route gaps closer than this are one outage episode
DATA_DEADLINE = 60.0        # s, INTERNAL benchmark: survey-complete -> imagery at GCS

# --- Declared operational constraints (INTERNAL, predeclared 2026-09-25) -----
# UAV-X has published no timing thresholds for Stage 1, so these are our own
# test constraints, fixed before re-running any comparison. A run can be
# "eventually complete" and still fail them.
CONSTRAINTS = {
    "imagery_complete_by_s": 900.0,      # all site imagery at the GCS
    "fleet_landed_by_s": 1200.0,         # every surviving UAV on its pad
    "high_priority_response_s": 240.0,   # new high-priority report -> its imagery at GCS
    "timely_sites_min_frac": 0.75,       # sites within DATA_DEADLINE
    "min_separation_m": 5.0,             # no controller-caused separation below this
    "collisions": 0,                     # no geometric contact (< 1.5 m, proxy)
    "geofence_violations": 0,
    "battery_depletions": 0,
    "min_landing_battery_pct": 15.0,     # the reserve
}

# --- Energy-aware relay management ------------------------------------------
# The GCS only assigns a slot if, from what it knows,
#     E_remaining > E_transit + E_on_station + E_return + E_reserve
# and it starts a relay handover once a relay's margin above (return + reserve)
# drops below HANDOVER_MARGIN. The onboard RTH (margin < 5%) is the backstop.
ON_STATION_BUDGET = 120.0   # s of hover budgeted when assigning a slot
HANDOVER_MARGIN = 12.0      # % margin at which the GCS dispatches a replacement
HANDOVER_RANGE = 60.0       # replacement this close to the slot and linked -> on station
HANDOVER_HOLD = 5.0         # s all dependants must stay served through the replacement
HANDOVER_TIMEOUT = 120.0    # s from dispatch before the handover is declared degraded
HANDOVER_CRITICAL = 7.0     # % margin below which the outgoing relay leaves regardless
BACKUP_OFFSET = 110.0       # m, lateral offset of a backup relay from the relay it shadows
# "spare": a backup relay only when a UAV is left over after every site is
# covered. "reserve": always hold one UAV back as a backup beside any relay
# carrying >= BACKUP_MIN_DEPENDANTS others (slower coverage, no single point
# of failure there). tools/evaluate.py measures both.
BACKUP_POLICY = "spare"      # "spare" | "reserve" | "conditional" (see gcs.py)
BACKUP_MIN_DEPENDANTS = 2
CONDITIONAL_DEPENDANTS = 3

# --- Mission ---------------------------------------------------------------
SURVEY_RADIUS = 5.0
SURVEY_TIME = 10.0
SURVEY_STANDOFF = 0.8       # hover this many cruise-altitudes back from a PoI
PLAN_PERIOD = 1.0

# Ground truth: survivors placed at each site in the world (tools/make_world.py
# uses this table). Only used to score perception, never by the swarm.
SURVIVORS_GT = {"P1": 3, "P2": 2, "P3": 4, "P4": 3, "P5": 2, "P6": 3, "P7": 4, "H1": 3}

# (id, x, y, priority). Priority 1 = normal, 2 = high.
POIS = [
    ("P1", -400.0, -600.0, 1),
    ("P2", -700.0, -100.0, 1),
    ("P3", 100.0, -700.0, 1),
    ("P4", 300.0, 200.0, 1),
    ("P5", -200.0, 700.0, 1),
    ("P6", 750.0, -200.0, 1),
    ("P7", 550.0, 550.0, 1),
]

# Scripted disturbances, used for the demo video and the headless benchmark.
# "fail" and "radio" pick their victim at runtime (the busiest relay / the
# deepest node) so the fault always lands where it hurts most.
#   fail "hard":     motor fault AND radio lost -> no EMERGENCY heartbeat; the
#                    GCS only finds out by timeout (the hard case).
#   fail "failsafe": motor fault, radio survives the controlled descent.
#   battery_sag:     an ENERGY-AVAILABILITY fault on the GCS-side relay: this
#                    much usable charge disappears at once (an abstraction of
#                    a failed cell; no voltage/current model).
# The critical fault is an ABSTRACTED mobility/fault response: the UAV makes
# a controlled vertical descent; no rotor-out dynamics are modelled.
SCENARIO = [
    (150.0, "fail", "hard"),
    (220.0, "new_poi", ("H1", -600.0, 850.0, 2)),
    (300.0, "radio", 45.0),
    (360.0, "battery_sag", 52.0),
]

# Safety thresholds (INTERNAL test meaning, geometric proxies: flight is
# kinematic, so there is no physical collision detection).
NEAR_MISS_M = 5.0           # separation target; closer = near-miss episode
COLLISION_M = 1.5           # geometric contact proxy (rotor-disc scale)
