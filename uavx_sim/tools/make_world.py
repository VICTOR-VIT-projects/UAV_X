"""Generate worlds/uavx_stage3.wbt (and its .wbproj) from uavx/config.py.

BVLOS-scale scene (2 km x 2 km operating area inside 4.8 km of terrain):
an earthquake + landslide has hit a rural district. Villages (one at every
search site, plus two more) with damaged and collapsed buildings, roads,
farmland and forest, a ridge along the north-east with a landslide scar,
mountains all around, survivors at every Point of Interest, a forward
Ground Control Station compound, and five DJI Mavic 2 Pro quadcopters.

Run this after changing the fleet, PoIs, GCS position or geofence:

    python tools/make_world.py

Assets (buildings, trees, pedestrians, appearances, the Mavic 2 Pro meshes)
are the official Webots R2025a assets, fetched by Webots on first load and
cached afterwards, so the first launch needs an internet connection.
"""

import math
import os
import random
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "controllers", "swarm_supervisor"))

from uavx import config as C           # noqa: E402
from uavx.agent import pad_position    # noqa: E402

WEBOTS = "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects"
MAVIC = f"{WEBOTS}/robots/dji/mavic/protos/Mavic2Pro/meshes"
DRONE_SCALE = 10.0          # Mavic 2 Pro is ~35 cm; drawn 10x so it reads at 200 m
WORLD_NAME = "uavx_stage3"

UAV_COLORS = [(0.90, 0.20, 0.20), (0.20, 0.55, 0.95), (0.95, 0.75, 0.10),
              (0.60, 0.30, 0.85), (0.10, 0.75, 0.55), (0.95, 0.45, 0.10)]

EXTERNPROTOS = [
    "objects/backgrounds/protos/TexturedBackground.proto",
    "objects/backgrounds/protos/TexturedBackgroundLight.proto",
    "appearances/protos/Grass.proto",
    "appearances/protos/Soil.proto",
    "appearances/protos/DryMud.proto",
    "appearances/protos/Asphalt.proto",
    "appearances/protos/RoughConcrete.proto",
    "appearances/protos/RedBricks.proto",
    "appearances/protos/CorrugatedMetal.proto",
    "appearances/protos/GlossyPaint.proto",
    "humans/pedestrian/protos/Pedestrian.proto",
    "objects/rocks/protos/Rock.proto",
    "objects/trees/protos/Pine.proto",
    "objects/trees/protos/Tree.proto",
    "objects/trees/protos/Forest.proto",
    "objects/buildings/protos/SuburbanHouse.proto",
    "objects/buildings/protos/BungalowStyleHouse.proto",
    "objects/buildings/protos/SimpleTwoFloorsHouse.proto",
    "objects/buildings/protos/HouseWithGarage.proto",
    "objects/buildings/protos/ModernSuburbanHouse.proto",
    "objects/buildings/protos/Barn.proto",
    "objects/buildings/protos/Warehouse.proto",
    "objects/traffic/protos/TrafficCone.proto",
    "objects/traffic/protos/WorkBarrier.proto",
    "objects/obstacles/protos/OilBarrel.proto",
    "vehicles/protos/toyota/ToyotaPriusSimple.proto",
    "vehicles/protos/generic/TruckSimple.proto",
]


# --------------------------------------------------------------- terrain
def smoothstep(a, b, x):
    t = min(1.0, max(0.0, (x - a) / (b - a)))
    return t * t * (3 - 2 * t)


def ground_h(x, y):
    """Terrain height. Flat valley floor inside the operating area, a ridge
    along the north-east edge (site of the landslide), mountains outside."""
    r = max(abs(x), abs(y))
    hills = smoothstep(1050, 1900, r) * (160 + 70 * math.sin(x / 370.0) * math.cos(y / 290.0)
                                         + 45 * math.sin((x + y) / 230.0))
    ridge = smoothstep(600, 1040, y) * 28.0 * smoothstep(-350, 250, x)
    rough = smoothstep(980, 1200, r) * 4.0 * math.sin(x / 70.0) * math.cos(y / 90.0)
    return hills + ridge + rough


def in_landslide(x, y):
    """Tongue of debris running down the ridge toward P7."""
    if not (600 <= y <= 1060):
        return False
    t = (y - 600) / 460.0                # 0 at the toe, 1 at the crown
    cx = 640 + 60 * t + 30 * math.sin(y / 60.0)
    half = 90 + 120 * t
    return abs(x - cx) <= half


# Villages: (name, x, y, radius, houses, collapsed piles). One at every
# search site (the survivors are there) plus two unaffected hamlets.
VILLAGES = [
    ("P1", -400, -600, 90, 6, 2), ("P2", -700, -100, 80, 5, 2), ("P3", 100, -700, 90, 6, 2),
    ("P4", 300, 200, 110, 8, 3), ("P5", -200, 700, 80, 5, 2), ("P6", 750, -200, 90, 6, 2),
    ("P7", 550, 550, 80, 4, 2), ("H1", -600, 850, 70, 4, 1),
    ("Ravipur", -150, -150, 90, 6, 1), ("Nandgaon", 650, -700, 70, 4, 0),
]
ROADS = [(-850, -850, -400, -600), (-400, -600, 100, -700), (100, -700, 650, -700),
         (650, -700, 750, -200), (-400, -600, -150, -150), (-150, -150, 300, 200),
         (300, 200, 750, -200), (300, 200, 550, 550), (-400, -600, -700, -100),
         (-700, -100, -200, 700), (-200, 700, -600, 850), (-150, -150, -200, 700)]


def in_village(x, y):
    for _, vx, vy, vr, _, _ in VILLAGES:
        dx, dy = x - vx, y - vy
        wob = 1.0 + 0.12 * math.sin(math.atan2(dy, dx) * 3 + vx)
        if dx * dx + dy * dy <= (vr * 1.25 * wob) ** 2:
            return True
    return False


def dist_to_segment(x, y, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy)))
    return math.hypot(x - (x1 + t * dx), y - (y1 + t * dy))


def near_road(x, y, margin):
    return any(dist_to_segment(x, y, *r) < margin for r in ROADS)


def elevation(name, x0, y0, nx, ny, sp, appearance, mask=None, lift=0.0):
    hs = []
    for j in range(ny):
        for i in range(nx):
            x, y = x0 + i * sp, y0 + j * sp
            h = ground_h(x, y)
            if mask is not None:
                h = h + lift if mask(x, y) else h - 0.8
            hs.append(f"{h:.2f}")
    rows = "\n        ".join(" ".join(hs[k:k + 20]) for k in range(0, len(hs), 20))
    return f"""Solid {{
  translation {x0} {y0} 0
  children [
    Shape {{
      appearance {appearance}
      geometry ElevationGrid {{
        xDimension {nx}
        xSpacing {sp}
        yDimension {ny}
        ySpacing {sp}
        height [
        {rows}
        ]
      }}
      castShadows FALSE
    }}
  ]
  name "{name}"
}}
"""


# ---------------------------------------------------------------- helpers
def f(*v):
    return " ".join(f"{x:.4g}" for x in v)


def axis_angle_look(pos, target):
    """Orientation (axis-angle) for a Viewpoint/Camera at pos looking at
    target with z up (Webots cameras look along +x)."""
    dx, dy, dz = (target[i] - pos[i] for i in range(3))
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.hypot(dx, dy))
    return axis_angle_from_yaw_pitch(yaw, pitch)


def axis_angle_from_yaw_pitch(yaw, pitch, roll=0.0):
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    # R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    r = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
         [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
         [-sp, cp * sr, cp * cr]]
    angle = math.acos(max(-1.0, min(1.0, (r[0][0] + r[1][1] + r[2][2] - 1) / 2)))
    if angle < 1e-9:
        return (0, 0, 1, 0)
    ax = (r[2][1] - r[1][2], r[0][2] - r[2][0], r[1][0] - r[0][1])
    n = math.sqrt(sum(a * a for a in ax))
    if n < 1e-9:                         # 180 degree rotation
        d = [math.sqrt(max(0.0, (r[i][i] + 1) / 2)) for i in range(3)]
        return (d[0], d[1], d[2], math.pi)
    return (ax[0] / n, ax[1] / n, ax[2] / n, angle)


def box(name, pos, size, appearance, rot=None, shadows=True):
    rot_line = f"\n  rotation {f(*rot)}" if rot else ""
    return f"""Solid {{
  translation {f(*pos)}{rot_line}
  children [ Shape {{
    appearance {appearance}
    geometry Box {{ size {f(*size)} }}
    castShadows {"TRUE" if shadows else "FALSE"}
  }} ]
  name "{name}"
}}
"""


def pbr(r, g, b, rough=0.9, metal=0.0, transparency=0.0, emissive=None):
    em = f" emissiveColor {f(*emissive)}" if emissive else ""
    return (f"PBRAppearance {{ baseColor {f(r, g, b)} roughness {rough} "
            f"metalness {metal} transparency {transparency}{em} }}")


# -------------------------------------------------------------- scenery
def collapsed_pile(name, x, y, rng, size=1.0):
    """Pancaked building: tilted concrete slabs, broken brick walls, rubble."""
    parts = []
    z0 = ground_h(x, y)
    for k in range(4):
        w, d = rng.uniform(7, 11) * size, rng.uniform(6, 9) * size
        ax = rng.uniform(-1, 1), rng.uniform(-1, 1), 0.0
        n = math.hypot(ax[0], ax[1]) or 1
        parts.append(f"""    Pose {{
      translation {f(rng.uniform(-2, 2), rng.uniform(-2, 2), 0.4 + k * 0.7)}
      rotation {f(ax[0] / n, ax[1] / n, 0, rng.uniform(0.05, 0.35))}
      children [ Shape {{
        appearance RoughConcrete {{ textureTransform TextureTransform {{ scale 3 3 }} }}
        geometry Box {{ size {f(w, d, 0.45)} }}
      }} ]
    }}""")
    for k in range(3):
        a = rng.uniform(0, math.pi * 2)
        parts.append(f"""    Pose {{
      translation {f(math.cos(a) * 4 * size, math.sin(a) * 4 * size, 1.2)}
      rotation {f(math.cos(a + 1.3), math.sin(a + 1.3), 0, rng.uniform(0.1, 0.5))}
      children [ Shape {{
        appearance RedBricks {{ textureTransform TextureTransform {{ scale 2 1 }} }}
        geometry Box {{ size {f(rng.uniform(3, 6), 0.35, rng.uniform(1.5, 3.2))} }}
      }} ]
    }}""")
    for k in range(7):
        s = rng.uniform(0.4, 1.2)
        g = rng.uniform(0.45, 0.62)
        parts.append(f"""    Pose {{
      translation {f(rng.uniform(-6, 6) * size, rng.uniform(-6, 6) * size, s * 0.3)}
      rotation {f(0, 0, 1, rng.uniform(0, 3))}
      children [ Shape {{
        appearance {pbr(g, g * 0.96, g * 0.9)}
        geometry Box {{ size {f(s * 1.6, s * 1.2, s * 0.7)} }}
      }} ]
    }}""")
    return f"""Solid {{
  translation {f(x, y, z0)}
  rotation 0 0 1 {rng.uniform(0, 3.14):.3f}
  children [
{chr(10).join(parts)}
  ]
  name "{name}"
}}
"""


def survivors(pid, x, y, rng, n):
    out = []
    shirts = [(0.95, 0.45, 0.05), (0.85, 0.1, 0.1), (0.95, 0.85, 0.1),
              (0.2, 0.45, 0.85), (0.9, 0.9, 0.9), (0.15, 0.6, 0.25)]
    for k in range(n):
        a = rng.uniform(0, math.pi * 2)
        r = rng.uniform(0.5, 3.0)
        px, py = x + math.cos(a) * r, y + math.sin(a) * r
        col = shirts[rng.randrange(len(shirts))]
        out.append(f"""Pedestrian {{
  translation {f(px, py, ground_h(px, py) + 1.27)}
  rotation 0 0 1 {rng.uniform(0, 6.28):.3f}
  name "survivor_{pid}_{k}"
  controller "<none>"
  shirtColor {f(*col)}
  pantsColor {f(*[rng.uniform(0.1, 0.4)] * 3)}
}}
""")
    return "".join(out)


def poi_marker(pid, x, y):
    """Search-site marker: survey ring on the ground plus a flag mast.
    The controller recolours POI_<id>_APP / POI_<id>_RING by state."""
    z = ground_h(x, y)
    return f"""DEF POI_{pid} Solid {{
  translation {f(x, y, z)}
  children [
    Pose {{
      translation 0 0 7
      children [ Shape {{
        appearance PBRAppearance {{ baseColor 0.9 0.9 0.9 roughness 0.5 metalness 0.6 }}
        geometry Cylinder {{ height 14 radius 0.15 }}
      }} ]
    }}
    Pose {{
      translation 1.6 0 12.6
      children [ Shape {{
        appearance DEF POI_{pid}_APP PBRAppearance {{
          baseColor 0.9 0.15 0.15 emissiveColor 0.6 0.1 0.1 roughness 0.6 metalness 0
        }}
        geometry Box {{ size 3.2 0.06 1.9 }}
      }} ]
    }}
    Pose {{
      translation 0 0 0.15
      children [ Shape {{
        appearance DEF POI_{pid}_RING PBRAppearance {{
          baseColor 0.9 0.15 0.15 emissiveColor 0.45 0.07 0.07 roughness 1 metalness 0 transparency 0.55
        }}
        geometry Cylinder {{ height 0.06 radius 22 subdivision 48 top TRUE bottom FALSE side FALSE }}
        castShadows FALSE
      }} ]
    }}
  ]
  name "poi_{pid}"
}}
"""


def mast(i):
    """Thin altitude line from the ground to UAV i (kept outside the drone so
    it stays vertical). At 2 km scale this is what makes each UAV trackable
    in the overview shot; the controller moves and resizes it."""
    col = UAV_COLORS[(i - 1) % len(UAV_COLORS)]
    return f"""DEF UAV_{i}_MAST Pose {{
  translation 0 0 -200
  children [ Shape {{
    appearance {pbr(*col, 1, 0, 0.35, col)}
    geometry DEF UAV_{i}_MAST_GEO Cylinder {{ height 1 radius 0.7 subdivision 8 }}
    castShadows FALSE
  }} ]
}}
"""


def drone(i):
    """DJI Mavic 2 Pro meshes (official Webots asset), scaled for visibility.
    Controller hooks: UAV_<i> pose, UAV_<i>_P<k> propellers, UAV_<i>_LED
    nav-light colour (role), UAV_<i>_BEAM survey cone."""
    x, y, z = pad_position(i)
    col = UAV_COLORS[(i - 1) % len(UAV_COLORS)]
    props = [(-0.177179, 0.127453, -0.0320282, "a"), (-0.177179, -0.127453, -0.0320282, "b"),
             (0.0548537, -0.151294, -0.00280468, "a"), (0.0548537, 0.151294, -0.00280468, "b")]
    prop_nodes, leds = [], []
    for k, (px, py, pz, ab) in enumerate(props, 1):
        prop_nodes.append(f"""        DEF UAV_{i}_P{k} Pose {{
          translation {f(px, py, pz)}
          children [
            Shape {{ appearance USE UAV_{i}_GREY geometry Mesh {{ url "{MAVIC}/helix_{ab}.obj" }} }}
            Shape {{ appearance USE UAV_{i}_METAL geometry Mesh {{ url "{MAVIC}/helix_{ab}_joint.obj" }} }}
          ]
        }}""")
        leds.append(f"""        Pose {{
          translation {f(px, py, pz - 0.012)}
          children [ Shape {{
            appearance {"DEF UAV_%d_LED " % i if k == 1 else "USE UAV_%d_LED" % i}{pbr(1, 1, 1, 0.3, 0, 0, (1, 1, 1)) if k == 1 else ""}
            geometry Sphere {{ radius 0.022 subdivision 2 }}
          }} ]
        }}""")
    return f"""DEF UAV_{i} Solid {{
  translation {f(x, y, z + 0.45)}
  children [
    Transform {{
      scale {DRONE_SCALE} {DRONE_SCALE} {DRONE_SCALE}
      children [
        Shape {{
          appearance DEF UAV_{i}_GREY PBRAppearance {{ baseColor 0.42 0.43 0.45 roughness 0.7 metalness 0 }}
          geometry Mesh {{ url "{MAVIC}/body.obj" }}
        }}
        Shape {{
          appearance DEF UAV_{i}_METAL PBRAppearance {{ baseColor 0.35 0.35 0.37 roughness 0.45 metalness 0.8 }}
          geometry Mesh {{ url "{MAVIC}/body_metal_parts.obj" }}
        }}
        Shape {{
          appearance PBRAppearance {{ baseColor 0.1 0.12 0.12 roughness 0.2 metalness 0 transparency 0.3 }}
          geometry Mesh {{ url "{MAVIC}/body_lenses.obj" }}
        }}
        Pose {{
          translation 0.0412774 -0.00469654 -0.00405862
          children [
            Shape {{ appearance USE UAV_{i}_METAL geometry Mesh {{ url "{MAVIC}/camera_yaw.obj" }} }}
            Pose {{
              translation 0.000625212 -0.00530346 -0.0207448
              children [
                Shape {{ appearance USE UAV_{i}_METAL geometry Mesh {{ url "{MAVIC}/camera_pitch.obj" }} }}
                Pose {{
                  translation 0.00754686 0.0160388 -0.00586401
                  children [
                    Shape {{ appearance USE UAV_{i}_METAL geometry Mesh {{ url "{MAVIC}/camera_chassis.obj" }} }}
                    Shape {{ appearance PBRAppearance {{ baseColor 0.05 0.05 0.1 roughness 0.1 metalness 0 }} geometry Mesh {{ url "{MAVIC}/camera_lens.obj" }} }}
                  ]
                }}
              ]
            }}
          ]
        }}
        Pose {{
          translation -0.075 0 0.033
          children [ Shape {{
            appearance {pbr(*col, 0.4, 0.2, 0, tuple(c * 0.5 for c in col))}
            geometry Box {{ size 0.07 0.022 0.004 }}
          }} ]
        }}
{chr(10).join(prop_nodes)}
{chr(10).join(leds)}
      ]
    }}
    DEF UAV_{i}_BEAM Pose {{
      translation 0 0 -5
      children [ Shape {{
        appearance DEF UAV_{i}_BEAM_APP PBRAppearance {{
          baseColor 0.3 1 0.5 emissiveColor 0.2 0.8 0.4 roughness 1 metalness 0 transparency 1
        }}
        geometry DEF UAV_{i}_BEAM_GEO Cone {{ bottomRadius 5 height 10 side TRUE bottom FALSE }}
        castShadows FALSE
      }} ]
    }}
  ]
  name "uav_{i}"
}}
"""


def link(a, b):
    return f"""DEF LINK_{a}_{b} Pose {{
  translation 0 0 -100
  children [ Shape {{
    appearance DEF LINK_{a}_{b}_APP PBRAppearance {{
      baseColor 0.2 0.9 1 emissiveColor 0.2 0.9 1 roughness 1 metalness 0 transparency 0.5
    }}
    geometry DEF LINK_{a}_{b}_GEO Cylinder {{ height 1 radius 1.2 subdivision 8 }}
    castShadows FALSE
  }} ]
}}
"""


def pad(i):
    x, y, _ = pad_position(i)
    white = pbr(0.95, 0.95, 0.95, 0.6)
    return f"""Solid {{
  translation {f(x, y, 0.06)}
  children [
    Shape {{ appearance {pbr(0.18, 0.19, 0.2, 0.7)} geometry Box {{ size 4.6 4.6 0.12 }} }}
    Pose {{ translation -0.8 0 0.065 children [ Shape {{ appearance {white} geometry Box {{ size 0.35 2.4 0.01 }} }} ] }}
    Pose {{ translation 0.8 0 0.065 children [ Shape {{ appearance {white} geometry Box {{ size 0.35 2.4 0.01 }} }} ] }}
    Pose {{ translation 0 0 0.065 children [ Shape {{ appearance {white} geometry Box {{ size 1.6 0.35 0.01 }} }} ] }}
  ]
  name "pad_{i}"
}}
"""


def gcs_compound():
    gx, gy, gz = C.GCS_POS
    cams = []
    for i in range(1, C.NUM_UAVS + 1):
        cams.append(f"""    DEF CAM_{i} Camera {{
      translation 0 0 -50
      name "cam_{i}"
      fieldOfView 0.9
      width 640
      height 480
      near 0.4
      far 4000
    }}""")
    return f"""DEF GCS Robot {{
  translation {f(gx, gy, 0)}
  children [
    Pose {{
      translation 0 0 1.35
      children [ Shape {{
        appearance CorrugatedMetal {{ colorOverride 0.85 0.87 0.88 textureTransform TextureTransform {{ scale 3 1 }} }}
        geometry Box {{ size 6.1 2.45 2.7 }}
      }} ]
    }}
    Pose {{
      translation 3.2 0.2 1.0
      children [ Shape {{ appearance {pbr(0.25, 0.3, 0.25, 0.8)} geometry Box {{ size 0.4 1.6 2.0 }} }} ]
    }}
    Pose {{
      translation -2 -0.3 {gz + 4:.1f}
      children [ Shape {{ appearance {pbr(0.75, 0.75, 0.78, 0.4, 0.8)} geometry Cylinder {{ height 11 radius 0.12 }} }} ]
    }}
    Pose {{
      translation -2 -0.3 {gz + 9.6:.1f}
      children [ Shape {{ appearance {pbr(1, 0.3, 0.2, 0.3, 0, 0, (1, 0.2, 0.1))} geometry Sphere {{ radius 0.35 subdivision 2 }} }} ]
    }}
    Pose {{
      translation -2 -0.3 {gz + 7.5:.1f}
      rotation 0 1 0 0.6
      children [ Shape {{ appearance {pbr(0.92, 0.92, 0.92, 0.5)} geometry Cylinder {{ height 0.15 radius 1.1 subdivision 24 }} }} ]
    }}
    Pose {{
      translation 0 0 0.04
      children [ Shape {{
        appearance {pbr(0.3, 0.6, 1.0, 1, 0, 0.96)}
        geometry Cylinder {{ height 0.05 radius {C.COMM_RANGE:.1f} subdivision 72 top TRUE bottom FALSE side FALSE }}
        castShadows FALSE
      }} ]
    }}
{chr(10).join(cams)}
  ]
  name "gcs"
  controller "swarm_supervisor"
  supervisor TRUE
}}
TruckSimple {{
  translation {f(gx - 9, gy + 1, 0.5)}
  rotation 0 0 1 1.57
  color 0.9 0.9 0.88
  trailer NULL
  name "command truck"
}}
""" + "".join(f"""TrafficCone {{ translation {f(gx + 6 + k * 4, gy + 5, 0)} name "cone_{k}" }}
""" for k in range(8)) + f"""WorkBarrier {{ translation {f(gx + 4, gy - 4, 0)} rotation 0 0 1 1.57 name "barrier_0" }}
WorkBarrier {{ translation {f(gx + 4, gy - 8, 0)} rotation 0 0 1 1.57 name "barrier_1" }}
OilBarrel {{ translation {f(gx - 4, gy - 4, 0.44)} name "barrel_0" }}
OilBarrel {{ translation {f(gx - 5, gy - 3, 0.44)} name "barrel_1" }}
"""


def geofence():
    xmin, xmax, ymin, ymax = C.GEOFENCE
    out = []
    n = 0
    for x in range(int(xmin), int(xmax) + 1, 100):
        for y in (ymin, ymax):
            out.append((x, y))
    for y in range(int(ymin) + 100, int(ymax), 100):
        for x in (xmin, xmax):
            out.append((x, y))
    posts = []
    for x, y in out:
        z = ground_h(x, y)
        posts.append(f"""Pose {{ translation {f(x, y, z + 6)} children [ Shape {{
  appearance {pbr(0.95, 0.35, 0.1, 0.5, 0, 0, (0.5, 0.15, 0.02))}
  geometry Cylinder {{ height 12 radius 0.8 subdivision 8 }} }} ] }}""")
        n += 1
    fh = 60.0
    w, h = xmax - xmin, ymax - ymin
    cx, cy = (xmin + xmax) / 2, (ymin + ymax) / 2
    wall = pbr(1.0, 0.35, 0.2, 1, 0, 0.93)
    walls = [((cx, ymin, fh / 2), (w, 1.0, fh)), ((cx, ymax, fh / 2 + 28), (w, 1.0, fh)),
             ((xmin, cy, fh / 2), (1.0, h, fh)), ((xmax, cy, fh / 2 + 14), (1.0, h, fh))]
    shapes = "\n".join(f"""Pose {{ translation {f(*p)} children [ Shape {{ appearance {wall}
  geometry Box {{ size {f(*s)} }} castShadows FALSE }} ] }}""" for p, s in walls)
    return f"""Solid {{
  children [
{chr(10).join(posts)}
{shapes}
  ]
  name "geofence"
}}
"""


# ------------------------------------------------------------------ main
HOUSES = ["SuburbanHouse", "BungalowStyleHouse", "SimpleTwoFloorsHouse", "HouseWithGarage",
          "ModernSuburbanHouse", "Barn", "Warehouse"]


def clear_spot(x, y, keep_out, min_d):
    return all(math.hypot(x - kx, y - ky) >= min_d for kx, ky in keep_out)


def main():
    rng = random.Random(11)
    gx, gy, gz = C.GCS_POS
    cam = (gx - 620.0, gy - 620.0, 820.0)
    look = (60.0, 60.0, 0.0)
    sites = {pid: (x, y) for pid, x, y, _ in C.POIS + [("H1", -600.0, 850.0, 2)]}
    village_xy = [(v[1], v[2]) for v in VILLAGES]

    out = ["#VRML_SIM R2025a utf8\n"]
    out += [f'EXTERNPROTO "{WEBOTS}/{p}"' for p in EXTERNPROTOS]
    out.append(f"""
WorldInfo {{
  info [
    "UAV-X: Resilient BVLOS Swarm Challenge -- Stage 1 proof of concept (v3, BVLOS scale)."
    "Earthquake + landslide in a rural district. 2 km x 2 km operating area, sites up to"
    "~2.4 km from the GCS, {C.NUM_UAVS} DJI Mavic 2 Pro UAVs, {C.COMM_RANGE:.0f} m air-to-air mesh radio."
    "Flight is kinematic (supervisor-driven), so gravity is off."
    "Generated by tools/make_world.py -- edit uavx/config.py, then regenerate."
  ]
  title "UAV-X Stage 1 PoC (BVLOS)"
  basicTimeStep {int(round(C.DT * 1000))}
  FPS 30
  gravity 0
  randomSeed {C.SEED}
}}
DEF VIEW Viewpoint {{
  orientation {f(*axis_angle_look(cam, look))}
  position {f(*cam)}
  near 1
  far 12000
}}
TexturedBackground {{
  texture "noon_cloudy_countryside"
}}
TexturedBackgroundLight {{
  texture "noon_cloudy_countryside"
  luminosity 1.1
}}
""")
    # ---- terrain: valley + ridge + mountains, then village and landslide overlays
    out.append(elevation("terrain", -2400, -2400, 121, 121, 40.0,
                         'Grass { type "maintained" colorOverride 0.8 0.86 0.7 '
                         'textureTransform TextureTransform { scale 600 600 } }'))
    for name, vx, vy, vr, _, _ in VILLAGES:
        ext = vr * 1.6
        n = int(2 * ext / 6.0) + 1
        out.append(elevation(f"ground {name}", vx - ext, vy - ext, n, n, 6.0,
                             'Soil { type "grey" color 0.82 0.78 0.72 '
                             'textureTransform TextureTransform { scale 20 20 } }',
                             mask=in_village, lift=0.06))
    out.append(elevation("landslide", 420, 590, 54, 49, 10.0,
                         "DryMud { textureTransform TextureTransform { scale 30 30 } }",
                         mask=in_landslide, lift=0.2))

    # ---- roads (with quake cracks)
    for k, (x1, y1, x2, y2) in enumerate(ROADS):
        L = math.hypot(x2 - x1, y2 - y1)
        yaw = math.atan2(y2 - y1, x2 - x1)
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2
        out.append(box(f"road_{k}", (mx, my, 0.12), (L + 7, 7, 0.12),
                       f"Asphalt {{ textureTransform TextureTransform {{ scale {L / 8:.0f} 1 }} }}",
                       rot=(0, 0, 1, yaw), shadows=False))
        for m, t in enumerate((0.2, 0.45, 0.7)):
            cx, cy = x1 + (x2 - x1) * t, y1 + (y2 - y1) * t
            out.append(box(f"crack_{k}_{m}", (cx, cy, 0.2), (rng.uniform(4, 10), 0.4, 0.02),
                           pbr(0.05, 0.05, 0.05, 1), rot=(0, 0, 1, yaw + rng.uniform(-0.7, 0.7)),
                           shadows=False))

    # ---- farmland (ploughed and fallow fields) between villages
    fields, tries = 0, 0
    tones = ['Soil { type "braun" color 0.9 0.8 0.7 textureTransform TextureTransform { scale 12 8 } }',
             'Grass { type "mossy" colorOverride 0.85 0.8 0.55 textureTransform TextureTransform { scale 15 10 } }']
    while fields < 22 and tries < 4000:
        tries += 1
        x, y = rng.uniform(-960, 960), rng.uniform(-960, 560)
        if in_village(x, y) or near_road(x, y, 70) or math.hypot(x - gx, y - gy) < 220:
            continue
        if not clear_spot(x, y, village_xy, 200):
            continue
        w, d = rng.uniform(90, 170), rng.uniform(60, 120)
        out.append(box(f"field_{fields}", (x, y, 0.05), (w, d, 0.06), rng.choice(tones),
                       rot=(0, 0, 1, rng.uniform(-0.5, 0.5)), shadows=False))
        fields += 1

    # ---- villages: damaged houses and collapsed buildings
    for name, vx, vy, vr, n_houses, n_piles in VILLAGES:
        keep = [sites[name]] if name in sites else []
        placed = []
        tries = 0
        while len(placed) < n_houses and tries < 500:
            tries += 1
            a, r = rng.uniform(0, 2 * math.pi), rng.uniform(15, vr)
            x, y = vx + math.cos(a) * r, vy + math.sin(a) * r
            if not clear_spot(x, y, keep, 34) or not clear_spot(x, y, placed, 24) or near_road(x, y, 11):
                continue
            placed.append((x, y))
            proto = rng.choice(HOUSES)
            damaged = name in sites and rng.random() < 0.6
            rot = axis_angle_from_yaw_pitch(rng.uniform(0, 6.28),
                                            rng.uniform(0.02, 0.08) if damaged else 0.0,
                                            rng.uniform(-0.05, 0.05) if damaged else 0.0)
            out.append(f"""{proto} {{
  translation {f(x, y, ground_h(x, y) - (0.4 if damaged else 0.0))}
  rotation {f(*rot)}
  name "house_{name}_{len(placed)}"
}}
""")
        k = 0
        while k < n_piles and tries < 1500:
            tries += 1
            a, r = rng.uniform(0, 2 * math.pi), rng.uniform(15, vr)
            x, y = vx + math.cos(a) * r, vy + math.sin(a) * r
            if not clear_spot(x, y, keep, 34) or not clear_spot(x, y, placed, 20) or near_road(x, y, 11):
                continue
            placed.append((x, y))
            out.append(collapsed_pile(f"collapse_{name}_{k}", x, y, rng, size=rng.uniform(0.9, 1.3)))
            k += 1

    # ---- search sites: a collapsed building right behind the survivors
    counts = C.SURVIVORS_GT        # ground truth, also used to score perception
    for pid, (x, y) in sites.items():
        d = math.hypot(x - gx, y - gy) or 1
        out.append(collapsed_pile(f"collapse_site_{pid}", x + (x - gx) / d * 12,
                                  y + (y - gy) / d * 12, rng, size=1.2))
        out.append(survivors(pid, x, y, rng, counts.get(pid, 2)))

    # ---- landslide debris: boulders, uprooted trees, buried houses
    k = 0
    while k < 45:
        x, y = rng.uniform(430, 940), rng.uniform(600, 1060)
        if not in_landslide(x, y):
            continue
        out.append(f"""Rock {{
  translation {f(x, y, ground_h(x, y) + 0.3)}
  rotation 0 0 1 {rng.uniform(0, 6.28):.2f}
  name "boulder_{k}"
  scale {rng.uniform(3, 9):.2f}
}}
""")
        k += 1
    for k in range(12):
        while True:
            x, y = rng.uniform(450, 920), rng.uniform(620, 1040)
            if in_landslide(x, y):
                break
        rot = axis_angle_from_yaw_pitch(rng.uniform(0, 6.28), 1.45)
        out.append(f"""Pine {{
  translation {f(x, y, ground_h(x, y) + 0.5)}
  rotation {f(*rot)}
  name "fallen pine {k}"
  enableBoundingObject FALSE
}}
""")
    for k, (x, y) in enumerate([(690, 760), (610, 700)]):
        rot = axis_angle_from_yaw_pitch(rng.uniform(0, 6.28), 0.2, 0.12)
        out.append(f"""SuburbanHouse {{
  translation {f(x, y, ground_h(x, y) - 2.5)}
  rotation {f(*rot)}
  name "buried house {k}"
}}
""")

    # ---- wrecked vehicles on the roads
    for k in range(7):
        x1, y1, x2, y2 = ROADS[rng.randrange(len(ROADS))]
        t = rng.uniform(0.15, 0.85)
        x, y = x1 + (x2 - x1) * t, y1 + (y2 - y1) * t
        if not clear_spot(x, y, list(sites.values()), 40):
            continue
        yaw = math.atan2(y2 - y1, x2 - x1) + rng.uniform(-0.6, 0.6)
        roll = rng.choice([0.0, 0.0, 1.57, 3.14])
        z = 1.4 if roll > 3 else (1.0 if roll > 1 else 0.5)
        col = rng.choice([(0.8, 0.8, 0.8), (0.1, 0.2, 0.5), (0.6, 0.1, 0.1), (0.15, 0.15, 0.15)])
        out.append(f"""ToyotaPriusSimple {{
  translation {f(x, y, z)}
  rotation {f(*axis_angle_from_yaw_pitch(yaw, 0.0, roll))}
  color {f(*col)}
  name "wreck_{k}"
}}
""")

    # ---- forest patches on the valley floor
    k, tries = 0, 0
    while k < 7 and tries < 3000:
        tries += 1
        x, y = rng.uniform(-900, 900), rng.uniform(-900, 520)
        if in_village(x, y) or near_road(x, y, 60) or math.hypot(x - gx, y - gy) < 250:
            continue
        if not clear_spot(x, y, village_xy, 220):
            continue
        pts = []
        for m in range(7):
            a = m / 7 * 2 * math.pi
            r = rng.uniform(55, 110)
            pts.append(f"{math.cos(a) * r:.0f} {math.sin(a) * r:.0f}")
        out.append(f"""Forest {{
  translation {f(x, y, ground_h(x, y))}
  shape [ {", ".join(pts)} ]
  density 0.012
  type "random"
  randomSeed {k + 1}
  groundTexture []
  maxHeight 16
  minHeight 8
  maxRadius 4
  minRadius 2
}}
""")
        k += 1

    # ---- trees on the surrounding mountains
    k = 0
    while k < 160:
        a = rng.uniform(0, math.pi * 2)
        r = rng.uniform(1060, 2000)
        x, y = math.cos(a) * r, math.sin(a) * r
        if max(abs(x), abs(y)) < 1050 or math.hypot(x - gx, y - gy) < 150:
            continue
        z = ground_h(x, y) - 0.3
        if rng.random() < 0.7:
            out.append(f"""Pine {{
  translation {f(x, y, z)}
  rotation 0 0 1 {rng.uniform(0, 6.28):.2f}
  name "tree_{k}"
  enableBoundingObject FALSE
}}
""")
        else:
            s_ = rng.uniform(2.5, 4.0)
            out.append(f"""Tree {{
  translation {f(x, y, z)}
  name "tree_{k}"
  scale {f(s_ * 1.2, s_ * 1.2, s_ * 3.2)}
}}
""")
        k += 1

    out.append(geofence())
    out.append(gcs_compound())
    for i in range(1, C.NUM_UAVS + 1):
        out.append(pad(i))
    for pid, x, y, _ in C.POIS:
        out.append(poi_marker(pid, x, y))
    for i in range(1, C.NUM_UAVS + 1):
        out.append(drone(i))
        out.append(mast(i))
    for a in range(0, C.NUM_UAVS + 1):
        for b in range(a + 1, C.NUM_UAVS + 1):
            out.append(link(a, b))

    wdir = os.path.join(ROOT, "worlds")
    path = os.path.join(wdir, f"{WORLD_NAME}.wbt")
    with open(path, "w", newline="\n") as fh:
        fh.write("\n".join(out))
    # Hide the per-camera overlays Webots would otherwise draw over the 3D
    # view (the feeds are shown in the GCS dashboard window instead).
    proj = os.path.join(wdir, f".{WORLD_NAME}.wbproj")
    if os.path.exists(proj):
        os.remove(proj)          # Webots marks it hidden; Windows won't overwrite that in place
    with open(proj, "w", newline="\n") as fh:
        fh.write("Webots Project File version R2025a\n")
        for i in range(1, C.NUM_UAVS + 1):
            fh.write(f"renderingDevicePerspectives: gcs:cam_{i};0;1;0;0\n")
    print(f"wrote {path}")


if __name__ == "__main__":
    main()
