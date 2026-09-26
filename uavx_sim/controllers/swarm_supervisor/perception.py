"""Onboard person counting: ground projection, site association and a
static-person track counter; plus the offline perception benchmark.

Production rule (used onboard, docs/PERCEPTION.md):
  1. YOLOv8n person boxes (conf >= DET_CONF) on the UAV's own gimbal frame.
  2. Each box's bottom-centre is projected through the known camera pose
     onto the local ground plane.
  3. Association: only points within ASSOC_R of the site count.
  4. Tracking: survivors are static, so points from successive frames that
     fall within TRACK_R of an existing track join it.
  5. Site count = number of tracks seen in at least TRACK_SUPPORT of the
     frames processed during the hover ("unique static tracks").
Scene ground truth is NEVER used here; it is read only by bench_score().
"""

import math
import re

DET_CONF = 0.30
ASSOC_R = 8.0         # m around the site centre
TRACK_R = 0.3         # m: chosen on the 76 m bench views (tools/perception_rules.py)
TRACK_SUPPORT = 0.2   # fraction of hover frames a track must appear in (same selection)


def project(box, cam_pos, yaw, pitch, fov, ground_z, w=640, h=480):
    """Ground point of the bottom-centre of `box` (x1, y1, x2, y2) seen from
    a camera at cam_pos with the given yaw/pitch (pitch > 0 looks down) and
    horizontal field of view. None if the ray doesn't hit the ground."""
    u = (box[0] + box[2]) / 2.0
    v = box[3]
    fx = (w / 2.0) / math.tan(fov / 2.0)
    cy_, sy_ = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    fwd = (cp * cy_, cp * sy_, -sp)
    left = (-sy_, cy_, 0.0)
    up = (sp * cy_, sp * sy_, cp)
    a, b = -(u - w / 2.0) / fx, -(v - h / 2.0) / fx
    ray = [fwd[i] + left[i] * a + up[i] * b for i in range(3)]
    if ray[2] >= -1e-6:
        return None
    t = (ground_z - cam_pos[2]) / ray[2]
    return (cam_pos[0] + ray[0] * t, cam_pos[1] + ray[1] * t)


class SiteCounter:
    """Accumulates associated ground points for one (UAV, site) survey."""

    def __init__(self, site_xy):
        self.site = site_xy
        self.frames = 0
        self.tracks = []          # [x, y, hits]
        self.max_frame = 0        # legacy rule: max associated boxes in one frame

    def add_frame(self, points):
        self.frames += 1
        near = [p for p in points if p is not None
                and math.hypot(p[0] - self.site[0], p[1] - self.site[1]) <= ASSOC_R]
        self.max_frame = max(self.max_frame, len(near))
        used = set()
        for p in near:
            best, bd = None, TRACK_R
            for i, tr in enumerate(self.tracks):
                if i in used:
                    continue
                d = math.hypot(p[0] - tr[0], p[1] - tr[1])
                if d < bd:
                    best, bd = i, d
            if best is None:
                self.tracks.append([p[0], p[1], 1])
                used.add(len(self.tracks) - 1)
            else:
                tr = self.tracks[best]
                n = tr[2]
                tr[0] = (tr[0] * n + p[0]) / (n + 1)
                tr[1] = (tr[1] * n + p[1]) / (n + 1)
                tr[2] = n + 1
                used.add(best)

    def confirmed(self):
        need = max(1, math.ceil(TRACK_SUPPORT * self.frames))
        return [t for t in self.tracks if t[2] >= need]

    def count(self):
        return len(self.confirmed())


# ------------------------------------------------------------- benchmark
def world_survivors(world_path):
    """Evaluator-only: survivor positions from the world file."""
    txt = open(world_path, encoding="utf-8").read()
    out = {}
    for x, y, name in re.findall(r'Pedestrian \{\s*translation ([-\d.e]+) ([-\d.e]+) [-\d.e]+'
                                 r'[^}]*?name "survivor_(\w+?)_\d+"', txt):
        out.setdefault(name, []).append((float(x), float(y)))
    return out


def bench_score(points_per_frame, site_xy, truth, match_r=1.5):
    """Score one view sequence three ways and match tracks to truth."""
    sc = SiteCounter(site_xy)
    raw_max = 0
    for pts in points_per_frame:
        raw_max = max(raw_max, len([p for p in pts if p is not None]))
        sc.add_frame(pts)
    tracks = sc.confirmed()
    unmatched = list(truth)
    tp = 0
    errs = []
    for t in sorted(tracks, key=lambda t: -t[2]):
        if not unmatched:
            break
        j = min(range(len(unmatched)), key=lambda k: math.hypot(t[0] - unmatched[k][0],
                                                                 t[1] - unmatched[k][1]))
        d = math.hypot(t[0] - unmatched[j][0], t[1] - unmatched[j][1])
        if d <= match_r:
            tp += 1
            errs.append(d)
            unmatched.pop(j)
    return {"truth": len(truth), "rule_max_any_box": raw_max, "rule_max_associated": sc.max_frame,
            "rule_static_tracks": len(tracks), "tp": tp, "fp": len(tracks) - tp,
            "fn": len(truth) - tp, "loc_err_m": round(sum(errs) / len(errs), 2) if errs else None}
