"""Perception benchmark (run inside Webots with UAVX_PERCEPTION_BENCH=<dir>).

For every site (P1-P7, H1), 3 hover altitudes (60/76/92 m) and 3 viewing
azimuths (-30/0/+30 deg around the survey standoff), the gimbal camera takes
5 jittered frames exactly as it would during a survey hover. Each frame goes
through the production detector and ground projection. The sequence is
scored with three counting rules and, for the production rule, matched
person-by-person against the survivors' true positions (evaluator only).
Writes <dir>/perception_bench.json and annotated frames <dir>/*.jpg.
"""

import json
import math
import os
import random

from PIL import ImageDraw

from perception import DET_CONF, bench_score, project, world_survivors


def run(sup, step_ms, frame_steps, cams, det, out_dir, root, C, ground_h, axis_angle_from_yaw_pitch,
        drone_z_offset, say):
    os.makedirs(out_dir, exist_ok=True)
    truth = world_survivors(os.path.join(root, "worlds", "uavx_stage3.wbt"))
    sites = {p[0]: (p[1], p[2]) for p in C.POIS}
    sites["H1"] = (-600.0, 850.0)
    gx, gy, _ = C.GCS_POS
    tr, rot, fovf = cams.f[1]
    rng = random.Random(3)
    rows = []
    for pid, (x, y) in sites.items():
        zg = ground_h(x, y)
        d = math.hypot(gx - x, gy - y) or 1.0
        ux, uy = (gx - x) / d, (gy - y) / d
        for alt in (60.0, 76.0, 92.0):
            for az in (-30, 0, 30):
                ca, sa = math.cos(math.radians(az)), math.sin(math.radians(az))
                rx, ry = ux * ca - uy * sa, ux * sa + uy * ca
                s = 0.8 * alt
                frames = []
                raw = {"iou0.70": [], "iou0.90": []}
                for k in range(5):
                    hx = x + rx * s + rng.uniform(-1.0, 1.0)
                    hy = y + ry * s + rng.uniform(-1.0, 1.0)
                    cz = alt + drone_z_offset - 0.9
                    dx, dy, dz = x - hx, y - hy, zg + 1.0 - cz
                    rngd = math.sqrt(dx * dx + dy * dy + dz * dz)
                    yaw, pitch = math.atan2(dy, dx), math.atan2(-dz, math.hypot(dx, dy))
                    fov = max(0.05, min(0.9, 2 * math.atan(6.0 / rngd)))
                    tr.setSFVec3f([hx - gx, hy - gy, cz])
                    rot.setSFRotation(list(axis_angle_from_yaw_pitch(yaw, pitch)))
                    fovf.setSFFloat(fov)
                    for _ in range(frame_steps + 1):
                        sup.step(step_ms)
                    rgb = cams.frame(1)
                    for tag, iou in (("iou0.70", 0.70), ("iou0.90", 0.90)):
                        dd_ = [dd for dd in det.detect(rgb, conf=DET_CONF, iou=iou) if dd[0] == "person"]
                        raw[tag].append([[round(c, 3) for c in p] + [round(dd[1], 3)]
                                         for dd in dd_
                                         for p in [project(dd[2:6], (hx, hy, cz), yaw, pitch, fov, zg)]
                                         if p is not None])
                    dets = [dd for dd in det.detect(rgb, conf=DET_CONF) if dd[0] == "person"]
                    pts = [project(dd[2:6], (hx, hy, cz), yaw, pitch, fov, zg) for dd in dets]
                    frames.append(pts)
                    if k == 0 and az == 0 and alt == 76.0:
                        from PIL import Image
                        im = Image.fromarray(rgb)
                        dr = ImageDraw.Draw(im)
                        for dd in dets:
                            dr.rectangle(dd[2:6], outline=(60, 255, 120), width=2)
                            dr.text((dd[2], dd[3] + 2), f"{dd[1]:.2f}", fill=(60, 255, 120))
                        im.save(os.path.join(out_dir, f"{pid}_alt76_az0.jpg"), quality=90)
                res = bench_score(frames, (x, y), truth.get(pid, []))
                res.update(site=pid, alt=alt, az=az, boxes_per_frame=[len(f) for f in frames],
                           raw_points=raw, truth_xy=truth.get(pid, []), site_xy=[x, y])
                rows.append(res)
                say(f"bench {pid} alt {alt:.0f} az {az:+d}: truth {res['truth']} | max-any "
                    f"{res['rule_max_any_box']} | max-assoc {res['rule_max_associated']} | tracks "
                    f"{res['rule_static_tracks']} (tp {res['tp']} fp {res['fp']} fn {res['fn']})")
    summary = {}
    for rule in ("rule_max_any_box", "rule_max_associated", "rule_static_tracks"):
        exact = sum(1 for r in rows if r[rule] == r["truth"])
        err = [r[rule] - r["truth"] for r in rows]
        summary[rule] = {"views": len(rows), "exact_count_views": exact,
                         "mean_abs_count_error": round(sum(abs(e) for e in err) / len(err), 3),
                         "overcount_views": sum(1 for e in err if e > 0),
                         "undercount_views": sum(1 for e in err if e < 0)}
    tp = sum(r["tp"] for r in rows)
    fp = sum(r["fp"] for r in rows)
    fn = sum(r["fn"] for r in rows)
    summary["production_matched"] = {
        "persons_true": tp + fn, "tp": tp, "fp": fp, "fn": fn,
        "precision": round(tp / (tp + fp), 3) if tp + fp else None,
        "recall": round(tp / (tp + fn), 3) if tp + fn else None,
        "note": "static-track rule, tracks matched to true survivor positions within 1.5 m"}
    with open(os.path.join(out_dir, "perception_bench.json"), "w") as f:
        json.dump({"summary": summary, "views": rows}, f, indent=1)
    say(json.dumps(summary, indent=1))
    return summary
