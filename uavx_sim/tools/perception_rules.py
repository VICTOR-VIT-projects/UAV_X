"""Choose and validate the onboard person-counting rule from the recorded
perception benchmark (results/perception/perception_bench.json).

    python tools/perception_rules.py

Split: parameters are chosen on the 76 m views only ("tune") and reported
on the 60 m and 92 m views ("validate"), which were not used for the
choice. Scoring matches confirmed tracks to true survivor positions
(evaluator only) within 1.5 m: TP / FP / FN per person, plus exact-count
rate per view. Writes results/perception/rule_selection.json.
"""

import itertools
import json
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "controllers", "swarm_supervisor"))
import perception as P                                     # noqa: E402

BENCH = os.path.join(ROOT, "results", "perception", "perception_bench.json")


def score(views, iou, track_r, support, rule):
    tp = fp = fn = exact = 0
    for v in views:
        frames = [[tuple(p[:2]) for p in f] for f in v["raw_points"][iou]]
        truth = [tuple(t) for t in v["truth_xy"]]
        if rule == "max_frame":
            n = max(len([p for p in f if math.hypot(p[0] - v["site_xy"][0],
                                                     p[1] - v["site_xy"][1]) <= P.ASSOC_R])
                    for f in frames)
            exact += n == len(truth)
            tp += min(n, len(truth))
            fp += max(0, n - len(truth))
            fn += max(0, len(truth) - n)
            continue
        old = (P.TRACK_R, P.TRACK_SUPPORT)
        P.TRACK_R, P.TRACK_SUPPORT = track_r, support
        r = P.bench_score(frames, tuple(v["site_xy"]), truth)
        P.TRACK_R, P.TRACK_SUPPORT = old
        tp, fp, fn = tp + r["tp"], fp + r["fp"], fn + r["fn"]
        exact += r["rule_static_tracks"] == r["truth"]
    return {"views": len(views), "exact_views": exact, "tp": tp, "fp": fp, "fn": fn,
            "precision": round(tp / (tp + fp), 3) if tp + fp else None,
            "recall": round(tp / (tp + fn), 3) if tp + fn else None}


def main():
    d = json.load(open(BENCH))
    views = d["views"]
    tune = [v for v in views if v["alt"] == 76.0]
    val = [v for v in views if v["alt"] != 76.0]
    grid = []
    for iou, tr, sup in itertools.product(("iou0.70", "iou0.90"), (0.3, 0.5, 1.0),
                                          (0.2, 0.4, 0.6)):
        s = score(tune, iou, tr, sup, "tracks")
        grid.append({"iou": iou, "track_r": tr, "support": sup, "tune": s})
    # Choose: fewest count errors on the tuning views, then fewest FP (a
    # false person is worse than a miss for tasking), then the default-like.
    best = min(grid, key=lambda g: (g["tune"]["fp"] + g["tune"]["fn"], g["tune"]["fp"],
                                    abs(g["support"] - 0.4)))
    out = {"split": "tune = 76 m views; validate = 60 m and 92 m views",
           "grid_on_tune": grid, "chosen": {k: best[k] for k in ("iou", "track_r", "support")},
           "chosen_validate": score(val, best["iou"], best["track_r"], best["support"], "tracks"),
           "baselines_validate": {
               "old rule: max boxes in one frame (iou 0.70, associated)":
                   score(val, "iou0.70", 0, 0, "max_frame"),
               "tracks with the v3.1 defaults (iou 0.70, 1.0 m, 0.4)":
                   score(val, "iou0.70", 1.0, 0.4, "tracks")},
           "chosen_all_views": score(views, best["iou"], best["track_r"], best["support"], "tracks")}
    with open(os.path.join(ROOT, "results", "perception", "rule_selection.json"), "w") as f:
        json.dump(out, f, indent=1)
    print(json.dumps({k: v for k, v in out.items() if k != "grid_on_tune"}, indent=1))


if __name__ == "__main__":
    main()
