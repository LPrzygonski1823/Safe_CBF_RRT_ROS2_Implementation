#!/usr/bin/env python3
"""
Planned vs driven path of the runs recorded by record_execution.py (README_EXPERIMENTS.md, §5).

    python3 src/cbf_rrt_planner/tools/plot_execution.py [DIR] --map maps/map_u.yaml [--label cbf]

For every <label>_<n>_plan.csv + <label>_<n>_driven.csv in DIR (default
src/cbf_rrt_planner/benchmark/experiments/execution) writes <label>_<n>.png: the map, the plan,
the driven trajectory with the robot footprint (real heading) every 0.3 m and the pose of the
smallest clearance. summary.csv (also printed) holds per run:

  plan_fp_clearance    minimum footprint clearance of the plan, footprint along the path tangent
                       (the same metric as min_footprint_clearance of cbf_benchmark / planner_node)
  driven_fp_clearance  the same, but for the driven poses with their recorded heading
  max_dev, mean_dev    distance of the driven positions from the plan polyline
  goal_error           distance of the last driven position from the end of the plan
  plan_length, driven_length [m], duration [s]

The driven trajectory is the robot's TF pose estimate, not ground truth. Clearances are measured
on the map without inflation; unknown cells count as obstacles. Needs numpy, matplotlib, Pillow.
"""

import argparse
import csv
import glob
import math
import os

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from PIL import Image

from plot_benchmark import footprint_poses, load_map, rectangle


# ---------------------------------------------------------------------------------------------
# map and clearance: ports of map_loader.cpp and path_clearance.cpp

class Grid:
    def __init__(self, yaml_path, occupied_threshold=65):
        meta = {}
        with open(yaml_path) as f:
            for line in f:
                line = line.split("#", 1)[0]
                if ":" in line:
                    k, v = line.split(":", 1)
                    if v.strip():
                        meta[k.strip()] = v.strip().strip("'\"")
        image = meta["image"]
        if not os.path.isabs(image):
            image = os.path.join(os.path.dirname(yaml_path), image)
        img = Image.open(image)
        maxval = 65535.0 if img.mode.startswith("I") else 255.0
        pixels = np.asarray(img if img.mode.startswith("I") else img.convert("L"), dtype=float)
        mode = meta.get("mode", "trinary")
        if mode == "raw":
            cells = np.where(pixels <= 100, pixels, -1)
        else:
            shade = pixels / maxval
            occ = shade if int(meta.get("negate", "0")) else 1.0 - shade
            ot, ft = float(meta["occupied_thresh"]), float(meta["free_thresh"])
            middle = np.rint((occ - ft) / (ot - ft) * 100.0) if mode == "scale" else -1
            cells = np.where(occ > ot, 100, np.where(occ < ft, 0, middle))
        cells = cells[::-1]                               # row 0 = bottom of the map, as in C++
        self.obstacle = (cells < 0) | (cells >= occupied_threshold)
        self.res = float(meta["resolution"])
        self.ox, self.oy = [float(v) for v in meta["origin"].strip("[]").split(",")[:2]]
        self.height, self.width = cells.shape


def box_distance(px, py, hx, hy):
    """signed distance from points (in the box frame) to a centred box, < 0 inside"""
    qx, qy = np.abs(px) - hx, np.abs(py) - hy
    return np.where((qx > 0) | (qy > 0), np.hypot(np.maximum(qx, 0), np.maximum(qy, 0)), np.maximum(qx, qy))


def pose_clearance(g, x, y, yaw, length, width, radius):
    """minimum distance from the footprint (length x width, centred at x, y, heading yaw) to an
    obstacle cell, capped at radius"""
    half_l, half_w, half_c = length / 2, width / 2, g.res / 2
    span = int(math.ceil((radius + math.hypot(half_l, half_w)) / g.res)) + 1
    col0, row0 = int(math.floor((x - g.ox) / g.res)), int(math.floor((y - g.oy) / g.res))
    r0, r1 = max(0, row0 - span), min(g.height - 1, row0 + span)
    c0, c1 = max(0, col0 - span), min(g.width - 1, col0 + span)
    if r0 > r1 or c0 > c1:
        return radius
    rr, cc = np.nonzero(g.obstacle[r0:r1 + 1, c0:c1 + 1])
    if not len(rr):
        return radius
    dx = g.ox + (cc + c0 + 0.5) * g.res - x
    dy = g.oy + (rr + r0 + 0.5) * g.res - y
    cos, sin = math.cos(yaw), math.sin(yaw)
    best = np.full(dx.shape, np.inf)
    for k in range(4):
        # cell corner in the robot frame vs the footprint, footprint corner vs the cell
        cx = dx + (half_c if k & 1 else -half_c)
        cy = dy + (half_c if k & 2 else -half_c)
        best = np.minimum(best, box_distance(cos * cx + sin * cy, -sin * cx + cos * cy, half_l, half_w))
        lx, ly = (half_l if k & 1 else -half_l), (half_w if k & 2 else -half_w)
        best = np.minimum(best, box_distance(cos * lx - sin * ly - dx, sin * lx + cos * ly - dy, half_c, half_c))
    return min(radius, float(best.min()))


def plan_clearance(g, path, length, width, radius):
    """min footprint clearance along a polyline, heading = segment tangent; every segment is
    sampled every map resolution including both ends (as computePathClearance)"""
    best, yaw = radius, 0.0
    for (x1, y1), (x2, y2) in zip(path[:-1], path[1:]):
        seg = math.hypot(x2 - x1, y2 - y1)
        if seg > 1e-9:
            yaw = math.atan2(y2 - y1, x2 - x1)
        n = max(1, math.ceil(seg / g.res))
        for s in range(n + 1):
            t = s / n
            best = min(best, pose_clearance(g, x1 + t * (x2 - x1), y1 + t * (y2 - y1), yaw, length, width, radius))
    return best


# ---------------------------------------------------------------------------------------------

def distance_to_polyline(points, path):
    a, b = path[:-1], path[1:]
    ab = b - a
    denom = np.maximum((ab ** 2).sum(1), 1e-12)
    t = np.clip(((points[:, None, :] - a[None]) * ab[None]).sum(2) / denom[None], 0, 1)
    closest = a[None] + t[..., None] * ab[None]
    return np.linalg.norm(points[:, None, :] - closest, axis=2).min(1)


def length_of(p):
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


def read(path, cols):
    with open(path, newline="") as f:
        return np.array([[float(r[c]) for c in cols] for r in csv.DictReader(f)]).reshape(-1, len(cols))


def every(driven, step):
    """indices of the driven poses spaced at least 'step' metres along the trajectory"""
    keep, acc = [0], 0.0
    for i in range(1, len(driven)):
        acc += math.hypot(*(driven[i, :2] - driven[i - 1, :2]))
        if acc >= step:
            keep.append(i)
            acc = 0.0
    return keep


def plot_run(name, plan, driven, clear, m, shown, extent, args, out):
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.imshow(shown, cmap="gray", vmin=0, vmax=1, extent=extent, origin="upper", interpolation="nearest")
    for i in every(driven, args.footprint_step):
        x, y, yaw = driven[i]
        ax.add_patch(Polygon(rectangle(x, y, yaw, *args.footprint), closed=True, facecolor="tab:orange",
                             edgecolor="tab:orange", alpha=0.18, linewidth=0.6))
    ax.plot(plan[:, 0], plan[:, 1], color="tab:blue", linewidth=2, label="plan (/planned_path)")
    ax.plot(driven[:, 0], driven[:, 1], color="tab:orange", linewidth=1.6,
            label=f"driven (TF {args.base_frame}), footprint every {args.footprint_step:g} m")
    i = int(np.argmin(clear))
    ax.add_patch(Polygon(rectangle(*driven[i], *args.footprint), closed=True, fill=False,
                         edgecolor="red", linewidth=1.6, label=f"smallest clearance {clear[i]:.3f} m"))
    ax.plot(*plan[0], "go", markersize=9, label="start")
    ax.plot(*plan[-1], "r*", markersize=15, label="goal")
    pts = np.vstack([plan, driven[:, :2]])
    ax.set_xlim(pts[:, 0].min() - 1, pts[:, 0].max() + 1)
    ax.set_ylim(pts[:, 1].min() - 1, pts[:, 1].max() + 1)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m] (map frame)")
    ax.set_ylabel("y [m]")
    ax.set_title(f"{name}: footprint clearance plan {m['plan_fp_clearance']:.3f} m, driven "
                 f"{m['driven_fp_clearance']:.3f} m; deviation max {m['max_dev']:.3f} m, mean "
                 f"{m['mean_dev']:.3f} m; goal error {m['goal_error']:.3f} m", fontsize=8)
    ax.legend(fontsize=8, loc="best")
    fig.tight_layout()
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dir", nargs="?", default=os.path.join("src", "cbf_rrt_planner", "benchmark", "experiments", "execution"))
    ap.add_argument("--map", required=True, help="map yaml the robot drove on (without inflation)")
    ap.add_argument("--label", help="only runs with this label")
    ap.add_argument("--base-frame", default="base_link", help="only for the legend")
    ap.add_argument("--footprint", type=float, nargs=2, default=[0.33, 0.28], metavar=("LENGTH", "WIDTH"))
    ap.add_argument("--footprint-step", type=float, default=0.3, help="spacing of drawn footprints [m]")
    ap.add_argument("--search-radius", type=float, default=2.0, help="clearances are capped at this [m]")
    args = ap.parse_args()

    grid = Grid(args.map)
    shown, extent = load_map(args.map)
    runs = sorted(f[:-len("_plan.csv")] for f in glob.glob(os.path.join(args.dir, "*_plan.csv")))
    if args.label:
        runs = [r for r in runs if os.path.basename(r).rsplit("_", 1)[0] == args.label]
    rows = []
    for run in runs:
        name = os.path.basename(run)
        plan = read(run + "_plan.csv", ["x", "y"])
        driven_t = read(run + "_driven.csv", ["t", "x", "y", "yaw"]) if os.path.exists(run + "_driven.csv") else np.empty((0, 4))
        if len(plan) < 2 or len(driven_t) < 2:
            print(f"{name}: plan or driven trajectory missing/too short - skipped")
            continue
        driven = driven_t[:, 1:]
        L, W = args.footprint
        clear = np.array([pose_clearance(grid, x, y, yaw, L, W, args.search_radius) for x, y, yaw in driven])
        dev = distance_to_polyline(driven[:, :2], plan)
        m = {
            "run": name,
            "plan_length": length_of(plan),
            "driven_length": length_of(driven[:, :2]),
            "duration": driven_t[-1, 0] - driven_t[0, 0],
            "plan_fp_clearance": plan_clearance(grid, plan, L, W, args.search_radius),
            "driven_fp_clearance": float(clear.min()),
            "max_dev": float(dev.max()),
            "mean_dev": float(dev.mean()),
            "goal_error": float(np.linalg.norm(driven[-1, :2] - plan[-1])),
        }
        rows.append(m)
        plot_run(name, plan, driven, clear, m, shown, extent, args, run + ".png")

    if not rows:
        raise SystemExit(f"no recorded runs in {args.dir}")
    cols = list(rows[0])
    with open(os.path.join(args.dir, "summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows({k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()} for r in rows)
    print(f"{'run':<16} {'plan cl':>8} {'driven cl':>9} {'max dev':>8} {'mean dev':>8} {'goal err':>8} "
          f"{'plan L':>7} {'driven L':>8} {'time':>6}")
    for r in rows:
        print(f"{r['run']:<16} {r['plan_fp_clearance']:>8.3f} {r['driven_fp_clearance']:>9.3f} {r['max_dev']:>8.3f} "
              f"{r['mean_dev']:>8.3f} {r['goal_error']:>8.3f} {r['plan_length']:>7.2f} {r['driven_length']:>8.2f} "
              f"{r['duration']:>6.1f}")
    print(f"(metres / seconds; figures and summary.csv in {args.dir})")


if __name__ == "__main__":
    main()
