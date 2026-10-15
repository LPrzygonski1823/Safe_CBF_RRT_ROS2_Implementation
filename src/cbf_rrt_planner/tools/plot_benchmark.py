#!/usr/bin/env python3
"""
Plots of the offline benchmark (cbf_benchmark) results.

    python3 src/cbf_rrt_planner/tools/plot_benchmark.py src/cbf_rrt_planner/benchmark/results.csv

Writes to <results dir>/plots/:
  pareto.png         path length ratio vs minimum footprint clearance, one dot per plan
  boxplots.png       distributions per configuration + success rate
  paths_goal<N>.png  all paths to one goal over the map, with the robot footprint drawn along
                     the median-length path of every configuration (swept area)
  summary.csv        medians / quartiles per configuration (also printed)

A configuration is psf_source + inflation radius + (c value or baseline). Rows of several batches
(e.g. one run per PSF source) are combined; select some with --batch.
Needs numpy, matplotlib and Pillow (no pandas).
"""

import argparse
import csv
import math
import os

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from PIL import Image

NUMERIC = [
    "psf_boundary_flux", "inflation_radius", "safety_weight_c", "start_x", "start_y", "goal_x", "goal_y",
    "straight_dist",
    "success", "length", "length_ratio", "mean_h", "min_h", "min_clearance", "mean_clearance",
    "min_footprint_clearance", "iterations_to_first_solution", "rejection_ratio", "plan_time_s",
]


def load_rows(path, batches):
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            if batches and r["batch"] not in batches:
                continue
            for k in NUMERIC:
                r[k] = float(r[k]) if r[k] not in ("", None) else math.nan
            r["group"] = group_name(r)
            rows.append(r)
    if not rows:
        raise SystemExit(f"no rows in {path}" + (f" for batches {batches}" if batches else ""))
    return rows


def group_name(r):
    infl = r["inflation_radius"]
    infl_txt = f"infl {infl:.2f}" if infl > 0 else "no infl"
    cfg = "baseline" if r["config"] == "baseline" else f"c={r['safety_weight_c']:g}"
    return f"{r['psf_source']} | {infl_txt} | {cfg}"


def ordered_groups(rows):
    def key(g):
        src, infl, cfg = g.split(" | ")
        return (src, infl, cfg == "baseline", float(cfg[2:]) if cfg != "baseline" else 0.0)
    return sorted({r["group"] for r in rows}, key=key)


def values(rows, group, column):
    return np.array([r[column] for r in rows
                     if r["group"] == group and r["success"] == 1 and not math.isnan(r[column])])


def colors_for(groups):
    cmap = plt.get_cmap("tab10" if len(groups) <= 10 else "tab20")
    return {g: cmap(i % cmap.N) for i, g in enumerate(groups)}


def plot_pareto(rows, groups, colors, out):
    fig, ax = plt.subplots(figsize=(8, 6))
    for g in groups:
        x = values_paired(rows, g, "length_ratio", "min_footprint_clearance")
        if not len(x):
            continue
        ax.scatter(x[:, 0], x[:, 1], s=14, alpha=0.45, color=colors[g])
        ax.scatter(np.median(x[:, 0]), np.median(x[:, 1]), s=140, color=colors[g],
                   edgecolor="black", linewidth=1.2, label=g, zorder=3)
    ax.axhline(0.0, color="red", linestyle="--", linewidth=1, label="footprint touches an obstacle")
    ax.set_xlabel("path length / straight-line distance")
    ax.set_ylabel("minimum footprint clearance [m]")
    ax.set_title("Length vs safety (large dots: medians)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def values_paired(rows, group, cx, cy):
    pts = [(r[cx], r[cy]) for r in rows if r["group"] == group and r["success"] == 1
           and not math.isnan(r[cx]) and not math.isnan(r[cy])]
    return np.array(pts).reshape(-1, 2)


def plot_boxplots(rows, groups, colors, out):
    panels = [
        ("length_ratio", "length / straight-line distance"),
        ("min_clearance", "min centre clearance [m]"),
        ("mean_clearance", "mean centre clearance [m]"),
        ("min_footprint_clearance", "min footprint clearance [m]"),
        ("plan_time_s", "planning time [s]"),
    ]
    fig, axes = plt.subplots(2, 3, figsize=(16, 9))
    labels = [g.replace(" | ", "\n") for g in groups]
    for ax, (col, title) in zip(axes.flat, panels):
        data = [values(rows, g, col) for g in groups]
        bp = ax.boxplot([d if len(d) else [np.nan] for d in data], patch_artist=True, showfliers=True)
        for patch, g in zip(bp["boxes"], groups):
            patch.set_facecolor(colors[g])
            patch.set_alpha(0.6)
        ax.set_title(title)
        ax.set_xticks(range(1, len(groups) + 1))
        ax.set_xticklabels(labels, fontsize=7)
        ax.grid(axis="y", alpha=0.3)
        if col == "min_footprint_clearance":
            ax.axhline(0.0, color="red", linestyle="--", linewidth=1)

    ax = axes.flat[5]
    rates = []
    for g in groups:
        tried = [r for r in rows if r["group"] == g]
        rates.append(100.0 * sum(r["success"] for r in tried) / len(tried))
    ax.bar(range(1, len(groups) + 1), rates, color=[colors[g] for g in groups], alpha=0.8)
    ax.set_ylim(0, 105)
    ax.set_title("success rate [%]")
    ax.set_xticks(range(1, len(groups) + 1))
    ax.set_xticklabels(labels, fontsize=7)
    ax.grid(axis="y", alpha=0.3)

    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def write_summary(rows, groups, out):
    cols = ["length_ratio", "length", "min_clearance", "mean_clearance", "min_footprint_clearance",
            "mean_h", "min_h", "iterations_to_first_solution", "plan_time_s"]
    header = ["group", "plans", "success_rate", "footprint_contacts"]
    for c in cols:
        header += [f"{c}_median", f"{c}_q1", f"{c}_q3"]
    lines = []
    for g in groups:
        tried = [r for r in rows if r["group"] == g]
        ok = [r for r in tried if r["success"] == 1]
        line = [g, len(tried), round(len(ok) / len(tried), 4),
                sum(1 for r in ok if r["min_footprint_clearance"] <= 0.0)]
        for c in cols:
            v = values(rows, g, c)
            line += [round(float(np.percentile(v, p)), 4) if len(v) else "" for p in (50, 25, 75)]
        lines.append(line)
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(lines)

    print(f"{'configuration':<38} {'plans':>5} {'succ':>5} {'len ratio':>9} {'min cl':>7} "
          f"{'mean cl':>7} {'fp cl':>7} {'contacts':>8}")
    for line in lines:
        med = dict(zip(header, line))
        print(f"{line[0]:<38} {line[1]:>5} {line[2]:>5.2f} "
              f"{fmt(med['length_ratio_median']):>9} {fmt(med['min_clearance_median']):>7} "
              f"{fmt(med['mean_clearance_median']):>7} {fmt(med['min_footprint_clearance_median']):>7} "
              f"{line[3]:>8}")
    print("(medians; clearances in m; mean_h / min_h in summary.csv are comparable only within one PSF source)")


def fmt(v):
    return f"{v:.3f}" if v != "" else "-"


def load_map(yaml_path):
    meta = {}
    with open(yaml_path) as f:
        for line in f:
            line = line.split("#", 1)[0]
            if ":" in line:
                k, v = line.split(":", 1)
                meta[k.strip()] = v.strip().strip("'\"")
    image = meta["image"]
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(yaml_path), image)
    pixels = np.asarray(Image.open(image).convert("L"), dtype=float) / 255.0
    occ = pixels if int(meta.get("negate", "0")) else 1.0 - pixels
    shown = np.full(occ.shape, 0.75)                       # unknown: grey
    shown[occ > float(meta["occupied_thresh"])] = 0.0      # occupied: black
    shown[occ < float(meta["free_thresh"])] = 1.0          # free: white
    res = float(meta["resolution"])
    ox, oy = [float(v) for v in meta["origin"].strip("[]").split(",")[:2]]
    h, w = shown.shape
    return shown, [ox, ox + w * res, oy, oy + h * res]


def load_paths(path, wanted):
    paths = {}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            key = (r["batch"], r["run"])
            if key in wanted:
                paths.setdefault(key, []).append((int(r["seq"]), float(r["x"]), float(r["y"])))
    return {k: np.array([(x, y) for _, x, y in sorted(v)]) for k, v in paths.items()}


def footprint_poses(path, step):
    """poses every 'step' metres along the polyline, heading = segment tangent"""
    poses = []
    carry = 0.0
    for (x1, y1), (x2, y2) in zip(path[:-1], path[1:]):
        seg = math.hypot(x2 - x1, y2 - y1)
        if seg < 1e-9:
            continue
        yaw = math.atan2(y2 - y1, x2 - x1)
        s = carry
        while s <= seg:
            t = s / seg
            poses.append((x1 + t * (x2 - x1), y1 + t * (y2 - y1), yaw))
            s += step
        carry = s - seg
    if len(path) >= 2:
        (x1, y1), (x2, y2) = path[-2], path[-1]
        poses.append((x2, y2, math.atan2(y2 - y1, x2 - x1)))
    return poses


def rectangle(x, y, yaw, length, width):
    c, s = math.cos(yaw), math.sin(yaw)
    pts = []
    for lx, ly in ((length / 2, width / 2), (-length / 2, width / 2),
                   (-length / 2, -width / 2), (length / 2, -width / 2)):
        pts.append((x + c * lx - s * ly, y + s * lx + c * ly))
    return pts


def plot_paths(rows, groups, colors, paths_csv, map_yaml, goal_id, fp_len, fp_wid, fp_step, out):
    first = next((r for r in rows if int(r["goal_id"]) == goal_id), None)
    if first is None:
        print(f"goal {goal_id} not in the selected rows - no path plot")
        return
    gx, gy = first["goal_x"], first["goal_y"]
    # batches with the same goal list share goal ids, so goals are matched by coordinates
    sel = [r for r in rows if r["success"] == 1
           and abs(r["goal_x"] - gx) < 1e-6 and abs(r["goal_y"] - gy) < 1e-6]
    paths = load_paths(paths_csv, {(r["batch"], r["run"]) for r in sel})

    shown, extent = load_map(map_yaml)
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(shown, cmap="gray", vmin=0, vmax=1, extent=extent, origin="upper", interpolation="nearest")

    sx, sy = first["start_x"], first["start_y"]
    xs, ys = [sx, gx], [sy, gy]
    for g in groups:
        runs = [r for r in sel if r["group"] == g and (r["batch"], r["run"]) in paths]
        if not runs:
            continue
        for r in runs:
            p = paths[(r["batch"], r["run"])]
            ax.plot(p[:, 0], p[:, 1], color=colors[g], linewidth=0.8, alpha=0.35)
            xs += list(p[:, 0])
            ys += list(p[:, 1])
        runs.sort(key=lambda r: r["length"])
        rep = paths[(runs[len(runs) // 2]["batch"], runs[len(runs) // 2]["run"])]
        ax.plot(rep[:, 0], rep[:, 1], color=colors[g], linewidth=2.2, label=f"{g} ({len(runs)} paths)")
        for x, y, yaw in footprint_poses(rep, fp_step):
            ax.add_patch(Polygon(rectangle(x, y, yaw, fp_len, fp_wid), closed=True, fill=True,
                                 facecolor=colors[g], edgecolor=colors[g], alpha=0.18, linewidth=0.6))

    ax.plot(sx, sy, "go", markersize=10, label="start")
    ax.plot(gx, gy, "r*", markersize=16, label="goal")
    margin = 1.0
    ax.set_xlim(min(xs) - margin, max(xs) + margin)
    ax.set_ylim(min(ys) - margin, max(ys) + margin)
    ax.set_xlabel("x [m] (map frame)")
    ax.set_ylabel("y [m]")
    ax.set_title(f"Goal {goal_id} ({gx:.2f}, {gy:.2f}): thin = every seed, thick + footprints "
                 f"({fp_len:.2f} x {fp_wid:.2f} m every {fp_step:.2f} m) = median-length path",
                 fontsize=9)
    ax.legend(fontsize=8, loc="best")
    ax.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("results", help="results CSV written by cbf_benchmark")
    ap.add_argument("--paths", help="paths CSV (default: <results>_paths.csv)")
    ap.add_argument("--map", default="maps/map.yaml", help="map yaml for the path plots")
    ap.add_argument("--batch", nargs="*", default=[], help="only these batch ids")
    ap.add_argument("--goal-id", type=int, nargs="*", default=[0], help="goals to draw paths for")
    ap.add_argument("--footprint", type=float, nargs=2, default=[0.33, 0.28], metavar=("LENGTH", "WIDTH"))
    ap.add_argument("--footprint-step", type=float, default=0.3, help="spacing of drawn footprints [m]")
    ap.add_argument("--out-dir", help="default: <results dir>/plots")
    args = ap.parse_args()

    rows = load_rows(args.results, set(args.batch))
    groups = ordered_groups(rows)
    colors = colors_for(groups)
    out_dir = args.out_dir or os.path.join(os.path.dirname(os.path.abspath(args.results)), "plots")
    os.makedirs(out_dir, exist_ok=True)
    stem, ext = os.path.splitext(args.results)
    paths_csv = args.paths or f"{stem}_paths{ext}"

    write_summary(rows, groups, os.path.join(out_dir, "summary.csv"))
    plot_pareto(rows, groups, colors, os.path.join(out_dir, "pareto.png"))
    plot_boxplots(rows, groups, colors, os.path.join(out_dir, "boxplots.png"))
    if os.path.exists(paths_csv) and os.path.exists(args.map):
        for gid in args.goal_id:
            plot_paths(rows, groups, colors, paths_csv, args.map, gid, args.footprint[0],
                       args.footprint[1], args.footprint_step,
                       os.path.join(out_dir, f"paths_goal{gid}.png"))
    else:
        print(f"skipping path plots: {paths_csv} or {args.map} not found")
    print(f"plots written to {out_dir}")


if __name__ == "__main__":
    main()
