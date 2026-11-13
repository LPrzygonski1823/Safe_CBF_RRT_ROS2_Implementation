#!/usr/bin/env python3
"""
Verdicts, tables and figures of the experiments in README_EXPERIMENTS.md.

    python3 src/cbf_rrt_planner/tools/analyze_experiments.py h1 [CSV] [--c 0.8] [--min-gain 0.2] [--max-length 1.10]
    python3 src/cbf_rrt_planner/tools/analyze_experiments.py h2 [CSV] [--tol 0.02] [--min-speedup 3.0]
    python3 src/cbf_rrt_planner/tools/analyze_experiments.py h3 [CSV] [--significant 0.25]
    python3 src/cbf_rrt_planner/tools/analyze_experiments.py h4 [CSV] [--limit 1.0]

CSV defaults to src/cbf_rrt_planner/benchmark/experiments/h1h2.csv (h1, h2), h3.csv or h4.csv.
The figure (h1.png, h2.png, h3.png) is written next to the CSV. h1-h3 need numpy and matplotlib;
h4 needs only the standard library, so it also runs on the robot.

All statistics are medians over successful plans. A Safe-CBF-RRT* plan is paired with the plain
RRT* plan of the same batch, goal and seed.
"""

import argparse
import csv
import math
import os
import statistics

EXP_DIR = os.path.join("src", "cbf_rrt_planner", "benchmark", "experiments")
FLOATS = ["inflation_radius", "safety_weight_c", "length", "min_footprint_clearance",
          "plan_time_s", "psf_time_s"]
COLORS = {"constant": "tab:blue", "centroid": "tab:orange"}


def load(path):
    if not os.path.exists(path):
        raise SystemExit(f"{path} not found - run the experiment first (README_EXPERIMENTS.md)")
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"no rows in {path}")
    for r in rows:
        for k in FLOATS:
            r[k] = float(r[k])
        r["success"] = int(r["success"])
    return rows


def med(values):
    v = [x for x in values if not math.isnan(x)]
    return statistics.median(v) if v else math.nan


def verdict(ok):
    return "CONFIRMED" if ok else "NOT CONFIRMED"


def psf_time(rows):
    """one PSF time per batch (median of its --psf-repeats), median over batches"""
    return med(list({r["batch"]: r["psf_time_s"] for r in rows}.values()))


# ---------------------------------------------------------------------------------------------
# H1 and H2: one c sweep plus baseline per PSF source


def trade_off(rows):
    out = {}
    for src in sorted({r["psf_source"] for r in rows}):
        ok = [r for r in rows if r["psf_source"] == src and r["success"] == 1]
        base = {(r["batch"], r["goal_id"], r["seed"]): r for r in ok if r["config"] == "baseline"}
        if not base:
            raise SystemExit(f"no baseline rows for psf_source={src} - the batch needs --baseline")
        curve = {}
        for c in sorted({r["safety_weight_c"] for r in ok if r["config"] == "cbf"}):
            pairs = [(r, base[(r["batch"], r["goal_id"], r["seed"])]) for r in ok
                     if r["config"] == "cbf" and r["safety_weight_c"] == c
                     and (r["batch"], r["goal_id"], r["seed"]) in base]
            curve[c] = {
                "clearance": med([r["min_footprint_clearance"] for r, _ in pairs]),
                "length": med([r["length"] for r, _ in pairs]),
                "paired_length": med([r["length"] / b["length"] for r, b in pairs]),
            }
        tried = [r for r in rows if r["psf_source"] == src]
        out[src] = {
            "curve": curve,
            "base_clearance": med([b["min_footprint_clearance"] for b in base.values()]),
            "base_length": med([b["length"] for b in base.values()]),
            "success_rate": sum(r["success"] for r in tried) / len(tried),
            "psf_time": psf_time(tried),
        }
    return out


def decreasing(values):
    return all(a > b for a, b in zip(values, values[1:]))


def h1(args):
    import matplotlib.pyplot as plt
    rows = load(args.csv or os.path.join(EXP_DIR, "h1h2.csv"))
    data = trade_off(rows)
    all_ok = True
    for src, d in data.items():
        curve = d["curve"]
        if args.c not in curve:
            raise SystemExit(f"c = {args.c} not in the results for {src}")
        print(f"\n=== {src} source (success rate of all plans {d['success_rate']:.1%}) ===")
        print(f"{'c':>6} {'clearance [m]':>14} {'gain [m]':>9} {'length [m]':>11} {'length/base':>12}")
        print(f"{'base':>6} {d['base_clearance']:>14.3f} {'':>9} {d['base_length']:>11.2f} {'':>12}")
        for c, p in curve.items():
            print(f"{c:>6g} {p['clearance']:>14.3f} {p['clearance'] - d['base_clearance']:>9.3f} "
                  f"{p['length']:>11.2f} {p['length'] / d['base_length']:>12.3f}")
        p = curve[args.c]
        gain = p["clearance"] - d["base_clearance"]
        ratio = p["length"] / d["base_length"]
        cs = sorted(curve)
        mono_cl = decreasing([curve[c]["clearance"] for c in cs])
        mono_len = decreasing([curve[c]["length"] for c in cs])
        ok = gain >= args.min_gain and ratio <= args.max_length and mono_cl and mono_len
        all_ok &= ok
        print(f"c = {args.c:g}: clearance gain {gain:.3f} m (needed >= {args.min_gain}), "
              f"length x{ratio:.3f} (allowed <= {args.max_length}); paired median length ratio "
              f"x{p['paired_length']:.3f}")
        print(f"monotonic in c: clearance {'yes' if mono_cl else 'NO'}, length {'yes' if mono_len else 'NO'}")
        print(f"-> {verdict(ok)} for the {src} source")
    print(f"\nH1: {verdict(all_ok)}")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.5))
    for src, d in data.items():
        cs = sorted(d["curve"])
        color = COLORS.get(src)
        ax1.plot(cs, [d["curve"][c]["clearance"] for c in cs], "o-", color=color, label=src)
        ax1.axhline(d["base_clearance"], color=color, linestyle="--", linewidth=1,
                    label=f"{src}: RRT* baseline")
        ax1.plot(args.c, d["base_clearance"] + args.min_gain, "v", color=color, markersize=9)
        ax2.plot(cs, [d["curve"][c]["length"] / d["base_length"] for c in cs], "o-",
                 color=color, label=src)
    ax1.set_xlabel("safety weight c")
    ax1.set_ylabel("median min footprint clearance [m]")
    ax1.set_title(f"clearance (triangles: baseline + {args.min_gain} m)")
    ax2.axhline(args.max_length, color="red", linestyle=":", label=f"limit x{args.max_length}")
    ax2.axvline(args.c, color="grey", linestyle=":")
    ax2.set_xlabel("safety weight c")
    ax2.set_ylabel("median length / median baseline length")
    ax2.set_title("path length")
    for ax in (ax1, ax2):
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    save(fig, args, "h1.png")


def h2(args):
    import numpy as np
    import matplotlib.pyplot as plt
    rows = load(args.csv or os.path.join(EXP_DIR, "h1h2.csv"))
    data = trade_off(rows)
    if set(data) != {"constant", "centroid"}:
        raise SystemExit(f"need both PSF sources, found {sorted(data)}")

    def points(src):
        d = data[src]
        pts = sorted((p["paired_length"], p["clearance"], c) for c, p in d["curve"].items())
        return np.array([p[0] for p in pts]), np.array([p[1] for p in pts]), [p[2] for p in pts]

    xk, yk, ck = points("constant")
    xc, yc, cc = points("centroid")
    print("centroid point -> constant clearance at the same paired length ratio")
    print(f"{'c':>6} {'length ratio':>13} {'centroid [m]':>13} {'constant [m]':>13} {'diff [m]':>9}")
    diffs = []
    for x, y, c in zip(xc, yc, cc):
        if xk[0] <= x <= xk[-1]:
            yi = float(np.interp(x, xk, yk))
            diffs.append(yi - y)
            print(f"{c:>6g} {x:>13.3f} {y:>13.3f} {yi:>13.3f} {yi - y:>9.3f}")
        else:
            print(f"{c:>6g} {x:>13.3f} {y:>13.3f} {'outside the constant curve':>23}")
    if not diffs:
        raise SystemExit("the two curves do not overlap in length - extend the c sweep")
    no_worse = min(diffs) >= -args.tol
    speedup = data["centroid"]["psf_time"] / data["constant"]["psf_time"]
    print(f"\nworst difference {min(diffs):.3f} m (tolerance {args.tol} m) -> "
          f"constant {'no worse' if no_worse else 'WORSE'} than centroid")
    print(f"PSF time: centroid {data['centroid']['psf_time']:.2f} s, constant "
          f"{data['constant']['psf_time']:.2f} s -> x{speedup:.2f} (needed >= {args.min_speedup})")
    print(f"\nH2: {verdict(no_worse and speedup >= args.min_speedup)}")

    fig, ax = plt.subplots(figsize=(7, 5.5))
    for src, x, y, cs in (("constant", xk, yk, ck), ("centroid", xc, yc, cc)):
        ax.plot(x, y, "o-", color=COLORS[src], label=f"{src} (PSF {data[src]['psf_time']:.2f} s)")
        for xi, yi, c in zip(x, y, cs):
            ax.annotate(f"{c:g}", (xi, yi), textcoords="offset points", xytext=(4, 4), fontsize=7)
    ax.plot(1.0, data["constant"]["base_clearance"], "ks", label="plain RRT*")
    ax.set_xlabel("median length / length of plain RRT* (paired)")
    ax.set_ylabel("median min footprint clearance [m]")
    ax.set_title("trade-off curves over c (labels: c)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    save(fig, args, "h2.png")


# ---------------------------------------------------------------------------------------------
# H3: inflation sweep of plain RRT* vs Safe-CBF-RRT* with the minimal inflation


def h3(args):
    import matplotlib.pyplot as plt
    rows = load(args.csv or os.path.join(EXP_DIR, "h3.csv"))

    def sel(tag, config, radius=None):
        return [r for r in rows if r["tag"] == tag and r["config"] == config
                and (radius is None or abs(r["inflation_radius"] - radius) < 1e-9)]

    def clearance(rs):
        return med([r["min_footprint_clearance"] for r in rs if r["success"] == 1])

    def success(rs):
        return sum(r["success"] for r in rs) / len(rs) if rs else math.nan

    cbf_match, cbf_near = sel("match", "cbf"), sel("near", "cbf")
    if not cbf_match or not cbf_near:
        raise SystemExit("missing Safe-CBF-RRT* rows for the 'match' or 'near' goal set")
    target = clearance(cbf_match)
    radii = sorted({r["inflation_radius"] for r in rows if r["config"] == "baseline"})

    print(f"Safe-CBF-RRT* (c = {cbf_match[0]['safety_weight_c']:g}, inflation "
          f"{cbf_match[0]['inflation_radius']:g} m): clearance {target:.3f} m on open goals, "
          f"near-obstacle goals reached in {success(cbf_near):.1%} of the plans\n")
    print(f"{'inflation [m]':>13} {'clearance [m]':>14} {'near reached':>13} {'goal_blocked':>13} {'no_path':>8}")
    table = []
    for rad in radii:
        m, n = sel("match", "baseline", rad), sel("near", "baseline", rad)
        row = (rad, clearance(m), success(n),
               sum(r["failure"] == "goal_blocked" for r in n), sum(r["failure"] == "no_path" for r in n))
        table.append(row)
        print(f"{row[0]:>13g} {row[1]:>14.3f} {row[2]:>13.1%} {row[3]:>13} {row[4]:>8}")

    matched = [row for row in table if row[1] >= target]
    if not matched:
        print(f"\nno radius up to {radii[-1]:g} m reaches the clearance {target:.3f} m - extend the sweep")
        print("\nH3: NOT VERIFIED")
    else:
        rad, cl, reached, _, _ = matched[0]
        unreachable = 1.0 - reached
        ok = unreachable >= args.significant and success(cbf_near) == 1.0
        print(f"\nsmallest radius with a comparable clearance: {rad:g} m ({cl:.3f} m >= {target:.3f} m)")
        print(f"near-obstacle plans lost with it: {unreachable:.1%} (significant from {args.significant:.0%}); "
              f"Safe-CBF-RRT* reaches {success(cbf_near):.1%}")
        print(f"\nH3: {verdict(ok)}")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.5))
    ax1.plot([t[0] for t in table], [t[1] for t in table], "o-", label="plain RRT*")
    ax1.axhline(target, color="green", linestyle="--", label="Safe-CBF-RRT*, inflation 0.22 m")
    ax1.set_ylabel("median min footprint clearance [m]")
    ax1.set_title("open goals: clearance")
    ax2.plot([t[0] for t in table], [100 * t[2] for t in table], "o-", label="plain RRT*")
    ax2.axhline(100 * success(cbf_near), color="green", linestyle="--", label="Safe-CBF-RRT*, inflation 0.22 m")
    ax2.set_ylim(-5, 105)
    ax2.set_ylabel("near-obstacle goals reached [% of plans]")
    ax2.set_title("near-obstacle goals: reachability")
    for ax in (ax1, ax2):
        if matched:
            ax.axvline(matched[0][0], color="grey", linestyle=":", label="comparable clearance")
        ax.set_xlabel("inflation radius of plain RRT* [m]")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    save(fig, args, "h3.png")


# ---------------------------------------------------------------------------------------------
# H4: computation time, one group per machine (--tag of the batch)


def h4(args):
    rows = load(args.csv or os.path.join(EXP_DIR, "h4.csv"))
    for tag in sorted({r["tag"] for r in rows}):
        rs = [r for r in rows if r["tag"] == tag]
        t_psf = psf_time(rs)
        plans = [r["plan_time_s"] for r in rs]
        t_plan = med(plans)
        ok = t_psf < args.limit and t_plan < args.limit
        print(f"\n=== {tag} (map {rs[0]['map']}, psf_source {rs[0]['psf_source']}, "
              f"{rs[0]['max_iterations']} iterations, {len(plans)} plans) ===")
        print(f"PSF generation: median {t_psf:.3f} s")
        print(f"planning:       median {t_plan:.3f} s, max {max(plans):.3f} s")
        print(f"-> H4 on {tag}: {verdict(ok)} (limit {args.limit} s)")


def save(fig, args, name):
    out = os.path.join(os.path.dirname(os.path.abspath(args.csv or os.path.join(EXP_DIR, "x.csv"))), name)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    print(f"figure: {out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("hypothesis", choices=["h1", "h2", "h3", "h4"])
    ap.add_argument("csv", nargs="?", help="results CSV (default: see above)")
    ap.add_argument("--c", type=float, default=0.8, help="h1: the c value of the hypothesis")
    ap.add_argument("--min-gain", type=float, default=0.2, help="h1: required clearance gain [m]")
    ap.add_argument("--max-length", type=float, default=1.10, help="h1: allowed length ratio")
    ap.add_argument("--tol", type=float, default=0.02, help="h2: clearance tolerance of 'no worse' [m]")
    ap.add_argument("--min-speedup", type=float, default=3.0, help="h2: required PSF speed-up")
    ap.add_argument("--significant", type=float, default=0.25, help="h3: 'significant' fraction of lost plans")
    ap.add_argument("--limit", type=float, default=1.0, help="h4: time limit [s]")
    args = ap.parse_args()
    {"h1": h1, "h2": h2, "h3": h3, "h4": h4}[args.hypothesis](args)


if __name__ == "__main__":
    main()
