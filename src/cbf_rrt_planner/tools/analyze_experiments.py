#!/usr/bin/env python3
"""
Verdicts, tables and figures of the experiments in README_EXPERIMENTS.md.

    python3 src/cbf_rrt_planner/tools/analyze_experiments.py h1 [CSV] [--c 0.8] [--min-gain 0.2] [--max-length 1.10]
    python3 src/cbf_rrt_planner/tools/analyze_experiments.py h2 [CSV] [--tol 0.02] [--min-speedup 3.0]
    python3 src/cbf_rrt_planner/tools/analyze_experiments.py h3 [CSV] [--significant 0.25]
    python3 src/cbf_rrt_planner/tools/analyze_experiments.py h4 [CSV] [--limit 1.0]
    python3 src/cbf_rrt_planner/tools/analyze_experiments.py ablation [CSV] [--c 0.8]
    python3 src/cbf_rrt_planner/tools/analyze_experiments.py psf [DIR] [--maps-dir maps]

CSV defaults to src/cbf_rrt_planner/benchmark/experiments/h1h2.csv (h1, h2), h3.csv, h4.csv or
ablation.csv; DIR of psf to experiments/psf. The figure (h1.png, h2.png, h3.png, ablation.png,
psf_<map>.png) is written next to the input. All commands except h4 need numpy and matplotlib;
h4 needs only the standard library, so it also runs on the robot.

All statistics are medians over successful plans. A Safe-CBF-RRT* plan is paired with the plain
RRT* plan of the same batch, goal and seed.
"""

import argparse
import csv
import math
import os
import statistics
import sys

EXP_DIR = os.path.join("src", "cbf_rrt_planner", "benchmark", "experiments")
FLOATS = ["inflation_radius", "safety_weight_c", "kappa", "length", "min_footprint_clearance",
          "rejection_ratio", "plan_time_s", "psf_time_s"]
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
# H4: computation time, one group per machine (--tag of the batch) and PSF source


def h4(args):
    rows = load(args.csv or os.path.join(EXP_DIR, "h4.csv"))
    for tag, src in sorted({(r["tag"], r["psf_source"]) for r in rows}):
        rs = [r for r in rows if r["tag"] == tag and r["psf_source"] == src]
        t_psf = psf_time(rs)
        plans = [r["plan_time_s"] for r in rs]
        t_plan = med(plans)
        ok = t_psf < args.limit and t_plan < args.limit
        print(f"\n=== {tag}, psf_source {src} (map {rs[0]['map']}, "
              f"{rs[0]['max_iterations']} iterations, {len(plans)} plans) ===")
        print(f"PSF generation: median {t_psf:.3f} s")
        print(f"planning:       median {t_plan:.3f} s, max {max(plans):.3f} s")
        # H4 is stated for the constant source; other sources are shown for comparison
        label = f"H4 on {tag}" if src == "constant" else f"{src} (comparison, not H4)"
        print(f"-> {label}: {verdict(ok)} (limit {args.limit} s)")


# ---------------------------------------------------------------------------------------------
# ablation (Section 5.3.2): which of the two mechanisms produces the clearance

ABLATION = {
    "baseline": "plain RRT*",
    "cbf_only": "CBF condition only (c = 1)",
    "cost_only": "cost term only (kappa = 1e6)",
    "full": "Safe-CBF-RRT* (both)",
}


def variant(r, c):
    if r["config"] == "baseline":
        return "baseline"
    if r["safety_weight_c"] == 1.0:
        return "cbf_only"
    if r["safety_weight_c"] == c:
        return "cost_only" if r["kappa"] >= 1e5 else "full"
    return None


def same_path(a, b):
    """share of the plans of 'a' with the length of the plan of 'b' for the same goal and seed,
    i.e. (up to rounding) the same path"""
    keys = [k for k in a if k in b and a[k]["success"] == 1 and b[k]["success"] == 1]
    return sum(abs(a[k]["length"] - b[k]["length"]) < 1e-6 for k in keys) / len(keys) if keys else math.nan


def ablation(args):
    import numpy as np
    import matplotlib.pyplot as plt
    rows = load(args.csv or os.path.join(EXP_DIR, "ablation.csv"))
    sets = [s for s in ("open", "near") if any(r["tag"] == s for r in rows)]
    summary = {}
    for s in sets:
        by = {v: {} for v in ABLATION}
        for r in rows:
            v = variant(r, args.c) if r["tag"] == s else None
            if v:
                by[v][(r["goal_id"], r["seed"])] = r
        missing = [v for v in ABLATION if not by[v]]
        if missing:
            raise SystemExit(f"goal set '{s}': no rows for {missing}")
        base = by["baseline"]
        print(f"\n=== {s} goals ({len(base)} plans per variant) ===")
        print(f"{'variant':<30} {'success':>8} {'clearance [m]':>14} {'length [m]':>11} {'length/base':>12} "
              f"{'rejected':>9} {'= RRT*':>7} {'= full':>7}")
        summary[s] = {}
        for v, label in ABLATION.items():
            rs = by[v]
            ok = [r for r in rs.values() if r["success"] == 1]
            st = {
                "success": len(ok) / len(rs),
                "clearance": med([r["min_footprint_clearance"] for r in ok]),
                "length": med([r["length"] for r in ok]),
                "paired_length": med([r["length"] / base[k]["length"] for k, r in rs.items()
                                      if r["success"] == 1 and base.get(k, {}).get("success") == 1]),
                "rejection": med([r["rejection_ratio"] for r in rs.values()]),
            }
            summary[s][v] = st
            print(f"{label:<30} {st['success']:>8.1%} {st['clearance']:>14.3f} {st['length']:>11.2f} "
                  f"{st['paired_length']:>12.3f} {st['rejection']:>9.2%} {same_path(rs, base):>7.0%} "
                  f"{same_path(rs, by['full']):>7.0%}")
        d = summary[s]
        print(f"clearance added by the cost term (full - CBF only):       "
              f"{d['full']['clearance'] - d['cbf_only']['clearance']:+.3f} m")
        print(f"clearance added by the CBF condition (full - cost only):  "
              f"{d['full']['clearance'] - d['cost_only']['clearance']:+.3f} m")
    print("\n'rejected': median share of the edges rejected by the CBF condition (for plain RRT* measured,"
          " not enforced; 0 for 'cost term only' by construction).\n'= RRT*' / '= full': share of the"
          " plans with the same path as plain RRT* / as Safe-CBF-RRT* for the same goal and seed.")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.5))
    x = np.arange(len(ABLATION))
    width = 0.8 / len(sets)
    for i, s in enumerate(sets):
        off = (i - (len(sets) - 1) / 2) * width
        ax1.bar(x + off, [summary[s][v]["clearance"] for v in ABLATION], width, label=f"{s} goals")
        ax2.bar(x + off, [summary[s][v]["paired_length"] for v in ABLATION], width, label=f"{s} goals")
    for ax, title in ((ax1, "median min footprint clearance [m]"),
                      (ax2, "median length / length of plain RRT* (paired)")):
        ax.set_xticks(x)
        ax.set_xticklabels([ABLATION[v].replace(" (", "\n(") for v in ABLATION], fontsize=8)
        ax.set_title(title)
        ax.grid(axis="y", alpha=0.3)
        ax.legend(fontsize=8)
    top = max(summary[s][v]["paired_length"] for s in sets for v in ABLATION)
    ax2.set_ylim(0.95, max(1.05, top + 0.02))
    save(fig, args, "ablation.png")


# ---------------------------------------------------------------------------------------------
# psf (Section 5.2.1): the two safety functions side by side, with the obstacle centroids


def psf(args):
    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from plot_benchmark import load_map

    base_dir = args.csv or os.path.join(EXP_DIR, "psf")
    if not os.path.isdir(base_dir):
        raise SystemExit(f"{base_dir} not found - run the psf experiment first (README_EXPERIMENTS.md)")
    stems = sorted({d.rsplit("_", 1)[0] for d in os.listdir(base_dir)
                    if d.endswith(("_centroid", "_constant")) and os.path.isdir(os.path.join(base_dir, d))})
    for stem in stems:
        cdir, kdir = (os.path.join(base_dir, f"{stem}_{s}") for s in ("centroid", "constant"))
        map_yaml = os.path.join(args.maps_dir, stem + ".yaml")
        if not (os.path.isdir(cdir) and os.path.isdir(kdir) and os.path.exists(map_yaml)):
            print(f"skipping {stem}: needs {cdir}, {kdir} and {map_yaml}")
            continue
        shown, extent = load_map(map_yaml)
        h_c = np.loadtxt(os.path.join(cdir, "h.csv"), delimiter=",")
        h_k = np.loadtxt(os.path.join(kdir, "h.csv"), delimiter=",")
        ux = np.loadtxt(os.path.join(cdir, "u_x.csv"), delimiter=",")
        uy = np.loadtxt(os.path.join(cdir, "u_y.csv"), delimiter=",")
        with open(os.path.join(cdir, "centroids.csv"), newline="") as f:
            cents = [(float(r["x"]), float(r["y"]), r["in_free_space"] == "1") for r in csv.DictReader(f)]

        fig, axes = plt.subplots(1, 3, figsize=(19, 6.2))

        def layers(ax, field, cmap, unit):
            # inflated obstacles (field = 0) light grey, the field itself, the real obstacles black
            ax.imshow(np.where(h_c <= 0, 1.0, np.nan), origin="lower", extent=extent,
                      cmap=ListedColormap(["#d9d9d9"]), interpolation="nearest")
            if field is not None:
                im = ax.imshow(np.ma.masked_where(h_c <= 0, field), origin="lower", extent=extent,
                               cmap=cmap, interpolation="nearest")
                fig.colorbar(im, ax=ax, fraction=0.046, pad=0.02, label=unit)
            ax.imshow(np.ma.masked_where(shown != 0, shown), origin="upper", extent=extent,
                      cmap=ListedColormap(["black"]), interpolation="nearest")

        def centroids(ax):
            free = [(x, y) for x, y, f in cents if f]
            inside = [(x, y) for x, y, f in cents if not f]
            if free:
                ax.plot(*zip(*free), "o", color="red", markeredgecolor="black", markersize=8,
                        label="centroid in free space")
            if inside:
                ax.plot(*zip(*inside), "x", color="white", markeredgewidth=2, markersize=8,
                        label="centroid inside the obstacle")
            ax.legend(fontsize=7, loc="upper right")

        layers(axes[0], h_c, "viridis", "h [m²]")
        centroids(axes[0])
        axes[0].set_title("centroid source: h")

        layers(axes[1], None, None, None)
        H, W = h_c.shape
        res = (extent[1] - extent[0]) / W
        step = max(1, W // 40)
        rr, cc = np.mgrid[step // 2:H:step, step // 2:W:step]
        keep = h_c[rr, cc] > 0
        mag = np.hypot(ux[rr, cc], uy[rr, cc])[keep]
        q = axes[1].quiver(extent[0] + (cc[keep] + 0.5) * res, extent[2] + (rr[keep] + 0.5) * res,
                           ux[rr, cc][keep] / np.maximum(mag, 1e-9), uy[rr, cc][keep] / np.maximum(mag, 1e-9),
                           mag, cmap="plasma", angles="xy", scale=45, width=0.003)
        fig.colorbar(q, ax=axes[1], fraction=0.046, pad=0.02, label="|u| [m]")
        centroids(axes[1])
        axes[1].set_title("centroid source: guidance field u (direction; colour = |u|)")

        layers(axes[2], h_k, "viridis", "h [m]")
        axes[2].set_title("constant source: h")

        for ax in axes:
            ax.set_xlim(extent[0], extent[1])
            ax.set_ylim(extent[2], extent[3])
            ax.set_aspect("equal")
            ax.set_xlabel("x [m]")
        axes[0].set_ylabel("y [m]")
        fig.suptitle(f"{stem}: inflated obstacles light grey, obstacles black", fontsize=10)
        fig.tight_layout()
        out = os.path.join(base_dir, f"psf_{stem}.png")
        fig.savefig(out, dpi=150)
        plt.close(fig)
        n_free = sum(f for _, _, f in cents)
        print(f"{stem}: {len(cents)} obstacles, {n_free} centroids in free space -> {out}")


def save(fig, args, name):
    out = os.path.join(os.path.dirname(os.path.abspath(args.csv or os.path.join(EXP_DIR, "x.csv"))), name)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    print(f"figure: {out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("hypothesis", choices=["h1", "h2", "h3", "h4", "ablation", "psf"])
    ap.add_argument("csv", nargs="?", help="results CSV, or the directory for psf (default: see above)")
    ap.add_argument("--c", type=float, default=0.8, help="h1, ablation: the c value of Safe-CBF-RRT*")
    ap.add_argument("--maps-dir", default="maps", help="psf: directory with <map>.yaml")
    ap.add_argument("--min-gain", type=float, default=0.2, help="h1: required clearance gain [m]")
    ap.add_argument("--max-length", type=float, default=1.10, help="h1: allowed length ratio")
    ap.add_argument("--tol", type=float, default=0.02, help="h2: clearance tolerance of 'no worse' [m]")
    ap.add_argument("--min-speedup", type=float, default=3.0, help="h2: required PSF speed-up")
    ap.add_argument("--significant", type=float, default=0.25, help="h3: 'significant' fraction of lost plans")
    ap.add_argument("--limit", type=float, default=1.0, help="h4: time limit [s]")
    args = ap.parse_args()
    {"h1": h1, "h2": h2, "h3": h3, "h4": h4, "ablation": ablation, "psf": psf}[args.hypothesis](args)


if __name__ == "__main__":
    main()
