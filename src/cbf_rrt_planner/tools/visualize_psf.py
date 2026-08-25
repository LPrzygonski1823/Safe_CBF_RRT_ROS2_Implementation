#!/usr/bin/env python3
"""
Tool to visually analize PSF heatmap output exported by PSFGenerator::exportToCsv().
default file search: /tmp/psf_debug (planner_node.cpp)
"""

import sys
import numpy as np
import matplotlib.pyplot as plt


def load_grid(path: str) -> np.ndarray:
    return np.loadtxt(path, delimiter=",")


def main():
    directory = sys.argv[1] if len(sys.argv) > 1 else "/tmp/psf_debug"

    h = load_grid(f"{directory}/h.csv")
    ux = load_grid(f"{directory}/u_x.csv")
    uy = load_grid(f"{directory}/u_y.csv")

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    im0 = axes[0].imshow(h, origin="lower", cmap="viridis")
    axes[0].set_title("h(x,y) - Poisson Safety Function")
    fig.colorbar(im0, ax=axes[0])

    im1 = axes[1].imshow(ux, origin="lower", cmap="coolwarm")
    axes[1].set_title("u_x")
    fig.colorbar(im1, ax=axes[1])

    im2 = axes[2].imshow(uy, origin="lower", cmap="coolwarm")
    axes[2].set_title("u_y")
    fig.colorbar(im2, ax=axes[2])

    plt.tight_layout()
    out_path = f"{directory}/psf_visualization.png"
    plt.savefig(out_path, dpi=150)
    print(f"Visualisation saved in {out_path}")
    plt.show()


if __name__ == "__main__":
    main()