#!/usr/bin/env python3
"""Demo: call the hybrid A* planner from Python and plot the result.

Run from the repo root with the interpreter the module was built against, e.g.:
    /home/ubuntu/visor/.venv/bin/python scripts/plot_plan.py

Frame is base_link: +x forward, +y left (a top-down robot view).
"""
import os
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "build"))

import hybrid_astar  # noqa: E402


def rect(x0, x1, y0, y1, step=0.1):
    """Fill an axis-aligned rectangle with points on a `step` grid (base_link)."""
    xs = np.arange(x0, x1 + 1e-6, step)
    ys = np.arange(y0, y1 + 1e-6, step)
    gx, gy = np.meshgrid(xs, ys)
    return np.column_stack([gx.ravel(), gy.ravel()])


def main(num_goals=16, radius=6.4):
    # Rectangular obstacles in base_link coordinates (x fwd, y left).
    cloud = np.vstack([
        rect(1.4, 1.6, -0.8, 0.8),    # wall straight ahead
        rect(2.8, 3.4, 1.8, 2.8),     # block front-left
        rect(2.0, 2.8, -2.6, -1.8),   # block front-right
    ]).astype(np.float32)

    config = os.path.join(REPO_ROOT, "config", "navigation.lua")

    # N goals sampled uniformly in [-90, +90] deg of the forward (+x) direction.
    angles = np.linspace(-np.pi / 4.0, np.pi / 4.0, num_goals)
    goals = np.column_stack([radius * np.cos(angles), radius * np.sin(angles)])

    fig, ax = plt.subplots(figsize=(8, 8))
    cmap = plt.get_cmap("hsv")

    # Top-down view. Plot +y (left) on the horizontal axis, inverted so left is
    # left; +x (forward) on the vertical axis.
    ax.scatter(cloud[:, 1], cloud[:, 0], c="k", marker="s", s=40,
               zorder=3, label="obstacles")

    reached = 0
    for i, (gx, gy) in enumerate(goals):
        color = cmap(i / num_goals)
        path = hybrid_astar.plan(cloud, (float(gx), float(gy)), config_path=config)
        ax.plot(gy, gx, "*", color=color, ms=12, zorder=4)
        if path.shape[0] == 0:
            ax.plot(gy, gx, "x", color="red", ms=10, mew=2, zorder=5)
            continue
        reached += 1
        ax.plot(path[:, 1], path[:, 0], "-", color=color, lw=1.5, zorder=2)

    print(f"reached {reached}/{num_goals} goals on the {radius} m circle")

    circle = plt.Circle((0.0, 0.0), radius, fill=False, ls="--",
                        color="gray", alpha=0.6, zorder=1)
    ax.add_patch(circle)
    ax.plot(0.0, 0.0, "ko", ms=10, zorder=5, label="robot")

    ax.set_xlabel("y (left, m)")
    ax.set_ylabel("x (forward, m)")
    ax.set_title(
        f"Hybrid A* paths to {num_goals} goals, +/-90 deg on a {radius} m arc")
    ax.invert_xaxis()
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    fig.tight_layout()
    out = os.path.join(REPO_ROOT, "plan.png")
    fig.savefig(out, dpi=120)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
