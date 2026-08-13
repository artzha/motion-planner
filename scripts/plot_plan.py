#!/usr/bin/env python3
"""Demo: call the hybrid A* planner from Python and plot the results.

Writes three figures:
  plan.png          one path to each of a fan of goals (``plan``)
  plan_diverse.png  several paths to *one* goal, contrasting how the two
                    diversity modes spread them (``plan_diverse``)
  plan_reseed.png   the same scene drawn twice under different seeds, showing
                    which parts of a ball-penalty set are redrawn

Run from the repo root with the interpreter the module was built against, e.g.:
    /home/ubuntu/visor/.venv/bin/python scripts/plot_plan.py
or, with the colcon overlay sourced, any interpreter that can import the module.

Frame is base_link: +x forward, +y left (a top-down robot view).
"""
import os
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

try:
    # Installed (colcon overlay sourced, or copied into site-packages).
    import hybrid_astar  # noqa: E402
except ImportError:
    # Plain-cmake build tree.
    sys.path.insert(0, os.path.join(REPO_ROOT, "build"))
    import hybrid_astar  # noqa: E402

CONFIG = os.path.join(REPO_ROOT, "config", "navigation.lua")


def rect(x0, x1, y0, y1, step=0.1):
    """Fill an axis-aligned rectangle with points on a `step` grid (base_link)."""
    xs = np.arange(x0, x1 + 1e-6, step)
    ys = np.arange(y0, y1 + 1e-6, step)
    gx, gy = np.meshgrid(xs, ys)
    return np.column_stack([gx.ravel(), gy.ravel()])


def scene():
    """Obstacles in base_link (x fwd, y left): a wall dead ahead plus two blocks.

    The wall is the interesting part: reaching anything beyond it means committing
    to passing left or right, so a diverse set of paths should use both gaps.
    """
    return np.vstack([
        rect(1.4, 1.6, -0.8, 0.8),    # wall straight ahead
        rect(2.8, 3.4, 1.8, 2.8),     # block front-left
        rect(2.0, 2.8, -2.6, -1.8),   # block front-right
    ]).astype(np.float32)


# --------------------------------------------------------------------------
# Path-set diversity metrics
# --------------------------------------------------------------------------


def resample(path, n=32):
    """Resample an (N, >=2) path to `n` points spaced evenly along arc length.

    Even spacing is what makes an index range a fraction of the path, which the
    mid-path metric below relies on.
    """
    xy = np.asarray(path, dtype=float)[:, :2]
    if len(xy) < 2:
        return np.repeat(xy[:1], n, axis=0) if len(xy) else np.zeros((n, 2))
    seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    if s[-1] <= 1e-9:
        return np.repeat(xy[:1], n, axis=0)
    t = np.linspace(0.0, s[-1], n)
    return np.column_stack([np.interp(t, s, xy[:, 0]), np.interp(t, s, xy[:, 1])])


def symmetric_hausdorff(a, b):
    """Worst-case separation between two polylines, in meters."""
    d = np.linalg.norm(a[:, None, :] - b[None, :, :], axis=-1)
    return float(max(d.min(axis=1).max(), d.min(axis=0).max()))


def separation(paths, mid_only=False):
    """(min, mean) pairwise symmetric Hausdorff over a set of paths.

    With `mid_only`, only the middle half of each path is compared. That is the
    number that matters here: every path in the set leaves the same pose and
    arrives at the same goal, so the endpoints are pinned and a whole-path
    metric reports separation the candidates do not actually have where it
    counts. The minimum matters more than the mean, since one near-duplicate
    pair is a wasted downstream evaluation no matter how spread the rest are.
    """
    pts = [resample(p) for p in paths]
    if mid_only:
        pts = [p[len(p) // 4:(3 * len(p)) // 4] for p in pts]
    pairs = [symmetric_hausdorff(pts[i], pts[j])
             for i in range(len(pts)) for j in range(i + 1, len(pts))]
    return (min(pairs), float(np.mean(pairs))) if pairs else (0.0, 0.0)


# --------------------------------------------------------------------------
# Figure 1: one path per goal over a fan of goals
# --------------------------------------------------------------------------


def plot_fan(num_goals=16, radius=6.4, noise=0.5):
    cloud = scene()
    angles = np.linspace(-np.pi / 4.0, np.pi / 4.0, num_goals)
    goals = np.column_stack([radius * np.cos(angles), radius * np.sin(angles)])

    fig, ax = plt.subplots(figsize=(8, 8))
    cmap = plt.get_cmap("hsv")
    ax.scatter(cloud[:, 1], cloud[:, 0], c="k", marker="s", s=40,
               zorder=3, label="obstacles")

    reached = 0
    for i, (gx, gy) in enumerate(goals):
        color = cmap(i / num_goals)
        # Per-goal seed + randomized weighted-A* factor so paths diversify
        # instead of collapsing onto the same corridor. w >= 1.2 keeps the
        # noise-inflated search within the expansion cap. Seeded for reproducibility.
        weight = np.random.default_rng(i).uniform(1.3, 2.0)
        path = hybrid_astar.plan(cloud, (float(gx), float(gy)),
                                 config_path=CONFIG, seed=i, noise=noise,
                                 weight=float(weight))
        ax.plot(gy, gx, "*", color=color, ms=12, zorder=4)
        if path.shape[0] == 0:
            ax.plot(gy, gx, "x", color="red", ms=10, mew=2, zorder=5)
            continue
        reached += 1
        ax.plot(path[:, 1], path[:, 0], "-", color=color, lw=1.5, zorder=2)

    print(f"reached {reached}/{num_goals} goals on the {radius} m circle")

    ax.add_patch(plt.Circle((0.0, 0.0), radius, fill=False, ls="--",
                            color="gray", alpha=0.6, zorder=1))
    ax.plot(0.0, 0.0, "ko", ms=10, zorder=5, label="robot")
    ax.set_xlabel("y (left, m)")
    ax.set_ylabel("x (forward, m)")
    ax.set_title(f"Hybrid A* paths to {num_goals} goals (noise eta={noise} m)")
    ax.invert_xaxis()
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    fig.tight_layout()
    out = os.path.join(REPO_ROOT, "plan.png")
    fig.savefig(out, dpi=120)
    print(f"wrote {out}")


# --------------------------------------------------------------------------
# Figure 2: several paths to one goal, noise vs ball penalty
# --------------------------------------------------------------------------


# Per-mode noise magnitude, since the two use it for different things. Under
# 'noise' it is the entire diversity mechanism, so it has to be large. Under
# 'ball' the repulsion does the separating and noise only redraws the detours
# between calls, which is not what this figure measures -- 0 here keeps the
# comparison reproducible. See plot_reseed for what it is actually for.
MODES = [
    ("noise", "independent seeded draws", {"noise": 0.5}),
    ("ball", "each round repelled from the accepted ones", {"noise": 0.0}),
]

# Three goals worth contrasting: one straight past the wall, where reaching it at
# all means committing to a side; one off to the left through open space, where
# nothing forces a choice and a planner is free to return the same path over and
# over; and one around the far corner of the front-left block, which is the case
# that punishes a heuristic blind to the repulsion -- the detours there have to
# thread a lane the accepted paths already occupy.
GOALS = [(4.6, 0.0), (3.0, 1.0), (4.5, 2.6)]


def plot_diverse(goals=GOALS, num_paths=5, **kwargs):
    cloud = scene()
    fig, axes = plt.subplots(len(goals), len(MODES),
                             figsize=(11, 5.6 * len(goals)), squeeze=False)
    cmap = plt.get_cmap("viridis")

    header = (f"{'goal':>12}  {'mode':<6}{'found':>6}{'mid min':>9}"
              f"{'mid mean':>10}{'full min':>10}{'max cost':>10}"
              f"{'max ratio':>11}{'ms/path':>9}")
    print(f"\n{num_paths} paths per goal "
          f"(mid = middle 50% of arc length, all distances in m)")
    print(header)
    print("-" * len(header))

    for row, goal in enumerate(goals):
        for col, (mode, blurb, mode_kwargs) in enumerate(MODES):
            ax = axes[row][col]
            plans = hybrid_astar.plan_diverse(cloud, goal, config_path=CONFIG,
                                              num_paths=num_paths, mode=mode,
                                              **{**mode_kwargs, **kwargs})
            ax.scatter(cloud[:, 1], cloud[:, 0], c="k", marker="s", s=24,
                       zorder=3)
            for i, p in enumerate(plans):
                path = p["path"]
                ax.plot(path[:, 1], path[:, 0], "-", lw=2.2,
                        color=cmap(i / max(len(plans) - 1, 1)),
                        label=f"{i}: {p['cost']:.2f} m  (x{p['ratio']:.2f})",
                        zorder=2)

            paths = [p["path"] for p in plans]
            mid_min, mid_mean = separation(paths, mid_only=True)
            full_min, _ = separation(paths)
            costs = [p["cost"] for p in plans] or [0.0]
            ratios = [p["ratio"] for p in plans] or [0.0]
            ms = 1e3 * float(np.mean([p["elapsed_s"] for p in plans])) \
                if plans else 0.0
            print(f"{str(goal):>12}  {mode:<6}{len(plans):>6}{mid_min:>9.2f}"
                  f"{mid_mean:>10.2f}{full_min:>10.2f}{max(costs):>10.2f}"
                  f"{max(ratios):>11.2f}{ms:>9.1f}")

            ax.plot(goal[1], goal[0], "r*", ms=18, zorder=5)
            ax.plot(0.0, 0.0, "ko", ms=9, zorder=5)
            ax.set_title(f"goal {goal}, mode='{mode}'\n{blurb}\n"
                         f"{len(plans)}/{num_paths} paths, mid-path separation "
                         f"min {mid_min:.2f} m / mean {mid_mean:.2f} m",
                         fontsize=10)
            ax.set_xlabel("y (left, m)")
            ax.set_ylabel("x (forward, m)")
            # 'box' rather than axis("equal"), which cannot adjust data limits
            # once the axes are laid out in a shared grid.
            ax.set_aspect("equal", adjustable="box")
            ax.grid(True, alpha=0.3)
            ax.invert_xaxis()
            ax.legend(loc="lower left", fontsize=8, title="path: cost (ratio)")

    fig.suptitle("Diverse paths to a single goal: independent noise vs "
                 "ball-penalty repulsion", fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.98))
    out = os.path.join(REPO_ROOT, "plan_diverse.png")
    fig.savefig(out, dpi=120)
    print(f"wrote {out}")


# --------------------------------------------------------------------------
# Figure 3: the same scene drawn twice, under different seeds
# --------------------------------------------------------------------------


def plot_reseed(goals=GOALS, num_paths=5, noise=0.15, seeds=(1, 5), **kwargs):
    """Ball-penalty draws of each scene under two different seeds.

    The balls are a function of the accepted paths, so without noise this mode
    answers an unchanged scene identically every time. That matters in the loop:
    when every candidate is rejected and the query is reissued from the same
    spot, an identical set gets rejected identically. Noise on the detour rounds
    gives the reissue something new to offer, while round 0 carries no noise so
    the optimum stays put.

    Both goals are shown because the amount of variation on offer is a property
    of the scene, not of the knob: past the wall there are only two sides, so a
    reseed mostly changes how many paths are found rather than where they go.
    """
    cloud = scene()
    fig, axes = plt.subplots(len(goals), len(seeds),
                             figsize=(11, 5.6 * len(goals)), squeeze=False)
    cmap = plt.get_cmap("viridis")

    print(f"\nreseed check (noise eta={noise} m, seeds {seeds[0]} vs {seeds[1]})")
    for row, goal in enumerate(goals):
        draws = []
        for col, seed in enumerate(seeds):
            ax = axes[row][col]
            plans = hybrid_astar.plan_diverse(cloud, goal, config_path=CONFIG,
                                              num_paths=num_paths, mode="ball",
                                              seed=seed, noise=noise, **kwargs)
            draws.append([p["path"] for p in plans])
            ax.scatter(cloud[:, 1], cloud[:, 0], c="k", marker="s", s=24,
                       zorder=3)
            for i, p in enumerate(plans):
                path = p["path"]
                ax.plot(path[:, 1], path[:, 0], "-", lw=2.2,
                        color=cmap(i / max(len(plans) - 1, 1)),
                        label=f"{i}: {p['cost']:.2f} m", zorder=2)
            ax.plot(goal[1], goal[0], "r*", ms=18, zorder=5)
            ax.plot(0.0, 0.0, "ko", ms=9, zorder=5)
            ax.set_title(f"goal {goal}, seed={seed}: {len(plans)}/{num_paths} "
                         f"paths", fontsize=10)
            ax.set_xlabel("y (left, m)")
            ax.set_ylabel("x (forward, m)")
            ax.set_aspect("equal", adjustable="box")
            ax.grid(True, alpha=0.3)
            ax.invert_xaxis()
            ax.legend(loc="lower left", fontsize=8, title="path: cost")

        a, b = draws
        n = min(len(a), len(b))
        print(f"  goal {str(goal):>12}: {len(a)} vs {len(b)} paths", end="")
        if n:
            deltas = [symmetric_hausdorff(resample(a[i]), resample(b[i]))
                      for i in range(n)]
            print(f", path 0 moved {deltas[0]:.3f} m (want ~0)", end="")
            if n > 1:
                print(f", detours moved mean "
                      f"{float(np.mean(deltas[1:])):.2f} m", end="")
        print()

    fig.suptitle(f"Ball penalty under two seeds (noise eta={noise} m): index 0 "
                 "is unchanged, the detour set is not", fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.98))
    out = os.path.join(REPO_ROOT, "plan_reseed.png")
    fig.savefig(out, dpi=120)
    print(f"wrote {out}")


if __name__ == "__main__":
    plot_fan()
    plot_diverse()
    plot_reseed()
