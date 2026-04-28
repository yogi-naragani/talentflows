"""Trajectory visualization for the paper.

Runs a single scenario across multiple seeds with acoustic on and
acoustic off, then plots the (x, y) trajectories side by side over
the obstacle layout. The output is the headline figure of the
acoustic-backup paper.

Usage:
    python -m experiments.sim.visualize \
        --scenario corridor_white_wall --seeds 0 1 2 3 4 \
        --out paper/figures/trajectory_acoustic_comparison.png
"""

import argparse
from typing import Iterable

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

from .runner import TrialConfig, run_trial_with_trajectory
from .world import World, Box


def _draw_world(ax, world: World, title: str) -> None:
    for b in world.obstacles:
        ax.add_patch(patches.Rectangle(
            (b.xmin, b.ymin), b.xmax - b.xmin, b.ymax - b.ymin,
            facecolor="0.85", edgecolor="0.5", linewidth=1.0))
    for i, w in enumerate(world.waypoints):
        ax.plot(w[0], w[1], marker="*", markersize=12,
                color="goldenrod", zorder=5)
        ax.annotate(f"w{i+1}", (w[0], w[1]), textcoords="offset points",
                    xytext=(6, 6), fontsize=8, color="goldenrod")
    ax.plot(0, 0, marker="o", markersize=8, color="black", zorder=5)
    ax.annotate("start", (0, 0), textcoords="offset points",
                xytext=(6, -10), fontsize=8)
    ax.set_title(title, fontsize=10)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)


def render(scenario: str, seeds: Iterable[int], out_path: str,
           duration_s: float = 30.0, advisor: str = "rule_tree") -> None:
    seeds = list(seeds)
    fig, (ax_off, ax_on) = plt.subplots(
        1, 2, figsize=(10, 4), sharex=True, sharey=True)
    world = World.scenario(scenario)
    _draw_world(ax_off, world, f"acoustic OFF ({scenario})")
    _draw_world(ax_on,  world, f"acoustic ON ({scenario})")

    for seed in seeds:
        for ac, ax, color in ((False, ax_off, "tab:red"),
                              (True,  ax_on,  "tab:blue")):
            metrics, traj = run_trial_with_trajectory(TrialConfig(
                scenario=scenario, advisor=advisor, seed=seed,
                duration_s=duration_s, use_acoustic=ac,
            ))
            if traj.shape[0] == 0:
                continue
            ax.plot(traj[:, 0], traj[:, 1], color=color, alpha=0.5,
                    linewidth=1.2)
            # Mark trajectory endpoint
            end_marker = "x" if metrics.collisions else "o"
            ax.plot(traj[-1, 0], traj[-1, 1], end_marker, color=color,
                    markersize=8, markeredgewidth=2)

    handles = [
        plt.Line2D([], [], color="tab:red", label="acoustic off"),
        plt.Line2D([], [], color="tab:blue", label="acoustic on"),
        plt.Line2D([], [], color="black", marker="x", linestyle="none",
                   markersize=8, markeredgewidth=2, label="collision"),
        plt.Line2D([], [], color="black", marker="o", linestyle="none",
                   markersize=8, label="stopped safely"),
    ]
    fig.legend(handles=handles, loc="lower center", ncol=4, fontsize=9,
               frameon=False, bbox_to_anchor=(0.5, -0.02))
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="corridor_white_wall")
    ap.add_argument("--seeds", nargs="+", type=int,
                    default=[0, 1, 2, 3, 4])
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--advisor", default="rule_tree")
    ap.add_argument("--out", required=True)
    args = ap.parse_args(argv)
    render(args.scenario, args.seeds, args.out,
           duration_s=args.duration, advisor=args.advisor)
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
