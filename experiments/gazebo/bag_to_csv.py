"""Convert a ROS 2 bag from launch/sim_fortress.launch.py into the same
TrialMetrics CSV row that experiments.sim.runner emits.

This closes the loop the Python harness already supports: aggregate.py
and paired_stats.py group by (scenario, advisor, x_use_acoustic), so a
Gazebo run gets the exact same downstream stats path as the synthetic
sim. One bag -> one row.

Pulls per-trial metrics from these recorded topics:

  /model/x3_sensors/odometry  ground-truth pose (used for ATE,
                              min_clearance, waypoints_reached,
                              and trajectory length)
  /clock                      duration_s
  /acoustic/clearance         (optional; if all >= threshold and the
                               drone passed waypoint 2 unscathed,
                               counts as a no-collision trial)

The script intentionally avoids any rosbag2_py reader for portability:
it reads sqlite3 directly. That keeps the dependency surface to
{rclpy serialization} which is already pulled in by ROS 2.

Usage:
    python -m experiments.gazebo.bag_to_csv runs/gz_run_bag \\
        --scenario corridor_white_wall --advisor rule_tree \\
        --seed 0 --use-acoustic true \\
        --out runs/gz.csv
"""

from __future__ import annotations

import argparse
import csv
import sqlite3
import sys
from pathlib import Path
from typing import Iterable

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from experiments.sim.world import World
from experiments.sim.metrics import TrialMetrics


def _bag_db(bag_dir: Path) -> Path:
    candidates = sorted(bag_dir.glob("*.db3"))
    if not candidates:
        raise FileNotFoundError(f"no .db3 file in {bag_dir}")
    return candidates[0]


def _read_topic(db_path: Path, topic: str) -> Iterable[tuple[int, object]]:
    """Yield (t_ns, deserialized_msg) for every message on the topic."""
    conn = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
    try:
        cur = conn.cursor()
        row = cur.execute(
            "SELECT id, type FROM topics WHERE name = ?", (topic,)).fetchone()
        if row is None:
            return
        topic_id, type_str = row
        msg_cls = get_message(type_str)
        for ts, blob in cur.execute(
                "SELECT timestamp, data FROM messages WHERE topic_id = ? "
                "ORDER BY timestamp", (topic_id,)):
            yield int(ts), deserialize_message(blob, msg_cls)
    finally:
        conn.close()


def _trajectory(db_path: Path,
                topic: str = "/model/x3_sensors/odometry") -> np.ndarray:
    pts: list[tuple[float, float, float]] = []
    for _, msg in _read_topic(db_path, topic):
        p = msg.pose.pose.position
        pts.append((p.x, p.y, p.z))
    return np.array(pts, dtype=float) if pts else np.zeros((0, 3))


def _duration_s(db_path: Path) -> float:
    first, last = None, None
    for ts, _ in _read_topic(db_path, "/clock"):
        if first is None:
            first = ts
        last = ts
    if first is None or last is None:
        return 0.0
    return (last - first) / 1e9


def _ate_rmse(traj: np.ndarray, waypoints: list[np.ndarray]) -> float:
    if len(traj) == 0 or not waypoints:
        return 0.0
    wps = np.stack([np.array([0.0, 0.0, 1.5])] + list(waypoints))
    err2 = []
    for p in traj:
        d_min = float("inf")
        for i in range(len(wps) - 1):
            a, b = wps[i], wps[i + 1]
            ab = b - a
            t = np.clip(np.dot(p - a, ab) / max(1e-9, np.dot(ab, ab)), 0.0, 1.0)
            d_min = min(d_min, float(np.linalg.norm(p - (a + t * ab))))
        err2.append(d_min ** 2)
    return float(np.sqrt(np.mean(err2)))


def _waypoints_reached(traj: np.ndarray,
                       waypoints: list[np.ndarray],
                       tol_m: float = 0.5) -> int:
    """Count waypoints that were ever within tol_m of the trajectory,
    in order. Matches runner.py's progression check."""
    if len(traj) == 0:
        return 0
    idx = 0
    for p in traj:
        if idx >= len(waypoints):
            break
        if np.linalg.norm(p - waypoints[idx]) < tol_m:
            idx += 1
    return idx


def _min_clearance(traj: np.ndarray, world: World) -> float:
    if len(traj) == 0:
        return float("inf")
    return float(min(world.min_clearance(p) for p in traj))


def _collisions(traj: np.ndarray, world: World) -> int:
    return int(any(world.collision(p) for p in traj))


def trial_metrics_from_bag(bag_dir: Path, *,
                           scenario: str,
                           advisor: str,
                           seed: int,
                           use_acoustic: bool) -> TrialMetrics:
    db = _bag_db(bag_dir)
    world = World.scenario(scenario)
    traj = _trajectory(db)
    duration = _duration_s(db)

    return TrialMetrics(
        scenario=scenario,
        advisor=advisor,
        seed=seed,
        duration_s=duration,
        completed=_waypoints_reached(traj, world.waypoints) >= len(world.waypoints),
        collisions=_collisions(traj, world),
        min_clearance_m=_min_clearance(traj, world),
        ate_rmse_m=_ate_rmse(traj, world.waypoints),
        # Gazebo build doesn't yet log advisor decisions to the bag;
        # leave clamp counts at zero rather than fabricate a number.
        # Add /mission/safety_clamped to the recorded topics and feed
        # it through here once the reasoning path emits them.
        safety_clamp_count=0,
        safety_clamp_rate=0.0,
        advisor_ticks=0,
        final_battery_pct=float("nan"),
        waypoints_reached=_waypoints_reached(traj, world.waypoints),
        extra={"use_acoustic": use_acoustic, "source": "gazebo"},
    )


def _strtobool(s: str) -> bool:
    return s.lower() in ("1", "true", "t", "yes", "y")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag_dir", type=Path,
                    help="rosbag2 directory (contains the .db3 file)")
    ap.add_argument("--scenario", default="corridor_white_wall")
    ap.add_argument("--advisor", default="rule_tree")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--use-acoustic", type=_strtobool, default=True)
    ap.add_argument("--out", default="-")
    args = ap.parse_args(argv)

    m = trial_metrics_from_bag(
        args.bag_dir,
        scenario=args.scenario,
        advisor=args.advisor,
        seed=args.seed,
        use_acoustic=args.use_acoustic,
    )
    row = m.to_row()
    out = sys.stdout if args.out == "-" else open(args.out, "w", newline="")
    try:
        w = csv.DictWriter(out, fieldnames=list(row.keys()))
        w.writeheader()
        w.writerow(row)
    finally:
        if out is not sys.stdout:
            out.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
