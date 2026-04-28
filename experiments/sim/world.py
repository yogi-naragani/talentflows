"""Scenario geometry: ground plane, walls, mission waypoints.

A scenario is a list of axis-aligned rectangular obstacles (walls)
plus a series of waypoints. The world supports two queries that the
synthetic sensors need:

    distance_along(p, dir_b)   -> nearest hit distance from p along dir
    texture_density(p, yaw)    -> [0, 1] proxy for VSLAM feature richness
                                  along the camera's forward axis

Coordinate frame: world is ENU (x forward / east, y left / north,
z up). Camera looks along +x in body frame.
"""

from dataclasses import dataclass, field
from typing import Sequence
import numpy as np


@dataclass
class Box:
    xmin: float; xmax: float
    ymin: float; ymax: float
    zmin: float; zmax: float
    texture: float = 1.0  # 0 = blank wall, 1 = rich features


@dataclass
class World:
    obstacles: list[Box] = field(default_factory=list)
    waypoints: list[np.ndarray] = field(default_factory=list)
    ground_z: float = 0.0
    ceiling_z: float = 5.0

    @staticmethod
    def scenario(name: str) -> "World":
        # The white-wall corridor is the base geometry; texture_mask_burst,
        # illumination_drop, and dual_failure_slam_and_range share it but
        # vary degradation (see DegradationSchedule.for_scenario).
        if name in ("corridor_white_wall", "texture_mask_burst",
                    "illumination_drop", "dual_failure_slam_and_range"):
            # Corridor with a textureless wall ahead. waypoint 3 is
            # placed past the wall on purpose: the only way to
            # complete the mission *without colliding* is to detect
            # the wall and avoid it. The acoustic backup is what makes
            # that detection possible when SLAM has nothing to track
            # on the white surface.
            return World(
                obstacles=[
                    Box(-1, 30, -3, -2, 0, 5, texture=1.0),    # left wall
                    Box(-1, 30,  2,  3, 0, 5, texture=1.0),    # right wall
                    Box(15, 16, -2,  2, 0, 5, texture=0.8),    # textured wall: pure obstacle
                ],
                waypoints=[np.array([5.0, 0.0, 1.5]),
                           np.array([14.0, 0.0, 1.5]),
                           np.array([18.0, 0.0, 1.5])],
            )
        if name == "open_with_obstacle":
            return World(
                obstacles=[Box(8, 10, -1, 1, 0, 3, texture=1.0)],
                waypoints=[np.array([5.0, 0.0, 1.5]),
                           np.array([15.0, 0.0, 1.5])],
            )
        if name == "empty":
            return World(
                obstacles=[],
                waypoints=[np.array([5.0, 0.0, 1.5])],
            )
        raise ValueError(f"unknown scenario: {name!r}")

    def collision(self, p: np.ndarray, radius: float = 0.18) -> bool:
        if p[2] < self.ground_z + radius or p[2] > self.ceiling_z - radius:
            return True
        for b in self.obstacles:
            if (b.xmin - radius <= p[0] <= b.xmax + radius
                and b.ymin - radius <= p[1] <= b.ymax + radius
                and b.zmin - radius <= p[2] <= b.zmax + radius):
                return True
        return False

    def distance_along(self, p: np.ndarray, dir_w: np.ndarray,
                       max_m: float = 10.0) -> float:
        """Nearest hit along a unit ray from p in world frame. Returns
        max_m if nothing is hit (incl. ground / ceiling)."""
        d = max_m
        # Ground / ceiling
        if dir_w[2] < -1e-6:
            d = min(d, (self.ground_z - p[2]) / dir_w[2])
        elif dir_w[2] > 1e-6:
            d = min(d, (self.ceiling_z - p[2]) / dir_w[2])
        # Boxes (slab method)
        for b in self.obstacles:
            t = _ray_box(p, dir_w, b)
            if t is not None and t < d:
                d = t
        return max(0.0, d)

    def texture_along(self, p: np.ndarray, yaw: float,
                      look_m: float = 5.0) -> float:
        dir_w = np.array([np.cos(yaw), np.sin(yaw), 0.0])
        d = self.distance_along(p, dir_w, max_m=look_m)
        if d >= look_m - 1e-3:
            return 1.0  # open space, default texture
        # Query the box that was hit (linear scan, fine for our N <= 10)
        hit = p + dir_w * d
        for b in self.obstacles:
            if (b.xmin - 0.05 <= hit[0] <= b.xmax + 0.05
                and b.ymin - 0.05 <= hit[1] <= b.ymax + 0.05
                and b.zmin - 0.05 <= hit[2] <= b.zmax + 0.05):
                return float(b.texture)
        return 1.0

    def min_clearance(self, p: np.ndarray) -> float:
        """Smallest distance from p to any obstacle face or floor/ceiling."""
        c = min(p[2] - self.ground_z, self.ceiling_z - p[2])
        for b in self.obstacles:
            dx = max(b.xmin - p[0], 0.0, p[0] - b.xmax)
            dy = max(b.ymin - p[1], 0.0, p[1] - b.ymax)
            dz = max(b.zmin - p[2], 0.0, p[2] - b.zmax)
            c = min(c, float(np.sqrt(dx * dx + dy * dy + dz * dz)))
        return c


def _ray_box(p: np.ndarray, d: np.ndarray, b: Box) -> float | None:
    tmin, tmax = -np.inf, np.inf
    for axis, (lo, hi) in enumerate(((b.xmin, b.xmax),
                                      (b.ymin, b.ymax),
                                      (b.zmin, b.zmax))):
        if abs(d[axis]) < 1e-9:
            if p[axis] < lo or p[axis] > hi:
                return None
            continue
        t1 = (lo - p[axis]) / d[axis]
        t2 = (hi - p[axis]) / d[axis]
        t1, t2 = (t1, t2) if t1 < t2 else (t2, t1)
        tmin = max(tmin, t1)
        tmax = min(tmax, t2)
        if tmin > tmax:
            return None
    return float(tmin) if tmin > 0 else None
