"""Metric scale recovery for monocular VSLAM using a 1D range sensor.

Idea: when the 1D beam intersects a locally planar surface (typically the
ground beneath the drone) and we have a current SLAM estimate of the
camera's height above that surface up to scale, the ratio between the
measured range and the SLAM-predicted range gives the metric scale.

We use a windowed RANSAC-style consensus to keep this robust to spurious
returns, beam misalignment near edges, and brief plane-violation events.
"""

from collections import deque
import numpy as np


class ScaleEstimator:
    def __init__(self, window: int = 50, min_inliers: int = 15,
                 inlier_tol: float = 0.05):
        self._buf: deque[tuple[float, float]] = deque(maxlen=window)
        self.min_inliers = min_inliers
        self.inlier_tol = inlier_tol  # relative

    def push(self, slam_predicted_range: float, measured_range_m: float) -> None:
        if slam_predicted_range > 1e-6 and measured_range_m > 0:
            self._buf.append((slam_predicted_range, measured_range_m))

    def estimate(self) -> float | None:
        if len(self._buf) < self.min_inliers:
            return None
        ratios = np.array([m / p for p, m in self._buf])
        med = float(np.median(ratios))
        inliers = ratios[np.abs(ratios - med) / med < self.inlier_tol]
        if inliers.size < self.min_inliers:
            return None
        return float(np.mean(inliers))
