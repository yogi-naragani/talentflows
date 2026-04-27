"""Visual SLAM front-end + back-end interface.

The implementation is intentionally a thin facade over an external SLAM
library (e.g. ORB-SLAM3, OpenVSLAM, or a custom keyframe-based system).
The paper's contribution is not the SLAM core itself but the way its
outputs are consumed by the fusion and reasoning layers.
"""

from dataclasses import dataclass
import numpy as np


@dataclass
class SlamPose:
    t_ns: int
    T_wc: np.ndarray          # 4x4 SE(3), camera in world (up-to-scale)
    covariance: np.ndarray    # 6x6
    tracking_ok: bool
    n_inliers: int


@dataclass
class SlamMapSummary:
    n_keyframes: int
    n_landmarks: int
    median_parallax_deg: float


class VisualSLAM:
    def __init__(self, config: dict):
        self.config = config
        self._scale = 1.0  # metric scale, set externally from 1D sensor

    def process_frame(self, t_ns: int, image: np.ndarray) -> SlamPose:
        raise NotImplementedError

    def map_summary(self) -> SlamMapSummary:
        raise NotImplementedError

    def set_metric_scale(self, scale: float) -> None:
        """Apply the scale factor recovered from the 1D range sensor."""
        self._scale = float(scale)

    def is_degraded(self) -> bool:
        """Heuristic flag the reasoning layer can poll."""
        raise NotImplementedError
