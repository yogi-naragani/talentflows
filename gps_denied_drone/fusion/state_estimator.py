"""Tightly-ish coupled state estimator.

Loosely couples VSLAM pose, IMU pre-integration (assumed handled inside
the SLAM back-end), and the 1D range sensor as a height/clearance
constraint. This is a placeholder Kalman-style update; the paper version
should specify the chosen filter (ESKF / MSCKF-lite) explicitly.
"""

from dataclasses import dataclass
import numpy as np


@dataclass
class State:
    p_w: np.ndarray          # 3
    v_w: np.ndarray          # 3
    R_wb: np.ndarray         # 3x3
    bias_a: np.ndarray       # 3
    bias_g: np.ndarray       # 3
    P: np.ndarray            # 15x15 covariance


class StateEstimator:
    def __init__(self):
        self.x = State(
            p_w=np.zeros(3),
            v_w=np.zeros(3),
            R_wb=np.eye(3),
            bias_a=np.zeros(3),
            bias_g=np.zeros(3),
            P=np.eye(15) * 1e-3,
        )

    def predict(self, dt: float, accel_b: np.ndarray, gyro_b: np.ndarray) -> None:
        raise NotImplementedError

    def update_slam_pose(self, T_wc: np.ndarray, cov: np.ndarray) -> None:
        raise NotImplementedError

    def update_range(self, range_m: float, beam_dir_b: np.ndarray,
                     ground_normal_w: np.ndarray, ground_d_w: float) -> None:
        """Update with a 1D beam intersecting a known ground plane."""
        raise NotImplementedError
