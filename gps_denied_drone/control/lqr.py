"""Analytical LQR-style position controller.

The full nonlinear MPC with CasADi (see ``mpc.py``) is the production
path. This module is a closed-form linearized fallback used by the
Python-only simulator: it produces a velocity setpoint that the
simplified quadrotor can track. The advisor's MPC weight overrides
and sensor-trust knobs are honored here as gain modulations, so the
sim still demonstrates the perception-aware behavior end-to-end.
"""

from dataclasses import dataclass
import numpy as np


@dataclass
class LQRConfig:
    kp_pos: float = 1.4
    kd_vel: float = 0.6
    kp_yaw: float = 1.5
    speed_cap_mps: float = 5.0
    accel_cap_mps2: float = 6.0


class LQRController:
    def __init__(self, cfg: LQRConfig | None = None):
        self.cfg = cfg or LQRConfig()
        self._w_pos_scale = 1.0
        self._w_vel_scale = 1.0
        self._speed_cap = self.cfg.speed_cap_mps

    def set_speed_cap(self, v_max: float) -> None:
        self._speed_cap = float(np.clip(v_max, 0.0, self.cfg.speed_cap_mps))

    def set_weight_overrides(self, scales: dict | None) -> None:
        if not scales:
            self._w_pos_scale = 1.0
            self._w_vel_scale = 1.0
            return
        self._w_pos_scale = float(np.clip(scales.get("Q_pos", 1.0), 0.1, 10.0))
        self._w_vel_scale = float(np.clip(scales.get("Q_vel", 1.0), 0.1, 10.0))

    def compute(self, p: np.ndarray, v: np.ndarray, yaw: float,
                p_ref: np.ndarray, yaw_ref: float = 0.0
                ) -> tuple[np.ndarray, float]:
        kp = self.cfg.kp_pos * self._w_pos_scale
        kd = self.cfg.kd_vel * self._w_vel_scale
        a_des = kp * (p_ref - p) - kd * v
        a_des = np.clip(a_des, -self.cfg.accel_cap_mps2, self.cfg.accel_cap_mps2)
        v_des = v + a_des * 0.05  # one MPC tick lookahead
        # Norm-cap to speed limit
        speed = float(np.linalg.norm(v_des))
        if speed > self._speed_cap and speed > 1e-6:
            v_des = v_des * (self._speed_cap / speed)

        yaw_err = ((yaw_ref - yaw + np.pi) % (2 * np.pi)) - np.pi
        yaw_rate_des = float(np.clip(self.cfg.kp_yaw * yaw_err, -2.0, 2.0))
        return v_des, yaw_rate_des
