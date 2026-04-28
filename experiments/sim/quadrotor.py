"""Point-mass quadrotor with first-order velocity tracking.

Intentionally simple: position and velocity in world frame, attitude
modelled as a yaw scalar with first-order tracking of a yaw setpoint.
Enough to expose the controller's response to weight modulation,
speed caps, and waypoint nudges from the advisor; not enough to
study aerodynamics. The full Gazebo path uses real rigid-body
dynamics; the Python sim is for paper-grade ablations.
"""

from dataclasses import dataclass, field
import numpy as np


@dataclass
class QuadrotorState:
    p: np.ndarray = field(default_factory=lambda: np.zeros(3))   # world position
    v: np.ndarray = field(default_factory=lambda: np.zeros(3))   # world velocity
    yaw: float = 0.0                                              # rad
    yaw_rate: float = 0.0
    motor_rpm: np.ndarray = field(default_factory=lambda: np.full(4, 5000.0))


@dataclass
class QuadrotorParams:
    mass_kg: float = 0.95
    accel_tau_s: float = 0.10        # 1st-order velocity tracking time constant
    yaw_tau_s: float = 0.20
    max_accel_mps2: float = 8.0
    max_yaw_rate_radps: float = 2.0


class Quadrotor:
    """Acts on a velocity setpoint v_des and a yaw-rate setpoint."""

    def __init__(self, params: QuadrotorParams | None = None,
                 state: QuadrotorState | None = None):
        self.p = params or QuadrotorParams()
        self.s = state or QuadrotorState()

    def step(self, dt: float, v_des: np.ndarray, yaw_rate_des: float) -> None:
        # Saturate desired velocity rate of change
        dv = (v_des - self.s.v) / self.p.accel_tau_s
        a = np.clip(dv, -self.p.max_accel_mps2, self.p.max_accel_mps2)
        self.s.v = self.s.v + a * dt
        self.s.p = self.s.p + self.s.v * dt

        dy = (yaw_rate_des - self.s.yaw_rate) / self.p.yaw_tau_s
        self.s.yaw_rate = self.s.yaw_rate + dy * dt
        self.s.yaw_rate = float(np.clip(
            self.s.yaw_rate, -self.p.max_yaw_rate_radps, self.p.max_yaw_rate_radps))
        self.s.yaw = (self.s.yaw + self.s.yaw_rate * dt) % (2 * np.pi)

        # Crude RPM proxy from |a|+gravity (used to drive ego-noise band)
        thrust_proxy = np.linalg.norm(a) + 9.81
        self.s.motor_rpm = np.full(4, 3000.0 + 600.0 * thrust_proxy)
