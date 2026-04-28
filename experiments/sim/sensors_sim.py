"""Synthetic sensors: feature counts, 1D range, acoustic, IMU, battery.

These produce values in the same shape as the ROS messages the real
nodes consume, so the runner can call the library code directly
without the ROS plumbing.
"""

from dataclasses import dataclass
import numpy as np

from .world import World
from .degradation import DegradationSchedule
from .quadrotor import QuadrotorState


@dataclass
class SyntheticReadings:
    slam_tracking_ok: bool
    slam_inliers: int
    range_m: float
    range_valid: bool
    acoustic_proximity: dict
    acoustic_clearance_m: float
    acoustic_confidence: float
    battery_pct: float


class SensorSim:
    def __init__(self, world: World, schedule: DegradationSchedule,
                 rng: np.random.Generator):
        self.world = world
        self.schedule = schedule
        self.rng = rng
        self._battery_pct = 100.0

    def step(self, t: float, dt: float, state: QuadrotorState) -> SyntheticReadings:
        # Battery drain: ~0.05 %/s baseline + extra under high accel proxy
        self._battery_pct = max(0.0, self._battery_pct - 0.05 * dt)

        # SLAM inliers proxy: texture * geometry, with a noise floor.
        tex = self.world.texture_along(state.p, state.yaw)
        deg = self.schedule.slam_factor(t)
        base = 220.0
        inliers = int(max(0.0, base * tex * deg + self.rng.normal(0, 8)))
        tracking_ok = inliers >= 25

        # 1D rangefinder pointing -z
        true_h = state.p[2] - self.world.ground_z
        meas = true_h + self.rng.normal(0, 0.02)
        range_valid = self.schedule.range_valid(t) and 0.05 < meas < 30.0

        # Acoustic proximity: high when an obstacle is close along a
        # body axis. Proximity = clip(1 - d/d_max, 0, 1).
        ac_prox, ac_clear, ac_conf = self._acoustic(state)

        return SyntheticReadings(
            slam_tracking_ok=tracking_ok,
            slam_inliers=inliers,
            range_m=float(meas),
            range_valid=bool(range_valid),
            acoustic_proximity=ac_prox,
            acoustic_clearance_m=ac_clear,
            acoustic_confidence=ac_conf,
            battery_pct=self._battery_pct,
        )

    def _acoustic(self, s: QuadrotorState) -> tuple[dict, float, float]:
        d_max = 4.0  # passive proximity reach (limited by SNR floor)
        # Body axes in world: forward = (cos yaw, sin yaw, 0), etc.
        c, sn = float(np.cos(s.yaw)), float(np.sin(s.yaw))
        body_axes = {
            "front": np.array([ c,  sn, 0.0]),
            "back":  np.array([-c, -sn, 0.0]),
            "left":  np.array([-sn,  c, 0.0]),
            "right": np.array([ sn, -c, 0.0]),
            "up":    np.array([0.0, 0.0, 1.0]),
            "down":  np.array([0.0, 0.0,-1.0]),
        }
        proximity, clearances = {}, {}
        for d, dir_w in body_axes.items():
            d_hit = self.world.distance_along(s.p, dir_w, max_m=d_max + 1.0)
            # Noisy proximity: simulates the -20 dB SNR floor of passive
            # ego-noise reflectometry. Sigma chosen so the rule_tree
            # still detects the wall reliably but the trigger distance
            # varies per trial.
            prox_clean = max(0.0, 1.0 - d_hit / d_max)
            prox = float(np.clip(prox_clean + self.rng.normal(0, 0.04),
                                 0.0, 1.0))
            proximity[d] = prox
            clearances[d] = float(d_hit + self.rng.normal(0, 0.05))
        worst = max(proximity, key=proximity.get)
        worst_clear = clearances[worst]
        # Confidence rises with proximity (only useful when something
        # is genuinely close); ego-noise SNR otherwise dominates.
        confidence = float(np.clip(proximity[worst] * 1.2
                                   + self.rng.normal(0, 0.05),
                                   0.0, 1.0))
        return proximity, worst_clear, confidence
