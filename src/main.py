"""End-to-end loop tying perception, fusion, control, and reasoning.

This is intentionally schematic; replace the placeholders with concrete
backends (sim or hardware) for experiments.
"""

from __future__ import annotations
import time

from src.perception.visual_slam import VisualSLAM
from src.sensors.range_sensor import RangeSensor, RangeReading
from src.fusion.scale_recovery import ScaleEstimator
from src.fusion.state_estimator import StateEstimator
from src.control.mpc import QuadrotorMPC, MPCConfig
from src.reasoning.gemini_nano import GeminiNanoClient, StateSummary


CONTROL_HZ = 100
REASONING_HZ = 2


def run(sensor_source, image_source, actuator_sink) -> None:
    slam = VisualSLAM(config={})
    rangefinder = RangeSensor()
    scale = ScaleEstimator()
    estimator = StateEstimator()
    mpc = QuadrotorMPC(MPCConfig())
    reasoner = GeminiNanoClient()

    last_reasoning = 0.0
    waypoint = (0.0, 0.0, 1.5)
    speed_cap = 3.0

    for t_ns, image, range_m, imu in sensor_source:
        # Perception
        pose = slam.process_frame(t_ns, image)

        # 1D sensor + scale recovery
        r = rangefinder.push(RangeReading(t_ns, range_m, valid=True))
        if r.valid and pose.tracking_ok:
            predicted = _slam_predicted_range(pose)
            scale.push(predicted, r.range_m)
            s = scale.estimate()
            if s is not None:
                slam.set_metric_scale(s)

        # Fusion
        estimator.predict(dt=imu.dt, accel_b=imu.accel, gyro_b=imu.gyro)
        if pose.tracking_ok:
            estimator.update_slam_pose(pose.T_wc, pose.covariance)
        if r.valid:
            estimator.update_range(
                r.range_m,
                beam_dir_b=imu.beam_dir_b,
                ground_normal_w=imu.ground_normal_w,
                ground_d_w=imu.ground_d_w,
            )

        # Reasoning (low rate, advisory only)
        now = time.monotonic()
        if now - last_reasoning > 1.0 / REASONING_HZ:
            last_reasoning = now
            summary = StateSummary(
                pos_xyz_m=tuple(estimator.x.p_w),
                vel_xyz_mps=tuple(estimator.x.v_w),
                yaw_deg=0.0,
                battery_pct=imu.battery_pct,
                slam_tracking_ok=pose.tracking_ok,
                slam_inliers=pose.n_inliers,
                range_m=rangefinder.latest_filtered(),
                waypoint_xyz_m=waypoint,
            )
            action = reasoner.decide(summary)
            waypoint = _apply_delta(waypoint, action.waypoint_delta_m)
            speed_cap = action.speed_cap_mps

        # Control
        sol = mpc.solve(x0=_pack_state(estimator.x),
                        x_ref=_reference(waypoint, speed_cap))
        actuator_sink.send(sol.u0)


def _slam_predicted_range(pose) -> float:
    raise NotImplementedError


def _apply_delta(wp, d):
    return (wp[0] + d[0], wp[1] + d[1], wp[2] + d[2])


def _pack_state(state):
    raise NotImplementedError


def _reference(wp, speed_cap):
    raise NotImplementedError
