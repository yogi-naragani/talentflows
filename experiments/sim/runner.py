"""Single-trial runner.

Wires the synthetic sensors and dynamics to the same library code
that the ROS 2 nodes use: HealthMonitor, GeminiNanoClient or
RuleTreeAdvisor, SafetySupervisor, and the LQR controller. The
advisor ticks at advisor_hz; the controller ticks at control_hz.

Returns a TrialMetrics for the trial. The runner is intentionally
deterministic given a seed so paper experiments are reproducible.
"""

from dataclasses import dataclass
import numpy as np

from gps_denied_drone.reasoning.monitor import HealthMonitor, signals_to_json_dict
from gps_denied_drone.reasoning.gemini_nano import GeminiNanoClient
from gps_denied_drone.reasoning.rule_tree import RuleTreeAdvisor
from gps_denied_drone.reasoning.safety import SafetySupervisor
from gps_denied_drone.control.lqr import LQRController, LQRConfig

from .quadrotor import Quadrotor, QuadrotorParams, QuadrotorState
from .world import World
from .degradation import DegradationSchedule
from .sensors_sim import SensorSim
from .metrics import TrialMetrics


@dataclass
class TrialConfig:
    scenario: str = "corridor_white_wall"
    advisor: str = "rule_tree"            # "slm" or "rule_tree"
    seed: int = 0
    duration_s: float = 30.0
    control_hz: float = 100.0
    advisor_hz: float = 1.0
    waypoint_tolerance_m: float = 0.5
    use_acoustic: bool = True             # ablation knob


def run_trial(cfg: TrialConfig) -> TrialMetrics:
    rng = np.random.default_rng(cfg.seed)
    world = World.scenario(cfg.scenario)
    schedule = DegradationSchedule.for_scenario(cfg.scenario)
    quad = Quadrotor(QuadrotorParams(),
                     QuadrotorState(p=np.array([0.0, 0.0, 1.5])))
    sensors = SensorSim(world, schedule, rng)
    monitor = HealthMonitor(window_s=5.0)
    safety = SafetySupervisor()
    advisor = (RuleTreeAdvisor() if cfg.advisor == "rule_tree"
               else GeminiNanoClient(backend=None))
    controller = LQRController(LQRConfig())

    # Mission state
    waypoints = list(world.waypoints)
    wp_idx = 0
    waypoint = waypoints[0]

    # Per-tick stats
    dt_ctrl = 1.0 / cfg.control_hz
    dt_adv = 1.0 / cfg.advisor_hz
    n_ctrl = int(cfg.duration_s * cfg.control_hz)
    next_advisor_t = 0.0

    collisions = 0
    collided = False
    min_clearance = float("inf")
    safety_clamps = 0
    advisor_ticks = 0
    pos_log: list[np.ndarray] = []
    current_mode = "nominal"

    t = 0.0
    for _ in range(n_ctrl):
        readings = sensors.step(t, dt_ctrl, quad.s)

        # Push monitor windows at sensor rate (cheap)
        monitor.push_slam(readings.slam_tracking_ok, readings.slam_inliers)
        monitor.push_range(readings.range_valid)

        # Advisor tick (low rate)
        if t >= next_advisor_t:
            next_advisor_t += dt_adv
            advisor_ticks += 1
            signals = monitor.observe(
                pos=tuple(quad.s.p), vel=tuple(quad.s.v),
                yaw_deg=float(np.degrees(quad.s.yaw)),
                slam_tracking_ok=readings.slam_tracking_ok,
                slam_inliers=readings.slam_inliers,
                range_m=readings.range_m if readings.range_valid else None,
                ac_prox=readings.acoustic_proximity if cfg.use_acoustic else None,
                ac_clear=readings.acoustic_clearance_m if cfg.use_acoustic else None,
                ac_conf=readings.acoustic_confidence if cfg.use_acoustic else 0.0,
                battery_pct=readings.battery_pct,
                imu_vibration=0.0,
                motor_saturation_ratio=0.0,
                waypoint=tuple(waypoint),
            )
            obs = signals_to_json_dict(signals)
            action = advisor.decide(obs)
            safe_action, report = safety.apply(
                action, current_pos=tuple(quad.s.p),
                battery_pct=readings.battery_pct)
            if report.clamped:
                safety_clamps += 1
            monitor.set_mode(safe_action.mode)
            current_mode = safe_action.mode

            # Apply advisor outputs to controller and waypoint nudge
            controller.set_speed_cap(safe_action.speed_cap_mps)
            controller.set_weight_overrides(safe_action.mpc_weights)
            waypoint = waypoint + np.array(safe_action.waypoint_delta_m)
            waypoint[2] = max(0.5, waypoint[2])

            # Mode-specific reference handling: hover/land freezes wp
            if safe_action.mode in ("hover", "land"):
                waypoint = quad.s.p.copy()
            elif safe_action.mode == "return":
                waypoint = np.array([0.0, 0.0, 1.5])

        # Controller tick (high rate). Acoustic clearance is enforced
        # here, at controller rate, not at advisor rate -- the advisor
        # alone is too slow to react to obstacles encountered between
        # ticks. The use_acoustic ablation flag gates this entire path.
        if cfg.use_acoustic:
            controller.set_acoustic(
                proximity=readings.acoustic_proximity,
                confidence=readings.acoustic_confidence)
        else:
            controller.set_acoustic(None, 0.0)
        v_des, yaw_rate_des = controller.compute(
            p=quad.s.p, v=quad.s.v, yaw=quad.s.yaw, p_ref=waypoint)
        quad.step(dt_ctrl, v_des, yaw_rate_des)

        # Bookkeeping. Collision is binary and terminal: point-mass
        # dynamics have no contact response, so we end the trial on
        # first contact rather than tallying ticks-inside-the-wall.
        if not collided and world.collision(quad.s.p):
            collisions = 1
            collided = True
        min_clearance = min(min_clearance, world.min_clearance(quad.s.p))
        pos_log.append(quad.s.p.copy())
        if collided:
            break

        # Waypoint progression: only advance when the advisor wants
        # forward motion. Hover / retreat / return / land suppress
        # progression so the controller honors the advisor's intent.
        if current_mode in ("nominal", "slow_advance", "search_features"):
            if (wp_idx < len(waypoints)
                    and np.linalg.norm(quad.s.p - waypoints[wp_idx])
                    < cfg.waypoint_tolerance_m):
                wp_idx += 1
                if wp_idx < len(waypoints):
                    waypoint = waypoints[wp_idx]

        t += dt_ctrl

    completed = wp_idx >= len(waypoints)
    ate = _ate(np.stack(pos_log), waypoints)
    return TrialMetrics(
        scenario=cfg.scenario,
        advisor=cfg.advisor,
        seed=cfg.seed,
        duration_s=cfg.duration_s,
        completed=completed,
        collisions=collisions,
        min_clearance_m=float(min_clearance),
        ate_rmse_m=float(ate),
        safety_clamp_count=safety_clamps,
        safety_clamp_rate=safety_clamps / max(1, advisor_ticks),
        advisor_ticks=advisor_ticks,
        final_battery_pct=float(sensors._battery_pct),
        waypoints_reached=wp_idx,
        extra={"use_acoustic": cfg.use_acoustic},
    )


def _ate(traj: np.ndarray, waypoints: list[np.ndarray]) -> float:
    """Crude ATE: minimum distance from each pose to the polyline of
    waypoints. Good enough for relative comparisons across ablations."""
    if not waypoints:
        return 0.0
    wps = np.stack([np.array([0.0, 0.0, 1.5])] + waypoints)
    err2 = []
    for p in traj:
        d_min = float("inf")
        for i in range(len(wps) - 1):
            d_min = min(d_min, _dist_to_segment(p, wps[i], wps[i + 1]))
        err2.append(d_min ** 2)
    return float(np.sqrt(np.mean(err2)))


def _dist_to_segment(p, a, b) -> float:
    ab = b - a
    t = np.clip(np.dot(p - a, ab) / max(1e-9, np.dot(ab, ab)), 0.0, 1.0)
    return float(np.linalg.norm(p - (a + t * ab)))
