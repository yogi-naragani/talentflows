"""Health monitor: aggregates all sensor and estimator signals into a
single structured observation that the SLM consumes.

Design intent: the monitor produces *facts*, never *decisions*. It does
no policy logic. The SLM (advisor) reads the structured observation
and emits an action; a separate SafetySupervisor enforces hard limits
on the action. This keeps the if/else tree out of Python -- the SLM
makes the contextual call, but never the safety call.
"""

from collections import deque
from dataclasses import dataclass, field, asdict
from typing import Iterable
import math
import time


@dataclass
class HealthSignals:
    # Pose / motion
    pos_xyz_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    vel_xyz_mps: tuple[float, float, float] = (0.0, 0.0, 0.0)
    yaw_deg: float = 0.0
    speed_mps: float = 0.0

    # SLAM
    slam_tracking_ok: bool = True
    slam_inliers: int = 0
    slam_inlier_trend: str = "steady"          # rising | steady | falling
    slam_seconds_since_dropout: float = float("inf")

    # 1D rangefinder
    range_m: float | None = None
    range_valid_ratio_5s: float = 1.0           # fraction of valid samples

    # Acoustic backup
    acoustic_proximity: dict | None = None
    acoustic_clearance_m: float | None = None
    acoustic_confidence: float = 0.0
    acoustic_worst_dir: str | None = None

    # Platform
    battery_pct: float = 100.0
    imu_vibration: float = 0.0                  # std of accel residual
    motor_saturation_ratio: float = 0.0          # 0..1 fraction recently saturated

    # Mission
    waypoint_xyz_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    distance_to_waypoint_m: float = 0.0
    seconds_in_current_mode: float = 0.0
    seconds_since_mission_start: float = 0.0

    # Recent advisor history (compact)
    recent_modes: list[str] = field(default_factory=list)


class HealthMonitor:
    """Maintains rolling windows over raw signals and produces a
    HealthSignals observation. Designed for the reasoning node.
    """

    def __init__(self, window_s: float = 5.0):
        self._window_s = window_s
        self._inliers: deque[tuple[float, int]] = deque()
        self._range_valid: deque[tuple[float, bool]] = deque()
        self._dropouts: deque[float] = deque()
        self._mode_history: deque[str] = deque(maxlen=10)
        self._mode_started_at = time.monotonic()
        self._mission_started_at = time.monotonic()
        self._current_mode = "nominal"

    # --- ingest ---
    def push_slam(self, tracking_ok: bool, inliers: int) -> None:
        now = time.monotonic()
        self._inliers.append((now, inliers))
        if not tracking_ok:
            self._dropouts.append(now)
        self._trim(now)

    def push_range(self, valid: bool) -> None:
        now = time.monotonic()
        self._range_valid.append((now, valid))
        self._trim(now)

    def set_mode(self, mode: str) -> None:
        if mode != self._current_mode:
            self._mode_history.append(mode)
            self._mode_started_at = time.monotonic()
            self._current_mode = mode

    # --- emit ---
    def observe(
        self,
        pos: tuple[float, float, float],
        vel: tuple[float, float, float],
        yaw_deg: float,
        slam_tracking_ok: bool,
        slam_inliers: int,
        range_m: float | None,
        ac_prox: dict | None,
        ac_clear: float | None,
        ac_conf: float,
        battery_pct: float,
        imu_vibration: float,
        motor_saturation_ratio: float,
        waypoint: tuple[float, float, float],
    ) -> HealthSignals:
        now = time.monotonic()
        self._trim(now)

        worst_dir = None
        if ac_prox:
            worst_dir = max(ac_prox, key=ac_prox.get)

        return HealthSignals(
            pos_xyz_m=tuple(pos),
            vel_xyz_mps=tuple(vel),
            yaw_deg=yaw_deg,
            speed_mps=math.sqrt(sum(v * v for v in vel)),
            slam_tracking_ok=slam_tracking_ok,
            slam_inliers=slam_inliers,
            slam_inlier_trend=self._inlier_trend(),
            slam_seconds_since_dropout=self._seconds_since_last_dropout(now),
            range_m=range_m,
            range_valid_ratio_5s=self._range_valid_ratio(),
            acoustic_proximity=ac_prox,
            acoustic_clearance_m=ac_clear,
            acoustic_confidence=ac_conf,
            acoustic_worst_dir=worst_dir,
            battery_pct=battery_pct,
            imu_vibration=imu_vibration,
            motor_saturation_ratio=motor_saturation_ratio,
            waypoint_xyz_m=tuple(waypoint),
            distance_to_waypoint_m=_dist(pos, waypoint),
            seconds_in_current_mode=now - self._mode_started_at,
            seconds_since_mission_start=now - self._mission_started_at,
            recent_modes=list(self._mode_history),
        )

    # --- helpers ---
    def _trim(self, now: float) -> None:
        cutoff = now - self._window_s
        while self._inliers and self._inliers[0][0] < cutoff:
            self._inliers.popleft()
        while self._range_valid and self._range_valid[0][0] < cutoff:
            self._range_valid.popleft()
        while self._dropouts and self._dropouts[0] < cutoff:
            self._dropouts.popleft()

    def _inlier_trend(self) -> str:
        if len(self._inliers) < 4:
            return "steady"
        first = sum(v for _, v in list(self._inliers)[: len(self._inliers) // 2])
        second = sum(v for _, v in list(self._inliers)[len(self._inliers) // 2 :])
        if second > first * 1.15:
            return "rising"
        if second < first * 0.85:
            return "falling"
        return "steady"

    def _seconds_since_last_dropout(self, now: float) -> float:
        if not self._dropouts:
            return float("inf")
        return now - self._dropouts[-1]

    def _range_valid_ratio(self) -> float:
        if not self._range_valid:
            return 1.0
        v = sum(1 for _, ok in self._range_valid if ok)
        return v / len(self._range_valid)


def _dist(a: Iterable[float], b: Iterable[float]) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def signals_to_json_dict(s: HealthSignals) -> dict:
    """Compact dict suitable for embedding in the SLM prompt. Drops
    None / inf values so the model isn't confused by them."""
    d = asdict(s)
    if d["slam_seconds_since_dropout"] == float("inf"):
        d["slam_seconds_since_dropout"] = None
    return {k: v for k, v in d.items() if v is not None}
