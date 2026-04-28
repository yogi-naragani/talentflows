"""Hand-coded rule-tree advisor.

This is the baseline that the on-device SLM is compared against. It
consumes the *same* HealthSignals observation and produces the *same*
ReasoningAction schema, so the SafetySupervisor and downstream
publishers do not need to know which advisor is in use.

Why this exists in this paper: the central empirical claim is that a
small on-device LLM matches or beats a hand-coded rule tree across
diverse degradation scenarios. The rule tree must therefore be
realistic -- not a strawman. We follow the same decision guidance
written into the SLM's system prompt, so any quality gap reflects
the LLM's contextual judgment rather than asymmetric knowledge.

Tuning of the thresholds below is intentionally exposed as
constants; reviewers can re-tune them per scenario in their reproductions.
"""

from gps_denied_drone.reasoning.gemini_nano import ReasoningAction
from gps_denied_drone.reasoning.monitor import HealthSignals


# Thresholds intentionally surfaced -- this is "what a reasonable
# engineer would write." Scenario-specific tuning is fair game for
# the baseline, and we report whether the SLM beats the best-tuned
# version of these.
SLAM_INLIER_OK = 100
SLAM_INLIER_DEGRADED = 50
RANGE_VALID_OK = 0.7
ACOUSTIC_CONF_TRUST = 0.5
ACOUSTIC_PROX_RETREAT = 0.7      # threshold for triggering retreat
ACOUSTIC_PROX_RESUME = 0.3        # hysteresis: stay in retreat above this
RETREAT_DELTA_M = 0.6
HOVER_STALE_S = 8.0
BATTERY_RETURN_PCT = 25.0
BATTERY_LAND_PCT = 12.0


class RuleTreeAdvisor:
    """Drop-in replacement for GeminiNanoClient with the same .decide()
    signature. Uses the structured observation produced by
    HealthMonitor.observe()."""

    def decide(self, observation: dict) -> ReasoningAction:
        s = _signals_from_dict(observation)
        recent = observation.get("recent_modes") or []

        # Battery overrides
        if s.battery_pct <= BATTERY_LAND_PCT:
            return ReasoningAction(
                mode="land", speed_cap_mps=1.0,
                rationale="rule: battery<=land", confidence=1.0)
        if s.battery_pct <= BATTERY_RETURN_PCT:
            return ReasoningAction(
                mode="return", speed_cap_mps=3.0,
                rationale="rule: battery<=return", confidence=1.0)

        # Acoustic alarm gates everything else. Hysteresis: trigger
        # retreat at ACOUSTIC_PROX_RETREAT, stay in retreat as long
        # as proximity is still above ACOUSTIC_PROX_RESUME.
        if s.acoustic_confidence > ACOUSTIC_CONF_TRUST and s.acoustic_proximity:
            max_prox = max(s.acoustic_proximity.values())
            in_retreat = bool(recent) and recent[-1] == "retreat"
            trigger = max_prox > ACOUSTIC_PROX_RETREAT or (
                in_retreat and max_prox > ACOUSTIC_PROX_RESUME)
            if trigger:
                return ReasoningAction(
                    mode="retreat",
                    waypoint_delta_m=_retreat_delta(s),
                    speed_cap_mps=0.8,
                    mpc_weights={"w_perception": 0.5, "Q_vel": 2.0},
                    sensor_trust={"slam": 0.2, "acoustic": 0.9, "range": 0.5},
                    rationale="rule: acoustic alarm",
                    confidence=0.9)

        # SLAM lost
        if not s.slam_tracking_ok:
            return ReasoningAction(
                mode="hover", speed_cap_mps=0.0,
                sensor_trust={"slam": 0.1, "acoustic": 0.6, "range": 0.7},
                rationale="rule: slam lost", confidence=0.8)

        # SLAM degrading
        if (s.slam_inliers < SLAM_INLIER_DEGRADED
                or s.slam_inlier_trend == "falling"):
            return ReasoningAction(
                mode="slow_advance", speed_cap_mps=1.5,
                yaw_strategy="track_features",
                mpc_weights={"w_perception": 2.0},
                rationale="rule: slam degrading", confidence=0.7)

        # Range mostly invalid
        if s.range_valid_ratio_5s < RANGE_VALID_OK:
            return ReasoningAction(
                mode="nominal", speed_cap_mps=3.0,
                sensor_trust={"range": 0.3},
                rationale="rule: range unreliable", confidence=0.7)

        # Stuck in hover for too long
        if (recent
                and observation.get("seconds_in_current_mode", 0.0) > HOVER_STALE_S
                and recent[-1] == "hover"):
            return ReasoningAction(
                mode="search_features", yaw_strategy="sweep_search",
                speed_cap_mps=0.5, replan=True,
                rationale="rule: hover stale", confidence=0.6)

        # Default: nominal
        return ReasoningAction(
            mode="nominal", speed_cap_mps=4.0,
            rationale="rule: nominal", confidence=0.9)


def _retreat_delta(s: HealthSignals) -> tuple[float, float, float]:
    if s.acoustic_worst_dir == "front":  return (-RETREAT_DELTA_M, 0.0, 0.0)
    if s.acoustic_worst_dir == "back":   return ( RETREAT_DELTA_M, 0.0, 0.0)
    if s.acoustic_worst_dir == "left":   return (0.0,  RETREAT_DELTA_M, 0.0)
    if s.acoustic_worst_dir == "right":  return (0.0, -RETREAT_DELTA_M, 0.0)
    if s.acoustic_worst_dir == "down":   return (0.0, 0.0,  RETREAT_DELTA_M)
    if s.acoustic_worst_dir == "up":     return (0.0, 0.0, -RETREAT_DELTA_M)
    return (0.0, 0.0, 0.0)


def _signals_from_dict(d: dict) -> HealthSignals:
    """Lift the JSON observation back into a typed HealthSignals so the
    rule tree reads naturally. Missing keys take dataclass defaults."""
    return HealthSignals(
        pos_xyz_m=tuple(d.get("pos_xyz_m", (0, 0, 0))),
        vel_xyz_mps=tuple(d.get("vel_xyz_mps", (0, 0, 0))),
        yaw_deg=float(d.get("yaw_deg", 0.0)),
        speed_mps=float(d.get("speed_mps", 0.0)),
        slam_tracking_ok=bool(d.get("slam_tracking_ok", True)),
        slam_inliers=int(d.get("slam_inliers", 0)),
        slam_inlier_trend=str(d.get("slam_inlier_trend", "steady")),
        slam_seconds_since_dropout=float(
            d.get("slam_seconds_since_dropout", float("inf")) or float("inf")),
        range_m=d.get("range_m"),
        range_valid_ratio_5s=float(d.get("range_valid_ratio_5s", 1.0)),
        acoustic_proximity=d.get("acoustic_proximity"),
        acoustic_clearance_m=d.get("acoustic_clearance_m"),
        acoustic_confidence=float(d.get("acoustic_confidence", 0.0)),
        acoustic_worst_dir=d.get("acoustic_worst_dir"),
        battery_pct=float(d.get("battery_pct", 100.0)),
        imu_vibration=float(d.get("imu_vibration", 0.0)),
        motor_saturation_ratio=float(d.get("motor_saturation_ratio", 0.0)),
        waypoint_xyz_m=tuple(d.get("waypoint_xyz_m", (0, 0, 0))),
        distance_to_waypoint_m=float(d.get("distance_to_waypoint_m", 0.0)),
        seconds_in_current_mode=float(d.get("seconds_in_current_mode", 0.0)),
        seconds_since_mission_start=float(d.get("seconds_since_mission_start", 0.0)),
        recent_modes=list(d.get("recent_modes", [])),
    )
