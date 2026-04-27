"""Safety supervisor.

Hard-clamps the SLM's proposed action to a verifiably safe envelope.
The SLM is *advisory*; this layer is the safety authority. If the
clamp materially changed the action, the supervisor records why so we
can audit it offline.

Hard rules:
  * altitude floor (no waypoint below MIN_ALT_M)
  * speed cap (never exceed MAX_SPEED_MPS regardless of LLM output)
  * waypoint delta cap per tick (anti-runaway)
  * mode whitelist
  * MPC weight scale clamps (anti-pathological-tuning)
  * sensor-trust clamps (every modality keeps a minimum trust unless
    fully invalid)
  * battery-low override: forces "return" or "land" below thresholds
  * geofence: optional axis-aligned bounding box
"""

from dataclasses import dataclass, replace
from typing import Iterable

VALID_MODES = {
    "nominal", "hover", "slow_advance", "retreat",
    "return", "land", "search_features",
}

MIN_ALT_M = 0.5
MAX_SPEED_MPS = 5.0
MAX_WP_DELTA_M = 2.0          # any single tick
MIN_BATTERY_RETURN = 25.0
MIN_BATTERY_LAND = 12.0
WEIGHT_MIN, WEIGHT_MAX = 0.1, 10.0
TRUST_MIN, TRUST_MAX = 0.0, 1.0


@dataclass
class Geofence:
    xmin: float = -50.0
    xmax: float =  50.0
    ymin: float = -50.0
    ymax: float =  50.0
    zmin: float = MIN_ALT_M
    zmax: float =  20.0


@dataclass
class SafetyReport:
    clamped: bool
    reasons: list[str]


class SafetySupervisor:
    def __init__(self, geofence: Geofence | None = None):
        self.geofence = geofence or Geofence()

    def apply(self, action, current_pos, battery_pct: float):
        """Return (safe_action, SafetyReport). action is a
        ReasoningAction-like object; we mutate via dataclasses.replace."""
        reasons: list[str] = []
        a = action

        # Battery overrides take precedence over LLM output.
        if battery_pct <= MIN_BATTERY_LAND:
            a = replace(a, mode="land", speed_cap_mps=min(a.speed_cap_mps, 1.0))
            reasons.append(f"battery_pct<={MIN_BATTERY_LAND}: forced land")
        elif battery_pct <= MIN_BATTERY_RETURN and a.mode not in ("return", "land"):
            a = replace(a, mode="return")
            reasons.append(f"battery_pct<={MIN_BATTERY_RETURN}: forced return")

        # Mode whitelist
        if a.mode not in VALID_MODES:
            reasons.append(f"unknown mode {a.mode!r}: -> hover")
            a = replace(a, mode="hover")

        # Speed cap
        if a.speed_cap_mps > MAX_SPEED_MPS:
            reasons.append(
                f"speed_cap_mps={a.speed_cap_mps} > {MAX_SPEED_MPS}: clamped")
            a = replace(a, speed_cap_mps=MAX_SPEED_MPS)
        if a.speed_cap_mps < 0.0:
            a = replace(a, speed_cap_mps=0.0)

        # Waypoint delta cap (norm)
        d = a.waypoint_delta_m
        norm = (d[0] ** 2 + d[1] ** 2 + d[2] ** 2) ** 0.5
        if norm > MAX_WP_DELTA_M:
            scale = MAX_WP_DELTA_M / norm
            a = replace(a, waypoint_delta_m=tuple(x * scale for x in d))
            reasons.append(
                f"waypoint_delta_m norm={norm:.2f}>{MAX_WP_DELTA_M}: scaled")

        # Altitude floor
        target_z = current_pos[2] + a.waypoint_delta_m[2]
        if target_z < MIN_ALT_M:
            lift = MIN_ALT_M - target_z
            d2 = list(a.waypoint_delta_m)
            d2[2] += lift
            a = replace(a, waypoint_delta_m=tuple(d2))
            reasons.append(f"target_z<{MIN_ALT_M}: lifted by {lift:.2f}")

        # Geofence
        target = tuple(current_pos[i] + a.waypoint_delta_m[i] for i in range(3))
        clamped_target = self._clamp_geofence(target)
        if clamped_target != target:
            d2 = tuple(clamped_target[i] - current_pos[i] for i in range(3))
            a = replace(a, waypoint_delta_m=d2)
            reasons.append("geofence: target clamped")

        # MPC weight scales
        if a.mpc_weights:
            cleaned = {}
            for k, v in a.mpc_weights.items():
                cv = max(WEIGHT_MIN, min(WEIGHT_MAX, float(v)))
                if cv != float(v):
                    reasons.append(f"mpc_weight {k} clamped {v}->{cv}")
                cleaned[k] = cv
            a = replace(a, mpc_weights=cleaned)

        # Sensor trust
        if a.sensor_trust:
            cleaned = {}
            for k, v in a.sensor_trust.items():
                cv = max(TRUST_MIN, min(TRUST_MAX, float(v)))
                if cv != float(v):
                    reasons.append(f"sensor_trust {k} clamped {v}->{cv}")
                cleaned[k] = cv
            a = replace(a, sensor_trust=cleaned)

        return a, SafetyReport(clamped=bool(reasons), reasons=reasons)

    def _clamp_geofence(self, p: Iterable[float]) -> tuple[float, float, float]:
        x, y, z = p
        g = self.geofence
        return (
            max(g.xmin, min(g.xmax, x)),
            max(g.ymin, min(g.ymax, y)),
            max(g.zmin, min(g.zmax, z)),
        )
