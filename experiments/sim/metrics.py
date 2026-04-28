"""Per-trial metrics. Kept JSON-serializable so the CLI can dump CSV."""

from dataclasses import dataclass, asdict


@dataclass
class TrialMetrics:
    scenario: str
    advisor: str
    seed: int
    duration_s: float
    completed: bool
    collisions: int
    min_clearance_m: float
    ate_rmse_m: float                  # absolute trajectory error
    safety_clamp_count: int
    safety_clamp_rate: float           # clamps per advisor tick
    advisor_ticks: int
    final_battery_pct: float
    waypoints_reached: int
    extra: dict | None = None

    def to_row(self) -> dict:
        d = asdict(self)
        d.pop("extra")
        if self.extra:
            d.update({f"x_{k}": v for k, v in self.extra.items()})
        return d
