"""Programmable degradation schedules.

Each scenario can declare a list of degradation events: SLAM track
loss windows, range-sensor blackouts, IMU bias bursts, etc. The
runner consults this on every tick and modulates the synthetic
sensor outputs accordingly. This is what lets us make controlled
SLAM-dropout claims in the paper.
"""

from dataclasses import dataclass


@dataclass
class DegradationEvent:
    kind: str            # "slam_dropout" | "range_invalid" | "texture_mask"
    t_start_s: float
    t_end_s: float
    severity: float = 1.0


@dataclass
class DegradationSchedule:
    events: list[DegradationEvent]

    def slam_factor(self, t: float) -> float:
        """Multiplier on the synthetic SLAM inlier count, 1.0 = healthy."""
        f = 1.0
        for e in self.events:
            if e.kind in ("slam_dropout", "texture_mask") and e.t_start_s <= t < e.t_end_s:
                f *= max(0.0, 1.0 - e.severity)
        return f

    def range_valid(self, t: float) -> bool:
        for e in self.events:
            if e.kind == "range_invalid" and e.t_start_s <= t < e.t_end_s:
                return False
        return True

    @staticmethod
    def for_scenario(name: str) -> "DegradationSchedule":
        if name == "corridor_white_wall":
            # The white wall in world.py already drops textures
            # geometrically; this adds nothing extra.
            return DegradationSchedule(events=[])
        if name == "texture_mask_burst":
            return DegradationSchedule(events=[
                DegradationEvent("texture_mask", 8.0, 10.0, severity=1.0),
            ])
        if name == "illumination_drop":
            return DegradationSchedule(events=[
                DegradationEvent("slam_dropout", 6.0, 7.5, severity=0.9),
            ])
        if name == "dual_failure_slam_and_range":
            return DegradationSchedule(events=[
                DegradationEvent("slam_dropout",   5.0, 8.0, severity=1.0),
                DegradationEvent("range_invalid", 5.5, 7.5),
            ])
        return DegradationSchedule(events=[])
