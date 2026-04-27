"""Simulation harness.

Recommended backend: a lightweight quadrotor sim (e.g. Flightmare or a
custom rigid-body sim with monocular rendering). The harness streams
synchronized (image, range, imu) tuples into ``src.main.run``.

This file is a placeholder so the experiment plan in the paper has a
concrete artifact to reference.
"""

from dataclasses import dataclass


@dataclass
class SimConfig:
    duration_s: float = 60.0
    image_hz: float = 30.0
    range_hz: float = 50.0
    imu_hz: float = 200.0
    seed: int = 0
    scenario: str = "warehouse_corridor"


def main(cfg: SimConfig) -> None:
    raise NotImplementedError


if __name__ == "__main__":
    main(SimConfig())
