"""Bench script: characterize the SNR floor of the passive acoustic
backup. Sweeps reflector distance and rotor RPM, computes the
ego-noise-to-reflection ratio at the mic. The output figure is
expected to appear in the paper as evidence that the passive signal
is recoverable above SNR threshold X for distances < Y metres.

This is a bench harness, not a flight test. Runs offline against
either recorded audio (preferred) or a synthetic reflection model
(useful for stub/CI).
"""

from dataclasses import dataclass


@dataclass
class SnrSweepConfig:
    rotor_rpms: tuple = (3000, 5000, 7000, 9000)
    distances_m: tuple = (0.3, 0.6, 1.0, 1.5, 2.0, 3.0)
    band_hz: tuple = (2000.0, 8000.0)
    reflector: str = "flat_plywood_1m2"
    mic_layout: str = "tetrahedral_4ch"


def main(cfg: SnrSweepConfig) -> None:
    raise NotImplementedError("Wire to either wav files or a synth model.")


if __name__ == "__main__":
    main(SnrSweepConfig())
