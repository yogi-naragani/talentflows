"""Passive acoustic proximity sensor.

Backup obstacle-position source for GPS-denied flight when visual SLAM
degrades. The sensor is one or more onboard microphones that listen to
the drone's own ego-noise (propeller wash) reflected from nearby
surfaces. Short reflection delays and elevated band-limited reverberation
correlate with proximate obstacles.

This module is platform-agnostic: it consumes a multi-channel audio
buffer plus the current motor RPMs (for ego-noise reference) and
produces:

  * a coarse proximity-per-direction vector (front, back, left, right,
    up, down) in [0, 1] where 1 = imminent contact.
  * an estimated minimum clearance in metres along the most-occluded
    direction, when reflection time-of-flight is identifiable.

The processing is intentionally light so it can run on the same
Tensor-class SoC as Gemini Nano. The implementation here is a stub --
the paper's contribution is the role of this signal in the fusion +
MPC + reasoning loop, not a novel DSP pipeline.
"""

from dataclasses import dataclass
import numpy as np


DIRECTIONS = ("front", "back", "left", "right", "up", "down")


@dataclass
class AcousticReading:
    t_ns: int
    proximity: dict[str, float]      # 0 (far) .. 1 (imminent)
    clearance_m: dict[str, float]    # NaN where unobservable
    confidence: float                # 0 .. 1


class AcousticProximitySensor:
    def __init__(self,
                 sample_rate_hz: int = 48000,
                 mic_layout: str = "tetrahedral_4ch",
                 ego_band_hz: tuple[float, float] = (2000.0, 8000.0),
                 reflection_window_ms: float = 30.0):
        self.sample_rate_hz = sample_rate_hz
        self.mic_layout = mic_layout
        self.ego_band_hz = ego_band_hz
        self.reflection_window_ms = reflection_window_ms

    def process(self,
                t_ns: int,
                audio_chw: np.ndarray,           # (channels, samples)
                motor_rpm: np.ndarray) -> AcousticReading:
        """Estimate per-direction proximity from ego-noise reflections.

        Pipeline (to implement):
          1. Band-pass filter to the ego-noise band derived from RPM.
          2. Subtract a learned ego-reference (whitening) per channel.
          3. Cross-correlate residual across mic pairs to localize
             reflection bearing; integrate energy in a short window
             (reflection_window_ms) for proximity.
          4. Where TDOA + bearing are consistent across pairs, recover
             time-of-flight -> clearance estimate.
        """
        raise NotImplementedError

    @staticmethod
    def empty(t_ns: int) -> AcousticReading:
        return AcousticReading(
            t_ns=t_ns,
            proximity={d: 0.0 for d in DIRECTIONS},
            clearance_m={d: float("nan") for d in DIRECTIONS},
            confidence=0.0,
        )
