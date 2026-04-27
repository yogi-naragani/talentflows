"""1D range sensor driver and outlier rejection.

A single-beam time-of-flight or radar altimeter pointing along a known
body-frame axis (typically -z, i.e. downward). Used for:
  * metric scale recovery for monocular VSLAM
  * ground-clearance safety
  * coarse obstacle gating along the beam axis
"""

from dataclasses import dataclass
from collections import deque
import numpy as np


@dataclass
class RangeReading:
    t_ns: int
    range_m: float
    valid: bool


class RangeSensor:
    def __init__(self, max_range_m: float = 40.0, window: int = 9):
        self.max_range_m = max_range_m
        self._window = deque(maxlen=window)

    def push(self, reading: RangeReading) -> RangeReading:
        if not reading.valid or reading.range_m <= 0 or reading.range_m > self.max_range_m:
            return RangeReading(reading.t_ns, reading.range_m, valid=False)
        self._window.append(reading.range_m)
        med = float(np.median(self._window))
        # Reject points that disagree with the local median by > 50%.
        if abs(reading.range_m - med) > 0.5 * med:
            return RangeReading(reading.t_ns, reading.range_m, valid=False)
        return reading

    def latest_filtered(self) -> float | None:
        if not self._window:
            return None
        return float(np.median(self._window))
