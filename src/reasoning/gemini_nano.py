"""On-device reasoning layer backed by Gemini Nano.

The reasoning layer runs at a low rate (1-5 Hz) and is *advisory*: it
proposes mission-level adjustments that the MPC layer is free to honor,
soften, or ignore based on safety constraints. This decoupling keeps the
hard real-time path classical and verifiable.

Contract:
  * Inputs:
      - compact JSON-serializable state summary (pose, vel, battery,
        SLAM health, recent range readings, current waypoint)
      - optional thumbnail / scene caption
  * Output:
      - a small action schema: {mode, waypoint_delta, speed_cap, notes}

Gemini Nano is targeted because it runs on-device (Pixel / Tensor) with
sub-second latency and no network dependency, which fits the GPS-denied
operational assumption (no comms guaranteed either).
"""

from dataclasses import dataclass, asdict
import json
from typing import Any


@dataclass
class StateSummary:
    pos_xyz_m: tuple[float, float, float]
    vel_xyz_mps: tuple[float, float, float]
    yaw_deg: float
    battery_pct: float
    slam_tracking_ok: bool
    slam_inliers: int
    range_m: float | None
    waypoint_xyz_m: tuple[float, float, float]
    notes: str = ""


@dataclass
class ReasoningAction:
    mode: str                                    # "nominal" | "hover" | "return" | "explore"
    waypoint_delta_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    speed_cap_mps: float = 5.0
    notes: str = ""


SYSTEM_PROMPT = """You are an on-board flight reasoning module for a
GPS-denied micro-drone. You receive a compact JSON state. You must reply
with a single JSON object matching the ReasoningAction schema. Be
conservative: prefer hover or slow exploration when SLAM tracking is
weak. Never propose altitude below 0.5 m or speed above 5 m/s.
"""


class GeminiNanoClient:
    """Thin wrapper around the on-device Gemini Nano runtime.

    The exact binding (AICore on Android, MediaPipe LLM Inference, etc.)
    is left abstract so the paper experiments can swap backends.
    """

    def __init__(self, backend: Any | None = None, max_tokens: int = 128):
        self.backend = backend
        self.max_tokens = max_tokens

    def decide(self, summary: StateSummary) -> ReasoningAction:
        prompt = SYSTEM_PROMPT + "\nSTATE:\n" + json.dumps(asdict(summary))
        raw = self._generate(prompt)
        return self._parse(raw)

    def _generate(self, prompt: str) -> str:
        if self.backend is None:
            # Offline fallback so the rest of the stack runs without a
            # Gemini Nano runtime present.
            return json.dumps(asdict(ReasoningAction(mode="nominal")))
        return self.backend.generate(prompt, max_tokens=self.max_tokens)

    @staticmethod
    def _parse(raw: str) -> ReasoningAction:
        try:
            data = json.loads(raw)
            return ReasoningAction(
                mode=data.get("mode", "hover"),
                waypoint_delta_m=tuple(data.get("waypoint_delta_m", (0.0, 0.0, 0.0))),
                speed_cap_mps=float(data.get("speed_cap_mps", 2.0)),
                notes=str(data.get("notes", "")),
            )
        except (ValueError, TypeError):
            return ReasoningAction(mode="hover", notes="parse_error")
