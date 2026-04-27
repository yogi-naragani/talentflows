"""On-device SLM advisor (Gemini Nano-class).

Role: replace the if/else tree that would otherwise live in Python.
The advisor consumes a structured HealthSignals observation and emits
a structured ReasoningAction that:

  * picks a mission mode,
  * proposes a small waypoint delta,
  * caps speed,
  * scales MPC cost weights,
  * adjusts per-sensor trust,
  * may request a global replan.

It is *advisory*: a SafetySupervisor downstream clamps every field to
a verifiably safe envelope. Never let the LLM be the safety authority.

Substitution: Gemini Nano runs only on Pixel/Tensor. On Pi 5 + AI HAT
we use a Gemini Nano-class SLM (default: Gemma 3 1B Q4_K_M via
llama.cpp). The schema and prompt are identical across backends.
"""

from dataclasses import dataclass, field, asdict
from typing import Any
import json


VALID_MODES = (
    "nominal", "hover", "slow_advance", "retreat",
    "return", "land", "search_features",
)
VALID_YAW = ("hold", "track_features", "sweep_search")
VALID_SENSORS = ("slam", "range", "acoustic")
VALID_WEIGHTS = ("Q_pos", "Q_vel", "Q_att", "w_perception")


@dataclass
class ReasoningAction:
    mode: str = "nominal"
    waypoint_delta_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    speed_cap_mps: float = 5.0
    yaw_strategy: str = "hold"
    # Multiplicative scales applied to MPC base weights, in [0.1, 10].
    mpc_weights: dict | None = None
    # Per-modality trust in [0, 1]. None means "leave unchanged".
    sensor_trust: dict | None = None
    replan: bool = False
    rationale: str = ""           # short, for log only
    confidence: float = 0.5


SCHEMA = {
    "mode": list(VALID_MODES),
    "waypoint_delta_m": "[float, float, float]   # metres, |delta| <= 2",
    "speed_cap_mps": "float in [0, 5]",
    "yaw_strategy": list(VALID_YAW),
    "mpc_weights": "{Q_pos|Q_vel|Q_att|w_perception: float in [0.1, 10]} or null",
    "sensor_trust": "{slam|range|acoustic: float in [0, 1]} or null",
    "replan": "bool",
    "rationale": "string, <= 80 chars",
    "confidence": "float in [0, 1]",
}


SYSTEM_PROMPT = """You are an on-board flight reasoning module for a
GPS-denied micro-drone. You replace a hand-written rule tree.

You will receive a JSON OBSERVATION describing the drone's pose,
sensor health (SLAM, 1D rangefinder, acoustic backup), platform
state, and recent action history. You MUST reply with a single JSON
object matching the ACTION schema below. Output JSON only -- no prose,
no markdown, no commentary.

ACTION schema:
""" + json.dumps(SCHEMA, indent=2) + """

Decision guidance (not exhaustive -- use judgement):

* slam_tracking_ok=true and inlier_trend in {steady,rising}:
    nominal mode, default weights, full sensor trust.
* slam_inlier_trend=falling but tracking still ok:
    slow_advance, raise w_perception scale to ~2, lower speed_cap.
* slam_tracking_ok=false:
    if acoustic_confidence > 0.5, retreat opposite acoustic_worst_dir
    by a small delta; otherwise hover. Lower slam trust to ~0.2.
* acoustic_confidence > 0.5 AND any proximity > 0.7:
    retreat from worst_dir by 0.5-1.0 m, mode=retreat, low speed.
* range_valid_ratio_5s < 0.5:
    drop range trust to ~0.3; do not rely on it for clearance.
* battery_pct in (25, 35]: prefer mode=return.
* battery_pct in (12, 25]: mode=return, force speed_cap <= 3.
* If you have been in retreat or hover for > 8 s with no improvement:
    set replan=true and propose search_features with a sweep yaw.
* If everything looks fine, output the trivial nominal action.

Hard limits (the SafetySupervisor enforces these regardless, but try
to respect them so your decision is not overridden):
  - speed_cap_mps <= 5
  - |waypoint_delta_m| <= 2
  - z >= 0.5 m above current pos floor
  - mode must be one of: """ + ", ".join(VALID_MODES) + """

Always include a short rationale (<= 80 chars) explaining the choice;
it is for log audit only.
"""

# Few-shot examples teach format + behaviour. Keep tiny -- the
# observation/action pair is the unit of truth.
FEW_SHOT: list[tuple[dict, dict]] = [
    (
        {
            "slam_tracking_ok": True, "slam_inliers": 220,
            "slam_inlier_trend": "steady",
            "range_m": 1.4, "range_valid_ratio_5s": 1.0,
            "acoustic_confidence": 0.1,
            "battery_pct": 88, "speed_mps": 2.1,
            "distance_to_waypoint_m": 6.5,
        },
        {
            "mode": "nominal", "waypoint_delta_m": [0, 0, 0],
            "speed_cap_mps": 4.0, "yaw_strategy": "hold",
            "mpc_weights": None, "sensor_trust": None,
            "replan": False, "rationale": "all healthy",
            "confidence": 0.9,
        },
    ),
    (
        {
            "slam_tracking_ok": True, "slam_inliers": 60,
            "slam_inlier_trend": "falling",
            "range_m": 1.5, "range_valid_ratio_5s": 0.95,
            "acoustic_confidence": 0.2,
            "battery_pct": 72, "speed_mps": 3.2,
        },
        {
            "mode": "slow_advance", "waypoint_delta_m": [0, 0, 0],
            "speed_cap_mps": 1.5, "yaw_strategy": "track_features",
            "mpc_weights": {"w_perception": 2.0},
            "sensor_trust": None,
            "replan": False, "rationale": "slam degrading, slow + look",
            "confidence": 0.7,
        },
    ),
    (
        {
            "slam_tracking_ok": False, "slam_inliers": 0,
            "slam_inlier_trend": "falling",
            "slam_seconds_since_dropout": 0.4,
            "acoustic_confidence": 0.7,
            "acoustic_proximity": {"front": 0.8, "back": 0.1, "left": 0.2,
                                   "right": 0.2, "up": 0.1, "down": 0.4},
            "acoustic_worst_dir": "front",
            "battery_pct": 64,
        },
        {
            "mode": "retreat", "waypoint_delta_m": [-0.6, 0, 0],
            "speed_cap_mps": 0.8, "yaw_strategy": "hold",
            "mpc_weights": {"w_perception": 0.5, "Q_vel": 2.0},
            "sensor_trust": {"slam": 0.1, "acoustic": 0.9, "range": 0.5},
            "replan": False, "rationale": "slam lost, obstacle ahead, back off",
            "confidence": 0.85,
        },
    ),
    (
        {
            "slam_tracking_ok": True, "slam_inliers": 180,
            "battery_pct": 22, "distance_to_waypoint_m": 12.0,
        },
        {
            "mode": "return", "waypoint_delta_m": [0, 0, 0],
            "speed_cap_mps": 3.0, "yaw_strategy": "hold",
            "mpc_weights": None, "sensor_trust": None,
            "replan": True, "rationale": "battery low, return",
            "confidence": 0.95,
        },
    ),
]


class GeminiNanoClient:
    """Backend-agnostic SLM wrapper. The actual generator is plugged in
    at construction (AICore on Android, llama.cpp / Gemma 3 1B on Pi 5,
    or a stub for offline tests)."""

    def __init__(self, backend: Any | None = None,
                 max_tokens: int = 192,
                 include_few_shot: bool = True):
        self.backend = backend
        self.max_tokens = max_tokens
        self.include_few_shot = include_few_shot

    def decide(self, observation: dict) -> ReasoningAction:
        prompt = self._build_prompt(observation)
        raw = self._generate(prompt)
        return self._parse(raw)

    def _build_prompt(self, observation: dict) -> str:
        parts = [SYSTEM_PROMPT]
        if self.include_few_shot:
            for obs, act in FEW_SHOT:
                parts.append("OBSERVATION:\n" + json.dumps(obs))
                parts.append("ACTION:\n" + json.dumps(act))
        parts.append("OBSERVATION:\n" + json.dumps(observation))
        parts.append("ACTION:\n")
        return "\n".join(parts)

    def _generate(self, prompt: str) -> str:
        if self.backend is None:
            # Conservative offline default: hover.
            return json.dumps({
                "mode": "hover", "waypoint_delta_m": [0, 0, 0],
                "speed_cap_mps": 1.0, "yaw_strategy": "hold",
                "mpc_weights": None, "sensor_trust": None,
                "replan": False, "rationale": "no backend, conservative",
                "confidence": 0.3,
            })
        return self.backend.generate(prompt, max_tokens=self.max_tokens)

    @staticmethod
    def _parse(raw: str) -> ReasoningAction:
        # Tolerate stray prose: extract the first {...} block.
        s = raw.strip()
        i, j = s.find("{"), s.rfind("}")
        if i < 0 or j < 0 or j <= i:
            return ReasoningAction(mode="hover", rationale="parse_error_no_json")
        try:
            data = json.loads(s[i:j + 1])
        except ValueError:
            return ReasoningAction(mode="hover", rationale="parse_error_json")

        def _tup3(v, default):
            try:
                t = tuple(float(x) for x in v)
                return t if len(t) == 3 else default
            except (TypeError, ValueError):
                return default

        return ReasoningAction(
            mode=str(data.get("mode", "hover")),
            waypoint_delta_m=_tup3(data.get("waypoint_delta_m"),
                                   (0.0, 0.0, 0.0)),
            speed_cap_mps=float(data.get("speed_cap_mps", 2.0)),
            yaw_strategy=str(data.get("yaw_strategy", "hold")),
            mpc_weights=data.get("mpc_weights") or None,
            sensor_trust=data.get("sensor_trust") or None,
            replan=bool(data.get("replan", False)),
            rationale=str(data.get("rationale", ""))[:120],
            confidence=float(data.get("confidence", 0.5)),
        )
