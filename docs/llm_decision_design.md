# SLM-driven monitoring and decision design

The point of putting a small language model on the drone is to *replace
the if/else tree* that grows whenever a new sensor or failure mode is
added. A rule tree is fast and verifiable but brittle and tedious; an
SLM is contextual and concise but unreliable in absolute terms. This
design splits responsibilities so we get both.

## Three layers

```
   Sensors --> [Monitor] --> observation -+
                                          |
                                          v
                                       [SLM advisor]
                                          |
                                          v
                                       proposed action
                                          |
                                          v
                              [SafetySupervisor]
                                          |
                                          v
                                  safe action --> MPC + waypoint pub
```

| Layer | Responsibility | Where it lives |
|---|---|---|
| Monitor | Aggregate raw signals into a *structured observation*. **No decisions.** Computes trends, ratios, time-since-last-X. | `gps_denied_drone/reasoning/monitor.py` |
| SLM advisor | Read the observation, output one structured action. **Replaces if/else.** | `gps_denied_drone/reasoning/gemini_nano.py` |
| SafetySupervisor | Clamp every field of the action to a verified safe envelope. **Authoritative on safety.** | `gps_denied_drone/reasoning/safety.py` |

The MPC stays classical and predictable. The SLM cannot move the drone
into the floor or above the speed cap because the supervisor rewrites
its action before publication.

## Observation (input to the SLM)

Built once per tick by `HealthMonitor.observe(...)`. JSON-serializable.
Compact -- we drop None / inf so the model isn't confused. Includes:

- pose, velocity, yaw, speed
- SLAM tracking flag, inlier count, inlier *trend* (rising / steady /
  falling), seconds since last dropout
- 1D rangefinder: latest value, valid-fraction over last 5 s
- acoustic: per-direction proximity, worst direction, clearance,
  confidence
- battery percent, IMU vibration, motor saturation ratio
- mission: current waypoint, distance to it, seconds in current mode,
  recent mode history

Trends matter more than instantaneous readings -- they let the SLM
distinguish "currently fine but degrading" from "stable noise."

## Action (output from the SLM)

`ReasoningAction` schema. JSON-only response, no prose:

```json
{
  "mode": "nominal | hover | slow_advance | retreat | return | land | search_features",
  "waypoint_delta_m": [dx, dy, dz],
  "speed_cap_mps": 0..5,
  "yaw_strategy": "hold | track_features | sweep_search",
  "mpc_weights": {"Q_pos": 0.1..10, "Q_vel": ..., "w_perception": ...} | null,
  "sensor_trust": {"slam": 0..1, "range": 0..1, "acoustic": 0..1} | null,
  "replan": false,
  "rationale": "<= 80 chars",
  "confidence": 0..1
}
```

Why this shape:

- **mode** is the coarsest knob -- modes map to MPC reference
  generators downstream.
- **waypoint_delta_m** lets the SLM nudge the immediate goal without
  redoing global planning.
- **mpc_weights** lets the SLM *retune* the controller (e.g. raise
  `w_perception` when SLAM is fragile, raise `Q_vel` when battery is
  low and we want smoother motion). Multiplicative scales bound to
  `[0.1, 10]`.
- **sensor_trust** lets the SLM downweight a modality the fusion
  layer should trust less right now (e.g. drop range trust when the
  beam keeps hitting clutter).
- **replan** is the escape hatch: ask the global planner for a fresh
  plan rather than nudging the current one.
- **rationale** is *log-only*, kept short to keep tokens bounded.

## Prompt design

System prompt: role, schema, decision guidance, hard limits.
Few-shot: 4 minimal observation/action pairs covering nominal,
slow-advance, retreat, return.
User: the current observation. The model emits a single JSON object.

Total prompt: well under 1k tokens. Output: <= 192 tokens. On Pi 5 +
Gemma 3 1B Q4_K_M via llama.cpp, one tick lands in roughly 0.3-0.6 s,
fitting the 0.5-2 Hz advisory rate with margin.

## Safety supervisor

Every field is clamped:

- `mode` must be in the whitelist (else -> `hover`).
- `speed_cap_mps` clamped to `[0, MAX_SPEED_MPS]`.
- `waypoint_delta_m` norm clamped to `MAX_WP_DELTA_M`.
- target altitude clamped to `>= MIN_ALT_M`.
- target position clamped to a geofence box.
- `mpc_weights` and `sensor_trust` clamped to their valid ranges.

Battery overrides the LLM:

- `battery_pct <= 25` forces `mode=return`
- `battery_pct <= 12` forces `mode=land` regardless of LLM output

When the supervisor changes anything it records the reason; the node
publishes `/mission/safety_clamped=true` and logs the reasons.

## What the SLM is good at, and what it isn't

Good at:

- Combining many weakly-correlated signals into one decision
  ("SLAM falling + acoustic cone in front + going forward fast" ->
  retreat) without us writing the conjunction.
- Adjusting numeric knobs (weights, trusts) by a small factor
  in a context-dependent way.
- Generalizing across scenarios we didn't anticipate.

Not good at:

- Hard real-time control (it ticks at 1 Hz, not 100 Hz).
- Numerical precision (treat its `waypoint_delta_m` as a hint).
- Safety guarantees (it will sometimes hallucinate; the supervisor
  is the safety net).

## Telemetry contract for offline analysis

Every tick the node logs the (observation, action, safe_action,
clamp_reasons, latency_ms) tuple to bag. The paper's evaluation uses
this to compute:

- Action quality vs. ground-truth optimal in scripted scenarios.
- Clamp frequency (lower is better -- the SLM is staying inside the
  envelope on its own).
- Tick latency under contention with SLAM and MPC.
- Behavior diversity (entropy over modes) by scenario.

## Why not just keep the if/else?

We tried. The matrix grows quadratically with sensors and failure
modes: 3 SLAM states x 2 range states x 4 acoustic states x 4 battery
bands x 3 mission phases is 288 cells, and adding "IMU vibration high"
doubles it. The rule tree becomes hard to test exhaustively. The SLM
collapses the matrix into a contextual judgment, and the safety
supervisor keeps it honest.
