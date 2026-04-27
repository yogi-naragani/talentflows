# GPS-Denied Drone Navigation with Gemini Nano + MPC + Visual SLAM + 1D Range Sensor

Research repository for a paper on resilient drone navigation in GPS-denied
environments. The system fuses four components:

1. **Visual SLAM** for 6-DoF pose and a sparse map from a forward camera.
2. **1D range sensor** (single-beam LiDAR / ToF / radar altimeter) for
   metric scale recovery and ground-clearance / obstacle gating.
3. **Model Predictive Control (MPC)** for real-time trajectory tracking
   under dynamic and actuator constraints.
4. **Gemini Nano** as an on-device reasoning layer for high-level mission
   adaptation, semantic anchoring, and recovery behaviors when SLAM
   degrades.

## Paper goal

Demonstrate that a small on-device LLM (Gemini Nano) can supervise a
classical perception + control stack to keep a drone localized and on
mission when GPS is unavailable, using only a monocular camera and a
single 1D range return.

## Repository layout

```
paper/                LaTeX sources, sections, figures, bibliography
src/
  perception/         Visual SLAM front/back-end interfaces
  sensors/            1D range sensor driver and outlier rejection
  fusion/             Scale recovery + state estimation
  control/            MPC formulation and solver wrapper
  reasoning/          Gemini Nano on-device prompting and policies
  main.py             End-to-end loop (sim or hardware)
experiments/          Datasets, simulation runners, ablation configs
requirements.txt
```

## Status

Scaffold only. Component interfaces are stubbed; experiments and paper
sections are placeholders to be filled in.
