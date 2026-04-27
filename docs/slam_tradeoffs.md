# VINS-Fusion vs ORB-SLAM3: Design tradeoffs for this paper

Goal: pick the SLAM backend for a GPS-denied micro-drone running
ROS 2 + Gazebo, with monocular forward camera + IMU + 1D rangefinder +
acoustic backup, supervised by an on-device Gemini Nano advisor.

## Quick comparison

| Axis | VINS-Fusion (HKUST) | ORB-SLAM3 (UZ) |
|---|---|---|
| Front-end | Optical-flow tracking on Shi-Tomasi corners | ORB feature extraction + descriptor matching |
| Back-end | Tightly-coupled sliding-window VIO + 4-DoF pose-graph | Local BA + multi-map (Atlas) + full BA on loop closure |
| Loop closure | DBoW2, optional | DBoW2, integrated; supports map merging across sessions |
| Camera modes | mono / stereo / mono+IMU / stereo+IMU | mono / stereo / RGB-D / mono+IMU / stereo+IMU |
| Map output | Sparse, optimized over sliding window | Sparse keyframe map, persists across runs (Atlas) |
| ROS support | First-class ROS 1; ROS 2 ports community-maintained | Community ROS 2 wrappers; not first-class upstream |
| License | GPLv3 | GPLv3 |
| Typical CPU footprint on mid-tier ARM | Lower; lighter front-end | Higher at startup; competitive in steady state |
| Robustness to low-texture / fast motion | Good optical-flow tracking; degrades gracefully | ORB descriptors brittle in low-texture; relocalization is strong |
| Scale (monocular) | Scale recovered via IMU pre-integration | Scale recovered via IMU; Atlas helps after dropouts |
| Re-localization after dropout | Limited (sliding window) | Strong: Atlas keeps prior maps and merges on re-entry |

## What our use case demands

1. **Monocular + IMU + occasional dropout** — we want graceful recovery
   when textures vanish (corner of an indoor scene, white wall).
2. **Persistent map across a 60 s mission** — the reasoner benefits
   from a stable map identifier so its advice is consistent.
3. **Reasonable CPU on a Tensor-class SoC** — Gemini Nano shares the
   compute budget. Sub-30% CPU on one big core is the target.
4. **Fast re-localization after acoustic-backup retreat** — when the
   drone hovers / backs out due to acoustic proximity warnings, we need
   the SLAM to relocalize into the prior map cleanly.
5. **ROS 2 + Gazebo Harmonic integration** — minimum integration pain.

## Verdict

**ORB-SLAM3 is the better default for this paper.** The deciding
factors are (a) the Atlas multi-map system, which handles the
SLAM-degradation -> acoustic-backup retreat -> SLAM recovery cycle
that is central to our experiments, and (b) stronger relocalization,
which makes the ablation cleaner: we can deliberately mask textures
mid-flight and measure the recovery time, isolating the acoustic
contribution.

VINS-Fusion is the better fallback if (a) we hit hard CPU budget
limits on the target SoC, or (b) we need first-class ROS 2 support
without third-party wrappers and the project lead is uncomfortable
with community ports.

## What we will report in the paper

* Both backends in the ablation table (one column each, holding the
  rest of the stack fixed) so the contribution of *our* components
  (1D-range scale recovery, acoustic backup, perception-aware MPC,
  Gemini Nano supervisor) is visible independent of the SLAM choice.
* Wall-clock relocalization-after-mask metric, since it is the most
  direct measure of the SLAM/acoustic interaction.

## Citations

- Campos et al., *ORB-SLAM3: An Accurate Open-Source Library for
  Visual, Visual-Inertial, and Multi-Map SLAM*, IEEE T-RO, 2021.
  https://arxiv.org/abs/2007.11898
- Qin et al., *VINS-Mono: A Robust and Versatile Monocular
  Visual-Inertial State Estimator*, IEEE T-RO, 2018.
- Qin & Shen, *VINS-Fusion* (extended multi-sensor branch), HKUST
  Aerial Robotics Group, code release.
- Merzlyakov & Macenski, *A Comparison of Modern General-Purpose
  Visual SLAM Approaches*, arXiv:2107.07589, 2021.
- Schmidt et al., *Visual-Inertial SLAM for Unstructured Outdoor
  Environments: Benchmarking Loop Closing*, J. Field Robotics, 2025.
- Mur-Artal & Tardós, *ORB-SLAM2*, IEEE T-RO, 2017 (Atlas evolution
  context).
- ROS 2 wrapper: `Mechazo11/ros2_orb_slam3` (for engineering risk
  estimate).
