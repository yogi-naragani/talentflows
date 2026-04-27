# Research-paper gaps survey

Background literature scan to identify gaps the paper can plausibly
close. Caveats throughout: arXiv abstract pages refused some fetches,
so a few summaries rely on search snippets and need verification before
final citation.

## Top 5 gaps

### Gap A: Perception-aware MPC with explicit SLAM-degradation fallback driven by an on-device LLM
PAMPC, PA-MPPI, and FoV-constrained MPC optimize visibility, but assume
the SLAM front-end keeps working. None co-design with a high-level
supervisor that *recognizes* degradation and changes mission intent.

- Falanga et al., "PAMPC: Perception-Aware MPC for Quadrotors,"
  IROS 2018, https://arxiv.org/abs/1804.04811
- "PA-MPPI: Perception-Aware Model Predictive Path Integral Control
  for Quadrotor Navigation in Unknown Environments," arXiv 2509.14978,
  2025

How our stack closes it: MPC's perception cost is reweighted at runtime
by Gemini Nano advisories triggered by SLAM-health metrics.

### Gap B: 1D rangefinder as a graceful-degradation fallback
1D rangefinders are widely used for monocular metric scale, but rarely
as a clearance-keeping primary altitude / proximity source when
tracking is fully lost.

- Zhang et al., "Scale Estimation and Correction of Monocular SLAM
  Based on Fusion of 1D Laser Range Finder and Vision," Sensors 18(6),
  2018, https://www.mdpi.com/1424-8220/18/6/1948
- "Visual-LiDAR Odometry with Monocular Scale Correction and Visual
  Bootstrapping," arXiv 2304.08978

How our stack closes it: dual role — scale during nominal flight, hard
clearance constraint in the MPC when vision fails.

### Gap C: Passive acoustic ego-noise reflectometry as an onboard SLAM backup on a flying micro-drone
Acoustic / ego-noise reflectometry is demonstrated for stationary
robots and for *external* drone localization, but not as an onboard
SLAM-backup proximity cue on a flying micro-drone.

- Fontaine et al., "Detecting acoustic reflectors using a robot's
  ego-noise," arXiv 2111.08327 (stationary listening robot)
- Müller, Kartsch et al., "BatDeck: Ultra Low-power Ultrasonic
  Ego-velocity Estimation and Obstacle Avoidance on Nano-drones,"
  arXiv 2412.10048 (uses *active* emitted ultrasound, not propeller
  ego-noise)

How our stack closes it: passive microphones using reflected propeller
noise as a soft proximity / clearance source that engages exactly when
vision degrades.

Honest caveat: BatDeck is the most direct competitor and achieves the
same outcome cheaper with a 30 mW transducer. Our passive angle (no
emission, lower acoustic detectability, no payload, multi-role mic)
must be justified explicitly. Ego-noise SNR at the mic is brutal
(~ -20 dB), so claims should stay at coarse proximity, not metric
ranging.

### Gap D: On-device small LLMs as real-time advisory (not command-translation) layers
Most SLM-on-drone work uses the model as a natural-language command
parser or task planner, not as a real-time advisor reasoning over
SLAM / IMU / acoustic health to recommend mission adaptation.

- "Taking Flight with Dialogue: Natural Language Control for PX4
  Drone Agent," arXiv 2506.07509 (command parsing)
- "LLM-Land: LLMs for Context-Aware Drone Landing," arXiv 2505.06399
  (task-level, not health-aware)

How our stack closes it: Gemini Nano runs at 0.5–2 Hz consuming
structured telemetry summaries and emitting MPC-cost or behavior
changes, not low-level commands.

Honest caveat: Gemini Nano (the Android/Pixel SLM) has no peer-
reviewed robotics deployment we could find; Google's "Gemini Robotics
On-Device" is a separate model. The paper must be precise about which
artifact is actually being used.

### Gap E: GPS-denied benchmarks that combine VSLAM + perception-aware control + LLM supervision + acoustic backup in ROS 2 / Gazebo
The 2025 Jarraya et al. UAV-localization survey calls hybrid
multi-sensor fusion the open frontier. Existing comparative work is
mostly trajectory-error benchmarking on EuRoC.

How our stack closes it: a reproducible ROS 2 + Gazebo testbed with
degradation injection (texture-poor walls, illumination drops, IMU
bias) is itself a contribution.

## Key references

1. Falanga et al., PAMPC, IROS 2018 — https://arxiv.org/abs/1804.04811
2. PA-MPPI, arXiv 2509.14978, 2025
3. Perception-aware planning, feature-limited environments,
   arXiv 2503.15273, 2025
4. Campos et al., ORB-SLAM3, IEEE T-RO 2021 —
   https://arxiv.org/abs/2007.11898
5. Qin et al., VINS-Fusion (extension of VINS-Mono), HKUST
6. Merzlyakov & Macenski, "A Comparison of Modern General-Purpose
   Visual SLAM Approaches," arXiv 2107.07589
7. Schmidt et al., Visual-Inertial SLAM loop-closure benchmark,
   J. Field Robotics 2025
8. Zhang et al., 1D laser + monocular scale, Sensors 2018
9. Visual-LiDAR monocular scale correction, arXiv 2304.08978
10. BatDeck, arXiv 2412.10048
11. Fontaine et al., ego-noise reflector detection, arXiv 2111.08327
12. Rotor-conditioned drone audition, EURASIP JASMP 2025
13. Jarraya et al., UAV GPS-denied review, Satellite Navigation 2025
14. Gemini Robotics On-Device announcement, Google DeepMind 2025
15. PX4 + ROS 2 + Ollama drone agent, arXiv 2506.07509

## VINS-Fusion vs ORB-SLAM3 — verdict

ORB-SLAM3 (mono-inertial mode). For a ROS 2 + Gazebo monocular drone
that needs metric scale and graceful degradation, ORB-SLAM3's Atlas
multi-map plus relocalization gives the cleanest "SLAM degraded ->
SLAM recovered" event boundary that the LLM advisory layer can hook
into. VINS-Fusion is a strong VIO but lacks ORB-SLAM3's loop-closure +
multi-map machinery and tends to underperform it on monocular-inertial
EuRoC accuracy in recent comparisons. Mature ROS 2 wrappers
(`Mechazo11/ros2_orb_slam3`, others) lower 2026 engineering risk.

## Skepticism

- "No one has done X" claims for Gaps A, C, D are based on
  title/abstract-level hits — treat as "likely novel," not "provably
  novel." Workshop papers and non-English venues may be missed.
- Jarraya et al. 2025 was reached via an EurekAlert press release —
  re-verify the primary PDF before citation.
- arXiv abstract pages returned 403 to WebFetch on several queries —
  PA-MPPI, BatDeck v2, and feature-limited planning rely on
  search-snippet text. Verify before final write-up.
- BatDeck is the closest competitor for the acoustic component; if
  reviewers see ours as "BatDeck without the transducer," we need a
  sharp argument for the passive angle.
- Gemini Nano vs. "Gemini Robotics On-Device" must not be conflated
  in the final paper.
