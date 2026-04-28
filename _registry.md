# Component registry

Status of every major component in the GPS-denied drone portfolio
project. Update after each working session.

Status legend:
- `done`     ready, tested, documented
- `wip`      partial, see notes
- `todo`     not started
- `n/a`      out of scope for this sprint

## Sim setup (Day 1-2)

| Item | Status | Notes |
|---|---|---|
| Ubuntu 22.04 / WSL2 / native | todo | user-side, see `docs/sprint_2week.md` |
| ROS 2 Humble | todo | user-side; Dockerfile available as fallback |
| Gazebo Harmonic | todo | user-side; Dockerfile available as fallback |
| PX4 SITL build | todo | user-side; gz_x500 target |
| `Dockerfile` | done | reproducible env baseline |
| ROS 2 ament_python package skeleton | done | `package.xml`, `setup.py`, `setup.cfg`, `resource/` |

## ROS 2 - PX4 bridge (Day 2-3)

| Item | Status | Notes |
|---|---|---|
| `px4_msgs` install | todo | user-side |
| XRCE-DDS agent | todo | user-side |
| `px4_bridge_node` (TrajectorySetpoint, OffboardControlMode) | wip | scaffold in `gps_denied_drone/nodes/px4_bridge.py` |
| Square-flight demo script | todo | `scripts/demo_square_flight.py` (TODO) |

## VIO baseline (Day 4-5)

| Item | Status | Notes |
|---|---|---|
| ORB-SLAM3 build (C++) | todo | user-side; `contracts/vio_pipeline.md` has the recipe |
| EuRoC MAV download | wip | `experiments/euroc/run_eval.py` script ready |
| ATE / RPE via `evo` | wip | same script |
| `results/vio_baseline.md` | todo | populated after first eval |

## Custom world + sensors (Day 6)

| Item | Status | Notes |
|---|---|---|
| `worlds/warehouse.sdf` | done | corridor + walls + X3 + camera + IMU + 1D range |
| Forward camera 30 Hz | done | in SDF |
| Downward 1D range (gpu_lidar 1x1) | done | in SDF |
| IMU 200 Hz | done | in SDF |
| GPS disabled in PX4 airframe | todo | user-side, EKF2 config |

## VIO in the loop (Day 7)

| Item | Status | Notes |
|---|---|---|
| Camera+IMU -> ORB-SLAM3 input | wip | `gps_denied_drone/nodes/perception_node.py` runs `cv2.ORB` proxy; ORB-SLAM3 wrapper is the upgrade path |
| Pose -> `/vehicle_visual_odometry` | todo | hook in `px4_bridge.py` |
| EKF2 external-vision config | todo | user-side |
| 30 s autonomous flight | todo | acceptance criterion |

## Learned component (Day 8-9, choose ONE)

| Item | Status | Notes |
|---|---|---|
| Option A: depth estimation (MiDaS / Depth Anything fine-tune) | todo | scaffold in `learning/depth/` (TODO) |
| Option B: PPO collision policy on Isaac Lab | n/a | unless A blocks |
| Brev cloud GPU | todo | user-side; spot instances |
| ONNX export + ROS 2 inference node | todo | only after training |

## Evaluation (continuous)

| Item | Status | Notes |
|---|---|---|
| `evo` integration | wip | `experiments/euroc/run_eval.py` |
| Trajectory plots | done | `experiments/sim/visualize.py` (Python harness only so far) |
| Paired stats | done | `experiments/sim/paired_stats.py` |
| Reproducible Makefile | done | `make headline` end-to-end |

## Paper + release (Day 10)

| Item | Status | Notes |
|---|---|---|
| LaTeX skeleton | done | `paper/main.tex` (currently steered toward an acoustic-backup paper, see "carryover" below) |
| Demo video | todo | 60 s screen capture |
| arXiv submission | todo | cs.RO category |
| README polish | wip | install + run instructions present, GIF demo missing |

## Carryover from earlier sessions (not on the critical path)

The repo carries earlier work from a different research direction
(passive acoustic backup + on-device SLM). It lives on two paper
branches and on the integration branch's simulator. None of it is
required for the portfolio submission, but it does not block either:

| Item | Status | Notes |
|---|---|---|
| Python-only sim with acoustic backup | done | `experiments/sim/` |
| Acoustic ablation result (40 trials, p < 1e-6) | done | not used in the portfolio paper |
| Two paper branches | done | optional follow-on / stretch |

The portfolio sprint focuses on PX4 + ORB-SLAM3 + EuRoC + a learned
component. Acoustic + SLM are decoupled and can be cherry-picked into
the final paper if there is room.
