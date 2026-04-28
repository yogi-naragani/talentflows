# GPS-Denied Drone Navigation Portfolio Project

> 2-week sprint variant of the canonical 8-week plan. Project goal,
> tool routing, and tech stack live in [`CLAUDE.md`](CLAUDE.md);
> day-by-day execution in [`docs/sprint_2week.md`](docs/sprint_2week.md);
> per-component status in [`_registry.md`](_registry.md). Per-component
> contracts in [`contracts/`](contracts/).

## At a glance

| Layer | Tool | Status |
|---|---|---|
| OS / middleware | Ubuntu 22.04 + ROS 2 Humble | user-side (`Dockerfile` available) |
| Simulator | Gazebo Harmonic + PX4 SITL (`gz_x500`) | user-side; world ready |
| VIO | ORB-SLAM3 (mono-inertial); cv2.ORB proxy as fallback | proxy done; ORB-SLAM3 build user-side |
| Trajectory eval | `evo` (Python) | script in `experiments/euroc/run_eval.py` |
| Learned component | depth (default) or RL policy on Brev | scaffold pending |
| Custom world | corridor + walls + X3 + camera + IMU + 1D range | done in `worlds/warehouse.sdf` |

## What this repository carries today

- A Python-only sim harness (`experiments/sim/`) that runs the
  Monitor + advisor + SafetySupervisor + LQR pipeline end to end with
  synthetic dynamics. Useful as a 7-test smoke harness for the
  controller and reasoning layers.
- A ROS 2 ament_python package (`gps_denied_drone/`) with six nodes
  (perception, range, fusion, MPC, reasoning, acoustic) plus a
  PX4 SITL bridge node (`px4_bridge`) for the mission loop.
- A complete Gazebo world (`worlds/warehouse.sdf`) with X3 quadrotor
  + camera + IMU + 1D range, plus the launch file that bridges
  everything to ROS 2.
- An EuRoC + `evo` evaluation script (`experiments/euroc/run_eval.py`)
  with two backends: a real-ORB-SLAM3 shell-out and a cv2.ORB
  feature-density proxy for Day-4 smoke-testing before ORB-SLAM3 is
  built.
- LaTeX paper skeleton (`paper/`).

## Original project carryover (acoustic backup, on-device SLM)

Earlier sessions explored a different research direction (passive
acoustic ego-noise reflectometry + Gemini Nano-class on-device SLM
supervisor). That work lives on its own paper branches
(`claude/paper-acoustic-backup-VgEQR`, `claude/paper-slm-advisor-VgEQR`)
and is decoupled from the portfolio sprint. The Python sim, the
acoustic backup, and the trajectory-comparison figure remain
runnable from the integration branch via `make headline` if you
want to see them.

Research repository for a paper on resilient drone navigation in
GPS-denied environments. The system fuses five components:

1. **Visual SLAM** for 6-DoF pose and a sparse map from a forward camera.
2. **1D range sensor** (single-beam LiDAR / ToF / radar altimeter) for
   metric scale recovery and ground-clearance / obstacle gating.
3. **Model Predictive Control (MPC)** for real-time trajectory tracking
   under dynamic and actuator constraints, with hard clearance
   constraints from the acoustic backup.
4. **Gemini Nano** as an on-device reasoning layer for high-level
   mission adaptation and recovery behaviors when SLAM degrades.
5. **Acoustic proximity sensor** (passive microphone array, ego-noise
   reflections) as a backup obstacle-position source so the drone can
   still avoid collisions when SLAM tracking is lost.

The repo is a **ROS 2 ament_python package** with a **Gazebo (gz-sim)**
simulation harness.

## Reproduce the headline result

One command, on a laptop, no Gazebo or ROS required:

```
make headline
```

Outputs the 40-trial ablation CSV, the trajectory comparison PNG,
and the paired Wilcoxon p-values. See `docs/quickstart.md` for a
walkthrough and what numbers to expect.

## Tested with

- ROS 2 Humble or Jazzy
- Gazebo Harmonic (`gz-sim 8`) via `ros_gz_bridge` / `ros_gz_sim`
- Python 3.10+

## Hardware target

Onboard compute: **Raspberry Pi 5 (8 GB / 16 GB) + Raspberry Pi AI HAT+
(Hailo-8 or Hailo-8L)**. Power budget ~15–18 W for the full compute
stack; ~10% of the motor power on a 500 g class drone. See
`docs/hardware_target.md` for the per-component breakdown,
CPU/NPU split, and an honest note on the Gemini Nano substitution
(Gemini Nano runs only on Pixel/Tensor; on Pi 5 we use a Gemini
Nano-class SLM such as Gemma 3 1B via llama.cpp).

## Layout

```
package.xml                ROS 2 package manifest
setup.py / setup.cfg       ament_python build
resource/gps_denied_drone  ament marker
gps_denied_drone/
  perception/              VSLAM facade
  sensors/                 1D rangefinder + acoustic proximity sensor
  fusion/                  scale recovery + state estimator
  control/                 perception-aware MPC + acoustic constraints
  reasoning/               Gemini Nano on-device client
  nodes/                   ROS 2 node wrappers (one per module)
launch/sim.launch.py       brings up gz-sim + bridge + 5 nodes
config/                    MPC + bridge YAML
worlds/warehouse.sdf       placeholder gz-sim world
experiments/               ablation matrix + analysis scripts
paper/                     IEEEtran LaTeX skeleton
```

## Build

```
cd ~/ros2_ws
ln -s /path/to/talentflows src/gps_denied_drone
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select gps_denied_drone
source install/setup.bash
```

## Run the simulation

```
ros2 launch gps_denied_drone sim.launch.py
```

This starts gz-sim with `worlds/warehouse.sdf`, the `ros_gz_bridge` for
camera / IMU / 1D-range / audio / motor topics, and the six nodes
(`perception_node`, `range_node`, `fusion_node`, `mpc_node`,
`reasoning_node`, `acoustic_node`).

## Deploy to Pi 5 + AI HAT (hardware-in-the-loop or flight)

```
ros2 launch gps_denied_drone deploy_pi5.launch.py \
     use_hailo:=true slm_backend:=llama_cpp_gemma3_1b
```

This brings up the six nodes only (no gz-sim — Gazebo is too heavy for
the Pi 5). Either let the topics come from real drivers, or run gz-sim
on a workstation on the same network for HIL.

## Status

Scaffold. The SLAM, MPC, and Gemini Nano integrations are stubbed. The
warehouse world references no drone model yet -- plug in PX4 SITL's
iris or the gz-sim X3 example.
