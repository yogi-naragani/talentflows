# GPS-Denied Drone Navigation with Gemini Nano + MPC + Visual SLAM + 1D Range Sensor

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
