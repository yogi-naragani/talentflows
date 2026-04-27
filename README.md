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
camera / IMU / 1D-range / motor topics, and the five nodes
(`perception_node`, `range_node`, `fusion_node`, `mpc_node`,
`reasoning_node`).

## Status

Scaffold. The SLAM, MPC, and Gemini Nano integrations are stubbed. The
warehouse world references no drone model yet -- plug in PX4 SITL's
iris or the gz-sim X3 example.
