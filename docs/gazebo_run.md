# Running on Gazebo Harmonic + ROS 2

The Python harness (`make headline`) is the recommended path for
paper-grade ablations -- it is fast, deterministic, and isolates
the advisor + safety + acoustic-clearance contribution. The Gazebo
build is the higher-fidelity validation: it exercises real image
perception, real rigid-body dynamics, and per-scenario variance
that the synthetic simulator does not capture.

## What the Gazebo path runs

```
            +---------------+        +---------------+
 gz-sim --->| ros_gz_bridge |---ROS--| six ROS 2     |
            +---------------+        | nodes         |
                                     +---------------+
```

Bridged topics (see `launch/sim.launch.py`):

| ROS 2 topic       | gz-sim topic     | ROS type                          |
|-------------------|------------------|-----------------------------------|
| `/clock`          | `/clock`         | `rosgraph_msgs/msg/Clock`         |
| `/camera/image_raw` | `/camera/image_raw` | `sensor_msgs/msg/Image`     |
| `/imu`            | `/imu`           | `sensor_msgs/msg/Imu`             |
| `/range/raw`      | `/range/raw`     | `sensor_msgs/msg/Range`           |
| `/cmd/motor`      | `/cmd/motor`     | `std_msgs/msg/Float32MultiArray`  |
| `/audio/raw`      | `/audio/raw`     | `std_msgs/msg/Float32MultiArray`  |
| `/motor/rpm`      | `/motor/rpm`     | `std_msgs/msg/Float32MultiArray`  |

The world file (`worlds/warehouse.sdf`) bakes in a 30 m corridor
that mirrors `experiments/sim/world.py`'s `corridor_white_wall`
scenario: side walls at $y = \pm 3$\,m and a partial wall at
$x = 15..16$\,m, $y = -2..2$\,m.

## Prerequisites

- ROS 2 Humble (Ubuntu 22.04) or Jazzy (Ubuntu 24.04)
- Gazebo Harmonic (`gz-sim 8`)
- `ros_gz_bridge`, `ros_gz_sim`, `cv_bridge`
- Python 3.10+ with `numpy`, `opencv-python`, `pyyaml`,
  `transforms3d`. CasADi is needed only when the production MPC
  is enabled; the LQR fallback (`gps_denied_drone/control/lqr.py`)
  drives the drone otherwise.

## Build

```bash
cd ~/ros2_ws
ln -s /path/to/talentflows src/gps_denied_drone
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select gps_denied_drone
source install/setup.bash
```

## Launch

```bash
ros2 launch gps_denied_drone sim.launch.py
```

This starts gz-sim with `worlds/warehouse.sdf` and the six nodes:

- `perception_node` -- `cv2.ORB` feature density on
  `/camera/image_raw`. Publishes `/slam/tracking_ok` (Bool) and
  `/slam/inliers` (Int32). The full ORB-SLAM3 backend is selectable
  by overriding the `backend` argument of
  `gps_denied_drone.perception.visual_slam.VisualSLAM`.
- `range_node` -- median outlier filter on `/range/raw` -> `/range/filtered`.
- `fusion_node` -- ESKF predict / update on IMU + SLAM + range.
  Stub today; pose passthrough until the production filter lands.
- `mpc_node` -- consumes fused odometry and the advisor outputs;
  emits `/cmd/motor`. The default solver is the analytical LQR
  (closed-form; honors advisor weights and the acoustic clearance
  constraint at controller rate).
- `reasoning_node` -- `Monitor + advisor + SafetySupervisor` at
  $0.5$--$2$\,Hz. The `advisor` ROS parameter selects `slm` or
  `rule_tree`; the `slm_backend` parameter selects between the
  conservative stub and the `llama_cpp_gemma3_1b` real backend.
- `acoustic_node` -- bandpass + per-direction proximity. Default
  is the synthetic backend (matches the Python harness); a
  microphone-array plugin can be wired by replacing the
  `<plugin>` block in `worlds/warehouse.sdf`.

## Mission script

The default world gets the drone to its first waypoint via the X3
quadrotor's built-in flight controller and the MPC. To drive a
mission programmatically, publish a `geometry_msgs/PoseStamped` on
`/mission/waypoint_raw`:

```bash
ros2 topic pub /mission/waypoint_raw geometry_msgs/PoseStamped \
    '{header: {frame_id: world}, pose: {position: {x: 14.0, y: 0.0, z: 1.5}}}'
```

The reasoning node forwards (and possibly nudges) this onto
`/mission/waypoint`, which the MPC tracks.

## Recording a run

```bash
ros2 bag record -o run-$(date +%s) \
    /clock /camera/image_raw /imu /range/raw \
    /odom/fused /slam/tracking_ok /slam/inliers \
    /acoustic/proximity /acoustic/clearance \
    /mission/waypoint /mission/mode /mission/safety_clamped \
    /cmd/motor
```

Then post-process the bag into the same CSV format the Python
harness emits via a small adapter (TODO:
`experiments/gazebo/bag_to_csv.py`). Until that adapter lands,
inspect the bag with `ros2 topic echo` or rqt.

## Ablations

The launch file accepts the same `use_acoustic` / `advisor` /
`slm_backend` arguments as the Pi 5 launch:

```bash
# Acoustic backup OFF (will collide with the wall)
ros2 launch gps_denied_drone sim.launch.py use_acoustic:=false

# SLM advisor with a real Gemma 3 1B GGUF
GEMMA3_GGUF=/path/to/gemma-3-1b.q4_k_m.gguf \
ros2 launch gps_denied_drone sim.launch.py \
    advisor:=slm slm_backend:=llama_cpp_gemma3_1b
```

## What we expect to see in the camera-ready

- Per-scenario variance in min-clearance and ATE, since the
  Gazebo dynamics + image perception interact non-trivially with
  scenario-specific degradation.
- The `cv2.ORB` inlier count should track texture density: on the
  textured corridor walls the count is several hundred; near the
  white wall it drops below the `min_features = 30` threshold and
  `tracking_ok` goes false. The Monitor's `slam_inlier_trend`
  field becomes a meaningful signal at this point.
- An end-to-end latency budget of **image-arrival -> motor-command
  out** that fits the Pi 5 + AI HAT power budget reported in
  `docs/hardware_target.md`.

## Open todos for the Gazebo run (not blocking the paper)

- Microphone-array plugin (option 1 in `warehouse.sdf`) so
  acoustic_node has a real audio stream.
- Bag -> CSV adapter so the same `aggregate.py` and
  `paired_stats.py` scripts can run on Gazebo data.
- Switch `mpc_node` from the LQR fallback to a CasADi NLP for the
  perception-aware production MPC. The `PerceptionAwareCost` and
  `AcousticConstraint` types in `gps_denied_drone/control/mpc.py`
  are stubs ready to be filled.
