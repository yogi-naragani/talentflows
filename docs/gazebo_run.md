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

## Running on Gazebo Fortress (Humble default)

Ubuntu 22.04 + ROS 2 Humble ships ros_gz paired with **Fortress**
(`gz-sim 6`, launcher `ign gazebo`), not Harmonic. The primary world
above (SDF 1.10, `gz-sim-*` plugin filenames, sibling-model sensors)
will not parse on Fortress. Use the sibling assets instead:

- `worlds/warehouse_fortress.sdf` -- same corridor and obstacle layout,
  in SDF 1.6 with `ignition-gazebo-*` plugin filenames.
- `worlds/models/x3_sensors/` -- a fork of the Fuel X3 quadrotor with
  the forward camera, downward 1D `gpu_lidar`, and IMU baked into
  `X3/base_link` (Fortress rejects cross-model fixed joints between an
  included model and a sibling sensor model).
- `launch/sim_fortress.launch.py` -- like `sim.launch.py` but uses the
  Fortress world, sets `IGN_GAZEBO_RESOURCE_PATH` so the local
  `x3_sensors` model is discoverable, bridges with `ignition.msgs.*`
  type names, and (by default) launches the `synthetic_acoustic_node`
  in place of `acoustic_node`.

```bash
ros2 launch gps_denied_drone sim_fortress.launch.py
```

## Synthetic acoustic backup

`gps_denied_drone/nodes/synthetic_acoustic_node.py` is the Gazebo-only
substitute for the real microphone pipeline. It subscribes to
`/model/x3_sensors/odometry`, builds the `corridor_white_wall` World
geometry from `experiments/sim/world.py`, and publishes
`/acoustic/proximity`, `/acoustic/clearance`, and `/acoustic/confidence`
at 20 Hz with the same shape and noise profile the Python harness
uses. It is **not** a stand-in for the production sensor (no DSP path,
no RPM-derived SNR floor) -- it exists so the `use_acoustic` ablation
is meaningful in the Gazebo build until a microphone plugin lands.

Disable with `synthetic_acoustic:=false`, which falls back to the real
`acoustic_node` (which will sit idle until `/audio/raw` is fed).

## Bag -> CSV

`experiments/gazebo/bag_to_csv.py` converts a recorded run into the
same per-trial row that `experiments.sim.runner.TrialMetrics.to_row`
emits, so `aggregate.py` and `paired_stats.py` work unchanged on
Gazebo data:

```bash
ros2 bag record -o runs/gz_run \\
    /clock /camera/image_raw /imu /range/raw /range/filtered \\
    /odom/fused /slam/tracking_ok /slam/inliers \\
    /acoustic/proximity /acoustic/clearance /acoustic/confidence \\
    /mission/waypoint /mission/mode /cmd/motor \\
    /model/x3_sensors/odometry

python -m experiments.gazebo.bag_to_csv runs/gz_run \\
    --scenario corridor_white_wall --advisor rule_tree \\
    --seed 0 --use-acoustic true --out runs/gz.csv
python -m experiments.sim.aggregate runs/gz.csv
```

The adapter reads the bag's sqlite3 file directly (no `rosbag2_py`
dependency) and uses `/model/x3_sensors/odometry` as ground-truth
pose for ATE, min-clearance, and waypoint-progression metrics.
`safety_clamp_*` and `advisor_ticks` are zero in the Gazebo row
until the reasoning node logs its decisions onto a bridged ROS topic.

## Open todos for the Gazebo run (not blocking the paper)

- Microphone-array plugin (option 1 in `warehouse.sdf`) so
  the production acoustic_node has a real audio stream and the
  synthetic substitute can be retired.
- Switch `mpc_node` from the LQR fallback to a CasADi NLP for the
  perception-aware production MPC. The `PerceptionAwareCost` and
  `AcousticConstraint` types in `gps_denied_drone/control/mpc.py`
  are stubs ready to be filled.
- Have the reasoning node publish `/mission/safety_clamped` and
  `/mission/mode` events to the bag so `bag_to_csv.py` can fill in
  the safety-clamp and advisor-tick columns.
