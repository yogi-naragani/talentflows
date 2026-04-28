# contracts/custom_world.md

**Pre-state:** `worlds/warehouse.sdf` is a complete world (corridor +
walls + X3 + camera + IMU + 1D range). Lint-tested by
`tests/test_gazebo_assets.py`.

**Post-state:** `ros2 launch gps_denied_drone sim.launch.py` brings up
gz-sim with `warehouse.sdf`, the ros_gz_bridge, and the six drone
nodes. The drone is published to ROS 2 topics
(`/camera/image_raw`, `/imu`, `/range/raw`); GPS is disabled.

**Dependencies:**
- `contracts/sim_setup.md`

**Breakage risk:** LOW. The SDF is well-formed and the launch file
parses; what's left is the EKF2 config patch on the PX4 side.

**Acceptance test:**
```bash
make gazebo-lint                                  # XML + Python parse
ros2 launch gps_denied_drone sim.launch.py        # spawns
ros2 topic hz /camera/image_raw                   # ~30 Hz
ros2 topic hz /imu                                # ~200 Hz
```

**Current status:**
- World: done.
- Camera + IMU + 1D range: done in SDF.
- Mic array: still a TODO (placeholder block in the SDF).
- GPS-disabled airframe: todo (user-side PX4 config).

**Watch out for:**
- The X3 model is included from gz-sim Fuel; first run needs a
  network connection. After that, the model is cached in
  `~/.gz/fuel/`.
- The downward 1D range emulator is a 1x1 `gpu_lidar`. Don't increase
  samples; downstream code assumes a single beam.
- Drone-relative sensor poses are set in SDF via a fixed joint.
  Verify with `ign topic -e -t /sensor/.../pose` if the camera ends
  up in the wrong place.
