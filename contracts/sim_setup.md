# contracts/sim_setup.md

**Pre-state:** Bare Ubuntu 22.04 (or Docker daemon for the fallback).
No ROS 2, no Gazebo, no PX4.

**Post-state:** `make px4_sitl gz_x500` spawns an X500 quadrotor in
Gazebo Harmonic. ROS 2 Humble is sourced. `ros2 topic list` shows
PX4 telemetry topics.

**Dependencies:** none (this is the foundation).

**Breakage risk:** HIGH. Mixed sources (Ubuntu, ROS apt, Gazebo apt,
PX4 source) can fight each other. If anything breaks, fall back to
the `Dockerfile` -- it pins versions and is reproducible.

**Acceptance test:**
```bash
make px4_sitl gz_x500   # in PX4-Autopilot dir
ros2 topic list | grep -q /fmu/out/vehicle_local_position
```

**Current status:** todo (user-side, Day 1 of the sprint).

**Watch out for:**
- ROS 2 Humble requires Ubuntu 22.04. Don't try 24.04.
- Gazebo Harmonic + ROS 2 Humble needs the `ros-humble-ros-gzharmonic`
  bridge package (community-maintained); check the Gazebo + ROS
  compatibility matrix before installing.
- PX4 SITL with Gazebo Harmonic moved from the older Ignition Gazebo
  recently -- follow the *current* PX4 docs, not stale forum posts.
