# contracts/vio_pipeline.md

**Pre-state:** ROS 2 Humble + Gazebo + PX4 SITL working. EuRoC dataset
downloaded under `~/datasets/EuRoC/`.

**Post-state:** ORB-SLAM3 (mono-inertial mode) builds and runs against
EuRoC sequences `MH_01_easy`, `MH_03_medium`, `V1_01_easy`. Each run
produces a TUM-format trajectory `.txt` under `experiments/runs/euroc/`.

**Dependencies:**
- `contracts/sim_setup.md`
- Pangolin, Eigen 3.x, OpenCV 4.x with `opencv_contrib`, DBoW2 (vendored
  in ORB-SLAM3 tree).

**Breakage risk:** MEDIUM-HIGH. ORB-SLAM3's CMake fights with newer
OpenCV; expect to patch one or two `find_package` lines.

**Acceptance test:**
```bash
python -m experiments.euroc.run_eval --sequences MH_01_easy
ls experiments/runs/euroc/MH_01_easy.tum   # exists, non-empty
```

**Current status:** wip. The Python-only OpenCV ORB proxy in
`gps_denied_drone/perception/visual_slam.py` already runs against any
image sequence; it produces a tracking-health signal but no pose.
ORB-SLAM3 itself is still user-side build work.

**Fallback if Day 4 slips:** run the eval against the OpenCV ORB proxy
to produce a feature-density-vs-time plot, defer the trajectory
comparison to Day 5 morning. The proxy is enough to demonstrate the
*pipeline*; the ORB-SLAM3 numbers go in last.

**Watch out for:**
- ORB-SLAM3's ROS 2 wrapper (e.g. `Mechazo11/ros2_orb_slam3`) is
  community-maintained and may lag. If the wrapper is broken, run
  ORB-SLAM3 standalone and bridge its output into ROS 2 manually.
- EuRoC ground-truth is in body frame; align trajectories with
  `evo_ape ... -a` (sim3 alignment) for monocular-inertial output.
