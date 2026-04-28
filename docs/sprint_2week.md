# 2-week sprint plan

Compresses the canonical 8-week plan in `CLAUDE.md` into 10 working
days at ~3 hours/day (~30 hours total). Cuts the buffer between weeks
and leans on the existing repo scaffolding (Python sim, OpenCV ORB
proxy, warehouse SDF, Makefile, tests, paper LaTeX).

## What "fast" buys you and what it costs

You buy:
- Working VIO loop in Gazebo with EuRoC numbers in your CV email by
  the end of week 2.
- A learned-component demo (depth, the cheaper choice).
- A 4-page write-up on arXiv.

You give up:
- Time to actually understand SLAM math. **You will be a button-pusher
  on Day 4-5 unless you read the ORB-SLAM3 paper in parallel.**
- A polished RL policy. Default to depth estimation (Option A in
  CLAUDE.md week 7).
- Any hardware story. Sim-only, stated openly in the paper.

## Day-by-day

### Day 1 - Environment, no excuses

**Done when:** `make px4_sitl gz_x500` spawns the X500 in Gazebo.

- Install Ubuntu 22.04 (native or WSL2-with-WSLg). If that takes
  more than 4 hours, fall back to the Docker route (see
  `Dockerfile`) and proceed.
- Install ROS 2 Humble + Gazebo Harmonic.
- Clone PX4-Autopilot, build SITL.
- `make px4_sitl gz_x500` -- confirm the drone spawns.
- Push environment notes (commit your `~/.bashrc` snippets to the repo
  in `docs/env_setup.md`).

If you finish early, take Day 2 today.

### Day 2 - PX4 -> ROS 2 bridge

**Done when:** `ros2 topic list` shows PX4 telemetry, your custom
node publishes a `TrajectorySetpoint` and the drone moves.

- Install `px4_msgs` and the XRCE-DDS agent.
- Use the scaffolded `gps_denied_drone/nodes/px4_bridge.py` -- it
  publishes `OffboardControlMode` and `TrajectorySetpoint` and
  forwards a pose into `/fmu/in/vehicle_visual_odometry`. Wire it
  into `launch/sim.launch.py`.
- Smoke test: arm + offboard + climb to 2 m + land.

### Day 3 - Square-flight demo + tag v0.1

**Done when:** `python scripts/demo_square_flight.py` flies a 5 m
square. Tag commit `v0.1-manual-control`.

- Write the square pattern as a sequence of `TrajectorySetpoint`
  messages in `scripts/demo_square_flight.py`.
- Record a 20 s screen capture (gnome-screen-recorder is fine).

### Day 4 - ORB-SLAM3 build + EuRoC

**Done when:** ORB-SLAM3 produces a trajectory `.txt` for
`MH_01_easy`.

- ORB-SLAM3 (mono-inertial) build. Pangolin + Eigen + OpenCV.
  This is the day most likely to slip; if it does, use
  `experiments/euroc/run_eval.py` against the OpenCV-ORB proxy
  perception node as a fallback baseline -- it produces the same
  trajectory file format.
- Download EuRoC MAV (script: `experiments/euroc/run_eval.py
  --download`).
- Run on `MH_01_easy`, `MH_03_medium`, `V1_01_easy`.

### Day 5 - Trajectory eval + tag v0.2

**Done when:** `evo_ape` reports ATE-RMSE for each EuRoC sequence
and the numbers land in `results/vio_baseline.md`. Tag
`v0.2-vio-baseline`.

- `pip install evo`.
- Run `python -m experiments.euroc.run_eval --eval-only`.
- Generate XY and error-over-time plots.
- Honest numbers in `results/vio_baseline.md` -- no rounding up.

**You should now be able to defend a number out loud:** "ATE on
MH_01_easy was 0.054 m, here is the plot, here is the trajectory file."

### Day 6 - Custom GPS-denied world

**Done when:** drone takes off in `worlds/warehouse.sdf` with GPS
disabled and publishes only camera + IMU.

- The world already exists (corridor, walls, X3 + sensors). What's
  left is wiring PX4 to use this world instead of the default
  `gz_x500`.
- Patch the PX4 airframe config to disable GPS
  (`EKF2_GPS_CTRL = 0`).

### Day 7 - VIO in the loop + tag v0.3

**Done when:** drone does a 30 s autonomous flight in
`warehouse.sdf` using ORB-SLAM3 pose only. Tag `v0.3-vio-in-loop`.

- Pipe camera + IMU into ORB-SLAM3.
- Publish ORB-SLAM3 pose as `/fmu/in/vehicle_visual_odometry` via
  `px4_bridge.py` (already scaffolded).
- Configure EKF2 external vision (`EKF2_HGT_REF`, `EKF2_AID_MASK`).
- Log ground-truth pose from Gazebo + estimated pose; compute live
  drift.

### Day 8 - Learned depth, training

**Done when:** depth model fine-tunes on synthetic Gazebo images on
Brev (cloud GPU); validation MSE drops vs zero-shot baseline.

- Pick MiDaS small or Depth Anything v2 small.
- Generate ~5k Gazebo image + ground-truth-depth pairs by flying
  the drone around `warehouse.sdf` and saving sensor + simulator
  state. Script in `learning/depth/dump_dataset.py`.
- Fine-tune on Brev. Train script in `learning/depth/train.py`.
- Save the best checkpoint.

If Day 8 looks like it will run long, **switch to zero-shot
inference** for Day 9 and fold the fine-tune into the limitations
section.

### Day 9 - Depth deployment + comparison

**Done when:** ROS 2 node runs the ONNX model on the camera stream,
publishes a depth image, and a side-by-side RViz screenshot
(predicted vs ground-truth) is in `results/depth_qualitative.md`.

- ONNX export.
- `gps_denied_drone/nodes/depth_node.py` (new).
- Capture RViz screenshot with predicted depth + GT depth + RGB.

### Day 10 - Write-up + release

**Done when:** repo is tagged `v1.0`, paper is on arXiv, demo GIF in
README.

- Repo cleanup: README has install + 1-command demo.
- Record the 60 s demo: takeoff -> autonomous flight in warehouse
  using VIO -> obstacle in front -> learned depth visualization ->
  land. Convert to GIF for README.
- Paper draft (4-6 pages, IEEE conference template):
  - Abstract: one paragraph.
  - Related work: VIO + learning-based UAV perception.
  - System architecture: one diagram (the existing
    `paper/figures/system_diagram.tex`, retargeted from the
    acoustic-backup framing).
  - Experiments: EuRoC numbers (Day 5) + autonomous flight result
    (Day 7) + depth qualitative (Day 9).
  - Limitations: sim-only, depth quality, etc.
- arXiv submission (cs.RO).
- Tag `v1.0`.

## Cut lines (priority order, drop right-to-left if time is short)

1. (LOW PRIO) Acoustic backup numbers from the older paper branches.
   Keep as bonus appendix only if time.
2. (LOW PRIO) Per-scenario degradation variance.
3. (MED PRIO) Learned depth fine-tune on synthetic data. Fall back to
   zero-shot.
4. (HIGH PRIO) Polished arXiv preprint. If everything else slips,
   release the repo with a `RESULTS.md` instead of an arXiv paper --
   that is still a real artifact in a CV email.

## What NOT to chase during the sprint

- **A perfect VIO score.** EuRoC ATE around 0.05-0.10 m on
  mono-inertial is fine. Don't tune for hours.
- **A novel SLAM contribution.** We are not contributing SLAM. We are
  building a system.
- **Real-flight stories.** Stated as future work in the paper.
- **The acoustic backup direction from earlier sessions.** Decoupled
  on its own branches. Touch it only if you finish Day 10 early.

## Maintenance

After every working session:
1. Update `_registry.md` with what changed.
2. Update the relevant `contracts/*.md` with new pre/post state.
3. Commit. Push.
4. Write 2 lines in `LEARNINGS.md` about what you actually understood
   today (not "I ran ORB-SLAM3"; "what is local bundle adjustment
   doing here, in my words").
