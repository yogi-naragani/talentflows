# CLAUDE.md - GPS-Denied Drone Portfolio Project

## Project Goal

Build a credible, end-to-end portfolio piece demonstrating GPS-denied
autonomous drone navigation in simulation, suitable for German PhD
applications (target labs: TU Berlin Leutenegger, TUM Ryll, DLR, Bonn
Stachniss, Freiburg Valada).

**Final deliverable:** Open-source GitHub repo + 4-6 page arXiv preprint
(or ICRA/IROS workshop submission) showing:
1. A custom drone simulated in Gazebo with onboard sensors only
   (camera + IMU, no GPS)
2. Visual-Inertial Odometry running in the loop for state estimation
3. One learned component (depth estimation OR collision avoidance policy)
4. Quantitative evaluation against ground truth

## Constraints

- **No hardware.** Sim-only pipeline.
- **Part-time.** ~15-20 hours/week realistic.
- **Limited GPU at home.** Use NVIDIA Brev for cloud GPU when training.
- **Claude Max plan with session limits.** Tool routing matters.
- **Background:** strong in ROS2, control, mechanical eng. Weak in: SLAM
  math, deep learning for vision, aerial dynamics. Plan accounts for this.

## Tech Stack

| Layer | Tool | Why |
|---|---|---|
| OS | Ubuntu 22.04 (native or WSL2) | ROS2 Humble compatibility |
| Middleware | ROS2 Humble | Industry standard, matches existing skills |
| Simulator | Gazebo Harmonic + PX4 SITL | Free, standard for drones |
| Drone autopilot | PX4 (SITL mode) | Most-used open-source autopilot |
| VIO baseline | ORB-SLAM3 or VINS-Fusion | Standard benchmarks; pick one |
| Trajectory eval | `evo` (Python) | Standard ATE/RPE tool |
| Learning framework | Isaac Lab on Brev (RL) OR PyTorch on Brev (perception) | Cloud GPU |
| Dataset | EuRoC MAV (for VIO baseline) | Free, standard benchmark |
| Version control | Git + GitHub | Public repo from day one |

## Tool Routing Policy

**Use Claude Code for:** ROS2 node scaffolding, launch files, message
defs, CMake/colcon builds, Dockerfiles, URDF/SDF generation, Python
data pipelines + plotting, training-loop boilerplate, README cleanup,
debugging build errors with full stack traces.

**Use Cursor / Codex for:** inline autocomplete, small one-off scripts
(<50 lines), single-file refactors.

**Do manually (no AI):** reading SLAM papers, factor graphs / Lie algebra
basics / Kalman filter math, designing experiments, interpreting
trajectory errors, writing the paper draft (let Claude polish, not
generate), choosing what to read next.

**Spec-block prompt template:**
```
CONTEXT: <paste relevant code/config>
TASK: <one specific thing>
CONSTRAINTS: <ROS2 Humble, no new dependencies, etc.>
OUTPUT FORMAT: <code only, or step-by-step, etc.>
DO NOT: explain what you're doing unless asked
```

## Per-Component Contract System

`_registry.md` at repo root lists every major component and its current
state. Per-component `.md` files in `/contracts/`:
- `contracts/sim_setup.md`
- `contracts/vio_pipeline.md`
- `contracts/custom_world.md`
- `contracts/learned_component.md`
- `contracts/evaluation.md`

Each contract: pre-state, post-state, dependencies, breakage risk.
Update after each session. Prevents Claude Code from regressing earlier
work in later sessions.

## Plan

The original 8-week plan is the canonical reference. This repo is being
executed on a 2-week sprint variant -- see `docs/sprint_2week.md`.

## Anti-patterns to Avoid

- Don't try to learn everything before starting. Start building, learn
  theory in parallel.
- Don't switch VIO systems mid-project. Pick one, commit.
- Don't optimize prematurely. Make it work end-to-end first, improve
  specific stages later.
- Don't let Claude write the paper's "contribution" or "novelty" claims.
  PIs can smell AI-generated research framing.
- Don't skip trajectory evaluation. Numbers are what makes this credible.
- Don't burn Claude Max session limits on Stack-Overflow questions.
  Use Cursor or Google.

## Success Criteria for Application Readiness

After the sprint, the user should be able to email a German PI:

> "I recently completed an open-source GPS-denied drone navigation
> pipeline using PX4, Gazebo, ORB-SLAM3, and a learned [depth/policy]
> component, available at github.com/yourusername/repo with results
> documented at arxiv.org/abs/XXXX. The work extends my prior research
> on differential-drive control (IEEE ICC 2024) into the aerial domain.
> I would be very interested in discussing how this could connect to
> your group's work on [specific paper of theirs]."

That's a real application. That gets read.

## Maintenance Notes

- Update `_registry.md` after every working session.
- Tag git commits at end of each milestone (`v0.1`, `v0.2`, ...) so
  rollback is possible if Claude Code regresses something.
- Keep `LEARNINGS.md` separate from contracts -- write down what you
  understood about SLAM math, EKF tuning, etc., in your own words.
- If the sprint slips, cut scope from the learned component (Day 8-9),
  not from the VIO baseline (Day 4-5). The baseline is what makes the
  project credible.
