# System overview and paper-acceptability assessment

## Mental map: how everything works together

### Dataflow

```
                       PHYSICAL DRONE / GAZEBO SIM
                                  |
       +--------------+-----------+-----------+----------------+
       |              |           |           |                |
   forward cam     IMU + 1D   motor RPMs   battery       4-ch microphone
       |          rangefinder      |          |                |
       v              v            v          v                v
 /camera/image_raw  /imu  /range/raw  /motor/rpm /battery/percent /audio/raw
       |              |       |          |          |             |
       v              v       v          v          |             v
 perception_node  fusion_node range_node     [bridged in]    acoustic_node
 (ORB-SLAM3       (ESKF +   (median +                       (ego-noise
  mono-inertial)   scale     outlier                         passive
       |          recovery)  reject)                         proximity)
       |              ^       |                                  |
  /slam/pose ---------+       |                                  |
  /slam/tracking_ok           v                                  v
       |             /odom/fused        /acoustic/{proximity,
       |                  |               clearance, confidence}
       |                  |                       |
       |        +---------+-----------+-----------+
       |        |                                 |
       v        v                                 v
                   monitor (HealthMonitor)
                          |
                          v
                  observation JSON
                          |
                          v
              SLM advisor (Gemini Nano on Pixel /
                 Gemma 3 1B on Pi 5 + AI HAT)
                          |
                          v
                  ReasoningAction JSON
                          |
                          v
              SafetySupervisor (hard clamp)
                          |
        +-----------------+------------------+
        v                 v                  v
 /mission/waypoint   /mpc/weight_       /mpc/sensor_trust
 /mission/speed_cap   overrides         /mission/mode
                          |                  |
                          v                  v
                  mpc_node (perception-aware
                  MPC with acoustic clearance
                  constraint)
                          |
                          v
                      /cmd/motor
                          |
                          v
              ros_gz_bridge -> Gazebo motors,
              OR PX4 / Betaflight on hardware
```

### Three timescales

| Loop | Rate | Owner | Latency target on Pi 5 |
|---|---|---|---|
| Inner control | 100 Hz | MPC | < 5 ms solve |
| Perception / fusion | 30 / 50 / 200 Hz | SLAM, range, IMU | < 30 ms image-to-pose |
| Advisory | 0.5--2 Hz | SLM | < 600 ms tick |

The inner control loop is classical and hard real-time. The advisory
loop is contextual and slow. The Monitor + SafetySupervisor pair is
how these timescales are joined safely: the SLM cannot violate the
inner loop's invariants because the supervisor rewrites its outputs
before publication.

### Four roles, who owns what

- **Estimator** (perception + fusion + range + acoustic): "where am I,
  is each sensor working?" Outputs odometry and per-sensor health.
- **Controller** (MPC): "given the current waypoint, weights, trusts,
  and clearance constraints, what motor command tracks it?"
- **Advisor** (Monitor + SLM): "given everything I know, what should
  the mission do for the next second?" Replaces the if/else tree.
- **Safety authority** (SafetySupervisor + MPC constraints): "no
  matter what anyone else says, the drone stays inside the safe
  envelope."

These four roles map cleanly to the directories under
`gps_denied_drone/`. No role mixes responsibilities; that is what
makes the system testable.

### Failure cascade walkthrough

A typical SLAM-degradation event, end to end:

1. **t=0**: drone is at 2 m altitude, 3 m/s, SLAM healthy (200 inliers).
2. **t=2 s**: textures vanish (white wall ahead). Inlier count starts
   falling. Monitor reports `slam_inlier_trend=falling`.
3. **t=2.5 s**: SLM tick runs. Sees falling trend, intact range,
   low acoustic. Emits `mode=slow_advance`, `w_perception=2.0`,
   `speed_cap=1.5`. Supervisor passes it through.
4. **t=3.0 s**: tracking lost. Monitor reports `slam_tracking_ok=false`,
   `slam_seconds_since_dropout=0.0`.
5. **t=3.5 s**: SLM tick runs. Sees dropout. If acoustic confidence
   < 0.5, emits `mode=hover`. If acoustic confidence > 0.5 with
   front proximity > 0.7, emits `mode=retreat`,
   `waypoint_delta_m=[-0.6, 0, 0]`, `sensor_trust={slam:0.1, acoustic:0.9}`.
6. **t=3.5 s+**: MPC tracks the new waypoint with hard acoustic
   clearance constraints active. Drone backs off.
7. **t=4--6 s**: textures return; SLAM relocalizes via ORB-SLAM3
   Atlas. Monitor reports `slam_seconds_since_dropout=2.5`. SLM
   returns to nominal.

The if/else version of this is hundreds of lines and hard to test
exhaustively; the SLM version is one prompt and four few-shot
examples.

## Paper-acceptability assessment

### Strengths

- **Genuinely novel combination**: I have no evidence in the
  literature of a paper joining (a) perception-aware MPC, (b)
  monocular VSLAM with 1D-rangefinder scale recovery and clearance
  fallback, (c) on-device SLM as a real-time advisor over health
  signals (not a command translator), and (d) passive ego-noise
  reflectometry as a SLAM-degradation backup, on a single ROS 2 +
  Gazebo testbed.
- **Reusable design pattern**: the Monitor / SLM / SafetySupervisor
  three-layer split is a defensible contribution on its own and
  generalizes beyond drones.
- **Open-source release**: full ROS 2 package + sim launch makes
  the work reproducible, which reviewers reward.
- **Honest hardware target**: Pi 5 + AI HAT with measured power and
  CPU budgets is concrete enough for engineers to repeat.

### Risks (in order of severity)

1. **Execution risk -- the code is scaffolded, not implemented.**
   `process_frame`, `solve`, `update_*`, and the acoustic DSP are
   `NotImplementedError`. No reviewer will accept results from stubs.
   This is the dominant risk.
2. **Acoustic SNR is brutal (-20 dB).** Passive ego-noise reflectometry
   on a flying drone is plausible but unproven. If it does not work
   well enough to demonstrate measurable obstacle avoidance, the
   "five-component" story collapses to four.
3. **SLM substitution muddles the title.** Calling it "Gemini Nano"
   while running Gemma 3 1B on Pi 5 invites reviewer pushback. The
   paper must either run on Pixel/Tensor for real, or rebrand
   honestly ("on-device Gemini Nano-class SLM").
4. **No real flight**. Sim + HIL is a workshop-tier story; for ICRA /
   IROS / RA-L acceptance, at minimum a tethered indoor flight is
   expected.
5. **Many components, each shallow.** Reviewers will probe whether
   we advanced any single component or just glued them together. The
   answer needs to be one of: (a) the SLM-advisor design pattern, or
   (b) the passive-acoustic-as-SLAM-backup result. Both need
   evidence, not just architecture.
6. **Statistical rigor**. One sim run per scenario is not enough;
   minimum is ~20 trials per scenario per ablation across multiple
   seeds.
7. **Direct competitor: BatDeck**. Same end goal (low-power
   obstacle avoidance on nano-drones) with active ultrasound. The
   passive angle (no emitter, multi-role mic, lower acoustic
   detectability) needs to be argued sharply.

### Target venues by realistic-acceptance probability

| Venue | Type | Bar | Realistic if scaffold is filled in |
|---|---|---|---|
| arXiv tech report | preprint | none | yes, today |
| ICRA / IROS workshops | workshop | low | very likely |
| ICUAS | UAV conference | medium | likely with sim + HIL |
| SSRR | safety/rescue robotics | medium | likely; fits the GPS-denied story |
| RA-L | journal letter | high | requires hardware flight + 2+ baselines |
| ICRA / IROS main track | top conference | very high | requires hardware + statistical eval + sharp single-claim novelty |
| T-RO | top journal | extreme | full system, extensive eval, 6+ months |

### Verdict

In its current scaffolded state: **strong arXiv tech report, viable
workshop submission**. Not yet a full conference paper.

To reach **ICUAS or RA-L** (~3--4 months of execution):

- Implement the perception, MPC, acoustic, and SLM pipeline end to
  end on at least the simulator.
- Run >= 20 trials per scenario per ablation. Report mean +/-
  std dev for each metric.
- Compare against >= 2 baselines: classical PAMPC alone, and a
  rule-tree supervisor instead of the SLM. The latter is the most
  important ablation: it answers the reviewer's question "what does
  the LLM actually add?"
- Verify the acoustic component on real recordings (rotor noise vs
  reflector at known distances) before claiming it as a contribution.

To reach **ICRA / IROS** (~6+ months, much more risk):

- Everything above, plus a real-flight indoor demo (tethered is fine).
- Pick a single sharp claim and lead with it. Two candidates:
  1. *"Passive ego-noise reflectometry is a viable SLAM backup on
     micro-drones"* -- a specific empirical claim, falsifiable.
  2. *"An on-device SLM as a real-time supervisor is competitive
     with or better than a hand-coded rule tree on N scenarios"* --
     a specific design claim, requires a controlled comparison.
- Either claim is publishable in the top tier on its own; both
  together is more than one paper.

### Reviewer concerns to preempt in the writing

- *"What does the LLM add over a hand-coded supervisor?"* -> ablation
  with a rule-tree advisor on the same observations.
- *"Why passive acoustic when BatDeck did it actively for less power?"*
  -> argue (a) no emitter, (b) the same mics serve drone-audition
  tasks (multi-role), (c) lower acoustic signature, and back each up
  with a number.
- *"Gemini Nano or Gemma 3?"* -> say plainly which one runs on which
  hardware and that the schema and prompt are identical.
- *"Where are the error bars?"* -> N >= 20 trials per cell, report
  std and a non-parametric significance test.
- *"Is this just system integration?"* -> lead with the single sharp
  claim, push the integration to a "system" subsection.

### Suggested paper structure given the current code

1. Introduction -- lead with the single sharp claim.
2. Related work -- already drafted; tighten the BatDeck contrast.
3. Method -- already three subsections (1D scale, MPC,
   reasoning); add the acoustic backup section explicitly (already
   started).
4. System and implementation -- ROS 2 + Gazebo, Pi 5 + AI HAT,
   power and CPU budget. This is where the open-source release
   and hardware target shine; one figure of the dataflow above.
5. Experiments -- four scenarios x five ablations x N trials,
   with the rule-tree-vs-SLM ablation called out as the central
   comparison.
6. Discussion -- limitations frankly stated (passive acoustic SNR
   floor, SLM hallucination rate, sim-to-real gap).
7. Conclusion + future work.
