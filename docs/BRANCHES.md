# Branch map

This repository hosts three branches that share the same code base
but diverge in paper focus.

## `claude/gps-denied-drone-research-VgEQR` (this branch -- integration)

The full system: ORB-SLAM3 mono-inertial, 1D rangefinder, passive
acoustic backup, perception-aware MPC, on-device SLM advisor (Monitor
+ SLM + SafetySupervisor), Gazebo Harmonic launch, Pi 5 + AI HAT
deployment. The integration branch carries every component but does
not lead any single paper claim.

Use this branch for:
- shared bug fixes and infrastructure changes that should reach both papers
- the open-source release artifact
- onboarding (read `docs/system_overview.md`)

## `claude/paper-acoustic-backup-VgEQR`

Lead claim: *passive ego-noise reflectometry is a viable
SLAM-degradation backup for GPS-denied micro-drones.*

Diff vs integration:
- focused paper title, abstract, introduction
- `experiments/configs/paper_acoustic.yaml` (acoustic-on/off x SLAM
  dropout x scenario matrix)
- `experiments/snr_analysis.py` SNR-floor bench harness
- `docs/paper_focus.md` states the single sharp claim and what is
  out of scope

Realistic venues: ICUAS, RA-L; ICRA / IROS with a real-flight demo.

## `claude/paper-slm-advisor-VgEQR`

Lead claim: *an on-device SLM, given the same observation as a
hand-coded rule tree, matches or beats it on in-distribution
scenarios and outperforms it on out-of-distribution scenarios, while
remaining bounded by an external safety supervisor.*

Diff vs integration:
- `gps_denied_drone/reasoning/rule_tree.py` -- `RuleTreeAdvisor` with
  the same `.decide(observation) -> ReasoningAction` signature as the
  SLM client
- `reasoning_node` accepts an `advisor` ROS parameter (`slm` |
  `rule_tree`) so paper experiments swap the advisor while keeping
  every other layer identical
- focused paper title, abstract, introduction
- `experiments/configs/paper_slm_vs_rules.yaml` (slm vs rule_tree x
  4 in-distribution + 4 out-of-distribution scenarios)
- `docs/paper_focus.md` states the single sharp claim and what is
  out of scope

Realistic venues: ICUAS, RA-L; ICRA / IROS if the OOD result is
decisive.

## How to keep both papers in sync

1. Land shared infrastructure changes (sensor drivers, MPC formulation,
   Gazebo bridge, Pi 5 deployment) on the integration branch.
2. Cherry-pick or merge into each paper branch. Avoid direct
   commits to a paper branch unless they are paper-specific (paper
   text, focused configs, that paper's baselines).
3. Avoid editing `paper/main.tex` or `paper/sections/01_introduction.tex`
   on the integration branch -- those files diverge per paper.

## Why two papers and not one

Per `docs/system_overview.md`, both claims are top-tier-publishable on
their own; together they exceed the scope of a single conference
paper and would dilute each other's narrative. Splitting them lets
each paper own its central comparison and statistical evaluation
without having to defend an unrelated component to reviewers.

The integration branch is the artifact released to readers who want
the full system. Each paper branch is the artifact released alongside
that paper's preprint.
