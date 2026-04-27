# Paper focus (this branch): SLM advisor vs rule-tree baseline

This branch leads the paper with a single sharp design + empirical
claim:

> An on-device small language model, given the same structured health
> observation as a hand-coded rule tree, matches or beats the rule
> tree on in-distribution GPS-denied flight scenarios and
> outperforms it on out-of-distribution scenarios -- while remaining
> bounded by an external safety supervisor.

Everything else in the repository -- ORB-SLAM3, the 1D rangefinder,
the passive acoustic backup, the perception-aware MPC, the Pi 5 + AI
HAT target -- is *the system the advisor supervises*, not the
contribution.

## Central comparison

`(advisor=slm)` vs `(advisor=rule_tree)` with everything else held
fixed (`experiments/configs/paper_slm_vs_rules.yaml`):

In-distribution scenarios (rule tree was tuned on these):
- corridor white wall
- illumination drop
- acoustic alarm front
- low battery far from origin

Out-of-distribution scenarios (rule tree was NOT tuned on these):
- dual failure (SLAM and range concurrently)
- intermittent acoustic with recovery
- texture mask during yaw sweep
- novel clutter pattern

25 trials per cell across 5 seeds.

## Baselines that strengthen the comparison

- `rule_tree_tuned_per_scenario`: upper bound on what a hand-coded
  baseline can achieve by overfitting per scenario. The fairest
  hard target for the SLM.
- `slm_no_few_shot`: SLM with system prompt only. Isolates the
  contribution of few-shot examples.
- `slm_smaller_model`: SLM with a 350M-param model. Latency / quality
  Pareto for deployment.

## What makes the comparison fair

- Both advisors consume the *same* JSON observation produced by the
  Monitor.
- Both emit the *same* JSON `ReasoningAction` schema.
- Both pass through the *same* SafetySupervisor.
- The rule-tree thresholds were derived from the identical decision
  guidance we wrote into the SLM's system prompt, so the rule tree is
  not a strawman.
- We report safety-clamp frequency for each: an advisor that gets
  clamped a lot is leaning on the supervisor and should not get
  credit.

The hand-coded baseline lives at
`gps_denied_drone/reasoning/rule_tree.py`. The reasoning node selects
the advisor at launch via the `advisor` ROS parameter
(`slm` or `rule_tree`).

## Out of scope for this paper

- The passive acoustic backup as a research contribution. We use it
  in the host system but make no claim about whether ego-noise
  reflectometry is a viable SLAM-degradation backup. That claim
  lives in the sibling branch `claude/paper-acoustic-backup-VgEQR`.

## Realistic venue

ICUAS or RA-L if execution is solid. ICRA / IROS main track if the
out-of-distribution result is decisive (SLM clearly wins) and a
real-flight demo lands.
