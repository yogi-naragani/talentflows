# contracts/evaluation.md

**Pre-state:** Trajectory `.txt` files exist for each EuRoC sequence
(`contracts/vio_pipeline.md` post-state).

**Post-state:** `results/vio_baseline.md` contains a table of
ATE-RMSE (m) and RPE-trans (m) per sequence, plus XY plots in
`results/figures/`. The autonomous-flight result from
`worlds/warehouse.sdf` is summarized in `results/autonomous_flight.md`
with ground-truth-vs-VIO drift over time.

**Dependencies:**
- `contracts/vio_pipeline.md`
- `contracts/custom_world.md`
- `evo` Python package

**Breakage risk:** LOW. Eval is mechanical.

**Acceptance test:**
```bash
python -m experiments.euroc.run_eval --eval-only \
    --sequences MH_01_easy MH_03_medium V1_01_easy
ls results/figures/*.png        # XY + error-over-time plots
```

**Current status:** wip. Script exists at
`experiments/euroc/run_eval.py`; it shells out to `evo_ape` and
collects the resulting CSV. Results are written into
`results/vio_baseline.md` after each run.

**Numbers to defend in a CV email:**
- ATE-RMSE per EuRoC sequence (mono-inertial expected: 0.04 - 0.12 m).
- Drift over 30 s of autonomous flight in `warehouse.sdf` (expected:
  < 0.5 m end-to-end translational drift on a textured corridor).

**Watch out for:**
- `evo` requires the trajectory to be pose-aligned with ground truth.
  Use `evo_traj tum --align ...` first, then `evo_ape`.
- For mono-inertial, use sim3 (scale + rotation + translation) alignment;
  for stereo or stereo-inertial, use SE(3).
- Save raw trajectory files alongside the plots; reviewers will want
  to re-run the eval.
