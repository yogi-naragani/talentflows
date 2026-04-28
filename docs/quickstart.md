# Quickstart: reproduce the headline result

This repository's central empirical claim is that a passive acoustic
backup, enforced as a hard MPC clearance constraint at controller
rate, eliminates collisions during scripted SLAM-degradation
scenarios. You can reproduce the result on a laptop in well under
a minute.

## One-command reproduction

```bash
git clone <repo> talentflows
cd talentflows
pip install numpy scipy matplotlib pytest
make headline
```

`make headline` runs `test`, `ablation`, `figure`, and `stats` in
that order and prints the artifact paths.

## What you get

| Artifact | What's in it |
|---|---|
| `experiments/runs/acoustic_on.csv` | 40 rule-tree trials with acoustic backup enabled |
| `experiments/runs/acoustic_off.csv` | 40 rule-tree trials with acoustic backup disabled |
| `experiments/runs/acoustic_combined.csv` | concatenated for paired tests |
| `experiments/runs/acoustic_summary.csv` | mean/std/min/max grouped by `(scenario, advisor, x_use_acoustic)` |
| `experiments/runs/acoustic_paired_stats.csv` | Wilcoxon signed-rank: $W$ and two-sided $p$ per metric |
| `paper/figures/trajectory_acoustic_comparison.png` | Top-down side-by-side ablation plot |

## What to expect

Across $4 \times 10 = 40$ trials per arm:

- **Collisions:** acoustic off `1.00 ± 0.00` (every trial); acoustic on `0.00 ± 0.00` (no trial).
- **Min clearance (m):** off `0.154 ± 0.008`; on `0.641 ± 0.037`.
- **Wilcoxon paired:** $p < 10^{-6}$ on collisions and on min clearance ($n=40$).

The figure shows the qualitative behavior: red trajectories
(acoustic off) pile into the wall and terminate at the collision
marker; blue trajectories (acoustic on) stop $\sim$0.6\,m short.

## Individual targets

Run only what you need:

```bash
make test         # 7-test suite, ~7 s
make ablation     # the 40-trial CSVs
make figure       # the trajectory PNG
make stats        # paired Wilcoxon on the saved CSV
make clean        # remove generated artifacts
```

## On the Pi 5 + AI HAT target

For deployment to the Raspberry Pi 5 + AI HAT+ (Hailo) target with a
real on-device SLM, see `docs/hardware_target.md`. The simulation
runs identically there; the difference is which backend
`gps_denied_drone.reasoning.gemini_nano` uses --
`gps_denied_drone.reasoning.llama_cpp_backend.create_backend(name)`
loads a Gemma 3 1B Q4_K_M GGUF via `llama-cpp-python` when one is
configured.

## Branch layout

`docs/BRANCHES.md` describes how the integration branch
(\textit{this} branch) and the two paper branches relate. Both
paper branches inherit the simulator and figure infrastructure
from here; each one's experiments section narrows the focus to a
single sharp claim.

## Where the numbers come from

- `experiments/sim/runner.py` -- per-trial loop tying the synthetic
  sensors to the same library code the ROS 2 nodes use (`Monitor`,
  `RuleTreeAdvisor` / `GeminiNanoClient`, `SafetySupervisor`,
  `LQRController`).
- `experiments/sim/sensors_sim.py` -- synthetic camera-feature
  counts, 1D range, and 6-direction acoustic proximity. Acoustic
  proximity is corrupted by additive Gaussian noise to model the
  $-20$ dB SNR floor of passive ego-noise reflectometry.
- `gps_denied_drone/control/lqr.py` -- analytical LQR-style
  controller honoring the advisor's $v_{\max}$, weight scales, and
  acoustic clearance. The clearance constraint is what makes the
  on/off comparison decisive: it is enforced every $10$ ms, not
  every advisor tick.

## Caveats

- The Python harness has no path planner, so mission completion is
  $0$ in both arms. The headline claim is **safety preservation
  under SLAM degradation**, not mission completion.
- Per-scenario means are identical because the controller-rate
  acoustic constraint is scenario-invariant in this harness. Real
  Gazebo runs (`ros2 launch gps_denied_drone sim.launch.py`) will
  introduce per-scenario variance; expect that to appear in the
  camera-ready.
- The acoustic-noise model is generous on SNR. The
  `experiments/snr_analysis.py` bench harness on
  `claude/paper-acoustic-backup-VgEQR` is where the real-hardware
  SNR floor will be measured before publication.
