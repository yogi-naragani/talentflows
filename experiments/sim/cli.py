"""Run an ablation grid and print a CSV of TrialMetrics rows.

Usage:
    python -m experiments.sim.cli \
        --advisors slm rule_tree \
        --scenarios corridor_white_wall texture_mask_burst \
        --seeds 0 1 2 3 4 \
        --duration 30
"""

import argparse
import csv
import sys

from .runner import TrialConfig, run_trial


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--advisors", nargs="+",
                    default=["rule_tree", "slm"])
    ap.add_argument("--scenarios", nargs="+",
                    default=["corridor_white_wall", "texture_mask_burst",
                             "illumination_drop", "dual_failure_slam_and_range"])
    ap.add_argument("--seeds", nargs="+", type=int, default=[0, 1, 2, 3, 4])
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--no-acoustic", action="store_true",
                    help="ablate the acoustic backup")
    ap.add_argument("--out", default="-",
                    help="csv path or '-' for stdout")
    args = ap.parse_args(argv)

    rows: list[dict] = []
    for scenario in args.scenarios:
        for advisor in args.advisors:
            for seed in args.seeds:
                cfg = TrialConfig(
                    scenario=scenario,
                    advisor=advisor,
                    seed=seed,
                    duration_s=args.duration,
                    use_acoustic=not args.no_acoustic,
                )
                m = run_trial(cfg)
                rows.append(m.to_row())

    if not rows:
        return 0
    out = sys.stdout if args.out == "-" else open(args.out, "w", newline="")
    try:
        w = csv.DictWriter(out, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    finally:
        if out is not sys.stdout:
            out.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
