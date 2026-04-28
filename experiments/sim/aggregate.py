"""Aggregate a TrialMetrics CSV into paper-ready group summaries.

Groups by (scenario, advisor, x_use_acoustic) and reports n, mean,
std, and min/max for the headline metrics: completion, collisions,
min_clearance_m, ate_rmse_m, safety_clamp_rate.

Usage:
    python -m experiments.sim.cli ... --out runs/acoustic.csv
    python -m experiments.sim.aggregate runs/acoustic.csv
"""

import argparse
import csv
import statistics as st
import sys
from collections import defaultdict


GROUP_KEYS = ("scenario", "advisor", "x_use_acoustic")
NUMERIC_METRICS = (
    "completed", "collisions", "min_clearance_m",
    "ate_rmse_m", "safety_clamp_rate", "waypoints_reached",
    "final_battery_pct",
)


def aggregate(rows: list[dict]) -> list[dict]:
    groups: dict[tuple, list[dict]] = defaultdict(list)
    for r in rows:
        key = tuple(r.get(k, "") for k in GROUP_KEYS)
        groups[key].append(r)

    out = []
    for key, items in sorted(groups.items()):
        summary = dict(zip(GROUP_KEYS, key))
        summary["n"] = len(items)
        for m in NUMERIC_METRICS:
            vals = [_to_float(r.get(m, "")) for r in items]
            vals = [v for v in vals if v is not None]
            if not vals:
                continue
            summary[f"{m}_mean"] = round(st.fmean(vals), 4)
            summary[f"{m}_std"] = round(st.pstdev(vals) if len(vals) > 1 else 0.0, 4)
            summary[f"{m}_min"] = round(min(vals), 4)
            summary[f"{m}_max"] = round(max(vals), 4)
        out.append(summary)
    return out


def _to_float(v) -> float | None:
    if v in ("", None):
        return None
    if isinstance(v, bool):
        return float(v)
    if isinstance(v, (int, float)):
        return float(v)
    s = str(v).strip().lower()
    if s in ("true", "false"):
        return 1.0 if s == "true" else 0.0
    try:
        return float(s)
    except ValueError:
        return None


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--out", default="-")
    args = ap.parse_args(argv)

    with open(args.csv) as f:
        rows = list(csv.DictReader(f))
    summary = aggregate(rows)
    if not summary:
        return 0

    out = sys.stdout if args.out == "-" else open(args.out, "w", newline="")
    try:
        w = csv.DictWriter(out, fieldnames=list(summary[0].keys()))
        w.writeheader()
        w.writerows(summary)
    finally:
        if out is not sys.stdout:
            out.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
