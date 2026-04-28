"""Paired statistical test for the acoustic on/off comparison.

We pair trials by (scenario, advisor, seed) and compare the matched
acoustic-on vs acoustic-off outcomes with a Wilcoxon signed-rank test.
This is more rigorous than the unpaired mean comparison reported in
the aggregator and is what the paper should cite for significance.

Stdlib only -- the Wilcoxon signed-rank test is short to implement
and we avoid pulling SciPy into a CI-light requirements list.

Usage:
    python -m experiments.sim.paired_stats experiments/runs/acoustic_combined.csv
"""

from __future__ import annotations
import argparse
import csv
import math
import sys
from collections import defaultdict


METRICS_LOWER_IS_BETTER = ("collisions", "ate_rmse_m", "safety_clamp_rate")
METRICS_HIGHER_IS_BETTER = ("min_clearance_m", "completed",
                            "waypoints_reached")


def wilcoxon_signed_rank(diffs: list[float]) -> tuple[float, float]:
    """Two-sided Wilcoxon signed-rank test.

    Returns (W_statistic, two_sided_p). Uses the normal approximation
    with tie-correction (valid for n >= ~10). Discards zero
    differences (Wilcoxon convention).
    """
    nonzero = [d for d in diffs if d != 0.0]
    n = len(nonzero)
    if n == 0:
        return 0.0, 1.0
    abs_d = sorted([abs(d) for d in nonzero])
    # Average ranks across ties
    ranks: list[float] = [0.0] * n
    i = 0
    while i < n:
        j = i
        while j + 1 < n and abs_d[j + 1] == abs_d[i]:
            j += 1
        avg_rank = (i + j) / 2.0 + 1.0
        for k in range(i, j + 1):
            ranks[k] = avg_rank
        i = j + 1
    # Map back to signed differences
    signs = [1 if d > 0 else -1 for d in nonzero]
    abs_to_rank: dict[float, list[float]] = defaultdict(list)
    for r, a in zip(ranks, abs_d):
        abs_to_rank[a].append(r)
    w_plus = 0.0
    w_minus = 0.0
    used = defaultdict(int)
    for d, s in zip(nonzero, signs):
        a = abs(d)
        rank = abs_to_rank[a][used[a]]
        used[a] += 1
        if s > 0:
            w_plus += rank
        else:
            w_minus += rank
    W = min(w_plus, w_minus)
    # Normal approximation
    mean = n * (n + 1) / 4.0
    # Tie correction
    tie_term = 0.0
    counts = defaultdict(int)
    for a in abs_d:
        counts[a] += 1
    for c in counts.values():
        if c > 1:
            tie_term += (c ** 3 - c)
    var = n * (n + 1) * (2 * n + 1) / 24.0 - tie_term / 48.0
    if var <= 0:
        return W, 1.0
    z = (W - mean) / math.sqrt(var)
    p = 2 * (1 - _phi(abs(z)))
    return W, p


def _phi(z: float) -> float:
    return 0.5 * (1 + math.erf(z / math.sqrt(2)))


def _to_float(v) -> float | None:
    if v in ("", None):
        return None
    s = str(v).strip().lower()
    if s in ("true", "false"):
        return 1.0 if s == "true" else 0.0
    try:
        return float(s)
    except ValueError:
        return None


def paired_compare(rows: list[dict],
                   metrics: tuple[str, ...] = METRICS_LOWER_IS_BETTER
                   + METRICS_HIGHER_IS_BETTER) -> list[dict]:
    """Group rows into matched (advisor, seed, scenario) pairs across
    x_use_acoustic = True / False, then run Wilcoxon on each metric.
    Returns one summary row per metric."""
    by_key: dict[tuple, dict] = defaultdict(dict)
    for r in rows:
        key = (r.get("scenario", ""), r.get("advisor", ""),
               r.get("seed", ""))
        flag = str(r.get("x_use_acoustic", "")).strip().lower()
        if flag in ("true", "false"):
            by_key[key][flag] = r

    results = []
    for m in metrics:
        diffs: list[float] = []
        for r in by_key.values():
            on = r.get("true")
            off = r.get("false")
            if on is None or off is None:
                continue
            v_on = _to_float(on.get(m))
            v_off = _to_float(off.get(m))
            if v_on is None or v_off is None:
                continue
            # Sign convention: positive diff means acoustic ON is
            # *better* on this metric.
            if m in METRICS_LOWER_IS_BETTER:
                diffs.append(v_off - v_on)
            else:
                diffs.append(v_on - v_off)
        if not diffs:
            continue
        W, p = wilcoxon_signed_rank(diffs)
        results.append({
            "metric": m,
            "n_pairs": len(diffs),
            "median_diff_on_minus_off": round(_median(diffs), 4),
            "wilcoxon_W": round(W, 4),
            "p_two_sided": round(p, 6),
            "direction": "lower_is_better" if m in METRICS_LOWER_IS_BETTER
                         else "higher_is_better",
        })
    return results


def _median(xs: list[float]) -> float:
    s = sorted(xs)
    n = len(s)
    if n % 2:
        return s[n // 2]
    return 0.5 * (s[n // 2 - 1] + s[n // 2])


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--out", default="-")
    args = ap.parse_args(argv)

    with open(args.csv) as f:
        rows = list(csv.DictReader(f))
    summary = paired_compare(rows)
    if not summary:
        print("no matched pairs", file=sys.stderr)
        return 1
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
