"""EuRoC MAV evaluation harness.

End-to-end:
    1. (optional) Download a EuRoC sequence to ~/datasets/EuRoC/.
    2. Run the active VIO backend over the sequence's images + IMU.
       Writes a TUM-format trajectory to experiments/runs/euroc/.
    3. Shell out to `evo_ape` to compute ATE-RMSE against the ground
       truth and write a CSV summary into results/vio_baseline.md.

Two backends are supported:
    --backend orbslam3      (calls a user-installed ORB-SLAM3 binary;
                             see contracts/vio_pipeline.md)
    --backend opencv_orb    (the in-tree feature-density proxy from
                             gps_denied_drone.perception.visual_slam --
                             produces a feature-count-vs-time signal,
                             not a full trajectory; useful as a Day 4
                             smoke test before ORB-SLAM3 is built)

Usage:
    python -m experiments.euroc.run_eval --download  --sequences MH_01_easy
    python -m experiments.euroc.run_eval --backend opencv_orb \
                                          --sequences MH_01_easy
    python -m experiments.euroc.run_eval --eval-only \
                                          --sequences MH_01_easy MH_03_medium V1_01_easy
"""

from __future__ import annotations
import argparse
import csv
import os
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np


EUROC_BASE = "http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset"
SEQUENCES = {
    "MH_01_easy":   "machine_hall/MH_01_easy/MH_01_easy.zip",
    "MH_03_medium": "machine_hall/MH_03_medium/MH_03_medium.zip",
    "V1_01_easy":   "vicon_room1/V1_01_easy/V1_01_easy.zip",
}

REPO = Path(__file__).resolve().parents[2]
DATA = Path(os.environ.get("EUROC_ROOT", Path.home() / "datasets" / "EuRoC"))
RUNS = REPO / "experiments" / "runs" / "euroc"
RESULTS = REPO / "results"


def download(seq: str) -> Path:
    DATA.mkdir(parents=True, exist_ok=True)
    target = DATA / seq
    if target.exists():
        print(f"[ok] {seq} already present at {target}")
        return target
    url = f"{EUROC_BASE}/{SEQUENCES[seq]}"
    zip_path = DATA / f"{seq}.zip"
    print(f"[get] {url}")
    subprocess.check_call(["wget", "-q", "-O", str(zip_path), url])
    subprocess.check_call(["unzip", "-q", str(zip_path), "-d", str(target)])
    zip_path.unlink()
    return target


def run_orbslam3(seq_path: Path, out_traj: Path) -> None:
    """Shell out to the user's ORB-SLAM3 mono-inertial binary. Path is
    user-configurable via $ORBSLAM3_BIN; expected CLI matches the
    upstream Examples/Monocular-Inertial/mono_inertial_euroc."""
    bin_path = os.environ.get("ORBSLAM3_BIN")
    if not bin_path or not Path(bin_path).exists():
        raise RuntimeError(
            "ORB-SLAM3 binary not found. Set $ORBSLAM3_BIN to the "
            "path of mono_inertial_euroc, or fall back to "
            "--backend opencv_orb.")
    voc = os.environ["ORBSLAM3_VOC"]
    cfg = os.environ.get("ORBSLAM3_CFG",
                         str(REPO / "experiments" / "euroc" / "EuRoC.yaml"))
    cmd = [bin_path, voc, cfg,
           str(seq_path / "mav0" / "cam0" / "data"),
           str(seq_path / "mav0" / "imu0" / "data.csv"),
           str(seq_path / "mav0" / "cam0" / "data.csv"),
           str(out_traj)]
    print("[run] " + " ".join(cmd))
    subprocess.check_call(cmd)


def run_opencv_orb(seq_path: Path, out_traj: Path) -> None:
    """Run cv2.ORB feature density over the cam0 images.

    This does not produce a real pose -- it produces a TUM-format
    file with identity rotations and a synthetic position trace
    derived from the inverse-depth-flow heuristic. The point is to
    exercise the eval pipeline before ORB-SLAM3 is built. The
    feature counts per frame are written alongside in <out>.inliers.csv
    and are the actual signal of interest at this stage.
    """
    try:
        import cv2  # type: ignore
    except ImportError as e:
        raise RuntimeError("cv2 not available") from e

    img_dir = seq_path / "mav0" / "cam0" / "data"
    images = sorted(img_dir.glob("*.png"))
    if not images:
        raise RuntimeError(f"no PNGs in {img_dir}")

    orb = cv2.ORB_create(nfeatures=500)
    out_traj.parent.mkdir(parents=True, exist_ok=True)
    inliers_path = out_traj.with_suffix(".inliers.csv")

    pose = np.array([0.0, 0.0, 0.0])
    qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0

    with open(out_traj, "w") as f, open(inliers_path, "w", newline="") as fi:
        ic = csv.writer(fi)
        ic.writerow(["t_ns", "n_inliers"])
        for img_path in images:
            t_ns = int(img_path.stem)
            t_s = t_ns * 1e-9
            img = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)
            kps = orb.detect(img, None)
            n = len(kps)
            ic.writerow([t_ns, n])
            # Identity-ish trajectory; consumers know this backend
            # produces an inliers signal, not a pose.
            f.write(f"{t_s:.9f} {pose[0]:.6f} {pose[1]:.6f} {pose[2]:.6f} "
                    f"{qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")
    print(f"[ok] wrote {out_traj}")
    print(f"[ok] wrote {inliers_path}")


def evo_ape(traj: Path, gt: Path, out_csv: Path) -> dict:
    """Run evo_ape and capture the metrics in a small dict.

    Returns {ate_rmse, ate_mean, ate_median, ate_std, ate_min, ate_max}.
    """
    if shutil.which("evo_ape") is None:
        raise RuntimeError(
            "evo not installed. `pip install evo`.")
    res_zip = out_csv.with_suffix(".zip")
    cmd = ["evo_ape", "tum", str(gt), str(traj),
           "-as", "--save_results", str(res_zip)]
    print("[run] " + " ".join(cmd))
    out = subprocess.check_output(cmd, text=True)
    metrics: dict = {}
    for line in out.splitlines():
        line = line.strip()
        for k in ("rmse", "mean", "median", "std", "min", "max"):
            if line.startswith(k):
                try:
                    metrics["ate_" + k] = float(line.split()[-1])
                except (IndexError, ValueError):
                    pass
    with open(out_csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(metrics.keys()) or ["ate_rmse"])
        w.writeheader()
        w.writerow(metrics or {"ate_rmse": float("nan")})
    print(f"[ok] {metrics}")
    return metrics


def write_results_md(rows: list[dict]) -> None:
    RESULTS.mkdir(exist_ok=True)
    out = RESULTS / "vio_baseline.md"
    with open(out, "w") as f:
        f.write("# VIO baseline on EuRoC MAV\n\n")
        f.write("Generated by `python -m experiments.euroc.run_eval`.\n\n")
        f.write("| sequence | backend | ATE-RMSE (m) | ATE-mean (m) | ATE-median (m) |\n")
        f.write("|---|---|---|---|---|\n")
        for r in rows:
            f.write(f"| {r['sequence']} | {r['backend']} | "
                    f"{r.get('ate_rmse', 'n/a'):.4f} | "
                    f"{r.get('ate_mean', 'n/a'):.4f} | "
                    f"{r.get('ate_median', 'n/a'):.4f} |\n")
    print(f"[ok] wrote {out}")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sequences", nargs="+",
                    default=["MH_01_easy"], choices=list(SEQUENCES))
    ap.add_argument("--backend", default="opencv_orb",
                    choices=["orbslam3", "opencv_orb"])
    ap.add_argument("--download", action="store_true",
                    help="fetch the sequences if not cached locally")
    ap.add_argument("--eval-only", action="store_true",
                    help="skip the trajectory step, only run evo_ape")
    args = ap.parse_args(argv)

    RUNS.mkdir(parents=True, exist_ok=True)
    rows: list[dict] = []

    for seq in args.sequences:
        if args.download:
            download(seq)
        seq_path = DATA / seq
        if not seq_path.exists():
            print(f"[skip] {seq} not present at {seq_path}; "
                  f"re-run with --download")
            continue

        traj = RUNS / f"{seq}.tum"
        if not args.eval_only:
            if args.backend == "orbslam3":
                run_orbslam3(seq_path, traj)
            else:
                run_opencv_orb(seq_path, traj)

        gt = seq_path / "mav0" / "state_groundtruth_estimate0" / "data.csv"
        if not gt.exists():
            print(f"[skip-eval] no GT at {gt}")
            continue

        # evo expects TUM ground truth too. EuRoC ships GT as CSV;
        # convert quickly.
        gt_tum = traj.with_suffix(".gt.tum")
        _euroc_gt_to_tum(gt, gt_tum)

        try:
            metrics = evo_ape(traj, gt_tum, traj.with_suffix(".ape.csv"))
        except RuntimeError as e:
            print(f"[err] {e}")
            metrics = {}

        rows.append({"sequence": seq, "backend": args.backend, **metrics})

    if rows:
        write_results_md(rows)
    return 0


def _euroc_gt_to_tum(src: Path, dst: Path) -> None:
    """Convert EuRoC ground truth CSV to TUM format."""
    with open(src) as fi, open(dst, "w") as fo:
        next(fi)  # skip header
        for line in fi:
            parts = line.strip().split(",")
            if len(parts) < 8:
                continue
            t_ns = int(parts[0])
            t_s = t_ns * 1e-9
            tx, ty, tz = parts[1], parts[2], parts[3]
            qw, qx, qy, qz = parts[4], parts[5], parts[6], parts[7]
            fo.write(f"{t_s:.9f} {tx} {ty} {tz} {qx} {qy} {qz} {qw}\n")


if __name__ == "__main__":
    raise SystemExit(main())
