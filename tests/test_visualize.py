"""Smoke test that the visualizer produces a non-empty PNG."""

import os
import tempfile

import matplotlib
matplotlib.use("Agg")  # headless

from experiments.sim import visualize


def test_visualize_writes_png(tmp_path):
    out = tmp_path / "traj.png"
    visualize.render(
        scenario="corridor_white_wall",
        seeds=[0, 1, 2],
        out_path=str(out),
        duration_s=10.0,
    )
    assert out.exists()
    assert out.stat().st_size > 5000
