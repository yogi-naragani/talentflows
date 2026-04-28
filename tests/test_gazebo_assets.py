"""Lint-style tests for the Gazebo assets.

We do not run gz-sim in CI, but we do verify that the assets that
gz-sim consumes parse cleanly: the SDF world is valid XML with the
expected top-level structure, and the launch file is valid Python
with the expected nodes and bridge mappings.
"""

from __future__ import annotations
import ast
import xml.etree.ElementTree as ET
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]


def test_warehouse_sdf_is_valid_xml():
    tree = ET.parse(REPO / "worlds" / "warehouse.sdf")
    root = tree.getroot()
    assert root.tag == "sdf", "root element must be <sdf>"
    world = root.find("world")
    assert world is not None, "missing <world>"
    assert world.get("name") == "warehouse"


def test_warehouse_sdf_has_drone_and_walls():
    root = ET.parse(REPO / "worlds" / "warehouse.sdf").getroot()
    names = {m.get("name") for m in root.iter("model")}
    # The wall ahead is what the acoustic backup must catch in the
    # corresponding Python scenario.
    assert "wall_ahead" in names
    assert "wall_left"  in names
    assert "wall_right" in names
    # The drone include resolves to a model named "drone".
    includes = list(root.iter("include"))
    drone_include = [i for i in includes
                     if (i.find("name") is not None
                         and i.find("name").text == "drone")]
    assert drone_include, "no <include> with <name>drone</name>"


def test_warehouse_sdf_has_camera_imu_and_rangefinder():
    root = ET.parse(REPO / "worlds" / "warehouse.sdf").getroot()
    sensors = list(root.iter("sensor"))
    types = {s.get("type") for s in sensors}
    assert "camera" in types
    assert "imu" in types
    assert "gpu_lidar" in types     # single-beam range emulation


def test_sim_launch_parses():
    src = (REPO / "launch" / "sim.launch.py").read_text()
    tree = ast.parse(src)
    # Must define generate_launch_description -- that is the entry
    # point ros2 launch looks for.
    fnames = {n.name for n in ast.walk(tree)
              if isinstance(n, ast.FunctionDef)}
    assert "generate_launch_description" in fnames


def test_sim_launch_brings_up_six_nodes():
    src = (REPO / "launch" / "sim.launch.py").read_text()
    expected = (
        "perception_node",
        "range_node",
        "fusion_node",
        "mpc_node",
        "reasoning_node",
        "acoustic_node",
    )
    for name in expected:
        assert name in src, f"sim.launch.py does not start {name}"


def test_sim_launch_has_bridge_for_required_topics():
    src = (REPO / "launch" / "sim.launch.py").read_text()
    # Each topic the nodes subscribe to must appear in the bridge
    # argument list, otherwise ros_gz_bridge will not forward the
    # gz-sim stream.
    required = (
        "/clock",
        "/camera/image_raw",
        "/imu",
        "/range/raw",
        "/cmd/motor",
    )
    for t in required:
        assert t in src, f"sim.launch.py missing bridge for {t}"


def test_deploy_pi5_launch_parses():
    p = REPO / "launch" / "deploy_pi5.launch.py"
    if not p.exists():
        return
    ast.parse(p.read_text())
