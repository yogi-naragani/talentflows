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


# --- Fortress (Humble + gz-sim 6) variant ----------------------------

def test_warehouse_fortress_sdf_is_valid_xml():
    tree = ET.parse(REPO / "worlds" / "warehouse_fortress.sdf")
    root = tree.getroot()
    assert root.tag == "sdf"
    world = root.find("world")
    assert world is not None
    assert world.get("name") == "warehouse"


def test_warehouse_fortress_has_walls_and_drone_include():
    root = ET.parse(REPO / "worlds" / "warehouse_fortress.sdf").getroot()
    model_names = {m.get("name") for m in root.iter("model")}
    for n in ("wall_ahead", "wall_left", "wall_right"):
        assert n in model_names, f"missing {n}"
    # The Fortress world includes the local x3_sensors fork, not the
    # Fuel X3 (which fails to attach sibling sensors on Fortress).
    includes = [i.find("uri").text.strip() for i in root.iter("include")
                if i.find("uri") is not None]
    assert any("x3_sensors" in u for u in includes), \
        "warehouse_fortress.sdf must include model://x3_sensors"


def test_x3_sensors_model_has_camera_imu_and_lidar():
    sdf = REPO / "worlds" / "models" / "x3_sensors" / "model.sdf"
    root = ET.parse(sdf).getroot()
    types = {s.get("type") for s in root.iter("sensor")}
    assert "camera" in types
    assert "imu" in types
    assert "gpu_lidar" in types


def test_sim_fortress_launch_parses_and_starts_nodes():
    src = (REPO / "launch" / "sim_fortress.launch.py").read_text()
    tree = ast.parse(src)
    fnames = {n.name for n in ast.walk(tree)
              if isinstance(n, ast.FunctionDef)}
    assert "generate_launch_description" in fnames
    for name in ("perception_node", "range_node", "fusion_node",
                 "mpc_node", "reasoning_node", "synthetic_acoustic_node"):
        assert name in src, f"sim_fortress.launch.py missing {name}"


def test_bag_to_csv_module_imports():
    # Cheap import smoke test; full bag conversion is exercised
    # manually against a recorded run (see docs/gazebo_run.md).
    import importlib
    importlib.import_module("experiments.gazebo.bag_to_csv")
