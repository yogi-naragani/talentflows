from setuptools import find_packages, setup
from glob import glob
import os

package_name = "gps_denied_drone"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test", "experiments", "paper"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "docs"), glob("docs/*.md")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.sdf")),
    ],
    install_requires=["setuptools", "numpy", "scipy", "opencv-python", "casadi"],
    zip_safe=True,
    maintainer="Anonymous",
    maintainer_email="dev@example.com",
    description="GPS-denied drone nav stack with on-device LLM supervision.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "perception_node = gps_denied_drone.nodes.perception_node:main",
            "range_node      = gps_denied_drone.nodes.range_node:main",
            "fusion_node     = gps_denied_drone.nodes.fusion_node:main",
            "mpc_node        = gps_denied_drone.nodes.mpc_node:main",
            "reasoning_node  = gps_denied_drone.nodes.reasoning_node:main",
            "acoustic_node   = gps_denied_drone.nodes.acoustic_node:main",
        ],
    },
)
