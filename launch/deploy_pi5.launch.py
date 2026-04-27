"""Hardware deployment on Raspberry Pi 5 + AI HAT (Hailo).

Brings up the five drone nodes against real or HIL topics. Does NOT
launch gz-sim — Gazebo does not run usefully in real time on the Pi 5.
For HIL, run gz-sim on a workstation on the same network and let DDS
discover the topics; for pure flight, the topics come from real
drivers (camera / IMU / range / mic).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("gps_denied_drone")
    mpc_yaml = PathJoinSubstitution([pkg_share, "config", "mpc.yaml"])

    use_npu = DeclareLaunchArgument(
        "use_hailo", default_value="false",
        description="Route ORB / scene classifier through Hailo NPU.")
    slm_backend = DeclareLaunchArgument(
        "slm_backend", default_value="llama_cpp_gemma3_1b",
        description="On-device SLM substitute for Gemini Nano on Pi 5.")

    nodes = [
        Node(package="gps_denied_drone", executable="perception_node",
             output="screen",
             parameters=[{"use_hailo": LaunchConfiguration("use_hailo")}]),
        Node(package="gps_denied_drone", executable="range_node",
             output="screen"),
        Node(package="gps_denied_drone", executable="fusion_node",
             output="screen"),
        Node(package="gps_denied_drone", executable="mpc_node",
             output="screen", parameters=[mpc_yaml]),
        Node(package="gps_denied_drone", executable="reasoning_node",
             output="screen",
             parameters=[{"slm_backend": LaunchConfiguration("slm_backend")}]),
        Node(package="gps_denied_drone", executable="acoustic_node",
             output="screen",
             parameters=[{"use_hailo": LaunchConfiguration("use_hailo")}]),
    ]

    return LaunchDescription([use_npu, slm_backend, *nodes])
