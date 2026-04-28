"""Bring up Gazebo Fortress (ign gazebo / gz-sim 6) + bridge + drone nodes.

This is the Humble + Fortress sibling of sim.launch.py (which targets
Harmonic / gz-sim 8). It selects worlds/warehouse_fortress.sdf, sets
IGN_GAZEBO_RESOURCE_PATH so the local worlds/models/x3_sensors fork is
discoverable, and uses ignition.msgs.* type names in the bridge.
The same six ROS 2 nodes run -- they only consume ROS topics so they
do not care which gz-sim version is underneath.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("gps_denied_drone")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    world_arg = DeclareLaunchArgument(
        "world", default_value=PathJoinSubstitution(
            [pkg_share, "worlds", "warehouse_fortress.sdf"]))

    # Default true on Fortress: there is no microphone plugin in the
    # SDF, so the regular acoustic_node would have nothing to consume.
    # Set synthetic_acoustic:=false once a real mic plugin lands.
    synth_arg = DeclareLaunchArgument("synthetic_acoustic", default_value="true")

    # x3_sensors lives at worlds/models/x3_sensors and is referenced via
    # `model://x3_sensors`, so the parent worlds/models directory must
    # be on IGN_GAZEBO_RESOURCE_PATH.
    models_path = os.path.join(pkg_share, "worlds", "models")
    set_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH", value=models_path)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": [LaunchConfiguration("world"), " -r"]}.items(),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            "/range/raw@sensor_msgs/msg/Range[ignition.msgs.LaserScan",
            "/X3/gazebo/command/twist@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            "/model/x3_sensors/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
        ],
        output="screen",
    )

    nodes = [
        Node(package="gps_denied_drone", executable="perception_node", output="screen"),
        Node(package="gps_denied_drone", executable="range_node",      output="screen"),
        Node(package="gps_denied_drone", executable="fusion_node",     output="screen"),
        Node(package="gps_denied_drone", executable="mpc_node",        output="screen"),
        Node(package="gps_denied_drone", executable="reasoning_node",  output="screen"),
        Node(package="gps_denied_drone", executable="acoustic_node",
             output="screen",
             condition=UnlessCondition(LaunchConfiguration("synthetic_acoustic"))),
        Node(package="gps_denied_drone", executable="synthetic_acoustic_node",
             output="screen",
             condition=IfCondition(LaunchConfiguration("synthetic_acoustic"))),
    ]

    return LaunchDescription(
        [world_arg, synth_arg, set_resource_path, gz_sim, bridge, *nodes])
