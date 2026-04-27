"""Bring up Gazebo (gz-sim), the ros_gz bridge, and the five drone nodes."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("gps_denied_drone")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    world_arg = DeclareLaunchArgument(
        "world", default_value=PathJoinSubstitution(
            [pkg_share, "worlds", "warehouse.sdf"]))

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": [LaunchConfiguration("world"), " -r"]}.items(),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/range/raw@sensor_msgs/msg/Range[gz.msgs.LaserScan",
            "/cmd/motor@std_msgs/msg/Float32MultiArray]gz.msgs.Actuators",
            "/audio/raw@std_msgs/msg/Float32MultiArray[gz.msgs.Float_V",
            "/motor/rpm@std_msgs/msg/Float32MultiArray[gz.msgs.Float_V",
        ],
        output="screen",
    )

    nodes = [
        Node(package="gps_denied_drone", executable="perception_node", output="screen"),
        Node(package="gps_denied_drone", executable="range_node",      output="screen"),
        Node(package="gps_denied_drone", executable="fusion_node",     output="screen"),
        Node(package="gps_denied_drone", executable="mpc_node",        output="screen"),
        Node(package="gps_denied_drone", executable="reasoning_node",  output="screen"),
        Node(package="gps_denied_drone", executable="acoustic_node",   output="screen"),
    ]

    return LaunchDescription([world_arg, gz_sim, bridge, *nodes])
