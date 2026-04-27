"""Fusion node: combines SLAM pose, IMU, and 1D range; runs scale
recovery; publishes a metric Odometry estimate."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from gps_denied_drone.fusion.scale_recovery import ScaleEstimator
from gps_denied_drone.fusion.state_estimator import StateEstimator


class FusionNode(Node):
    def __init__(self):
        super().__init__("fusion_node")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("slam_pose_topic", "/slam/pose")
        self.declare_parameter("range_topic", "/range/filtered")
        self.declare_parameter("odom_topic", "/odom/fused")

        self.scale = ScaleEstimator()
        self.estimator = StateEstimator()

        self.create_subscription(Imu, self.get_parameter("imu_topic").value,
                                 self.on_imu, 100)
        self.create_subscription(PoseWithCovarianceStamped,
                                 self.get_parameter("slam_pose_topic").value,
                                 self.on_slam, 30)
        self.create_subscription(Range, self.get_parameter("range_topic").value,
                                 self.on_range, 50)
        self.odom_pub = self.create_publisher(
            Odometry, self.get_parameter("odom_topic").value, 50)

    def on_imu(self, msg: Imu) -> None:
        pass  # estimator.predict(...)

    def on_slam(self, msg: PoseWithCovarianceStamped) -> None:
        pass  # estimator.update_slam_pose(...)

    def on_range(self, msg: Range) -> None:
        pass  # scale.push(...) ; estimator.update_range(...)


def main():
    rclpy.init()
    node = FusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
