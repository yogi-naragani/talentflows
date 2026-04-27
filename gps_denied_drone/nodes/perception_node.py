"""Perception node: subscribes to a Gazebo camera, runs visual SLAM,
publishes pose estimates and a tracking-health diagnostic."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge

from gps_denied_drone.perception.visual_slam import VisualSLAM


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("pose_topic", "/slam/pose")
        self.declare_parameter("health_topic", "/slam/tracking_ok")

        self.bridge = CvBridge()
        self.slam = VisualSLAM(config={})

        self.create_subscription(
            Image, self.get_parameter("image_topic").value,
            self.on_image, 10)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter("pose_topic").value, 10)
        self.health_pub = self.create_publisher(
            Bool, self.get_parameter("health_topic").value, 10)

    def on_image(self, msg: Image) -> None:
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        t_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        try:
            pose = self.slam.process_frame(t_ns, cv_img)
        except NotImplementedError:
            return
        self.pose_pub.publish(_to_pose_msg(pose, msg.header))
        self.health_pub.publish(Bool(data=pose.tracking_ok))


def _to_pose_msg(pose, header):
    out = PoseWithCovarianceStamped()
    out.header = header
    # T_wc -> position + quaternion: left to the SLAM backend integration.
    return out


def main():
    rclpy.init()
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
