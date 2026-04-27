"""Reasoning node: low-rate (1-5 Hz) advisory loop backed by Gemini Nano.
Aggregates state, asks the model for a mission-level adjustment, and
publishes a (possibly perturbed) waypoint plus a speed cap."""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32, Float32MultiArray

from gps_denied_drone.reasoning.gemini_nano import (
    GeminiNanoClient, StateSummary,
)
from gps_denied_drone.sensors.acoustic_sensor import DIRECTIONS


class ReasoningNode(Node):
    def __init__(self):
        super().__init__("reasoning_node")
        self.declare_parameter("rate_hz", 2.0)
        self.declare_parameter("waypoint_in",  "/mission/waypoint_raw")
        self.declare_parameter("waypoint_out", "/mission/waypoint")
        self.declare_parameter("speed_cap_topic", "/mission/speed_cap")

        self.client = GeminiNanoClient(backend=None)  # plug in AICore here
        self._odom = None
        self._range = None
        self._slam_ok = True
        self._wp_in = None
        self._ac_prox: dict[str, float] | None = None
        self._ac_clear: float | None = None
        self._ac_conf: float = 0.0

        self.create_subscription(Odometry, "/odom/fused", self._set_odom, 10)
        self.create_subscription(Range, "/range/filtered", self._set_range, 10)
        self.create_subscription(Bool, "/slam/tracking_ok", self._set_slam, 10)
        self.create_subscription(PoseStamped,
                                 self.get_parameter("waypoint_in").value,
                                 self._set_wp, 10)
        self.create_subscription(Float32MultiArray, "/acoustic/proximity",
                                 self._set_ac_prox, 10)
        self.create_subscription(Range, "/acoustic/clearance",
                                 self._set_ac_clear, 10)
        self.create_subscription(Float32, "/acoustic/confidence",
                                 self._set_ac_conf, 10)
        self.wp_pub = self.create_publisher(
            PoseStamped, self.get_parameter("waypoint_out").value, 10)
        self.cap_pub = self.create_publisher(
            Float32, self.get_parameter("speed_cap_topic").value, 10)

        self.create_timer(
            1.0 / float(self.get_parameter("rate_hz").value), self.tick)

    def _set_odom(self, msg): self._odom = msg
    def _set_range(self, msg): self._range = msg
    def _set_slam(self, msg): self._slam_ok = bool(msg.data)
    def _set_wp(self, msg): self._wp_in = msg

    def _set_ac_prox(self, msg):
        if len(msg.data) >= len(DIRECTIONS):
            self._ac_prox = {d: float(v) for d, v in zip(DIRECTIONS, msg.data)}

    def _set_ac_clear(self, msg):
        self._ac_clear = float(msg.range) if msg.range != float("inf") else None

    def _set_ac_conf(self, msg):
        self._ac_conf = float(msg.data)

    def tick(self) -> None:
        if self._odom is None or self._wp_in is None:
            return
        p = self._odom.pose.pose.position
        v = self._odom.twist.twist.linear
        wp = self._wp_in.pose.position
        summary = StateSummary(
            pos_xyz_m=(p.x, p.y, p.z),
            vel_xyz_mps=(v.x, v.y, v.z),
            yaw_deg=0.0,
            battery_pct=100.0,
            slam_tracking_ok=self._slam_ok,
            slam_inliers=0,
            range_m=float(self._range.range) if self._range else None,
            waypoint_xyz_m=(wp.x, wp.y, wp.z),
            acoustic_proximity=self._ac_prox,
            acoustic_clearance_m=self._ac_clear,
            acoustic_confidence=self._ac_conf,
        )
        action = self.client.decide(summary)

        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.pose.position.x = wp.x + action.waypoint_delta_m[0]
        out.pose.position.y = wp.y + action.waypoint_delta_m[1]
        out.pose.position.z = max(0.5, wp.z + action.waypoint_delta_m[2])
        self.wp_pub.publish(out)
        self.cap_pub.publish(Float32(data=min(5.0, action.speed_cap_mps)))


def main():
    rclpy.init()
    node = ReasoningNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
