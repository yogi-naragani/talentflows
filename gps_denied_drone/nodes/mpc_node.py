"""MPC node: subscribes to fused odometry and reference waypoints,
solves perception-aware MPC, publishes motor commands to Gazebo."""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray

from gps_denied_drone.control.mpc import QuadrotorMPC, MPCConfig


class MPCNode(Node):
    def __init__(self):
        super().__init__("mpc_node")
        self.declare_parameter("odom_topic", "/odom/fused")
        self.declare_parameter("waypoint_topic", "/mission/waypoint")
        self.declare_parameter("cmd_topic", "/cmd/motor")
        self.declare_parameter("rate_hz", 100.0)

        self.mpc = QuadrotorMPC(MPCConfig())
        self._x0 = None
        self._wp = None

        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value,
            self.on_odom, 50)
        self.create_subscription(
            PoseStamped, self.get_parameter("waypoint_topic").value,
            self.on_waypoint, 10)
        self.cmd_pub = self.create_publisher(
            Float32MultiArray, self.get_parameter("cmd_topic").value, 10)

        self.create_timer(
            1.0 / float(self.get_parameter("rate_hz").value), self.tick)

    def on_odom(self, msg: Odometry) -> None:
        self._x0 = msg

    def on_waypoint(self, msg: PoseStamped) -> None:
        self._wp = msg

    def tick(self) -> None:
        if self._x0 is None or self._wp is None:
            return
        # sol = self.mpc.solve(x0=..., x_ref=...)
        # self.cmd_pub.publish(Float32MultiArray(data=list(sol.u0)))


def main():
    rclpy.init()
    node = MPCNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
