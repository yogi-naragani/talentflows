"""PX4 SITL bridge node.

Bridges the drone stack (perception, fusion, MPC, advisor) onto the
PX4 message namespace consumed by the autopilot in SITL.

Outbound to PX4:
    /fmu/in/offboard_control_mode      px4_msgs/OffboardControlMode
    /fmu/in/trajectory_setpoint        px4_msgs/TrajectorySetpoint
    /fmu/in/vehicle_command            px4_msgs/VehicleCommand
    /fmu/in/vehicle_visual_odometry    px4_msgs/VehicleOdometry
                                       (so EKF2 can use VIO instead of GPS)

Inbound from PX4:
    /fmu/out/vehicle_local_position    px4_msgs/VehicleLocalPosition
    /fmu/out/vehicle_status            px4_msgs/VehicleStatus

The node:
1. Reads /mission/waypoint (PoseStamped) from the reasoning chain
2. Reads /odom/fused (Odometry) from the fusion node, which is the
   ORB-SLAM3 (or OpenCvOrb proxy) output once the VIO loop is closed
3. Publishes velocity-mode TrajectorySetpoint at 50 Hz toward the
   waypoint, and forwards /odom/fused as /fmu/in/vehicle_visual_odometry

Two failure-mode caveats up front:
- The px4_msgs Python package must be installed on the host. If it is
  missing, this module imports but the node refuses to start (logs and
  exits cleanly) so the rest of the launch file isn't blocked.
- PX4 EKF2 must be configured with EKF2_AID_MASK to accept external
  vision (see contracts/sim_setup.md). The bridge does not mutate
  parameters; do that from QGroundControl or `param set`.
"""

from __future__ import annotations
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                       QoSHistoryPolicy, QoSDurabilityPolicy)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


def _import_px4_msgs():
    try:
        from px4_msgs.msg import (OffboardControlMode,  # type: ignore
                                  TrajectorySetpoint,
                                  VehicleCommand, VehicleOdometry,
                                  VehicleLocalPosition, VehicleStatus)
        return {
            "OffboardControlMode": OffboardControlMode,
            "TrajectorySetpoint": TrajectorySetpoint,
            "VehicleCommand": VehicleCommand,
            "VehicleOdometry": VehicleOdometry,
            "VehicleLocalPosition": VehicleLocalPosition,
            "VehicleStatus": VehicleStatus,
        }
    except ImportError:
        return None


PX4_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class PX4BridgeNode(Node):
    def __init__(self):
        super().__init__("px4_bridge")
        msgs = _import_px4_msgs()
        if msgs is None:
            self.get_logger().error(
                "px4_msgs not installed. Skipping PX4 bridge -- the rest "
                "of the stack will run but PX4 will not see velocity "
                "commands or visual odometry. Install via:\n"
                "  cd ~/ros2_ws/src && git clone https://github.com/PX4/px4_msgs\n"
                "  colcon build --packages-select px4_msgs")
            self._enabled = False
            return
        self._enabled = True
        self._msgs = msgs
        self._target_xyz = (0.0, 0.0, 1.5)
        self._yaw = 0.0
        self._latest_odom: Odometry | None = None
        self._armed = False
        self._offboard = False

        self.declare_parameter("publish_rate_hz", 50.0)

        # Outbound to PX4
        self.ocm_pub = self.create_publisher(
            msgs["OffboardControlMode"], "/fmu/in/offboard_control_mode", PX4_QOS)
        self.tsp_pub = self.create_publisher(
            msgs["TrajectorySetpoint"], "/fmu/in/trajectory_setpoint", PX4_QOS)
        self.vcmd_pub = self.create_publisher(
            msgs["VehicleCommand"], "/fmu/in/vehicle_command", PX4_QOS)
        self.vio_pub = self.create_publisher(
            msgs["VehicleOdometry"], "/fmu/in/vehicle_visual_odometry", PX4_QOS)

        # Inbound from PX4
        self.create_subscription(
            msgs["VehicleStatus"], "/fmu/out/vehicle_status",
            self._on_status, PX4_QOS)
        self.create_subscription(
            msgs["VehicleLocalPosition"], "/fmu/out/vehicle_local_position",
            lambda _msg: None, PX4_QOS)

        # Inbound from our stack
        self.create_subscription(
            PoseStamped, "/mission/waypoint", self._on_waypoint, 10)
        self.create_subscription(
            Odometry, "/odom/fused", self._on_odom, 30)

        self.create_timer(
            1.0 / float(self.get_parameter("publish_rate_hz").value),
            self._tick)

    # ----- subs -----
    def _on_status(self, msg) -> None:
        self._armed = bool(msg.arming_state == 2)        # 2 = ARMED in PX4

    def _on_waypoint(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self._target_xyz = (float(p.x), float(p.y), float(p.z))

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_odom = msg
        self._publish_vio(msg)

    # ----- ticks -----
    def _tick(self) -> None:
        if not self._enabled:
            return
        msgs = self._msgs
        now_us = self.get_clock().now().nanoseconds // 1000

        ocm = msgs["OffboardControlMode"]()
        ocm.timestamp = now_us
        ocm.position = True
        ocm.velocity = False
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = False
        self.ocm_pub.publish(ocm)

        tsp = msgs["TrajectorySetpoint"]()
        tsp.timestamp = now_us
        # PX4 uses NED. Convert from our ENU world frame
        # (x_e, y_n, z_u) -> (x_n, y_e, -z_d) is the EKF2 convention
        # used in PX4 SITL.
        ex, ey, ez = self._target_xyz
        tsp.position = [float(ey), float(ex), float(-ez)]
        tsp.yaw = float(self._yaw)
        self.tsp_pub.publish(tsp)

    def _publish_vio(self, odom: Odometry) -> None:
        msgs = self._msgs
        v = msgs["VehicleOdometry"]()
        v.timestamp = self.get_clock().now().nanoseconds // 1000
        v.timestamp_sample = v.timestamp
        v.pose_frame = v.POSE_FRAME_NED
        p = odom.pose.pose.position
        # ENU -> NED on the position
        v.position = [float(p.y), float(p.x), float(-p.z)]
        q = odom.pose.pose.orientation
        v.q = [float(q.w), float(q.x), float(q.y), float(q.z)]
        # Velocity in body frame is left zero until fusion fills it in.
        self.vio_pub.publish(v)


def main():
    rclpy.init()
    node = PX4BridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
