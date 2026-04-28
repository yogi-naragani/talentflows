"""Synthetic acoustic backup for the Gazebo build.

gz-sim has no microphone sensor, so until a custom plugin (option 1 in
worlds/warehouse.sdf) lands, the acoustic_node has no /audio/raw stream
to consume. This node fills that gap by computing per-direction
proximity from the drone's current pose against the same world geometry
the Python harness uses (experiments.sim.world.World.scenario), and
publishes the same three topics the acoustic_node would.

It is *not* a drop-in for the production sensor: there is no DSP path,
no SNR floor that depends on RPM, and no microphone-array failure mode.
It exists so the acoustic ablation knob is meaningful in the Gazebo
build and so the fusion / MPC / reasoning loop receives a non-empty
backup signal.

Selecting between this and acoustic_node is a launch-file decision
(see launch/sim_fortress.launch.py).
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import Float32, Float32MultiArray, MultiArrayDimension

from experiments.sim.world import World
from gps_denied_drone.sensors.acoustic_sensor import DIRECTIONS


class SyntheticAcousticNode(Node):
    def __init__(self):
        super().__init__("synthetic_acoustic_node")
        self.declare_parameter("scenario", "corridor_white_wall")
        self.declare_parameter("odom_topic", "/model/x3_sensors/odometry")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("d_max_m", 4.0)
        self.declare_parameter("noise_seed", 0)

        self.world = World.scenario(self.get_parameter("scenario").value)
        self.d_max = float(self.get_parameter("d_max_m").value)
        self.rng = np.random.default_rng(
            int(self.get_parameter("noise_seed").value))

        self._pos = np.array([0.0, 0.0, 1.5])
        self._yaw = 0.0
        self._got_odom = False

        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value,
            self._on_odom, 50)
        self.prox_pub = self.create_publisher(
            Float32MultiArray, "/acoustic/proximity", 10)
        self.clear_pub = self.create_publisher(
            Range, "/acoustic/clearance", 10)
        self.conf_pub = self.create_publisher(
            Float32, "/acoustic/confidence", 10)

        self.create_timer(
            1.0 / float(self.get_parameter("publish_rate_hz").value),
            self.tick)

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # Yaw from quaternion (Z-up): atan2(2(wz+xy), 1-2(yy+zz)).
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._pos = np.array([p.x, p.y, p.z])
        self._yaw = math.atan2(siny_cosp, cosy_cosp)
        self._got_odom = True

    def tick(self) -> None:
        if not self._got_odom:
            return

        c, sn = float(np.cos(self._yaw)), float(np.sin(self._yaw))
        body_axes = {
            "front": np.array([ c,  sn, 0.0]),
            "back":  np.array([-c, -sn, 0.0]),
            "left":  np.array([-sn,  c, 0.0]),
            "right": np.array([ sn, -c, 0.0]),
            "up":    np.array([0.0, 0.0, 1.0]),
            "down":  np.array([0.0, 0.0, -1.0]),
        }
        proximity: dict[str, float] = {}
        clearances: dict[str, float] = {}
        for d, dir_w in body_axes.items():
            d_hit = self.world.distance_along(
                self._pos, dir_w, max_m=self.d_max + 1.0)
            prox_clean = max(0.0, 1.0 - d_hit / self.d_max)
            prox = float(np.clip(
                prox_clean + self.rng.normal(0, 0.04), 0.0, 1.0))
            proximity[d] = prox
            clearances[d] = float(d_hit + self.rng.normal(0, 0.05))

        worst = max(proximity, key=proximity.get)
        confidence = float(np.clip(
            proximity[worst] * 1.2 + self.rng.normal(0, 0.05), 0.0, 1.0))

        msg = Float32MultiArray()
        msg.layout.dim.append(MultiArrayDimension(
            label="dir", size=len(DIRECTIONS), stride=len(DIRECTIONS)))
        msg.data = [float(proximity[d]) for d in DIRECTIONS]
        self.prox_pub.publish(msg)

        rng = Range()
        rng.header.stamp = self.get_clock().now().to_msg()
        rng.radiation_type = Range.INFRARED
        rng.field_of_view = 6.28
        rng.min_range = 0.1
        rng.max_range = 10.0
        rng.range = float(clearances[worst])
        self.clear_pub.publish(rng)

        self.conf_pub.publish(Float32(data=confidence))


def main():
    rclpy.init()
    node = SyntheticAcousticNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
