"""Acoustic proximity node.

Subscribes to a multi-channel audio topic (bridged from Gazebo via a
microphone plugin or a real audio device) and motor RPM, runs the
passive acoustic proximity pipeline, and publishes:

  /acoustic/proximity   std_msgs/Float32MultiArray  (6 directions)
  /acoustic/clearance   sensor_msgs/Range           (worst direction)
  /acoustic/confidence  std_msgs/Float32

Designed as a *backup* to visual SLAM: downstream consumers (fusion,
MPC, reasoning) should weight it inversely with SLAM confidence and use
its clearance term as a hard MPC inequality constraint when it is
above threshold confidence.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32, Float32MultiArray, MultiArrayDimension

from gps_denied_drone.sensors.acoustic_sensor import (
    AcousticProximitySensor, DIRECTIONS,
)


class AcousticNode(Node):
    def __init__(self):
        super().__init__("acoustic_node")
        self.declare_parameter("audio_topic", "/audio/raw")
        self.declare_parameter("rpm_topic", "/motor/rpm")
        self.declare_parameter("sample_rate_hz", 48000)
        self.declare_parameter("publish_rate_hz", 20.0)

        self.sensor = AcousticProximitySensor(
            sample_rate_hz=int(self.get_parameter("sample_rate_hz").value))
        self._audio_buf: np.ndarray | None = None
        self._rpm: np.ndarray = np.zeros(4, dtype=np.float32)

        self.create_subscription(
            Float32MultiArray, self.get_parameter("audio_topic").value,
            self._on_audio, 10)
        self.create_subscription(
            Float32MultiArray, self.get_parameter("rpm_topic").value,
            self._on_rpm, 10)

        self.prox_pub = self.create_publisher(
            Float32MultiArray, "/acoustic/proximity", 10)
        self.clear_pub = self.create_publisher(
            Range, "/acoustic/clearance", 10)
        self.conf_pub = self.create_publisher(
            Float32, "/acoustic/confidence", 10)

        self.create_timer(
            1.0 / float(self.get_parameter("publish_rate_hz").value),
            self.tick)

    def _on_audio(self, msg: Float32MultiArray) -> None:
        if not msg.layout.dim:
            return
        ch = msg.layout.dim[0].size
        n = len(msg.data) // ch
        self._audio_buf = np.asarray(msg.data, dtype=np.float32).reshape(ch, n)

    def _on_rpm(self, msg: Float32MultiArray) -> None:
        self._rpm = np.asarray(msg.data, dtype=np.float32)

    def tick(self) -> None:
        if self._audio_buf is None:
            return
        t_ns = self.get_clock().now().nanoseconds
        try:
            r = self.sensor.process(t_ns, self._audio_buf, self._rpm)
        except NotImplementedError:
            r = AcousticProximitySensor.empty(t_ns)

        prox = Float32MultiArray()
        prox.layout.dim.append(
            MultiArrayDimension(label="dir", size=len(DIRECTIONS), stride=len(DIRECTIONS)))
        prox.data = [float(r.proximity[d]) for d in DIRECTIONS]
        self.prox_pub.publish(prox)

        worst_dir = max(r.proximity, key=r.proximity.get) if r.proximity else None
        rng = Range()
        rng.header.stamp = self.get_clock().now().to_msg()
        rng.radiation_type = Range.INFRARED  # placeholder; no acoustic enum
        rng.field_of_view = 6.28
        rng.min_range = 0.1
        rng.max_range = 10.0
        c = r.clearance_m.get(worst_dir, float("nan")) if worst_dir else float("nan")
        rng.range = float(c) if np.isfinite(c) else float("inf")
        self.clear_pub.publish(rng)

        self.conf_pub.publish(Float32(data=float(r.confidence)))


def main():
    rclpy.init()
    node = AcousticNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
