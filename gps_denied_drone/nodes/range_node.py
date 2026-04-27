"""Range node: subscribes to a Gazebo 1D rangefinder, applies the median
outlier filter from gps_denied_drone.sensors, and republishes a clean
range stream."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

from gps_denied_drone.sensors.range_sensor import RangeSensor, RangeReading


class RangeNode(Node):
    def __init__(self):
        super().__init__("range_node")
        self.declare_parameter("input_topic", "/range/raw")
        self.declare_parameter("output_topic", "/range/filtered")
        self.declare_parameter("max_range_m", 40.0)

        self.rf = RangeSensor(
            max_range_m=self.get_parameter("max_range_m").value)
        self.create_subscription(
            Range, self.get_parameter("input_topic").value,
            self.on_range, 50)
        self.pub = self.create_publisher(
            Range, self.get_parameter("output_topic").value, 50)

    def on_range(self, msg: Range) -> None:
        t_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        out = self.rf.push(RangeReading(t_ns, float(msg.range), valid=True))
        if out.valid:
            self.pub.publish(msg)


def main():
    rclpy.init()
    node = RangeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
