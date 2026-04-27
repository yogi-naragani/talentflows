"""Reasoning node: low-rate (0.5-2 Hz) on-device SLM advisor.

Pipeline per tick:
  1. HealthMonitor builds a structured observation from the latest
     ROS messages (no policy logic here).
  2. GeminiNanoClient (Gemini Nano on Pixel, or Gemma 3 1B-class via
     llama.cpp on Pi 5) emits a ReasoningAction.
  3. SafetySupervisor clamps the action to a verifiably safe envelope
     and records what changed for log audit.
  4. Node publishes:
        /mission/waypoint        PoseStamped (adjusted)
        /mission/speed_cap       Float32
        /mpc/weight_overrides    Float32MultiArray (Q_pos,Q_vel,Q_att,w_perception)
        /mpc/sensor_trust        Float32MultiArray (slam, range, acoustic)
        /mission/mode            String
        /mission/safety_clamped  Bool

Hard rule: the SLM is advisory; the SafetySupervisor and the MPC's
own constraints are the safety authority.
"""

import json
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32, Float32MultiArray, String

from gps_denied_drone.reasoning.gemini_nano import (
    GeminiNanoClient, VALID_WEIGHTS, VALID_SENSORS,
)
from gps_denied_drone.reasoning.rule_tree import RuleTreeAdvisor
from gps_denied_drone.reasoning.monitor import HealthMonitor, signals_to_json_dict
from gps_denied_drone.reasoning.safety import SafetySupervisor
from gps_denied_drone.sensors.acoustic_sensor import DIRECTIONS


class ReasoningNode(Node):
    def __init__(self):
        super().__init__("reasoning_node")
        self.declare_parameter("rate_hz", 1.0)
        self.declare_parameter("waypoint_in",  "/mission/waypoint_raw")
        self.declare_parameter("waypoint_out", "/mission/waypoint")
        self.declare_parameter("slm_backend", "stub")
        # Central ablation knob: "slm" or "rule_tree". The reasoning
        # node intentionally accepts both so paper experiments can swap
        # the advisor while keeping the rest of the stack identical.
        self.declare_parameter("advisor", "slm")

        advisor = str(self.get_parameter("advisor").value).lower()
        if advisor == "rule_tree":
            self.client = RuleTreeAdvisor()
            self.get_logger().info("advisor: rule_tree (baseline)")
        else:
            self.client = GeminiNanoClient(backend=None)
            self.get_logger().info("advisor: slm (Gemini Nano-class)")
        self.monitor = HealthMonitor(window_s=5.0)
        self.safety = SafetySupervisor()

        # Latest signals
        self._odom: Odometry | None = None
        self._range: Range | None = None
        self._slam_ok = True
        self._slam_inliers = 0
        self._wp_in: PoseStamped | None = None
        self._ac_prox: dict | None = None
        self._ac_clear: float | None = None
        self._ac_conf: float = 0.0
        self._battery_pct = 100.0

        # Subs
        self.create_subscription(Odometry, "/odom/fused", self._set_odom, 10)
        self.create_subscription(Range, "/range/filtered", self._set_range, 10)
        self.create_subscription(Bool, "/slam/tracking_ok", self._set_slam_ok, 10)
        self.create_subscription(PoseStamped,
                                 self.get_parameter("waypoint_in").value,
                                 self._set_wp, 10)
        self.create_subscription(Float32MultiArray, "/acoustic/proximity",
                                 self._set_ac_prox, 10)
        self.create_subscription(Range, "/acoustic/clearance",
                                 self._set_ac_clear, 10)
        self.create_subscription(Float32, "/acoustic/confidence",
                                 self._set_ac_conf, 10)
        self.create_subscription(Float32, "/battery/percent",
                                 self._set_battery, 10)

        # Pubs
        self.wp_pub        = self.create_publisher(PoseStamped, self.get_parameter("waypoint_out").value, 10)
        self.cap_pub       = self.create_publisher(Float32, "/mission/speed_cap", 10)
        self.weights_pub   = self.create_publisher(Float32MultiArray, "/mpc/weight_overrides", 10)
        self.trust_pub     = self.create_publisher(Float32MultiArray, "/mpc/sensor_trust", 10)
        self.mode_pub      = self.create_publisher(String, "/mission/mode", 10)
        self.clamped_pub   = self.create_publisher(Bool, "/mission/safety_clamped", 10)

        self.create_timer(
            1.0 / float(self.get_parameter("rate_hz").value), self.tick)

    # --- subs ---
    def _set_odom(self, msg): self._odom = msg
    def _set_range(self, msg):
        self._range = msg
        self.monitor.push_range(valid=msg.range != float("inf"))
    def _set_slam_ok(self, msg):
        self._slam_ok = bool(msg.data)
        self.monitor.push_slam(self._slam_ok, self._slam_inliers)
    def _set_wp(self, msg): self._wp_in = msg
    def _set_ac_prox(self, msg):
        if len(msg.data) >= len(DIRECTIONS):
            self._ac_prox = {d: float(v) for d, v in zip(DIRECTIONS, msg.data)}
    def _set_ac_clear(self, msg):
        self._ac_clear = float(msg.range) if msg.range != float("inf") else None
    def _set_ac_conf(self, msg): self._ac_conf = float(msg.data)
    def _set_battery(self, msg): self._battery_pct = float(msg.data)

    # --- tick ---
    def tick(self) -> None:
        if self._odom is None or self._wp_in is None:
            return

        p = self._odom.pose.pose.position
        v = self._odom.twist.twist.linear
        wp = self._wp_in.pose.position

        signals = self.monitor.observe(
            pos=(p.x, p.y, p.z),
            vel=(v.x, v.y, v.z),
            yaw_deg=0.0,
            slam_tracking_ok=self._slam_ok,
            slam_inliers=self._slam_inliers,
            range_m=float(self._range.range) if self._range else None,
            ac_prox=self._ac_prox,
            ac_clear=self._ac_clear,
            ac_conf=self._ac_conf,
            battery_pct=self._battery_pct,
            imu_vibration=0.0,
            motor_saturation_ratio=0.0,
            waypoint=(wp.x, wp.y, wp.z),
        )

        observation = signals_to_json_dict(signals)
        action = self.client.decide(observation)
        safe_action, report = self.safety.apply(
            action,
            current_pos=(p.x, p.y, p.z),
            battery_pct=self._battery_pct,
        )

        self.monitor.set_mode(safe_action.mode)
        self._publish(safe_action, report, base_wp=(wp.x, wp.y, wp.z))

        if report.clamped:
            self.get_logger().warn(
                "safety clamped: " + "; ".join(report.reasons))
        self.get_logger().debug(
            f"action: {json.dumps(_action_to_dict(safe_action))}")

    def _publish(self, a, report, base_wp):
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.pose.position.x = base_wp[0] + a.waypoint_delta_m[0]
        out.pose.position.y = base_wp[1] + a.waypoint_delta_m[1]
        out.pose.position.z = base_wp[2] + a.waypoint_delta_m[2]
        self.wp_pub.publish(out)

        self.cap_pub.publish(Float32(data=float(a.speed_cap_mps)))
        self.mode_pub.publish(String(data=a.mode))
        self.clamped_pub.publish(Bool(data=bool(report.clamped)))

        w = a.mpc_weights or {}
        self.weights_pub.publish(Float32MultiArray(
            data=[float(w.get(k, 1.0)) for k in VALID_WEIGHTS]))
        t = a.sensor_trust or {}
        self.trust_pub.publish(Float32MultiArray(
            data=[float(t.get(k, 1.0)) for k in VALID_SENSORS]))


def _action_to_dict(a):
    return {
        "mode": a.mode,
        "waypoint_delta_m": list(a.waypoint_delta_m),
        "speed_cap_mps": a.speed_cap_mps,
        "yaw_strategy": a.yaw_strategy,
        "mpc_weights": a.mpc_weights,
        "sensor_trust": a.sensor_trust,
        "replan": a.replan,
        "rationale": a.rationale,
        "confidence": a.confidence,
    }


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
