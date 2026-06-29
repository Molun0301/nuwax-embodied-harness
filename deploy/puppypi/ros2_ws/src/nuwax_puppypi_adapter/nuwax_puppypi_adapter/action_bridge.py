"""Bridge nuwax MQTT/ROS action JSON to PuppyPi velocity and Nav2 goals."""

from __future__ import annotations

import json
import math
from typing import Any, Dict, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from puppy_control_msgs.msg import Velocity
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class PuppyPiActionBridge(Node):
    def __init__(self) -> None:
        super().__init__("puppypi_action_bridge")

        self.declare_parameter("action_topic", "/robot/incoming_action")
        self.declare_parameter("velocity_topic", "/puppy_control/velocity")
        self.declare_parameter("nav2_action_name", "/navigate_to_pose")
        self.declare_parameter("max_x_cm_s", 12.0)
        self.declare_parameter("max_yaw_rate", 0.6)
        self.declare_parameter("map_frame", "map")

        action_topic = str(self.get_parameter("action_topic").value)
        velocity_topic = str(self.get_parameter("velocity_topic").value)
        nav2_action_name = str(self.get_parameter("nav2_action_name").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.max_x = float(self.get_parameter("max_x_cm_s").value)
        self.max_yaw = float(self.get_parameter("max_yaw_rate").value)

        self.velocity_publisher = self.create_publisher(Velocity, velocity_topic, 10)
        self.action_subscription = self.create_subscription(
            String,
            action_topic,
            self.on_action,
            10,
        )
        self.nav_client = ActionClient(self, NavigateToPose, nav2_action_name)
        self.get_logger().info(
            "Listening on %s; velocity=%s; nav2=%s",
            action_topic,
            velocity_topic,
            nav2_action_name,
        )

    def publish_stop(self) -> None:
        stop = Velocity()
        stop.x = 0.0
        stop.y = 0.0
        stop.yaw_rate = 0.0
        self.velocity_publisher.publish(stop)

    def publish_velocity(self, linear_x_m_s: float, yaw_rate: float) -> None:
        output = Velocity()
        output.x = clamp(linear_x_m_s * 100.0, -self.max_x, self.max_x)
        output.y = 0.0
        output.yaw_rate = clamp(yaw_rate, -self.max_yaw, self.max_yaw)
        self.velocity_publisher.publish(output)

    def on_action(self, message: String) -> None:
        payload = self._parse_payload(message.data)
        if payload is None:
            return

        action_type = str(payload.get("action") or payload.get("type") or "").strip().lower()
        if action_type in {"stop", "stop_motion", "halt"}:
            self.publish_stop()
            self.get_logger().info("Stop command applied")
            return

        if action_type in {"move", "velocity", "cmd_vel"}:
            linear_x = float(payload.get("linear_x", payload.get("x", 0.0)))
            yaw_rate = float(payload.get("yaw_rate", payload.get("angular_z", 0.0)))
            self.publish_velocity(linear_x, yaw_rate)
            self.get_logger().info("Velocity command: x=%.2f m/s yaw=%.2f", linear_x, yaw_rate)
            return

        if action_type in {"navigate", "navigate_to_pose", "goto"}:
            self._send_nav_goal(payload)
            return

        self.get_logger().warning("Unsupported action payload: %s", payload)

    def _parse_payload(self, raw: str) -> Optional[Dict[str, Any]]:
        text = str(raw or "").strip()
        if not text:
            return None
        try:
            parsed = json.loads(text)
        except json.JSONDecodeError:
            self.get_logger().warning("Invalid JSON action: %s", text[:200])
            return None
        if not isinstance(parsed, dict):
            self.get_logger().warning("Action payload must be a JSON object")
            return None
        return parsed

    def _send_nav_goal(self, payload: Dict[str, Any]) -> None:
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 action server not available")
            return

        pose = payload.get("pose") if isinstance(payload.get("pose"), dict) else payload
        x = float(pose.get("x", 0.0))
        y = float(pose.get("y", 0.0))
        yaw = float(pose.get("yaw", pose.get("theta", 0.0)))

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 goal rejected")
            return
        self.get_logger().info("Nav2 goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Nav2 goal succeeded")
        else:
            self.get_logger().warning("Nav2 goal finished with status=%s", status)


def main() -> None:
    rclpy.init()
    node = PuppyPiActionBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
