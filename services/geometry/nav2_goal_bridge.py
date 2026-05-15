#!/usr/bin/env python3
"""Nav2 Goal Bridge — topic-to-action 桥接节点。

解决问题：平台进程使用 rmw_fastrtps_cpp，Nav2 进程使用 rmw_cyclonedds_cpp。
ROS2 Foxy 下跨 RMW 的 action 通信存在 bug（goal position 字段被清零）。
pub/sub 跨 RMW 正常工作，因此本节点：
  - 订阅 /nuwax/nav2_goal (PoseStamped) — 接收平台下发的导航目标
  - 订阅 /nuwax/nav2_cancel (Empty) — 接收取消请求
  - 发布 /nuwax/nav2_result (String) — 报告导航结果（JSON）
  - 发布 /nuwax/nav2_feedback (String) — 报告导航反馈（JSON）
  - 内部使用 NavigateToPose action client 与 bt_navigator 通信（同 RMW）

本脚本由 start_go2_slam_nav2.sh 在 CycloneDDS 环境中启动。
"""
import json
import sys
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty, String
from nav2_msgs.action import NavigateToPose


class Nav2GoalBridge(Node):
    def __init__(self):
        super().__init__("nav2_goal_bridge")
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)

        self._action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._goal_handle = None
        self._lock = threading.Lock()

        self.create_subscription(PoseStamped, "/nuwax/nav2_goal", self._on_goal, qos)
        self.create_subscription(Empty, "/nuwax/nav2_cancel", self._on_cancel, qos)
        self._result_pub = self.create_publisher(String, "/nuwax/nav2_result", qos)
        self._feedback_pub = self.create_publisher(String, "/nuwax/nav2_feedback", qos)
        self.get_logger().info("Nav2 Goal Bridge 已启动，等待 /nuwax/nav2_goal")

    def _on_goal(self, msg: PoseStamped):
        self.get_logger().info(
            "收到导航目标 (%.3f, %.3f, %.3f) frame=%s",
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
            msg.header.frame_id,
        )
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 action server 不可用")
            self._publish_result("failed", "action_server_unavailable")
            return
        with self._lock:
            if self._goal_handle is not None:
                self.get_logger().info("取消旧目标，发送新目标")
                self._goal_handle.cancel_goal_async()
                self._goal_handle = None

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg
        future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._on_feedback
        )
        future.add_done_callback(self._on_goal_response)

    def _on_cancel(self, msg: Empty):
        with self._lock:
            if self._goal_handle is not None:
                self.get_logger().info("收到取消请求")
                self._goal_handle.cancel_goal_async()
                self._goal_handle = None
                self._publish_result("cancelled", "user_cancel")

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 拒绝目标")
            self._publish_result("failed", "goal_rejected")
            return
        self.get_logger().info("Nav2 接受目标")
        with self._lock:
            self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result()
        status = result.status
        with self._lock:
            self._goal_handle = None
        if status == 4:  # STATUS_SUCCEEDED
            self._publish_result("succeeded", "")
        elif status == 5:  # STATUS_CANCELED
            self._publish_result("cancelled", "nav2_cancelled")
        else:
            self._publish_result("failed", f"nav2_status_{status}")

    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        dist = fb.distance_remaining if hasattr(fb, "distance_remaining") else -1.0
        msg = String()
        msg.data = json.dumps({"distance_remaining": dist})
        self._feedback_pub.publish(msg)

    def _publish_result(self, status: str, detail: str):
        msg = String()
        msg.data = json.dumps({"status": status, "detail": detail})
        self._result_pub.publish(msg)
        self.get_logger().info("导航结果: %s %s", status, detail)


def main():
    rclpy.init()
    node = Nav2GoalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
