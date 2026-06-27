#!/usr/bin/env python3
"""将 RViz /goal_pose 转为 Nav2 navigate_to_pose action（Foxy GoalTool 备用链）。"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

class RvizGoalBridge(Node):
    def __init__(self) -> None:
        super().__init__('rviz_goal_bridge')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('nav_action_name', 'navigate_to_pose')
        self.declare_parameter('goal_frame', 'map')

        self._goal_topic = str(self.get_parameter('goal_topic').value)
        self._action_name = str(self.get_parameter('nav_action_name').value).strip('/')
        self._goal_frame = str(self.get_parameter('goal_frame').value).strip('/') or 'map'
        self._goal_handle = None
        self._nav_client: Optional[ActionClient] = None
        self.get_logger().set_level(LoggingSeverity.INFO)

        try:
            from nav2_msgs.action import NavigateToPose

            self._action_type = NavigateToPose
            self._nav_client = ActionClient(self, NavigateToPose, self._action_name)
        except Exception as exc:
            self.get_logger().error(f'[rviz_goal_bridge] NavigateToPose 不可用：{exc}')
            raise

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(PoseStamped, self._goal_topic, self._on_goal_pose, qos)
        self.create_timer(5.0, self._log_nav_server_status)
        self.get_logger().info(
            f'[rviz_goal_bridge] {self._goal_topic} -> /{self._action_name} '
            f'(frame={self._goal_frame})'
        )

    def _nav2_ready_for_goal(self) -> bool:
        return self._nav_client is not None and self._nav_client.server_is_ready()

    def _log_nav_server_status(self) -> None:
        if self._nav2_ready_for_goal():
            return
        self.get_logger().warn(
            '[rviz_goal_bridge] navigate_to_pose action server 未就绪；'
            '请确认 Nav2 已启动约 15s。'
        )

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        if self._nav_client is None:
            return
        if not self._nav2_ready_for_goal():
            self.get_logger().warn(
                f'[rviz_goal_bridge] 收到 {self._goal_topic}，但 Nav2 action server 未就绪，目标已丢弃。'
            )
            return

        goal = self._action_type.Goal()
        goal.pose = msg
        if not str(goal.pose.header.frame_id or '').strip():
            goal.pose.header.frame_id = self._goal_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        if self._goal_handle is not None and self._goal_handle.accepted:
            try:
                cancel_future = self._goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(lambda _f: None)
            except Exception:
                pass

        send_future = self._nav_client.send_goal_async(
            goal,
            feedback_callback=self._on_feedback,
        )
        send_future.add_done_callback(self._on_goal_response)
        px = float(goal.pose.pose.position.x)
        py = float(goal.pose.pose.position.y)
        self.get_logger().info(
            f'[rviz_goal_bridge] 已转发导航目标 ({px:.2f}, {py:.2f}) frame={goal.pose.header.frame_id}'
        )

    def _on_goal_response(self, future) -> None:
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().warn('[rviz_goal_bridge] Nav2 拒绝了导航目标（常见：map TF 未就绪或代价地图为空）。')
            return
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        dist = getattr(feedback, 'distance_remaining', None)
        if dist is None:
            return
        try:
            dist_f = float(dist)
        except (TypeError, ValueError):
            return
        if math.isfinite(dist_f):
            self.get_logger().info(f'[rviz_goal_bridge] 距目标剩余 {dist_f:.2f} m', throttle_duration_sec=3.0)

    def _on_result(self, future) -> None:
        try:
            result = future.result().result
            status = future.result().status
        except Exception as exc:
            self.get_logger().warn(f'[rviz_goal_bridge] 导航结果异常：{exc}')
            return
        self.get_logger().info(f'[rviz_goal_bridge] 导航结束 status={status} result={result}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RvizGoalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()