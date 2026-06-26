#!/usr/bin/env python3
"""回环期间保护 Nav2：取消在途导航，避免 bt_navigator 在 map→odom 跳变时 SIGSEGV。

Foxy 上 bt_navigator 崩溃后 lifecycle 不会自动重新激活；本节点在 action server
短暂不可用时触发 lifecycle_manager RESET+STARTUP。
"""

from __future__ import annotations

import time
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from rclpy.action import ActionClient
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

_ACTIVE_STATUSES = (
    GoalStatus.STATUS_ACCEPTED,
    GoalStatus.STATUS_EXECUTING,
    GoalStatus.STATUS_CANCELING,
)

_INFO_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

_STATUS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

_CMD_STARTUP = 0
_CMD_RESET = 3
_CMD_SHUTDOWN = 4


class NavLoopGuard(Node):
    def __init__(self) -> None:
        super().__init__('nav_loop_guard')
        self.declare_parameter('info_topic', '/rtabmap/info')
        self.declare_parameter('info_topic_fallback', '/info')
        self.declare_parameter('nav_action_name', 'navigate_to_pose')
        self.declare_parameter('nav_action_status_topic', '/navigate_to_pose/_action/status')
        self.declare_parameter('lifecycle_manager_name', 'lifecycle_manager_navigation')
        self.declare_parameter('loop_cancel_cooldown_sec', 2.0)
        self.declare_parameter('nav_recovery_check_sec', 2.0)
        self.declare_parameter('nav_down_grace_sec', 12.0)
        self.declare_parameter('nav_recovery_cooldown_sec', 30.0)
        self.declare_parameter('nav_recovery_settle_sec', 3.0)

        self._info_topic = str(self.get_parameter('info_topic').value)
        self._info_fallback = str(self.get_parameter('info_topic_fallback').value)
        self._nav_action_name = str(self.get_parameter('nav_action_name').value).strip('/')
        self._nav_status_topic = str(self.get_parameter('nav_action_status_topic').value)
        self._lifecycle_manager = str(self.get_parameter('lifecycle_manager_name').value)
        self._loop_cooldown = max(0.5, float(self.get_parameter('loop_cancel_cooldown_sec').value))
        self._recovery_check = max(1.0, float(self.get_parameter('nav_recovery_check_sec').value))
        self._nav_down_grace = max(2.0, float(self.get_parameter('nav_down_grace_sec').value))
        self._recovery_cooldown = max(
            15.0, float(self.get_parameter('nav_recovery_cooldown_sec').value),
        )
        self._recovery_settle = max(
            1.0, float(self.get_parameter('nav_recovery_settle_sec').value),
        )

        self._last_loop_id = 0
        self._last_proximity_id = 0
        self._last_loop_cancel_at = 0.0
        self._nav_goal_active = False
        self._nav_was_ready = False
        self._nav_down_since: Optional[float] = None
        self._lifecycle_reset_inflight = False
        self._last_lifecycle_reset_at = 0.0
        self._recovery_startup_timer = None

        self.get_logger().set_level(LoggingSeverity.INFO)

        self._nav_client: Optional[ActionClient] = None
        try:
            from nav2_msgs.action import NavigateToPose

            self._action_type = NavigateToPose
            self._nav_client = ActionClient(self, NavigateToPose, self._nav_action_name)
        except Exception as exc:
            self.get_logger().warn(f'[nav_loop_guard] NavigateToPose 不可用：{exc}')

        self._manage_nodes_client = None
        try:
            from nav2_msgs.srv import ManageLifecycleNodes

            self._manage_nodes_type = ManageLifecycleNodes
            service_name = f'/{self._lifecycle_manager}/manage_nodes'
            self._manage_nodes_client = self.create_client(ManageLifecycleNodes, service_name)
        except Exception as exc:
            self.get_logger().warn(f'[nav_loop_guard] ManageLifecycleNodes 不可用：{exc}')

        self._clear_costmap_type = None
        self._clear_global_client = None
        self._clear_local_client = None
        try:
            from nav2_msgs.srv import ClearEntireCostmap

            self._clear_costmap_type = ClearEntireCostmap
            self._clear_global_client = self.create_client(
                ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap',
            )
            self._clear_local_client = self.create_client(
                ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap',
            )
        except Exception as exc:
            self.get_logger().warn(f'[nav_loop_guard] ClearEntireCostmap 不可用：{exc}')

        try:
            from rtabmap_msgs.msg import Info as RtabmapInfo

            self.create_subscription(
                RtabmapInfo, self._info_topic, self._on_rtabmap_info, _INFO_QOS,
            )
            if self._info_fallback and self._info_fallback != self._info_topic:
                self.create_subscription(
                    RtabmapInfo, self._info_fallback, self._on_rtabmap_info, _INFO_QOS,
                )
        except ImportError as exc:
            self.get_logger().error(f'[nav_loop_guard] rtabmap_msgs 不可用：{exc}')

        self.create_subscription(
            GoalStatusArray, self._nav_status_topic, self._on_nav_status, _STATUS_QOS,
        )
        self.create_timer(self._recovery_check, self._check_nav_server_health)

        self.get_logger().info(
            '[nav_loop_guard] 已启动：回环时取消在途导航；bt_navigator 崩溃后尝试 lifecycle 恢复。'
        )

    def _on_nav_status(self, msg: GoalStatusArray) -> None:
        active = False
        for entry in msg.status_list:
            if int(entry.status) in _ACTIVE_STATUSES:
                active = True
                break
        self._nav_goal_active = active

    def _on_rtabmap_info(self, msg) -> None:
        loop_id = int(getattr(msg, 'loop_closure_id', 0) or 0)
        proximity_id = int(getattr(msg, 'proximity_detection_id', 0) or 0)
        loop_event = loop_id > self._last_loop_id
        if loop_id > self._last_loop_id:
            self._last_loop_id = loop_id
        if proximity_id > self._last_proximity_id:
            self._last_proximity_id = proximity_id
        if not loop_event or not self._nav_goal_active:
            return
        now = time.monotonic()
        if now - self._last_loop_cancel_at < self._loop_cooldown:
            return
        self._last_loop_cancel_at = now
        self.get_logger().warn(
            f'[nav_loop_guard] 检测到回环（loop_closure_id={loop_id} proximity={proximity_id}），'
            '取消在途导航以避免 bt_navigator 在 map→odom 跳变时崩溃；'
            '请等待约 2s 后重新下发 2D Goal。'
        )
        self._cancel_active_navigation()
        self._clear_costmaps()

    def _cancel_active_navigation(self) -> None:
        if self._nav_client is None:
            return
        try:
            cancel_future = self._nav_client.cancel_all_goals_async()
            cancel_future.add_done_callback(self._on_cancel_done)
        except Exception as exc:
            self.get_logger().warn(f'[nav_loop_guard] 取消导航目标失败：{exc}')

    def _on_cancel_done(self, future) -> None:
        try:
            future.result()
            self.get_logger().info('[nav_loop_guard] 已请求取消全部 navigate_to_pose 目标。')
        except Exception as exc:
            self.get_logger().warn(f'[nav_loop_guard] 取消导航回调失败：{exc}')

    def _clear_costmaps(self) -> None:
        for label, client in (
            ('global', self._clear_global_client),
            ('local', self._clear_local_client),
        ):
            if client is None or self._clear_costmap_type is None or not client.service_is_ready():
                continue
            request = self._clear_costmap_type.Request()
            future = client.call_async(request)
            future.add_done_callback(
                lambda fut, name=label: self._on_clear_costmap_done(name, fut),
            )

    @staticmethod
    def _on_clear_costmap_done(label: str, future) -> None:
        try:
            future.result()
        except Exception:
            pass

    def _check_nav_server_health(self) -> None:
        if self._nav_client is None:
            return
        ready = self._nav_client.server_is_ready()
        now = time.monotonic()
        if ready:
            self._nav_was_ready = True
            self._nav_down_since = None
            return
        if not self._nav_was_ready:
            return
        if self._nav_down_since is None:
            self._nav_down_since = now
            return
        if now - self._nav_down_since < self._nav_down_grace:
            return
        if self._lifecycle_reset_inflight:
            return
        if now - self._last_lifecycle_reset_at < self._recovery_cooldown:
            return
        self.get_logger().warn(
            f'[nav_loop_guard] navigate_to_pose 服务不可用 ≥{self._nav_down_grace:.0f}s，'
            '尝试 SHUTDOWN→STARTUP 恢复 Nav2（常见于 bt_navigator SIGSEGV 后）。'
        )
        self._recover_navigation_lifecycle()

    def _cancel_recovery_startup_timer(self) -> None:
        if self._recovery_startup_timer is not None:
            self._recovery_startup_timer.cancel()
            self._recovery_startup_timer.destroy()
            self._recovery_startup_timer = None

    def _recover_navigation_lifecycle(self) -> None:
        if self._manage_nodes_client is None:
            return
        if not self._manage_nodes_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn('[nav_loop_guard] lifecycle manage_nodes 服务不可用。')
            return
        self._lifecycle_reset_inflight = True
        self._last_lifecycle_reset_at = time.monotonic()
        self._cancel_recovery_startup_timer()
        self._cancel_active_navigation()
        self._send_lifecycle_command(_CMD_SHUTDOWN, self._on_shutdown_done)

    def _send_lifecycle_command(self, command: int, done_cb) -> None:
        request = self._manage_nodes_type.Request()
        request.command = command
        future = self._manage_nodes_client.call_async(request)
        future.add_done_callback(done_cb)

    def _on_shutdown_done(self, future) -> None:
        try:
            response = future.result()
            if not response.success:
                self.get_logger().warn(
                    f'[nav_loop_guard] lifecycle SHUTDOWN 失败：{response}；仍尝试 STARTUP。'
                )
            else:
                self.get_logger().info(
                    f'[nav_loop_guard] lifecycle SHUTDOWN 完成，'
                    f'{self._recovery_settle:.0f}s 后 STARTUP。'
                )
        except Exception as exc:
            self.get_logger().warn(
                f'[nav_loop_guard] lifecycle SHUTDOWN 异常：{exc}；仍尝试 STARTUP。'
            )
        self._cancel_recovery_startup_timer()
        self._recovery_startup_timer = self.create_timer(
            self._recovery_settle, self._on_recovery_startup_timer,
        )

    def _on_recovery_startup_timer(self) -> None:
        self._cancel_recovery_startup_timer()
        self._send_lifecycle_command(_CMD_STARTUP, self._on_lifecycle_startup_done)

    def _on_lifecycle_startup_done(self, future) -> None:
        self._lifecycle_reset_inflight = False
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    '[nav_loop_guard] Nav2 lifecycle STARTUP 完成，可继续探索/下发导航目标。'
                )
                self._nav_down_since = None
                return
            self.get_logger().warn(
                f'[nav_loop_guard] lifecycle STARTUP 失败：{response}；尝试 RESET+STARTUP。'
            )
        except Exception as exc:
            self.get_logger().warn(
                f'[nav_loop_guard] lifecycle STARTUP 异常：{exc}；尝试 RESET+STARTUP。'
            )
        self._send_lifecycle_command(_CMD_RESET, self._on_lifecycle_reset_done)

    def _on_lifecycle_reset_done(self, future) -> None:
        try:
            response = future.result()
            if not response.success:
                self.get_logger().warn(
                    f'[nav_loop_guard] lifecycle RESET 失败：{response}；仍尝试 STARTUP。'
                )
        except Exception as exc:
            self.get_logger().warn(f'[nav_loop_guard] lifecycle RESET 异常：{exc}')
        self._cancel_recovery_startup_timer()
        self._recovery_startup_timer = self.create_timer(
            self._recovery_settle, self._on_recovery_startup_retry_timer,
        )

    def _on_recovery_startup_retry_timer(self) -> None:
        self._cancel_recovery_startup_timer()
        self._send_lifecycle_command(_CMD_STARTUP, self._on_lifecycle_startup_retry_done)

    def _on_lifecycle_startup_retry_done(self, future) -> None:
        self._lifecycle_reset_inflight = False
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    '[nav_loop_guard] Nav2 lifecycle RESET+STARTUP 完成，可继续探索。'
                )
                self._nav_down_since = None
            else:
                self.get_logger().warn(f'[nav_loop_guard] lifecycle STARTUP 失败：{response}')
        except Exception as exc:
            self.get_logger().warn(f'[nav_loop_guard] lifecycle STARTUP 异常：{exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavLoopGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()