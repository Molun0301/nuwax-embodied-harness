#!/usr/bin/env python3
"""周期性检查 go2 栈关键就绪项，中文日志提示导航无反应的常见原因。"""

from __future__ import annotations

import time

import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.node import Node


class Go2StackHealth(Node):
    def __init__(self) -> None:
        super().__init__('go2_stack_health')
        self.declare_parameter('check_interval_sec', 10.0)
        self.declare_parameter('odom_topic', '/go2/odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'go2_base_link')
        self.declare_parameter('nav_action_name', 'navigate_to_pose')

        self._interval = max(5.0, float(self.get_parameter('check_interval_sec').value))
        self._odom_topic = str(self.get_parameter('odom_topic').value)
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._nav_action = str(self.get_parameter('nav_action_name').value).strip('/')
        self._last_odom_at = 0.0
        self._odom_seen = False

        self.get_logger().set_level(LoggingSeverity.INFO)
        self.create_subscription(
            __import__('nav_msgs.msg', fromlist=['Odometry']).Odometry,
            self._odom_topic,
            self._on_odom,
            10,
        )
        self.create_timer(self._interval, self._check)
        self.get_logger().info('[go2_stack_health] 导航就绪检查已启动。')

    def _on_odom(self, _msg) -> None:
        self._odom_seen = True
        self._last_odom_at = time.monotonic()

    def _check(self) -> None:
        issues = []
        if not self._odom_seen or (time.monotonic() - self._last_odom_at) > 3.0:
            issues.append(
                f'无 {self._odom_topic}（机器狗未接网线/未开机/未切运动模式？）'
            )

        try:
            from tf2_ros import Buffer, TransformListener

            if not hasattr(self, '_tf_buffer'):
                self._tf_buffer = Buffer()
                self._tf_listener = TransformListener(self._tf_buffer, self)
            if not self._tf_buffer.can_transform(
                self._map_frame,
                self._base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.3),
            ):
                issues.append(
                    f'TF {self._map_frame}->{self._base_frame} 不可用（请遥控慢走几步建图）'
                )
        except Exception:
            issues.append(f'TF {self._map_frame}->{self._base_frame} 查询失败')

        try:
            from nav2_msgs.action import NavigateToPose
            from rclpy.action import ActionClient

            from go2_slam_nav._nav2_readiness import nav2_action_ready

            if not hasattr(self, '_nav_client'):
                self._nav_client = ActionClient(self, NavigateToPose, self._nav_action)
            ready, reason = nav2_action_ready(
                self,
                self._nav_client,
                stabilize_sec=3.0,
                ready_since_attr='_go2_stack_health_nav2_ready_since',
            )
            if not ready:
                issues.append(reason)
        except Exception:
            issues.append('Nav2 action 客户端初始化失败')

        if issues:
            self.get_logger().warn(
                '[go2_stack_health] 导航未就绪：%s' % '；'.join(issues)
            )
        else:
            self.get_logger().info(
                '[go2_stack_health] 导航就绪：odom + map TF + Nav2 action 均正常，可下发 2D Goal。',
                throttle_duration_sec=30.0,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Go2StackHealth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()