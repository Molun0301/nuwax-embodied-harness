"""Drive forward and rotate to prime SLAM map before explore_lite starts."""

import math
import time
from enum import Enum, auto
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class _Phase(Enum):
    WAIT_ODOM = auto()
    FORWARD = auto()
    ROTATE = auto()
    WAIT_NAV2 = auto()
    DONE = auto()


class ExplorePreroll(Node):
    def __init__(self):
        super().__init__('explore_preroll')

        self.declare_parameter('distance_m', 1.0)
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('scan_rotation_rad', 6.28)
        self.declare_parameter('angular_speed', 0.3)
        self.declare_parameter('odom_topic', '/go2/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_wait_timeout', 30.0)
        self.declare_parameter('max_duration', 60.0)
        self.declare_parameter('cmd_vel_rate', 20.0)
        self.declare_parameter('nav2_wait_timeout', 90.0)
        self.declare_parameter('nav2_stabilize_sec', 8.0)
        self.declare_parameter('nav2_check_period_sec', 1.0)
        self.declare_parameter('nav2_action_name', 'navigate_to_pose')
        self.declare_parameter('follow_path_action_name', 'follow_path')

        self._distance_m = float(self.get_parameter('distance_m').value)
        self._linear_speed = float(self.get_parameter('linear_speed').value)
        self._scan_rotation_rad = float(self.get_parameter('scan_rotation_rad').value)
        self._angular_speed = float(self.get_parameter('angular_speed').value)
        self._odom_topic = self.get_parameter('odom_topic').value
        self._cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self._odom_wait_timeout = float(self.get_parameter('odom_wait_timeout').value)
        self._max_duration = float(self.get_parameter('max_duration').value)
        self._nav2_wait_timeout = float(self.get_parameter('nav2_wait_timeout').value)
        self._nav2_stabilize_sec = float(self.get_parameter('nav2_stabilize_sec').value)
        self._nav2_check_period = max(
            0.5, float(self.get_parameter('nav2_check_period_sec').value),
        )
        nav2_action = str(self.get_parameter('nav2_action_name').value)
        follow_path_action = str(self.get_parameter('follow_path_action_name').value)
        cmd_rate = float(self.get_parameter('cmd_vel_rate').value)

        self._phase = _Phase.WAIT_ODOM
        self._start_x = None
        self._start_y = None
        self._start_yaw = None
        self._latest_odom = None
        self._move_start = None
        self._rotation_accum = 0.0
        self._prev_yaw = None
        self._odom_deadline = (
            self.get_clock().now()
            + rclpy.duration.Duration(seconds=self._odom_wait_timeout)
        )
        self._nav2_deadline = None
        self._nav2_ready_since: Optional[float] = None
        self._last_nav2_check_mono = 0.0

        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self.create_subscription(Odometry, self._odom_topic, self._odom_cb, odom_qos)
        self.create_timer(1.0 / cmd_rate, self._tick)

        self._nav_client = None
        self._follow_path_client = None
        try:
            from nav2_msgs.action import FollowPath, NavigateToPose
            self._nav_client = ActionClient(self, NavigateToPose, nav2_action)
            self._follow_path_client = ActionClient(self, FollowPath, follow_path_action)
        except ImportError:
            pass

        scan_part = (
            f'，原地旋转 {math.degrees(self._scan_rotation_rad):.0f}°'
            if self._scan_rotation_rad > 0.1 else ''
        )
        self.get_logger().info(
            f'探索预滚动：前进 {self._distance_m:.2f} m，'
            f'速度 {self._linear_speed:.2f} m/s{scan_part}，然后等待 Nav2'
        )

    @property
    def done(self) -> bool:
        return self._phase == _Phase.DONE

    @staticmethod
    def _yaw_from_odom(msg: Odometry) -> float:
        q = msg.pose.pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _odom_cb(self, msg: Odometry):
        self._latest_odom = msg
        if self._start_x is not None:
            return
        self._start_x = msg.pose.pose.position.x
        self._start_y = msg.pose.pose.position.y
        self._start_yaw = self._yaw_from_odom(msg)
        self._prev_yaw = self._start_yaw
        self._move_start = self.get_clock().now()
        self._phase = _Phase.FORWARD
        self.get_logger().info(
            f'里程计基准点：({self._start_x:.3f}, {self._start_y:.3f})'
        )

    def _distance_traveled(self) -> float:
        if self._latest_odom is None or self._start_x is None:
            return 0.0
        pose = self._latest_odom.pose.pose.position
        return math.hypot(pose.x - self._start_x, pose.y - self._start_y)

    def _publish_stop(self):
        stop = Twist()
        for _ in range(5):
            self._cmd_pub.publish(stop)

    def _finish(self, reason: str):
        if self.done:
            return
        self._phase = _Phase.DONE
        self._publish_stop()
        self.get_logger().info(reason)

    def _normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _update_rotation(self) -> float:
        if self._latest_odom is None or self._prev_yaw is None:
            return 0.0
        yaw = self._yaw_from_odom(self._latest_odom)
        delta = self._normalize_angle(yaw - self._prev_yaw)
        self._prev_yaw = yaw
        self._rotation_accum += abs(delta)
        return self._rotation_accum

    def _enter_wait_nav2(self):
        self._publish_stop()
        self._phase = _Phase.WAIT_NAV2
        self._nav2_deadline = (
            self.get_clock().now()
            + rclpy.duration.Duration(seconds=self._nav2_wait_timeout)
        )
        self._nav2_ready_since = None
        self._last_nav2_check_mono = 0.0
        self.get_logger().info(
            f'预滚动完成，等待 Nav2 action server 就绪（稳定 {self._nav2_stabilize_sec:.0f}s 后启动 explore）...'
        )

    def _nav2_execution_ready(self) -> tuple:
        """仅检查 action server（不用 ros2 lifecycle CLI，避免 Jetson 负载下误超时）。"""
        if self._nav_client is None or self._follow_path_client is None:
            self._nav2_ready_since = None
            return False, 'Nav2 action types unavailable'
        if not self._nav_client.server_is_ready():
            self._nav2_ready_since = None
            return False, 'navigate_to_pose action server 未就绪'
        if not self._follow_path_client.server_is_ready():
            self._nav2_ready_since = None
            return False, 'follow_path action server 未就绪（controller_server）'
        now = time.monotonic()
        if self._nav2_ready_since is None:
            self._nav2_ready_since = now
            remain = self._nav2_stabilize_sec
            return False, f'Nav2 action 已就绪，稳定等待 {remain:.0f}s'
        remain = self._nav2_stabilize_sec - (now - self._nav2_ready_since)
        if remain > 0.0:
            return False, f'Nav2 action 已就绪，稳定等待 {remain:.0f}s'
        return True, ''

    def _tick(self):
        if self.done:
            return

        now = self.get_clock().now()

        if self._phase in (_Phase.FORWARD, _Phase.ROTATE) and self._move_start is not None:
            elapsed = (now - self._move_start).nanoseconds / 1e9
            if elapsed > self._max_duration:
                self.get_logger().warn(
                    f'预滚动运动超时 ({self._max_duration:.1f}s)，进入 Nav2 等待'
                )
                self._enter_wait_nav2()
                return

        if self._phase == _Phase.WAIT_ODOM:
            if now >= self._odom_deadline:
                self._finish('里程计超时，跳过预滚动')
            return

        if self._phase == _Phase.FORWARD:
            traveled = self._distance_traveled()
            if traveled >= self._distance_m:
                if self._scan_rotation_rad > 0.1:
                    self._rotation_accum = 0.0
                    self._prev_yaw = (
                        self._yaw_from_odom(self._latest_odom)
                        if self._latest_odom else self._start_yaw
                    )
                    self._phase = _Phase.ROTATE
                    self.get_logger().info(
                        f'前进完成 {traveled:.2f} m，开始原地旋转扫图'
                    )
                else:
                    self._enter_wait_nav2()
                return

            cmd = Twist()
            cmd.linear.x = self._linear_speed
            self._cmd_pub.publish(cmd)
            return

        if self._phase == _Phase.ROTATE:
            rotated = self._update_rotation()
            if rotated >= self._scan_rotation_rad:
                self.get_logger().info(
                    f'旋转扫图完成 {math.degrees(rotated):.0f}°'
                )
                self._enter_wait_nav2()
                return

            cmd = Twist()
            cmd.angular.z = self._angular_speed
            self._cmd_pub.publish(cmd)
            return

        if self._phase == _Phase.WAIT_NAV2:
            if self._nav_client is None:
                self._finish('Nav2 action type unavailable — starting explore anyway')
                return

            check_now = time.monotonic()
            if check_now - self._last_nav2_check_mono < self._nav2_check_period:
                return
            self._last_nav2_check_mono = check_now

            ready, reason = self._nav2_execution_ready()
            if ready:
                self._finish(
                    '预滚动完成：Nav2 action 就绪且已稳定，启动 explore_lite'
                )
                return
            if reason:
                self.get_logger().info(
                    f'等待 Nav2 就绪：{reason}',
                    throttle_duration_sec=5.0,
                )
            if self._nav2_deadline is not None and now >= self._nav2_deadline:
                self._finish(
                    f'Nav2 等待超时 ({self._nav2_wait_timeout:.0f}s) — starting explore anyway'
                )


def main(args=None):
    rclpy.init(args=args)
    node = ExplorePreroll()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node._publish_stop()
        for client in (getattr(node, '_nav_client', None),
                       getattr(node, '_follow_path_client', None)):
            if client is not None:
                try:
                    client.destroy()
                except Exception:
                    pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()