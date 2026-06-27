#!/usr/bin/env python3
"""订阅 Nav2 / explore_lite 状态，用中文输出导航/探索进度日志。"""

import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

_MODE_LABELS = {
    'mapping': '建图',
    'explore': '探索',
    'localization': '定位',
}

_GOAL_STATUS_CN = {
    GoalStatus.STATUS_UNKNOWN: '未知',
    GoalStatus.STATUS_ACCEPTED: '已接受',
    GoalStatus.STATUS_EXECUTING: '执行中',
    GoalStatus.STATUS_CANCELING: '取消中',
    GoalStatus.STATUS_SUCCEEDED: '成功',
    GoalStatus.STATUS_CANCELED: '已取消',
    GoalStatus.STATUS_ABORTED: '中断',
}

_ACTIVE_STATUSES = (
    GoalStatus.STATUS_EXECUTING,
    GoalStatus.STATUS_ACCEPTED,
    GoalStatus.STATUS_CANCELING,
)

_EXPLORE_STATUS_CN = {
    'exploration_started': '探索已开始',
    'exploration_in_progress': '探索进行中',
    'exploration_paused': '探索已暂停',
    'exploration_complete': '探索完成（无前沿）',
    'returning_to_origin': '返回起点',
    'returned_to_origin': '已回到起点',
}


class NavProgressMonitor(Node):
    def __init__(self) -> None:
        super().__init__('nav_progress_monitor')
        self.declare_parameter('slam_mode', 'mapping')
        self.declare_parameter('heartbeat_sec', 8.0)
        self.declare_parameter('frontier_stats_log_sec', 30.0)
        self.declare_parameter('aborted_warn_sec', 8.0)
        self.declare_parameter('nav_action_status_topic', '/navigate_to_pose/_action/status')
        self.declare_parameter('nav_action_feedback_topic', '/navigate_to_pose/_action/feedback')
        self.declare_parameter('explore_status_topic', '/explore/status')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('cmd_vel_stale_sec', 6.0)

        slam_mode = str(self.get_parameter('slam_mode').value).strip().lower()
        self._mode_label = _MODE_LABELS.get(slam_mode, slam_mode or '建图')
        self._heartbeat_sec = max(5.0, float(self.get_parameter('heartbeat_sec').value))
        self._frontier_stats_log_sec = max(
            10.0, float(self.get_parameter('frontier_stats_log_sec').value),
        )
        self._aborted_warn_sec = max(2.0, float(self.get_parameter('aborted_warn_sec').value))
        self._nav_status_topic = str(self.get_parameter('nav_action_status_topic').value)
        self._explore_topic = str(self.get_parameter('explore_status_topic').value)

        self._goal_status = GoalStatus.STATUS_UNKNOWN
        self._active_goal_id = None
        self._distance_remaining = -1.0
        self._explore_status = ''
        self._last_explore_status = ''
        self._nav_ready = False
        self._status_count = 0
        self._nav_client = None
        self._recovery_pending = False
        self._abort_warn_timer = None
        self._cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self._cmd_vel_stale_sec = max(3.0, float(self.get_parameter('cmd_vel_stale_sec').value))
        self._last_cmd_vel_mono = 0.0
        self._last_cmd_vel_nonzero = False
        self._goal_start_mono = 0.0
        self._blacklist_count = 0
        self._frontiers_found = 0
        self._frontiers_available = 0
        self._last_heartbeat_key = None
        self._last_heartbeat_log_mono = 0.0
        self._last_frontier_stats_log_mono = 0.0

        self.get_logger().set_level(LoggingSeverity.INFO)

        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        explore_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            GoalStatusArray, self._nav_status_topic, self._on_nav_status, status_qos,
        )

        feedback_type = self._load_feedback_type()
        if feedback_type is not None:
            feedback_topic = str(self.get_parameter('nav_action_feedback_topic').value)
            self.create_subscription(
                feedback_type, feedback_topic, self._on_nav_feedback, status_qos,
            )

        try:
            from explore_lite_msgs.msg import ExploreStatus
            self.create_subscription(
                ExploreStatus, self._explore_topic, self._on_explore_status, explore_qos,
            )
        except ImportError as exc:
            self.get_logger().warn(
                f'[任务进度] explore_lite_msgs 不可用，探索状态日志关闭：{exc}'
            )

        try:
            from nav2_msgs.action import NavigateToPose
            action_name = self._nav_status_topic.replace('/_action/status', '')
            self._nav_client = rclpy.action.ActionClient(self, NavigateToPose, action_name)
        except Exception:
            self._nav_client = None

        self.create_subscription(
            Twist, self._cmd_vel_topic, self._on_cmd_vel, status_qos,
        )

        self.create_timer(self._heartbeat_sec, self._heartbeat)
        self.get_logger().info(
            f'[任务进度] 导航监控已启动，模式={self._mode_label}，'
            f'订阅 {self._nav_status_topic}，心跳 {self._heartbeat_sec:.0f}s。'
        )

    @staticmethod
    def _load_feedback_type():
        try:
            from nav2_msgs.action import NavigateToPose
            return NavigateToPose.Impl.FeedbackMessage
        except Exception:
            return None

    @staticmethod
    def _goal_id(entry) -> tuple:
        goal_info = getattr(entry, 'goal_info', None)
        if goal_info is None:
            return ()
        goal_id = getattr(goal_info, 'goal_id', None)
        if goal_id is None:
            return ()
        uuid = getattr(goal_id, 'uuid', None)
        if uuid is None:
            return ()
        return tuple(uuid)

    def _pick_active_status(self, status_list):
        """Prefer running goals; stale ABORTED entries stay in the action status array."""
        executing = None
        accepted = None
        for entry in status_list:
            status = int(entry.status)
            if status == GoalStatus.STATUS_EXECUTING:
                executing = entry
            elif status == GoalStatus.STATUS_ACCEPTED and accepted is None:
                accepted = entry
        if executing is not None:
            return executing
        if accepted is not None:
            return accepted
        return status_list[-1]

    def _cancel_abort_warn_timer(self) -> None:
        if self._abort_warn_timer is not None:
            self._abort_warn_timer.cancel()
            self._abort_warn_timer.destroy()
            self._abort_warn_timer = None

    def _schedule_abort_warn(self) -> None:
        self._cancel_abort_warn_timer()
        self._abort_warn_timer = self.create_timer(
            self._aborted_warn_sec, self._on_abort_warn_timeout)

    def _on_abort_warn_timeout(self) -> None:
        self._cancel_abort_warn_timer()
        if self._goal_status != GoalStatus.STATUS_ABORTED:
            return
        self.get_logger().warn(
            f'[任务进度] 模式={self._mode_label} | 导航阶段=失败 | '
            f'目标在 {self._aborted_warn_sec:.0f}s 内未恢复。'
            '可能原因：规划失败、被障碍阻挡或控制器超时；'
            'Nav2 将换目标或等待 explore_lite 重试。'
        )

    def _on_nav_status(self, msg: GoalStatusArray) -> None:
        self._status_count += 1
        self._nav_ready = True
        if not msg.status_list:
            if self._goal_status != GoalStatus.STATUS_UNKNOWN:
                self._goal_status = GoalStatus.STATUS_UNKNOWN
                self._active_goal_id = None
                self._recovery_pending = False
                self._cancel_abort_warn_timer()
                self.get_logger().info(
                    f'[任务进度] 模式={self._mode_label} | 导航阶段=空闲 | '
                    'Nav2 就绪，等待导航目标。'
                )
            return

        active = self._pick_active_status(msg.status_list)
        new_status = int(active.status)
        new_goal_id = self._goal_id(active)
        if new_status == self._goal_status and new_goal_id == self._active_goal_id:
            return

        old_status = self._goal_status
        old_cn = _GOAL_STATUS_CN.get(old_status, str(old_status))
        new_cn = _GOAL_STATUS_CN.get(new_status, str(new_status))
        self._goal_status = new_status
        self._active_goal_id = new_goal_id

        if new_status == GoalStatus.STATUS_EXECUTING:
            self._cancel_abort_warn_timer()
            self._goal_start_mono = time.monotonic()
            if self._recovery_pending:
                self._recovery_pending = False
                self.get_logger().info(
                    f'[任务进度] 模式={self._mode_label} | 导航阶段=跟路径 | '
                    f'{self._explore_stats_part()}'
                    'Nav2 恢复后继续向目标点移动。'
                )
            else:
                self.get_logger().info(
                    f'[任务进度] 模式={self._mode_label} | 导航阶段=跟路径 | '
                    f'{self._explore_stats_part()}'
                    f'目标状态={new_cn}，开始移动。'
                )
        elif new_status == GoalStatus.STATUS_SUCCEEDED:
            self._recovery_pending = False
            self._cancel_abort_warn_timer()
            self.get_logger().info(
                f'[任务进度] 模式={self._mode_label} | 导航阶段=到达 | '
                f'目标状态={new_cn}，本段导航完成。'
            )
            self._distance_remaining = -1.0
        elif new_status == GoalStatus.STATUS_ABORTED:
            # BT recovery aborts follow_path/plan briefly; not a terminal failure.
            self._recovery_pending = True
            self._distance_remaining = -1.0
            self._schedule_abort_warn()
            self.get_logger().info(
                f'[任务进度] 模式={self._mode_label} | 导航阶段=恢复中 | '
                f'本段路径受阻（{old_cn}→{new_cn}），Nav2 正在 spin/wait 重试。'
            )
        elif new_status == GoalStatus.STATUS_CANCELED:
            self._recovery_pending = False
            self._cancel_abort_warn_timer()
            self.get_logger().info(
                f'[任务进度] 模式={self._mode_label} | 导航阶段=取消 | '
                f'目标状态={new_cn}。'
            )
            self._distance_remaining = -1.0
        elif new_status == GoalStatus.STATUS_ACCEPTED:
            self._cancel_abort_warn_timer()
            self.get_logger().info(
                f'[任务进度] 模式={self._mode_label} | 导航阶段=规划 | '
                f'目标状态={new_cn}，全局路径计算中。'
            )
        else:
            self.get_logger().info(
                f'[任务进度] 模式={self._mode_label} | 导航阶段=更新 | '
                f'目标状态 {old_cn} → {new_cn}。'
            )

    def _on_cmd_vel(self, msg: Twist) -> None:
        moving = (
            abs(msg.linear.x) > 0.02
            or abs(msg.linear.y) > 0.02
            or abs(msg.angular.z) > 0.05
        )
        if moving:
            self._last_cmd_vel_mono = time.monotonic()
            self._last_cmd_vel_nonzero = True

    def _cmd_vel_stale(self) -> bool:
        if not self._last_cmd_vel_nonzero:
            if self._goal_status != GoalStatus.STATUS_EXECUTING:
                return False
            if self._goal_start_mono <= 0.0:
                return False
            return (time.monotonic() - self._goal_start_mono) > self._cmd_vel_stale_sec
        return (time.monotonic() - self._last_cmd_vel_mono) > self._cmd_vel_stale_sec

    def _cmd_vel_stale_elapsed(self) -> float:
        if self._last_cmd_vel_nonzero:
            return max(0.0, time.monotonic() - self._last_cmd_vel_mono)
        if self._goal_start_mono > 0.0:
            return max(0.0, time.monotonic() - self._goal_start_mono)
        return 0.0

    def _explore_stats_part(self) -> str:
        if self._mode_label != '探索':
            return ''
        return (
            f'已拉黑 {self._blacklist_count} 点，'
            f'可用前沿 {self._frontiers_available}/{self._frontiers_found}，'
        )

    def _is_stuck_like(self) -> bool:
        if self._goal_status == GoalStatus.STATUS_EXECUTING and self._cmd_vel_stale():
            return True
        if self._goal_status == GoalStatus.STATUS_ABORTED and self._recovery_pending:
            return True
        return False

    def _should_log_frontier_stats(self) -> bool:
        now = time.monotonic()
        if (now - self._last_frontier_stats_log_mono) < self._frontier_stats_log_sec:
            return False
        self._last_frontier_stats_log_mono = now
        return True

    def _on_nav_feedback(self, msg) -> None:
        feedback = msg.feedback
        dist = float(getattr(feedback, 'distance_remaining', -1.0))
        if dist >= 0.0:
            self._distance_remaining = dist

    def _on_explore_status(self, msg) -> None:
        status = str(getattr(msg, 'status', '') or '')
        blacklist = int(getattr(msg, 'blacklist_count', 0))
        frontiers_found = int(getattr(msg, 'frontiers_found', 0))
        frontiers_available = int(getattr(msg, 'frontiers_available', 0))

        blacklist_changed = blacklist != self._blacklist_count
        frontier_stats_changed = (
            frontiers_found != self._frontiers_found
            or frontiers_available != self._frontiers_available
        )

        if blacklist > self._blacklist_count:
            self.get_logger().warn(
                f'[任务进度] 模式={self._mode_label} | 探索拉黑 | '
                f'已拉黑 {blacklist} 点，可用前沿 {frontiers_available}/{frontiers_found}。'
            )
        elif (
            frontier_stats_changed
            and self._mode_label == '探索'
            and self._should_log_frontier_stats()
        ):
            self.get_logger().info(
                f'[任务进度] 模式={self._mode_label} | 前沿统计 | '
                f'已拉黑 {blacklist} 点，可用前沿 {frontiers_available}/{frontiers_found}。'
            )

        self._blacklist_count = blacklist
        self._frontiers_found = frontiers_found
        self._frontiers_available = frontiers_available

        if not status:
            return
        if status != self._last_explore_status:
            self._last_explore_status = status
            self._explore_status = status
            label = _EXPLORE_STATUS_CN.get(status, status)
            level = (
                self.get_logger().warn
                if status == 'exploration_complete'
                else self.get_logger().info
            )
            level(
                f'[任务进度] 模式={self._mode_label} | 探索状态={label} | '
                f'已拉黑 {blacklist} 点，可用前沿 {frontiers_available}/{frontiers_found}。'
            )
        elif blacklist_changed or frontier_stats_changed:
            self._last_heartbeat_key = None

    def _nav_phase_cn(self) -> str:
        if not self._nav2_server_available():
            return 'Nav2 未就绪'
        if self._goal_status == GoalStatus.STATUS_EXECUTING:
            return '跟路径'
        if self._goal_status == GoalStatus.STATUS_ACCEPTED:
            return '规划中'
        if self._goal_status in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED):
            return '空闲'
        if self._goal_status == GoalStatus.STATUS_ABORTED:
            return '恢复中' if self._recovery_pending else '失败'
        return '空闲'

    def _nav2_server_available(self) -> bool:
        if self._nav_client is not None and self._nav_client.server_is_ready():
            return True
        return self._nav_ready

    def _heartbeat_snapshot_key(self):
        stale = self._cmd_vel_stale()
        stale_bucket = int(self._cmd_vel_stale_elapsed() // 2) if stale else 0
        dist_bucket = (
            round(self._distance_remaining, 1)
            if self._distance_remaining >= 0.0
            else -1.0
        )
        return (
            self._goal_status,
            dist_bucket,
            self._explore_status,
            self._blacklist_count,
            self._frontiers_available,
            stale,
            stale_bucket,
            self._recovery_pending,
        )

    def _should_log_heartbeat(self) -> bool:
        key = self._heartbeat_snapshot_key()
        now = time.monotonic()
        if key == self._last_heartbeat_key:
            return False
        if (
            not self._is_stuck_like()
            and self._last_heartbeat_log_mono > 0.0
            and (now - self._last_heartbeat_log_mono) < self._heartbeat_sec * 0.9
        ):
            return False
        self._last_heartbeat_key = key
        self._last_heartbeat_log_mono = now
        return True

    def _heartbeat(self) -> None:
        if not self._nav2_server_available():
            if not self._should_log_heartbeat():
                return
            self.get_logger().warn(
                f'[任务进度] 模式={self._mode_label} | 导航阶段=未就绪 | '
                f'Nav2 action server ({self._nav_status_topic.rsplit("/", 2)[0]}) 尚不可用。'
                '请确认 Nav2 已启动（explore 模式约 nav2_startup_delay 后 15s）。'
            )
            return

        if not self._should_log_heartbeat():
            return

        explore_part = ''
        if self._explore_status:
            explore_part = (
                f'探索={_EXPLORE_STATUS_CN.get(self._explore_status, self._explore_status)}，'
            )
        stats_part = self._explore_stats_part()

        dist_part = ''
        if self._goal_status == GoalStatus.STATUS_EXECUTING and self._distance_remaining >= 0.0:
            dist_part = f'剩余约 {self._distance_remaining:.2f}m，'

        if self._goal_status == GoalStatus.STATUS_EXECUTING:
            if self._cmd_vel_stale():
                elapsed = self._cmd_vel_stale_elapsed()
                cancel_in = max(0.0, self._cmd_vel_stale_sec - elapsed)
                self.get_logger().warn(
                    f'[任务进度] 模式={self._mode_label} | 导航阶段=静止等待 | '
                    f'{explore_part}{stats_part}{dist_part}'
                    f'已静止 {elapsed:.0f}s（无 cmd_vel），'
                    f'约 {cancel_in:.0f}s 后 explore 拉黑换点。'
                )
            else:
                self.get_logger().info(
                    f'[任务进度] 模式={self._mode_label} | 导航阶段=移动中 | '
                    f'{explore_part}{stats_part}{dist_part}'
                    f'机器狗正在向目标点移动。'
                )
        elif self._goal_status == GoalStatus.STATUS_ABORTED and self._recovery_pending:
            self.get_logger().info(
                f'[任务进度] 模式={self._mode_label} | 导航阶段=脱困重试 | '
                f'{explore_part}{stats_part}'
                'Nav2 正在 spin/wait 重试，机器狗可能短暂静止。'
            )
        elif self._goal_status in (GoalStatus.STATUS_UNKNOWN, GoalStatus.STATUS_SUCCEEDED,
                                   GoalStatus.STATUS_CANCELED):
            self.get_logger().info(
                f'[任务进度] 模式={self._mode_label} | 导航阶段={self._nav_phase_cn()} | '
                f'{explore_part}{stats_part}'
                'Nav2 就绪，等待新目标。'
            )
        else:
            self.get_logger().info(
                f'[任务进度] 模式={self._mode_label} | 导航阶段={self._nav_phase_cn()} | '
                f'{explore_part}{stats_part}'
                f'目标状态={_GOAL_STATUS_CN.get(self._goal_status, self._goal_status)}。'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavProgressMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()