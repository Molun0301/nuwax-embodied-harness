#!/usr/bin/env python3
"""
Frontier-based autonomous exploration for Nav2.

Periodically scans the OccupancyGrid published on /map and finds frontiers
(free cells adjacent to unknown space). It scores each frontier by a mix of
information gain, travel cost, turn cost, and local clearance, then dispatches
the best goal to Nav2 via the /navigate_to_pose action. Failed goals are
blacklisted briefly so we don't loop on the same unreachable spot.

Stops only when no frontiers remain. On repeated Nav2 failures it escalates
strategy (relaxed -> corridor -> retry reset) instead of ending exploration.

Tunables (ROS parameters, all overridable from launch):
  - map_topic           : /map
  - global_frame        : map
  - robot_frame         : base_link
  - replan_period       : 3.0   seconds between exploration ticks
  - min_frontier_size   : 10    cells (after clustering) to be considered
  - info_gain_radius    : 1.5   meters used to estimate unknown area around a goal
  - info_gain_weight    : 1.0   reward for unknown area near a frontier
  - size_weight         : 0.6   reward for larger frontiers
  - travel_weight       : 0.8   penalty for long navigation
  - turn_weight         : 0.25  penalty for large heading changes
  - clearance_weight    : 0.8   reward for open space around the goal
  - max_goal_candidates_per_frontier : 80 cells sampled per frontier cluster
  - blacklist_radius    : 0.5   meters; nearby failed goals share a slot
  - blacklist_ttl       : 60.0  seconds before a failed goal is retryable
  - goal_yaw_to_robot   : true  face the goal back toward current robot
                                heading (helps Nav2 reach feasibility)
"""
import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Vector3
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class FrontierCluster:
    """One connected frontier blob."""
    cells: np.ndarray  # (N, 2) array of (row, col)
    centroid_world: Tuple[float, float]
    size: int


@dataclass
class FrontierCandidate:
    """Frontier cluster plus the concrete goal chosen for it."""
    cluster: FrontierCluster
    goal_world: Tuple[float, float]
    goal_cell: Tuple[int, int]
    info_gain: int
    clearance_m: float
    heading_error: float
    travel_m: float
    score: float


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        # ---- Parameters
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('replan_period', 3.0)
        self.declare_parameter('min_frontier_size', 10)
        self.declare_parameter('info_gain_radius', 1.5)
        self.declare_parameter('info_gain_weight', 1.0)
        self.declare_parameter('size_weight', 0.6)
        self.declare_parameter('travel_weight', 0.8)
        self.declare_parameter('turn_weight', 0.25)
        self.declare_parameter('clearance_weight', 0.8)
        self.declare_parameter('max_goal_candidates_per_frontier', 80)
        self.declare_parameter('blacklist_radius', 0.5)
        self.declare_parameter('blacklist_ttl', 60.0)
        self.declare_parameter('goal_yaw_to_robot', True)
        self.declare_parameter('max_consecutive_failures', 10)
        # After this many consecutive nav failures, relax goal clearance / travel filters.
        self.declare_parameter('failures_before_relaxed', 4)
        # Prefer nearer goals with gentler turns (office corridor friendly).
        self.declare_parameter('failures_before_corridor', 7)
        # Clear blacklist, cooldown, then retry — never treat as exploration complete.
        self.declare_parameter('failures_before_retry_reset', 10)
        self.declare_parameter('relaxed_min_goal_clearance_m', 0.15)
        self.declare_parameter('relaxed_min_travel_m', 0.35)
        self.declare_parameter('corridor_travel_weight_scale', 1.35)
        self.declare_parameter('corridor_turn_weight_scale', 1.5)
        self.declare_parameter('retry_cooldown_sec', 12.0)
        # Grace period after launch before sending the first goal. Nav2's
        # lifecycle manager reports 'active' before the bt_navigator finishes
        # wiring up its internal action clients (compute_path_to_pose,
        # follow_path, ...), so sending a goal immediately races and gets
        # rejected with 'send_goal failed'. Wait a few seconds to be safe.
        # Before frontier scoring, send one short forward goal so RTAB-Map/Nav2
        # get fresh odometry and map updates (stack is unreliable if the robot
        # never moves after launch).
        self.declare_parameter('bootstrap_forward_enabled', True)
        self.declare_parameter('bootstrap_forward_m', 0.2)
        self.declare_parameter('startup_grace_period', 8.0)
        # Hard timeout for a single navigation goal. Foxy's bt_navigator
        # occasionally segfaults mid-execution (CycloneDDS rmw bug); when that
        # happens the result_future never fires and we would otherwise hang
        # forever with active_goal_handle != None. After this timeout we
        # cancel, blacklist the goal, and try a new frontier.
        self.declare_parameter('goal_timeout', 60.0)
        # Ignore frontier goals closer than this (m). Prevents picking a goal the
        # robot is already beside, which Nav2 immediately marks as succeeded.
        self.declare_parameter('min_travel_m', 0.5)
        # Skip goal cells with less than this clearance to occupied space (m).
        self.declare_parameter('min_goal_clearance_m', 0.25)
        # After Nav2 reports success, reject the result if the robot is still
        # farther than this from the dispatched goal (planner snap / early stop).
        self.declare_parameter('goal_success_tolerance', 0.45)
        # Do not treat Nav2 abort/reject as final until the goal had time to execute
        # and the robot had a chance to move (prevents rapid frontier hopping in RViz).
        self.declare_parameter('min_nav_commit_sec', 15.0)
        self.declare_parameter('min_failure_robot_move_m', 0.12)
        self.declare_parameter('max_goal_retries', 2)
        self.declare_parameter('goal_retry_delay_sec', 3.0)
        # While waiting for Nav2 to accept send_goal_async, hold the dispatch slot.
        self.declare_parameter('goal_accept_timeout', 12.0)
        # After ABORTED / stuck-without-motion, nudge forward/back to leave costmap pinch.
        self.declare_parameter('stuck_escape_enabled', True)
        self.declare_parameter('stuck_escape_forward_m', 0.35)
        self.declare_parameter('stuck_escape_after_failures', 2)
        self.declare_parameter('stuck_escape_max_attempts', 3)
        # Pause between frontier goals so Nav2 is not hammered with short hops.
        # Optional idle gap before the *next* frontier after a verified arrival.
        # 0 = disabled (robot should not stand still between goals).
        self.declare_parameter('post_goal_pause_sec', 0.0)
        # Nav2 xy_goal_tolerance is ~0.15m; reject instant SUCCEEDED with no motion.
        self.declare_parameter('min_goal_progress_m', 0.40)
        self.declare_parameter('success_min_elapsed_sec', 4.0)
        self.declare_parameter('global_plan_topic', '/plan')
        self.declare_parameter('next_goal_topic', '/exploration/next_goal')

        gp = self.get_parameter
        self.map_topic = gp('map_topic').value
        self.global_frame = gp('global_frame').value
        self.robot_frame = gp('robot_frame').value
        self.replan_period = float(gp('replan_period').value)
        self.min_frontier_size = int(gp('min_frontier_size').value)
        self.info_gain_radius = float(gp('info_gain_radius').value)
        self.info_gain_weight = float(gp('info_gain_weight').value)
        self.size_weight = float(gp('size_weight').value)
        self.travel_weight = float(gp('travel_weight').value)
        self.turn_weight = float(gp('turn_weight').value)
        self.clearance_weight = float(gp('clearance_weight').value)
        self.max_goal_candidates_per_frontier = int(gp('max_goal_candidates_per_frontier').value)
        self.blacklist_radius = float(gp('blacklist_radius').value)
        self.blacklist_ttl = float(gp('blacklist_ttl').value)
        self.goal_yaw_to_robot = bool(gp('goal_yaw_to_robot').value)
        self.max_consecutive_failures = int(gp('max_consecutive_failures').value)
        self.failures_before_relaxed = int(gp('failures_before_relaxed').value)
        self.failures_before_corridor = int(gp('failures_before_corridor').value)
        self.failures_before_retry_reset = int(gp('failures_before_retry_reset').value)
        self.relaxed_min_goal_clearance_m = float(gp('relaxed_min_goal_clearance_m').value)
        self.relaxed_min_travel_m = float(gp('relaxed_min_travel_m').value)
        self.corridor_travel_weight_scale = float(gp('corridor_travel_weight_scale').value)
        self.corridor_turn_weight_scale = float(gp('corridor_turn_weight_scale').value)
        self.retry_cooldown_sec = float(gp('retry_cooldown_sec').value)
        self.bootstrap_forward_enabled = bool(gp('bootstrap_forward_enabled').value)
        self.bootstrap_forward_m = float(gp('bootstrap_forward_m').value)
        self.startup_grace_period = float(gp('startup_grace_period').value)
        self.goal_timeout = float(gp('goal_timeout').value)
        self.min_travel_m = float(gp('min_travel_m').value)
        self.min_goal_clearance_m = float(gp('min_goal_clearance_m').value)
        self.goal_success_tolerance = float(gp('goal_success_tolerance').value)
        self.min_nav_commit_sec = float(gp('min_nav_commit_sec').value)
        self.min_failure_robot_move_m = float(gp('min_failure_robot_move_m').value)
        self.max_goal_retries = int(gp('max_goal_retries').value)
        self.goal_retry_delay_sec = float(gp('goal_retry_delay_sec').value)
        self.goal_accept_timeout = float(gp('goal_accept_timeout').value)
        self.stuck_escape_enabled = bool(gp('stuck_escape_enabled').value)
        self.stuck_escape_forward_m = float(gp('stuck_escape_forward_m').value)
        self.stuck_escape_after_failures = int(gp('stuck_escape_after_failures').value)
        self.stuck_escape_max_attempts = int(gp('stuck_escape_max_attempts').value)
        self.post_goal_pause_sec = float(gp('post_goal_pause_sec').value)
        self.min_goal_progress_m = float(gp('min_goal_progress_m').value)
        self.success_min_elapsed_sec = float(gp('success_min_elapsed_sec').value)
        self.global_plan_topic = str(gp('global_plan_topic').value)
        self.next_goal_topic = str(gp('next_goal_topic').value)

        # ---- State
        self.latest_map: Optional[OccupancyGrid] = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # blacklist: list of (x, y, expire_time)
        self.blacklist: List[Tuple[float, float, float]] = []

        # NavigateToPose action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # navigation state
        self.active_goal_handle = None
        self.active_goal_xy: Optional[Tuple[float, float]] = None
        self.active_goal_yaw: Optional[float] = None
        self.active_goal_send_time: Optional[float] = None
        self.active_clusters_snapshot: Optional[List[FrontierCluster]] = None
        self.active_chosen_snapshot: Optional[FrontierCandidate] = None
        self.latest_plan: Optional[Path] = None
        self.consecutive_failures = 0
        self.exploration_done = False
        self._shutdown_requested = False
        self._idle_heartbeat_timer = None
        self._cooldown_until: Optional[float] = None
        self._active_strategy = 'normal'
        self._bootstrap_forward_pending = self.bootstrap_forward_enabled
        self._bootstrap_goal_active = False
        self._goal_send_robot_xy: Optional[Tuple[float, float]] = None
        self._goal_retry_count = 0
        self._retry_resend_at: Optional[float] = None
        self._stuck_escape_pending = False
        self._stuck_escape_attempts = 0
        self._stuck_escape_active = False
        self._post_goal_pause_until: Optional[float] = None
        self._active_goal_travel_m: Optional[float] = None

        # Wall-clock startup time used for the initial grace period.
        self.start_time = self.get_clock().now().nanoseconds * 1e-9

        # ---- Subscriber: /map is published with TRANSIENT_LOCAL by RTAB-Map
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_cb, map_qos)

        # ---- Visualization publisher
        # /frontiers (MarkerArray):
        #   - cluster spheres (cyan, radius scales with size)
        #   - chosen goal (green arrow)
        #   - blacklist points (red X)
        self.viz_pub = self.create_publisher(MarkerArray, '/frontiers', 5)
        self.next_goal_pub = self.create_publisher(PoseStamped, self.next_goal_topic, 5)
        # Subscribe Nav2 global plan for the nav_plan marker overlay on /frontiers.
        self.plan_sub = self.create_subscription(
            Path, self.global_plan_topic, self.plan_cb, 5)

        # ---- Timer: re-evaluate periodically
        self.timer = self.create_timer(self.replan_period, self.tick)

        self.get_logger().info(
            f'FrontierExplorer started. map_topic={self.map_topic}, '
            f'replan={self.replan_period}s, '
            f'min_frontier_size={self.min_frontier_size} cells, '
            f'info_gain_radius={self.info_gain_radius:.1f}m'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def map_cb(self, msg: OccupancyGrid):
        self.latest_map = msg

    def plan_cb(self, msg: Path):
        self.latest_plan = msg

    def _publish_done_heartbeat(self):
        if not self.exploration_done:
            return
        self.publish_status('DONE (exploration finished)')

    def _goal_elapsed_sec(self, now: Optional[float] = None) -> float:
        if self.active_goal_send_time is None:
            return 0.0
        if now is None:
            now = self.get_clock().now().nanoseconds * 1e-9
        return max(0.0, now - self.active_goal_send_time)

    def _robot_movement_since_send(self) -> float:
        if self._goal_send_robot_xy is None:
            return 0.0
        robot = self.get_robot_pose()
        if robot is None:
            return 0.0
        sx, sy = self._goal_send_robot_xy
        return math.hypot(robot[0] - sx, robot[1] - sy)

    def _is_premature_failure(self) -> bool:
        return (
            self._goal_elapsed_sec() < self.min_nav_commit_sec
            and self._robot_movement_since_send() < self.min_failure_robot_move_m
        )

    @staticmethod
    def _nav_status_is_terminal(status: int) -> bool:
        # Nav2 planner/controller abort = unreachable; retrying the same pose loops forever.
        return status in {
            GoalStatus.STATUS_ABORTED,
            GoalStatus.STATUS_CANCELED,
        }

    def _should_retry_goal_failure(self, reason: str, status: Optional[int] = None) -> bool:
        if status is not None and self._nav_status_is_terminal(status):
            return False
        if reason in {'success_too_soon', 'success_too_far'}:
            return self._goal_retry_count < self.max_goal_retries
        if reason in {'rejected', 'accept_timeout', 'retry_resend_failed'}:
            return False
        if reason.startswith('status_'):
            try:
                code = int(reason.split('_', 1)[1])
            except (IndexError, ValueError):
                code = None
            if code is not None and self._nav_status_is_terminal(code):
                return False
        return self._is_premature_failure()

    def _start_post_goal_pause(self) -> None:
        if self.post_goal_pause_sec <= 0.0:
            return
        pause = self.post_goal_pause_sec
        self._post_goal_pause_until = (
            self.get_clock().now().nanoseconds * 1e-9 + pause
        )
        self.get_logger().info(
            f'Pausing {pause:.1f}s before next frontier (verified arrival).'
        )

    def _success_is_meaningful(
        self,
        *,
        dist: Optional[float],
        moved: float,
        elapsed: float,
    ) -> bool:
        """Only count success when the robot is actually at the goal in map frame."""
        if dist is None:
            return False
        if dist > self.goal_success_tolerance:
            return False
        travel = float(self._active_goal_travel_m or 0.0)
        if travel >= 0.8:
            # Nav2 may report SUCCEEDED early; require real progress along the leg.
            progress = 1.0 - min(dist / travel, 1.0)
            if progress < 0.55 and moved < travel * 0.50:
                self.get_logger().warn(
                    f'Rejecting Nav2 SUCCEEDED: only {progress * 100:.0f}% of '
                    f'{travel:.2f}m leg (moved={moved:.2f}m dist={dist:.2f}m).',
                    throttle_duration_sec=3.0,
                )
                return False
        if travel > 0.5 and moved < 0.10:
            return False
        return True

    def _maybe_queue_stuck_escape(self) -> None:
        if not self.stuck_escape_enabled:
            return
        if self._stuck_escape_attempts >= self.stuck_escape_max_attempts:
            return
        if self._robot_movement_since_send() >= self.min_failure_robot_move_m:
            return
        if self.consecutive_failures < self.stuck_escape_after_failures:
            return
        self._stuck_escape_pending = True

    def _finalize_goal_failure(
        self,
        goal_xy: Optional[Tuple[float, float]],
        *,
        reason: str,
        status: Optional[int] = None,
    ) -> None:
        if goal_xy is not None:
            self.add_to_blacklist(*goal_xy)
        self.consecutive_failures += 1
        self._maybe_queue_stuck_escape()
        label = f'status_{status}' if status is not None else reason
        self.get_logger().warn(
            f'Goal {goal_xy} abandoned ({label}); blacklisted. '
            f'failures={self.consecutive_failures}, moved='
            f'{self._robot_movement_since_send():.2f}m.'
        )
        self._clear_active_goal()

    def _schedule_same_goal_retry(self, reason: str, status: Optional[int] = None) -> bool:
        if not self._should_retry_goal_failure(reason, status):
            return False
        if self._goal_retry_count >= self.max_goal_retries:
            return False
        self._goal_retry_count += 1
        self.active_goal_handle = None
        self.active_goal_send_time = None
        delay = (
            0.8
            if reason in {'success_too_soon', 'success_too_far'}
            else self.goal_retry_delay_sec
        )
        self._retry_resend_at = self.get_clock().now().nanoseconds * 1e-9 + delay
        if self._stuck_escape_active:
            strategy = 'escape'
        elif self._bootstrap_goal_active:
            strategy = 'bootstrap'
        else:
            strategy = self._active_strategy
        self.get_logger().warn(
            f'Premature nav failure ({reason}) on {self.active_goal_xy}; '
            f'retry {self._goal_retry_count}/{self.max_goal_retries} in '
            f'{self.goal_retry_delay_sec:.0f}s (robot moved '
            f'{self._robot_movement_since_send():.2f}m, '
            f'elapsed {self._goal_elapsed_sec():.1f}s).'
        )
        hold_label = (
            'HOLDING GOAL'
            if reason in {'success_too_soon', 'success_too_far'}
            else 'RETRY'
        )
        self.publish_status(
            f'STRATEGY:{strategy} {hold_label} {self._goal_retry_count}/'
            f'{self.max_goal_retries} ({reason})'
        )
        return True

    def _resend_active_goal(self) -> bool:
        if self.active_goal_xy is None:
            return False
        robot = self.get_robot_pose()
        if robot is None:
            return False
        gx, gy = self.active_goal_xy
        if not self.send_nav_goal(gx, gy, robot):
            self._retry_resend_at = (
                self.get_clock().now().nanoseconds * 1e-9 + self.goal_retry_delay_sec
            )
            self.get_logger().warn(
                f'Resend to {self.active_goal_xy} failed (Nav2 unavailable); '
                f'retrying in {self.goal_retry_delay_sec:.0f}s.'
            )
            return True
        return True

    def _dispatch_stuck_escape(self, robot_pose: Tuple[float, float, float]) -> None:
        rx, ry, ryaw = robot_pose
        attempt = self._stuck_escape_attempts
        self._stuck_escape_attempts += 1
        self._stuck_escape_pending = False
        if attempt % 2 == 0:
            dist = self.stuck_escape_forward_m
            label = 'forward'
        else:
            dist = -self.stuck_escape_forward_m * 0.85
            label = 'back'
        gx = rx + dist * math.cos(ryaw)
        gy = ry + dist * math.sin(ryaw)
        self.get_logger().warn(
            f'Stuck escape #{self._stuck_escape_attempts}: {label} '
            f'{abs(dist):.2f}m to ({gx:.2f},{gy:.2f}) after '
            f'{self.consecutive_failures} nav failures.'
        )
        self.publish_status(
            f'STRATEGY:escape {label} {abs(dist):.2f}m '
            f'({self._stuck_escape_attempts}/{self.stuck_escape_max_attempts})'
        )
        self.publish_markers(status=f'STRATEGY:escape {label}')
        self._stuck_escape_active = True
        self._goal_retry_count = 0
        self._retry_resend_at = None
        self.send_nav_goal(gx, gy, robot_pose)

    def _handle_in_flight_navigation(self, now: float) -> bool:
        """Return True to keep tick from dispatching a new frontier goal."""

        if self._retry_resend_at is not None:
            if now < self._retry_resend_at:
                remaining = self._retry_resend_at - now
                if self._stuck_escape_active:
                    strategy = 'escape'
                elif self._bootstrap_goal_active:
                    strategy = 'bootstrap'
                else:
                    strategy = self._active_strategy
                self.publish_markers(
                    status=(
                        f'STRATEGY:{strategy} RETRY WAIT ({remaining:.0f}s, '
                        f'{self._goal_retry_count}/{self.max_goal_retries})'
                    ),
                    clusters=self.active_clusters_snapshot,
                    chosen=self.active_chosen_snapshot,
                    navigating=True,
                )
                return True
            self._retry_resend_at = None
            if self._resend_active_goal():
                return True
            self.fail_active_goal('retry_resend_failed', force=True)
            return False

        if self.active_goal_handle is None:
            elapsed = self._goal_elapsed_sec(now)
            if elapsed <= self.goal_accept_timeout:
                if self._stuck_escape_active:
                    strategy = 'escape'
                elif self._bootstrap_goal_active:
                    strategy = 'bootstrap'
                else:
                    strategy = self._active_strategy
                self.publish_markers(
                    status=f'STRATEGY:{strategy} PENDING ({elapsed:.0f}s)',
                    clusters=self.active_clusters_snapshot,
                    chosen=self.active_chosen_snapshot,
                    navigating=True,
                )
                return True
            self.get_logger().warn(
                f'Goal {self.active_goal_xy} not accepted within '
                f'{self.goal_accept_timeout:.0f}s.'
            )
            if self._bootstrap_goal_active:
                if self._schedule_same_goal_retry('accept_timeout'):
                    return True
                self._finish_bootstrap_goal(succeeded=False)
                self.publish_status('STRATEGY:bootstrap ACCEPT TIMEOUT')
                return False
            if self._schedule_same_goal_retry('accept_timeout'):
                return True
            self.fail_active_goal('accept_timeout', force=True)
            return False

        elapsed = self._goal_elapsed_sec(now)
        if elapsed > self.goal_timeout:
            self.get_logger().warn(
                f'Goal {self.active_goal_xy} timed out after {elapsed:.1f}s '
                f'(>{self.goal_timeout:.0f}s). Cancelling; Nav2 may have crashed.'
            )
            try:
                self.active_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f'cancel_goal_async failed: {e}')
            if self._bootstrap_goal_active:
                if self._schedule_same_goal_retry('timeout'):
                    return True
                self._finish_bootstrap_goal(succeeded=False)
                self.publish_status('STRATEGY:bootstrap TIMEOUT')
                return False
            if self._schedule_same_goal_retry('timeout'):
                return True
            self.fail_active_goal('timeout', force=True)
            return False

        if self._stuck_escape_active:
            nav_status = f'STRATEGY:escape NAVIGATING ({elapsed:.0f}s)'
        elif self._bootstrap_goal_active:
            nav_status = f'STRATEGY:bootstrap NAVIGATING ({elapsed:.0f}s)'
        else:
            nav_status = f'NAVIGATING ({elapsed:.0f}s)'
        self.publish_markers(
            status=nav_status,
            clusters=self.active_clusters_snapshot,
            chosen=self.active_chosen_snapshot,
            navigating=True,
        )
        return True

    def tick(self):
        if self.exploration_done:
            # 探索结束：停止派点，但保持本节点进程存活，避免 ros2 launch/go2.sh 随子进程退出。
            # RTAB-Map/Nav2 仅由平台 stop_mapping 统一结束。
            if not self._shutdown_requested:
                self._shutdown_requested = True
                self.get_logger().info(
                    'Exploration complete. frontier_explorer is idle; '
                    'RTAB-Map/Nav2 remain running until stop_mapping.'
                )
                for _ in range(3):
                    self.publish_status('DONE (exploration finished)')
                    time.sleep(0.2)
                self.timer.cancel()
                if self.active_goal_handle is not None:
                    try:
                        self.active_goal_handle.cancel_goal_async()
                    except Exception:
                        pass
                self._clear_active_goal()
                self._idle_heartbeat_timer = self.create_timer(10.0, self._publish_done_heartbeat)
            return

        now = self.get_clock().now().nanoseconds * 1e-9

        # Startup grace: give Nav2's bt_navigator time to fully come up before
        # we start firing goals at it (otherwise the first goal is rejected
        # with 'send_goal failed', see comment on startup_grace_period).
        if now - self.start_time < self.startup_grace_period:
            self.publish_status(
                f'WARMING UP ({self.startup_grace_period - (now - self.start_time):.1f}s)'
            )
            return

        # Hold the current dispatch until Nav2 finishes or we exhaust retries.
        # active_goal_xy is set at send time; active_goal_handle arrives async.
        if self.active_goal_xy is not None:
            if not self._handle_in_flight_navigation(now):
                return
            return

        if self._post_goal_pause_until is not None:
            if now < self._post_goal_pause_until:
                remaining = self._post_goal_pause_until - now
                self.publish_status(
                    f'STRATEGY:{self._active_strategy} PAUSE ({remaining:.0f}s)'
                )
                self.publish_markers(
                    status=f'PAUSE before next goal ({remaining:.0f}s)',
                    clusters=self.active_clusters_snapshot,
                    chosen=self.active_chosen_snapshot,
                )
                return
            self._post_goal_pause_until = None

        if self._cooldown_until is not None:
            if now < self._cooldown_until:
                remaining = self._cooldown_until - now
                self.publish_status(
                    f'STRATEGY: cooldown ({remaining:.0f}s, failures={self.consecutive_failures})'
                )
                return
            self._cooldown_until = None
            self.get_logger().info(
                'Cooldown finished; resuming frontier exploration with fresh blacklist.'
            )

        if self.consecutive_failures >= self.failures_before_retry_reset:
            self._trigger_retry_reset(now)
            return

        # 1) Robot pose
        robot = self.get_robot_pose()
        if robot is None:
            self.get_logger().warn(
                'No TF map->base_link yet, skipping exploration tick.',
                throttle_duration_sec=5.0)
            return
        rx, ry, ryaw = robot

        if (
            self._stuck_escape_pending
            and self.active_goal_xy is None
            and not self._bootstrap_forward_pending
        ):
            self._dispatch_stuck_escape((rx, ry, ryaw))
            return

        if self._bootstrap_forward_pending:
            gx = rx + self.bootstrap_forward_m * math.cos(ryaw)
            gy = ry + self.bootstrap_forward_m * math.sin(ryaw)
            self.get_logger().info(
                f'Bootstrap: dispatching forward goal ({gx:.2f},{gy:.2f}) '
                f'{self.bootstrap_forward_m:.2f}m ahead to refresh map/Nav2.'
            )
            self.publish_status(
                f'STRATEGY:bootstrap FORWARD {self.bootstrap_forward_m:.2f}m'
            )
            self.publish_markers(status='STRATEGY:bootstrap DISPATCHING')
            self._goal_retry_count = 0
            self._retry_resend_at = None
            self.send_nav_goal(gx, gy, (rx, ry, ryaw))
            self._bootstrap_goal_active = self.active_goal_xy is not None
            return

        if self.latest_map is None:
            self.get_logger().info(
                f'Waiting for first /map on topic "{self.map_topic}" '
                f'(check QoS / topic name)',
                throttle_duration_sec=5.0)
            self.publish_status('WAITING for /map')
            return

        # 2) Frontier detection
        all_clusters = self.find_frontier_clusters(self.latest_map)
        if not all_clusters:
            self.publish_markers(status='NO FRONTIERS')
            self.get_logger().info('No frontiers found. Exploration COMPLETE.')
            self.exploration_done = True
            return

        # 3) Expire blacklisted goals before scoring. Blacklist filtering is
        # applied to the concrete candidate goal, not just the cluster centroid.
        self.expire_blacklist()

        # 4) Score and pick best (strategy adjusts clearance / scoring after nav failures)
        selection = self._selection_params()
        self._active_strategy = selection['strategy']
        best = self.pick_best(all_clusters, robot, selection=selection)
        if best is None:
            status = (
                f'WAITING:{selection["strategy"]} (no reachable frontiers, '
                f'failures={self.consecutive_failures})'
            )
            self.publish_markers(status=status, clusters=all_clusters)
            self.get_logger().info(
                'No frontier goal meets filters for strategy=%s; waiting.'
                % selection['strategy'],
                throttle_duration_sec=5.0,
            )
            return
        gx, gy = best.goal_world

        # 5) Remember snapshot for RViz while navigating
        self.active_clusters_snapshot = all_clusters
        self.active_chosen_snapshot = best

        # 6) Visualize all candidates + chosen goal
        self.publish_markers(
            status=f'STRATEGY:{selection["strategy"]} DISPATCHING GOAL',
            clusters=all_clusters,
            chosen=best,
        )

        # 7) Send goal (new frontier — reset per-goal retry budget)
        self._goal_retry_count = 0
        self._retry_resend_at = None
        self._active_goal_travel_m = float(best.travel_m)
        if not self.send_nav_goal(gx, gy, (rx, ry, ryaw)):
            return

        self.get_logger().info(
            f'Selected frontier: ({gx:.2f},{gy:.2f}) score={best.score:.2f} '
            f'gain={best.info_gain} size={best.cluster.size} '
            f'd={best.travel_m:.2f}m turn={math.degrees(best.heading_error):.0f}deg '
            f'clear={best.clearance_m:.2f}m from {len(all_clusters)} candidates'
        )

    # ------------------------------------------------------------------
    # Frontier detection
    # ------------------------------------------------------------------
    def find_frontier_clusters(self, occgrid: OccupancyGrid) -> List[FrontierCluster]:
        """Find connected frontier components in the occupancy grid.

        A frontier cell is a FREE cell (value = 0) that has at least one
        UNKNOWN (-1) cell among its 4-neighborhood. We then run 8-connected
        labeling to merge nearby frontier cells into clusters.
        """
        info = occgrid.info
        h, w = info.height, info.width
        if h == 0 or w == 0:
            return []

        grid = np.array(occgrid.data, dtype=np.int16).reshape(h, w)
        free = (grid == 0)
        unknown = (grid < 0)

        # 4-neighbour shifted unknown masks (no need for padding; use slicing)
        unk_up = np.zeros_like(unknown)
        unk_dn = np.zeros_like(unknown)
        unk_lf = np.zeros_like(unknown)
        unk_rt = np.zeros_like(unknown)
        unk_up[1:, :] = unknown[:-1, :]
        unk_dn[:-1, :] = unknown[1:, :]
        unk_lf[:, 1:] = unknown[:, :-1]
        unk_rt[:, :-1] = unknown[:, 1:]
        frontier = free & (unk_up | unk_dn | unk_lf | unk_rt)

        if not np.any(frontier):
            return []

        # Connected components (8-connected). Use scipy if available,
        # else fall back to a simple BFS labelling.
        try:
            from scipy.ndimage import label
            structure = np.ones((3, 3), dtype=np.int8)
            labels, num = label(frontier, structure=structure)
        except ImportError:
            labels, num = self._bfs_label(frontier)

        clusters: List[FrontierCluster] = []
        ox = info.origin.position.x
        oy = info.origin.position.y
        res = info.resolution
        for lbl in range(1, num + 1):
            rs, cs = np.where(labels == lbl)
            if rs.size < self.min_frontier_size:
                continue
            cells = np.stack([rs, cs], axis=1)
            # Centroid in world coords (cell-center convention)
            cx = ox + (cs.mean() + 0.5) * res
            cy = oy + (rs.mean() + 0.5) * res
            clusters.append(FrontierCluster(
                cells=cells, centroid_world=(cx, cy), size=int(rs.size)
            ))
        return clusters

    @staticmethod
    def _bfs_label(mask: np.ndarray):
        """Simple 8-connected component labelling fallback (no scipy)."""
        h, w = mask.shape
        labels = np.zeros((h, w), dtype=np.int32)
        next_label = 0
        # iterative BFS to avoid recursion limits
        nbrs = [(-1, -1), (-1, 0), (-1, 1),
                (0, -1),           (0, 1),
                (1, -1),  (1, 0),  (1, 1)]
        for r0 in range(h):
            for c0 in range(w):
                if not mask[r0, c0] or labels[r0, c0]:
                    continue
                next_label += 1
                stack = [(r0, c0)]
                while stack:
                    r, c = stack.pop()
                    if labels[r, c]:
                        continue
                    labels[r, c] = next_label
                    for dr, dc in nbrs:
                        nr, nc = r + dr, c + dc
                        if 0 <= nr < h and 0 <= nc < w and mask[nr, nc] and not labels[nr, nc]:
                            stack.append((nr, nc))
        return labels, next_label

    # ------------------------------------------------------------------
    # Goal selection
    # ------------------------------------------------------------------
    def _selection_params(self) -> dict:
        failures = self.consecutive_failures
        if failures >= self.failures_before_corridor:
            strategy = 'corridor'
            min_travel = self.relaxed_min_travel_m
            min_clearance = self.relaxed_min_goal_clearance_m
            travel_weight = self.travel_weight * self.corridor_travel_weight_scale
            turn_weight = self.turn_weight * self.corridor_turn_weight_scale
        elif failures >= self.failures_before_relaxed:
            strategy = 'relaxed'
            min_travel = self.relaxed_min_travel_m
            min_clearance = self.relaxed_min_goal_clearance_m
            travel_weight = self.travel_weight
            turn_weight = self.turn_weight
        else:
            strategy = 'normal'
            min_travel = self.min_travel_m
            min_clearance = self.min_goal_clearance_m
            travel_weight = self.travel_weight
            turn_weight = self.turn_weight
        return {
            'strategy': strategy,
            'min_travel_m': min_travel,
            'min_goal_clearance_m': min_clearance,
            'travel_weight': travel_weight,
            'turn_weight': turn_weight,
        }

    def _trigger_retry_reset(self, now: float) -> None:
        self.get_logger().warn(
            f'{self.consecutive_failures} consecutive navigation failures — '
            f'switching strategy: clear blacklist, cooldown {self.retry_cooldown_sec:.0f}s, '
            f'then retry (exploration continues).'
        )
        self.blacklist.clear()
        self.consecutive_failures = 0
        self._active_strategy = 'retry_reset'
        self._cooldown_until = now + max(1.0, self.retry_cooldown_sec)
        self.publish_status(
            f'STRATEGY:retry_reset (cooldown {self.retry_cooldown_sec:.0f}s)'
        )

    def pick_best(self, clusters: List[FrontierCluster],
                  robot_pose: Tuple[float, float, float],
                  *,
                  selection: Optional[dict] = None) -> Optional[FrontierCandidate]:
        selection = selection or self._selection_params()
        grid = np.array(self.latest_map.data, dtype=np.int16).reshape(
            self.latest_map.info.height, self.latest_map.info.width
        )
        info = self.latest_map.info
        best = None
        best_score = -float('inf')
        for c in clusters:
            candidate = self.evaluate_cluster(
                c, robot_pose, grid, info, selection=selection,
            )
            if self.is_blacklisted(*candidate.goal_world):
                continue
            if candidate.travel_m < selection['min_travel_m']:
                continue
            if candidate.clearance_m < selection['min_goal_clearance_m']:
                continue
            if candidate.score > best_score:
                best_score = candidate.score
                best = candidate
        return best

    def evaluate_cluster(self, cluster: FrontierCluster,
                         robot_pose: Tuple[float, float, float],
                         grid: np.ndarray,
                         info,
                         *,
                         selection: Optional[dict] = None) -> FrontierCandidate:
        selection = selection or self._selection_params()
        travel_weight = float(selection['travel_weight'])
        turn_weight = float(selection['turn_weight'])
        rx, ry, ryaw = robot_pose
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y

        best_goal = None
        best_goal_cell = None
        best_goal_score = -float('inf')
        best_gain = 0
        best_clearance = 0.0
        best_heading_error = 0.0
        best_travel = 0.0

        gain_radius_cells = max(1, int(round(self.info_gain_radius / res))) if res > 1e-6 else 3
        clearance_radius_cells = max(1, int(round(0.35 / res))) if res > 1e-6 else 3

        cells = cluster.cells
        max_samples = max(1, self.max_goal_candidates_per_frontier)
        if cells.shape[0] > max_samples:
            step = max(1, int(math.ceil(cells.shape[0] / max_samples)))
            cells = cells[::step]

        for rr, cc in cells:
            if not self._is_valid_cell(rr, cc, grid.shape):
                continue
            gx = ox + (cc + 0.5) * res
            gy = oy + (rr + 0.5) * res
            travel = math.hypot(gx - rx, gy - ry)
            bearing = math.atan2(gy - ry, gx - rx)
            heading_error = abs(self._angle_diff(bearing, ryaw))
            info_gain = self._count_unknown_in_radius(grid, rr, cc, gain_radius_cells)
            clearance_m = self._estimate_clearance_m(grid, rr, cc, clearance_radius_cells, res)
            local_score = (
                self.info_gain_weight * math.log1p(info_gain) +
                self.size_weight * math.log1p(cluster.size) -
                travel_weight * travel -
                turn_weight * heading_error +
                self.clearance_weight * clearance_m
            )
            if local_score > best_goal_score:
                best_goal_score = local_score
                best_goal = (gx, gy)
                best_goal_cell = (rr, cc)
                best_gain = info_gain
                best_clearance = clearance_m
                best_heading_error = heading_error
                best_travel = travel

        if best_goal is None:
            best_goal = cluster.centroid_world
            best_goal_cell = (int(round((best_goal[1] - oy) / res)), int(round((best_goal[0] - ox) / res)))
            best_gain = 0
            best_clearance = 0.0
            best_heading_error = abs(self._angle_diff(math.atan2(best_goal[1] - ry, best_goal[0] - rx), ryaw))
            best_travel = math.hypot(best_goal[0] - rx, best_goal[1] - ry)
            best_goal_score = (
                self.size_weight * math.log1p(cluster.size) -
                travel_weight * best_travel -
                turn_weight * best_heading_error
            )

        return FrontierCandidate(
            cluster=cluster,
            goal_world=best_goal,
            goal_cell=best_goal_cell,
            info_gain=best_gain,
            clearance_m=best_clearance,
            heading_error=best_heading_error,
            travel_m=best_travel,
            score=best_goal_score,
        )

    @staticmethod
    def _is_valid_cell(r: int, c: int, shape: Tuple[int, int]) -> bool:
        return 0 <= r < shape[0] and 0 <= c < shape[1]

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        return math.atan2(math.sin(a - b), math.cos(a - b))

    def _count_unknown_in_radius(self, grid: np.ndarray, r: int, c: int, radius_cells: int) -> int:
        r0 = max(0, r - radius_cells)
        r1 = min(grid.shape[0], r + radius_cells + 1)
        c0 = max(0, c - radius_cells)
        c1 = min(grid.shape[1], c + radius_cells + 1)
        window = grid[r0:r1, c0:c1]
        return int(np.count_nonzero(window < 0))

    def _estimate_clearance_m(self, grid: np.ndarray, r: int, c: int,
                              radius_cells: int, res: float) -> float:
        r0 = max(0, r - radius_cells)
        r1 = min(grid.shape[0], r + radius_cells + 1)
        c0 = max(0, c - radius_cells)
        c1 = min(grid.shape[1], c + radius_cells + 1)
        window = grid[r0:r1, c0:c1]
        occupied = np.argwhere(window > 50)
        if occupied.size == 0:
            return radius_cells * res
        rr = r - r0
        cc = c - c0
        dists = np.sqrt((occupied[:, 0] - rr) ** 2 + (occupied[:, 1] - cc) ** 2)
        return float(np.min(dists) * res)

    # ------------------------------------------------------------------
    # Blacklist
    # ------------------------------------------------------------------
    def expire_blacklist(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.blacklist = [(x, y, t) for (x, y, t) in self.blacklist if t > now]

    def is_blacklisted(self, x: float, y: float) -> bool:
        return any(
            math.hypot(x - bx, y - by) < self.blacklist_radius
            for (bx, by, _) in self.blacklist
        )

    def add_to_blacklist(self, x: float, y: float):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.blacklist.append((x, y, now + self.blacklist_ttl))
        self.get_logger().info(
            f'Blacklisted ({x:.2f},{y:.2f}) for {self.blacklist_ttl:.0f}s '
            f'(blacklist size={len(self.blacklist)})'
        )

    # ------------------------------------------------------------------
    # Visualization
    # ------------------------------------------------------------------
    def publish_status(self, text: str):
        """Heartbeat text for warming-up / done states."""
        self.publish_markers(status=text)

    def publish_markers(self,
                        *,
                        status: Optional[str] = None,
                        clusters: Optional[List[FrontierCluster]] = None,
                        chosen: Optional[FrontierCandidate] = None,
                        navigating: bool = False):
        """Publish a full RViz marker set on /frontiers (MarkerArray).

        During navigation we keep the active goal arrow, robot-to-goal line,
        and the latest Nav2 global plan visible instead of wiping markers.
        """
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        robot_pose = self.get_robot_pose()

        clear = Marker()
        clear.header.frame_id = self.global_frame
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        cluster_list = clusters or []
        if cluster_list:
            ma.markers.append(self._cluster_text_marker(cluster_list, stamp))
            for i, c in enumerate(cluster_list):
                m = Marker()
                m.header.frame_id = self.global_frame
                m.header.stamp = stamp
                m.ns = 'frontier_spheres'
                m.id = i
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = c.centroid_world[0]
                m.pose.position.y = c.centroid_world[1]
                m.pose.position.z = 0.05
                m.pose.orientation.w = 1.0
                d = max(0.15, min(0.6, 0.05 + 0.01 * math.sqrt(c.size)))
                m.scale = Vector3(x=d, y=d, z=d)
                is_chosen = (
                    chosen is not None
                    and math.hypot(
                        c.centroid_world[0] - chosen.cluster.centroid_world[0],
                        c.centroid_world[1] - chosen.cluster.centroid_world[1],
                    ) < 0.05
                )
                if is_chosen:
                    m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.95)
                else:
                    m.color = ColorRGBA(r=0.0, g=0.7, b=1.0, a=0.7)
                ma.markers.append(m)

        goal_xy = None
        goal_yaw = None
        goal_ns = 'frontier_goal'
        if navigating and self.active_goal_xy is not None:
            goal_xy = self.active_goal_xy
            goal_yaw = self.active_goal_yaw
            goal_ns = 'active_frontier_goal'
        elif chosen is not None:
            goal_xy = chosen.goal_world
            rx, ry, _ = robot_pose if robot_pose is not None else (0.0, 0.0, 0.0)
            goal_yaw = math.atan2(goal_xy[1] - ry, goal_xy[0] - rx)

        if goal_xy is not None and goal_yaw is not None:
            ma.markers.append(self._goal_arrow_marker(
                goal_xy[0], goal_xy[1], goal_yaw, ns=goal_ns, stamp=stamp))
            if robot_pose is not None:
                ma.markers.append(self._robot_to_goal_line(
                    robot_pose[0], robot_pose[1], goal_xy[0], goal_xy[1], stamp))

        if self.latest_plan is not None and self.latest_plan.poses:
            ma.markers.append(self._nav_plan_marker(self.latest_plan, stamp))

        if self.blacklist:
            cubes = Marker()
            cubes.header.frame_id = self.global_frame
            cubes.header.stamp = stamp
            cubes.ns = 'frontier_blacklist'
            cubes.id = 0
            cubes.type = Marker.CUBE_LIST
            cubes.action = Marker.ADD
            cubes.pose.orientation.w = 1.0
            cubes.scale = Vector3(x=0.2, y=0.2, z=0.2)
            cubes.color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.7)
            for (bx, by, _) in self.blacklist:
                cubes.points.append(Point(x=bx, y=by, z=0.1))
            ma.markers.append(cubes)

        if status is not None:
            m = Marker()
            m.header.frame_id = self.global_frame
            m.header.stamp = stamp
            m.ns = 'frontier_status'
            m.id = 0
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            if robot_pose is not None:
                m.pose.position.x, m.pose.position.y = robot_pose[0], robot_pose[1]
            m.pose.position.z = 1.5
            m.pose.orientation.w = 1.0
            m.scale.z = 0.3
            m.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)
            m.text = f'[explorer] {status}'
            ma.markers.append(m)

        self.viz_pub.publish(ma)

    def _goal_arrow_marker(self, gx: float, gy: float, yaw: float,
                           *, ns: str, stamp) -> Marker:
        """Horizontal ground-plane arrow at the exploration goal."""
        m = Marker()
        m.header.frame_id = self.global_frame
        m.header.stamp = stamp
        m.ns = ns
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.position = Point(x=gx, y=gy, z=0.12)
        m.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))
        m.scale = Vector3(x=0.9, y=0.18, z=0.18)
        m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        return m

    def _robot_to_goal_line(self, rx: float, ry: float,
                            gx: float, gy: float, stamp) -> Marker:
        m = Marker()
        m.header.frame_id = self.global_frame
        m.header.stamp = stamp
        m.ns = 'goal_approach_line'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.06
        m.color = ColorRGBA(r=0.2, g=1.0, b=0.2, a=0.85)
        m.points = [
            Point(x=rx, y=ry, z=0.08),
            Point(x=gx, y=gy, z=0.08),
        ]
        return m

    def _nav_plan_marker(self, path: Path, stamp) -> Marker:
        m = Marker()
        m.header.frame_id = path.header.frame_id or self.global_frame
        m.header.stamp = stamp
        m.ns = 'nav_plan'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.05
        m.color = ColorRGBA(r=0.1, g=0.5, b=1.0, a=0.9)
        for pose_stamped in path.poses:
            p = pose_stamped.pose.position
            m.points.append(Point(x=p.x, y=p.y, z=0.06))
        return m

    def _cluster_text_marker(self, clusters: List[FrontierCluster], stamp) -> Marker:
        """Single text marker showing cluster count near the first cluster."""
        m = Marker()
        m.header.frame_id = self.global_frame
        m.header.stamp = stamp
        m.ns = 'frontier_label'
        m.id = 0
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        # Place above first cluster
        cx, cy = clusters[0].centroid_world
        m.pose.position.x = cx
        m.pose.position.y = cy
        m.pose.position.z = 0.8
        m.pose.orientation.w = 1.0
        m.scale.z = 0.25
        m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
        m.text = f'{len(clusters)} frontiers'
        return m

    # ------------------------------------------------------------------
    # TF helper
    # ------------------------------------------------------------------
    def get_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_frame, Time())
        except TransformException:
            return None
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return (tf.transform.translation.x, tf.transform.translation.y, yaw)

    # ------------------------------------------------------------------
    # Nav2 action client
    # ------------------------------------------------------------------
    def send_nav_goal(self, gx: float, gy: float,
                      robot_pose: Tuple[float, float, float]) -> bool:
        if self.active_goal_xy is not None and self.active_goal_handle is not None:
            return False
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(
                'navigate_to_pose action server not available; will retry.')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.global_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position = Point(x=gx, y=gy, z=0.0)
        # Yaw: face toward goal (from current robot pose) for nicer
        # approach. The robot is rotation-symmetric for nav purposes anyway.
        rx, ry, ryaw = robot_pose
        if self.goal_yaw_to_robot:
            yaw = math.atan2(gy - ry, gx - rx)
        else:
            yaw = ryaw
        goal_msg.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)
        )

        self.active_goal_xy = (gx, gy)
        self.active_goal_yaw = yaw
        self.active_goal_send_time = self.get_clock().now().nanoseconds * 1e-9
        self._goal_send_robot_xy = (rx, ry)
        self.active_goal_handle = None

        next_goal = PoseStamped()
        next_goal.header = goal_msg.pose.header
        next_goal.pose = goal_msg.pose.pose
        self.next_goal_pub.publish(next_goal)

        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.on_goal_response)
        return True

    def on_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as e:
            self.get_logger().warn(f'Goal request error: {e}')
            self.fail_active_goal('request_error')
            return
        if not handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2.')
            self.fail_active_goal('rejected')
            return
        self.active_goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.on_goal_result)

    def on_goal_result(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().warn(f'Goal result error: {e}')
            self.fail_active_goal()
            return
        status = result.status
        if self._stuck_escape_active:
            succeeded = status == GoalStatus.STATUS_SUCCEEDED
            if succeeded:
                self.get_logger().info('Stuck escape step completed.')
                self._stuck_escape_attempts = 0
                self.consecutive_failures = max(0, self.consecutive_failures - 1)
            else:
                self.get_logger().warn(
                    f'Stuck escape failed (status={status}).'
                )
            self._clear_active_goal()
            return
        if self._bootstrap_goal_active:
            succeeded = status == GoalStatus.STATUS_SUCCEEDED
            if succeeded:
                self.get_logger().info('Bootstrap forward step completed.')
            else:
                self.get_logger().warn(
                    f'Bootstrap forward step failed (status={status}); '
                    f'continuing with frontier exploration.'
                )
            self._finish_bootstrap_goal(succeeded=succeeded)
            return
        goal_xy = self.active_goal_xy
        if status == GoalStatus.STATUS_SUCCEEDED:
            robot = self.get_robot_pose()
            dist = None
            if robot is not None and goal_xy is not None:
                dist = math.hypot(robot[0] - goal_xy[0], robot[1] - goal_xy[1])
            moved = self._robot_movement_since_send()
            elapsed = self._goal_elapsed_sec()
            if dist is not None and dist > self.goal_success_tolerance:
                self.get_logger().warn(
                    f'Nav2 SUCCEEDED but robot is {dist:.2f}m from goal {goal_xy} '
                    f'(tolerance={self.goal_success_tolerance:.2f}m); holding goal.'
                )
                if self._schedule_same_goal_retry('success_too_far'):
                    return
                self._finalize_goal_failure(goal_xy, reason='success_too_far')
                return
            if not self._success_is_meaningful(dist=dist, moved=moved, elapsed=elapsed):
                self.get_logger().warn(
                    f'Nav2 false SUCCEEDED on {goal_xy}: moved={moved:.2f}m '
                    f'elapsed={elapsed:.1f}s dist={dist} '
                    f'travel={self._active_goal_travel_m}; holding same goal.'
                )
                if self._schedule_same_goal_retry('success_too_soon'):
                    return
                self._finalize_goal_failure(goal_xy, reason='success_too_soon')
                return
            self.get_logger().info(
                f'Reached frontier {goal_xy}'
                + (f' (dist={dist:.2f}m, moved={moved:.2f}m, {elapsed:.1f}s)' if dist is not None else '')
                + '; pausing before next.'
            )
            self.consecutive_failures = 0
            self._active_strategy = 'normal'
            self._goal_retry_count = 0
            self._stuck_escape_attempts = 0
            self._start_post_goal_pause()
        else:
            self.get_logger().warn(
                f'Goal {goal_xy} failed (status={status}).'
            )
            if self._schedule_same_goal_retry(f'status_{status}', status=status):
                return
            self._finalize_goal_failure(
                goal_xy, reason=f'status_{status}', status=status,
            )
            return
        self._clear_active_goal()

    def fail_active_goal(self, reason: str = 'failed', *, force: bool = False):
        if self._stuck_escape_active:
            if not force and self._schedule_same_goal_retry(reason):
                return
            self._clear_active_goal()
            return
        if self._bootstrap_goal_active:
            if not force and self._schedule_same_goal_retry(reason):
                return
            self._finish_bootstrap_goal(succeeded=False)
            return
        goal_xy = self.active_goal_xy
        if not force and self._schedule_same_goal_retry(reason):
            return
        self._finalize_goal_failure(goal_xy, reason=reason)

    def _finish_bootstrap_goal(self, *, succeeded: bool) -> None:
        self._bootstrap_goal_active = False
        self._bootstrap_forward_pending = False
        if succeeded:
            self.consecutive_failures = 0
            self._active_strategy = 'normal'
        self._clear_active_goal()

    def _clear_active_goal(self):
        self.active_goal_handle = None
        self.active_goal_xy = None
        self.active_goal_yaw = None
        self.active_goal_send_time = None
        self._goal_send_robot_xy = None
        self._retry_resend_at = None
        self._stuck_escape_active = False
        self._active_goal_travel_m = None
        self.active_clusters_snapshot = None
        self.active_chosen_snapshot = None


def main():
    rclpy.init()
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
