#!/usr/bin/env python3
"""
Frontier-based autonomous exploration for Nav2.

Periodically scans the OccupancyGrid published on /map and finds frontiers
(free cells adjacent to unknown space). It scores each frontier by a mix of
information gain, travel cost, turn cost, and local clearance, then dispatches
the best goal to Nav2 via the /navigate_to_pose action. Failed goals are
blacklisted briefly so we don't loop on the same unreachable spot.

Stops automatically when no frontiers remain (map fully explored).

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
        # Grace period after launch before sending the first goal. Nav2's
        # lifecycle manager reports 'active' before the bt_navigator finishes
        # wiring up its internal action clients (compute_path_to_pose,
        # follow_path, ...), so sending a goal immediately races and gets
        # rejected with 'send_goal failed'. Wait a few seconds to be safe.
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
        self.startup_grace_period = float(gp('startup_grace_period').value)
        self.goal_timeout = float(gp('goal_timeout').value)
        self.min_travel_m = float(gp('min_travel_m').value)
        self.min_goal_clearance_m = float(gp('min_goal_clearance_m').value)
        self.goal_success_tolerance = float(gp('goal_success_tolerance').value)
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

        if self.latest_map is None:
            self.get_logger().info(
                f'Waiting for first /map on topic "{self.map_topic}" '
                f'(check QoS / topic name)',
                throttle_duration_sec=5.0)
            self.publish_status('WAITING for /map')
            return

        # Watchdog: if a goal has been active too long, assume bt_navigator
        # is wedged (or has crashed silently) and recover. This prevents the
        # explorer from getting stuck forever when Nav2 dies mid-navigation.
        if self.active_goal_handle is not None:
            elapsed = (now - self.active_goal_send_time
                       if self.active_goal_send_time is not None else 0.0)
            if elapsed > self.goal_timeout:
                self.get_logger().warn(
                    f'Goal {self.active_goal_xy} timed out after {elapsed:.1f}s '
                    f'(>{self.goal_timeout:.0f}s). Cancelling and blacklisting; '
                    f'Nav2 may have crashed.'
                )
                try:
                    self.active_goal_handle.cancel_goal_async()
                except Exception as e:
                    self.get_logger().warn(f'cancel_goal_async failed: {e}')
                self.fail_active_goal()
                self.publish_status('TIMEOUT (recovering)')
                return
            # Already navigating; keep goal + plan visible in RViz.
            self.publish_markers(
                status=f'NAVIGATING ({elapsed:.0f}s)',
                clusters=self.active_clusters_snapshot,
                chosen=self.active_chosen_snapshot,
                navigating=True,
            )
            return
        if self.consecutive_failures >= self.max_consecutive_failures:
            self.get_logger().warn(
                f'Stopping exploration: {self.consecutive_failures} consecutive '
                f'navigation failures. Map may be fully explored or blocked.'
            )
            self.exploration_done = True
            self.publish_status('DONE (exploration finished)')
            return

        # 1) Robot pose
        robot = self.get_robot_pose()
        if robot is None:
            self.get_logger().warn(
                'No TF map->base_link yet, skipping exploration tick.',
                throttle_duration_sec=5.0)
            return
        rx, ry, ryaw = robot

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

        # 4) Score and pick best
        best = self.pick_best(all_clusters, robot)
        if best is None:
            self.publish_markers(
                status='WAITING (no reachable frontiers)',
                clusters=all_clusters,
            )
            self.get_logger().info(
                'No frontier goal meets min travel / blacklist filters; waiting.',
                throttle_duration_sec=5.0)
            return
        gx, gy = best.goal_world
        self.get_logger().info(
            f'Selected frontier: ({gx:.2f},{gy:.2f}) score={best.score:.2f} '
            f'gain={best.info_gain} size={best.cluster.size} '
            f'd={best.travel_m:.2f}m turn={math.degrees(best.heading_error):.0f}deg '
            f'clear={best.clearance_m:.2f}m from {len(all_clusters)} candidates'
        )

        # 5) Remember snapshot for RViz while navigating
        self.active_clusters_snapshot = all_clusters
        self.active_chosen_snapshot = best

        # 6) Visualize all candidates + chosen goal
        self.publish_markers(
            status='DISPATCHING GOAL',
            clusters=all_clusters,
            chosen=best,
        )

        # 7) Send goal
        self.send_nav_goal(gx, gy, (rx, ry, ryaw))

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
    def pick_best(self, clusters: List[FrontierCluster],
                  robot_pose: Tuple[float, float, float]) -> Optional[FrontierCandidate]:
        grid = np.array(self.latest_map.data, dtype=np.int16).reshape(
            self.latest_map.info.height, self.latest_map.info.width
        )
        info = self.latest_map.info
        best = None
        best_score = -float('inf')
        for c in clusters:
            candidate = self.evaluate_cluster(c, robot_pose, grid, info)
            if self.is_blacklisted(*candidate.goal_world):
                continue
            if candidate.travel_m < self.min_travel_m:
                continue
            if candidate.clearance_m < self.min_goal_clearance_m:
                continue
            if candidate.score > best_score:
                best_score = candidate.score
                best = candidate
        return best

    def evaluate_cluster(self, cluster: FrontierCluster,
                         robot_pose: Tuple[float, float, float],
                         grid: np.ndarray,
                         info) -> FrontierCandidate:
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
                self.travel_weight * travel -
                self.turn_weight * heading_error +
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
                self.travel_weight * best_travel -
                self.turn_weight * best_heading_error
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
                      robot_pose: Tuple[float, float, float]):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(
                'navigate_to_pose action server not available; will retry.')
            return

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

        next_goal = PoseStamped()
        next_goal.header = goal_msg.pose.header
        next_goal.pose = goal_msg.pose.pose
        self.next_goal_pub.publish(next_goal)

        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as e:
            self.get_logger().warn(f'Goal request error: {e}')
            self.fail_active_goal()
            return
        if not handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2.')
            self.fail_active_goal()
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
        goal_xy = self.active_goal_xy
        if status == GoalStatus.STATUS_SUCCEEDED:
            robot = self.get_robot_pose()
            dist = None
            if robot is not None and goal_xy is not None:
                dist = math.hypot(robot[0] - goal_xy[0], robot[1] - goal_xy[1])
            if dist is not None and dist > self.goal_success_tolerance:
                self.get_logger().warn(
                    f'Nav2 succeeded but robot is {dist:.2f}m from goal '
                    f'{goal_xy} (tolerance={self.goal_success_tolerance:.2f}m); '
                    f'treating as incomplete and blacklisting.'
                )
                if goal_xy is not None:
                    self.add_to_blacklist(*goal_xy)
                self.consecutive_failures += 1
            else:
                self.get_logger().info(
                    f'Reached frontier {goal_xy}'
                    + (f' (dist={dist:.2f}m)' if dist is not None else '')
                    + ', looking for next.'
                )
                self.consecutive_failures = 0
        else:
            self.get_logger().warn(
                f'Goal {goal_xy} failed (status={status}); blacklisting.'
            )
            if goal_xy is not None:
                self.add_to_blacklist(*goal_xy)
            self.consecutive_failures += 1
        self._clear_active_goal()

    def fail_active_goal(self):
        if self.active_goal_xy is not None:
            self.add_to_blacklist(*self.active_goal_xy)
        self._clear_active_goal()
        self.consecutive_failures += 1

    def _clear_active_goal(self):
        self.active_goal_handle = None
        self.active_goal_xy = None
        self.active_goal_yaw = None
        self.active_goal_send_time = None
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
