#!/usr/bin/env python3
"""
Frontier-based autonomous exploration for Nav2.

Periodically scans the OccupancyGrid published on /map and finds frontiers
(free cells adjacent to unknown space). Picks the best frontier cluster as
the next exploration goal and dispatches it to Nav2 via the
/navigate_to_pose action. Failed goals are blacklisted briefly so we don't
loop on the same unreachable spot.

Stops automatically when no frontiers remain (map fully explored).

Tunables (ROS parameters, all overridable from launch):
  - map_topic           : /map
  - global_frame        : map
  - robot_frame         : base_link
  - replan_period       : 3.0   seconds between exploration ticks
  - min_frontier_size   : 10    cells (after clustering) to be considered
  - distance_weight     : 0.5   alpha in score = size - alpha * distance_in_cells
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
from nav_msgs.msg import OccupancyGrid
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


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        # ---- Parameters
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('replan_period', 3.0)
        self.declare_parameter('min_frontier_size', 10)
        self.declare_parameter('distance_weight', 0.5)
        self.declare_parameter('blacklist_radius', 0.5)
        self.declare_parameter('blacklist_ttl', 60.0)
        self.declare_parameter('goal_yaw_to_robot', True)
        self.declare_parameter('max_consecutive_failures', 5)
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

        gp = self.get_parameter
        self.map_topic = gp('map_topic').value
        self.global_frame = gp('global_frame').value
        self.robot_frame = gp('robot_frame').value
        self.replan_period = float(gp('replan_period').value)
        self.min_frontier_size = int(gp('min_frontier_size').value)
        self.distance_weight = float(gp('distance_weight').value)
        self.blacklist_radius = float(gp('blacklist_radius').value)
        self.blacklist_ttl = float(gp('blacklist_ttl').value)
        self.goal_yaw_to_robot = bool(gp('goal_yaw_to_robot').value)
        self.max_consecutive_failures = int(gp('max_consecutive_failures').value)
        self.startup_grace_period = float(gp('startup_grace_period').value)
        self.goal_timeout = float(gp('goal_timeout').value)

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
        self.active_goal_send_time: Optional[float] = None
        self.consecutive_failures = 0
        self.exploration_done = False
        self._shutdown_requested = False

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

        # ---- Timer: re-evaluate periodically
        self.timer = self.create_timer(self.replan_period, self.tick)

        self.get_logger().info(
            f'FrontierExplorer started. map_topic={self.map_topic}, '
            f'replan={self.replan_period}s, '
            f'min_frontier_size={self.min_frontier_size} cells'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def map_cb(self, msg: OccupancyGrid):
        self.latest_map = msg

    def tick(self):
        if self.exploration_done:
            self.publish_status('DONE (exploration finished)')
            if not self._shutdown_requested:
                self._shutdown_requested = True
                self.get_logger().info('Exploration complete. Shutting down frontier_explorer node.')
                self.timer.cancel()
                self.destroy_node()
                rclpy.shutdown()
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
            # Already navigating; do not interrupt.
            self.publish_status(f'NAVIGATING ({elapsed:.0f}s)')
            return
        if self.consecutive_failures >= self.max_consecutive_failures:
            self.get_logger().warn(
                f'Stopping exploration: {self.consecutive_failures} consecutive '
                f'navigation failures. Map may be fully explored or blocked.'
            )
            self.exploration_done = True
            return

        # 1) Robot pose
        robot = self.get_robot_xy()
        if robot is None:
            self.get_logger().warn(
                'No TF map->base_link yet, skipping exploration tick.',
                throttle_duration_sec=5.0)
            return

        # 2) Frontier detection
        all_clusters = self.find_frontier_clusters(self.latest_map)
        if not all_clusters:
            self.publish_viz([], None)
            self.get_logger().info('No frontiers found. Exploration COMPLETE.')
            self.exploration_done = True
            return

        # 3) Filter out blacklisted clusters
        self.expire_blacklist()
        clusters = [c for c in all_clusters if not self.is_blacklisted(*c.centroid_world)]
        if not clusters:
            self.publish_viz(all_clusters, None)
            self.get_logger().info(
                'All frontiers are blacklisted; waiting for blacklist to expire.',
                throttle_duration_sec=5.0)
            return

        # 4) Score and pick best
        best = self.pick_best(clusters, robot)
        gx, gy = best.centroid_world
        self.get_logger().info(
            f'Selected frontier: ({gx:.2f},{gy:.2f}) size={best.size} '
            f'from {len(clusters)} candidates'
        )

        # 5) Visualize all candidates + chosen goal
        self.publish_viz(all_clusters, best)

        # 6) Send goal
        self.send_nav_goal(gx, gy, robot)

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
                  robot_xy: Tuple[float, float]) -> FrontierCluster:
        rx, ry = robot_xy
        res = self.latest_map.info.resolution
        # Normalize: distance is in meters; size is in cells. Convert distance
        # to "cell units" so the user-facing weight is dimensionally consistent.
        best = None
        best_score = -float('inf')
        for c in clusters:
            cx, cy = c.centroid_world
            d_m = math.hypot(cx - rx, cy - ry)
            d_cells = d_m / res if res > 1e-6 else d_m
            score = c.size - self.distance_weight * d_cells
            if score > best_score:
                best_score = score
                best = c
        return best

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
    def publish_viz(self,
                    clusters: List[FrontierCluster],
                    chosen: Optional[FrontierCluster]):
        """Publish RViz markers on /frontiers (MarkerArray)."""
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # 0) DELETEALL to clear stale markers from previous tick
        clear = Marker()
        clear.header.frame_id = self.global_frame
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        # 1) All frontier clusters as cyan SPHERE_LIST (one sphere per cluster,
        #    radius scaled by cluster size).
        if clusters:
            spheres = Marker()
            spheres.header.frame_id = self.global_frame
            spheres.header.stamp = stamp
            spheres.ns = 'frontier_clusters'
            spheres.id = 0
            spheres.type = Marker.SPHERE_LIST
            spheres.action = Marker.ADD
            spheres.pose.orientation.w = 1.0
            # SPHERE_LIST: per-marker scale is the *uniform* sphere diameter;
            # to get per-cluster size we publish each as separate Marker below.
            ma.markers.append(self._cluster_text_marker(clusters, stamp))
            for i, c in enumerate(clusters):
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
                # Diameter scaled with size, clamped to [0.15, 0.6] m.
                d = max(0.15, min(0.6, 0.05 + 0.01 * math.sqrt(c.size)))
                m.scale = Vector3(x=d, y=d, z=d)
                # Cyan unless chosen, then green
                is_chosen = (chosen is not None
                             and c.centroid_world == chosen.centroid_world)
                if is_chosen:
                    m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.95)
                else:
                    m.color = ColorRGBA(r=0.0, g=0.7, b=1.0, a=0.7)
                ma.markers.append(m)

        # 2) Chosen goal as a tall green ARROW pointing up.
        if chosen is not None:
            arrow = Marker()
            arrow.header.frame_id = self.global_frame
            arrow.header.stamp = stamp
            arrow.ns = 'frontier_goal'
            arrow.id = 0
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            gx, gy = chosen.centroid_world
            # Vertical arrow: from (gx,gy,1.0) down to (gx,gy,0.1)
            arrow.points = [
                Point(x=gx, y=gy, z=1.0),
                Point(x=gx, y=gy, z=0.1),
            ]
            arrow.scale = Vector3(x=0.05, y=0.12, z=0.0)  # shaft / head
            arrow.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            ma.markers.append(arrow)

        # 3) Blacklist points as red X (CUBE_LIST scaled small).
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

        self.viz_pub.publish(ma)

    def publish_status(self, text: str):
        """Always-on heartbeat: shows explorer's current state as text in RViz.
        Lets you visually confirm the node is alive even when no frontiers exist.
        """
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # Clear previous markers
        clear = Marker()
        clear.header.frame_id = self.global_frame
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        # Status text near origin (or robot if known)
        m = Marker()
        m.header.frame_id = self.global_frame
        m.header.stamp = stamp
        m.ns = 'frontier_status'
        m.id = 0
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        rxy = self.get_robot_xy()
        if rxy is not None:
            m.pose.position.x, m.pose.position.y = rxy
        m.pose.position.z = 1.5
        m.pose.orientation.w = 1.0
        m.scale.z = 0.3
        m.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)
        m.text = f'[explorer] {text}'
        ma.markers.append(m)

        self.viz_pub.publish(ma)

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
    def get_robot_xy(self) -> Optional[Tuple[float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_frame, Time())
        except TransformException:
            return None
        return (tf.transform.translation.x, tf.transform.translation.y)

    # ------------------------------------------------------------------
    # Nav2 action client
    # ------------------------------------------------------------------
    def send_nav_goal(self, gx: float, gy: float,
                      robot_xy: Tuple[float, float]):
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
        rx, ry = robot_xy
        yaw = math.atan2(gy - ry, gx - rx)
        goal_msg.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)
        )

        self.active_goal_xy = (gx, gy)
        self.active_goal_send_time = self.get_clock().now().nanoseconds * 1e-9
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
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'Reached frontier {self.active_goal_xy}, looking for next.'
            )
            self.consecutive_failures = 0
        else:
            self.get_logger().warn(
                f'Goal {self.active_goal_xy} failed (status={status}); blacklisting.'
            )
            if self.active_goal_xy is not None:
                self.add_to_blacklist(*self.active_goal_xy)
            self.consecutive_failures += 1
        self.active_goal_handle = None
        self.active_goal_xy = None
        self.active_goal_send_time = None

    def fail_active_goal(self):
        if self.active_goal_xy is not None:
            self.add_to_blacklist(*self.active_goal_xy)
        self.active_goal_handle = None
        self.active_goal_xy = None
        self.active_goal_send_time = None
        self.consecutive_failures += 1


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
