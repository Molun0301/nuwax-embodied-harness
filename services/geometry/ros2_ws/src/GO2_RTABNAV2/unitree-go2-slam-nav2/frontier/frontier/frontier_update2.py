import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from action_msgs.msg import GoalStatusArray
from visualization_msgs.msg import Marker, MarkerArray
from scipy.ndimage import binary_erosion
from tf2_ros import Buffer, TransformListener
from enum import Enum
import numpy as np
import math

class State(Enum):
    IDLE = 1
    EXPLORING = 2
    MOVING = 3

def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class Exploration(Node):
    """
    A ROS2 node for autonomous exploration using a frontier (border) approach.
    
    Instead of evaluating every free cell based on information gain, this version first
    finds unvisited free cells that form a frontier (i.e. cells adjacent to visited cells).
    Then, it selects from those frontier cells a candidate goal. In particular, it prefers
    candidates that lie within the sensor's forward field-of-view and chooses the farthest one,
    so as to “push” the exploration boundary outward.
    """
    def __init__(self):
        super().__init__('frontier_exploration')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_topic', 'goal_pose')
        self.declare_parameter(
            'navigate_status_topic', '/navigate_to_pose/_action/status')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'go2_base_link')
        self.declare_parameter('tf_timeout', 0.5)
        self.declare_parameter('timer_period', 5.0)
        self.declare_parameter('min_goal_distance', 1.0)
        self.declare_parameter('sensor_fov_deg', 100.0)
        self.declare_parameter('sensor_range', 1.0)
        self.declare_parameter('downsample_factor', 2)

        map_topic = self.get_parameter('map_topic').value
        goal_topic = self.get_parameter('goal_topic').value
        navigate_status_topic = self.get_parameter('navigate_status_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.tf_timeout = self.get_parameter('tf_timeout').value
        timer_period = self.get_parameter('timer_period').value
        self.min_goal_distance = self.get_parameter('min_goal_distance').value
        self.SENSOR_FOV = math.radians(self.get_parameter('sensor_fov_deg').value)
        self.sensor_range = self.get_parameter('sensor_range').value
        self.downsample_factor = self.get_parameter('downsample_factor').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.robot_pose_valid = False

        # Subscribers
        self.map_subscriber = self.create_subscription(
            OccupancyGrid, map_topic, self.map_update, 10)
        self.goal_status_subscriber = self.create_subscription(
            GoalStatusArray, navigate_status_topic, self.goal_status, 10)

        # Publishers
        self.goal_publisher = self.create_publisher(PoseStamped, goal_topic, 10)
        self.marker_publisher = self.create_publisher(MarkerArray, 'candidate_markers', 10)

        # Variables
        self.map_array = None         # Full-resolution occupancy grid (reshaped from msg.data)
        self.map_info = None          # Map meta-information
        self.downsampled_map = None   # Binary free-space map (downsampled & eroded)
        self.visited_grid = None      # Same shape as downsampled_map; visited cells marked as 1
        self.state = State.IDLE
        self.goal_reached = True
        self.robot_x_pose = 0.0
        self.robot_y_pose = 0.0

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.robot_orientation = 0.0

        self.get_logger().info(
            f'Frontier exploration ready: map={map_topic}, pose_tf='
            f'{self.map_frame}<-{self.base_frame}, goal={goal_topic}, '
            f'nav_status={navigate_status_topic}, '
            f'min_goal_distance={self.min_goal_distance}m'
        )

    def map_update(self, msg):
        """Updates the map, creates a binary free-space map, downsamples and erodes it."""
        self.map_array = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info

        # Create a binary free-space map:
        # OccupancyGrid: 0 => free, -1 => unknown, 100 => occupied.
        binary_map = np.zeros_like(self.map_array, dtype=bool)
        binary_map[self.map_array == 0] = True

        # Downsample for efficiency.
        self.downsampled_map = self.downsample_map(binary_map, self.downsample_factor)

        # Erode to ensure candidates lie well inside free space.
        if self.downsampled_map is not None:
            struct_elem = np.ones((3, 3), dtype=bool)
            self.downsampled_map = binary_erosion(self.downsampled_map, structure=struct_elem)
        self.downsampled_map = self.downsampled_map.astype(int)

        if self.downsampled_map is None:
            return

        if (
            self.visited_grid is None
            or self.visited_grid.shape != self.downsampled_map.shape
        ):
            self.visited_grid = np.zeros_like(self.downsampled_map, dtype=int)
            if self.robot_pose_valid:
                self.mark_area_visited(sensor_range=self.sensor_range)

    def downsample_map(self, map_array, factor):
        """Downsamples the given 2D boolean map by the specified factor."""
        height, width = map_array.shape
        new_height, new_width = height // factor, width // factor
        downsampled = np.zeros((new_height, new_width), dtype=map_array.dtype)
        for y in range(new_height):
            for x in range(new_width):
                cell_mean = np.mean(map_array[y*factor:(y+1)*factor, x*factor:(x+1)*factor])
                downsampled[y, x] = 1 if cell_mean > 0.5 else 0
        return downsampled

    def timer_callback(self):
        """Periodically updates the state and selects a new goal if needed."""
        if not self.refresh_robot_pose_from_tf():
            self.get_logger().warn(
                'Robot pose unavailable; waiting for map TF before exploring.',
                throttle_duration_sec=5.0,
            )
            return

        if self.state == State.IDLE:
            self.state = State.EXPLORING

        elif self.state == State.EXPLORING:
            if self.goal_reached:
                frontier = self.find_frontier_candidates()
                if frontier:
                    candidate = self.select_frontier_candidate(frontier)
                    if candidate:
                        self.publish_goal(candidate)
                        self.publish_candidate_markers(frontier)
                    else:
                        self.get_logger().info("No valid frontier candidate selected.")
                else:
                    self.get_logger().info("No frontier candidates detected. Exploration may be complete.")
            else:
                self.get_logger().info("Waiting for current goal to be reached.")

        elif self.state == State.MOVING:
            if self.goal_reached:
                self.state = State.EXPLORING

    def goal_status(self, msg):
        """Handles updates from the Nav2 navigate_to_pose action status topic."""
        if msg.status_list:
            current_status = msg.status_list[-1].status
            if current_status == 2:  # ACTIVE
                self.goal_reached = False
            elif current_status in (4, 5, 6):  # SUCCEEDED, CANCELED, ABORTED
                self.goal_reached = True
                self.state = State.EXPLORING

    def refresh_robot_pose_from_tf(self):
        """Lookup robot pose in map frame and refresh the visited grid."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except Exception as exc:
            self.get_logger().debug(f'TF lookup failed: {exc}')
            return False

        self.robot_x_pose = transform.transform.translation.x
        self.robot_y_pose = transform.transform.translation.y
        self.robot_orientation = yaw_from_quaternion(transform.transform.rotation)
        self.robot_pose_valid = True
        self.update_visited_grid(self.robot_x_pose, self.robot_y_pose)
        self.mark_area_visited(sensor_range=self.sensor_range)
        return True

    def update_visited_grid(self, x, y):
        """Marks the cell corresponding to the robot's current position as visited in the downsampled visited grid."""
        if self.visited_grid is None or self.map_info is None:
            return
        grid_resolution = self.map_info.resolution * self.downsample_factor
        grid_x = int((x - self.map_info.origin.position.x) / grid_resolution)
        grid_y = int((y - self.map_info.origin.position.y) / grid_resolution)
        if 0 <= grid_x < self.visited_grid.shape[1] and 0 <= grid_y < self.visited_grid.shape[0]:
            self.visited_grid[grid_y, grid_x] = 1

    def mark_area_visited(self, sensor_range=1):
        """
        Marks cells within sensor_range (in meters) of the robot's current position as visited
        in the downsampled visited grid.
        """
        if self.visited_grid is None or self.map_info is None:
            return
        grid_resolution = self.map_info.resolution * self.downsample_factor
        grid_x = int((self.robot_x_pose - self.map_info.origin.position.x) / grid_resolution)
        grid_y = int((self.robot_y_pose - self.map_info.origin.position.y) / grid_resolution)
        grid_range = int(np.ceil(sensor_range / grid_resolution))
        for i in range(-grid_range, grid_range + 1):
            for j in range(-grid_range, grid_range + 1):
                cell_x = grid_x + i
                cell_y = grid_y + j
                if 0 <= cell_x < self.visited_grid.shape[1] and 0 <= cell_y < self.visited_grid.shape[0]:
                    if np.hypot(i * grid_resolution, j * grid_resolution) <= sensor_range:
                        self.visited_grid[cell_y, cell_x] = 1

    def grid_to_world(self, grid_x, grid_y, downsampled=True):
        resolution = self.map_info.resolution
        if downsampled:
            resolution *= self.downsample_factor
        return (
            grid_x * resolution + self.map_info.origin.position.x + resolution / 2.0,
            grid_y * resolution + self.map_info.origin.position.y + resolution / 2.0,
        )

    def find_visited_frontier_candidates(self):
        """Free cells adjacent to visited cells in the downsampled map."""
        candidates = []
        if self.visited_grid is None or self.downsampled_map is None:
            return candidates

        height, width = self.visited_grid.shape
        for y in range(height):
            for x in range(width):
                if self.downsampled_map[y, x] != 1 or self.visited_grid[y, x] != 0:
                    continue
                neighbor_found = False
                for j in range(max(0, y - 1), min(height, y + 2)):
                    for i in range(max(0, x - 1), min(width, x + 2)):
                        if i == x and j == y:
                            continue
                        if self.visited_grid[j, i] == 1:
                            neighbor_found = True
                            break
                    if neighbor_found:
                        break
                if neighbor_found:
                    candidates.append(self.grid_to_world(x, y, downsampled=True))
        return candidates

    def find_classic_frontier_candidates(self):
        """Unknown cells adjacent to free space in the occupancy grid."""
        candidates = []
        if self.map_array is None or self.map_info is None:
            return candidates

        height, width = self.map_array.shape
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if self.map_array[y, x] != -1:
                    continue
                neighborhood = self.map_array[y - 1:y + 2, x - 1:x + 2]
                if np.any(neighborhood == 0):
                    candidates.append(self.grid_to_world(x, y, downsampled=False))
        return candidates

    def find_frontier_candidates(self):
        visited_frontiers = self.find_visited_frontier_candidates()
        if visited_frontiers:
            self.get_logger().info(
                f'Detected {len(visited_frontiers)} visited-frontier candidates.'
            )
            return visited_frontiers

        classic_frontiers = self.find_classic_frontier_candidates()
        self.get_logger().info(
            f'Detected {len(classic_frontiers)} classic frontier candidates.'
        )
        return classic_frontiers

    def select_frontier_candidate(self, frontier):
        """
        Select a frontier goal in map frame.
        Prefer candidates in the forward sensor arc, then farthest beyond min distance.
        """
        if not frontier:
            return None
        candidate_worlds = list(frontier)

        # Filter candidates within the sensor's forward field-of-view.
        candidates_in_fov = []
        for candidate in candidate_worlds:
            dx = candidate[0] - self.robot_x_pose
            dy = candidate[1] - self.robot_y_pose
            candidate_angle = math.atan2(dy, dx)
            dtheta = abs(math.atan2(math.sin(candidate_angle - self.robot_orientation),
                                    math.cos(candidate_angle - self.robot_orientation)))
            if dtheta <= self.SENSOR_FOV/2:
                candidates_in_fov.append(candidate)
        
        # Define a helper to compute Euclidean distance.
        def dist(candidate):
            return math.hypot(candidate[0] - self.robot_x_pose, candidate[1] - self.robot_y_pose)
        
        pool = candidates_in_fov if candidates_in_fov else candidate_worlds
        reachable = [c for c in pool if dist(c) >= self.min_goal_distance]
        if not reachable:
            self.get_logger().info(
                f'No frontier candidate beyond {self.min_goal_distance:.2f} m minimum.'
            )
            return None

        chosen = max(reachable, key=dist)
        self.get_logger().info(
            f'Selected frontier goal: {chosen} (distance {dist(chosen):.2f} m)'
        )
        return chosen

    def publish_goal(self, goal):
        """Publishes a new goal to the goal topic."""
        if goal:
            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = self.map_frame
            goal_msg.pose.position = Point(x=goal[0], y=goal[1], z=0.0)
            self.goal_publisher.publish(goal_msg)
            self.state = State.MOVING
            self.get_logger().info(f"Published new goal: {goal}")

    def publish_candidate_markers(self, candidates):
        """Publishes markers for visualization of frontier candidate cells."""
        marker_array = MarkerArray()
        marker_id = 0
        marker_size = self.map_info.resolution * self.downsample_factor * 0.8
        for (world_x, world_y) in candidates:
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = world_x
            marker.pose.position.y = world_y
            marker.pose.position.z = 0.0
            marker.scale.x = marker_size
            marker.scale.y = marker_size
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker_array.markers.append(marker)
            marker_id += 1
        self.marker_publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    exploration_node = Exploration()
    rclpy.spin(exploration_node)
    exploration_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

