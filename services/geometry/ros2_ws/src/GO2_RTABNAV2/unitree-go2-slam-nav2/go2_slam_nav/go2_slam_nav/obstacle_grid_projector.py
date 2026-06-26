"""Fuse RTAB-Map /map with /cloud_obstacles into a 2D grid for Nav2 and explore_lite."""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener


def _cloud_xyz_arrays(cloud: np.ndarray):
    """Return x/y/z float arrays from structured or Nx3 point clouds."""
    if cloud.size == 0:
        return (
            np.array([], dtype=np.float64),
            np.array([], dtype=np.float64),
            np.array([], dtype=np.float64),
        )

    if cloud.dtype.names and all(name in cloud.dtype.names for name in ('x', 'y', 'z')):
        return (
            np.asarray(cloud['x'], dtype=np.float64),
            np.asarray(cloud['y'], dtype=np.float64),
            np.asarray(cloud['z'], dtype=np.float64),
        )

    arr = np.asarray(cloud, dtype=np.float64)
    if arr.ndim == 2 and arr.shape[1] >= 3:
        return arr[:, 0], arr[:, 1], arr[:, 2]

    raise ValueError(f'unsupported point cloud array shape/dtype: {arr.shape} {cloud.dtype}')


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


class ObstacleGridProjector(Node):
    """
    Build /map_obstacles from RTAB-Map /map (free/unknown) plus 3D obstacle points.

    Obstacle cells come from /cloud_obstacles projection so 2D matches the 3D view.
    Optional live /go2/cloud overlay gives sub-second nearby obstacle feedback.
    """

    def __init__(self):
        super().__init__('obstacle_grid_projector')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('cloud_obstacles_topic', '/cloud_obstacles')
        self.declare_parameter('live_cloud_topic', '/go2/cloud')
        self.declare_parameter('live_cloud_enabled', True)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_base_frame', 'go2_base_link')
        self.declare_parameter('output_topic', '/map_obstacles')
        self.declare_parameter('output_updates_topic', '/map_obstacles_updates')
        self.declare_parameter('min_obstacle_height', 0.04)
        self.declare_parameter('max_obstacle_height', 1.5)
        self.declare_parameter('obstacle_value', 100)
        self.declare_parameter('inflate_cells', 1)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('publish_on_cloud', True)
        self.declare_parameter('min_publish_interval', 0.1)
        self.declare_parameter('live_cloud_max_range', 6.0)
        self.declare_parameter('max_cloud_points', 25000)
        self.declare_parameter('max_live_cloud_points', 12000)
        self.declare_parameter('transform_tolerance', 0.3)

        map_topic = self.get_parameter('map_topic').value
        cloud_topic = self.get_parameter('cloud_obstacles_topic').value
        live_cloud_topic = self.get_parameter('live_cloud_topic').value
        output_topic = self.get_parameter('output_topic').value
        output_updates_topic = self.get_parameter('output_updates_topic').value
        publish_rate = float(self.get_parameter('publish_rate').value)

        self._live_cloud_enabled = bool(self.get_parameter('live_cloud_enabled').value)
        self._publish_on_cloud = bool(self.get_parameter('publish_on_cloud').value)
        self._min_publish_interval = max(
            0.05, float(self.get_parameter('min_publish_interval').value))
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._base_frame = str(self.get_parameter('robot_base_frame').value)
        self._transform_tolerance = float(self.get_parameter('transform_tolerance').value)

        self._last_cloud = None
        self._last_live_cloud = None
        self._map_msg = None
        self._dirty = False
        self._last_published_size = (0, 0)
        self._last_publish_mono = 0.0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        cloud_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        out_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(OccupancyGrid, map_topic, self._map_cb, map_qos)
        self.create_subscription(PointCloud2, cloud_topic, self._cloud_cb, cloud_qos)
        if self._live_cloud_enabled and live_cloud_topic:
            self.create_subscription(
                PointCloud2, live_cloud_topic, self._live_cloud_cb, cloud_qos)

        update_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pub = self.create_publisher(OccupancyGrid, output_topic, out_qos)
        self._updates_pub = self.create_publisher(
            OccupancyGridUpdate, output_updates_topic, update_qos)

        period = 1.0 / publish_rate if publish_rate > 0.0 else 0.1
        self.create_timer(period, self._timer_publish)

        self.get_logger().info(
            f'Obstacle grid: {map_topic} + {cloud_topic}'
            f'{" + " + live_cloud_topic if self._live_cloud_enabled else ""} '
            f'-> {output_topic} @ {publish_rate:.1f} Hz '
            f'(cloud-triggered={self._publish_on_cloud})'
        )

    def _map_cb(self, msg: OccupancyGrid):
        prev = self._map_msg
        self._map_msg = msg
        self._dirty = True
        size_changed = (
            prev is None
            or prev.info.width != msg.info.width
            or prev.info.height != msg.info.height
            or prev.info.origin.position.x != msg.info.origin.position.x
            or prev.info.origin.position.y != msg.info.origin.position.y
        )
        if size_changed:
            self._publish(force=True)
        else:
            self._request_publish()

    def _cloud_cb(self, msg: PointCloud2):
        self._last_cloud = msg
        self._dirty = True
        if self._publish_on_cloud:
            self._request_publish()

    def _live_cloud_cb(self, msg: PointCloud2):
        self._last_live_cloud = msg
        self._dirty = True
        if self._publish_on_cloud:
            self._request_publish()

    def _timer_publish(self):
        if self._dirty:
            self._request_publish(force=True)

    def _request_publish(self, force: bool = False):
        if self._map_msg is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if not force and (now - self._last_publish_mono) < self._min_publish_interval:
            return
        self._publish(force=True)

    def _read_cloud_points(self, cloud_msg: PointCloud2, max_points: int):
        try:
            cloud = pc2.read_points_numpy(
                cloud_msg, field_names=('x', 'y', 'z'), skip_nans=True)
        except Exception:
            cloud = np.array(
                list(pc2.read_points(
                    cloud_msg, field_names=('x', 'y', 'z'), skip_nans=True)),
                dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')],
            )

        n = len(cloud)
        if n == 0:
            return None, None, None

        if n > max_points > 0:
            idx = np.linspace(0, n - 1, max_points, dtype=np.int64)
            cloud = cloud[idx]

        return _cloud_xyz_arrays(cloud)

    def _map_from_base_xy(self, xs: np.ndarray, ys: np.ndarray):
        tf_stamped = self._tf_buffer.lookup_transform(
            self._map_frame,
            self._base_frame,
            Time(),
            timeout=rclpy.duration.Duration(
                seconds=self._transform_tolerance))
        t = tf_stamped.transform.translation
        q = tf_stamped.transform.rotation
        yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        xm = cos_y * xs - sin_y * ys + t.x
        ym = sin_y * xs + cos_y * ys + t.y
        return xm, ym

    def _project_points(
        self,
        cells: np.ndarray,
        info,
        xs: np.ndarray,
        ys: np.ndarray,
        zs: np.ndarray,
        obs_val: int,
        inflate: int,
    ) -> int:
        min_z = float(self.get_parameter('min_obstacle_height').value)
        max_z = float(self.get_parameter('max_obstacle_height').value)

        mask = (zs >= min_z) & (zs <= max_z)
        if not np.any(mask):
            return 0

        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        width = info.width
        height = info.height

        mx = ((xs[mask] - ox) / res).astype(np.int32)
        my = ((ys[mask] - oy) / res).astype(np.int32)
        valid = (mx >= 0) & (mx < width) & (my >= 0) & (my < height)
        if not np.any(valid):
            return 0

        mx = mx[valid]
        my = my[valid]
        flat = my * width + mx
        cells[flat] = obs_val

        if inflate > 0:
            for di in range(-inflate, inflate + 1):
                for dj in range(-inflate, inflate + 1):
                    if di == 0 and dj == 0:
                        continue
                    cx = mx + di
                    cy = my + dj
                    inb = (cx >= 0) & (cx < width) & (cy >= 0) & (cy < height)
                    cells[cy[inb] * width + cx[inb]] = obs_val

        return int(len(flat))

    def _project_cloud(self, cells: np.ndarray, info, obs_val: int, inflate: int) -> int:
        if self._last_cloud is None:
            return 0

        max_points = int(self.get_parameter('max_cloud_points').value)
        xs, ys, zs = self._read_cloud_points(self._last_cloud, max_points)
        if xs is None:
            return 0

        return self._project_points(cells, info, xs, ys, zs, obs_val, inflate)

    def _project_live_cloud(self, cells: np.ndarray, info, obs_val: int, inflate: int) -> int:
        if not self._live_cloud_enabled or self._last_live_cloud is None:
            return 0

        max_points = int(self.get_parameter('max_live_cloud_points').value)
        max_range = float(self.get_parameter('live_cloud_max_range').value)
        xs, ys, zs = self._read_cloud_points(self._last_live_cloud, max_points)
        if xs is None:
            return 0

        if max_range > 0.0:
            near = (xs * xs + ys * ys) <= (max_range * max_range)
            if not np.any(near):
                return 0
            xs, ys, zs = xs[near], ys[near], zs[near]

        try:
            xm, ym = self._map_from_base_xy(xs, ys)
        except Exception as exc:
            self.get_logger().warn(
                f'live cloud TF skipped: {exc}',
                throttle_duration_sec=5.0,
            )
            return 0

        return self._project_points(cells, info, xm, ym, zs, obs_val, inflate)

    def _publish(self, force: bool = False):
        if self._map_msg is None:
            return
        if not force and not self._dirty:
            return

        info = self._map_msg.info
        width = info.width
        height = info.height
        if width == 0 or height == 0:
            return

        obs_val = int(self.get_parameter('obstacle_value').value)
        inflate = max(0, int(self.get_parameter('inflate_cells').value))

        try:
            base = np.array(self._map_msg.data, dtype=np.int8, copy=True)
            cells = base.copy()
            cloud_count = self._project_cloud(cells, info, obs_val, inflate)
            live_count = self._project_live_cloud(cells, info, obs_val, inflate)
        except Exception as exc:
            self.get_logger().error(f'obstacle grid publish failed: {exc}')
            return

        stamp = self.get_clock().now().to_msg()

        out = OccupancyGrid()
        out.header = self._map_msg.header
        out.header.stamp = stamp
        out.info = info
        out.data = cells.tolist()
        self._pub.publish(out)

        current_size = (width, height)
        size_changed = current_size != self._last_published_size
        self._last_published_size = current_size

        if not size_changed:
            changed = np.flatnonzero(cells != base)
            if changed.size > 0:
                update = OccupancyGridUpdate()
                update.header = out.header
                update.x = 0
                update.y = 0
                update.width = width
                update.height = height
                update.data = cells.reshape(-1).tolist()
                self._updates_pub.publish(update)

        self._dirty = False
        self._last_publish_mono = self.get_clock().now().nanoseconds * 1e-9

        self.get_logger().info(
            f'已发布 /map_obstacles ({width}x{height})，'
            f'rtab 点云 {cloud_count}，实时 {live_count} 点',
            throttle_duration_sec=5.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleGridProjector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()