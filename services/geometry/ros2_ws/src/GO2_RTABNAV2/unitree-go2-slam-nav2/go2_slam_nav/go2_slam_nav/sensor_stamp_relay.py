import math
from collections import deque
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster


def _rotation_matrix_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def _quat_to_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-6:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])


def _stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class SensorStampRelay(Node):
    """Re-stamp GO2 sensor data and publish RTAB-Map-friendly body-frame clouds."""

    def __init__(self):
        super().__init__('sensor_stamp_relay')

        self.declare_parameter('cloud_in', '/utlidar/cloud_deskewed')
        self.declare_parameter('cloud_out', '/go2/cloud')
        self.declare_parameter('odom_in', '/utlidar/robot_odom')
        self.declare_parameter('odom_out', '/go2/odom')
        self.declare_parameter('republish_cloud', True)
        self.declare_parameter('republish_odom', True)
        self.declare_parameter('publish_odom_tf', True)
        self.declare_parameter('cloud_frame_mode', 'deskewed')
        self.declare_parameter('deskewed_max_odom_delta', 0.25)
        self.declare_parameter('odom_buffer_size', 60)
        # GO2 onboard stamps often differ from host clock; default wall-time restamp.
        self.declare_parameter('use_source_stamp', False)
        self.declare_parameter('deskewed_sync_odom', True)
        self.declare_parameter('transform_cloud_to_base', False)
        self.declare_parameter('flatten_odom_3dof', True)
        self.declare_parameter('odom_frame_id', 'go2_odom')
        self.declare_parameter('base_frame_id', 'go2_base_link')
        self.declare_parameter('lidar_extrinsic_x', 0.28945)
        self.declare_parameter('lidar_extrinsic_y', 0.0)
        self.declare_parameter('lidar_extrinsic_z', -0.046825)
        self.declare_parameter('lidar_extrinsic_roll', 0.0)
        self.declare_parameter('lidar_extrinsic_pitch', 2.8782)
        self.declare_parameter('lidar_extrinsic_yaw', 0.0)

        self.publish_odom_tf = _as_bool(self.get_parameter('publish_odom_tf').value)
        self.flatten_odom_3dof = _as_bool(self.get_parameter('flatten_odom_3dof').value)
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.cloud_frame_mode = str(self.get_parameter('cloud_frame_mode').value).lower()
        self.deskewed_max_odom_delta = float(self.get_parameter('deskewed_max_odom_delta').value)
        self.odom_buffer_size = int(self.get_parameter('odom_buffer_size').value)
        self.use_source_stamp = _as_bool(self.get_parameter('use_source_stamp').value)
        self.deskewed_sync_odom = _as_bool(self.get_parameter('deskewed_sync_odom').value)
        self.tf_broadcaster = TransformBroadcaster(self)
        self._odom_buffer = deque(maxlen=max(self.odom_buffer_size, 2))
        self._clock_skew_logged = False

        if _as_bool(self.get_parameter('transform_cloud_to_base').value):
            self.cloud_frame_mode = 'raw'

        r_base_to_lidar = _rotation_matrix_rpy(
            self.get_parameter('lidar_extrinsic_roll').value,
            self.get_parameter('lidar_extrinsic_pitch').value,
            self.get_parameter('lidar_extrinsic_yaw').value,
        )
        self._lidar_translation = np.array([
            self.get_parameter('lidar_extrinsic_x').value,
            self.get_parameter('lidar_extrinsic_y').value,
            self.get_parameter('lidar_extrinsic_z').value,
        ], dtype=np.float64)
        self._base_from_lidar_rotation = r_base_to_lidar

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        if self.get_parameter('republish_odom').value:
            odom_out = self.get_parameter('odom_out').value
            self.odom_pub = self.create_publisher(Odometry, odom_out, qos)
            self.create_subscription(
                Odometry,
                self.get_parameter('odom_in').value,
                self._odom_cb,
                qos,
            )
            self.get_logger().info(
                f'Odom relay: {self.get_parameter("odom_in").value} -> {odom_out} '
                f'(flatten_3dof={self.flatten_odom_3dof}, sync={self.deskewed_sync_odom})'
            )

        if self.get_parameter('republish_cloud').value:
            cloud_out = self.get_parameter('cloud_out').value
            self.cloud_pub = self.create_publisher(PointCloud2, cloud_out, qos)
            self.create_subscription(
                PointCloud2,
                self.get_parameter('cloud_in').value,
                self._cloud_cb,
                qos,
            )
            self.get_logger().info(
                f'Cloud relay: {self.get_parameter("cloud_in").value} -> {cloud_out} '
                f'(mode={self.cloud_frame_mode}, use_source_stamp={self.use_source_stamp})'
            )

    def _wall_stamp(self):
        return self.get_clock().now().to_msg()

    def _output_stamp(self, source_stamp):
        if self.use_source_stamp:
            wall = self.get_clock().now()
            skew = abs(_stamp_to_sec(source_stamp) - wall.nanoseconds * 1e-9)
            if skew > 1.0:
                if not self._clock_skew_logged:
                    self.get_logger().warn(
                        f'GO2/cloud stamp skew {skew:.1f}s vs host — using wall time '
                        f'(set use_source_stamp:=false to silence)'
                    )
                    self._clock_skew_logged = True
                return self._wall_stamp()
            return source_stamp
        return self._wall_stamp()

    def _make_cloud_header(self, stamp) -> Header:
        header = Header()
        header.stamp = stamp
        header.frame_id = self.base_frame_id
        return header

    def _nearest_odom(self, cloud_stamp) -> Optional[Odometry]:
        if not self._odom_buffer:
            return None

        cloud_time = _stamp_to_sec(cloud_stamp)
        best = self._odom_buffer[0]
        best_dt = abs(_stamp_to_sec(best.header.stamp) - cloud_time)

        for odom in list(self._odom_buffer)[1:]:
            dt = abs(_stamp_to_sec(odom.header.stamp) - cloud_time)
            if dt < best_dt:
                best = odom
                best_dt = dt

        if best_dt > self.deskewed_max_odom_delta:
            self.get_logger().warn(
                f'deskewed cloud skipped: cloud/odom delta {best_dt * 1000.0:.1f} ms > '
                f'{self.deskewed_max_odom_delta * 1000.0:.0f} ms',
                throttle_duration_sec=5.0,
            )
            return None
        return best

    def _transform_odom_cloud_to_base(self, msg: PointCloud2, odom: Odometry) -> list:
        pose = odom.pose.pose
        rot = _quat_to_rotation_matrix(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        trans = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=np.float64)
        rot_body_from_odom = rot.T

        field_names = [field.name for field in msg.fields]
        points = list(pc2.read_points(msg, field_names=field_names, skip_nans=False))
        transformed = []
        for point in points:
            values = list(point)
            x, y, z = values[0], values[1], values[2]
            if not np.isfinite(x) or not np.isfinite(y) or not np.isfinite(z):
                transformed.append(tuple(values))
                continue
            p_base = rot_body_from_odom @ (np.array([x, y, z], dtype=np.float64) - trans)
            values[0], values[1], values[2] = float(p_base[0]), float(p_base[1]), float(p_base[2])
            transformed.append(tuple(values))
        return transformed

    def _transform_raw_cloud_to_base(self, msg: PointCloud2) -> list:
        field_names = [field.name for field in msg.fields]
        points = list(pc2.read_points(msg, field_names=field_names, skip_nans=False))
        transformed = []
        for point in points:
            values = list(point)
            x, y, z = values[0], values[1], values[2]
            if not np.isfinite(x) or not np.isfinite(y) or not np.isfinite(z):
                transformed.append(tuple(values))
                continue
            p_lidar = np.array([x, y, z], dtype=np.float64)
            p_base = self._base_from_lidar_rotation @ p_lidar + self._lidar_translation
            values[0], values[1], values[2] = float(p_base[0]), float(p_base[1]), float(p_base[2])
            transformed.append(tuple(values))
        return transformed

    def _build_odom_msg(self, msg: Odometry, stamp) -> Odometry:
        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = self.odom_frame_id
        out.child_frame_id = self.base_frame_id
        out.pose = msg.pose
        out.twist = msg.twist

        if self.flatten_odom_3dof:
            q = msg.pose.pose.orientation
            yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)
            out.pose.pose.position.z = 0.0
            out.pose.pose.orientation.x = 0.0
            out.pose.pose.orientation.y = 0.0
            out.pose.pose.orientation.z = math.sin(yaw * 0.5)
            out.pose.pose.orientation.w = math.cos(yaw * 0.5)
        return out

    def _publish_odom_tf(self, out: Odometry, stamp) -> None:
        if not self.publish_odom_tf:
            return
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame_id
        transform.child_frame_id = self.base_frame_id
        transform.transform.translation.x = out.pose.pose.position.x
        transform.transform.translation.y = out.pose.pose.position.y
        transform.transform.translation.z = out.pose.pose.position.z
        transform.transform.rotation = out.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def _publish_synced_odom(self, odom: Odometry, stamp) -> None:
        if not hasattr(self, 'odom_pub'):
            return
        out = self._build_odom_msg(odom, stamp)
        self.odom_pub.publish(out)
        self._publish_odom_tf(out, stamp)

    def _cloud_cb(self, msg: PointCloud2):
        stamp = self._output_stamp(msg.header.stamp)
        header = self._make_cloud_header(stamp)

        if self.cloud_frame_mode == 'deskewed':
            odom = self._nearest_odom(msg.header.stamp)
            if odom is None:
                if not self._odom_buffer:
                    self.get_logger().warn(
                        'deskewed cloud skipped: no robot_odom in buffer yet',
                        throttle_duration_sec=5.0,
                    )
                return
            transformed = self._transform_odom_cloud_to_base(msg, odom)
            out = pc2.create_cloud(header, msg.fields, transformed)
            out.is_dense = msg.is_dense
            self.cloud_pub.publish(out)
            if self.deskewed_sync_odom:
                self._publish_synced_odom(odom, stamp)
            return

        if self.cloud_frame_mode == 'raw':
            transformed = self._transform_raw_cloud_to_base(msg)
            out = pc2.create_cloud(header, msg.fields, transformed)
            out.is_dense = msg.is_dense
            self.cloud_pub.publish(out)
            return

        out = PointCloud2()
        out.header = header
        out.height = msg.height
        out.width = msg.width
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.data = msg.data
        out.is_dense = msg.is_dense
        self.cloud_pub.publish(out)

    def _odom_cb(self, msg: Odometry):
        self._odom_buffer.append(msg)

        if self.cloud_frame_mode == 'deskewed' and self.deskewed_sync_odom:
            return

        stamp = self._output_stamp(msg.header.stamp)
        out = self._build_odom_msg(msg, stamp)
        self.odom_pub.publish(out)
        self._publish_odom_tf(out, stamp)


def main(args=None):
    rclpy.init(args=args)
    node = SensorStampRelay()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()