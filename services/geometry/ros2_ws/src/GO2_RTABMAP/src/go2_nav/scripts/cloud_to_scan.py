#!/usr/bin/env python3
"""
Convert a PointCloud2 (any frame) into a LaserScan in target_frame.

If input frame != target_frame, applies a tf2 lookup (latest available) to
transform points before slicing. This is required when subscribing to
/utlidar/cloud_deskewed (in 'odom' frame), so the scan ends up in 'base_link'
with origin at the robot, not at the odom origin.

QoS:
  - subscriber: BEST_EFFORT (matches GO2 publishers)
  - publisher : RELIABLE (matches Nav2 costmap default)

Vectorized with numpy for performance.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, LaserScan
import tf2_ros
from tf2_ros import TransformException


class CloudToScan(Node):
    def __init__(self):
        super().__init__('cloud_to_scan')

        # ROS parameters
        self.declare_parameter('input_topic', '/utlidar/cloud_deskewed')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('target_frame', 'base_link')
        # Height slice is interpreted in target_frame.
        # GO2 base_link is at body center (~0.30m above ground), so the ground
        # plane sits around z=-0.30 in base_link. Use [-0.20, +0.30] to capture
        # walls/obstacles between knee-height and head-height, skipping ground.
        self.declare_parameter('min_height', -0.20)
        self.declare_parameter('max_height', 0.30)
        self.declare_parameter('range_min', 0.30)
        self.declare_parameter('range_max', 8.0)
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('angle_increment', 0.01)  # ~0.57 deg

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.min_height = float(self.get_parameter('min_height').value)
        self.max_height = float(self.get_parameter('max_height').value)
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.angle_min = float(self.get_parameter('angle_min').value)
        self.angle_max = float(self.get_parameter('angle_max').value)
        self.angle_increment = float(self.get_parameter('angle_increment').value)

        self.num_ranges = int((self.angle_max - self.angle_min) / self.angle_increment)

        # tf2 setup (only used when input frame != target frame)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.sub = self.create_subscription(
            PointCloud2, self.input_topic, self.cloud_cb, qos_sub)
        self.pub = self.create_publisher(LaserScan, self.output_topic, qos_pub)

        self.get_logger().info(
            f'CloudToScan: {self.input_topic} -> {self.output_topic} '
            f'(frame={self.target_frame}, '
            f'z={self.min_height:.2f}..{self.max_height:.2f}, '
            f'r={self.range_min:.2f}..{self.range_max:.2f})'
        )

    def cloud_cb(self, msg: PointCloud2):
        # Parse fields to find x, y, z offsets
        x_off = y_off = z_off = -1
        for f in msg.fields:
            if f.name == 'x':
                x_off = f.offset
            elif f.name == 'y':
                y_off = f.offset
            elif f.name == 'z':
                z_off = f.offset
        if x_off < 0 or y_off < 0 or z_off < 0:
            return

        num_points = msg.width * msg.height
        if num_points == 0:
            return

        # Vectorized parse: extract float32 x,y,z columns from packed bytes.
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if raw.size < num_points * msg.point_step:
            return
        pts_bytes = raw[:num_points * msg.point_step].reshape(num_points, msg.point_step)
        x = pts_bytes[:, x_off:x_off + 4].copy().view(np.float32).ravel()
        y = pts_bytes[:, y_off:y_off + 4].copy().view(np.float32).ravel()
        z = pts_bytes[:, z_off:z_off + 4].copy().view(np.float32).ravel()

        # Transform points to target_frame if needed (e.g. odom -> base_link).
        # Use Time() (=0, "latest available") instead of msg.header.stamp:
        # rclpy's single-threaded executor blocks the TF listener while this
        # callback runs, so timeout-based lookups can't actually wait. The
        # latest TF is at most a few ms stale, which is fine for Nav2.
        if msg.header.frame_id and msg.header.frame_id != self.target_frame:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    msg.header.frame_id,
                    Time())  # Time(0) = latest available
            except TransformException as e:
                self.get_logger().warn(
                    f'TF {msg.header.frame_id} -> {self.target_frame} failed: {e}',
                    throttle_duration_sec=2.0)
                return
            x, y, z = self._apply_transform(tf, x, y, z)

        # Filter: finite + height slice (in target_frame)
        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        valid &= (z >= self.min_height) & (z <= self.max_height)
        if not np.any(valid):
            ranges = np.full(self.num_ranges, np.inf, dtype=np.float32)
        else:
            x = x[valid]
            y = y[valid]
            r = np.sqrt(x * x + y * y)
            angle = np.arctan2(y, x)

            # Range + angle filter
            keep = (r >= self.range_min) & (r <= self.range_max) & \
                   (angle >= self.angle_min) & (angle <= self.angle_max)
            r = r[keep]
            angle = angle[keep]

            # Bin into angular bins; min range per bin
            ranges = np.full(self.num_ranges, np.inf, dtype=np.float32)
            if r.size > 0:
                idx = ((angle - self.angle_min) / self.angle_increment).astype(np.int32)
                idx = np.clip(idx, 0, self.num_ranges - 1)
                np.minimum.at(ranges, idx, r)

        # Build LaserScan message
        scan = LaserScan()
        scan.header = msg.header
        scan.header.frame_id = self.target_frame
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges.tolist()

        self.pub.publish(scan)

    @staticmethod
    def _apply_transform(tf, x, y, z):
        """Apply a geometry_msgs/TransformStamped (returned by
        lookup_transform(target, source)) to numpy arrays expressed in `source`,
        producing arrays expressed in `target`. Formula: p' = R p + t."""
        q = tf.transform.rotation
        t = tf.transform.translation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        # Quaternion -> rotation matrix (right-handed, q assumed normalized)
        xx = qx * qx; yy = qy * qy; zz = qz * qz
        xy = qx * qy; xz = qx * qz; yz = qy * qz
        wx = qw * qx; wy = qw * qy; wz = qw * qz
        r00 = 1.0 - 2.0 * (yy + zz)
        r01 = 2.0 * (xy - wz)
        r02 = 2.0 * (xz + wy)
        r10 = 2.0 * (xy + wz)
        r11 = 1.0 - 2.0 * (xx + zz)
        r12 = 2.0 * (yz - wx)
        r20 = 2.0 * (xz - wy)
        r21 = 2.0 * (yz + wx)
        r22 = 1.0 - 2.0 * (xx + yy)
        x_new = r00 * x + r01 * y + r02 * z + t.x
        y_new = r10 * x + r11 * y + r12 * z + t.y
        z_new = r20 * x + r21 * y + r22 * z + t.z
        return x_new, y_new, z_new


def main():
    rclpy.init()
    node = CloudToScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
