#!/usr/bin/env python3
"""发布机器狗在全局地图 map 坐标系下的位姿（go2_base_link）。"""

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class MapRobotPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__('map_robot_pose_publisher')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'go2_base_link')
        self.declare_parameter('output_topic', '/go2/pose_map')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('tf_timeout', 0.3)

        self._map_frame = str(self.get_parameter('map_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._output_topic = str(self.get_parameter('output_topic').value)
        self._tf_timeout = float(self.get_parameter('tf_timeout').value)
        rate = max(1.0, float(self.get_parameter('publish_rate').value))

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._pub = self.create_publisher(PoseStamped, self._output_topic, 10)
        self._warned_no_tf = False
        self.create_timer(1.0 / rate, self._publish_pose)

        self.get_logger().info(
            f'发布 map 位姿: {self._map_frame} -> {self._base_frame} '
            f'on {self._output_topic} ({rate:.1f} Hz)'
        )

    def _publish_pose(self) -> None:
        try:
            tf_msg: TransformStamped = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self._tf_timeout),
            )
        except Exception as exc:
            if not self._warned_no_tf:
                self.get_logger().warn(
                    f'等待 TF {self._map_frame} -> {self._base_frame}：{exc}'
                )
                self._warned_no_tf = True
            return

        if self._warned_no_tf:
            self.get_logger().info(
                f'TF {self._map_frame} -> {self._base_frame} 已可用，开始发布 {self._output_topic}'
            )
            self._warned_no_tf = False

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self._map_frame
        pose.pose.position.x = tf_msg.transform.translation.x
        pose.pose.position.y = tf_msg.transform.translation.y
        pose.pose.position.z = tf_msg.transform.translation.z
        pose.pose.orientation = tf_msg.transform.rotation
        self._pub.publish(pose)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapRobotPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()