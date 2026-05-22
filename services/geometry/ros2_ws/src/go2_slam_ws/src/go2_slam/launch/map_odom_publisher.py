import argparse

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformException, TransformListener


class MapOdomPublisher(Node):
    def __init__(self, map_frame, base_frame, topic, rate_hz):
        super().__init__('map_odom_publisher')
        self.map_frame = map_frame
        self.base_frame = base_frame
        self.pub = self.create_publisher(Odometry, topic, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_odom)
        self.get_logger().info(
            f'Publishing corrected odom on {topic}: {map_frame} -> {base_frame}'
        )

    def publish_odom(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException:
            return

        msg = Odometry()
        msg.header = tf.header
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = tf.transform.translation.x
        msg.pose.pose.position.y = tf.transform.translation.y
        msg.pose.pose.position.z = tf.transform.translation.z
        msg.pose.pose.orientation = tf.transform.rotation
        self.pub.publish(msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map-frame', default='map')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--topic', default='/rtabmap/localization_odom')
    parser.add_argument('--rate', type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = MapOdomPublisher(args.map_frame, args.base_frame, args.topic, args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
