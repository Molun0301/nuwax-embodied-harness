import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTfBridge(Node):
    """Republish GO2 built-in odometry and broadcast odom -> base_link TF."""

    def __init__(self):
        super().__init__('odom_tf_bridge')

        self.declare_parameter('input_odom_topic', '/utlidar/robot_odom')
        self.declare_parameter('output_odom_topic', '/odom')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)

        input_topic = self.get_parameter('input_odom_topic').value
        output_topic = self.get_parameter('output_odom_topic').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, output_topic, 10)
        self.create_subscription(Odometry, input_topic, self.odom_callback, 10)

        self.get_logger().info(
            f'Bridging {input_topic} -> {output_topic} '
            f'(publish_tf={self.publish_tf})'
        )

    def odom_callback(self, msg: Odometry):
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose = msg.pose
        odom.twist = msg.twist
        self.odom_pub.publish(odom)

        if not self.publish_tf:
            return

        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self.odom_frame_id
        transform.child_frame_id = self.base_frame_id
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBridge()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()