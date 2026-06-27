"""Relay isolated GO2 SLAM TF tree to /tf for Nav2."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class Go2TfRelay(Node):
    def __init__(self):
        super().__init__('go2_tf_relay')

        tf_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
        )
        static_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._tf_pub = self.create_publisher(TFMessage, '/tf', tf_qos)
        self._tf_static_pub = self.create_publisher(TFMessage, '/tf_static', static_qos)
        self.create_subscription(TFMessage, '/go2/tf', self._relay_tf, tf_qos)
        self.create_subscription(TFMessage, '/go2/tf_static', self._relay_tf_static, static_qos)

        self.get_logger().info('Relaying /go2/tf -> /tf, /go2/tf_static -> /tf_static')

    def _relay_tf(self, msg: TFMessage):
        self._tf_pub.publish(msg)

    def _relay_tf_static(self, msg: TFMessage):
        self._tf_static_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Go2TfRelay()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()