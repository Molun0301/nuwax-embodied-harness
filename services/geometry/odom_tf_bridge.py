from __future__ import annotations

import argparse

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Nuwax odom 到 TF 桥。")
    parser.add_argument("--odom-topic", default="/utlidar/robot_odom")
    parser.add_argument("--default-child-frame", default="body")
    parser.add_argument("--child-frame", default="")
    parser.add_argument("--node-name", default="nuwax_odom_tf_bridge")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = rclpy.create_node(args.node_name)
    broadcaster = TransformBroadcaster(node)
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=20,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )

    def on_odom(message: Odometry) -> None:
        transform = TransformStamped()
        transform.header = message.header
        transform.child_frame_id = args.child_frame or message.child_frame_id or args.default_child_frame
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation = message.pose.pose.orientation
        broadcaster.sendTransform(transform)

    node.create_subscription(Odometry, args.odom_topic, on_odom, qos)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
