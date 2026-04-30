from __future__ import annotations

import argparse

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Nuwax odom 到 TF 桥。")
    parser.add_argument("--odom-topic", default="/nuwax/geometry/odom")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--default-child-frame", default="body")
    parser.add_argument("--child-frame", default="")
    parser.add_argument("--node-name", default="nuwax_odom_tf_bridge")
    return parser.parse_args()


def _normalize_frame_id(value: str | None) -> str:
    return (value or "").strip().lstrip("/")


def main() -> None:
    args = parse_args()
    odom_frame = _normalize_frame_id(args.odom_frame) or "odom"
    default_child_frame = _normalize_frame_id(args.default_child_frame) or "body"
    child_frame_override = _normalize_frame_id(args.child_frame)
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
        parent_frame = odom_frame or _normalize_frame_id(message.header.frame_id) or "odom"
        child_frame = child_frame_override or _normalize_frame_id(message.child_frame_id) or default_child_frame
        if parent_frame == child_frame:
            node.get_logger().warning(
                f"跳过自引用 odom TF：parent_frame={parent_frame}, child_frame={child_frame}"
            )
            return
        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
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
