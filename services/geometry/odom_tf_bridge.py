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
    # 显式留空表示「锁不死」：尊重 odom 消息携带的 header.frame_id / child_frame_id；
    # 仅当消息也未带 frame 时才退到 --default-* 兜底。
    parser.add_argument("--odom-frame", default="")
    parser.add_argument("--default-odom-frame", default="odom")
    parser.add_argument("--default-child-frame", default="base_footprint")
    parser.add_argument("--child-frame", default="")
    parser.add_argument("--node-name", default="nuwax_odom_tf_bridge")
    return parser.parse_args()


def _normalize_frame_id(value: str | None) -> str:
    return (value or "").strip().lstrip("/")


def main() -> None:
    args = parse_args()
    # 锁定值（显式 CLI 给出）优先；否则尊重 odom 消息携带的 frame；再退到默认。
    odom_frame_override = _normalize_frame_id(args.odom_frame)
    default_odom_frame = _normalize_frame_id(args.default_odom_frame) or "odom"
    child_frame_override = _normalize_frame_id(args.child_frame)
    default_child_frame = _normalize_frame_id(args.default_child_frame) or "base_footprint"
    rclpy.init()
    node = rclpy.create_node(args.node_name)
    broadcaster = TransformBroadcaster(node)
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=20,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    # 保留上一次有效 stamp，拦截零时间戳与重复 stamp，避免 tf2 抛 TF_REPEAT_DATA / TF_OLD_DATA。
    last_stamp: list[tuple[int, int] | None] = [None]

    def on_odom(message: Odometry) -> None:
        parent_frame = (
            odom_frame_override
            or _normalize_frame_id(message.header.frame_id)
            or default_odom_frame
        )
        child_frame = (
            child_frame_override
            or _normalize_frame_id(message.child_frame_id)
            or default_child_frame
        )
        if parent_frame == child_frame:
            node.get_logger().warning(
                f"跳过自引用 odom TF：parent_frame={parent_frame}, child_frame={child_frame}"
            )
            return
        stamp_sec = int(message.header.stamp.sec)
        stamp_nanosec = int(message.header.stamp.nanosec)
        if stamp_sec == 0 and stamp_nanosec == 0:
            node.get_logger().warning("跳过零时间戳 odom TF，避免下游 lookup 失败。")
            return
        current_stamp = (stamp_sec, stamp_nanosec)
        if last_stamp[0] == current_stamp:
            node.get_logger().debug(
                f"跳过重复时间戳 odom TF：stamp={stamp_sec}.{stamp_nanosec:09d}"
            )
            return
        last_stamp[0] = current_stamp
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
