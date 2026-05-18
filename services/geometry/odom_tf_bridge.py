from __future__ import annotations

import argparse

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Nuwax odom 到 TF 桥。")
    parser.add_argument("--odom-topic", required=True)
    # 锁定值（CLI 必填）优先；--default-* 仅作为 odom 消息未带 frame 时的兜底，
    # 但本身也必须由调用方显式指定（不再硬编码字面值）。
    parser.add_argument("--odom-frame", default="")
    parser.add_argument("--default-odom-frame", required=True)
    parser.add_argument("--default-child-frame", required=True)
    parser.add_argument("--child-frame", default="")
    parser.add_argument("--node-name", default="nuwax_odom_tf_bridge")
    return parser.parse_args()


def _normalize_frame_id(value: str | None) -> str:
    return (value or "").strip().lstrip("/")


def main() -> None:
    args = parse_args()
    # 锁定值（显式 CLI 给出）优先；否则尊重 odom 消息携带的 frame；再退到 --default-*。
    odom_frame_override = _normalize_frame_id(args.odom_frame)
    default_odom_frame = _normalize_frame_id(args.default_odom_frame)
    child_frame_override = _normalize_frame_id(args.child_frame)
    default_child_frame = _normalize_frame_id(args.default_child_frame)
    if not default_odom_frame or not default_child_frame:
        raise SystemExit(
            "odom_tf_bridge: --default-odom-frame / --default-child-frame 必须显式指定，禁止硬编码兜底。"
        )
    rclpy.init()
    node = rclpy.create_node(args.node_name)
    broadcaster = TransformBroadcaster(node)
    # Jetson 内存有限：depth=1 避免大缓冲区分配；BEST_EFFORT 减少 DDS 内部队列开销。
    # odom/TF 为高频数据，丢帧可接受，不需要 RELIABLE 重传。
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
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
    except MemoryError:
        node.get_logger().fatal("odom_tf_bridge: 内存不足（MemoryError），优雅退出。")
    except Exception as exc:  # noqa: BLE001
        node.get_logger().fatal(f"odom_tf_bridge: 未预期异常，优雅退出：{exc}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
