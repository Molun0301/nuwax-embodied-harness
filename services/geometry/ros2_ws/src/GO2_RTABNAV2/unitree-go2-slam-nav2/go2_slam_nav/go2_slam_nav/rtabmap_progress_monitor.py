#!/usr/bin/env python3
"""订阅 /rtabmap/info，用中文输出建图/定位/回环进度日志。"""

from typing import List

import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

_MODE_LABELS = {
    'mapping': '建图',
    'explore': '探索',
    'localization': '定位',
}

# 与 rtabmap_slam CoreWrapper 中 info 发布端默认 QoS 对齐（KeepLast + Reliable）。
INFO_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class RtabmapProgressMonitor(Node):
    def __init__(self) -> None:
        super().__init__('rtabmap_progress_monitor')
        # rtabmap 默认发布相对话题 info（ROS2 下为 /info）；launch 会 remap 到 /rtabmap/info
        self.declare_parameter('info_topic', '/rtabmap/info')
        self.declare_parameter('info_topic_fallback', '/info')
        self.declare_parameter('heartbeat_sec', 15.0)
        self.declare_parameter('startup_grace_sec', 30.0)
        self.declare_parameter('slam_mode', 'mapping')

        self._info_topic = str(self.get_parameter('info_topic').value)
        self._info_topic_fallback = str(
            self.get_parameter('info_topic_fallback').value
        )
        self._heartbeat_sec = max(5.0, float(self.get_parameter('heartbeat_sec').value))
        self._startup_grace_sec = max(0.0, float(self.get_parameter('startup_grace_sec').value))
        slam_mode = str(self.get_parameter('slam_mode').value).strip().lower()
        self._mode_label = _MODE_LABELS.get(slam_mode, slam_mode or '建图')
        self._last_loop_id = 0
        self._last_proximity_id = 0
        self._last_ref_id = -1
        self._nodes = 0
        self._msg_count = 0
        self._loop_triggered = False
        self._first_msg_logged = False
        self._start_time = self.get_clock().now()

        # 保证进度日志不被其它节点的 warn 级联设置淹没。
        self.get_logger().set_level(LoggingSeverity.INFO)

        try:
            from rtabmap_msgs.msg import Info as RtabmapInfo
        except ImportError as exc:
            self.get_logger().error(
                f'[任务进度] 无法导入 rtabmap_msgs，回环监控未启动：{exc}'
            )
            raise

        self._info_msg_type = RtabmapInfo
        self._active_topic = ''
        self._subscribe_info(RtabmapInfo, self._info_topic)
        if self._info_topic_fallback and self._info_topic_fallback != self._info_topic:
            self._subscribe_info(RtabmapInfo, self._info_topic_fallback)
        self.create_timer(self._heartbeat_sec, self._heartbeat)
        topics = self._info_topic
        if self._info_topic_fallback and self._info_topic_fallback != self._info_topic:
            topics = f'{self._info_topic}（备用 {self._info_topic_fallback}）'
        self.get_logger().info(
            f'[任务进度] 回环监控已启动，模式={self._mode_label}，'
            f'订阅 {topics}，心跳 {self._heartbeat_sec:.0f}s，'
            f'启动宽限 {self._startup_grace_sec:.0f}s。'
        )

    def _subscribe_info(self, msg_type, topic: str) -> None:
        self.create_subscription(
            msg_type, topic, lambda msg, t=topic: self._on_info(msg, t), INFO_QOS,
        )

    def _loop_status_cn(self) -> str:
        if not self._loop_triggered:
            return '回环已触发=否'
        return (
            f'回环已触发=是(loop={self._last_loop_id}, '
            f'proximity={self._last_proximity_id})'
        )

    @staticmethod
    def _node_count(msg) -> int:
        wm_state = getattr(msg, 'wm_state', None) or []
        if wm_state:
            return len(wm_state)
        stats_keys = getattr(msg, 'stats_keys', None) or []
        stats_values = getattr(msg, 'stats_values', None) or []
        for key, value in zip(stats_keys, stats_values):
            if key in ('Memory/Working_memory_size', 'Keypoints/words', 'Nodes'):
                return int(value)
        return 0

    def _on_info(self, msg, topic: str) -> None:
        self._active_topic = topic
        self._msg_count += 1
        loop_id = int(getattr(msg, 'loop_closure_id', 0) or 0)
        proximity_id = int(getattr(msg, 'proximity_detection_id', 0) or 0)
        ref_id = int(getattr(msg, 'ref_id', -1) or -1)
        nodes = self._node_count(msg)
        self._nodes = nodes

        if not self._first_msg_logged:
            self._first_msg_logged = True
            self.get_logger().info(
                f'[任务进度] 模式={self._mode_label} | 已收到 {self._active_topic}，'
                f'RTAB-Map 正在处理数据（ref_id={ref_id}，工作记忆节点={nodes}）。'
            )

        if loop_id > self._last_loop_id:
            self._loop_triggered = True
            self.get_logger().info(
                f'[任务进度] 模式={self._mode_label} | 回环已触发=是 | '
                f'类型=全局回环 loop_closure_id={loop_id}，参考节点={ref_id}，'
                f'图谱节点={nodes}。'
                '将校正全局地图，并更新 map→odom TF（机器狗在地图坐标系中的位姿）。'
            )
            self._last_loop_id = loop_id

        if proximity_id > self._last_proximity_id:
            self._loop_triggered = True
            self.get_logger().info(
                f'[任务进度] 模式={self._mode_label} | 回环已触发=是 | '
                f'类型=邻近回环 proximity_detection_id={proximity_id}，'
                f'参考节点={ref_id}，图谱节点={nodes}。'
                '将校正全局地图，并更新 map→odom TF（机器狗在地图坐标系中的位姿）。'
            )
            self._last_proximity_id = proximity_id

        if ref_id != self._last_ref_id and ref_id >= 0:
            self._last_ref_id = ref_id

    def _info_topics(self) -> List[str]:
        topics = [self._info_topic]
        if self._info_topic_fallback and self._info_topic_fallback not in topics:
            topics.append(self._info_topic_fallback)
        return topics

    def _existing_info_topic(self) -> str:
        expected = f'{self._info_msg_type.__module__}/{self._info_msg_type.__name__}'
        for name, types in self.get_topic_names_and_types():
            if name in self._info_topics() and expected in types:
                return name
        return ''

    def _no_data_hint(self) -> str:
        return (
            'RTAB-Map 仅在同步处理点云+里程计后发布 info（且有订阅者时才会发）。'
            '请检查：ros2 node list | grep rtabmap；'
            'ros2 topic hz /info 或 /rtabmap/info；'
            'ros2 topic hz /go2/cloud；ros2 topic hz /go2/odom；'
            'rtabmap 日志是否有点云/TF 同步错误。'
        )

    def _heartbeat(self) -> None:
        if self._msg_count <= 0:
            elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
            if elapsed < self._startup_grace_sec:
                return

            existing = self._existing_info_topic()
            if existing:
                self.get_logger().warn(
                    f'[任务进度] 模式={self._mode_label} | 话题 {existing} 已存在但尚无数据，'
                    f'rtabmap 可能未收到点云/里程计。{self._no_data_hint()}'
                )
            else:
                self.get_logger().warn(
                    f'[任务进度] 模式={self._mode_label} | '
                    f'话题 {self._info_topic} / {self._info_topic_fallback} 均不存在，'
                    f'rtabmap 节点可能未启动。{self._no_data_hint()}'
                )
            return

        if not self._loop_triggered:
            self.get_logger().info(
                f'[任务进度] 模式={self._mode_label} | {self._loop_status_cn()} | '
                f'图谱节点={self._nodes}，当前 ref_id={self._last_ref_id}，'
                f'累计收到 info={self._msg_count} 条。'
                '请慢速重访已扫区域以触发全局邻近回环；'
                '触发后 map→odom TF 会随之校正。'
            )
        else:
            self.get_logger().info(
                f'[任务进度] 模式={self._mode_label} | {self._loop_status_cn()} | '
                f'图谱节点={self._nodes}，当前 ref_id={self._last_ref_id}，'
                f'累计收到 info={self._msg_count} 条。'
                'map→odom TF 已由 RTAB-Map 维护并随回环优化更新。'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RtabmapProgressMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()