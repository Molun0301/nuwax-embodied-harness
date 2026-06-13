#!/usr/bin/env python3
"""
Relay Nav2 /plan and /local_plan for RViz visualization.

Nav2 Foxy planner_server only calls publishPlan() when count_subscribers() > 0
on the internal lifecycle publisher. RViz QoS often does not match, so planning
still works (action path) but /plan stays empty and no green line appears.

This node keeps a compatible subscription (so Nav2 publishes) and republishes
the last path on /global_plan and /local_plan_viz for RViz.
"""
import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class PlanRelay(Node):
    def __init__(self):
        super().__init__('plan_relay')

        self.declare_parameter('input_global_plan_topic', '/plan')
        self.declare_parameter('input_local_plan_topic', '/local_plan')
        self.declare_parameter('output_global_plan_topic', '/global_plan')
        self.declare_parameter('output_local_plan_topic', '/local_plan_viz')

        in_global = str(self.get_parameter('input_global_plan_topic').value)
        in_local = str(self.get_parameter('input_local_plan_topic').value)
        out_global = str(self.get_parameter('output_global_plan_topic').value)
        out_local = str(self.get_parameter('output_local_plan_topic').value)

        # Match Nav2 Foxy planner_server publisher (depth 1, reliable, volatile).
        nav2_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        rviz_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._global_pub = self.create_publisher(Path, out_global, rviz_qos)
        self._local_pub = self.create_publisher(Path, out_local, rviz_qos)

        self.create_subscription(Path, in_global, self._on_global, nav2_qos)
        self.create_subscription(Path, in_local, self._on_local, nav2_qos)

        self.get_logger().info(
            f'PlanRelay: {in_global} -> {out_global}, '
            f'{in_local} -> {out_local} (keeps Nav2 plan publishing alive)'
        )

    def _on_global(self, msg: Path):
        if not msg.poses:
            return
        self._global_pub.publish(msg)

    def _on_local(self, msg: Path):
        if not msg.poses:
            return
        self._local_pub.publish(msg)


def main():
    rclpy.init()
    node = PlanRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
