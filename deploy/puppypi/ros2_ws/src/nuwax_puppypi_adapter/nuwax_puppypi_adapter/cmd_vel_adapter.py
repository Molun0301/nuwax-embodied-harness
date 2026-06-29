"""Bridge geometry_msgs/Twist to puppy_control_msgs/Velocity."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from puppy_control_msgs.msg import Velocity
from rclpy.node import Node


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class PuppyPiCmdVelAdapter(Node):
    def __init__(self) -> None:
        super().__init__("puppypi_cmd_vel_adapter")

        self.declare_parameter("input_cmd_vel", "/cmd_vel")
        self.declare_parameter("output_velocity", "/puppy_control/velocity")
        self.declare_parameter("max_x_cm_s", 12.0)
        self.declare_parameter("max_yaw_rate", 0.6)

        input_topic = str(self.get_parameter("input_cmd_vel").value)
        output_topic = str(self.get_parameter("output_velocity").value)
        self.max_x = float(self.get_parameter("max_x_cm_s").value)
        self.max_yaw = float(self.get_parameter("max_yaw_rate").value)

        self.publisher = self.create_publisher(Velocity, output_topic, 10)
        self.subscription = self.create_subscription(
            Twist,
            input_topic,
            self.on_cmd_vel,
            10,
        )
        self.get_logger().info("Bridge %s -> %s", input_topic, output_topic)

    def on_cmd_vel(self, message: Twist) -> None:
        output = Velocity()
        output.x = clamp(message.linear.x * 100.0, -self.max_x, self.max_x)
        output.y = 0.0
        output.yaw_rate = clamp(message.angular.z, -self.max_yaw, self.max_yaw)
        self.publisher.publish(output)


def main() -> None:
    rclpy.init()
    node = PuppyPiCmdVelAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
