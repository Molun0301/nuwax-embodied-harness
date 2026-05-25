#!/usr/bin/env python3
"""
cmd_vel to Unitree GO2 Sport API bridge.
Subscribes to /cmd_vel (geometry_msgs/Twist) and publishes
Move commands to /api/sport/request (unitree_api/msg/Request).
"""

import json
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_api.msg import Request, RequestHeader, RequestIdentity, RequestPolicy


SPORT_API_ID_MOVE = 1008


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')

        self.declare_parameter('max_vx', 0.5)
        self.declare_parameter('max_vy', 0.0)  # disable lateral motion
        self.declare_parameter('max_vyaw', 1.0)
        self.declare_parameter('cmd_timeout', 0.5)

        self.max_vx = self.get_parameter('max_vx').value
        self.max_vy = self.get_parameter('max_vy').value
        self.max_vyaw = self.get_parameter('max_vyaw').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        self.pub = self.create_publisher(Request, '/api/sport/request', 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        self.last_cmd_time = time.time()
        self.timer = self.create_timer(0.1, self.timer_cb)
        self.last_twist = Twist()
        self.has_cmd = False

        self.get_logger().info(
            f'CmdVel bridge started. Limits: vx={self.max_vx}, vy={self.max_vy}, vyaw={self.max_vyaw}'
        )

    def clamp(self, val, limit):
        return max(-limit, min(limit, val))

    def cmd_vel_cb(self, msg: Twist):
        self.last_twist = msg
        self.last_cmd_time = time.time()
        self.has_cmd = True
        self.send_move(msg)

    def timer_cb(self):
        if self.has_cmd and (time.time() - self.last_cmd_time) > self.cmd_timeout:
            self.has_cmd = False
            self.send_move(Twist())
            self.get_logger().info('cmd_vel timeout, sending stop')

    def send_move(self, twist: Twist):
        vx = self.clamp(twist.linear.x, self.max_vx)
        vy = self.clamp(twist.linear.y, self.max_vy)
        vyaw = self.clamp(twist.angular.z, self.max_vyaw)

        req = Request()

        req.header.identity.id = int(time.time() * 1000) % (2**31)
        req.header.identity.api_id = SPORT_API_ID_MOVE
        req.header.policy.noreply = True

        req.parameter = json.dumps({"x": vx, "y": vy, "z": vyaw})

        self.pub.publish(req)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
