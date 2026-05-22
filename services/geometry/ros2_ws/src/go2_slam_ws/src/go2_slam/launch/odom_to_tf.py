import argparse
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdomToTF(Node):
    def __init__(self, odom_topic, odom_frame, base_frame, planar):
        super().__init__('odom_to_tf')
        self.odom_frame = odom_frame
        self.base_frame = base_frame
        self.planar = planar
        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(Odometry, odom_topic, self.cb, 10)

    @staticmethod
    def _yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def cb(self, msg):
        t = TransformStamped()
        t.header = msg.header
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        if self.planar:
            yaw = self._yaw_from_quaternion(msg.pose.pose.orientation)
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = math.sin(yaw * 0.5)
            t.transform.rotation.w = math.cos(yaw * 0.5)
        else:
            t.transform.translation.z = msg.pose.pose.position.z
            t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--odom-topic', default='/utlidar/robot_odom')
    parser.add_argument('--odom-frame', default='odom')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--planar', default='true')
    args = parser.parse_args()
    planar = args.planar.lower() in ('1', 'true', 'yes', 'on')

    rclpy.init()
    node = OdomToTF(args.odom_topic, args.odom_frame, args.base_frame, planar)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
