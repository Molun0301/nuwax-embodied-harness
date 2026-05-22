import argparse

import rclpy
from rclpy.node import Node
from rtabmap_msgs.msg import Info
from std_srvs.srv import Empty


class AutoSwitchToMapping(Node):
    def __init__(self, info_topic, service_names, min_matches):
        super().__init__('auto_switch_to_mapping')
        self.min_matches = min_matches
        self.matches = 0
        self.switching = False
        self.clients = [self.create_client(Empty, name) for name in service_names]
        self.sub = self.create_subscription(Info, info_topic, self.info_cb, 10)
        self.get_logger().info(
            'Waiting for RTAB-Map relocalization before switching to mapping '
            f'(info_topic={info_topic}, services={",".join(service_names)})'
        )

    def info_cb(self, msg):
        if self.switching:
            return
        matched = msg.loop_closure_id > 0 or msg.proximity_detection_id > 0
        if not matched:
            return

        self.matches += 1
        self.get_logger().info(
            'RTAB-Map relocalization match detected '
            f'(loop_closure_id={msg.loop_closure_id}, '
            f'proximity_detection_id={msg.proximity_detection_id}, '
            f'matches={self.matches}/{self.min_matches})'
        )
        if self.matches < self.min_matches:
            return

        self.switching = True
        client = None
        for candidate in self.clients:
            if candidate.wait_for_service(timeout_sec=1.0):
                client = candidate
                break
        if client is None:
            self.get_logger().error('Mapping mode service is not available.')
            rclpy.shutdown()
            return

        future = client.call_async(Empty.Request())
        future.add_done_callback(self.switched_cb)

    def switched_cb(self, future):
        try:
            future.result()
        except Exception as exc:
            self.get_logger().error(f'Failed to switch RTAB-Map to mapping mode: {exc}')
        else:
            self.get_logger().info('RTAB-Map switched to mapping mode.')
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--info-topic', default='/info')
    parser.add_argument(
        '--service',
        action='append',
        default=['/set_mode_mapping', '/rtabmap/set_mode_mapping'],
    )
    parser.add_argument('--min-matches', type=int, default=1)
    args = parser.parse_args()

    rclpy.init()
    node = AutoSwitchToMapping(args.info_topic, args.service, max(1, args.min_matches))
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
