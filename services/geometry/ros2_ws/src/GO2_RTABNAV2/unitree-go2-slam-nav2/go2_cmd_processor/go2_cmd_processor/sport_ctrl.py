import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Empty
from unitree_api.msg import Request

class Go2CmdProcessor(Node):
    def __init__(self):
        super().__init__("go2_cmd_processor")

        # Declare parameters
        self.declare_parameter("rate", 200.0)
        self.declare_parameter("cmd_vel_timeout", 0.25)  # Timeout in seconds
        self.declare_parameter("log_level", "info")  # Logging level
        self.declare_parameter("min_linear_speed", 0.08)
        self.declare_parameter("min_lateral_speed", 0.0)
        self.declare_parameter("allow_lateral", False)
        self.declare_parameter("min_angular_speed", 0.08)
        self.declare_parameter("zero_epsilon", 0.02)
        self.declare_parameter("allow_reverse_in_recovery", True)
        self.declare_parameter(
            "backup_action_status_topic", "/backup/_action/status",
        )

        # Set the logger level based on the parameter
        log_level = self.get_parameter("log_level").value.lower()
        log_severity = {
            "debug": LoggingSeverity.DEBUG,
            "info": LoggingSeverity.INFO,
            "warn": LoggingSeverity.WARN,
            "error": LoggingSeverity.ERROR,
            "fatal": LoggingSeverity.FATAL,
        }.get(log_level, LoggingSeverity.INFO)
        self.get_logger().set_level(log_severity)

        self.rate = self.get_parameter("rate").value
        self.cmd_vel_timeout = self.get_parameter("cmd_vel_timeout").value
        self.min_linear_speed = self.get_parameter("min_linear_speed").value
        self.min_lateral_speed = self.get_parameter("min_lateral_speed").value
        self.allow_lateral = bool(self.get_parameter("allow_lateral").value)
        self.min_angular_speed = self.get_parameter("min_angular_speed").value
        self.zero_epsilon = self.get_parameter("zero_epsilon").value
        self.allow_reverse_in_recovery = bool(
            self.get_parameter("allow_reverse_in_recovery").value,
        )
        self._backup_active = False

        # Time intervals
        self.publish_interval = 1.0 / self.rate
        self.cmd_vel_timeout_duration = self.cmd_vel_timeout
        self.cmd_vel_counter = 0.0

        # Publishers
        self.publisher = self.create_publisher(Request, "/api/sport/request", 10)

        # Subscribers
        self.subscription = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10
        )

        if self.allow_reverse_in_recovery:
            status_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
            backup_topic = str(
                self.get_parameter("backup_action_status_topic").value,
            )
            self.create_subscription(
                GoalStatusArray,
                backup_topic,
                self._backup_status_cb,
                status_qos,
            )
            self.get_logger().info(
                f'Reverse allowed only during Nav2 BackUp ({backup_topic})',
            )

        # Services
        self.create_service(Empty, "stand_up", self.stand_up_callback)
        self.create_service(Empty, "lay_down", self.lay_down_callback)
        self.create_service(Empty, "recover_stand", self.recover_stand_callback)
        self.create_service(Empty, "damping", self.damping_callback)
        self.create_service(Empty, "dance1", self.dance1_callback)
        self.create_service(Empty, "dance2", self.dance2_callback)

        # Timers
        self.timer = self.create_timer(self.publish_interval, self.timer_callback)

        # Command state
        self.current_request = None
        self.reset_state = False

        self.get_logger().info("Go2 Command Processor Node Started")

    def create_request(self, api_id, parameters=None):
        """Helper function to create a Request message."""
        req = Request()
        req.header.identity.api_id = api_id
        if parameters:
            req.parameter = parameters
        return req

    def _backup_status_cb(self, msg: GoalStatusArray) -> None:
        self._backup_active = any(
            int(status.status) == GoalStatus.STATUS_EXECUTING
            for status in msg.status_list
        )

    def apply_velocity_limits(self, msg: Twist) -> Twist:
        """GO2: forward + yaw only; reverse only during Nav2 BackUp recovery."""
        vx = msg.linear.x
        vy = 0.0 if not self.allow_lateral else msg.linear.y
        wz = msg.angular.z

        def snap_min(value: float, minimum: float) -> float:
            """Snap sub-min commands to minimum (stiction); pass >= min through."""
            if abs(value) <= self.zero_epsilon:
                return 0.0
            if abs(value) < minimum:
                return minimum if value > 0.0 else -minimum
            return value

        reverse_ok = self.allow_reverse_in_recovery and self._backup_active
        if vx < 0.0 and not reverse_ok:
            vx = 0.0
        elif vx > 0.0:
            vx = snap_min(vx, self.min_linear_speed)
        if self.allow_lateral:
            vy = snap_min(vy, self.min_lateral_speed)
        if abs(wz) > self.zero_epsilon:
            wz = snap_min(wz, self.min_angular_speed)

        out = Twist()
        out.linear.x = vx
        out.linear.y = vy
        out.angular.z = wz
        return out

    def cmd_vel_callback(self, msg: Twist):
        """
        Callback for cmd_vel messages. Converts the velocity commands into SportMode commands.
        """
        limited = self.apply_velocity_limits(msg)
        self.get_logger().info(
            f"cmd_vel in=({msg.linear.x:.3f},{msg.linear.y:.3f},{msg.angular.z:.3f}) "
            f"out=({limited.linear.x:.3f},{limited.linear.y:.3f},{limited.angular.z:.3f})"
        )

        # Create a SportMode Move request
        req = self.create_request(
            api_id=1008,  # API ID for "Move"
            parameters=(
                f'{{"x": {limited.linear.x}, "y": {limited.linear.y}, '
                f'"z": {limited.angular.z}}}'
            ),
        )

        # Publish the request
        self.publisher.publish(req)
        self.current_request = req
        self.cmd_vel_counter = 0.0  # Reset the timeout counter

    def timer_callback(self):
        """
        Timer callback to handle periodic command publishing and timeout.
        """
        if self.cmd_vel_counter >= self.cmd_vel_timeout_duration:
            # Send Idle command if cmd_vel has timed out
            self.publish_idle_command()
        else:
            self.cmd_vel_counter += self.publish_interval

        # Handle reset state
        if self.reset_state:
            self.reset_state = False
            self.publish_idle_command()

    def publish_idle_command(self):
        """Publishes an Idle command."""
        req = self.create_request(api_id=0)  # API ID for "Idle"
        self.publisher.publish(req)
        self.get_logger().info("Published Idle command")

    # Service Callbacks
    def stand_up_callback(self, request, response):
        self.publisher.publish(self.create_request(api_id=1004))  # StandUp
        self.reset_state = True
        self.get_logger().info("Stand Up command sent")
        return response

    def lay_down_callback(self, request, response):
        self.publisher.publish(self.create_request(api_id=1005))  # StandDown
        self.reset_state = True
        self.get_logger().info("Lay Down command sent")
        return response

    def recover_stand_callback(self, request, response):
        self.publisher.publish(self.create_request(api_id=1006))  # RecoveryStand
        self.reset_state = True
        self.get_logger().info("Recover Stand command sent")
        return response

    def damping_callback(self, request, response):
        self.publisher.publish(self.create_request(api_id=1001))  # Damping
        self.reset_state = True
        self.get_logger().info("Damping command sent")
        return response

    def dance1_callback(self, request, response):
        self.publisher.publish(self.create_request(api_id=1022))  # Dance1
        self.reset_state = True
        self.get_logger().info("Dance1 command sent")
        return response

    def dance2_callback(self, request, response):
        self.publisher.publish(self.create_request(api_id=1023))  # Dance2
        self.reset_state = True
        self.get_logger().info("Dance2 command sent")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Go2CmdProcessor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()


