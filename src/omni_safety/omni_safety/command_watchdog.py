import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


class CommandWatchdog(Node):
    def __init__(self):
        super().__init__('command_watchdog')
        self.declare_parameter('command_timeout_sec', 0.25)
        self.declare_parameter('publish_frequency_hz', 20.0)
        self.timeout_sec = float(self.get_parameter('command_timeout_sec').value)
        frequency_hz = float(self.get_parameter('publish_frequency_hz').value)
        if self.timeout_sec <= 0 or frequency_hz <= 0:
            raise ValueError('safety parameters must be positive')
        self.last_command_time = self.get_clock().now()
        self.last_command = Twist()
        self.estop_active = False
        self.safety_zone_stop = False
        self.safety_speed_factor = 1.0

        self.publisher = self.create_publisher(Twist, 'safe_cmd_vel', 10)
        self.watched_publisher = self.create_publisher(Twist, 'watched_cmd_vel', 10)
        self.state_publisher = self.create_publisher(Bool, 'safety_stop', 10)

        self.create_subscription(Twist, 'cmd_vel', self._on_command, 10)
        self.create_subscription(Bool, 'estop', self._on_estop, 10)
        self.create_subscription(Bool, 'safety_zone_stop', self._on_safety_zone_stop, 10)
        self.create_subscription(Float32, 'safety_speed_factor', self._on_speed_factor, 10)
        self.create_timer(1.0 / frequency_hz, self._publish)

    def _on_command(self, message):
        values = (message.linear.x, message.linear.y, message.angular.z)
        if all(math.isfinite(value) for value in values):
            self.last_command = message
            self.last_command_time = self.get_clock().now()

    def _on_estop(self, message):
        self.estop_active = bool(message.data)

    def _on_safety_zone_stop(self, message):
        self.safety_zone_stop = bool(message.data)

    def _on_speed_factor(self, message):
        val = float(message.data)
        if math.isfinite(val):
            self.safety_speed_factor = max(0.0, min(1.0, val))

    def _publish(self):
        age_sec = ((self.get_clock().now() - self.last_command_time).nanoseconds
                   * 1e-9)
        timed_out = age_sec > self.timeout_sec
        stopped = self.estop_active or timed_out or self.safety_zone_stop

        out_cmd = Twist()
        if not stopped:
            factor = self.safety_speed_factor
            out_cmd.linear.x = self.last_command.linear.x * factor
            out_cmd.linear.y = self.last_command.linear.y * factor
            out_cmd.angular.z = self.last_command.angular.z * factor

        self.publisher.publish(out_cmd)
        self.watched_publisher.publish(out_cmd)

        state = Bool()
        state.data = stopped
        self.state_publisher.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = CommandWatchdog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()