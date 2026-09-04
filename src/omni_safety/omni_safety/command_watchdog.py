import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


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
        self.publisher = self.create_publisher(Twist, 'safe_cmd_vel', 10)
        self.state_publisher = self.create_publisher(Bool, 'safety_stop', 10)
        self.create_subscription(Twist, 'cmd_vel', self._on_command, 10)
        self.create_subscription(Bool, 'estop', self._on_estop, 10)
        self.create_timer(1.0 / frequency_hz, self._publish)

    def _on_command(self, message):
        values = (message.linear.x, message.linear.y, message.angular.z)
        if all(math.isfinite(value) for value in values):
            self.last_command = message
            self.last_command_time = self.get_clock().now()

    def _on_estop(self, message):
        self.estop_active = bool(message.data)

    def _publish(self):
        age_sec = ((self.get_clock().now() - self.last_command_time).nanoseconds
                   * 1e-9)
        stopped = self.estop_active or age_sec > self.timeout_sec
        self.publisher.publish(Twist() if stopped else self.last_command)
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