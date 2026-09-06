import math
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32

from omni_safety.safety_zone import evaluate_safety_zone


class SafetyZoneNode(Node):
    def __init__(self):
        super().__init__('safety_zone_node')
        self.declare_parameter('slow_zone_m', 1.0)
        self.declare_parameter('stop_zone_m', 0.5)
        self.declare_parameter('command_timeout_sec', 0.25)
        self.declare_parameter('publish_frequency_hz', 20.0)

        self.slow_zone_m = float(self.get_parameter('slow_zone_m').value)
        self.stop_zone_m = float(self.get_parameter('stop_zone_m').value)
        self.timeout_sec = float(self.get_parameter('command_timeout_sec').value)
        freq = float(self.get_parameter('publish_frequency_hz').value)

        self.latest_scan = None
        self.last_command = Twist()
        self.last_command_time = self.get_clock().now()
        self.estop_active = False

        self.cmd_vel_pub = self.create_publisher(Twist, 'safe_cmd_vel', 10)
        self.safety_stop_pub = self.create_publisher(Bool, 'safety_stop', 10)
        self.speed_factor_pub = self.create_publisher(Float32, 'safety_speed_factor', 10)

        self.create_subscription(LaserScan, 'scan', self._on_scan, 10)
        self.create_subscription(Twist, 'cmd_vel', self._on_command, 10)
        self.create_subscription(Bool, 'estop', self._on_estop, 10)

        self.create_timer(1.0 / freq, self._publish_safety)

    def _on_scan(self, msg: LaserScan):
        self.latest_scan = msg

    def _on_command(self, msg: Twist):
        values = (msg.linear.x, msg.linear.y, msg.angular.z)
        if all(math.isfinite(v) for v in values):
            self.last_command = msg
            self.last_command_time = self.get_clock().now()

    def _on_estop(self, msg: Bool):
        self.estop_active = bool(msg.data)

    def _publish_safety(self):
        age_sec = (self.get_clock().now() - self.last_command_time).nanoseconds * 1e-9
        timed_out = age_sec > self.timeout_sec

        safety_stop = False
        speed_factor = 1.0

        if self.latest_scan is not None and self.latest_scan.ranges:
            try:
                safety_stop, speed_factor, _ = evaluate_safety_zone(
                    self.latest_scan.ranges,
                    self.latest_scan.angle_min,
                    self.latest_scan.angle_increment,
                    vx=self.last_command.linear.x,
                    vy=self.last_command.linear.y,
                    slow_zone_m=self.slow_zone_m,
                    stop_zone_m=self.stop_zone_m,
                )
            except Exception:
                safety_stop = True
                speed_factor = 0.0

        should_stop = self.estop_active or timed_out or safety_stop

        out_cmd = Twist()
        if not should_stop:
            out_cmd.linear.x = self.last_command.linear.x * speed_factor
            out_cmd.linear.y = self.last_command.linear.y * speed_factor
            out_cmd.angular.z = self.last_command.angular.z * speed_factor

        self.cmd_vel_pub.publish(out_cmd)

        stop_msg = Bool()
        stop_msg.data = should_stop
        self.safety_stop_pub.publish(stop_msg)

        factor_msg = Float32()
        factor_msg.data = 0.0 if should_stop else float(speed_factor)
        self.speed_factor_pub.publish(factor_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyZoneNode()
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

