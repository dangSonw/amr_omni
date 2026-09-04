import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from rclpy.impl.implementation_singleton import rclpy_implementation
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Int32MultiArray, String

from .stm32_contract import TOPICS, validate_twist


class Stm32Bridge(Node):
    """Run the fixed-rate Jetson side of the STM32 ROS 2 contract.

    XRCE-DDS serial framing is owned by ``micro_ros_agent`` and the firmware
    client. This node deliberately does not open the serial device or define a
    second wire protocol. It validates the safety command and republishes the
    latest command at a deterministic rate, which makes the same ROS topic
    contract usable by the Gazebo STM32 model and by a real MCU.
    """

    def __init__(self):
        super().__init__('stm32_bridge')
        self.declare_parameter('enabled', False)
        self.declare_parameter('command_input_topic', TOPICS['safe_cmd_vel'])
        self.declare_parameter(
            'micro_ros_command_topic', TOPICS['stm32_cmd_vel'])
        self.declare_parameter('status_topic', TOPICS['status'])
        self.declare_parameter('diagnostics_topic', TOPICS['diagnostics'])
        self.declare_parameter('hardware_status_topic', 'hardware_status')
        self.declare_parameter('wheel_state_topic', TOPICS['wheel_state'])
        self.declare_parameter(
            'encoder_counts_topic', TOPICS['encoder_counts'])
        self.declare_parameter('imu_topic', TOPICS['imu'])
        self.declare_parameter('command_timeout_sec', 0.25)
        self.declare_parameter('command_frequency_hz', 50.0)
        self.declare_parameter('max_linear_speed_mps', 0.54)
        self.declare_parameter('max_angular_speed_rad_s', 3.0)
        self.declare_parameter('debug_telemetry', False)
        self.declare_parameter('debug_telemetry_frequency_hz', 2.0)

        self.enabled = bool(self.get_parameter('enabled').value)
        self.command_input_topic = str(
            self.get_parameter('command_input_topic').value)
        self.micro_ros_command_topic = str(
            self.get_parameter('micro_ros_command_topic').value)
        self.status_topic = str(self.get_parameter('status_topic').value)
        self.diagnostics_topic = str(
            self.get_parameter('diagnostics_topic').value)
        hardware_status_topic = str(
            self.get_parameter('hardware_status_topic').value)
        wheel_state_topic = str(
            self.get_parameter('wheel_state_topic').value)
        encoder_counts_topic = str(
            self.get_parameter('encoder_counts_topic').value)
        imu_topic = str(self.get_parameter('imu_topic').value)
        self.command_timeout_sec = float(
            self.get_parameter('command_timeout_sec').value)
        self.command_frequency_hz = float(
            self.get_parameter('command_frequency_hz').value)
        self.max_linear_speed_mps = float(
            self.get_parameter('max_linear_speed_mps').value)
        self.max_angular_speed_rad_s = float(
            self.get_parameter('max_angular_speed_rad_s').value)
        self.debug_telemetry = bool(
            self.get_parameter('debug_telemetry').value)
        self.debug_telemetry_frequency_hz = float(
            self.get_parameter('debug_telemetry_frequency_hz').value)
        if (self.command_timeout_sec <= 0 or
                self.command_frequency_hz <= 0 or
                self.max_linear_speed_mps <= 0 or
                self.max_angular_speed_rad_s <= 0 or
                self.debug_telemetry_frequency_hz <= 0):
            raise ValueError('invalid STM32 bridge safety parameters')

        self.last_command_time = self.get_clock().now()
        self.last_command = Twist()
        self.command_timed_out = True
        self.last_wheel_state = None
        self.last_encoder_counts = None
        self.last_imu = None
        self.hardware_status_publisher = self.create_publisher(
            String, hardware_status_topic, 10)
        self.command_publisher = self.create_publisher(
            Twist, self.micro_ros_command_topic, 10)
        self.create_subscription(
            Twist, self.command_input_topic, self._on_command, 10)
        self.create_subscription(
            String, self.status_topic, self._on_status, 10)
        self.create_subscription(
            DiagnosticArray, self.diagnostics_topic, self._on_diagnostics, 10)
        self.create_subscription(
            Float32MultiArray, wheel_state_topic, self._on_wheel_state, 10)
        self.create_subscription(
            Int32MultiArray, encoder_counts_topic,
            self._on_encoder_counts, 10)
        self.create_subscription(
            Imu, imu_topic, self._on_imu, 10)
        self.create_timer(1.0 / self.command_frequency_hz,
                          self._publish_command)
        if self.debug_telemetry:
            self.create_timer(1.0 / self.debug_telemetry_frequency_hz,
                              self._print_telemetry)
        self._publish_status(
            'disabled: micro-ROS command relay is inactive'
            if not self.enabled else 'waiting: micro-ROS status')

    def _on_command(self, message):
        values = (message.linear.x, message.linear.y, message.angular.z)
        if not validate_twist(values, self.max_linear_speed_mps,
                              self.max_angular_speed_rad_s):
            self.get_logger().warning('Ignoring invalid or unsafe cmd_vel')
            return
        self.last_command_time = self.get_clock().now()
        self.command_timed_out = False
        self.last_command = message

    def _on_status(self, message):
        self._publish_status('mcu: ' + str(message.data))

    def _on_diagnostics(self, message):
        highest_level = max(
            (self._diagnostic_level(status.level) for status in message.status),
            default=0)
        if highest_level >= 2:
            self._publish_status('mcu: diagnostic fault')
        elif highest_level == 1:
            self._publish_status('mcu: diagnostic warning')

    def _on_wheel_state(self, message):
        if len(message.data) != 12:
            self.get_logger().warning(
                'Ignoring wheel_state with %d values; expected 12' %
                len(message.data))
            return
        self.last_wheel_state = tuple(float(value) for value in message.data)

    def _on_encoder_counts(self, message):
        if len(message.data) != 4:
            self.get_logger().warning(
                'Ignoring encoder_counts with %d values; expected 4' %
                len(message.data))
            return
        self.last_encoder_counts = tuple(int(value) for value in message.data)

    def _on_imu(self, message):
        self.last_imu = message

    def _print_telemetry(self):
        if (self.last_encoder_counts is None or
                self.last_wheel_state is None or self.last_imu is None):
            self.get_logger().info('STM32 telemetry waiting for encoder/IMU')
            return
        measured = self.last_wheel_state[:4]
        self.get_logger().info(
            'STM32 telemetry encoder_counts=[%d, %d, %d, %d] '
            'encoder_rad_s=[%.3f, %.3f, %.3f, %.3f] '
            'imu_accel_mps2=[%.3f, %.3f, %.3f] '
            'imu_gyro_rad_s=[%.3f, %.3f, %.3f]' % (
                self.last_encoder_counts[0], self.last_encoder_counts[1],
                self.last_encoder_counts[2], self.last_encoder_counts[3],
                measured[0], measured[1], measured[2], measured[3],
                self.last_imu.linear_acceleration.x,
                self.last_imu.linear_acceleration.y,
                self.last_imu.linear_acceleration.z,
                self.last_imu.angular_velocity.x,
                self.last_imu.angular_velocity.y,
                self.last_imu.angular_velocity.z))

    def _diagnostic_level(self, level):
        if isinstance(level, bytes):
            return level[0] if level else 0
        return int(level)

    def _publish_command(self):
        age_sec = ((self.get_clock().now() - self.last_command_time).nanoseconds
                   * 1e-9)
        timed_out = age_sec > self.command_timeout_sec
        if timed_out and not self.command_timed_out:
            self._publish_status('stopped: command timeout')
        self.command_timed_out = timed_out
        if self.enabled:
            self.command_publisher.publish(Twist() if timed_out
                                           else self.last_command)

    def _publish_status(self, text):
        message = String()
        message.data = text
        self.hardware_status_publisher.publish(message)

    def destroy_node(self):
        if self.enabled and rclpy.ok():
            try:
                self.command_publisher.publish(Twist())
            except rclpy_implementation.RCLError:
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Stm32Bridge()
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