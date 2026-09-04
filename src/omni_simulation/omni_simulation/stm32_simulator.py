import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.impl.implementation_singleton import rclpy_implementation
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float32MultiArray, Float64, Int32MultiArray, String
from tf2_ros import TransformBroadcaster

from omni_control.kinematics import (
    WHEEL_ORDER,
    forward_kinematics,
    inverse_kinematics,
)
from omni_control.pid import ScalarKalman, WheelSpeedPid


class Stm32Simulator(Node):
    """Run the STM32 control and state-estimation loop against Gazebo sensors."""

    def __init__(self):
        super().__init__('stm32_simulator')
        self.declare_parameter('wheel_radius_m', 0.03)
        self.declare_parameter('wheelbase_m', 0.1312)
        self.declare_parameter('track_width_m', 0.1312)
        self.declare_parameter('max_wheel_speed_rad_s', 18.0)
        self.declare_parameter('max_linear_speed_mps', 0.54)
        self.declare_parameter('max_angular_speed_rad_s', 3.0)
        self.declare_parameter('command_timeout_sec', 0.25)
        self.declare_parameter('control_frequency_hz', 100.0)
        self.declare_parameter('telemetry_frequency_hz', 50.0)
        self.declare_parameter('encoder_counts_per_revolution', 2048.0)
        self.declare_parameter('motor_kp', 0.08)
        self.declare_parameter('motor_ki', 0.25)
        self.declare_parameter('motor_kd', 0.0005)
        self.declare_parameter('command_topic', 'stm32_cmd_vel')
        self.declare_parameter('joint_states_topic', 'joint_states')
        self.declare_parameter('imu_input_topic', 'imu')
        self.declare_parameter('use_joint_encoder_input', True)
        self.declare_parameter('use_imu_input', True)
        self.declare_parameter('simulation_mode', False)
        self.declare_parameter('actuator_topics', [
            '/model/amr_omni/joint/omni_wheel_joint_1/cmd_vel',
            '/model/amr_omni/joint/omni_wheel_joint_2/cmd_vel',
            '/model/amr_omni/joint/omni_wheel_joint_3/cmd_vel',
            '/model/amr_omni/joint/omni_wheel_joint_4/cmd_vel',
        ])
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('imu_output_topic', 'imu/data_raw')
        self.declare_parameter('wheel_state_topic', 'wheel_state')
        self.declare_parameter('diagnostics_topic', 'diagnostics')
        self.declare_parameter('status_topic', 'status')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_frame', 'imu_link')

        self._read_parameters()
        self.command = (0.0, 0.0, 0.0)
        self.command_time = self.get_clock().now()
        self.estop_active = False
        self.measured_wheel_speeds = [0.0] * 4
        self.target_wheel_speeds = [0.0] * 4
        self.motor_commands = [0.0] * 4
        self.last_joint_stamp_sec = None
        self.encoder_counts = [0] * 4
        self.last_encoder_counts = [None] * 4
        self.last_imu_time = self.get_clock().now()
        self.last_imu = Imu()
        self.imu_received = False
        self.x_m = 0.0
        self.y_m = 0.0
        self.yaw_rad = 0.0
        self.last_telemetry_time = None

        self.speed_pids = [
            WheelSpeedPid(self.motor_kp, self.motor_ki, self.motor_kd, 1.0)
            for _ in range(4)
        ]
        self.velocity_filters = [ScalarKalman(0.2, 0.04) for _ in range(3)]

        self.actuator_publishers = [
            self.create_publisher(Float64, topic, 10)
            for topic in self.actuator_topics
        ]
        self.odom_publisher = self.create_publisher(
            Odometry, self.odom_topic, 10)
        self.imu_publisher = self.create_publisher(
            Imu, self.imu_output_topic, 10)
        self.wheel_state_publisher = self.create_publisher(
            Float32MultiArray, self.wheel_state_topic, 10)
        self.encoder_counts_publisher = self.create_publisher(
            Int32MultiArray, 'encoder_counts', 10)
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, self.diagnostics_topic, 10)
        self.status_publisher = self.create_publisher(String, self.status_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            Twist, self.command_topic, self._on_command, 10)
        self.create_subscription(
            Bool, 'estop', self._on_estop, 10)
        if self.simulation_mode and self.use_joint_encoder_input:
            self.create_subscription(
                JointState, self.joint_states_topic, self._on_joint_state, 10,
                raw=True)
        if self.simulation_mode and self.use_imu_input:
            self.create_subscription(
                Imu, self.imu_input_topic, self._on_imu, 10, raw=True)
        self.create_timer(1.0 / self.control_frequency_hz, self._control_step)
        self.create_timer(1.0 / self.telemetry_frequency_hz,
                          self._telemetry_step)
        self._publish_status('starting')

    def _read_parameters(self):
        self.wheel_radius_m = float(self.get_parameter('wheel_radius_m').value)
        self.wheelbase_m = float(self.get_parameter('wheelbase_m').value)
        self.track_width_m = float(self.get_parameter('track_width_m').value)
        self.max_wheel_speed_rad_s = float(
            self.get_parameter('max_wheel_speed_rad_s').value)
        self.max_linear_speed_mps = float(
            self.get_parameter('max_linear_speed_mps').value)
        self.max_angular_speed_rad_s = float(
            self.get_parameter('max_angular_speed_rad_s').value)
        self.command_timeout_sec = float(
            self.get_parameter('command_timeout_sec').value)
        self.control_frequency_hz = float(
            self.get_parameter('control_frequency_hz').value)
        self.telemetry_frequency_hz = float(
            self.get_parameter('telemetry_frequency_hz').value)
        self.encoder_counts_per_revolution = float(
            self.get_parameter('encoder_counts_per_revolution').value)
        self.encoder_counts_per_rad = (
            self.encoder_counts_per_revolution / (2.0 * math.pi))
        self.motor_kp = float(self.get_parameter('motor_kp').value)
        self.motor_ki = float(self.get_parameter('motor_ki').value)
        self.motor_kd = float(self.get_parameter('motor_kd').value)
        self.command_topic = str(self.get_parameter('command_topic').value)
        self.joint_states_topic = str(
            self.get_parameter('joint_states_topic').value)
        self.imu_input_topic = str(
            self.get_parameter('imu_input_topic').value)
        self.use_joint_encoder_input = bool(
            self.get_parameter('use_joint_encoder_input').value)
        self.use_imu_input = bool(
            self.get_parameter('use_imu_input').value)
        self.simulation_mode = bool(
            self.get_parameter('simulation_mode').value)
        self.actuator_topics = [
            str(topic) for topic in self.get_parameter('actuator_topics').value
        ]
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.imu_output_topic = str(
            self.get_parameter('imu_output_topic').value)
        self.wheel_state_topic = str(
            self.get_parameter('wheel_state_topic').value)
        self.diagnostics_topic = str(
            self.get_parameter('diagnostics_topic').value)
        self.status_topic = str(self.get_parameter('status_topic').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.imu_frame = str(self.get_parameter('imu_frame').value)
        if (min(self.wheel_radius_m, self.wheelbase_m, self.track_width_m,
                self.max_wheel_speed_rad_s, self.max_linear_speed_mps,
                self.max_angular_speed_rad_s, self.command_timeout_sec,
                self.control_frequency_hz, self.telemetry_frequency_hz,
                self.encoder_counts_per_revolution) <= 0):
            raise ValueError('invalid STM32 simulator parameters')
        if len(self.actuator_topics) != 4 or not all(self.actuator_topics):
            raise ValueError('four actuator topics are required')

    def _on_command(self, message):
        values = (message.linear.x, message.linear.y, message.angular.z)
        if (not all(math.isfinite(value) for value in values) or
                abs(values[0]) > self.max_linear_speed_mps or
                abs(values[1]) > self.max_linear_speed_mps or
                abs(values[2]) > self.max_angular_speed_rad_s):
            self.get_logger().warning(
                'Ignoring invalid or unsafe stm32_cmd_vel')
            return
        self.command = tuple(float(value) for value in values)
        self.command_time = self.get_clock().now()

    def _on_estop(self, message):
        self.estop_active = bool(message.data)

    def _on_joint_state(self, message):
        if isinstance(message, (bytes, bytearray, memoryview)):
            try:
                message = deserialize_message(bytes(message), JointState)
            except (TypeError, ValueError, rclpy_implementation.RCLError) as exc:
                self.get_logger().error('Invalid joint encoder message: %s' % exc)
                self._stop_outputs()
                return
        positions = dict(zip(message.name, message.position))
        velocities = dict(zip(message.name, message.velocity)) if (hasattr(message, 'velocity') and len(message.velocity) == len(message.name)) else {}
        stamp_sec = self._message_stamp_to_sec(message)
        delta_sec = None
        if self.last_joint_stamp_sec is not None:
            candidate = stamp_sec - self.last_joint_stamp_sec
            if 0.0 < candidate <= 1.0:
                delta_sec = candidate
        for index, name in enumerate(WHEEL_ORDER):
            position = positions.get(name)
            if position is not None and math.isfinite(position):
                encoder_count = int(round(
                    position * self.encoder_counts_per_rad))
                self.encoder_counts[index] = encoder_count
                if (delta_sec is not None and
                        self.last_encoder_counts[index] is not None):
                    speed = ((encoder_count -
                              self.last_encoder_counts[index]) /
                             self.encoder_counts_per_rad /
                             delta_sec)
                    if math.isfinite(speed):
                        self.measured_wheel_speeds[index] = speed
                self.last_encoder_counts[index] = encoder_count
            vel = velocities.get(name)
            if vel is not None and math.isfinite(vel):
                self.measured_wheel_speeds[index] = vel
        self.last_joint_stamp_sec = stamp_sec

    def _on_imu(self, message):
        if isinstance(message, (bytes, bytearray, memoryview)):
            try:
                message = deserialize_message(bytes(message), Imu)
            except (TypeError, ValueError, rclpy_implementation.RCLError) as exc:
                self.get_logger().error('Invalid IMU message: %s' % exc)
                self.imu_received = False
                return
        self.last_imu = message
        self.last_imu_time = self.get_clock().now()
        self.imu_received = True

    def _message_stamp_to_sec(self, message):
        stamp = message.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return self.get_clock().now().nanoseconds * 1e-9
        return stamp.sec + stamp.nanosec * 1e-9

    def _control_step(self):
        age_sec = ((self.get_clock().now() - self.command_time).nanoseconds
                   * 1e-9)
        stale = age_sec > self.command_timeout_sec
        if (stale or self.estop_active or
                all(value == 0.0 for value in self.command)):
            self.command = (0.0, 0.0, 0.0)
            self.target_wheel_speeds = [0.0] * 4
            self.motor_commands = [0.0] * 4
            for controller in self.speed_pids:
                controller.reset()
            for publisher in self.actuator_publishers:
                message = Float64()
                message.data = 0.0
                publisher.publish(message)
            return

        target = self.command
        try:
            desired = inverse_kinematics(
                target[0], target[1], target[2], self.wheel_radius_m,
                self.wheelbase_m, self.track_width_m,
                self.max_wheel_speed_rad_s)
        except ValueError:
            desired = (0.0, 0.0, 0.0, 0.0)

        for index in range(4):
            command = float(desired[index])
            self.target_wheel_speeds[index] = command
            self.motor_commands[index] = command
            message = Float64()
            message.data = command
            self.actuator_publishers[index].publish(message)

    def _telemetry_step(self):
        try:
            wheel_twist = forward_kinematics(
                self.measured_wheel_speeds, self.wheel_radius_m,
                self.wheelbase_m, self.track_width_m)
        except ValueError:
            wheel_twist = (0.0, 0.0, 0.0)
        filtered = [
            self.velocity_filters[index].update(
                wheel_twist[index], 1.0 / self.telemetry_frequency_hz)
            for index in range(3)
        ]
        imu_age_sec = ((self.get_clock().now() - self.last_imu_time).nanoseconds
                       * 1e-9)
        if (self.imu_received and imu_age_sec <= self.command_timeout_sec and
                math.isfinite(self.last_imu.angular_velocity.z)):
            filtered[2] = self.velocity_filters[2].update(
                self.last_imu.angular_velocity.z,
                1.0 / self.telemetry_frequency_hz)
        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9
        if self.last_telemetry_time is not None:
            delta_sec = now_sec - self.last_telemetry_time
            if 0.0 < delta_sec <= 1.0:
                cos_yaw = math.cos(self.yaw_rad)
                sin_yaw = math.sin(self.yaw_rad)
                self.x_m += (cos_yaw * filtered[0] - sin_yaw * filtered[1]) * delta_sec
                self.y_m += (sin_yaw * filtered[0] + cos_yaw * filtered[1]) * delta_sec
                self.yaw_rad = math.atan2(
                    math.sin(self.yaw_rad + filtered[2] * delta_sec),
                    math.cos(self.yaw_rad + filtered[2] * delta_sec),
                )
        self.last_telemetry_time = now_sec
        stamp = now.to_msg()
        self._publish_odometry(stamp, filtered)
        self._publish_imu(stamp)
        self._publish_wheel_state()
        self._publish_encoder_counts()
        self._publish_diagnostics(stamp, imu_age_sec)

    def _publish_odometry(self, stamp, velocity):
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame
        message.pose.pose.position.x = self.x_m
        message.pose.pose.position.y = self.y_m
        message.pose.pose.orientation.z = math.sin(self.yaw_rad / 2.0)
        message.pose.pose.orientation.w = math.cos(self.yaw_rad / 2.0)
        message.twist.twist.linear.x = velocity[0]
        message.twist.twist.linear.y = velocity[1]
        message.twist.twist.angular.z = velocity[2]
        message.pose.covariance[0] = 0.001
        message.pose.covariance[7] = 0.001
        message.pose.covariance[35] = 0.01
        message.twist.covariance[0] = 0.01
        message.twist.covariance[7] = 0.01
        message.twist.covariance[35] = 0.02
        self.odom_publisher.publish(message)

        transform = TransformStamped()
        transform.header = message.header
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x_m
        transform.transform.translation.y = self.y_m
        transform.transform.rotation.z = message.pose.pose.orientation.z
        transform.transform.rotation.w = message.pose.pose.orientation.w
        self.tf_broadcaster.sendTransform(transform)

    def _publish_imu(self, stamp):
        message = Imu()
        message.header.stamp = stamp
        message.header.frame_id = self.imu_frame
        message.linear_acceleration = self.last_imu.linear_acceleration
        message.angular_velocity = self.last_imu.angular_velocity
        message.orientation = self.last_imu.orientation
        self.imu_publisher.publish(message)

    def _publish_wheel_state(self):
        message = Float32MultiArray()
        message.data = (
            list(self.measured_wheel_speeds) +
            list(self.target_wheel_speeds) +
            list(self.motor_commands)
        )
        self.wheel_state_publisher.publish(message)

    def _publish_encoder_counts(self):
        message = Int32MultiArray()
        message.data = list(self.encoder_counts)
        self.encoder_counts_publisher.publish(message)

    def _publish_diagnostics(self, stamp, imu_age_sec):
        message = DiagnosticArray()
        message.header.stamp = stamp
        status = DiagnosticStatus()
        status.name = 'stm32_simulator'
        status.hardware_id = 'gazebo-stm32-firmware-model'
        status.level = (DiagnosticStatus.ERROR if self.estop_active
                        else DiagnosticStatus.OK)
        status.message = 'estop' if self.estop_active else 'ok'
        status.values = [
            KeyValue(key='command_age_sec', value='%.4f' % (
                (self.get_clock().now() - self.command_time).nanoseconds * 1e-9)),
            KeyValue(key='imu_age_sec', value='%.4f' % imu_age_sec),
        ]
        message.status = [status]
        self.diagnostics_publisher.publish(message)
        self._publish_status(status.message)

    def _publish_status(self, text):
        message = String()
        message.data = text
        self.status_publisher.publish(message)

    def _stop_outputs(self):
        self.command = (0.0, 0.0, 0.0)
        self.estop_active = True
        self.target_wheel_speeds = [0.0] * 4
        self.motor_commands = [0.0] * 4
        for controller in self.speed_pids:
            controller.reset()
        for publisher in self.actuator_publishers:
            message = Float64()
            message.data = 0.0
            publisher.publish(message)

    def destroy_node(self):
        if rclpy.ok():
            for publisher in self.actuator_publishers:
                message = Float64()
                message.data = 0.0
                try:
                    publisher.publish(message)
                except rclpy_implementation.RCLError:
                    pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Stm32Simulator()
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