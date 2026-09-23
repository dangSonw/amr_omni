import json
import math
import random

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.exceptions import InvalidHandle
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float64, String
from tf2_ros import TransformBroadcaster

from omni_control.kinematics import (
    WHEEL_ORDER,
    forward_kinematics,
    inverse_kinematics,
)

POSE_POSITION_COV = 0.001
POSE_YAW_COV = 0.01
TWIST_LINEAR_COV = 0.01
TWIST_ANGULAR_COV = 0.02
IMU_ORIENT_COV = 0.0025
IMU_GYRO_COV = 0.0001
IMU_ACCEL_COV = 0.01

# Serial Protocol Constants & Message IDs (Matching firmware/stm32_f407vg_arduino_sim)
SERIAL_HEADER_SYNC_0 = 0xAA
SERIAL_HEADER_SYNC_1 = 0x55
SERIAL_TAIL_BYTE = 0x7D
SERIAL_MAX_PAYLOAD_LEN = 64
SERIAL_FRAME_OVERHEAD = 8
CMD_CALIB_TRIGGER_IMU = 0x10
RESP_ACK_NACK = 0x80

STANDARD_GRAVITY_MPS2 = 9.80665
DEFAULT_GYRO_NOISE_DENSITY = 1.4e-4
DEFAULT_ACCEL_NOISE_DENSITY = 1.9e-3
DEFAULT_INFLATION_ALPHA = 1.8


class SimplePID:
    """Simple PID controller matching firmware WheelSpeedPid behavior."""
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_limit=1.0):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.output_limit = float(output_limit)
        self.integral = 0.0
        self.previous_measurement = 0.0
        self.initialized = False

    def reset(self):
        self.integral = 0.0
        self.previous_measurement = 0.0
        self.initialized = False

    def is_passthrough(self):
        return abs(self.kp - 1.0) < 1e-5 and abs(self.ki) < 1e-5 and abs(self.kd) < 1e-5

    def set_tunings(self, kp, ki, kd):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)

    def update(self, setpoint, measurement, dt):
        if not math.isfinite(setpoint) or not math.isfinite(measurement) or dt <= 0:
            self.reset()
            return 0.0
        if self.is_passthrough():
            self.integral = 0.0
            self.previous_measurement = measurement
            self.initialized = True
            return 0.0
        error = setpoint - measurement
        if not self.initialized:
            self.previous_measurement = measurement
            self.initialized = True
        derivative = -(measurement - self.previous_measurement) / dt
        candidate_integral = max(-self.output_limit, min(self.output_limit, self.integral + error * dt))
        output = self.kp * error + self.ki * candidate_integral + self.kd * derivative
        output = max(-self.output_limit, min(self.output_limit, output))
        if abs(output) < self.output_limit or output * error < 0:
            self.integral = candidate_integral
        self.previous_measurement = measurement
        return output


class ScalarKalman:
    """Internal scalar Kalman filter used to simulate STM32 sensor smoothing."""

    def __init__(self, process_noise, measurement_noise):
        if process_noise < 0 or measurement_noise <= 0:
            raise ValueError('Kalman noise parameters are invalid')
        self.process_noise = float(process_noise)
        self.measurement_noise = float(measurement_noise)
        self.estimate_value = 0.0
        self.covariance = 1.0
        self.initialized = False

    def set_noise(self, process_noise, measurement_noise):
        if process_noise < 0 or measurement_noise <= 0:
            raise ValueError('Kalman noise parameters are invalid')
        self.process_noise = float(process_noise)
        self.measurement_noise = float(measurement_noise)

    def reset(self, estimate=0.0, covariance=1.0):
        if not math.isfinite(estimate) or covariance <= 0:
            raise ValueError('Kalman reset values are invalid')
        self.estimate_value = float(estimate)
        self.covariance = float(covariance)
        self.initialized = True

    def update(self, measurement, delta_sec):
        if not math.isfinite(measurement):
            raise ValueError('Kalman measurement must be finite')
        delta_sec = max(float(delta_sec), 1e-6)
        if not self.initialized:
            self.reset(measurement)
            return self.estimate_value

        self.covariance += self.process_noise * delta_sec
        gain = self.covariance / (self.covariance + self.measurement_noise)
        self.estimate_value += gain * (measurement - self.estimate_value)
        self.covariance *= 1.0 - gain
        return self.estimate_value


class EncoderPll:
    """Second-Order Phase-Locked Loop (PLL) Tracking Observer & LinuxCNC M/T Hybrid Velocity Estimator.
    
    Provides critically damped (zeta = 1.0) velocity estimation with zero steady-state phase lag,
    16-bit hardware timer rollover-safe difference arithmetic, and 50 ms zero-speed watchdog.
    Faithful port of firmware/stm32_f407vg_arduino_sim/src/encoder_pll.cpp.
    """

    def __init__(self, bandwidth_rad_s=20.0, counts_per_revolution=2048):
        self.counts_per_rev = max(1, int(counts_per_revolution))
        self.rad_per_count = (2.0 * math.pi) / float(self.counts_per_rev)
        self.pll_bandwidth = 20.0
        self.kp = 0.0
        self.ki = 0.0
        self.pos_estimate_rad = 0.0
        self.vel_estimate_rad_s = 0.0
        self.pos_error_rad = 0.0
        self.time_since_last_pulse_sec = 0.0
        self.zero_speed_timeout_sec = 0.050  # 50 ms timeout
        self.prev_timer_count = 0
        self.timer_initialized = False
        self.set_bandwidth(bandwidth_rad_s)

    def set_bandwidth(self, bandwidth_rad_s):
        self.pll_bandwidth = float(bandwidth_rad_s) if bandwidth_rad_s > 0.0 else 20.0
        self.kp = 2.0 * self.pll_bandwidth
        self.ki = 0.25 * (self.kp * self.kp)  # ki = omega_pll^2 (zeta = 1.0 critical damping)

    def reset(self, initial_pos_rad=0.0, initial_vel_rad_s=0.0):
        self.pos_estimate_rad = float(initial_pos_rad)
        self.vel_estimate_rad_s = float(initial_vel_rad_s)
        self.pos_error_rad = 0.0
        self.time_since_last_pulse_sec = 0.0
        self.timer_initialized = False

    def update(self, delta_counts, delta_sec):
        dt = float(delta_sec) if delta_sec > 0.0 else 0.01

        # 1. Zero-speed watchdog tracking
        if delta_counts == 0:
            self.time_since_last_pulse_sec += dt
            if self.time_since_last_pulse_sec >= (self.zero_speed_timeout_sec - 0.0001):
                self.vel_estimate_rad_s = 0.0
                self.pos_error_rad = 0.0
                return 0.0
        else:
            self.time_since_last_pulse_sec = 0.0

        # 2. Measured angular increment
        delta_theta_m = float(delta_counts) * self.rad_per_count

        # 3. Predicted angular increment
        delta_theta_pred = dt * self.vel_estimate_rad_s

        # 4. Innovation residual
        residual = self.pos_error_rad + delta_theta_m - delta_theta_pred
        prior_vel = self.vel_estimate_rad_s

        # 5. State corrections (Second-Order tracking observer)
        self.vel_estimate_rad_s += dt * self.ki * residual
        pos_correction = dt * self.kp * residual
        self.pos_estimate_rad += delta_theta_pred + pos_correction
        self.pos_error_rad = residual - pos_correction

        # Zero-crossing prevention when no pulses arrive (monotonic decay)
        if delta_counts == 0:
            if ((prior_vel > 0.0 and self.vel_estimate_rad_s < 0.0) or
                    (prior_vel < 0.0 and self.vel_estimate_rad_s > 0.0)):
                self.vel_estimate_rad_s = 0.0
                self.pos_error_rad = 0.0

        # 6. Numerical safeguards
        if not math.isfinite(self.vel_estimate_rad_s) or not math.isfinite(self.pos_estimate_rad):
            self.reset(0.0, 0.0)
            return 0.0

        # 7. LinuxCNC M/T decay envelope at ultra-low speeds
        if delta_counts == 0 and self.time_since_last_pulse_sec > 0.0:
            max_possible_speed = self.rad_per_count / self.time_since_last_pulse_sec
            if abs(self.vel_estimate_rad_s) > max_possible_speed:
                self.vel_estimate_rad_s = max_possible_speed if self.vel_estimate_rad_s > 0.0 else -max_possible_speed

        return self.vel_estimate_rad_s

    def update_raw(self, current_raw_timer_count, delta_sec):
        current_raw_timer_count = int(current_raw_timer_count) & 0xFFFF
        if not self.timer_initialized:
            self.prev_timer_count = current_raw_timer_count
            self.timer_initialized = True
            return self.vel_estimate_rad_s
        diff = (current_raw_timer_count - self.prev_timer_count) & 0xFFFF
        if diff >= 0x8000:
            diff -= 0x10000
        self.prev_timer_count = current_raw_timer_count
        return self.update(diff, delta_sec)



def compute_imu_covariances(dt_sec):
    """Compute IMU covariance matrices for ROS 2 Imu message.

    BNO08x handles calibration internally - this only provides noise
    covariance values for EKF fusion.
    """
    dt = max(float(dt_sec), 0.001)
    alpha_sq = DEFAULT_INFLATION_ALPHA ** 2

    gyro_var = max(1.0e-4, alpha_sq * (DEFAULT_GYRO_NOISE_DENSITY ** 2) / dt)
    accel_var = max(1.0e-2, alpha_sq * (DEFAULT_ACCEL_NOISE_DENSITY ** 2) / dt)

    angular_vel_cov = [
        1.0e6, 0.0, 0.0,
        0.0, 1.0e6, 0.0,
        0.0, 0.0, gyro_var
    ]
    linear_accel_cov = [
        accel_var, 0.0, 0.0,
        0.0, accel_var, 0.0,
        0.0, 0.0, accel_var
    ]
    return angular_vel_cov, linear_accel_cov

def compute_crc16_ccitt(data: bytes, init_val: int = 0xFFFF) -> int:
    crc = init_val
    for byte in data:
        crc ^= (byte << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def serialize_serial_frame(seq: int, msg_id: int, payload: bytes = b'') -> bytes:
    if len(payload) > SERIAL_MAX_PAYLOAD_LEN:
        raise ValueError('Payload exceeds max length')
    header_and_body = bytearray([
        SERIAL_HEADER_SYNC_0,
        SERIAL_HEADER_SYNC_1,
        len(payload),
        seq & 0xFF,
        msg_id & 0xFF,
    ])
    header_and_body.extend(payload)
    crc = compute_crc16_ccitt(bytes(header_and_body[2:]), 0xFFFF)
    crc_bytes = bytes([crc & 0xFF, (crc >> 8) & 0xFF])
    return bytes(header_and_body) + crc_bytes + bytes([SERIAL_TAIL_BYTE])


def deserialize_serial_frame(buffer: bytes):
    if len(buffer) < SERIAL_FRAME_OVERHEAD:
        return None, 0
    if buffer[0] != SERIAL_HEADER_SYNC_0 or buffer[1] != SERIAL_HEADER_SYNC_1:
        return None, 0
    payload_len = buffer[2]
    if payload_len > SERIAL_MAX_PAYLOAD_LEN:
        return None, 0
    total_len = SERIAL_FRAME_OVERHEAD + payload_len
    if len(buffer) < total_len:
        return None, 0
    if buffer[total_len - 1] != SERIAL_TAIL_BYTE:
        return None, 0

    crc_recv = buffer[5 + payload_len] | (buffer[6 + payload_len] << 8)
    crc_calc = compute_crc16_ccitt(buffer[2: 5 + payload_len], 0xFFFF)
    if crc_recv != crc_calc:
        return None, 0

    seq = buffer[3]
    msg_id = buffer[4]
    payload = buffer[5: 5 + payload_len]
    return {'seq': seq, 'msg_id': msg_id, 'payload': payload}, total_len


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
        self.declare_parameter('motor_kp', 1.0)
        self.declare_parameter('motor_ki', 0.0)
        self.declare_parameter('motor_kd', 0.0)
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
        self.declare_parameter('odom_topic', 'wheel/odom')
        self.declare_parameter('imu_output_topic', 'imu/data')
        self.declare_parameter('wheel_state_topic', 'wheel_state')
        self.declare_parameter('diagnostics_topic', 'diagnostics')
        self.declare_parameter('status_topic', 'status')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_frame', 'imu_link')

        self._read_parameters()
        self.command = (0.0, 0.0, 0.0)
        self.command_time = self.get_clock().now()
        self.estop_active = False
        self.measured_wheel_speeds = [0.0] * 4
        self.raw_wheel_speeds = [0.0] * 4
        self.target_wheel_speeds = [0.0] * 4
        self.motor_commands = [0.0] * 4
        self.last_joint_stamp_sec = None
        self.encoder_counts = [0] * 4
        self.last_encoder_counts = [None] * 4
        self.last_imu_time = self.get_clock().now()
        self._last_imu_pub_time = None
        self.last_imu = Imu()
        self.imu_received = False
        self.x_m = 0.0
        self.y_m = 0.0
        self.yaw_rad = 0.0
        self.imu_integrated_yaw_rad = 0.0
        self.last_telemetry_time = None
        self.wheel_filters = [ScalarKalman(0.5, 0.04) for _ in range(4)]
        self.wheel_plls = [
            EncoderPll(bandwidth_rad_s=20.0, counts_per_revolution=self.encoder_counts_per_revolution)
            for _ in range(4)
        ]
        self.raw_accel_sample = [0.0, 0.0, 9.80665]
        self.raw_gyro_sample = [0.0, 0.0, 0.0]
        self.raw_mag_sample = [0.0, 20.0, -45.0]
        self.running_gyro_variance = 0.0001
        self.running_accel_variance = 0.0025
        self.is_stationary = True
        self.sim_orientation = None  # [roll_deg, pitch_deg, yaw_deg] or None
        self.serial_seq = 0
        self.speed_pids = [SimplePID(self.motor_kp, self.motor_ki, self.motor_kd) for _ in range(4)]

        self.actuator_publishers = [
            self.create_publisher(Float64, topic, 10)
            for topic in self.actuator_topics
        ]
        self.odom_publisher = self.create_publisher(
            Odometry, self.odom_topic, 10)
        self.imu_publisher = self.create_publisher(
            Imu, self.imu_output_topic, 10)
        self.status_publisher = self.create_publisher(String, self.status_topic, 10)
        self.debug_publisher = self.create_publisher(String, 'debug/data', 10)
        self.config_cmd_sub = self.create_subscription(
            String, 'config/cmd', self._on_config_command, 10)
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
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
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

    def _on_config_command(self, message):
        try:
            payload = json.loads(message.data)
        except Exception as exc:
            self.get_logger().error(f"Failed to parse config/cmd JSON: {exc}")
            return

        if "motor_kp" in payload:
            val = payload["motor_kp"]
            arr = val if isinstance(val, list) else [float(val)] * 4
            for i in range(min(4, len(arr))):
                self.speed_pids[i].kp = float(arr[i])

        if "motor_ki" in payload:
            val = payload["motor_ki"]
            arr = val if isinstance(val, list) else [float(val)] * 4
            for i in range(min(4, len(arr))):
                self.speed_pids[i].ki = float(arr[i])

        if "motor_kd" in payload:
            val = payload["motor_kd"]
            arr = val if isinstance(val, list) else [float(val)] * 4
            for i in range(min(4, len(arr))):
                self.speed_pids[i].kd = float(arr[i])

        if "kalman_q" in payload:
            val = payload["kalman_q"]
            arr = val if isinstance(val, list) else [float(val)] * 4
            for i in range(min(4, len(arr))):
                self.wheel_filters[i].process_noise = float(arr[i])

        if "kalman_r" in payload:
            val = payload["kalman_r"]
            arr = val if isinstance(val, list) else [float(val)] * 4
            for i in range(min(4, len(arr))):
                self.wheel_filters[i].measurement_noise = float(arr[i])

        self.get_logger().info(f"Updated PID/Kalman config from config/cmd: {payload}")

    def _on_joint_state(self, message):
        if isinstance(message, (bytes, bytearray, memoryview)):
            try:
                message = deserialize_message(bytes(message), JointState)
            except (TypeError, ValueError, InvalidHandle, Exception) as exc:
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
        dt = delta_sec if delta_sec is not None else (1.0 / self.control_frequency_hz)

        for index, name in enumerate(WHEEL_ORDER):
            position = positions.get(name)
            if position is not None and math.isfinite(position):
                encoder_count = int(round(
                    position * self.encoder_counts_per_rad))
                self.encoder_counts[index] = encoder_count
                if self.last_encoder_counts[index] is not None:
                    delta_counts = encoder_count - self.last_encoder_counts[index]
                else:
                    delta_counts = 0

                # Second-Order PLL tracking observer for wheel angular velocity
                pll_speed = self.wheel_plls[index].update(delta_counts, dt)
                self.measured_wheel_speeds[index] = pll_speed

                if delta_sec is not None and self.last_encoder_counts[index] is not None and delta_sec > 0:
                    speed = ((encoder_count -
                              self.last_encoder_counts[index]) /
                             self.encoder_counts_per_rad /
                             delta_sec)
                    if math.isfinite(speed):
                        self.raw_wheel_speeds[index] = speed
                        self.wheel_filters[index].update(speed, delta_sec)
                self.last_encoder_counts[index] = encoder_count

            vel = velocities.get(name)
            if vel is not None and math.isfinite(vel) and (position is None):
                self.raw_wheel_speeds[index] = vel
                self.measured_wheel_speeds[index] = (
                    self.wheel_filters[index].update(vel, dt)
                )
        self.last_joint_stamp_sec = stamp_sec

    def _on_imu(self, message):
        if isinstance(message, (bytes, bytearray, memoryview)):
            try:
                message = deserialize_message(bytes(message), Imu)
            except (TypeError, ValueError, InvalidHandle, Exception) as exc:
                self.get_logger().error('Invalid IMU message: %s' % exc)
                self.imu_received = False
                return
        self.last_imu = message
        self.last_imu_time = self.get_clock().now()
        self.imu_received = True

        if self.sim_orientation is not None:
            roll_r = math.radians(self.sim_orientation[0])
            pitch_r = math.radians(self.sim_orientation[1])
            yaw_r = math.radians(self.sim_orientation[2])
            g = STANDARD_GRAVITY_MPS2
            raw_a = [
                -g * math.sin(pitch_r),
                g * math.sin(roll_r) * math.cos(pitch_r),
                g * math.cos(roll_r) * math.cos(pitch_r),
            ]
            raw_w = [0.0, 0.0, 0.0]
            quat = Rotation.from_euler('xyz', [roll_r, pitch_r, yaw_r]).as_quat()
            self.last_imu.orientation.x = float(quat[0])
            self.last_imu.orientation.y = float(quat[1])
            self.last_imu.orientation.z = float(quat[2])
            self.last_imu.orientation.w = float(quat[3])
        else:
            raw_a = [
                float(message.linear_acceleration.x),
                float(message.linear_acceleration.y),
                float(message.linear_acceleration.z),
            ]
            raw_w = [
                float(message.angular_velocity.x),
                float(message.angular_velocity.y),
                float(message.angular_velocity.z),
            ]
        self.raw_accel_sample = list(raw_a)
        self.raw_gyro_sample = list(raw_w)

        # Running variance update
        gyro_norm_sq = raw_w[0]**2 + raw_w[1]**2 + raw_w[2]**2
        self.running_gyro_variance = 0.95 * self.running_gyro_variance + 0.05 * gyro_norm_sq
        accel_err = (math.sqrt(raw_a[0]**2 + raw_a[1]**2 + raw_a[2]**2) - STANDARD_GRAVITY_MPS2)**2
        self.running_accel_variance = 0.95 * self.running_accel_variance + 0.05 * accel_err
        # Simple stationary detection: low gyro and near-1g accel
        self.is_stationary = (gyro_norm_sq < 0.01 and accel_err < 0.25)

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

        dt = 1.0 / self.control_frequency_hz
        for index in range(4):
            target = float(desired[index])
            self.target_wheel_speeds[index] = target
            measured = self.measured_wheel_speeds[index]
            feedback = self.speed_pids[index].update(target, measured, dt)
            feedforward = target / self.max_wheel_speed_rad_s
            command = max(-1.0, min(1.0, feedforward + feedback))
            self.motor_commands[index] = command
            message = Float64()
            message.data = command * self.max_wheel_speed_rad_s
            self.actuator_publishers[index].publish(message)

    def _telemetry_step(self):
        try:
            wheel_twist = forward_kinematics(
                self.measured_wheel_speeds, self.wheel_radius_m,
                self.wheelbase_m, self.track_width_m)
        except ValueError:
            wheel_twist = (0.0, 0.0, 0.0)
        vx = wheel_twist[0]
        vy = wheel_twist[1]
        wz = wheel_twist[2]

        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9
        if self.last_telemetry_time is not None:
            delta_sec = now_sec - self.last_telemetry_time
            if 0.0 < delta_sec <= 1.0:
                self.yaw_rad = math.atan2(
                    math.sin(self.yaw_rad + wz * delta_sec),
                    math.cos(self.yaw_rad + wz * delta_sec),
                )
                cos_yaw = math.cos(self.yaw_rad)
                sin_yaw = math.sin(self.yaw_rad)
                self.x_m += (cos_yaw * vx - sin_yaw * vy) * delta_sec
                self.y_m += (sin_yaw * vx + cos_yaw * vy) * delta_sec
        self.last_telemetry_time = now_sec
        stamp = now.to_msg()
        self._publish_odometry(stamp, (vx, vy, wz))
        self._publish_imu(stamp)
        self._publish_status('estop' if self.estop_active else 'ok')

        # Cập nhật từ trường mô phỏng 3 trục theo góc yaw thân xe + nhiễu nhẹ cảm biến (~0.25 uT)
        cos_yaw = math.cos(self.yaw_rad)
        sin_yaw = math.sin(self.yaw_rad)
        mag_noise_std = 0.25  # uT
        self.raw_mag_sample = [
            round(20.0 * cos_yaw + random.gauss(0.0, mag_noise_std), 2),
            round(-20.0 * sin_yaw + random.gauss(0.0, mag_noise_std), 2),
            round(-45.0 + random.gauss(0.0, mag_noise_std), 2),
        ]
        
        debug_data = {
            'raw_wheel_speed_rad_s': self.raw_wheel_speeds,
            'filtered_wheel_speed_rad_s': self.measured_wheel_speeds,
            'target_wheel_speed_rad_s': self.target_wheel_speeds,
            'motor_output': self.motor_commands,
            'imu_accel_xyz': self.raw_accel_sample,
            'imu_gyro_xyz': self.raw_gyro_sample,
            'imu_mag_xyz': getattr(self, 'raw_mag_sample', [0.0, 20.0, -45.0]),
            'imu_quaternion_xyzw': [
                self.last_imu.orientation.x,
                self.last_imu.orientation.y,
                self.last_imu.orientation.z,
                self.last_imu.orientation.w,
            ],
            'body_vx_mps': vx,
            'body_vy_mps': vy,
            'body_wz_rad_s': wz,
            'cmd_vx_mps': self.command[0],
            'cmd_vy_mps': self.command[1],
            'cmd_wz_rad_s': self.command[2],
            'noise_profile': 'gazebo_native',
            'is_stationary': self.is_stationary,
            'raw_gyro_stddev': math.sqrt(max(0.0, self.running_gyro_variance)),
            'raw_accel_stddev': math.sqrt(max(0.0, self.running_accel_variance)),
            'imu_integrated_yaw_deg': round(math.degrees(self.imu_integrated_yaw_rad), 2),
            'time_delay_ms': 12.5,
            'sim_orientation': self.sim_orientation,
        }
        msg = String()
        msg.data = json.dumps(debug_data)
        self.debug_publisher.publish(msg)

    def _publish_odometry(self, stamp, velocity):
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame
        message.pose.pose.position.x = self.x_m
        message.pose.pose.position.y = self.y_m
        quat = Rotation.from_euler('z', self.yaw_rad).as_quat()
        message.pose.pose.orientation.x = float(quat[0])
        message.pose.pose.orientation.y = float(quat[1])
        message.pose.pose.orientation.z = float(quat[2])
        message.pose.pose.orientation.w = float(quat[3])
        message.twist.twist.linear.x = velocity[0]
        message.twist.twist.linear.y = velocity[1]
        message.twist.twist.angular.z = velocity[2]
        pose_cov = [0.0] * 36
        pose_cov[0] = POSE_POSITION_COV
        pose_cov[7] = POSE_POSITION_COV
        pose_cov[14] = 1.0e6
        pose_cov[21] = 1.0e6
        pose_cov[28] = 1.0e6
        pose_cov[35] = POSE_YAW_COV
        message.pose.covariance = pose_cov

        twist_cov = [0.0] * 36
        twist_cov[0] = TWIST_LINEAR_COV
        twist_cov[7] = TWIST_LINEAR_COV
        twist_cov[14] = 1.0e6
        twist_cov[21] = 1.0e6
        twist_cov[28] = 1.0e6
        twist_cov[35] = TWIST_ANGULAR_COV
        message.twist.covariance = twist_cov
        self.odom_publisher.publish(message)

        if self.publish_tf:
            transform = TransformStamped()
            transform.header = message.header
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self.x_m
            transform.transform.translation.y = self.y_m
            transform.transform.rotation = message.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

    def _publish_imu(self, stamp):
        message = Imu()
        message.header.stamp = stamp
        message.header.frame_id = self.imu_frame
        message.orientation = self.last_imu.orientation

        raw_accel = getattr(
            self, 'raw_accel_sample',
            [self.last_imu.linear_acceleration.x, self.last_imu.linear_acceleration.y, self.last_imu.linear_acceleration.z]
        )
        raw_gyro = getattr(
            self, 'raw_gyro_sample',
            [self.last_imu.angular_velocity.x, self.last_imu.angular_velocity.y, self.last_imu.angular_velocity.z]
        )
        # BNO08x has internal sensor fusion & calibration, no external 6-face AN4508 or gyro drift filter
        calib_accel = list(raw_accel)
        calib_gyro = list(raw_gyro)

        message.angular_velocity.x = float(calib_gyro[0])
        message.angular_velocity.y = float(calib_gyro[1])
        message.angular_velocity.z = float(calib_gyro[2])
        message.linear_acceleration.x = float(calib_accel[0])
        message.linear_acceleration.y = float(calib_accel[1])
        message.linear_acceleration.z = float(calib_accel[2])

        # Measured real dt from clock (Bug 5 fix)
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self._last_imu_pub_time is not None:
            dt_sec = max(1e-4, min(1.0, now_sec - self._last_imu_pub_time))
        else:
            dt_sec = 1.0 / self.telemetry_frequency_hz
        self._last_imu_pub_time = now_sec

        self.imu_integrated_yaw_rad += float(calib_gyro[2]) * dt_sec
        ang_cov, lin_cov = compute_imu_covariances(dt_sec)
        message.orientation_covariance = [
            1.0e6, 0.0, 0.0,
            0.0, 1.0e6, 0.0,
            0.0, 0.0, IMU_ORIENT_COV,
        ]
        message.angular_velocity_covariance = ang_cov
        message.linear_acceleration_covariance = lin_cov
        self.imu_publisher.publish(message)

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
        for pll in self.wheel_plls:
            pll.reset(0.0, 0.0)
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
                except (InvalidHandle, Exception):
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