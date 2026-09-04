from __future__ import annotations

import logging
import math
import threading
import time
from typing import List, Optional

from app.bridges.base import BaseRobotBridge
from app.models import (
    ImuTelemetry,
    LidarTelemetry,
    OdometryTelemetry,
    RobotConfig,
    RobotStatus,
    WheelTelemetry,
)
from app.services.stream_monitor import stream_monitor

logger = logging.getLogger("amr_web.ros2_bridge")

# Kiểm tra sự sẵn sàng của rclpy
try:
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, LaserScan
    from std_msgs.msg import Bool, Float32MultiArray, Int32MultiArray, String
    HAS_RCLPY = True
except ImportError as e:
    logger.debug(f"rclpy không thể import: {e}")
    HAS_RCLPY = False
    LaserScan = None
    Imu = None
    Odometry = None
    Twist = None
    Float32MultiArray = None
    Int32MultiArray = None
    String = None
    DiagnosticArray = None
    Bool = None
    Node = None
    SingleThreadedExecutor = None
    qos_profile_sensor_data = None


class Ros2Bridge(BaseRobotBridge):
    """
    ROS 2 Jazzy Bridge sử dụng rclpy.
    Kết nối trực tiếp vào ROS graph của robot (Gazebo / Renode STM32 / Jetson thật).
    Đọc chính xác dữ liệu từ các topic:
      - /scan (LiDAR, BestEffort sensor QoS)
      - /odom (Odometry)
      - /wheel_state (12 floats từ STM32: measured, target, motor PWM)
      - /encoder_counts (4 int ticks từ STM32)
      - /imu/data_raw và /imu (IMU)
      - /status và /diagnostics (Trạng thái và chẩn đoán STM32)
      - /safe_cmd_vel và /stm32_cmd_vel (Giám sát luồng an toàn Jetson -> STM32)
    Gửi lệnh điều khiển chuẩn:
      - /cmd_vel (Twist -> watchdog -> safe_cmd_vel -> stm32_bridge -> stm32)
      - /estop (Dừng khẩn cấp)
    """

    def __init__(self):
        if not HAS_RCLPY:
            raise RuntimeError("rclpy không khả dụng. Hãy source /opt/ros/jazzy/setup.bash.")

        self.config = RobotConfig()
        self.node: Optional[Node] = None
        self.executor: Optional[SingleThreadedExecutor] = None
        self.spin_thread: Optional[threading.Thread] = None
        self.running = False
        self.start_time = time.time()

        # Trạng thái kết nối và an toàn
        self.status_msg = "OK"
        self.estop_active = False
        self.safety_stop = False
        self.last_cmd_time = time.time()

        # Dữ liệu 4 bánh Mecanum từ STM32
        self.measured_wheel_speeds = [0.0, 0.0, 0.0, 0.0]
        self.target_wheel_speeds = [0.0, 0.0, 0.0, 0.0]
        self.motor_commands = [0.0, 0.0, 0.0, 0.0]
        self.encoder_ticks = [0, 0, 0, 0]

        # Dữ liệu IMU 6-DoF
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 9.81
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0

        # Dữ liệu Odometry
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.odom_vx = 0.0
        self.odom_vy = 0.0
        self.odom_wz = 0.0

        # Dữ liệu LiDAR
        self.lidar_ranges: List[float] = []
        self.lidar_points: List[List[float]] = []

    async def start(self) -> None:
        if self.running:
            return

        if not rclpy.ok():
            rclpy.init()

        self.node = Node("amr_web_bridge")
        # Đồng bộ thời gian mô phỏng Gazebo nếu có
        if not self.node.has_parameter("use_sim_time"):
            try:
                self.node.declare_parameter("use_sim_time", True)
            except Exception:
                pass
        self.start_time = time.time()

        # Publishers
        self.cmd_vel_pub = self.node.create_publisher(Twist, "cmd_vel", 10)
        self.safe_cmd_vel_pub = self.node.create_publisher(Twist, "safe_cmd_vel", 10)
        self.stm32_cmd_vel_pub = self.node.create_publisher(Twist, "stm32_cmd_vel", 10)
        self.estop_pub = self.node.create_publisher(Bool, "estop", 10)

        # Subscribers - Cảm biến môi trường & Gazebo (dùng SensorDataQoS BestEffort)
        self.node.create_subscription(LaserScan, "scan", self._on_scan, qos_profile_sensor_data)
        self.node.create_subscription(Imu, "imu/data_raw", self._on_imu, qos_profile_sensor_data)
        self.node.create_subscription(Imu, "imu", self._on_imu, qos_profile_sensor_data)

        # Subscribers - Phản hồi từ STM32 / Mô phỏng STM32
        self.node.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.node.create_subscription(Float32MultiArray, "wheel_state", self._on_wheel_state, 10)
        self.node.create_subscription(Int32MultiArray, "encoder_counts", self._on_encoder_counts, 10)
        self.node.create_subscription(String, "status", self._on_status, 10)
        self.node.create_subscription(DiagnosticArray, "diagnostics", self._on_diagnostics, 10)

        # Subscribers - Giám sát an toàn Jetson Watchdog & stm32_bridge
        self.node.create_subscription(Bool, "safety_stop", self._on_safety_stop, 10)
        self.node.create_subscription(Twist, "safe_cmd_vel", self._on_safe_cmd_vel, 10)
        self.node.create_subscription(Twist, "stm32_cmd_vel", self._on_stm32_cmd_vel, 10)

        # Chạy executor trong background thread
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.running = True

        def _run_spin():
            try:
                self.executor.spin()
            except (rclpy.executors.ExternalShutdownException, Exception):
                pass

        self.spin_thread = threading.Thread(target=_run_spin, daemon=True)
        self.spin_thread.start()
        logger.info("ROS 2 Bridge node 'amr_web_bridge' đã khởi động và đăng ký các topic.")

    async def stop(self) -> None:
        self.running = False
        if self.executor:
            self.executor.shutdown()
        if self.node:
            self.node.destroy_node()
        if self.spin_thread and self.spin_thread.is_alive():
            self.spin_thread.join(timeout=1.0)
        if rclpy.ok():
            rclpy.shutdown()
        logger.info("ROS 2 Bridge node đã dừng.")

    async def send_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        if not self.node or not self.running:
            return
        if self.estop_active:
            logger.warning("E-STOP đang kích hoạt, từ chối gửi lệnh vận tốc.")
            return
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)

        # Gửi tới cmd_vel chuẩn (được giám sát bởi command_watchdog)
        self.cmd_vel_pub.publish(msg)
        self.last_cmd_time = time.time()
        stream_monitor.record("navigation_cmd_vel", f"vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f}")

    async def send_estop(self, active: bool) -> None:
        if not self.node or not self.running:
            return
        self.estop_active = active
        msg = Bool()
        msg.data = bool(active)
        self.estop_pub.publish(msg)
        # Nếu estop kích hoạt, gửi ngay lập tức lệnh vận tốc 0
        if active:
            zero_twist = Twist()
            self.cmd_vel_pub.publish(zero_twist)
            self.safe_cmd_vel_pub.publish(zero_twist)
            self.stm32_cmd_vel_pub.publish(zero_twist)

    def reset_odometry(self) -> None:
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        logger.info("Reset odometry coordinates to (0, 0, 0)")

    def get_status(self) -> RobotStatus:
        now = time.time()
        # Nếu không nhận được lệnh trong 0.5s -> watchdog stop
        watchdog_triggered = (now - self.last_cmd_time) > self.config.safety_timeout_sec
        return RobotStatus(
            mode="ros2",
            connected=self.running and (self.node is not None),
            estop_active=self.estop_active,
            safety_stop=self.safety_stop or watchdog_triggered,
            uptime_sec=round(now - self.start_time, 1),
            cpu_percent=0.0,
            ram_percent=0.0,
            active_streams=stream_monitor.get_active_count(),
            timestamp_sec=now,
        )

    def get_wheel_telemetry(self) -> WheelTelemetry:
        return WheelTelemetry(
            target_rad_s=[round(s, 2) for s in self.target_wheel_speeds],
            measured_rad_s=[round(s, 2) for s in self.measured_wheel_speeds],
            encoder_ticks=list(self.encoder_ticks),
            pwm_commands=[round(s, 2) for s in self.motor_commands],
        )

    def get_imu_telemetry(self) -> ImuTelemetry:
        return ImuTelemetry(
            roll_deg=round(self.roll_deg, 2),
            pitch_deg=round(self.pitch_deg, 2),
            yaw_deg=round(self.yaw_deg % 360.0, 2),
            accel_x=round(self.accel_x, 3),
            accel_y=round(self.accel_y, 3),
            accel_z=round(self.accel_z, 3),
            gyro_x=round(self.gyro_x, 3),
            gyro_y=round(self.gyro_y, 3),
            gyro_z=round(self.gyro_z, 3),
        )

    def get_lidar_telemetry(self) -> LidarTelemetry:
        return LidarTelemetry(
            ranges=self.lidar_ranges,
            points=self.lidar_points,
            angle_min=-math.pi,
            angle_max=math.pi,
            angle_increment=(2.0 * math.pi / max(1, len(self.lidar_ranges))),
            range_min=0.12,
            range_max=12.0,
        )

    def get_odometry(self) -> OdometryTelemetry:
        return OdometryTelemetry(
            x=round(self.odom_x, 3),
            y=round(self.odom_y, 3),
            theta_rad=round(self.odom_theta, 3),
            vx=round(self.odom_vx, 3),
            vy=round(self.odom_vy, 3),
            wz=round(self.odom_wz, 3),
        )

    def get_config(self) -> RobotConfig:
        return self.config

    def update_config(self, config: RobotConfig) -> RobotConfig:
        self.config = config
        return self.config

    # Alias tiện ích
    get_wheels = get_wheel_telemetry
    get_imu = get_imu_telemetry
    get_lidar = get_lidar_telemetry

    # ROS 2 Callbacks
    def _on_scan(self, msg: LaserScan):
        ranges: List[float] = []
        points: List[List[float]] = []
        angle = msg.angle_min
        min_dist = float("inf")

        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                ranges.append(float(r))
                px = float(r * math.cos(angle))
                py = float(r * math.sin(angle))
                points.append([px, py])
                if r < min_dist:
                    min_dist = r
            else:
                ranges.append(0.0)
            angle += msg.angle_increment

        self.lidar_ranges = ranges
        self.lidar_points = points
        dist_str = f"min={min_dist:.2f}m" if min_dist != float("inf") else "min=N/A"
        stream_monitor.record("sensor_lidar", f"rays={len(ranges)}, valid={len(points)}, {dist_str}")

    def _on_odom(self, msg: Odometry):
        self.odom_x = float(msg.pose.pose.position.x)
        self.odom_y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.odom_theta = float(math.atan2(siny_cosp, cosy_cosp))
        self.odom_vx = float(msg.twist.twist.linear.x)
        self.odom_vy = float(msg.twist.twist.linear.y)
        self.odom_wz = float(msg.twist.twist.angular.z)
        stream_monitor.record("localization_odom", f"x={self.odom_x:.2f}m, y={self.odom_y:.2f}m, th={math.degrees(self.odom_theta):.1f}°")

    def _on_wheel_state(self, msg: Float32MultiArray):
        data = list(msg.data)
        # Format chuẩn từ STM32: 12 giá trị (4 measured, 4 target, 4 motor command)
        if len(data) >= 4:
            self.measured_wheel_speeds = [float(v) for v in data[0:4]]
        if len(data) >= 8:
            self.target_wheel_speeds = [float(v) for v in data[4:8]]
        if len(data) >= 12:
            self.motor_commands = [float(v) for v in data[8:12]]

        meas_summary = [round(v, 2) for v in self.measured_wheel_speeds]
        stream_monitor.record("stm32_wheel_state", f"speeds={meas_summary} rad/s")

    def _on_encoder_counts(self, msg: Int32MultiArray):
        if len(msg.data) >= 4:
            self.encoder_ticks = [int(v) for v in msg.data[:4]]
            stream_monitor.record("stm32_encoder_counts", f"ticks={self.encoder_ticks}")

    def _on_imu(self, msg: Imu):
        self.accel_x = float(msg.linear_acceleration.x)
        self.accel_y = float(msg.linear_acceleration.y)
        self.accel_z = float(msg.linear_acceleration.z)
        self.gyro_x = float(msg.angular_velocity.x)
        self.gyro_y = float(msg.angular_velocity.y)
        self.gyro_z = float(msg.angular_velocity.z)

        q = msg.orientation
        # Tính toán Roll, Pitch, Yaw từ Quaternion
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.roll_deg = float(math.degrees(math.atan2(sinr_cosp, cosr_cosp)))

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1.0:
            self.pitch_deg = float(math.degrees(math.copysign(math.pi / 2, sinp)))
        else:
            self.pitch_deg = float(math.degrees(math.asin(sinp)))

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw_deg = float(math.degrees(math.atan2(siny_cosp, cosy_cosp)))

        stream_monitor.record("stm32_imu", f"yaw={self.yaw_deg:.1f}°, gz={self.gyro_z:.2f} rad/s")

    def _on_status(self, msg: String):
        self.status_msg = str(msg.data)
        stream_monitor.record("stm32_status", f"status: {msg.data[:30]}")

    def _on_diagnostics(self, msg: DiagnosticArray):
        count = len(msg.status)
        first_msg = msg.status[0].message if count > 0 else "ok"
        stream_monitor.record("stm32_diagnostics", f"entries={count}, {first_msg}")

    def _on_safety_stop(self, msg: Bool):
        self.safety_stop = bool(msg.data)

    def _on_safe_cmd_vel(self, msg: Twist):
        stream_monitor.record("jetson_cmd_vel", f"safe: vx={msg.linear.x:.2f}, vy={msg.linear.y:.2f}, wz={msg.angular.z:.2f}")

    def _on_stm32_cmd_vel(self, msg: Twist):
        stream_monitor.record("jetson_cmd_vel", f"stm32: vx={msg.linear.x:.2f}, vy={msg.linear.y:.2f}, wz={msg.angular.z:.2f}")
