from __future__ import annotations

import asyncio
import logging
import math
import os
import queue
import threading
import time
try:
    import psutil
    HAS_PSUTIL = True
except ImportError:
    HAS_PSUTIL = False
    psutil = None

import json

from app.bridges.base import BaseRobotBridge
from app.models import (
    DebugTelemetry,
    ImuTelemetry,
    LidarTelemetry,
    OdometryTelemetry,
    PathTelemetry,
    RobotConfig,
    RobotStatus,
    WheelTelemetry,
)
from app.services.stream_monitor import stream_monitor
from app.services.grid_planner import GridMapPlanner, OmniMpcController

logger = logging.getLogger("amr_web.ros2_bridge")

try:
    import cv2
    import numpy as np
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

# Kiểm tra sự sẵn sàng của rclpy
try:
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import OccupancyGrid, Odometry, Path
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, Imu, LaserScan
    from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32MultiArray, String
    HAS_RCLPY = True
except ImportError as e:
    logger.debug(f"rclpy không thể import: {e}")
    HAS_RCLPY = False
    LaserScan = None
    Imu = None
    Odometry = None
    Twist = None
    PoseStamped = None
    OccupancyGrid = None
    Path = None
    Image = None
    Float32 = None
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
        self.config = RobotConfig()
        self.node: Optional[Node] = None
        self.executor: Optional[SingleThreadedExecutor] = None
        self.spin_thread: Optional[threading.Thread] = None
        self.running = False
        self.start_time = time.time()
        self._data_lock = threading.Lock()  # Protects shared sensor data from ROS thread vs AsyncIO
        self._planner_lock = threading.Lock()  # Protects grid_planner operations from concurrent threads
        self._has_filtered_odom = False
        self._has_scan_match = False
        self._scan_queue: queue.Queue = queue.Queue(maxsize=1)
        self._scan_thread: Optional[threading.Thread] = None

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
        self.imu_yaw_rad: Optional[float] = None
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 9.81
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 1.0
        self.debug_telemetry: Optional[DebugTelemetry] = None

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

        # Dữ liệu Camera, Map và Path (Phase 5 & 6)
        self.grid_planner = GridMapPlanner()
        self.mpc_controller = OmniMpcController()
        self.current_waypoint_index: int = 0
        self.global_path: List[List[float]] = []
        self.local_path: List[List[float]] = []
        self.map_data: Optional[dict] = None
        self.latest_camera_frame: Optional[bytes] = None
        self.latest_depth_frame: Optional[bytes] = None
        self.active_goal: Optional[dict] = None
        self.nav_state: str = "idle"
        self.autonomy_thread: Optional[threading.Thread] = None
        self.cmd_vel_pub = None
        self.safe_cmd_vel_pub = None
        self.stm32_cmd_vel_pub = None
        self.estop_pub = None
        self.goal_pub = None
        self.calib_cmd_pub = None
        self.config_cmd_pub = None
        self.latest_calib_data: dict = {}
        self.calib_status_data: dict = {}

    async def start(self) -> None:
        if self.running:
            return

        if not HAS_RCLPY:
            logger.warning("rclpy không khả dụng trong môi trường hiện tại.")
            self.connected = False
            return

        if not rclpy.ok():
            if "ROS_LOG_DIR" not in os.environ:
                log_dir = os.path.join(os.environ.get("AMR_WORKSPACE", os.getcwd()), ".temp_ros_log")
                os.makedirs(log_dir, exist_ok=True)
                os.environ["ROS_LOG_DIR"] = log_dir
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
        self.calib_cmd_pub = self.node.create_publisher(String, "calib/cmd", 10)
        self.config_cmd_pub = self.node.create_publisher(String, "config/cmd", 10)
        if PoseStamped is not None:
            self.goal_pub = self.node.create_publisher(PoseStamped, "goal_pose", 10)

        # Subscribers - Cảm biến môi trường & Gazebo (dùng SensorDataQoS BestEffort)
        self.node.create_subscription(LaserScan, "scan", self._on_scan, qos_profile_sensor_data)
        self.node.create_subscription(String, "calib/status", self._on_calib_status, 10)

        # Subscribers - Phản hồi từ STM32 & EKF
        self.node.create_subscription(Odometry, "wheel/odom", self._on_wheel_odom, 10)
        self.node.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.node.create_subscription(Odometry, "odometry/filtered", self._on_odom, 10)
        if Imu is not None:
            self.node.create_subscription(Imu, "imu/data", self._on_imu, qos_profile_sensor_data)
            self.node.create_subscription(Imu, "imu", self._on_imu, qos_profile_sensor_data)
        self.node.create_subscription(String, "status", self._on_status, 10)
        self.node.create_subscription(String, "debug/data", self._on_debug_data, 10)
        if Float32MultiArray is not None:
            self.node.create_subscription(Float32MultiArray, "wheel_state", self._on_wheel_state, 10)
        if Int32MultiArray is not None:
            self.node.create_subscription(Int32MultiArray, "encoder_counts", self._on_encoder_counts, 10)
        if DiagnosticArray is not None:
            self.node.create_subscription(DiagnosticArray, "diagnostics", self._on_diagnostics, 10)

        # Subscribers - Giám sát an toàn Jetson Watchdog & stm32_bridge
        self.node.create_subscription(Bool, "safety_stop", self._on_safety_stop, 10)
        self.node.create_subscription(Twist, "safe_cmd_vel", self._on_safe_cmd_vel, 10)
        self.node.create_subscription(Twist, "watched_cmd_vel", self._on_watched_cmd_vel, 10)
        self.node.create_subscription(Twist, "stm32_cmd_vel", self._on_stm32_cmd_vel, 10)
        if Float32 is not None:
            self.node.create_subscription(Float32, "safety_speed_factor", self._on_safety_speed_factor, 10)

        # Subscribers - Autonomy & Telemetry nâng cao (Phase 5)
        if OccupancyGrid is not None:
            self.node.create_subscription(OccupancyGrid, "map", self._on_map, 10)
        if Path is not None:
            self.node.create_subscription(Path, "plan", self._on_global_plan, 10)
            self.node.create_subscription(Path, "local_plan", self._on_local_plan, 10)
        if Image is not None:
            # Lắng nghe camera RGB từ cả mô phỏng Gazebo (/camera) và robot thực tế (/camera/image_raw)
            self.node.create_subscription(Image, "camera/image_raw", self._on_camera, qos_profile_sensor_data)
            self.node.create_subscription(Image, "camera", self._on_camera, qos_profile_sensor_data)
            # Lắng nghe camera chiều sâu (Depth camera)
            self.node.create_subscription(Image, "camera/depth/image_raw", self._on_depth_camera, qos_profile_sensor_data)
            self.node.create_subscription(Image, "depth", self._on_depth_camera, qos_profile_sensor_data)

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

        # Khởi động luồng xử lý quét bản đồ nền (Scan Worker Thread) để không làm block ROS executor
        self._scan_thread = threading.Thread(target=self._scan_worker, daemon=True)
        self._scan_thread.start()

        # Khởi động bộ điều khiển tự hành kín (Closed-loop Omni Goal Tracker)
        self.autonomy_thread = threading.Thread(target=self._autonomy_loop, daemon=True)
        self.autonomy_thread.start()
        logger.info("ROS 2 Bridge node 'amr_web_bridge' đã khởi động, sẵn sàng luồng Camera và Tự hành.")

    async def stop(self) -> None:
        self.running = False
        self.active_goal = None
        if self._scan_queue:
            try:
                self._scan_queue.put_nowait(None)
            except Exception:
                pass
        if self.executor:
            self.executor.shutdown()
        if self.node:
            self.node.destroy_node()
        if self.spin_thread and self.spin_thread.is_alive():
            self.spin_thread.join(timeout=1.0)
        if self.autonomy_thread and self.autonomy_thread.is_alive():
            self.autonomy_thread.join(timeout=0.5)
        if self._scan_thread and self._scan_thread.is_alive():
            self._scan_thread.join(timeout=0.5)
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
        stream_monitor.record("estop", f"active={active}")
        # Nếu estop kích hoạt, hủy ngay lập tức mục tiêu tự hành và gửi lệnh vận tốc 0
        if active:
            self.active_goal = None
            self.nav_state = "cancelled"
            self.global_path = []
            self.local_path = []
            if hasattr(self, "mpc_controller") and self.mpc_controller:
                self.mpc_controller.reset()
            zero_twist = Twist()
            for _ in range(3):
                self.cmd_vel_pub.publish(zero_twist)
                self.safe_cmd_vel_pub.publish(zero_twist)
                self.stm32_cmd_vel_pub.publish(zero_twist)

    def reset_odometry(self) -> None:
        with self._data_lock:
            self.odom_x = 0.0
            self.odom_y = 0.0
            self.odom_theta = 0.0
            self.imu_yaw_rad = 0.0
            self._has_scan_match = False
            self._has_filtered_odom = False
        with self._planner_lock:
            if hasattr(self, "grid_planner") and self.grid_planner:
                self.grid_planner.clear()
        logger.info("Reset odometry coordinates to (0, 0, 0)")

    def get_status(self) -> RobotStatus:
        now = time.time()
        # Nếu không nhận được lệnh trong 0.5s -> watchdog stop
        watchdog_triggered = (now - self.last_cmd_time) > self.config.safety_timeout_sec
        cpu = 0.0
        ram = 0.0
        try:
            cpu = round(psutil.cpu_percent(), 1)
            ram = round(psutil.virtual_memory().percent, 1)
        except Exception:
            pass
        return RobotStatus(
            mode="ros2",
            connected=self.running and (self.node is not None),
            estop_active=self.estop_active,
            safety_stop=self.safety_stop or watchdog_triggered,
            uptime_sec=round(now - self.start_time, 1),
            cpu_percent=cpu,
            ram_percent=ram,
            active_streams=stream_monitor.get_active_count(),
            timestamp_sec=now,
        )

    def get_wheel_telemetry(self) -> WheelTelemetry:
        with self._data_lock:
            target = [round(s, 2) for s in self.target_wheel_speeds]
            measured = [round(s, 2) for s in self.measured_wheel_speeds]
            ticks = list(self.encoder_ticks)
            pwms = [round(s, 2) for s in self.motor_commands]
        return WheelTelemetry(
            target_rad_s=target,
            measured_rad_s=measured,
            encoder_ticks=ticks,
            pwm_commands=pwms,
        )

    def get_imu_telemetry(self) -> ImuTelemetry:
        with self._data_lock:
            roll = self.roll_deg
            pitch = self.pitch_deg
            yaw = self.yaw_deg
            ax = self.accel_x
            ay = self.accel_y
            az = self.accel_z
            gx = self.gyro_x
            gy = self.gyro_y
            gz = self.gyro_z
            qx = getattr(self, "qx", 0.0)
            qy = getattr(self, "qy", 0.0)
            qz = getattr(self, "qz", 0.0)
            qw = getattr(self, "qw", 1.0)
        return ImuTelemetry(
            roll_deg=round(roll, 2),
            pitch_deg=round(pitch, 2),
            yaw_deg=round(yaw % 360.0, 2),
            accel_x=round(ax, 3),
            accel_y=round(ay, 3),
            accel_z=round(az, 3),
            gyro_x=round(gx, 3),
            gyro_y=round(gy, 3),
            gyro_z=round(gz, 3),
            qx=round(qx, 4),
            qy=round(qy, 4),
            qz=round(qz, 4),
            qw=round(qw, 4),
        )

    def get_debug_telemetry(self) -> Optional[DebugTelemetry]:
        if getattr(self, "debug_telemetry", None) is not None:
            return self.debug_telemetry
        with self._data_lock:
            meas_speeds = [round(s, 2) for s in self.measured_wheel_speeds]
            target_speeds = [round(s, 2) for s in self.target_wheel_speeds]
            motor_cmds = [round(s, 2) for s in self.motor_commands]
            qx = round(getattr(self, "qx", 0.0), 4)
            qy = round(getattr(self, "qy", 0.0), 4)
            qz = round(getattr(self, "qz", 0.0), 4)
            qw = round(getattr(self, "qw", 1.0), 4)
            vx = round(self.odom_vx, 3)
            vy = round(self.odom_vy, 3)
            wz = round(self.odom_wz, 3)
        return DebugTelemetry(
            raw_wheel_speed_rad_s=meas_speeds,
            filtered_wheel_speed_rad_s=meas_speeds,
            target_wheel_speed_rad_s=target_speeds,
            motor_output=motor_cmds,
            imu_quaternion_xyzw=[qx, qy, qz, qw],
            body_vx_mps=vx,
            body_vy_mps=vy,
            body_wz_rad_s=wz,
            cmd_vx_mps=0.0,
            cmd_vy_mps=0.0,
            cmd_wz_rad_s=0.0,
        )

    def get_lidar_telemetry(self) -> LidarTelemetry:
        with self._data_lock:
            ranges = list(self.lidar_ranges)
            points = list(self.lidar_points)
        return LidarTelemetry(
            ranges=ranges,
            points=points,
            angle_min=-math.pi,
            angle_max=math.pi,
            angle_increment=(2.0 * math.pi / max(1, len(ranges))),
            range_min=0.12,
            range_max=12.0,
        )

    def get_odometry(self) -> OdometryTelemetry:
        with self._data_lock:
            ox = self.odom_x
            oy = self.odom_y
            oth = self.odom_theta
            ovx = self.odom_vx
            ovy = self.odom_vy
            owz = self.odom_wz
        return OdometryTelemetry(
            x=round(ox, 3),
            y=round(oy, 3),
            theta_rad=round(oth, 3),
            vx=round(ovx, 3),
            vy=round(ovy, 3),
            wz=round(owz, 3),
        )

    def get_config(self) -> RobotConfig:
        return self.config

    def update_config(self, config: RobotConfig) -> RobotConfig:
        self.config = config
        if self.node and self.running and getattr(self, "config_cmd_pub", None):
            payload = {
                "motor_kp": config.motor_kp,
                "motor_ki": config.motor_ki,
                "motor_kd": config.motor_kd,
                "kalman_q": getattr(config, "kalman_q", [0.5, 0.5, 0.5, 0.5]),
                "kalman_r": getattr(config, "kalman_r", [0.04, 0.04, 0.04, 0.04]),
            }
            msg = String()
            msg.data = json.dumps(payload)
            self.config_cmd_pub.publish(msg)
            stream_monitor.record("config_cmd", f"kp={config.motor_kp}")
            logger.info(f"Published config to 'config/cmd': {payload}")
        return self.config

    # Alias tiện ích
    get_wheels = get_wheel_telemetry
    get_imu = get_imu_telemetry
    get_lidar = get_lidar_telemetry

    def _scan_worker(self):
        """Xử lý tích lũy bản đồ GridMapPlanner.add_scan trong background thread riêng để không block ROS executor."""
        while self.running:
            try:
                item = self._scan_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if item is None or not self.running:
                break

            x, y, theta, pts = item
            try:
                if hasattr(self, "grid_planner") and self.grid_planner and pts:
                    with self._planner_lock:
                        self.grid_planner.add_scan(x, y, theta, pts)
                        matched = getattr(self.grid_planner, "last_matched_pose", None)
                    if matched:
                        corr_x, corr_y, corr_th = matched
                        with self._data_lock:
                            self.odom_x = corr_x
                            self.odom_y = corr_y
                            if not getattr(self, "imu_received", False) and not getattr(self, "_has_filtered_odom", False):
                                self.odom_theta = corr_th
                                self.yaw_deg = float(math.degrees(corr_th))
                            self._has_scan_match = True
            except Exception as e:
                logger.debug(f"Lỗi add_scan worker: {e}")

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

        with self._data_lock:
            roll = abs(getattr(self, "roll_deg", 0.0))
            pitch = abs(getattr(self, "pitch_deg", 0.0))
            self.lidar_ranges = ranges
            self.lidar_points = points
            cur_x = self.odom_x
            cur_y = self.odom_y
            cur_th = self.odom_theta

        # Bỏ qua tích lũy bản đồ khi robot bị nghiêng pitch/roll > 2.0 độ (tránh tia laser quét trúng sàn nhà)
        is_tilted = (roll > 2.0 or pitch > 2.0)
        if hasattr(self, "grid_planner") and self.grid_planner and points and not is_tilted:
            try:
                self._scan_queue.put_nowait((cur_x, cur_y, cur_th, points))
            except queue.Full:
                pass

        dist_str = f"min={min_dist:.2f}m" if min_dist != float("inf") else "min=N/A"
        stream_monitor.record("sensor_lidar", f"rays={len(ranges)}, valid={len(points)}, {dist_str}")

    def _on_wheel_odom(self, msg: Odometry):
        now = time.time()
        last_wheel_time = getattr(self, "_last_wheel_odom_time", None)
        self._last_wheel_odom_time = now

        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        wz = float(msg.twist.twist.angular.z)
        pos_x = float(msg.pose.pose.position.x)
        pos_y = float(msg.pose.pose.position.y)

        # Trích xuất yaw từ quaternion bánh xe (dự phòng khi không có IMU)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        wheel_yaw = float(math.atan2(siny_cosp, cosy_cosp))

        with self._data_lock:
            self.wheel_vx = vx
            self.wheel_vy = vy
            self.wheel_wz = wz
            if not getattr(self, "_has_filtered_odom", False):
                self.odom_vx = self.wheel_vx
                self.odom_vy = self.wheel_vy
                self.odom_wz = self.wheel_wz
                if not getattr(self, "imu_received", False):
                    self.odom_theta = wheel_yaw
                    self.yaw_deg = float(math.degrees(wheel_yaw))
                if not getattr(self, "_has_scan_match", False):
                    self.odom_x = pos_x
                    self.odom_y = pos_y
                elif last_wheel_time is not None:
                    dt = now - last_wheel_time
                    if 0.0 < dt < 0.2:
                        cos_th = math.cos(self.odom_theta)
                        sin_th = math.sin(self.odom_theta)
                        self.odom_x += (cos_th * self.wheel_vx - sin_th * self.wheel_vy) * dt
                        self.odom_y += (sin_th * self.wheel_vx + cos_th * self.wheel_vy) * dt

        stream_monitor.record("stm32_wheel_odom", f"vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f}")

    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        th = float(math.atan2(siny_cosp, cosy_cosp))
        deg = math.degrees(th)
        px = float(msg.pose.pose.position.x)
        py = float(msg.pose.pose.position.y)
        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        wz = float(msg.twist.twist.angular.z)

        with self._data_lock:
            self._has_filtered_odom = True
            self.odom_x = px
            self.odom_y = py
            self.odom_theta = th
            self.yaw_deg = deg
            self.odom_vx = vx
            self.odom_vy = vy
            self.odom_wz = wz

        stream_monitor.record("localization_odom", f"x={px:.2f}m, y={py:.2f}m, th={deg:.1f}°")

    def _on_wheel_state(self, msg: Float32MultiArray):
        data = list(msg.data)
        # Format chuẩn từ STM32: 12 giá trị (4 measured, 4 target, 4 motor command)
        with self._data_lock:
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
            ticks = [int(v) for v in msg.data[:4]]
            with self._data_lock:
                self.encoder_ticks = ticks
            stream_monitor.record("stm32_encoder_counts", f"ticks={ticks}")

    def _on_imu(self, msg: Imu):
        ax = float(msg.linear_acceleration.x)
        ay = float(msg.linear_acceleration.y)
        az = float(msg.linear_acceleration.z)
        gx = float(msg.angular_velocity.x)
        gy = float(msg.angular_velocity.y)
        gz = float(msg.angular_velocity.z)

        q = msg.orientation
        qx = float(q.x)
        qy = float(q.y)
        qz = float(q.z)
        qw = float(q.w)
        # Tính toán Roll, Pitch, Yaw từ Quaternion
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll_deg = float(math.degrees(math.atan2(sinr_cosp, cosr_cosp)))

        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1.0:
            pitch_deg = float(math.degrees(math.copysign(math.pi / 2, sinp)))
        else:
            pitch_deg = float(math.degrees(math.asin(sinp)))

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        yaw_deg = float(math.degrees(yaw_rad))

        with self._data_lock:
            self.accel_x = ax
            self.accel_y = ay
            self.accel_z = az
            self.gyro_x = gx
            self.gyro_y = gy
            self.gyro_z = gz
            self.qx = qx
            self.qy = qy
            self.qz = qz
            self.qw = qw
            self.roll_deg = roll_deg
            self.pitch_deg = pitch_deg
            self.yaw_deg = yaw_deg
            self.imu_yaw_rad = float(yaw_rad)
            self.imu_received = True
            # Luôn cập nhật odom_theta từ IMU khi chưa có EKF (filtered odom) để heading không bị trôi/kẹt khi xoay
            if not getattr(self, "_has_filtered_odom", False):
                self.odom_theta = float(yaw_rad)

        stream_monitor.record("stm32_imu_data", f"yaw={yaw_deg:.1f}°, gz={gz:.2f} rad/s")

    def _on_debug_data(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.debug_telemetry = DebugTelemetry(**data)
            self.latest_calib_data.update(data)
            stream_monitor.record("stm32_debug_data", f"debug data received ({len(msg.data)} bytes)")
        except Exception as e:
            logger.debug(f"Error parsing debug/data: {e}")

    def _on_calib_status(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.calib_status_data = data
            self.latest_calib_data.update(data)
            stream_monitor.record("calib_status", f"status={data.get('status', 'ok')}")
        except Exception as e:
            logger.debug(f"Error parsing calib/status: {e}")

    def send_calib_cmd(self, payload: dict):
        if self.calib_cmd_pub and self.running:
            msg = String()
            msg.data = json.dumps(payload)
            self.calib_cmd_pub.publish(msg)
            cmd_type = payload.get("cmd") or payload.get("action", "unknown")
            stream_monitor.record("calib_cmd", f"cmd={cmd_type}")

    def _on_status(self, msg: String):
        self.status_msg = str(msg.data)
        stream_monitor.record("stm32_status", f"status: {msg.data[:30]}")

    def _on_diagnostics(self, msg: DiagnosticArray):
        count = len(msg.status)
        first_msg = msg.status[0].message if count > 0 else "ok"
        stream_monitor.record("stm32_diagnostics", f"entries={count}, {first_msg}")

    def _on_safety_stop(self, msg: Bool):
        self.safety_stop = bool(msg.data)
        stream_monitor.record("safety_stop", f"stop={msg.data}")

    def _on_safe_cmd_vel(self, msg: Twist):
        stream_monitor.record("jetson_cmd_vel", f"safe: vx={msg.linear.x:.2f}, vy={msg.linear.y:.2f}, wz={msg.angular.z:.2f}")

    def _on_watched_cmd_vel(self, msg: Twist):
        stream_monitor.record("watched_cmd_vel", f"watched: vx={msg.linear.x:.2f}, vy={msg.linear.y:.2f}, wz={msg.angular.z:.2f}")

    def _on_stm32_cmd_vel(self, msg: Twist):
        stream_monitor.record("stm32_cmd_vel", f"stm32: vx={msg.linear.x:.2f}, vy={msg.linear.y:.2f}, wz={msg.angular.z:.2f}")

    def _on_safety_speed_factor(self, msg: Float32):
        stream_monitor.record("safety_speed_factor", f"factor={msg.data:.2f}")

    def _on_map(self, msg: OccupancyGrid):
        try:
            self.map_data = {
                "resolution": float(msg.info.resolution),
                "width": int(msg.info.width),
                "height": int(msg.info.height),
                "origin_x": float(msg.info.origin.position.x),
                "origin_y": float(msg.info.origin.position.y),
                "data": list(msg.data),
            }
            stream_monitor.record("map", f"{msg.info.width}x{msg.info.height} @ {msg.info.resolution:.2f}m")
        except Exception as e:
            logger.debug(f"Lỗi parse map: {e}")

    def _on_global_plan(self, msg: Path):
        path = [[float(p.pose.position.x), float(p.pose.position.y)] for p in msg.poses]
        with self._data_lock:
            self.global_path = path
        stream_monitor.record("global_plan", f"{len(path)} waypoints")

    def _on_local_plan(self, msg: Path):
        path = [[float(p.pose.position.x), float(p.pose.position.y)] for p in msg.poses]
        with self._data_lock:
            self.local_path = path
        stream_monitor.record("local_plan", f"{len(path)} waypoints")

    def _on_camera(self, msg: Image):
        if not HAS_CV2:
            return
        try:
            height, width = msg.height, msg.width
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            if msg.encoding in ("rgb8", "bgr8") and len(arr) == height * width * 3:
                frame = arr.reshape((height, width, 3))
                if msg.encoding == "rgb8":
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                _, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
                self.latest_camera_frame = buf.tobytes()
            stream_monitor.record("camera_rgb", f"{width}x{height} {msg.encoding}")
        except Exception as e:
            logger.debug(f"Lỗi xử lý ảnh camera RGB: {e}")

    def _on_depth_camera(self, msg: Image):
        if not HAS_CV2:
            return
        try:
            height, width = msg.height, msg.width
            if msg.encoding == "32FC1":
                # Gazebo float32 depth in meters
                depth = np.frombuffer(msg.data, dtype=np.float32).reshape((height, width))
                depth = np.nan_to_num(depth, nan=5.0, posinf=5.0, neginf=0.0)
                norm = np.clip((depth / 4.0) * 255.0, 0, 255).astype(np.uint8)
                # Đảo để vật thể gần thì sáng/nóng
                norm = 255 - norm
                colored = cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)
                _, buf = cv2.imencode('.jpg', colored, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
                self.latest_depth_frame = buf.tobytes()
            elif msg.encoding == "16UC1":
                # 16-bit uint millimeter depth
                depth = np.frombuffer(msg.data, dtype=np.uint16).reshape((height, width))
                norm = np.clip((depth / 4000.0) * 255.0, 0, 255).astype(np.uint8)
                norm = 255 - norm
                colored = cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)
                _, buf = cv2.imencode('.jpg', colored, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
                self.latest_depth_frame = buf.tobytes()
            elif msg.encoding in ("rgb8", "bgr8") and len(msg.data) == height * width * 3:
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))
                _, buf = cv2.imencode('.jpg', arr, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
                self.latest_depth_frame = buf.tobytes()
            stream_monitor.record("camera_depth", f"{width}x{height} {msg.encoding}")
        except Exception as e:
            logger.debug(f"Lỗi xử lý ảnh camera Depth: {e}")

    def _publish_twist(self, vx: float, vy: float, wz: float):
        if not self.cmd_vel_pub or not self.running or self.estop_active:
            return
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self.cmd_vel_pub.publish(msg)

    def _compute_nav_velocities(
        self,
        tx: float,
        ty: float,
        gx: Optional[float] = None,
        gy: Optional[float] = None,
        dist_to_final: Optional[float] = None,
    ):
        """Tính toán vector vận tốc né vật cản LiDAR và ưu tiên xoay đầu xe theo hướng di chuyển."""
        with self._data_lock:
            cur_ox = self.odom_x
            cur_oy = self.odom_y
            cur_oth = self.odom_theta
            cur_pts = list(self.lidar_points)

        if gx is None:
            gx = tx
        if gy is None:
            gy = ty
        if dist_to_final is None:
            dist_to_final = math.hypot(gx - cur_ox, gy - cur_oy)

        # 1. VÙNG ĐỆM TIẾP CẬN ĐÍCH (Final Approach Deadband):
        # Khi cự ly tới đích cuối cùng < 0.25m:
        # TRIỆT TIÊU TOÀN BỘ VẬN TỐC GÓC wz = 0.0 để xe tuyệt đối không bị rung lắc / xoay vòng đảo chiều!
        if dist_to_final < 0.25:
            dx = gx - cur_ox
            dy = gy - cur_oy
            cos_th = math.cos(cur_oth)
            sin_th = math.sin(cur_oth)
            # Vector mục tiêu trong hệ robot frame
            rx = cos_th * dx + sin_th * dy
            ry = -sin_th * dx + cos_th * dy
            r_norm = math.hypot(rx, ry)
            crawl_speed = max(0.03, min(0.12, dist_to_final * 0.45))
            if r_norm > 1e-4:
                vr_x = crawl_speed * (rx / r_norm)
                vr_y = crawl_speed * (ry / r_norm)
            else:
                vr_x = 0.0
                vr_y = 0.0
            wz = 0.0
            local_path = [
                [round(cur_ox, 3), round(cur_oy, 3)],
                [round(gx, 3), round(gy, 3)],
            ]
            return vr_x, vr_y, wz, dist_to_final, local_path

        # 2. DI CHUYỂN BÌNH THƯỜNG THEO WAYPOINT TRUNG GIAN (tx, ty)
        dx = tx - cur_ox
        dy = ty - cur_oy
        dist = math.hypot(dx, dy)
        if dist < 1e-4:
            return 0.0, 0.0, 0.0, dist_to_final, []

        cos_th = math.cos(cur_oth)
        sin_th = math.sin(cur_oth)
        # Vector mục tiêu trong hệ robot frame
        u_att_x = dx / dist
        u_att_y = dy / dist
        u_att_rx = cos_th * u_att_x + sin_th * u_att_y
        u_att_ry = -sin_th * u_att_x + cos_th * u_att_y

        # Lực đẩy từ vật cản LiDAR (Repulsive Force trong robot frame)
        d_safe = 0.85
        f_rep_rx = 0.0
        f_rep_ry = 0.0
        front_obstacle_count = 0
        left_clearance = 0
        right_clearance = 0

        for pt in cur_pts:
            px, py = pt[0], pt[1]
            p_dist = math.hypot(px, py)
            if 0.05 < p_dist < d_safe:
                weight = ((d_safe - p_dist) / (d_safe * max(0.15, p_dist))) ** 2
                f_rep_rx += -weight * (px / p_dist)
                f_rep_ry += -weight * (py / p_dist)

                if px > 0.05 and abs(py) < 0.35:
                    front_obstacle_count += 1
                if py > 0:
                    left_clearance += 1
                else:
                    right_clearance += 1

        # Lực rẽ tiếp tuyến (Tangential Steering) khi có vật cản trước mặt
        f_tangent_ry = 0.0
        if front_obstacle_count > 0:
            steer_sign = 1.0 if (u_att_ry >= 0 and left_clearance <= right_clearance) else -1.0
            f_tangent_ry = steer_sign * min(2.5, 0.4 * front_obstacle_count)

        # Tổng hợp lực (Artificial Potential Field)
        k_rep = 0.35
        k_tan = 0.45
        f_total_rx = u_att_rx + k_rep * f_rep_rx
        f_total_ry = u_att_ry + k_rep * f_rep_ry + k_tan * f_tangent_ry

        total_norm = math.hypot(f_total_rx, f_total_ry)
        if total_norm > 1e-4:
            f_total_rx /= total_norm
            f_total_ry /= total_norm

        # Góc lệch giữa hướng di chuyển mong muốn và hướng hiện tại của đầu xe
        phi_move = math.atan2(f_total_ry, f_total_rx)

        # Tốc độ di chuyển tổng hợp (giảm tốc khi gần đích hoặc có vật cản)
        base_speed = min(0.35, max(0.08, dist_to_final * 0.7))
        if front_obstacle_count > 5:
            base_speed = max(0.08, base_speed * 0.6)

        # Ưu tiên xoay đầu xe theo hướng di chuyển (Turn-First Priority)
        wz = float(np.clip(2.0 * phi_move, -1.0, 1.0))
        align_scale = max(0.15, math.cos(phi_move) ** 2)
        total_speed = base_speed * align_scale

        # Phân bổ vận tốc: Xe ưu tiên chạy tiến theo trục dọc thân xe (vr_x)
        vr_x = total_speed * math.cos(phi_move)
        vr_y = total_speed * math.sin(phi_move) * 0.3  # Giảm trôi ngang để xe chạy về phía trước

        # Dừng khẩn cấp nếu vật cản quá sát trước mặt (< 22cm)
        for pt in cur_pts:
            if 0 < pt[0] < 0.22 and abs(pt[1]) < 0.18:
                vr_x = min(0.0, vr_x)

        # Tính toán các điểm lộ trình né vật cản (Local Path) để vẽ lên Web Canvas
        move_theta_world = cur_oth + phi_move
        step1_x = cur_ox + min(0.4, dist * 0.5) * math.cos(move_theta_world)
        step1_y = cur_oy + min(0.4, dist * 0.5) * math.sin(move_theta_world)
        local_path = [
            [round(cur_ox, 3), round(cur_oy, 3)],
            [round(step1_x, 3), round(step1_y, 3)],
            [round(tx, 3), round(ty, 3)],
        ]

        return vr_x, vr_y, wz, dist_to_final, local_path

    def _autonomy_loop(self):
        """Vòng lặp bám đích tự hành khép kín MPC né vật cản (Closed-loop Omni MPC Tracker) 15Hz."""
        loop_counter = 0
        while self.running:
            time.sleep(0.066)
            with self._data_lock:
                goal = dict(self.active_goal) if self.active_goal else None
                estop = self.estop_active
                ox = self.odom_x
                oy = self.odom_y
                oth = self.odom_theta
                lidar_pts = list(self.lidar_points)
                g_path = list(self.global_path)

            if not goal or estop:
                continue

            loop_counter += 1
            gx = goal["x"]
            gy = goal["y"]
            dist_to_final = math.hypot(gx - ox, gy - oy)

            # 1. Kiểm tra xem đã đến đích trong ngưỡng chính xác 8cm chưa
            if dist_to_final <= 0.08:
                for _ in range(3):
                    self._publish_twist(0.0, 0.0, 0.0)
                    time.sleep(0.02)
                with self._data_lock:
                    self.nav_state = "reached"
                    self.active_goal = None
                    self.local_path = []
                self.mpc_controller.reset()
                logger.info(f"Tự hành MPC: Đã đến điểm đích ({gx:.2f}, {gy:.2f}) thành công và dừng hẳn!")
                continue

            # 2. Liên tục tái lập kế hoạch động (Dynamic Replanning) mỗi ~0.26s (4 chu kỳ) để né vật cản mới xuất hiện
            if loop_counter % 4 == 0 and hasattr(self, "grid_planner") and self.grid_planner:
                with self._planner_lock:
                    new_path = self.grid_planner.plan_path(ox, oy, gx, gy)
                if new_path and len(new_path) >= 2:
                    with self._data_lock:
                        self.global_path = new_path
                    g_path = new_path

            # 3. Tính toán quỹ đạo tối ưu MPC né vật cản
            vr_x, vr_y, wz, dist, local_path = self.mpc_controller.compute(
                ox,
                oy,
                oth,
                g_path,
                gx,
                gy,
                lidar_pts,
            )

            with self._data_lock:
                self.nav_state = "navigating"
                self.local_path = local_path
            self._publish_twist(vr_x, vr_y, wz)

    def get_battery_telemetry(self) -> BatteryTelemetry:
        return self.battery

    def get_path_telemetry(self) -> PathTelemetry:
        with self._planner_lock:
            obstacles = (
                self.grid_planner.get_obstacle_points(max_points=800)
                if hasattr(self, "grid_planner") and self.grid_planner
                else []
            )
        with self._data_lock:
            g_path = list(self.global_path)
            l_path = list(self.local_path)
        return PathTelemetry(
            global_path=g_path,
            local_path=l_path,
            obstacles=obstacles,
        )

    def clear_map_memory(self):
        """Xóa bộ nhớ bản đồ vật cản và làm mới lộ trình."""
        with self._planner_lock:
            if hasattr(self, "grid_planner") and self.grid_planner:
                self.grid_planner.clear()
        with self._data_lock:
            self._has_scan_match = False
            self.global_path = []
            self.local_path = []
        logger.info("Bộ nhớ bản đồ vật cản và lộ trình đã được xóa trắng.")

    def get_map(self) -> Optional[dict]:
        return self.map_data

    async def send_nav_goal(self, x: float, y: float, theta: float = 0.0, frame_id: str = "map") -> bool:
        goal = {"x": float(x), "y": float(y), "theta": float(theta)}
        with self._data_lock:
            self.active_goal = goal
            self.nav_state = "navigating"
            self.current_waypoint_index = 0
            cur_ox = self.odom_x
            cur_oy = self.odom_y

        # Sinh quỹ đạo toàn cục A* né tường và vật cản đã lưu trong bộ nhớ
        g_path = []
        if hasattr(self, "grid_planner") and self.grid_planner:
            with self._planner_lock:
                g_path = self.grid_planner.plan_path(cur_ox, cur_oy, x, y)
        if not g_path:
            steps = 15
            g_path = [
                [
                    round(cur_ox + (x - cur_ox) * (i / steps), 3),
                    round(cur_oy + (y - cur_oy) * (i / steps), 3),
                ]
                for i in range(steps + 1)
            ]

        l_path = [
            [round(cur_ox, 3), round(cur_oy, 3)],
            [round(x, 3), round(y, 3)],
        ]

        with self._data_lock:
            self.global_path = g_path
            self.local_path = l_path

        # Phát hành tới Nav2 /goal_pose nếu stack Nav2 đang chạy
        if self.running and self.node and self.goal_pub:
            msg = PoseStamped()
            msg.header.frame_id = frame_id
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.pose.position.x = float(x)
            msg.pose.position.y = float(y)
            msg.pose.position.z = 0.0
            msg.pose.orientation.z = math.sin(theta * 0.5)
            msg.pose.orientation.w = math.cos(theta * 0.5)
            self.goal_pub.publish(msg)
        return True

    async def cancel_nav_goal(self) -> bool:
        with self._data_lock:
            self.active_goal = None
            self.nav_state = "cancelled"
            self.global_path = []
            self.local_path = []
        if hasattr(self, "mpc_controller") and self.mpc_controller:
            self.mpc_controller.reset()
        # Gửi chuỗi xung vận tốc 0 đa cổng để phanh khẩn cấp lập tức
        for _ in range(5):
            self._publish_twist(0.0, 0.0, 0.0)
            if self.node and self.cmd_vel_pub:
                zero_twist = Twist()
                self.cmd_vel_pub.publish(zero_twist)
                if hasattr(self, "safe_cmd_vel_pub") and self.safe_cmd_vel_pub:
                    self.safe_cmd_vel_pub.publish(zero_twist)
                if hasattr(self, "stm32_cmd_vel_pub") and self.stm32_cmd_vel_pub:
                    self.stm32_cmd_vel_pub.publish(zero_twist)
            await asyncio.sleep(0.01)
        return True

    def get_nav_status(self) -> dict:
        with self._data_lock:
            goal = dict(self.active_goal) if self.active_goal else None
            state = self.nav_state
            ox = self.odom_x
            oy = self.odom_y
        dist = 0.0
        if goal:
            dist = math.hypot(goal["x"] - ox, goal["y"] - oy)
        return {
            "state": state,
            "goal": goal,
            "distance_remaining_m": round(dist, 2),
        }

    def get_latest_camera_frame(self, camera_type: str = "rgb") -> Optional[bytes]:
        if camera_type == "depth":
            if self.latest_depth_frame:
                return self.latest_depth_frame
            return self._render_depth_fallback_hud()
        else:
            if self.latest_camera_frame:
                return self.latest_camera_frame
            return self._render_rgb_fallback_hud()

    def _render_rgb_fallback_hud(self) -> Optional[bytes]:
        if not HAS_CV2:
            return None
        with self._data_lock:
            ox = self.odom_x
            oy = self.odom_y
            yaw = self.yaw_deg
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        img[:] = (20, 24, 33)
        # Tâm ngắm
        cv2.line(img, (320, 220), (320, 260), (0, 230, 180), 1)
        cv2.line(img, (300, 240), (340, 240), (0, 230, 180), 1)
        cv2.circle(img, (320, 240), 16, (0, 230, 180), 1)
        # Thông số HUD
        cv2.putText(img, "AMR OMNI CAM 01 [RGB LIVE]", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 230, 180), 2)
        cv2.putText(img, f"POS: ({ox:.2f}m, {oy:.2f}m) YAW: {yaw:.1f} deg",
                    (20, 440), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.putText(img, f"BATT: {self.battery.voltage_v:.1f}V ({self.battery.soc_percent:.0f}%)",
                    (420, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 120), 1)
        _, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
        return buf.tobytes()

    def _render_depth_fallback_hud(self) -> Optional[bytes]:
        if not HAS_CV2:
            return None
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        # Gradient mô phỏng Turbo Colormap cho Depth
        for y in range(480):
            val = int((y / 480.0) * 255)
            img[y, :] = (val, 128, 255 - val)
        img = cv2.applyColorMap(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), cv2.COLORMAP_TURBO)
        cv2.putText(img, "AMR OMNI DEPTH CAM [TURBO COLORMAP]", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(img, f"RANGE: 0.1m - 5.0m | SENSOR: ACTIVE",
                    (20, 440), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        _, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
        return buf.tobytes()


