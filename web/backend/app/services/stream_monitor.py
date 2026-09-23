import time
from collections import deque
from typing import Dict, List, Optional
from app.models import StreamInfo


class StreamTracker:
    def __init__(
        self,
        stream_id: str,
        name: str,
        source: str,
        destination: str,
        topic: str,
        message_type: str,
        target_frequency_hz: float,
    ):
        self.stream_id = stream_id
        self.name = name
        self.source = source
        self.destination = destination
        self.topic = topic
        self.message_type = message_type
        self.target_frequency_hz = target_frequency_hz

        self.packet_count = 0
        self.last_timestamp_sec = 0.0
        self.packet_times: deque = deque(maxlen=60)
        self.payload_preview: Optional[str] = None

    def record_packet(self, payload_preview: Optional[str] = None):
        now = time.time()
        self.packet_count += 1
        self.last_timestamp_sec = now
        self.packet_times.append(now)
        if payload_preview:
            self.payload_preview = payload_preview

    def get_actual_frequency_hz(self, now: float) -> float:
        if len(self.packet_times) < 2:
            return 0.0
        # Tính tần số trong cửa sổ 1 giây gần nhất
        recent = [t for t in self.packet_times if now - t <= 1.0]
        if len(recent) < 2:
            # Fallback tính theo khoảng thời gian giữa gói đầu và cuối
            span = self.packet_times[-1] - self.packet_times[0]
            return len(self.packet_times) / span if span > 0.05 else 0.0
        span = recent[-1] - recent[0]
        return (len(recent) - 1) / span if span > 0.05 else 0.0

    def get_status(self, now: float) -> str:
        if self.last_timestamp_sec == 0:
            return "offline"
        age = now - self.last_timestamp_sec
        if age > 3.0:
            return "offline"
        if age > 1.0:
            return "stale"
        actual_hz = self.get_actual_frequency_hz(now)
        if actual_hz < (self.target_frequency_hz * 0.4):
            return "degraded"
        return "active"

    def to_info(self, now: float) -> StreamInfo:
        age_ms = (now - self.last_timestamp_sec) * 1000.0 if self.last_timestamp_sec > 0 else 9999.0
        actual_hz = self.get_actual_frequency_hz(now)
        status = self.get_status(now)
        return StreamInfo(
            id=self.stream_id,
            name=self.name,
            source=self.source,
            destination=self.destination,
            topic=self.topic,
            message_type=self.message_type,
            target_frequency_hz=self.target_frequency_hz,
            actual_frequency_hz=round(actual_hz, 1),
            packet_count=self.packet_count,
            last_timestamp_sec=round(self.last_timestamp_sec, 3),
            latency_ms=round(age_ms, 1),
            status=status,
            payload_preview=self.payload_preview,
        )


class StreamMonitor:
    def __init__(self):
        self.streams: Dict[str, StreamTracker] = {}
        self._init_default_streams()

    def _init_default_streams(self):
        defaults = [
            # --- Nhóm 1: Giao Tiếp STM32 ↔ Jetson & Web ---
            (
                "watched_cmd_vel",
                "Lệnh Đã Qua Watchdog (Watchdog → Safety Zone)",
                "Jetson (Watchdog)",
                "Jetson (Safety Zone)",
                "watched_cmd_vel",
                "geometry_msgs/Twist",
                50.0,
            ),
            (
                "jetson_cmd_vel",
                "Lệnh Vận Tốc An Toàn (Safety Zone → STM32)",
                "Jetson (Safety Zone)",
                "STM32 MCU",
                "safe_cmd_vel",
                "geometry_msgs/Twist",
                50.0,
            ),
            (
                "stm32_cmd_vel",
                "Lệnh Vận Tốc Chấp Hành (Bridge → STM32)",
                "Jetson (Bridge)",
                "STM32 MCU",
                "stm32_cmd_vel",
                "geometry_msgs/Twist",
                50.0,
            ),
            (
                "stm32_wheel_odom",
                "Odometry Bánh Xe (STM32 → Jetson)",
                "STM32 MCU (Encoders)",
                "Jetson Nano (EKF)",
                "wheel/odom",
                "nav_msgs/Odometry",
                50.0,
            ),
            (
                "stm32_imu_data",
                "Dữ Liệu IMU Quaternion 9-DoF (BNO080 → Jetson)",
                "BNO080 / STM32",
                "Jetson Nano (EKF)",
                "imu/data",
                "sensor_msgs/Imu",
                50.0,
            ),
            (
                "stm32_debug_data",
                "Dữ Liệu Telemetry Gỡ Lỗi (STM32 → Web)",
                "STM32 MCU",
                "Web Dashboard",
                "debug/data",
                "std_msgs/String",
                50.0,
            ),
            (
                "stm32_status",
                "Trạng Thái Sức Khỏe MCU (STM32 → Jetson/Web)",
                "STM32 MCU",
                "Jetson & Web",
                "status",
                "std_msgs/String",
                10.0,
            ),
            (
                "stm32_diagnostics",
                "Chẩn Đoán Phần Cứng STM32 (STM32 → Jetson)",
                "STM32 MCU",
                "Jetson Nano",
                "diagnostics",
                "diagnostic_msgs/DiagnosticArray",
                10.0,
            ),
            (
                "stm32_wheel_state",
                "Chi Tiết Vận Tốc 4 Bánh (STM32 → Jetson)",
                "STM32 MCU",
                "Jetson Nano",
                "wheel_state",
                "std_msgs/Float32MultiArray",
                50.0,
            ),
            (
                "stm32_encoder_counts",
                "Số Ticks Encoder 4 Bánh (STM32 → Jetson)",
                "STM32 MCU",
                "Jetson Nano",
                "encoder_counts",
                "std_msgs/Int32MultiArray",
                50.0,
            ),
            (
                "config_cmd",
                "Cài Đặt Cấu Hình 4xPID & 4xKalman (Web → STM32)",
                "Web Dashboard",
                "STM32 MCU",
                "config/cmd",
                "std_msgs/String",
                1.0,
            ),
            (
                "estop",
                "Dừng Khẩn Cấp E-Stop (Web/Watchdog → STM32)",
                "Web / Watchdog",
                "STM32 MCU",
                "estop",
                "std_msgs/Bool",
                10.0,
            ),
            (
                "hardware_status",
                "Trạng Thái Cầu Nối Phần Cứng (Bridge → Hệ Thống)",
                "Jetson (Bridge)",
                "Jetson System",
                "hardware_status",
                "std_msgs/String",
                10.0,
            ),
            # --- Nhóm 2: Điều Khiển, Định Vị & Cảm Biến Môi Trường ---
            (
                "navigation_cmd_vel",
                "Lệnh Vận Tốc Điều Khiển (Web/Nav2 → Watchdog)",
                "Web Cockpit / Nav2",
                "Jetson (Watchdog)",
                "cmd_vel",
                "geometry_msgs/Twist",
                20.0,
            ),
            (
                "safety_stop",
                "Cảnh Báo Dừng An Toàn LiDAR (Safety Zone → Hệ Thống)",
                "Jetson (Safety Zone)",
                "Web Dashboard",
                "safety_stop",
                "std_msgs/Bool",
                20.0,
            ),
            (
                "safety_speed_factor",
                "Hệ Số Giảm Tốc Vật Cản (Safety Zone → Hệ Thống)",
                "Jetson (Safety Zone)",
                "Web Dashboard",
                "safety_speed_factor",
                "std_msgs/Float32",
                20.0,
            ),
            (
                "sensor_lidar",
                "Dữ Liệu Quét 2D LiDAR (LiDAR → Safety/Nav2)",
                "LiDAR 2D Sensor",
                "Jetson (Safety/Nav2)",
                "scan",
                "sensor_msgs/LaserScan",
                10.0,
            ),
            (
                "localization_odom",
                "Ước Lượng Tư Thế EKF (EKF → Nav2/Web)",
                "Jetson (robot_loc)",
                "Nav2 & Web",
                "odom",
                "nav_msgs/Odometry",
                50.0,
            ),
            (
                "map",
                "Bản Đồ Lưới Tọa Độ (SLAM → Nav2/Web)",
                "Jetson (SLAM Toolbox)",
                "Nav2 & Web",
                "map",
                "nav_msgs/OccupancyGrid",
                1.0,
            ),
            (
                "global_plan",
                "Đường Đi Toàn Cục (Nav2 → Web)",
                "Jetson (Nav2 Planner)",
                "Web Dashboard",
                "plan",
                "nav_msgs/Path",
                2.0,
            ),
            (
                "local_plan",
                "Quỹ Đạo Điều Khiển Cục Bộ (Nav2 → Web)",
                "Jetson (Nav2 Controller)",
                "Web Dashboard",
                "local_plan",
                "nav_msgs/Path",
                10.0,
            ),
            (
                "camera_rgb",
                "Luồng Video RGB Camera (Camera → Web)",
                "Camera RGB Sensor",
                "Web Dashboard",
                "camera/image_raw",
                "sensor_msgs/Image",
                15.0,
            ),
        ]
        for sid, name, src, dst, topic, msg_type, target_hz in defaults:
            self.streams[sid] = StreamTracker(
                stream_id=sid,
                name=name,
                source=src,
                destination=dst,
                topic=topic,
                message_type=msg_type,
                target_frequency_hz=target_hz,
            )

    def record(self, stream_id: str, payload_preview: Optional[str] = None):
        if stream_id in self.streams:
            self.streams[stream_id].record_packet(payload_preview)
        else:
            self.streams[stream_id] = StreamTracker(
                stream_id=stream_id,
                name=stream_id,
                source="System",
                destination="Jetson / Web",
                topic=stream_id,
                message_type="std_msgs/String",
                target_frequency_hz=10.0,
            )
            self.streams[stream_id].record_packet(payload_preview)

    def get_all(self) -> List[StreamInfo]:
        now = time.time()
        return [tracker.to_info(now) for tracker in self.streams.values()]

    def get(self, stream_id: str) -> Optional[StreamInfo]:
        now = time.time()
        tracker = self.streams.get(stream_id)
        return tracker.to_info(now) if tracker else None

    def get_active_count(self) -> int:
        now = time.time()
        return sum(1 for tracker in self.streams.values() if tracker.get_status(now) == "active")


stream_monitor = StreamMonitor()

