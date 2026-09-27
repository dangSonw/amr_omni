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
        stream_type: str = "stream",  # "stream", "command", "autonomy"
    ):
        self.stream_id = stream_id
        self.name = name
        self.source = source
        self.destination = destination
        self.topic = topic
        self.message_type = message_type
        self.target_frequency_hz = target_frequency_hz
        self.stream_type = stream_type

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
        # Cửa sổ tính toán thích ứng theo tần số đích (tối thiểu 3.0s để không báo chậm ảo đối với stream 1Hz-2Hz)
        window = max(3.0, 4.0 / max(0.1, self.target_frequency_hz))
        recent = [t for t in self.packet_times if now - t <= window]
        if len(recent) < 2:
            return 0.0
        span = recent[-1] - recent[0]
        return (len(recent) - 1) / span if span > 0.05 else 0.0

    def get_status(self, now: float) -> str:
        if self.last_timestamp_sec == 0:
            if self.stream_type in ("command", "autonomy"):
                return "standby"
            return "offline"

        age = now - self.last_timestamp_sec

        # Với lệnh điều khiển và tự hành: khi không có lệnh phát ra thì chuyển sang standby thay vì báo slow/lỗi
        if self.stream_type in ("command", "autonomy"):
            if age > 2.0:
                return "standby"
            return "active"

        # Với stream liên tục (cảm biến, odom, camera, map)
        if self.stream_type == "stream":
            if self.target_frequency_hz <= 2.0:
                timeout = max(5.0, 3.0 / self.target_frequency_hz)
                if age > timeout:
                    return "offline"
                if age > timeout * 0.5:
                    return "stale"
            else:
                if age > 3.0:
                    return "offline"
                if age > 1.5:
                    return "stale"

        # Đánh giá tần số thực tế (tránh cảnh báo chậm sai lệch trong môi trường WSL2/mô phỏng)
        actual_hz = self.get_actual_frequency_hz(now)
        if self.target_frequency_hz <= 2.0:
            threshold = max(0.1, self.target_frequency_hz * 0.10)
        elif "camera" in self.stream_id:
            threshold = 0.5  # Camera trong WSL2 mô phỏng đạt >= 0.5Hz là mượt mà
        else:
            # Mô phỏng Gazebo trong WSL2 dùng kms_swrast có RTF ~0.08, nên stream 50Hz đạt ~3.5-4.0Hz tường
            threshold = max(0.5, min(2.0, self.target_frequency_hz * 0.05))

        if actual_hz < threshold:
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
            # --- Nhóm 1: Điều Khiển & Giao Tiếp STM32 (9 topics) ---
            (
                "navigation_cmd_vel",
                "Lệnh Vận Tốc Điều Khiển (Web/Nav2 → Watchdog)",
                "Web Cockpit / Nav2",
                "Jetson (Watchdog)",
                "cmd_vel",
                "geometry_msgs/Twist",
                20.0,
                "command",
            ),
            (
                "watched_cmd_vel",
                "Lệnh Đã Qua Watchdog (Watchdog → Safety Zone)",
                "Jetson (Watchdog)",
                "Jetson (Safety Zone)",
                "watched_cmd_vel",
                "geometry_msgs/Twist",
                50.0,
                "command",
            ),
            (
                "jetson_cmd_vel",
                "Lệnh Vận Tốc An Toàn (Safety Zone → Bridge)",
                "Jetson (Safety Zone)",
                "Jetson (Bridge)",
                "safe_cmd_vel",
                "geometry_msgs/Twist",
                50.0,
                "command",
            ),
            (
                "stm32_cmd_vel",
                "Lệnh Vận Tốc Chấp Hành (Bridge → STM32)",
                "Jetson (Bridge)",
                "STM32 MCU",
                "stm32_cmd_vel",
                "geometry_msgs/Twist",
                50.0,
                "command",
            ),
            (
                "stm32_wheel_odom",
                "Odometry Vận Tốc Bánh Xe (STM32 → Jetson)",
                "STM32 MCU (Encoders)",
                "Jetson (EKF)",
                "wheel/odom",
                "nav_msgs/Odometry",
                50.0,
                "stream",
            ),
            (
                "stm32_imu_data",
                "Dữ Liệu IMU Quaternion 9-DoF (BNO080 → Jetson)",
                "BNO080 / STM32",
                "Jetson (EKF)",
                "imu/data",
                "sensor_msgs/Imu",
                50.0,
                "stream",
            ),
            (
                "stm32_debug_data",
                "Dữ Liệu Telemetry Gỡ Lỗi (STM32 → Web)",
                "STM32 MCU",
                "Web Dashboard",
                "debug/data",
                "std_msgs/String",
                50.0,
                "stream",
            ),
            (
                "config_cmd",
                "Cài Đặt Cấu Hình 4xPID & 4xKalman (Web → STM32)",
                "Web Dashboard",
                "STM32 MCU",
                "config/cmd",
                "std_msgs/String",
                1.0,
                "command",
            ),
            (
                "estop",
                "Dừng Khẩn Cấp E-Stop (Web/Watchdog → STM32)",
                "Web / Watchdog",
                "STM32 MCU",
                "estop",
                "std_msgs/Bool",
                10.0,
                "command",
            ),
            # --- Nhóm 2: An Toàn, Cảm Biến, Định Vị & Tự Hành (9 topics) ---
            (
                "safety_stop",
                "Cảnh Báo Dừng An Toàn LiDAR (Safety Zone → Hệ Thống)",
                "Jetson (Safety Zone)",
                "Web Dashboard",
                "safety_stop",
                "std_msgs/Bool",
                20.0,
                "stream",
            ),
            (
                "safety_speed_factor",
                "Hệ Số Giảm Tốc Vật Cản (Safety Zone → Hệ Thống)",
                "Jetson (Safety Zone)",
                "Web Dashboard",
                "safety_speed_factor",
                "std_msgs/Float32",
                20.0,
                "stream",
            ),
            (
                "sensor_lidar",
                "Dữ Liệu Quét 2D LiDAR (LiDAR → Safety/Nav2)",
                "LiDAR 2D Sensor",
                "Jetson (Safety/Nav2)",
                "scan",
                "sensor_msgs/LaserScan",
                10.0,
                "stream",
            ),
            (
                "localization_odom",
                "Ước Lượng Tư Thế EKF (EKF → Nav2/Web)",
                "Jetson (robot_loc)",
                "Nav2 & Web",
                "odometry/filtered",
                "nav_msgs/Odometry",
                50.0,
                "stream",
            ),
            (
                "map",
                "Bản Đồ Lưới Tọa Độ (SLAM / GridMap → Nav2/Web)",
                "Jetson (SLAM/GridMap)",
                "Nav2 & Web",
                "map",
                "nav_msgs/OccupancyGrid",
                1.0,
                "stream",
            ),
            (
                "global_plan",
                "Đường Đi Toàn Cục (Nav2/A* → Web)",
                "Jetson (Nav2/A*)",
                "Web Dashboard",
                "plan",
                "nav_msgs/Path",
                2.0,
                "autonomy",
            ),
            (
                "local_plan",
                "Quỹ Đạo Điều Khiển Cục Bộ (Nav2/MPC → Web)",
                "Jetson (Nav2/MPC)",
                "Web Dashboard",
                "local_plan",
                "nav_msgs/Path",
                10.0,
                "autonomy",
            ),
            (
                "camera_rgb",
                "Luồng Video RGB Camera (Camera → Web)",
                "Camera RGB Sensor",
                "Web Dashboard",
                "camera/image_raw",
                "sensor_msgs/Image",
                15.0,
                "stream",
            ),
            (
                "camera_depth",
                "Luồng Ảnh Đo Độ Sâu (Depth Camera → Web)",
                "Camera Depth Sensor",
                "Web Dashboard",
                "camera/depth/image_raw",
                "sensor_msgs/Image",
                15.0,
                "stream",
            ),
        ]
        for sid, name, src, dst, topic, msg_type, target_hz, stream_type in defaults:
            self.streams[sid] = StreamTracker(
                stream_id=sid,
                name=name,
                source=src,
                destination=dst,
                topic=topic,
                message_type=msg_type,
                target_frequency_hz=target_hz,
                stream_type=stream_type,
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
                stream_type="stream",
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
        return sum(1 for tracker in self.streams.values() if tracker.get_status(now) in ("active", "standby"))


stream_monitor = StreamMonitor()


