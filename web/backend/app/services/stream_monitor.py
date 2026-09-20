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
            (
                "jetson_cmd_vel",
                "Lệnh Vận Tốc Thân Xe (Jetson → STM32)",
                "Jetson Nano (Watchdog)",
                "STM32 MCU",
                "safe_cmd_vel",
                "geometry_msgs/Twist",
                50.0,
            ),
            (
                "stm32_wheel_odom",
                "Odometry Vận Tốc Bánh Xe (STM32 → Jetson)",
                "STM32 MCU (Encoders + FK)",
                "Jetson Nano (EKF)",
                "wheel/odom",
                "nav_msgs/Odometry",
                50.0,
            ),
            (
                "stm32_imu_data",
                "Dữ Liệu Quaternion & Vận Tốc Góc (BNO080 → Jetson)",
                "BNO080 / STM32",
                "Jetson Nano (EKF)",
                "imu/data",
                "sensor_msgs/Imu",
                50.0,
            ),
            (
                "stm32_status",
                "Trạng Thái Sức Khỏe MCU (STM32 → Jetson)",
                "STM32 MCU",
                "Jetson Nano",
                "status",
                "std_msgs/String",
                10.0,
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

