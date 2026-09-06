from typing import List, Optional
from pydantic import BaseModel, Field


class TwistCommand(BaseModel):
    linear_x: float = Field(0.0, description="Vận tốc tiến/lùi (m/s)")
    linear_y: float = Field(0.0, description="Vận tốc di chuyển ngang strafe (m/s)")
    angular_z: float = Field(0.0, description="Vận tốc quay góc yaw (rad/s)")


class EStopCommand(BaseModel):
    active: bool = Field(..., description="Trạng thái dừng khẩn cấp E-Stop")


class RobotStatus(BaseModel):
    mode: str = Field("mock", description="Chế độ hoạt động: simulation, hardware, mock")
    connected: bool = Field(False, description="Trạng thái kết nối ROS / Bridge")
    estop_active: bool = Field(False, description="Cờ E-Stop đang kích hoạt")
    safety_stop: bool = Field(False, description="Cờ dừng an toàn từ watchdog")
    uptime_sec: float = Field(0.0, description="Thời gian hoạt động (giây)")
    cpu_percent: float = Field(0.0, description="Phần trăm sử dụng CPU")
    ram_percent: float = Field(0.0, description="Phần trăm sử dụng RAM")
    active_streams: int = Field(0, description="Số luồng dữ liệu đang hoạt động")
    timestamp_sec: float = Field(0.0, description="Thời gian epoch")


class WheelTelemetry(BaseModel):
    names: List[str] = Field(
        default_factory=lambda: [
            "omni_wheel_1 (FR)",
            "omni_wheel_2 (FL)",
            "omni_wheel_3 (RL)",
            "omni_wheel_4 (RR)",
        ]
    )
    target_rad_s: List[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    measured_rad_s: List[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    encoder_ticks: List[int] = Field(default_factory=lambda: [0, 0, 0, 0])
    pwm_commands: List[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])


class ImuTelemetry(BaseModel):
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 9.81
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0


class DebugTelemetry(BaseModel):
    raw_wheel_speed_rad_s: List[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    filtered_wheel_speed_rad_s: List[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    target_wheel_speed_rad_s: List[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    motor_output: List[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    imu_quaternion_xyzw: List[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0, 1.0])
    body_vx_mps: float = 0.0
    body_vy_mps: float = 0.0
    body_wz_rad_s: float = 0.0
    cmd_vx_mps: float = 0.0
    cmd_vy_mps: float = 0.0
    cmd_wz_rad_s: float = 0.0


class LidarTelemetry(BaseModel):
    angle_min: float = -3.14159265
    angle_max: float = 3.14159265
    angle_increment: float = 0.01745329
    range_min: float = 0.12
    range_max: float = 12.0
    ranges: List[float] = Field(default_factory=list)
    # Tọa độ 2D Cartesian [x, y] tính sẵn để frontend vẽ Canvas cực nhanh
    points: List[List[float]] = Field(default_factory=list)


class OdometryTelemetry(BaseModel):
    x: float = 0.0
    y: float = 0.0
    theta_rad: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


class StreamInfo(BaseModel):
    id: str
    name: str
    source: str
    destination: str
    topic: str
    message_type: str
    target_frequency_hz: float
    actual_frequency_hz: float = 0.0
    packet_count: int = 0
    last_timestamp_sec: float = 0.0
    latency_ms: float = 0.0
    status: str = "offline"  # "active", "degraded", "stale", "offline"
    payload_preview: Optional[str] = None


class RobotConfig(BaseModel):
    wheel_radius_m: float = 0.03
    wheelbase_m: float = 0.1312
    track_width_m: float = 0.1312
    max_wheel_speed_rad_s: float = 18.0
    max_linear_speed_mps: float = 0.54
    max_angular_speed_rad_s: float = 3.0
    command_timeout_sec: float = 0.25
    safety_timeout_sec: float = 0.5
    control_frequency_hz: float = 100.0
    telemetry_frequency_hz: float = 50.0
    motor_kp: float = 0.08
    motor_ki: float = 0.25
    motor_kd: float = 0.0005
    debug_telemetry: bool = True


class NavGoalRequest(BaseModel):
    x: float = Field(..., description="Tọa độ mục tiêu X (m)")
    y: float = Field(..., description="Tọa độ mục tiêu Y (m)")
    theta_rad: float = Field(0.0, description="Góc quay mục tiêu (rad)")
    frame_id: str = Field("map", description="Frame ID tọa độ (map/odom)")


class Waypoint(BaseModel):
    x: float
    y: float
    theta_rad: float = 0.0


class WaypointsRequest(BaseModel):
    waypoints: List[Waypoint]
    loop: bool = False


class MapMetadata(BaseModel):
    resolution: float = 0.05
    width: int = 0
    height: int = 0
    origin_x: float = 0.0
    origin_y: float = 0.0


class PathTelemetry(BaseModel):
    global_path: List[List[float]] = Field(default_factory=list)
    local_path: List[List[float]] = Field(default_factory=list)
    obstacles: List[List[float]] = Field(default_factory=list)


class SaveMapRequest(BaseModel):
    map_name: str = Field("map", description="Tên file bản đồ cần lưu")

