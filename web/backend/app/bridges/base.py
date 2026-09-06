from abc import ABC, abstractmethod
from typing import Optional
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


class BaseRobotBridge(ABC):
    @abstractmethod
    async def start(self) -> None:
        """Khởi động bridge, kết nối topic hoặc giả lập."""
        pass

    @abstractmethod
    async def stop(self) -> None:
        """Dừng bridge và dọn dẹp tài nguyên."""
        pass

    @abstractmethod
    async def send_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        """Gửi lệnh vận tốc tới robot."""
        pass

    @abstractmethod
    async def send_estop(self, active: bool) -> None:
        """Bật/tắt trạng thái dừng khẩn cấp."""
        pass

    @abstractmethod
    def get_status(self) -> RobotStatus:
        """Lấy trạng thái tổng quan của robot."""
        pass

    @abstractmethod
    def get_wheel_telemetry(self) -> WheelTelemetry:
        """Lấy thông số 4 bánh xe."""
        pass

    @abstractmethod
    def get_imu_telemetry(self) -> ImuTelemetry:
        """Lấy dữ liệu cảm biến IMU."""
        pass

    @abstractmethod
    def get_lidar_telemetry(self) -> LidarTelemetry:
        """Lấy dữ liệu cảm biến LiDAR."""
        pass

    @abstractmethod
    def get_odometry(self) -> OdometryTelemetry:
        """Lấy thông tin vị trí odometry."""
        pass

    @abstractmethod
    def get_config(self) -> RobotConfig:
        """Lấy cấu hình hiện tại của xe."""
        pass

    @abstractmethod
    def update_config(self, config: RobotConfig) -> RobotConfig:
        """Cập nhật cấu hình xe."""
        pass

    @abstractmethod
    def reset_odometry(self) -> None:
        """Reset vị trí x, y, theta về 0."""
        pass

    def get_path_telemetry(self) -> PathTelemetry:
        """Lấy lộ trình toàn cục và cục bộ."""
        return PathTelemetry()

    def get_debug_telemetry(self) -> Optional[DebugTelemetry]:
        """Lấy dữ liệu telemetry chi tiết phục vụ đồ thị debug."""
        return None

    def get_map(self) -> Optional[dict]:
        """Lấy dữ liệu OccupancyGrid bản đồ."""
        return None

    async def send_nav_goal(self, x: float, y: float, theta: float = 0.0, frame_id: str = "map") -> bool:
        """Gửi tọa độ mục tiêu cho Nav2 / bộ điều khiển tự hành."""
        return True

    async def cancel_nav_goal(self) -> bool:
        """Hủy bỏ mục tiêu tự hành hiện tại và dừng xe."""
        return True

    def clear_map_memory(self) -> None:
        """Xóa bộ nhớ bản đồ vật cản tích lũy."""
        pass

    def get_nav_status(self) -> dict:
        """Lấy trạng thái tự hành hiện tại."""
        return {"state": "idle", "goal": None}

    def get_latest_camera_frame(self, camera_type: str = "rgb") -> Optional[bytes]:
        """Lấy frame ảnh mới nhất (rgb hoặc depth) dạng JPEG bytes."""
        return None


