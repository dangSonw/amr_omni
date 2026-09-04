from abc import ABC, abstractmethod
from app.models import (
    ImuTelemetry,
    LidarTelemetry,
    OdometryTelemetry,
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
