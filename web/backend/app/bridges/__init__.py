import logging
import os
from app.bridges.base import BaseRobotBridge

logger = logging.getLogger("amr_web.bridge")

_current_bridge: BaseRobotBridge | None = None


def get_bridge() -> BaseRobotBridge:
    global _current_bridge
    if _current_bridge is not None:
        return _current_bridge

    requested_mode = os.getenv("ROBOT_BRIDGE_MODE", "auto").lower()

    if requested_mode == "ros2":
        from app.bridges.ros2_bridge import Ros2Bridge, HAS_RCLPY
        if not HAS_RCLPY:
            raise RuntimeError(
                "Chế độ kết nối được chọn là 'ros2' nhưng rclpy không khả dụng trong môi trường hiện tại! "
                "Hãy đảm bảo bạn đã source ROS 2 (/opt/ros/jazzy/setup.bash) "
                "và workspace overlay (install/ros2_jazzy/local_setup.bash)."
            )
        logger.info("Khởi tạo ROS 2 Jazzy Bridge (rclpy)...")
        _current_bridge = Ros2Bridge()
        return _current_bridge

    if requested_mode == "ros1":
        from app.bridges.ros1_bridge import Ros1Bridge
        logger.info("Khởi tạo ROS 1 Melodic Bridge (Jetson)...")
        _current_bridge = Ros1Bridge()
        return _current_bridge

    # Auto mode: kiểm tra ROS 2 trước, fallback sang ROS 1
    try:
        from app.bridges.ros2_bridge import Ros2Bridge, HAS_RCLPY
        if HAS_RCLPY:
            logger.info("Khởi tạo ROS 2 Jazzy Bridge (rclpy) ở chế độ tự động...")
            _current_bridge = Ros2Bridge()
            return _current_bridge
    except Exception as e:
        logger.warning(f"Không thể khởi tạo ROS 2 Bridge: {e}. Thử ROS 1...")

    try:
        from app.bridges.ros1_bridge import Ros1Bridge
        logger.info("Khởi tạo ROS 1 Melodic Bridge (Jetson)...")
        _current_bridge = Ros1Bridge()
        return _current_bridge
    except Exception as e:
        logger.warning(f"Không thể khởi tạo ROS 1 Bridge: {e}.")

    from app.bridges.ros2_bridge import Ros2Bridge
    _current_bridge = Ros2Bridge()
    return _current_bridge
