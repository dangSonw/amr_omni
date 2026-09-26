import math
import threading
import time
from unittest.mock import MagicMock

from app.bridges.ros2_bridge import Ros2Bridge


class DummyPoint:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class DummyQuaternion:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w


class DummyPose:
    def __init__(self, x=0.0, y=0.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        self.position = DummyPoint(x, y, z)
        self.orientation = DummyQuaternion(qx, qy, qz, qw)


class DummyTwist:
    def __init__(self, vx=0.0, vy=0.0, wz=0.0):
        self.linear = DummyPoint(vx, vy, 0.0)
        self.angular = DummyPoint(0.0, 0.0, wz)


class DummyOdometryMsg:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, vx=0.0, vy=0.0, wz=0.0):
        qw = math.cos(yaw / 2.0)
        qz = math.sin(yaw / 2.0)
        self.pose = MagicMock()
        self.pose.pose = DummyPose(x, y, 0.0, 0.0, 0.0, qz, qw)
        self.twist = MagicMock()
        self.twist.twist = DummyTwist(vx, vy, wz)


class DummyImuMsg:
    def __init__(self, yaw=0.0):
        qw = math.cos(yaw / 2.0)
        qz = math.sin(yaw / 2.0)
        self.orientation = DummyQuaternion(0.0, 0.0, qz, qw)
        self.linear_acceleration = DummyPoint(0.0, 0.0, 9.81)
        self.angular_velocity = DummyPoint(0.0, 0.0, 0.0)


class DummyScanMsg:
    def __init__(self, num_points=180, dist=1.5):
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = (2.0 * math.pi) / num_points
        self.range_min = 0.15
        self.range_max = 12.0
        self.ranges = [dist] * num_points


def test_c1_imu_does_not_overwrite_filtered_odom():
    """C1: Kiểm tra _on_imu KHÔNG ghi đè self.odom_theta khi đã có filtered odom."""
    bridge = Ros2Bridge()
    assert bridge.odom_theta == 0.0
    assert not bridge._has_filtered_odom

    # 1. Khi chưa có filtered odom: IMU cập nhật odom_theta
    imu_msg1 = DummyImuMsg(yaw=0.785)  # 45 deg
    bridge._on_imu(imu_msg1)
    assert math.isclose(bridge.odom_theta, 0.785, abs_tol=1e-3)
    assert math.isclose(bridge.yaw_deg, 45.0, abs_tol=0.1)

    # 2. Khi /odometry/filtered cập nhật: _has_filtered_odom = True
    odom_msg = DummyOdometryMsg(x=1.0, y=2.0, yaw=1.57)  # 90 deg
    bridge._on_odom(odom_msg)
    assert bridge._has_filtered_odom
    assert math.isclose(bridge.odom_theta, 1.57, abs_tol=1e-3)

    # 3. IMU message mới đến sau: KHÔNG được ghi đè odom_theta
    imu_msg2 = DummyImuMsg(yaw=0.1)
    bridge._on_imu(imu_msg2)
    # odom_theta vẫn giữ giá trị từ EKF odom (1.57), không bị ghi đè thành 0.1
    assert math.isclose(bridge.odom_theta, 1.57, abs_tol=1e-3)
    # Trong khi IMU yaw riêng vẫn được ghi nhận
    assert math.isclose(bridge.imu_yaw_rad, 0.1, abs_tol=1e-3)


def test_c2_data_lock_thread_safety():
    """C2: Kiểm tra đa luồng đọc và ghi dữ liệu odom/lidar không bị crash hoặc race condition."""
    bridge = Ros2Bridge()
    running = True

    def writer_thread():
        step = 0
        while running:
            step += 1
            odom_msg = DummyOdometryMsg(x=float(step), y=float(step * 2), yaw=0.1 * step)
            bridge._on_odom(odom_msg)
            time.sleep(0.001)

    def reader_thread():
        while running:
            odom = bridge.get_odometry()
            assert odom is not None
            # Tọa độ y luôn là 2 * x (atomic snapshot consistency)
            assert math.isclose(odom.y, 2.0 * odom.x, abs_tol=1e-2)
            time.sleep(0.001)

    threads = [
        threading.Thread(target=writer_thread),
        threading.Thread(target=reader_thread),
    ]
    for t in threads:
        t.start()

    time.sleep(0.1)
    running = False
    for t in threads:
        t.join()


def test_c3_scan_worker_non_blocking_execution():
    """C3: Kiểm tra _on_scan không gọi add_scan trực tiếp đồng bộ trong callback thread mà đưa qua worker."""
    bridge = Ros2Bridge()
    bridge.running = True

    # Khởi động worker thread
    worker_thread = threading.Thread(target=bridge._scan_worker, daemon=True)
    worker_thread.start()

    scan_msg = DummyScanMsg(num_points=100, dist=2.0)

    # Đo thời gian _on_scan: phải cực kỳ nhanh (< 1ms) vì không chạy heavy add_scan đồng bộ
    t0 = time.perf_counter()
    bridge._on_scan(scan_msg)
    elapsed_ms = (time.perf_counter() - t0) * 1000.0

    assert elapsed_ms < 5.0, f"_on_scan took too long: {elapsed_ms:.2f}ms"
    assert len(bridge.lidar_points) > 0

    # Chờ worker thread lấy và xử lý scan
    time.sleep(0.1)

    # Bộ nhớ bản đồ grid_planner đã được worker cập nhật
    obstacles = bridge.get_path_telemetry().obstacles
    assert len(obstacles) > 0, "Scan worker should have updated grid_planner obstacles in background"

    bridge.running = False
    bridge._scan_queue.put(None)
    worker_thread.join(timeout=0.5)
