import asyncio
import math
import random
import time
from typing import List, Tuple

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


class MockSimulatorBridge(BaseRobotBridge):
    """Mô phỏng động học xe AMR Omni 4 bánh Mecanum và cảm biến (LiDAR, IMU, Encoders).

    Cung cấp môi trường chạy độc lập cho Web Dashboard khi không kết nối ROS.
    """

    def __init__(self):
        self.config = RobotConfig()
        self.running = False
        self._task: asyncio.Task | None = None

        # Robot state
        self.start_time = time.time()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        # Command & safety
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0
        self.last_cmd_time = time.time()
        self.estop_active = False
        self.safety_stop = False

        # Wheel speeds (rad/s) and encoder ticks
        self.target_wheel_speeds = [0.0, 0.0, 0.0, 0.0]
        self.measured_wheel_speeds = [0.0, 0.0, 0.0, 0.0]
        self.encoder_ticks = [0, 0, 0, 0]

        # IMU state
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.gyro_z = 0.0
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 9.81

        # LiDAR state
        self.lidar_ranges: List[float] = [0.0] * 360
        self.lidar_points: List[List[float]] = []

        # Virtual room boundaries & obstacles for LiDAR ray-tracing
        # Room: -4.0 to +4.0 m in x and y.
        # Obstacles: rectangles [x_min, y_min, x_max, y_max]
        self.obstacles = [
            (-4.0, -4.0, 4.0, 4.0),  # Room outer wall
            (1.0, 1.0, 2.0, 2.5),    # Table / pillar 1
            (-2.5, -2.0, -1.0, -1.2),# Wall obstacle 2
            (1.5, -2.5, 2.5, -1.5),  # Pallet box 3
            (-2.0, 1.5, -1.0, 2.5),  # Equipment rack 4
        ]

    async def start(self) -> None:
        if self.running:
            return
        self.running = True
        self.start_time = time.time()
        self._task = asyncio.create_task(self._simulation_loop())

    async def stop(self) -> None:
        self.running = False
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self._task = None

    async def send_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        now = time.time()
        # Giới hạn tốc độ theo config
        max_lin = self.config.max_linear_speed_mps
        max_ang = self.config.max_angular_speed_rad_s
        self.target_vx = max(-max_lin, min(max_lin, vx))
        self.target_vy = max(-max_lin, min(max_lin, vy))
        self.target_wz = max(-max_ang, min(max_ang, wz))
        self.last_cmd_time = now

        # Ghi nhận packet vào luồng cmd_vel
        stream_monitor.record(
            "navigation_cmd_vel",
            f"vx={self.target_vx:.2f}, vy={self.target_vy:.2f}, wz={self.target_wz:.2f}",
        )

    async def send_estop(self, active: bool) -> None:
        self.estop_active = active
        if active:
            self.target_vx = 0.0
            self.target_vy = 0.0
            self.target_wz = 0.0

    def get_status(self) -> RobotStatus:
        now = time.time()
        age = now - self.last_cmd_time
        active_count = sum(
            1 for s in stream_monitor.get_all() if s.status == "active"
        )
        return RobotStatus(
            mode="mock",
            connected=True,
            estop_active=self.estop_active,
            safety_stop=self.safety_stop or age > self.config.command_timeout_sec,
            uptime_sec=round(now - self.start_time, 1),
            cpu_percent=round(12.5 + random.uniform(-2, 3), 1),
            ram_percent=38.2,
            active_streams=active_count,
            timestamp_sec=now,
        )

    def get_wheel_telemetry(self) -> WheelTelemetry:
        return WheelTelemetry(
            target_rad_s=[round(s, 2) for s in self.target_wheel_speeds],
            measured_rad_s=[round(s, 2) for s in self.measured_wheel_speeds],
            encoder_ticks=list(self.encoder_ticks),
            pwm_commands=[round(s * 10.0, 1) for s in self.measured_wheel_speeds],
        )

    def get_imu_telemetry(self) -> ImuTelemetry:
        return ImuTelemetry(
            roll_deg=round(math.degrees(self.roll), 2),
            pitch_deg=round(math.degrees(self.pitch), 2),
            yaw_deg=round(math.degrees(self.yaw) % 360.0, 2),
            accel_x=round(self.accel_x, 3),
            accel_y=round(self.accel_y, 3),
            accel_z=round(self.accel_z, 3),
            gyro_x=round(random.uniform(-0.01, 0.01), 3),
            gyro_y=round(random.uniform(-0.01, 0.01), 3),
            gyro_z=round(self.gyro_z, 3),
        )

    def get_lidar_telemetry(self) -> LidarTelemetry:
        return LidarTelemetry(
            angle_min=-math.pi,
            angle_max=math.pi,
            angle_increment=2.0 * math.pi / 360.0,
            range_min=0.12,
            range_max=12.0,
            ranges=self.lidar_ranges,
            points=self.lidar_points,
        )

    def get_odometry(self) -> OdometryTelemetry:
        return OdometryTelemetry(
            x=round(self.x, 3),
            y=round(self.y, 3),
            theta_rad=round(self.theta, 3),
            vx=round(self.vx, 3),
            vy=round(self.vy, 3),
            wz=round(self.wz, 3),
        )

    def get_config(self) -> RobotConfig:
        return self.config

    def update_config(self, config: RobotConfig) -> RobotConfig:
        self.config = config
        return self.config

    def reset_odometry(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.encoder_ticks = [0, 0, 0, 0]

    def _inverse_kinematics(self, vx: float, vy: float, wz: float) -> Tuple[float, float, float, float]:
        r = self.config.wheel_radius_m
        l_plus_w = 0.5 * (self.config.wheelbase_m + self.config.track_width_m)
        diagonal = math.sqrt(0.5)

        # Động học 4 bánh Mecanum tiêu chuẩn (khớp với kinematics.py của dự án)
        v1 = diagonal * vx + diagonal * vy + l_plus_w * wz
        v2 = -diagonal * vx + diagonal * vy + l_plus_w * wz
        v3 = -diagonal * vx - diagonal * vy + l_plus_w * wz
        v4 = diagonal * vx - diagonal * vy + l_plus_w * wz

        w1 = v1 / r
        w2 = v2 / r
        w3 = v3 / r
        w4 = v4 / r

        # Scale nếu vượt quá max_wheel_speed
        max_speed = self.config.max_wheel_speed_rad_s
        scale = max(1.0, max(abs(w) for w in (w1, w2, w3, w4)) / max_speed)
        return (w1 / scale, w2 / scale, w3 / scale, w4 / scale)

    def _update_lidar(self):
        """Mô phỏng chùm quét LiDAR 360 tia (15Hz) trong phòng 8x8m."""
        num_rays = 360
        ranges = []
        points = []

        # Tọa độ thế giới của robot
        rx = self.x
        ry = self.y
        rtheta = self.theta

        for i in range(num_rays):
            # Góc tia quét trong hệ tọa độ robot
            ray_angle_robot = -math.pi + i * (2.0 * math.pi / num_rays)
            # Góc tia quét trong hệ tọa độ thế giới (world)
            world_angle = rtheta + ray_angle_robot
            cos_a = math.cos(world_angle)
            sin_a = math.sin(world_angle)

            # Ray-cast tới các bức tường ngoài [-4, 4]
            dist = 10.0  # max range

            # Tường x = 4.0
            if cos_a > 1e-4:
                d = (4.0 - rx) / cos_a
                y_hit = ry + d * sin_a
                if -4.0 <= y_hit <= 4.0 and 0 < d < dist:
                    dist = d
            # Tường x = -4.0
            elif cos_a < -1e-4:
                d = (-4.0 - rx) / cos_a
                y_hit = ry + d * sin_a
                if -4.0 <= y_hit <= 4.0 and 0 < d < dist:
                    dist = d

            # Tường y = 4.0
            if sin_a > 1e-4:
                d = (4.0 - ry) / sin_a
                x_hit = rx + d * cos_a
                if -4.0 <= x_hit <= 4.0 and 0 < d < dist:
                    dist = d
            # Tường y = -4.0
            elif sin_a < -1e-4:
                d = (-4.0 - ry) / sin_a
                x_hit = rx + d * cos_a
                if -4.0 <= x_hit <= 4.0 and 0 < d < dist:
                    dist = d

            # Kiểm tra va chạm với các khối chướng ngại vật nội bộ
            for x1, y1, x2, y2 in self.obstacles[1:]:
                # Cắt cạnh trái x1
                if cos_a > 1e-4:
                    d = (x1 - rx) / cos_a
                    yh = ry + d * sin_a
                    if y1 <= yh <= y2 and 0 < d < dist:
                        dist = d
                # Cắt cạnh phải x2
                if cos_a < -1e-4:
                    d = (x2 - rx) / cos_a
                    yh = ry + d * sin_a
                    if y1 <= yh <= y2 and 0 < d < dist:
                        dist = d
                # Cắt cạnh dưới y1
                if sin_a > 1e-4:
                    d = (y1 - ry) / sin_a
                    xh = rx + d * cos_a
                    if x1 <= xh <= x2 and 0 < d < dist:
                        dist = d
                # Cắt cạnh trên y2
                if sin_a < -1e-4:
                    d = (y2 - ry) / sin_a
                    xh = rx + d * cos_a
                    if x1 <= xh <= x2 and 0 < d < dist:
                        dist = d

            # Thêm nhiễu nhẹ Gaussian mô phỏng cảm biến thật
            dist_with_noise = max(0.15, min(12.0, dist + random.gauss(0.0, 0.015)))
            ranges.append(round(dist_with_noise, 3))

            # Tọa độ điểm chướng ngại vật trong hệ quy chiếu robot
            px = dist_with_noise * math.cos(ray_angle_robot)
            py = dist_with_noise * math.sin(ray_angle_robot)
            points.append([round(px, 3), round(py, 3)])

        self.lidar_ranges = ranges
        self.lidar_points = points

    async def _simulation_loop(self):
        dt = 0.02  # 50 Hz loop
        lidar_counter = 0

        while self.running:
            loop_start = time.time()
            now = loop_start

            # 1. Watchdog check
            cmd_age = now - self.last_cmd_time
            if self.estop_active or cmd_age > self.config.command_timeout_sec:
                self.safety_stop = True
                active_vx = 0.0
                active_vy = 0.0
                active_wz = 0.0
            else:
                self.safety_stop = False
                active_vx = self.target_vx
                active_vy = self.target_vy
                active_wz = self.target_wz

            # Ghi nhận luồng Jetson -> STM32
            stream_monitor.record(
                "jetson_cmd_vel",
                f"vx={active_vx:.2f}, vy={active_vy:.2f}, wz={active_wz:.2f}",
            )

            # 2. Động lực học mượt mà (smooth acceleration)
            smooth = 0.25
            self.vx += (active_vx - self.vx) * smooth
            self.vy += (active_vy - self.vy) * smooth
            self.wz += (active_wz - self.wz) * smooth

            # 3. Tính tốc độ 4 bánh xe
            w_targets = self._inverse_kinematics(self.vx, self.vy, self.wz)
            self.target_wheel_speeds = list(w_targets)

            # Bánh xe phản hồi có trễ và nhiễu thực tế
            for i in range(4):
                meas = self.measured_wheel_speeds[i] + (w_targets[i] - self.measured_wheel_speeds[i]) * 0.35
                meas += random.gauss(0.0, 0.03) if abs(meas) > 0.05 else 0.0
                self.measured_wheel_speeds[i] = meas

                # Tích lũy encoder ticks (2048 ticks / rev)
                delta_ticks = int(meas * dt * (2048.0 / (2.0 * math.pi)))
                self.encoder_ticks[i] += delta_ticks

            # 4. Tích phân Odometry (vị trí robot)
            # Vận tốc trong hệ tọa độ robot chuyển sang hệ thế giới
            delta_x_robot = self.vx * dt
            delta_y_robot = self.vy * dt
            delta_theta = self.wz * dt

            cos_th = math.cos(self.theta)
            sin_th = math.sin(self.theta)
            self.x += cos_th * delta_x_robot - sin_th * delta_y_robot
            self.y += sin_th * delta_x_robot + cos_th * delta_y_robot
            self.theta += delta_theta
            self.theta = (self.theta + math.pi) % (2.0 * math.pi) - math.pi

            # Giữ robot trong tường phòng 8x8m
            self.x = max(-3.7, min(3.7, self.x))
            self.y = max(-3.7, min(3.7, self.y))

            # 5. Cập nhật IMU
            self.yaw = self.theta
            self.gyro_z = self.wz + random.gauss(0.0, 0.01)
            # Gia tốc kế đo gia tốc tịnh tiến + rung động
            self.accel_x = (active_vx - self.vx) / dt + random.gauss(0.0, 0.05)
            self.accel_y = (active_vy - self.vy) / dt + random.gauss(0.0, 0.05)
            self.accel_z = 9.81 + random.gauss(0.0, 0.02)

            # 6. Ghi nhận luồng dữ liệu STM32 -> Jetson
            stream_monitor.record("stm32_wheel_state", f"{[round(s, 2) for s in self.measured_wheel_speeds]}")
            stream_monitor.record("stm32_encoder_counts", f"{self.encoder_ticks}")
            stream_monitor.record("stm32_imu", f"yaw={math.degrees(self.yaw):.1f}°, gz={self.gyro_z:.2f}")
            stream_monitor.record("localization_odom", f"x={self.x:.2f}, y={self.y:.2f}, th={math.degrees(self.theta):.1f}°")

            # Luồng status và diagnostics 10Hz
            if int(now * 10) % 5 == 0:
                stream_monitor.record("stm32_status", "STATE=OK, BAT=24.2V, MCU_TEMP=38C")
                stream_monitor.record("stm32_diagnostics", "DRIVE_MOTORS=OK, CAN_BUS=OK, WATCHDOG=OK")

            # 7. Cập nhật LiDAR ở tần số 15 Hz (~mỗi 3-4 vòng lặp)
            lidar_counter += 1
            if lidar_counter >= 3:
                lidar_counter = 0
                self._update_lidar()
                stream_monitor.record("sensor_lidar", f"rays=360, min={min(self.lidar_ranges):.2f}m")

            # Camera 30Hz simulation
            if lidar_counter % 2 == 0:
                stream_monitor.record("sensor_camera", "640x480 RGB @ 30fps")

            # Giữ nhịp 50 Hz
            elapsed = time.time() - loop_start
            sleep_time = max(0.001, dt - elapsed)
            await asyncio.sleep(sleep_time)

