import asyncio
import json
import logging
import time
try:
    import psutil
    HAS_PSUTIL = True
except ImportError:
    HAS_PSUTIL = False
    psutil = None

logger = logging.getLogger("amr_web.ros1_bridge")

from app.bridges.base import BaseRobotBridge
from app.models import (
    ImuTelemetry,
    LidarTelemetry,
    OdometryTelemetry,
    PathTelemetry,
    RobotConfig,
    RobotStatus,
    WheelTelemetry,
)
from app.services.stream_monitor import stream_monitor


class Ros1Bridge(BaseRobotBridge):
    """ROS 1 Melodic Bridge dành cho Jetson Nano qua Rosbridge WebSocket client."""

    def __init__(self, ws_url: str = "ws://localhost:9090"):
        self.ws_url = ws_url
        self.config = RobotConfig()
        self.running = False
        self.connected = False
        self.start_time = time.time()
        self._task: Optional[asyncio.Task] = None

        # Telemetry cache
        self.estop_active = False
        self.safety_stop = False
        self.measured_wheel_speeds = [0.0, 0.0, 0.0, 0.0]
        self.target_wheel_speeds = [0.0, 0.0, 0.0, 0.0]
        self.encoder_ticks = [0, 0, 0, 0]
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.yaw_deg = 0.0
        self.lidar_ranges: List[float] = []
        self.lidar_points: List[List[float]] = []
        self.active_goal: Optional[dict] = None
        self.global_path: List[List[float]] = []
        self.local_path: List[List[float]] = []
        self.nav_state: str = "idle"
        self._ws = None

    async def start(self) -> None:
        self.running = True
        self.start_time = time.time()
        self._task = asyncio.create_task(self._connect_loop())

    async def stop(self) -> None:
        self.running = False
        if self._ws:
            try:
                await self._ws.close()
            except Exception:
                pass
            self._ws = None
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self._task = None

    async def _connect_loop(self):
        """Thử kết nối tới rosbridge_server (port 9090)."""
        try:
            import websockets
        except ImportError:
            logger.warning("Thư viện 'websockets' không khả dụng.")
            self.connected = False
            return

        while self.running:
            try:
                async with websockets.connect(self.ws_url, ping_interval=5) as ws:
                    self._ws = ws
                    self.connected = True
                    # Subscribe to topics
                    await ws.send(json.dumps({"op": "subscribe", "topic": "/scan", "type": "sensor_msgs/LaserScan"}))
                    await ws.send(json.dumps({"op": "subscribe", "topic": "/odom", "type": "nav_msgs/Odometry"}))
                    await ws.send(json.dumps({"op": "subscribe", "topic": "/wheel_state", "type": "std_msgs/Float32MultiArray"}))

                    while self.running:
                        msg_str = await ws.recv()
                        msg = json.loads(msg_str)
                        topic = msg.get("topic")
                        data = msg.get("msg", {})

                        if topic == "/wheel_state":
                            if "data" in data and len(data["data"]) >= 4:
                                self.measured_wheel_speeds = [float(v) for v in data["data"][:4]]
                                stream_monitor.record("stm32_wheel_state", str(self.measured_wheel_speeds))
                        elif topic == "/odom":
                            pos = data.get("pose", {}).get("pose", {}).get("position", {})
                            self.odom_x = pos.get("x", 0.0)
                            self.odom_y = pos.get("y", 0.0)
                            stream_monitor.record("localization_odom", f"x={self.odom_x:.2f}, y={self.odom_y:.2f}")
                        elif topic == "/scan":
                            ranges = data.get("ranges", [])
                            self.lidar_ranges = ranges
                            stream_monitor.record("sensor_lidar", f"rays={len(ranges)}")
            except Exception:
                self.connected = False
                self._ws = None
                await asyncio.sleep(2.0)

    async def send_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        stream_monitor.record("navigation_cmd_vel", f"vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f}")
        if self.connected and self._ws is not None:
            try:
                msg = {
                    "op": "publish",
                    "topic": "/cmd_vel",
                    "msg": {
                        "linear": {"x": float(vx), "y": float(vy), "z": 0.0},
                        "angular": {"x": 0.0, "y": 0.0, "z": float(wz)},
                    },
                }
                await self._ws.send(json.dumps(msg))
            except Exception:
                pass

    async def send_estop(self, active: bool) -> None:
        self.estop_active = active
        if self.connected and self._ws is not None:
            try:
                msg = {
                    "op": "publish",
                    "topic": "/estop",
                    "msg": {"data": bool(active)},
                }
                await self._ws.send(json.dumps(msg))
            except Exception:
                pass

    def get_status(self) -> RobotStatus:
        now = time.time()
        cpu = 0.0
        ram = 0.0
        try:
            cpu = round(psutil.cpu_percent(), 1)
            ram = round(psutil.virtual_memory().percent, 1)
        except Exception:
            pass
        return RobotStatus(
            mode="hardware",
            connected=self.connected,
            estop_active=self.estop_active,
            safety_stop=self.safety_stop,
            uptime_sec=round(now - self.start_time, 1),
            cpu_percent=cpu,
            ram_percent=ram,
            active_streams=len([s for s in stream_monitor.get_all() if s.status == "active"]),
            timestamp_sec=now,
        )

    def get_wheel_telemetry(self) -> WheelTelemetry:
        return WheelTelemetry(
            target_rad_s=self.target_wheel_speeds,
            measured_rad_s=self.measured_wheel_speeds,
            encoder_ticks=self.encoder_ticks,
        )

    def get_imu_telemetry(self) -> ImuTelemetry:
        return ImuTelemetry(yaw_deg=self.yaw_deg)

    def get_lidar_telemetry(self) -> LidarTelemetry:
        return LidarTelemetry(ranges=self.lidar_ranges, points=self.lidar_points)

    def get_odometry(self) -> OdometryTelemetry:
        return OdometryTelemetry(x=self.odom_x, y=self.odom_y, theta_rad=self.odom_theta)

    def get_config(self) -> RobotConfig:
        return self.config

    def update_config(self, config: RobotConfig) -> RobotConfig:
        self.config = config
        return self.config

    def reset_odometry(self) -> None:
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0

    async def cancel_nav_goal(self) -> bool:
        self.active_goal = None
        self.nav_state = "cancelled"
        self.global_path = []
        self.local_path = []
        await self.send_cmd_vel(0.0, 0.0, 0.0)
        return True

    async def send_nav_goal(self, x: float, y: float, theta: float = 0.0, frame_id: str = "map") -> bool:
        self.active_goal = {"x": float(x), "y": float(y), "theta": float(theta)}
        self.nav_state = "navigating"
        steps = 15
        self.global_path = [
            [
                round(self.odom_x + (x - self.odom_x) * (i / steps), 3),
                round(self.odom_y + (y - self.odom_y) * (i / steps), 3),
            ]
            for i in range(steps + 1)
        ]
        self.local_path = [
            [round(self.odom_x, 3), round(self.odom_y, 3)],
            [round(x, 3), round(y, 3)],
        ]
        return True

    def get_path_telemetry(self) -> PathTelemetry:
        return PathTelemetry(global_path=self.global_path, local_path=self.local_path)

    def clear_map_memory(self) -> None:
        self.global_path = []
        self.local_path = []
