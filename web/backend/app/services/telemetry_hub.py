import asyncio
import json
import logging
from typing import Set
from fastapi import WebSocket

from app.bridges import get_bridge
from app.services.stream_monitor import stream_monitor

logger = logging.getLogger("amr_web.telemetry_hub")


class TelemetryHub:
    def __init__(self):
        self.active_connections: Set[WebSocket] = set()
        self._broadcast_task: asyncio.Task | None = None
        self.running = False

    async def start(self):
        if self.running:
            return
        self.running = True
        self._broadcast_task = asyncio.create_task(self._broadcast_loop())
        logger.info("TelemetryHub broadcast loop started.")

    async def stop(self):
        self.running = False
        if self._broadcast_task:
            self._broadcast_task.cancel()
            try:
                await self._broadcast_task
            except asyncio.CancelledError:
                pass
            self._broadcast_task = None
        logger.info("TelemetryHub broadcast loop stopped.")

    async def register(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.add(websocket)
        logger.info(f"WebSocket client connected. Total clients: {len(self.active_connections)}")

    def unregister(self, websocket: WebSocket):
        self.active_connections.discard(websocket)
        logger.info(f"WebSocket client disconnected. Total clients: {len(self.active_connections)}")

    async def handle_client_message(self, data: dict):
        msg_type = data.get("type")
        bridge = get_bridge()

        if msg_type == "cmd_vel":
            vx = float(data.get("vx", 0.0))
            vy = float(data.get("vy", 0.0))
            wz = float(data.get("wz", 0.0))
            await bridge.send_cmd_vel(vx, vy, wz)
        elif msg_type == "estop":
            active = bool(data.get("active", False))
            await bridge.send_estop(active)
        elif msg_type == "reset_odom":
            bridge.reset_odometry()

    async def _broadcast_loop(self):
        # Broadcast ở tần số 20 Hz
        interval = 1.0 / 20.0
        bridge = get_bridge()

        while self.running:
            if self.active_connections:
                try:
                    payload = {
                        "type": "telemetry",
                        "status": bridge.get_status().model_dump(),
                        "wheels": bridge.get_wheel_telemetry().model_dump(),
                        "imu": bridge.get_imu_telemetry().model_dump(),
                        "odom": bridge.get_odometry().model_dump(),
                        "lidar": bridge.get_lidar_telemetry().model_dump(),
                        "streams": [s.model_dump() for s in stream_monitor.get_all()],
                    }
                    message = json.dumps(payload)

                    # Gửi tới tất cả các client
                    disconnected = set()
                    for ws in self.active_connections:
                        try:
                            await ws.send_text(message)
                        except Exception:
                            disconnected.add(ws)

                    for ws in disconnected:
                        self.unregister(ws)

                except Exception as e:
                    logger.error(f"Lỗi khi broadcast telemetry: {e}")

            await asyncio.sleep(interval)


telemetry_hub = TelemetryHub()

