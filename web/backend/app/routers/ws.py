import json
import logging
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from app.services.telemetry_hub import telemetry_hub

router = APIRouter(tags=["websockets"])
logger = logging.getLogger("amr_web.ws")


@router.websocket("/ws/telemetry")
async def websocket_telemetry_endpoint(websocket: WebSocket):
    await telemetry_hub.register(websocket)
    try:
        while True:
            # Lắng nghe các lệnh điều khiển gửi ngược từ client qua cùng socket
            raw_text = await websocket.receive_text()
            try:
                data = json.loads(raw_text)
                await telemetry_hub.handle_client_message(data, websocket=websocket)
            except json.JSONDecodeError:
                pass
    except WebSocketDisconnect:
        telemetry_hub.unregister(websocket)
    except Exception as e:
        logger.error(f"Lỗi WebSocket: {e}")
        telemetry_hub.unregister(websocket)

