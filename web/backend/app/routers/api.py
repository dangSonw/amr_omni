import asyncio
import os
import subprocess
from typing import List, Optional
from fastapi import APIRouter
from starlette.responses import StreamingResponse

from app.bridges import get_bridge
from app.models import (
    EStopCommand,
    NavGoalRequest,
    PathTelemetry,
    RobotConfig,
    RobotStatus,
    SaveMapRequest,
    StreamInfo,
    TwistCommand,
    WaypointsRequest,
)
from app.services.stream_monitor import stream_monitor

try:
    import cv2
    import numpy as np
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

router = APIRouter(prefix="/api", tags=["robot"])


def _get_placeholder_jpeg(label: str) -> bytes:
    if HAS_CV2:
        img = np.zeros((240, 320, 3), dtype=np.uint8)
        img[:] = (22, 15, 9)  # Dark background BGR
        cv2.putText(img, "AMR OMNI VISION", (55, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (148, 163, 184), 2)
        cv2.putText(img, f"Dang cho luong {label}...", (35, 135), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (99, 102, 241), 1)
        cv2.rectangle(img, (10, 10), (310, 230), (51, 65, 85), 1)
        _, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
        return buf.tobytes()
    return b""


@router.get("/status", response_model=RobotStatus)
async def get_robot_status():
    bridge = get_bridge()
    return bridge.get_status()


@router.get("/config", response_model=RobotConfig)
async def get_robot_config():
    bridge = get_bridge()
    return bridge.get_config()


@router.post("/config", response_model=RobotConfig)
async def update_robot_config(config: RobotConfig):
    bridge = get_bridge()
    return bridge.update_config(config)


@router.get("/streams", response_model=List[StreamInfo])
async def get_stream_matrix():
    """Lấy danh sách và tình trạng chi tiết của từng luồng dữ liệu Jetson <-> STM32 <-> Sensors."""
    return stream_monitor.get_all()


@router.post("/cmd-vel")
async def send_cmd_vel(cmd: TwistCommand):
    bridge = get_bridge()
    await bridge.send_cmd_vel(cmd.linear_x, cmd.linear_y, cmd.angular_z)
    return {"status": "ok", "command": cmd}


@router.post("/estop")
async def toggle_estop(cmd: EStopCommand):
    bridge = get_bridge()
    await bridge.send_estop(cmd.active)
    return {"status": "ok", "estop_active": cmd.active}


@router.post("/reset-odom")
async def reset_odometry():
    bridge = get_bridge()
    bridge.reset_odometry()
    return {"status": "ok", "message": "Odometry reset to (0, 0, 0)"}


# Phase 5: Autonomy & Advanced Dashboard APIs


@router.get("/path", response_model=PathTelemetry)
async def get_path():
    bridge = get_bridge()
    return bridge.get_path_telemetry()


@router.get("/map")
async def get_map_data():
    bridge = get_bridge()
    data = bridge.get_map()
    if data:
        return {"status": "ok", "map": data}
    return {"status": "no_map", "message": "Chưa nhận được OccupancyGrid từ topic /map"}


@router.post("/nav/goal")
async def dispatch_nav_goal(goal: NavGoalRequest):
    bridge = get_bridge()
    success = await bridge.send_nav_goal(goal.x, goal.y, goal.theta_rad, goal.frame_id)
    return {
        "status": "ok" if success else "error",
        "dispatched": success,
        "goal": goal,
    }


@router.post("/nav/waypoints")
async def dispatch_waypoints(req: WaypointsRequest):
    bridge = get_bridge()
    if not req.waypoints:
        return {"status": "error", "message": "Danh sách waypoints rỗng"}
    # Dispatch điểm đầu tiên ngay lập tức
    first = req.waypoints[0]
    await bridge.send_nav_goal(first.x, first.y, first.theta_rad)
    return {
        "status": "ok",
        "count": len(req.waypoints),
        "loop": req.loop,
        "first_goal": first,
    }


@router.post("/map/save")
async def save_map(req: SaveMapRequest):
    """Lưu bản đồ qua nav2_map_server hoặc CLI script."""
    ws_root = os.environ.get("AMR_WORKSPACE", os.getcwd())
    save_dir = os.path.join(ws_root, "maps")
    os.makedirs(save_dir, exist_ok=True)
    target_prefix = os.path.join(save_dir, req.map_name)

    # Thử chạy map_saver_cli của ROS 2 nếu có
    cmd = ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", target_prefix]
    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        return {
            "status": "saving",
            "map_prefix": target_prefix,
            "message": "Đang tiến hành lưu bản đồ",
        }
    except Exception as e:
        return {
            "status": "fallback",
            "map_prefix": target_prefix,
            "message": f"Chạy lệnh lưu: {e}",
        }


@router.post("/map/clear")
async def clear_map():
    bridge = get_bridge()
    bridge.clear_map_memory()
    return {"status": "ok", "message": "Đã xóa bộ nhớ bản đồ vật cản."}


@router.post("/nav/cancel")
async def cancel_nav_goal():
    bridge = get_bridge()
    success = await bridge.cancel_nav_goal()
    return {"status": "ok" if success else "error", "message": "Đã hủy mục tiêu tự hành và dừng xe."}


@router.get("/nav/status")
async def get_nav_status():
    bridge = get_bridge()
    return {"status": "ok", "navigation": bridge.get_nav_status()}


@router.get("/camera/stream")
async def camera_mjpeg_stream(type: str = "rgb"):
    """MJPEG camera stream cho frontend (hỗ trợ type='rgb' hoặc 'depth')."""
    bridge = get_bridge()

    async def frame_generator():
        while True:
            frame_bytes = bridge.get_latest_camera_frame(camera_type=type)
            if frame_bytes:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n"
                )
                await asyncio.sleep(0.066)  # ~15 FPS
            else:
                placeholder = _get_placeholder_jpeg("RGB" if type == "rgb" else "Depth")
                if placeholder:
                    yield (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n\r\n" + placeholder + b"\r\n"
                    )
                await asyncio.sleep(0.5)

    return StreamingResponse(
        frame_generator(),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )

