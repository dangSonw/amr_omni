from typing import List
from fastapi import APIRouter

from app.bridges import get_bridge
from app.models import (
    EStopCommand,
    RobotConfig,
    RobotStatus,
    StreamInfo,
    TwistCommand,
)
from app.services.stream_monitor import stream_monitor

router = APIRouter(prefix="/api", tags=["robot"])


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
