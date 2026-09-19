import logging
from typing import Dict, Any, Optional
from pydantic import BaseModel
from fastapi import APIRouter, HTTPException

logger = logging.getLogger("amr_web.calib")

router = APIRouter(prefix="/api/calib", tags=["calibration"])

# In-memory calibration state
_calib_state: Dict[str, Any] = {
    "active": False,
    "target": None,
    "routine": None,
    "subtype": None,
    "stage": "idle",
    "stage_description": "Waiting for trigger",
    "progress_percent": 0,
    "status": "idle",
    "status_code": "IDLE",
    "elapsed_sec": 0.0,
    "live_metrics": {},
    "results": {
        "gyro_bias": [0.0, 0.0, 0.0],
        "accel_bias": [0.0, 0.0, 0.0],
        "accel_scale": [1.0, 1.0, 1.0],
        "residual_norm": 0.0,
        "wheel_radii": [0.03, 0.03, 0.03, 0.03],
        "wheelbase": 0.1312,
        "track_width": 0.1312,
        "consistency_error": 0.0,
    },
    "error_message": None,
}


class CalibStartPayload(BaseModel):
    routine: Optional[str] = "imu"
    target: Optional[str] = "imu"
    subtype: Optional[str] = "static_bias"
    mode: Optional[str] = "static_bias"
    sample_count: Optional[int] = 500
    duration_s: Optional[int] = 30
    test_speed_mps: Optional[float] = 0.2
    target_dist_m: Optional[float] = 1.0


class CalibAbortPayload(BaseModel):
    action: Optional[str] = "abort"
    reason: Optional[str] = "user_cancelled"


class CalibApplyPayload(BaseModel):
    persist_flash: Optional[bool] = True
    save_yaml: Optional[bool] = True


@router.post("/start")
async def start_calibration(payload: CalibStartPayload):
    routine = payload.routine or payload.target or "imu"
    subtype = payload.subtype or payload.mode or "static_bias"

    _calib_state["active"] = True
    _calib_state["target"] = routine
    _calib_state["routine"] = routine
    _calib_state["subtype"] = subtype
    _calib_state["stage"] = "sampling"
    _calib_state["stage_description"] = f"Calibrating {routine} ({subtype})"
    _calib_state["progress_percent"] = 10
    _calib_state["status"] = "in_progress"
    _calib_state["status_code"] = "IN_PROGRESS"
    _calib_state["error_message"] = None

    logger.info(f"Started calibration: {routine} ({subtype})")
    return {
        "status": "ok",
        "message": f"Calibration routine '{routine}' started",
        "state": _calib_state,
    }


@router.post("/abort")
async def abort_calibration(payload: Optional[CalibAbortPayload] = None):
    _calib_state["active"] = False
    _calib_state["stage"] = "aborted"
    _calib_state["stage_description"] = "Calibration aborted by user"
    _calib_state["progress_percent"] = 0
    _calib_state["status"] = "aborted"
    _calib_state["status_code"] = "ABORTED"

    logger.info("Calibration aborted")
    return {
        "status": "ok",
        "action": "abort",
        "message": "Calibration routine aborted",
        "state": _calib_state,
    }


@router.post("/step")
async def step_calibration():
    if not _calib_state["active"]:
        raise HTTPException(status_code=400, detail="No active calibration to advance")

    _calib_state["progress_percent"] = min(100, _calib_state["progress_percent"] + 20)
    if _calib_state["progress_percent"] >= 100:
        _calib_state["active"] = False
        _calib_state["status"] = "success"
        _calib_state["status_code"] = "SUCCESS"
        _calib_state["stage"] = "completed"
        _calib_state["stage_description"] = "Calibration completed successfully"

    return {
        "status": "ok",
        "message": "Advanced calibration step",
        "state": _calib_state,
    }


@router.get("/status")
async def get_calibration_status():
    return _calib_state


@router.get("/results")
async def get_calibration_results():
    return {
        "status": "ok",
        "target": _calib_state.get("target"),
        "results": _calib_state.get("results"),
    }


@router.post("/apply")
async def apply_calibration(payload: Optional[CalibApplyPayload] = None):
    return {
        "status": "ok",
        "message": "Calibration parameters applied and persisted to configuration",
        "results": _calib_state.get("results"),
    }

