"""
Calibration Service: Lifecycle management, parameter persistence, and telemetry summary.
Implements PROJECT.md § Interface Contracts & § Code Layout.
"""
from pathlib import Path
import os
import uuid
import yaml
import logging
from typing import Dict, Any, Optional

logger = logging.getLogger("amr_web.calib_service")

DEFAULT_CONFIG_DIR = Path(os.environ.get("AMR_CONFIG_DIR", "/home/sonev/amr_omni/config"))

_calib_state_ref: Optional[Dict[str, Any]] = None


def register_calib_state(state: Dict[str, Any]) -> None:
    """Register reference to calibration state dictionary."""
    global _calib_state_ref
    _calib_state_ref = state


def get_calib_telemetry_summary() -> Dict[str, Any]:
    """Return compact calibration state summary for telemetry broadcast."""
    if _calib_state_ref is None:
        return {
            "active": False,
            "stage": "idle",
            "progress_percent": 0,
            "status": "idle",
            "is_calibrated": False,
        }
    return {
        "active": bool(_calib_state_ref.get("active", False)),
        "stage": str(_calib_state_ref.get("stage", "idle")),
        "progress_percent": int(_calib_state_ref.get("progress_percent", 0)),
        "status": str(_calib_state_ref.get("status", "idle")),
        "is_calibrated": bool(_calib_state_ref.get("is_calibrated", False)),
    }


def get_config_dir(config_dir: Optional[Path | str] = None) -> Path:
    """Resolve config directory dynamically from parameter or environment."""
    if config_dir is not None:
        return Path(config_dir)
    return Path(os.environ.get("AMR_CONFIG_DIR", "/home/sonev/amr_omni/config"))


def save_imu_calib_yaml(
    results: Dict[str, Any],
    config_dir: Optional[Path | str] = None,
    frame_id: str = "imu_link"
) -> Path:
    """
    Atomically persist IMU calibration parameters to config/imu_calib.yaml.
    Matches schema in PROJECT.md:
      imu_calib:
        gyro_bias: [bx, by, bz]
        accel_scale: [sx, sy, sz]
        accel_bias: [ax, ay, az]
        frame_id: "imu_link"
    """
    target_dir = get_config_dir(config_dir)
    target_dir.mkdir(parents=True, exist_ok=True)
    target_file = target_dir / "imu_calib.yaml"
    tmp_file = target_dir / f"{target_file.name}.tmp.{uuid.uuid4().hex}"

    gyro_bias = [float(v) for v in results.get("gyro_bias", [0.0, 0.0, 0.0])]
    accel_scale = [float(v) for v in results.get("accel_scale", [1.0, 1.0, 1.0])]
    accel_bias = [float(v) for v in results.get("accel_bias", [0.0, 0.0, 0.0])]

    payload = {
        "imu_calib": {
            "gyro_bias": gyro_bias,
            "accel_scale": accel_scale,
            "accel_bias": accel_bias,
            "frame_id": str(results.get("frame_id", frame_id)),
        }
    }

    try:
        with open(tmp_file, "w", encoding="utf-8") as f:
            yaml.safe_dump(payload, f, default_flow_style=False, sort_keys=False)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp_file, target_file)
    finally:
        if tmp_file.exists():
            try:
                tmp_file.unlink()
            except OSError:
                pass
    logger.info(f"Atomically saved IMU calibration to {target_file}")
    return target_file


def save_wheel_calib_yaml(
    results: Dict[str, Any],
    config_dir: Optional[Path | str] = None,
    wheelbase: float = 0.1312,
    track_width: float = 0.1312
) -> Path:
    """
    Atomically persist wheel calibration parameters to config/wheel_calib.yaml.
    Matches schema in PROJECT.md:
      wheel_calib:
        wheel_radius: [r1, r2, r3, r4]
        wheelbase: 0.1312
        track_width: 0.1312
    """
    target_dir = get_config_dir(config_dir)
    target_dir.mkdir(parents=True, exist_ok=True)
    target_file = target_dir / "wheel_calib.yaml"
    tmp_file = target_dir / f"{target_file.name}.tmp.{uuid.uuid4().hex}"

    radii_raw = results.get("wheel_radius", results.get("wheel_radii", [0.03, 0.03, 0.03, 0.03]))
    radii = [float(r) for r in radii_raw]
    wb = float(results.get("wheelbase", wheelbase))
    tw = float(results.get("track_width", track_width))

    payload = {
        "wheel_calib": {
            "wheel_radius": radii,
            "wheelbase": wb,
            "track_width": tw,
        }
    }

    try:
        with open(tmp_file, "w", encoding="utf-8") as f:
            yaml.safe_dump(payload, f, default_flow_style=False, sort_keys=False)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp_file, target_file)
    finally:
        if tmp_file.exists():
            try:
                tmp_file.unlink()
            except OSError:
                pass
    logger.info(f"Atomically saved wheel calibration to {target_file}")
    return target_file


def persist_calibration_yaml(
    results: Dict[str, Any],
    config_dir: Optional[Path | str] = None
) -> Dict[str, str]:
    """Persist all available calibration outputs to disk and return paths."""
    saved_files = {}
    if "gyro_bias" in results or "accel_scale" in results or "accel_bias" in results:
        imu_path = save_imu_calib_yaml(results, config_dir=config_dir)
        saved_files["imu_calib"] = str(imu_path)

    if "wheel_radius" in results or "wheel_radii" in results:
        wheel_path = save_wheel_calib_yaml(results, config_dir=config_dir)
        saved_files["wheel_calib"] = str(wheel_path)

    return saved_files
