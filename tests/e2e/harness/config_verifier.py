"""
Authoritative Configuration & Single TF Authority Verifier.
Validates ROS 2 YAML configuration files against contracts specified in PROJECT.md.
"""
from pathlib import Path
import yaml
import numpy as np


class ConfigVerifier:
    """Verifies system YAML configs against specifications in PROJECT.md."""

    def __init__(self, workspace_root: Path | str):
        self.root = Path(workspace_root)

    def load_yaml(self, rel_path: str) -> dict:
        """Load and parse YAML file relative to workspace root."""
        file_path = self.root / rel_path
        if not file_path.exists():
            raise FileNotFoundError(f"Configuration file not found: {file_path}")
        with open(file_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        return data or {}

    def verify_ekf_config(self, rel_path: str = "src/omni_localization/config/ekf.yaml") -> dict:
        """
        Verify ekf.yaml satisfies:
          - publish_tf: true
          - odom0_config: vx, vy, wz enabled
          - imu0_config: yaw, wz, ax, ay enabled
          - process_noise_covariance: 15x15 all diagonal elements strictly positive
          - initial_estimate_covariance: 15x15 all diagonal elements strictly positive
        """
        data = self.load_yaml(rel_path)
        params = data.get("ekf_filter_node", {}).get("ros__parameters", {})

        results = {
            "publish_tf": params.get("publish_tf") is True,
            "odom0_config_valid": False,
            "imu0_config_valid": False,
            "process_noise_diag_positive": False,
            "initial_estimate_diag_positive": False,
            "missing_fields": [],
        }

        # Check odom0_config
        odom0_cfg = params.get("odom0_config", [])
        if len(odom0_cfg) == 15:
            # Indices: vx=6, vy=7, wz=11
            if odom0_cfg[6] and odom0_cfg[7] and odom0_cfg[11]:
                results["odom0_config_valid"] = True

        # Check imu0_config
        imu0_cfg = params.get("imu0_config", [])
        if len(imu0_cfg) == 15:
            # Indices: yaw=5, wz=11, ax=12, ay=13
            if imu0_cfg[5] and imu0_cfg[11] and imu0_cfg[12] and imu0_cfg[13]:
                results["imu0_config_valid"] = True

        # Check process_noise_covariance
        p_noise = params.get("process_noise_covariance")
        if p_noise and len(p_noise) == 225:
            arr = np.array(p_noise).reshape(15, 15)
            diag = np.diag(arr)
            if np.all(diag > 0.0):
                results["process_noise_diag_positive"] = True
        elif not p_noise:
            results["missing_fields"].append("process_noise_covariance")

        # Check initial_estimate_covariance
        init_cov = params.get("initial_estimate_covariance")
        if init_cov and len(init_cov) == 225:
            arr = np.array(init_cov).reshape(15, 15)
            diag = np.diag(arr)
            if np.all(diag > 0.0):
                results["initial_estimate_diag_positive"] = True
        elif not init_cov:
            results["missing_fields"].append("initial_estimate_covariance")

        return results

    def verify_laser_filter_config(self, rel_path: str = "src/omni_perception/config/laser_filter.yaml") -> dict:
        """
        Verify laser_filter.yaml satisfies:
          - Footprint box filter limits: min_x: -0.135, max_x: 0.135, min_y: -0.135, max_y: 0.135
        """
        data = self.load_yaml(rel_path)
        chain = data.get("scan_to_scan_filter_chain", {}).get("ros__parameters", {})

        box_params = {}
        for k, v in chain.items():
            if isinstance(v, dict) and v.get("type") == "laser_filters/LaserScanBoxFilter":
                box_params = v.get("params", {})
                break

        min_x = box_params.get("min_x")
        max_x = box_params.get("max_x")
        min_y = box_params.get("min_y")
        max_y = box_params.get("max_y")
        box_frame = box_params.get("box_frame")

        return {
            "has_box_filter": bool(box_params),
            "box_frame_is_base_link": (box_frame == "base_link"),
            "min_x": min_x,
            "max_x": max_x,
            "min_y": min_y,
            "max_y": max_y,
            "is_calibrated_0135": (
                min_x == -0.135 and max_x == 0.135 and
                min_y == -0.135 and max_y == 0.135
            ),
        }

    def verify_imu_calib_yaml(self, file_path: Path | str) -> bool:
        """Validate imu_calib.yaml format."""
        p = Path(file_path)
        if not p.exists():
            return False
        with open(p, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        calib = data.get("imu_calib", {})
        if not ("gyro_bias" in calib and "accel_scale" in calib and "accel_bias" in calib):
            return False
        if len(calib["gyro_bias"]) != 3 or len(calib["accel_scale"]) != 3 or len(calib["accel_bias"]) != 3:
            return False
        return True

    def verify_wheel_calib_yaml(self, file_path: Path | str) -> bool:
        """Validate wheel_calib.yaml format."""
        p = Path(file_path)
        if not p.exists():
            return False
        with open(p, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        calib = data.get("wheel_calib", {})
        if not ("wheel_radius" in calib and "wheelbase" in calib and "track_width" in calib):
            return False
        if len(calib["wheel_radius"]) != 4:
            return False
        return True
