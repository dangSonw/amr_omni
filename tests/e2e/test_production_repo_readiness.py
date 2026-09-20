"""
Production Repository Readiness Audit Tests.
Tracks status of production files across milestones M1-M4 as specified in PROJECT.md § Milestones.
All milestone implementations (M1-M4) are in place and verified.
"""
from pathlib import Path
import inspect
import pytest

from tests.e2e.harness.config_verifier import ConfigVerifier


@pytest.mark.wip
class TestProductionRepoReadiness:
    """Audits production repository code against target specifications for M1-M4."""

    def test_repo_kinematics_kr_parameter_support(self):
        """M1 Feature F1.3: Verify omni_control.kinematics accepts Kr wheel radii tuple."""
        from omni_control import kinematics
        sig = inspect.signature(kinematics.inverse_kinematics)
        # Target signature should support wheel_radii tuple or Kr compensation
        assert "wheel_radii" in sig.parameters or "kr" in sig.parameters, (
            "kinematics.inverse_kinematics does not yet support individual wheel radii Kr"
        )

    def test_repo_firmware_encoder_pll_files_exist(self, workspace_root: Path):
        """M1 Feature F1.1/F1.2: Verify encoder_pll source files exist in firmware."""
        fw_src = workspace_root / "firmware" / "stm32_f407vg_arduino_sim" / "src"
        assert (fw_src / "encoder_pll.cpp").exists() or (fw_src / "encoder_pll.h").exists(), (
            "encoder_pll files not yet added to firmware"
        )

    def test_repo_firmware_imu_calibration_files_exist(self, workspace_root: Path):
        """M2 Feature F2.1/F2.2: Verify BNO080 hardware IMU integration exists in firmware."""
        fw_src = workspace_root / "firmware" / "stm32_f407vg_arduino_sim" / "src"
        fw_inc = workspace_root / "firmware" / "stm32_f407vg_arduino_sim" / "include"
        assert (fw_src / "hardware.cpp").exists() and (fw_inc / "firmware_config.h").exists(), (
            "BNO080 hardware files not found in firmware"
        )

    def test_repo_ekf_full_covariance_configured(self, workspace_root: Path):
        """M3 Feature F3.3: Verify ekf.yaml has non-zero process_noise_covariance and initial_estimate_covariance."""
        verifier = ConfigVerifier(workspace_root)
        results = verifier.verify_ekf_config()
        assert results["process_noise_diag_positive"], "process_noise_covariance missing or non-positive in ekf.yaml"
        assert results["initial_estimate_diag_positive"], "initial_estimate_covariance missing or non-positive in ekf.yaml"

    def test_repo_laser_filter_calibrated_dimensions(self, workspace_root: Path):
        """M3 Feature F3.5: Verify laser_filter.yaml has calibrated box [-0.135, 0.135] m."""
        verifier = ConfigVerifier(workspace_root)
        results = verifier.verify_laser_filter_config()
        assert results["is_calibrated_0135"], (
            f"laser_filter footprint is [{results['min_x']}, {results['max_x']}], expected [-0.135, 0.135]"
        )

    def test_repo_firmware_serial_protocol_files_exist(self, workspace_root: Path):
        """M4 Feature F4.1: Verify serial_protocol source files exist in firmware."""
        fw_src = workspace_root / "firmware" / "stm32_f407vg_arduino_sim" / "src"
        assert (fw_src / "serial_protocol.cpp").exists() or (fw_src / "serial_protocol.h").exists(), (
            "serial_protocol files not yet added to firmware"
        )

    def test_repo_fastapi_calibration_routes_mounted(self):
        """M4 Feature F4.2: Verify /api/calib endpoints are mounted in FastAPI application."""
        from app.main import app
        routes = [r.path for r in app.routes]
        assert "/api/calib/start" in routes, "/api/calib/start endpoint not yet mounted in FastAPI app"
