"""
Tier 2 Boundary Cases: Mathematical Singularities and Degenerate Geometries.
"""
import numpy as np
import pytest

from tests.e2e.harness.kinematics_oracle import MecanumKinematicsOracle
from tests.e2e.harness.imu_calib_oracle import ST_AN4508_Calibrator


@pytest.mark.tier2
class TestBoundaryMathSingularities:
    """Boundary test cases for mathematical condition numbers, singularities, and degenerate inputs."""

    def test_boundary_zero_wheel_radius_rejected(self):
        """Verify zero or negative wheel radius raises ValueError."""
        with pytest.raises(ValueError, match="Wheel radii must be positive"):
            MecanumKinematicsOracle(wheel_radii=(0.03, 0.0, 0.03, 0.03))

    def test_boundary_zero_wheelbase_trackwidth_rejected(self):
        """Verify zero wheelbase or track width raises ValueError."""
        with pytest.raises(ValueError, match="Wheelbase and track width must be positive"):
            MecanumKinematicsOracle(wheelbase_m=0.0, track_width_m=0.1312)

    def test_boundary_nan_inf_twist_rejected(self):
        """Verify NaN or Inf twist commands raise ValueError."""
        kinematics = MecanumKinematicsOracle()
        with pytest.raises(ValueError, match="Twist velocities must be finite"):
            kinematics.inverse_kinematics(float("nan"), 0.0, 0.0)

        with pytest.raises(ValueError, match="Twist velocities must be finite"):
            kinematics.inverse_kinematics(0.0, float("inf"), 0.0)

    def test_boundary_high_wheel_radius_asymmetry(self):
        """Verify highly asymmetrical wheel radii (e.g. 50% wear differential) preserve consistency."""
        extreme_radii = (0.020, 0.040, 0.025, 0.035)
        kinematics = MecanumKinematicsOracle(wheel_radii=extreme_radii)

        err = kinematics.round_trip_error(0.3, -0.2, 0.4)
        assert err < 1e-5

    def test_boundary_degenerate_accelerometer_data_rejected(self):
        """Verify degenerate calibration data (zero difference between +g and -g) is detected."""
        calibrator = ST_AN4508_Calibrator()
        # Same value for +X and -X (sensor dead on X axis)
        px_p = np.array([[0.0, 0.0, 0.0]])
        px_m = np.array([[0.0, 0.0, 0.0]])
        py_p = np.array([[0.0, 9.8, 0.0]])
        py_m = np.array([[0.0, -9.8, 0.0]])
        pz_p = np.array([[0.0, 0.0, 9.8]])
        pz_m = np.array([[0.0, 0.0, -9.8]])

        with pytest.raises(ValueError, match="Degenerate measurements"):
            calibrator.calibrate(px_p, px_m, py_p, py_m, pz_p, pz_m)
