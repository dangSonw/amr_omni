"""
Tier 2 Boundary Cases: Extreme Speeds and Dynamic Saturation.
"""
import math
import pytest

from tests.e2e.harness.encoder_oracle import ODrivePLLObserver, LinuxCNCHybridEstimator
from tests.e2e.harness.kinematics_oracle import MecanumKinematicsOracle


@pytest.mark.tier2
class TestBoundarySpeedsDynamics:
    """Boundary test cases for extreme velocities and dynamic limits."""

    def test_boundary_creeping_speed(self):
        """Verify tracking at extremely low creeping speed (0.001 m/s) below raw resolution."""
        pll = ODrivePLLObserver(omega_pll=40.0)
        dt = 0.002
        creeping_v = 0.002  # rad/s

        pos = 0.0
        for _ in range(1000):
            pos += creeping_v * dt
            pll.update(pos, dt)

        assert abs(pll.vel_estimate - creeping_v) < 0.001

    def test_boundary_linear_speed_saturation(self):
        """Verify kinematics scales down wheel velocities when exceeding max wheel speed."""
        kinematics = MecanumKinematicsOracle(max_wheel_speed_rad_s=50.0)
        # Extreme command vx = 3.0 m/s which would demand wheel speeds > 70 rad/s
        speeds = kinematics.inverse_kinematics(vx=3.0, vy=0.0, wz=0.0, apply_scaling=True)
        max_speed = max(abs(s) for s in speeds)
        assert max_speed <= 50.0 + 1e-6

    def test_boundary_extreme_angular_velocity(self):
        """Verify kinematics consistency under rapid spinning (10.0 rad/s)."""
        kinematics = MecanumKinematicsOracle(max_wheel_speed_rad_s=200.0)
        wz_extreme = 10.0
        err = kinematics.round_trip_error(0.0, 0.0, wz_extreme)
        assert err < 1e-5

    def test_boundary_simultaneous_3axis_extremes(self):
        """Verify simultaneous extreme multi-axis commands do not result in NaN or division by zero."""
        kinematics = MecanumKinematicsOracle(max_wheel_speed_rad_s=100.0)
        speeds = kinematics.inverse_kinematics(vx=2.5, vy=2.5, wz=5.0, apply_scaling=True)
        assert all(math.isfinite(s) for s in speeds)
        assert max(abs(s) for s in speeds) <= 100.0 + 1e-6

    def test_boundary_step_velocity_reversal(self):
        """Verify instantaneous direction reversal (+1.5 m/s to -1.5 m/s) does not diverge."""
        pll = ODrivePLLObserver(omega_pll=100.0)
        dt = 0.001
        pos = 0.0

        # Run forward at 20 rad/s for 300 ms
        for _ in range(300):
            pos += 20.0 * dt
            pll.update(pos, dt)
        assert pll.vel_estimate > 15.0

        # Step reverse to -20 rad/s for 300 ms
        for _ in range(300):
            pos -= 20.0 * dt
            pll.update(pos, dt)

        assert pll.vel_estimate < -15.0
        assert math.isfinite(pll.vel_estimate)
