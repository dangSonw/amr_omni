"""
Tier 1 Feature Coverage: Encoder Estimation & Mecanum Kinematics.
Covers:
  - F1.1 ODrive 2nd-Order PLL Tracking Observer (5 tests)
  - F1.2 LinuxCNC M/T Hybrid Velocity Estimation (5 tests)
  - F1.3 Kinematics Consistency & Kr Radius Compensation (5 tests)
"""
import math
import numpy as np
import pytest

from tests.e2e.harness.encoder_oracle import ODrivePLLObserver, LinuxCNCHybridEstimator
from tests.e2e.harness.kinematics_oracle import MecanumKinematicsOracle


@pytest.mark.tier1
class TestF11_ODrivePLLObserver:
    """F1.1: ODrive 2nd-Order PLL discrete-time tracking observer."""

    def test_f1_1_pll_constant_velocity_tracking(self):
        """Verify PLL tracks a constant velocity ramp position without steady-state lag."""
        omega_target = 10.0  # rad/s
        dt = 0.001          # 1 kHz loop
        pll = ODrivePLLObserver(omega_pll=100.0)

        # Simulate 0.5s ramp
        pos = 0.0
        for _ in range(500):
            pos += omega_target * dt
            pll.update(pos, dt)

        # Steady-state velocity should be within 0.1% of target
        assert math.isclose(pll.vel_estimate, omega_target, rel_tol=1e-3)
        assert math.isclose(pll.pos_estimate, pos, abs_tol=0.015)

    def test_f1_1_pll_quantization_smoothing(self):
        """Verify PLL eliminates discrete quantization chatter from integer encoder counts."""
        dt = 0.002
        cpr = 4000
        true_vel = 0.5  # rad/s (slow speed)
        pll = ODrivePLLObserver(omega_pll=50.0)

        raw_diff_velocities = []
        pll_velocities = []

        continuous_pos = 0.0
        last_quantized = 0

        for _ in range(300):
            continuous_pos += true_vel * dt
            # Quantize to discrete encoder pulses
            quantized_counts = round((continuous_pos / (2.0 * math.pi)) * cpr)
            quantized_pos = (quantized_counts / cpr) * (2.0 * math.pi)

            # Raw difference velocity (chattery)
            raw_v = (quantized_counts - last_quantized) * (2.0 * math.pi / cpr) / dt
            last_quantized = quantized_counts
            raw_diff_velocities.append(raw_v)

            # PLL estimated velocity
            _, vel_est = pll.update(quantized_pos, dt)
            pll_velocities.append(vel_est)

        # Skip initial convergence transient
        pll_std = np.std(pll_velocities[100:])
        raw_std = np.std(raw_diff_velocities[100:])

        # PLL must have significantly lower variance than raw difference
        assert pll_std < raw_std * 0.2
        assert abs(np.mean(pll_velocities[150:]) - true_vel) < 0.05

    def test_f1_1_pll_step_acceleration_convergence(self):
        """Verify PLL converges following a sharp acceleration step without diverging."""
        pll = ODrivePLLObserver(omega_pll=80.0)
        dt = 0.001

        # Step acceleration: v increases from 0 to 20 rad/s over 50 ms
        pos = 0.0
        vel = 0.0
        for step in range(500):
            if step < 50:
                vel += (20.0 / 50)  # ramp acceleration
            pos += vel * dt
            pll.update(pos, dt)

        # After ramp finishes, velocity must converge to 20.0
        assert math.isclose(pll.vel_estimate, 20.0, rel_tol=5e-3)

    def test_f1_1_pll_bandwidth_tuning(self):
        """Verify higher PLL bandwidth results in faster rise time to step input."""
        dt = 0.001
        pll_fast = ODrivePLLObserver(omega_pll=150.0)
        pll_slow = ODrivePLLObserver(omega_pll=30.0)

        pos = 0.0
        fast_reach_time = None
        slow_reach_time = None

        target_v = 5.0
        for i in range(300):
            t = i * dt
            pos += target_v * dt
            _, v_fast = pll_fast.update(pos, dt)
            _, v_slow = pll_slow.update(pos, dt)

            if fast_reach_time is None and v_fast >= 0.9 * target_v:
                fast_reach_time = t
            if slow_reach_time is None and v_slow >= 0.9 * target_v:
                slow_reach_time = t

        assert fast_reach_time is not None
        assert slow_reach_time is not None
        assert fast_reach_time < slow_reach_time

    def test_f1_1_pll_zero_steady_state_error(self):
        """Verify theoretical zero steady-state error on constant speed position ramps."""
        pll = ODrivePLLObserver(omega_pll=120.0)
        dt = 0.0005
        vel_cmd = 15.0

        pos = 0.0
        for _ in range(1000):
            pos += vel_cmd * dt
            pos_est, vel_est = pll.update(pos, dt)

        assert abs(vel_est - vel_cmd) < 1e-4
        assert abs(pos_est - pos) < 0.01


@pytest.mark.tier1
class TestF12_LinuxCNCHybridEstimator:
    """F1.2: LinuxCNC M/T Hybrid Velocity Estimation."""

    def test_f1_2_linuxcnc_low_speed_t_method(self):
        """Verify accurate velocity estimation when pulse rate is low (T-method regime)."""
        estimator = LinuxCNCHybridEstimator(cpr=4000, f_clk=84_000_000.0)
        # 1 pulse over 0.01 seconds -> delta_m = 1, N_ticks = 840,000
        delta_m = 1
        n_ticks = 840000  # exactly 10 ms at 84 MHz
        expected_vel = (2.0 * math.pi * 1) / (4000 * 0.01)  # ~0.1570796 rad/s

        estimated_vel = estimator.calculate_velocity(delta_m, n_ticks)
        assert math.isclose(estimated_vel, expected_vel, rel_tol=1e-5)

    def test_f1_2_linuxcnc_high_speed_m_method(self):
        """Verify accurate velocity estimation under high pulse rates (M-method regime)."""
        estimator = LinuxCNCHybridEstimator(cpr=4000, f_clk=84_000_000.0)
        # 200 pulses in 10 ms -> delta_m = 200, N_ticks = 840,000
        delta_m = 200
        n_ticks = 840000
        expected_vel = (2.0 * math.pi * 200) / (4000 * 0.01)  # ~31.4159 rad/s

        estimated_vel = estimator.calculate_velocity(delta_m, n_ticks)
        assert math.isclose(estimated_vel, expected_vel, rel_tol=1e-5)

    def test_f1_2_linuxcnc_16bit_timer_overflow(self):
        """Verify safe 16-bit timer counter rollover subtraction across boundary."""
        estimator = LinuxCNCHybridEstimator()
        # Case A: Forward overflow from 65530 to 5 -> +11 counts
        diff_fwd = estimator.safe_timer_rollover_diff(current_cnt=5, last_cnt=65530)
        assert diff_fwd == 11

        # Case B: Backward underflow from 5 to 65530 -> -11 counts
        diff_rev = estimator.safe_timer_rollover_diff(current_cnt=65530, last_cnt=5)
        assert diff_rev == -11

    def test_f1_2_linuxcnc_zero_speed_watchdog_timeout(self):
        """Verify zero-speed watchdog resets velocity to 0 when pulse edges stop."""
        estimator = LinuxCNCHybridEstimator(watchdog_timeout_s=0.03)
        # Motor was moving: delta_m = 10
        v = estimator.update_sample(current_pulse_cnt=100, last_pulse_cnt=90,
                                   current_timer_cnt=840000, last_timer_cnt=0,
                                   dt_sample=0.01)
        assert v > 0.0

        # Motor stops: delta_m = 0, dt_sample exceeds watchdog threshold (0.05s > 0.03s)
        v_stopped = estimator.update_sample(current_pulse_cnt=100, last_pulse_cnt=100,
                                           current_timer_cnt=1680000, last_timer_cnt=840000,
                                           dt_sample=0.05)
        assert v_stopped == 0.0

    def test_f1_2_linuxcnc_bidirectional_reversal(self):
        """Verify smooth handling and sign accuracy during direction reversal."""
        estimator = LinuxCNCHybridEstimator(cpr=4000, f_clk=84_000_000.0)
        v_pos = estimator.calculate_velocity(delta_m=50, n_timer_ticks=840000)
        v_neg = estimator.calculate_velocity(delta_m=-50, n_timer_ticks=840000)

        assert v_pos > 0.0
        assert v_neg < 0.0
        assert math.isclose(v_pos, -v_neg, rel_tol=1e-9)


@pytest.mark.tier1
class TestF13_KinematicsConsistencyKr:
    """F1.3: Mecanum Kinematics Consistency & Kr Wheel Radius Compensation."""

    def test_f1_3_kinematics_roundtrip_consistency(self):
        """Verify FK(IK(v)) == v within 1e-5 error across arbitrary twists."""
        kinematics = MecanumKinematicsOracle(wheelbase_m=0.1312, track_width_m=0.1312)
        test_twists = [
            (0.5, 0.0, 0.0),
            (0.0, 0.5, 0.0),
            (0.0, 0.0, 1.0),
            (0.35, -0.25, 0.8),
            (-0.4, 0.3, -0.6),
        ]
        for vx, vy, wz in test_twists:
            err = kinematics.round_trip_error(vx, vy, wz)
            assert err < 1e-5, f"Round trip error {err} exceeds 1e-5 for twist {(vx, vy, wz)}"

    def test_f1_3_kinematics_pure_translation_x(self):
        """Verify pure forward translation produces equal forward wheel commands."""
        kinematics = MecanumKinematicsOracle()
        speeds = kinematics.inverse_kinematics(vx=0.5, vy=0.0, wz=0.0, apply_scaling=False)
        d = math.sqrt(0.5)
        expected = (0.5 * d) / 0.03  # ~11.785 rad/s
        # In Mecanum: Wheel 1 & 4 are +d, Wheel 2 & 3 are -d
        assert math.isclose(abs(speeds[0]), expected, rel_tol=1e-4)
        assert math.isclose(abs(speeds[1]), expected, rel_tol=1e-4)
        assert math.isclose(abs(speeds[2]), expected, rel_tol=1e-4)
        assert math.isclose(abs(speeds[3]), expected, rel_tol=1e-4)

    def test_f1_3_kinematics_pure_strafe_y(self):
        """Verify pure lateral strafing produces symmetric opposing wheel pairs."""
        kinematics = MecanumKinematicsOracle()
        speeds = kinematics.inverse_kinematics(vx=0.0, vy=0.4, wz=0.0, apply_scaling=False)
        # All speeds must have equal magnitude
        mags = [abs(s) for s in speeds]
        assert all(math.isclose(m, mags[0], rel_tol=1e-5) for m in mags)

        # Forward kinematics must reconstruct exactly (0, 0.4, 0)
        reconstructed = kinematics.forward_kinematics(speeds)
        assert math.isclose(reconstructed[0], 0.0, abs_tol=1e-5)
        assert math.isclose(reconstructed[1], 0.4, rel_tol=1e-5)
        assert math.isclose(reconstructed[2], 0.0, abs_tol=1e-5)

    def test_f1_3_kinematics_pure_rotation_z(self):
        """Verify pure yaw rotation drives all wheels with equal magnitude in rotation direction."""
        kinematics = MecanumKinematicsOracle()
        wz_cmd = 1.5  # rad/s
        speeds = kinematics.inverse_kinematics(vx=0.0, vy=0.0, wz=wz_cmd, apply_scaling=False)

        # For pure rotation, all 4 wheels spin in the same direction with equal speed
        assert math.isclose(speeds[0], speeds[1], rel_tol=1e-5)
        assert math.isclose(speeds[1], speeds[2], rel_tol=1e-5)
        assert math.isclose(speeds[2], speeds[3], rel_tol=1e-5)

        reconstructed = kinematics.forward_kinematics(speeds)
        assert math.isclose(reconstructed[2], wz_cmd, rel_tol=1e-5)

    def test_f1_3_kinematics_kr_individual_wheel_compensation(self):
        """Verify individual wheel radius errors Kr are compensated without round-trip loss."""
        # Non-uniform radii (e.g. 2% manufacturing tolerance / tire wear)
        radii = (0.0305, 0.0295, 0.0300, 0.0298)
        kinematics_kr = MecanumKinematicsOracle(wheel_radii=radii)

        twist = (0.3, 0.2, 0.5)
        speeds = kinematics_kr.inverse_kinematics(*twist, apply_scaling=False)
        reconstructed = kinematics_kr.forward_kinematics(speeds)

        for expected, actual in zip(twist, reconstructed):
            assert math.isclose(expected, actual, abs_tol=1e-5)
