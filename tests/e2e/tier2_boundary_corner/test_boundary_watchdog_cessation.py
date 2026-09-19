"""
Tier 2 Boundary Cases: Zero-Speed Watchdog and Pulse Cessation.
"""
import pytest

from tests.e2e.harness.encoder_oracle import LinuxCNCHybridEstimator


@pytest.mark.tier2
class TestBoundaryWatchdogCessation:
    """Boundary test cases for watchdog timeout when pulses cease."""

    def test_boundary_watchdog_exact_timeout_boundary(self):
        """Verify watchdog triggers exactly when elapsed time reaches threshold."""
        timeout = 0.040
        estimator = LinuxCNCHybridEstimator(watchdog_timeout_s=timeout)

        # First pulse interval: moving
        v1 = estimator.update_sample(10, 0, 100000, 0, dt_sample=0.010)
        assert v1 > 0.0

        # Sub-timeout quiet period (0.035s < 0.040s) holds last velocity
        v_sub = estimator.update_sample(10, 10, 200000, 100000, dt_sample=0.035)
        assert v_sub == v1

        # Timeout reached (0.041s > 0.040s) sets velocity to 0.0
        v_timeout = estimator.update_sample(10, 10, 300000, 200000, dt_sample=0.041)
        assert v_timeout == 0.0

    def test_boundary_watchdog_stray_noise_pulse_during_stop(self):
        """Verify single stray edge doesn't trigger false high velocity."""
        estimator = LinuxCNCHybridEstimator(cpr=4000, f_clk=84_000_000.0)
        # Motor stopped for 1 second, then single stray pulse (delta_m = 1) over 1s (84,000,000 ticks)
        vel = estimator.calculate_velocity(delta_m=1, n_timer_ticks=84_000_000)
        # 2*pi / 4000 = ~0.00157 rad/s (essentially zero)
        assert vel < 0.002

    def test_boundary_gradual_deceleration_to_stop(self):
        """Verify gradual deceleration smoothly approaches zero without hanging."""
        estimator = LinuxCNCHybridEstimator(watchdog_timeout_s=0.03)
        velocities = []

        # Decelerating pulse rates
        delta_counts = [50, 30, 15, 5, 1, 0]
        dt = 0.01
        for i, dm in enumerate(delta_counts):
            v = estimator.update_sample(
                current_pulse_cnt=sum(delta_counts[:i+1]),
                last_pulse_cnt=sum(delta_counts[:i]),
                current_timer_cnt=(i+1)*840000,
                last_timer_cnt=i*840000,
                dt_sample=dt if dm > 0 else 0.05
            )
            velocities.append(v)

        # Monotonically non-increasing trend to 0
        assert velocities[-1] == 0.0
        assert velocities[0] > velocities[1] > velocities[2]

    def test_boundary_long_standstill_hold(self):
        """Verify robot stopped for extended duration consistently outputs 0 velocity."""
        estimator = LinuxCNCHybridEstimator(watchdog_timeout_s=0.02)
        # Initial stop
        estimator.update_sample(0, 0, 0, 0, dt_sample=0.1)

        for _ in range(50):
            v = estimator.update_sample(0, 0, 0, 0, dt_sample=0.05)
            assert v == 0.0

    def test_boundary_high_frequency_dither_jitter(self):
        """Verify alternating jitter (+1, -1, +1, -1) averages to zero net motion."""
        estimator = LinuxCNCHybridEstimator(cpr=4000, f_clk=84_000_000.0)
        net_vel = 0.0
        for i in range(10):
            dm = 1 if (i % 2 == 0) else -1
            v = estimator.calculate_velocity(dm, 84000)
            net_vel += v

        assert abs(net_vel) < 1e-9
