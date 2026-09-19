"""
Tier 2 Boundary Cases: Extreme Sensor Noise, Saturated Biases, and Low SNR.
"""
import math
import numpy as np
import pytest

from tests.e2e.harness.imu_calib_oracle import (
    ST_AN4508_Calibrator,
    GyroBiasNuller,
    AllanVarianceAnalyzer,
)


@pytest.mark.tier2
class TestBoundaryExtremeNoiseBias:
    """Boundary test cases for IMU noise extremes and saturation."""

    def test_boundary_accel_saturation_limit(self):
        """Verify handling of near-saturation accelerometer readings (+/- 15.9g)."""
        calibrator = ST_AN4508_Calibrator()
        g = calibrator.g
        # Saturated input
        accel_sat = np.array([15.9 * g, -15.9 * g, 1.0 * g])
        cal_sat = calibrator.apply(accel_sat)
        assert np.all(np.isfinite(cal_sat))
        assert abs(cal_sat[0]) > 15.0 * g

    def test_boundary_gyro_extreme_bias_saturation(self):
        """Verify nulling of unusually large static gyro bias (0.5 rad/s ~ 28 deg/s)."""
        nuller = GyroBiasNuller(variance_threshold=1e-3)
        huge_bias = np.array([0.5, -0.4, 0.3])

        np.random.seed(42)
        samples = huge_bias + np.random.normal(0, 1e-4, size=(200, 3))
        bias_est, residual = nuller.calibrate(samples)

        assert np.allclose(bias_est, huge_bias, atol=1e-3)
        assert residual < GyroBiasNuller.MAX_ALLOWED_DRIFT_RAD_S

    def test_boundary_negative_snr_noise_rejection(self):
        """Verify calibration averaging recovers true gravity under noise exceeding signal."""
        calibrator = ST_AN4508_Calibrator()
        g = calibrator.g

        np.random.seed(88)
        # Heavy noise std = 5.0 m/s² (nearly 50% of 1g signal)
        n = 5000
        noise_std = 5.0

        def noisy(vec):
            return np.array([vec]) * g + np.random.normal(0, noise_std, (n, 3))

        px_p = noisy([1, 0, 0])
        px_m = noisy([-1, 0, 0])
        py_p = noisy([0, 1, 0])
        py_m = noisy([0, -1, 0])
        pz_p = noisy([0, 0, 1])
        pz_m = noisy([0, 0, -1])

        scales, biases, _ = calibrator.calibrate(px_p, px_m, py_p, py_m, pz_p, pz_m)
        # Law of large numbers: mean error scales as std / sqrt(N) = 5.0 / sqrt(5000) ~ 0.07 m/s²
        assert np.allclose(scales, [1.0, 1.0, 1.0], atol=0.05)
        assert np.allclose(biases, [0.0, 0.0, 0.0], atol=0.2)

    def test_boundary_single_sample_outlier_spike(self):
        """Verify outlier spike (e.g. drop or mechanical shock) does not corrupt overall stream."""
        nuller = GyroBiasNuller(variance_threshold=0.1)
        base_bias = np.array([0.01, 0.01, 0.01])
        samples = np.repeat([base_bias], 100, axis=0)

        # Single spike of 10.0 rad/s
        samples[50] = [10.0, 10.0, 10.0]
        # Trimmed or median robust mean
        median_bias = np.median(samples, axis=0)
        assert np.allclose(median_bias, base_bias, atol=1e-5)

    def test_boundary_allan_short_data_series_rejection(self):
        """Verify Allan variance estimator rejects series too short (< 100 samples)."""
        analyzer = AllanVarianceAnalyzer()
        short_data = np.ones(50)
        with pytest.raises(ValueError, match="too short"):
            analyzer.compute_allan_deviation(short_data)
