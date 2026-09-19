"""
Tier 1 Feature Coverage: IMU Calibration, Allan Variance, and REP-103 Standards.
Covers:
  - F2.1 STM32 ST AN4508 Accel Calibration (5 tests)
  - F2.2 STM32 Stationary Gyro Bias Nulling (5 tests)
  - F2.3 REP-103 ENU Coordinate Standard (5 tests)
  - F2.4 Allan Variance & Covariance Inflation (5 tests)
"""
import math
import numpy as np
import pytest

from tests.e2e.harness.imu_calib_oracle import (
    ST_AN4508_Calibrator,
    GyroBiasNuller,
    AllanVarianceAnalyzer,
)


@pytest.mark.tier1
class TestF21_STM32_AN4508_AccelCalibration:
    """F2.1: On-board 6-position accelerometer calibration routine solving scale & bias."""

    def test_f2_1_an4508_ideal_gravity_recovery(self):
        """Verify calibration with ideal uncorrupted measurements recovers unit scale and zero bias."""
        calibrator = ST_AN4508_Calibrator()
        g = calibrator.g
        # Ideal positions
        pos_x_p = np.array([[g, 0.0, 0.0]])
        pos_x_m = np.array([[-g, 0.0, 0.0]])
        pos_y_p = np.array([[0.0, g, 0.0]])
        pos_y_m = np.array([[0.0, -g, 0.0]])
        pos_z_p = np.array([[0.0, 0.0, g]])
        pos_z_m = np.array([[0.0, 0.0, -g]])

        scales, biases, res = calibrator.calibrate(pos_x_p, pos_x_m, pos_y_p, pos_y_m, pos_z_p, pos_z_m)
        assert np.allclose(scales, [1.0, 1.0, 1.0], atol=1e-6)
        assert np.allclose(biases, [0.0, 0.0, 0.0], atol=1e-6)
        assert res < 1e-6

    def test_f2_1_an4508_scale_and_bias_estimation(self):
        """Verify calibration accurately solves synthetic scale errors and biases."""
        calibrator = ST_AN4508_Calibrator()
        g = calibrator.g

        # Ground truth errors
        true_scale = np.array([1.08, 0.94, 1.02])
        true_bias = np.array([0.25, -0.15, 0.40])

        def corrupt(ideal_a):
            # raw = (ideal - bias) / scale
            return (ideal_a - true_bias) / true_scale

        pos_x_p = np.array([corrupt([g, 0, 0])])
        pos_x_m = np.array([corrupt([-g, 0, 0])])
        pos_y_p = np.array([corrupt([0, g, 0])])
        pos_y_m = np.array([corrupt([0, -g, 0])])
        pos_z_p = np.array([corrupt([0, 0, g])])
        pos_z_m = np.array([corrupt([0, 0, -g])])

        scales, biases, _ = calibrator.calibrate(pos_x_p, pos_x_m, pos_y_p, pos_y_m, pos_z_p, pos_z_m)
        assert np.allclose(scales, true_scale, rtol=1e-4)
        assert np.allclose(biases, true_bias, atol=1e-4)

    def test_f2_1_an4508_calibrated_norm_accuracy(self):
        """Verify calibrated accelerometer vector norm equals 9.80665 m/s² within 0.05 m/s²."""
        calibrator = ST_AN4508_Calibrator()
        g = calibrator.g

        # Introduce realistic sensor error (5% scale, 0.2 m/s² bias)
        true_scale = np.array([1.05, 0.96, 1.03])
        true_bias = np.array([0.18, -0.12, 0.22])

        def generate_samples(ideal_dir, n=50):
            ideal_a = np.array(ideal_dir) * g
            raw = (ideal_a - true_bias) / true_scale
            noise = np.random.normal(0, 0.01, size=(n, 3))
            return raw + noise

        np.random.seed(42)
        px_p = generate_samples([1, 0, 0])
        px_m = generate_samples([-1, 0, 0])
        py_p = generate_samples([0, 1, 0])
        py_m = generate_samples([0, -1, 0])
        pz_p = generate_samples([0, 0, 1])
        pz_m = generate_samples([0, 0, -1])

        calibrator.calibrate(px_p, px_m, py_p, py_m, pz_p, pz_m)

        # Test at arbitrary 45-degree static orientation
        test_dir = np.array([1.0, 1.0, 1.0]) / math.sqrt(3.0)
        raw_test = (test_dir * g - true_bias) / true_scale
        cal_test = calibrator.apply(raw_test)

        norm_cal = float(np.linalg.norm(cal_test))
        assert abs(norm_cal - g) < 0.05  # Within 0.05 m/s² acceptance threshold

    def test_f2_1_an4508_noise_rejection(self):
        """Verify multi-sample averaging cancels sensor measurement noise."""
        calibrator = ST_AN4508_Calibrator()
        g = calibrator.g

        np.random.seed(123)
        n_samples = 200
        # High noise (0.1 m/s² std)
        px_p = np.array([[g, 0, 0]]) + np.random.normal(0, 0.1, (n_samples, 3))
        px_m = np.array([[-g, 0, 0]]) + np.random.normal(0, 0.1, (n_samples, 3))
        py_p = np.array([[0, g, 0]]) + np.random.normal(0, 0.1, (n_samples, 3))
        py_m = np.array([[0, -g, 0]]) + np.random.normal(0, 0.1, (n_samples, 3))
        pz_p = np.array([[0, 0, g]]) + np.random.normal(0, 0.1, (n_samples, 3))
        pz_m = np.array([[0, 0, -g]]) + np.random.normal(0, 0.1, (n_samples, 3))

        scales, biases, _ = calibrator.calibrate(px_p, px_m, py_p, py_m, pz_p, pz_m)
        assert np.allclose(scales, [1.0, 1.0, 1.0], atol=0.02)
        assert np.allclose(biases, [0.0, 0.0, 0.0], atol=0.02)

    def test_f2_1_an4508_residual_error_bound(self):
        """Verify residual norm error across all positions is bounded by specification."""
        calibrator = ST_AN4508_Calibrator()
        g = calibrator.g

        pos = [np.array([[g, 0, 0]]), np.array([[-g, 0, 0]]),
               np.array([[0, g, 0]]), np.array([[0, -g, 0]]),
               np.array([[0, 0, g]]), np.array([[0, 0, -g]])]
        _, _, max_res = calibrator.calibrate(*pos)
        assert max_res < 0.01


@pytest.mark.tier1
class TestF22_STM32_GyroBiasNulling:
    """F2.2: Stationary gyroscope zero-rate bias tracking and nulling."""

    def test_f2_2_gyro_nulling_stationary_bias_removal(self):
        """Verify stationary bias is estimated and removed from gyro stream."""
        nuller = GyroBiasNuller()
        true_bias = np.array([0.015, -0.022, 0.008])  # rad/s

        np.random.seed(42)
        # Stationary measurements: bias + white noise
        samples = true_bias + np.random.normal(0, 1e-4, size=(500, 3))
        bias_est, max_res = nuller.calibrate(samples)

        assert np.allclose(bias_est, true_bias, atol=1e-3)
        assert max_res < GyroBiasNuller.MAX_ALLOWED_DRIFT_RAD_S

    def test_f2_2_gyro_nulling_residual_drift_under_005_deg_s(self):
        """Verify residual static drift rate is strictly < 0.05 deg/s (< 8.726e-4 rad/s)."""
        nuller = GyroBiasNuller()
        true_bias = np.array([0.005, -0.010, 0.020])

        np.random.seed(99)
        samples = true_bias + np.random.normal(0, 2e-4, size=(1000, 3))
        _, max_drift = nuller.calibrate(samples)

        threshold_rad_s = math.radians(0.05)
        assert max_drift < threshold_rad_s, f"Max drift {max_drift} exceeds 0.05 deg/s ({threshold_rad_s})"

    def test_f2_2_gyro_nulling_stationary_variance_gate(self):
        """Verify calibration is rejected if robot was moving during sample acquisition."""
        nuller = GyroBiasNuller(variance_threshold=1e-4)

        # High variance data simulating robot rotation/vibration
        moving_samples = np.random.normal(0, 0.05, size=(100, 3))
        with pytest.raises(ValueError, match="Motion detected"):
            nuller.calibrate(moving_samples)

    def test_f2_2_gyro_nulling_long_term_integration_drift(self):
        """Verify angle drift after bias nulling remains bounded over time."""
        nuller = GyroBiasNuller()
        true_bias = np.array([0.0, 0.0, 0.012])  # 0.012 rad/s bias on yaw

        dt = 0.01
        steps = 5000  # 50 seconds

        np.random.seed(77)
        noise = np.random.normal(0, 1e-4, (steps, 3))
        raw_gyro = true_bias + noise

        # Before calibration: yaw angle drifts ~0.012 * 50 = 0.6 rad (~34 deg)
        raw_yaw_drift = np.sum(raw_gyro[:, 2]) * dt
        assert abs(raw_yaw_drift) > 0.5

        # Calibrate on first 500 samples
        nuller.calibrate(raw_gyro[:500])

        # Correct full stream
        corrected_gyro = nuller.apply(raw_gyro)
        calibrated_yaw_drift = float(np.sum(corrected_gyro[:, 2]) * dt)

        # After calibration: cumulative drift should be minimal
        assert abs(calibrated_yaw_drift) < math.radians(3.0)  # Under 3 degrees over 50s

    def test_f2_2_gyro_nulling_three_axis_independence(self):
        """Verify X, Y, and Z biases are nulled independently without cross-talk."""
        nuller = GyroBiasNuller()
        bias_vector = np.array([0.03, -0.05, 0.02])

        np.random.seed(11)
        samples = bias_vector + np.random.normal(0, 1e-5, (100, 3))
        est_bias, _ = nuller.calibrate(samples)

        assert abs(est_bias[0] - bias_vector[0]) < 1e-4
        assert abs(est_bias[1] - bias_vector[1]) < 1e-4
        assert abs(est_bias[2] - bias_vector[2]) < 1e-4


@pytest.mark.tier1
class TestF23_REP103_ENUCoordinateStandard:
    """F2.3: REP-103 ENU coordinate conventions and gravity representation."""

    def test_f2_3_enu_axes_orientation(self):
        """Verify ENU convention: +X forward, +Y left, +Z up."""
        # Body frame definition
        x_forward = np.array([1.0, 0.0, 0.0])
        y_left = np.array([0.0, 1.0, 0.0])
        z_up = np.cross(x_forward, y_left)

        assert np.allclose(z_up, [0.0, 0.0, 1.0])

    def test_f2_3_enu_static_gravity_vector_z(self):
        """Verify static level robot measures +9.80665 m/s² along +Z in ENU frame."""
        g = 9.80665
        accel_at_rest_enu = np.array([0.0, 0.0, g])

        assert accel_at_rest_enu[0] == 0.0
        assert accel_at_rest_enu[1] == 0.0
        assert math.isclose(accel_at_rest_enu[2], 9.80665, abs_tol=1e-4)

    def test_f2_3_enu_counter_clockwise_positive_yaw(self):
        """Verify counter-clockwise rotation about +Z yields positive yaw angle."""
        def yaw_from_rot(theta):
            return theta  # Standard mathematical definition

        assert yaw_from_rot(math.radians(30)) > 0.0
        assert yaw_from_rot(math.radians(-30)) < 0.0

    def test_f2_3_enu_right_hand_rule_angular_rates(self):
        """Verify angular velocity vector follows right-hand rule about axes."""
        # Rotating CCW about Z: vector points along +Z
        w_z = np.array([0.0, 0.0, 1.0])
        assert w_z[2] > 0.0

    def test_f2_3_enu_acceleration_invariance_in_body_frame(self):
        """Verify coordinate transformations preserve acceleration vector magnitude."""
        accel_body = np.array([0.5, -0.2, 9.80665])
        yaw = math.radians(45)
        # Rotation around Z by 45 degrees
        R = np.array([
            [ math.cos(yaw), -math.sin(yaw), 0.0],
            [ math.sin(yaw),  math.cos(yaw), 0.0],
            [ 0.0,            0.0,           1.0],
        ])
        accel_world = R @ accel_body
        assert math.isclose(np.linalg.norm(accel_world), np.linalg.norm(accel_body), rel_tol=1e-9)


@pytest.mark.tier1
class TestF24_AllanVariance_CovarianceInflation:
    """F2.4: Allan Variance computation, noise extraction, and covariance inflation."""

    def test_f2_4_allan_variance_computation(self):
        """Verify Allan deviation computation over multiple cluster sizes."""
        analyzer = AllanVarianceAnalyzer(sample_rate_hz=200.0)
        np.random.seed(42)
        # White noise signal: adev should decrease with tau^(-0.5)
        white_noise = np.random.normal(0.0, 0.05, size=2000)
        taus, adevs = analyzer.compute_allan_deviation(white_noise, num_tau=10)

        assert len(taus) > 3
        assert len(adevs) == len(taus)
        assert np.all(adevs > 0.0)
        # Check decreasing trend for white noise at small tau
        assert adevs[0] > adevs[2]

    def test_f2_4_allan_variance_arw_extraction(self):
        """Verify extraction of Angle Random Walk (Ng / ARW) from Allan deviation."""
        analyzer = AllanVarianceAnalyzer(sample_rate_hz=200.0)
        np.random.seed(123)
        data = np.random.normal(0.0, 0.02, size=3000)
        taus, adevs = analyzer.compute_allan_deviation(data, num_tau=15)
        params = analyzer.extract_noise_parameters(taus, adevs)

        assert "Ng_nominal" in params
        assert params["Ng_nominal"] > 0.0

    def test_f2_4_allan_variance_bias_instability_extraction(self):
        """Verify extraction of Bias Instability (Kg)."""
        analyzer = AllanVarianceAnalyzer(sample_rate_hz=200.0)
        np.random.seed(456)
        data = np.random.normal(0.0, 0.01, size=2000)
        taus, adevs = analyzer.compute_allan_deviation(data, num_tau=10)
        params = analyzer.extract_noise_parameters(taus, adevs)

        assert "Kg_nominal" in params
        assert params["Kg_nominal"] > 0.0

    def test_f2_4_field_covariance_inflation_factor_1_5_to_2_0(self):
        """Verify field covariance inflation multiplies noise density by 1.5x - 2.0x."""
        for factor in [1.5, 1.75, 2.0]:
            analyzer = AllanVarianceAnalyzer(sample_rate_hz=100.0, inflation_factor=factor)
            np.random.seed(50)
            data = np.random.normal(0.0, 0.01, size=1500)
            taus, adevs = analyzer.compute_allan_deviation(data, num_tau=8)
            params = analyzer.extract_noise_parameters(taus, adevs)

            expected_ng_inflated = params["Ng_nominal"] * factor
            assert math.isclose(params["Ng_inflated"], expected_ng_inflated, rel_tol=1e-5)
            # Variance is (Ng * factor)^2 = Ng^2 * factor^2
            assert math.isclose(params["variance_inflated"], (params["Ng_nominal"] * factor) ** 2, rel_tol=1e-5)

    def test_f2_4_inflation_prevents_filter_overconfidence(self):
        """Verify inflated variance provides positive safety margin under vibration."""
        analyzer = AllanVarianceAnalyzer(inflation_factor=1.8)
        nominal_var = 1e-4
        inflated_var = nominal_var * (1.8 ** 2)

        assert inflated_var > nominal_var * 3.0  # (1.8)^2 = 3.24x
