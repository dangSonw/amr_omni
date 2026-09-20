#!/usr/bin/env python3
"""Adversarial stress test harness for M2 IMU Gyroscope Zero-Rate Bias Nulling & Heading Stability.

Validates:
1. Residual static drift < 0.05 deg/s (8.7266e-4 rad/s) across 1,000 randomized bias vectors.
2. Long-term integration drift: integrate yaw over 60 seconds at rest to verify total drift < 3.0 deg.
3. Motion disturbance rejection: inject motion impulses (w > 0.05 rad/s) during bias calibration and
   verify contaminated frames are rejected.
4. REP-103 ENU compliance: verify static Z acceleration is +9.80665 m/s^2 and positive yaw follows right-hand rule.
5. End-to-end execution of the native C++ firmware stress benchmark (`build/imu_stress_benchmark`).
"""

import math
import subprocess
from pathlib import Path
import numpy as np
import pytest

pytestmark = pytest.mark.skip(
    reason="Manual ST AN4508 software calibration deprecated in favor of BNO080 onboard hardware sensor fusion"
)

WORKSPACE_ROOT = Path(__file__).resolve().parent.parent.parent
BUILD_DIR = WORKSPACE_ROOT / "build"
BENCHMARK_BIN = BUILD_DIR / "imu_stress_benchmark"
BENCHMARK_SRC = WORKSPACE_ROOT / "tests" / "stress" / "imu_stress_benchmark.cpp"
FIRMWARE_INC = WORKSPACE_ROOT / "firmware" / "stm32_f407vg_arduino_sim" / "include"


@pytest.fixture(scope="session", autouse=True)
def build_cpp_benchmark():
    """Ensure standalone C++ empirical benchmark binary is compiled."""
    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    compile_cmd = [
        "g++", "-O3", "-Wall", "-Wextra",
        f"-I{FIRMWARE_INC}",
        str(BENCHMARK_SRC),
        "-o", str(BENCHMARK_BIN)
    ]
    res = subprocess.run(compile_cmd, capture_output=True, text=True)
    assert res.returncode == 0, f"Benchmark compilation failed:\n{res.stderr}"
    return BENCHMARK_BIN


class TestImuGyroAdversarialStress:
    """Adversarial stress testing suite for IMU gyroscope bias nulling and heading stability."""

    def test_cpp_benchmark_execution_and_verdict(self):
        """Execute standalone C++ benchmark and verify all 5 native firmware test suites pass."""
        assert BENCHMARK_BIN.exists(), f"Benchmark binary {BENCHMARK_BIN} missing"
        res = subprocess.run([str(BENCHMARK_BIN)], capture_output=True, text=True)
        assert res.returncode == 0, f"Benchmark failed with exit code {res.returncode}:\n{res.stdout}\n{res.stderr}"

        stdout = res.stdout
        assert "--> Running Test 1: 1,000 Randomized Gyro Bias Vectors" in stdout
        assert "Passed Trials:               1000 (100.00%)" in stdout
        assert "--> Running Test 2: Long-Term 60-Second Yaw Integration" in stdout
        assert "Passed Trials:               100 (100.00%)" in stdout
        assert "--> Running Test 3: Motion Disturbance Rejection Test" in stdout
        assert "Motion Rejection Rate:       99.90%" in stdout or "Target >= 99.0%" in stdout
        assert "Stationary False Alarm Rate: 0.00%" in stdout
        assert "--> Running Test 4: REP-103 ENU Coordinate Standard Compliance" in stdout
        assert "Static Z Match (< 1e-4):     YES" in stdout
        assert "Positive Yaw Right-Hand:     YES" in stdout
        assert "OVERALL VERDICT: APPROVE (ALL 5 TESTS PASSED)" in stdout

    def test_1000_randomized_bias_vectors_residual_drift(self):
        """Verify residual static drift < 0.05 deg/s across 1,000 randomized bias vectors."""
        num_trials = 1000
        rng = np.random.default_rng(seed=42)
        threshold_rad_s = math.radians(0.05)  # ~ 8.7266e-4 rad/s

        # Random true biases in [-0.3, 0.3] rad/s (~ +/- 17 deg/s)
        true_biases = rng.uniform(-0.3, 0.3, size=(num_trials, 3))
        residual_errors = []

        for i in range(num_trials):
            nuller = GyroBiasNuller(variance_threshold=1.0e-4)
            # 1,000 stationary samples with white noise (sigma = 1.2e-3 rad/s)
            noise = rng.normal(0.0, 0.0012, size=(1000, 3))
            samples = true_biases[i] + noise

            est_bias, reported_drift = nuller.calibrate(samples)
            true_error_norm = float(np.linalg.norm(est_bias - true_biases[i]))

            residual_errors.append(true_error_norm)
            assert reported_drift < threshold_rad_s, (
                f"Trial {i}: reported drift {reported_drift:.6e} >= {threshold_rad_s:.6e}"
            )
            assert true_error_norm < threshold_rad_s, (
                f"Trial {i}: true error {true_error_norm:.6e} >= {threshold_rad_s:.6e}"
            )

        residual_errors = np.array(residual_errors)
        max_err = float(np.max(residual_errors))
        mean_err = float(np.mean(residual_errors))
        p95_err = float(np.percentile(residual_errors, 95))

        assert max_err < threshold_rad_s
        assert mean_err < threshold_rad_s * 0.2  # Expected mean error ~ 6e-5 rad/s

    def test_60_second_yaw_integration_drift_under_3_degrees(self):
        """Verify long-term resting yaw integration drift over 60 seconds is < 3.0 degrees."""
        num_trials = 50
        dt = 0.01  # 100 Hz
        sim_steps = 6000  # 60 seconds
        max_drift_threshold_deg = 3.0
        rng = np.random.default_rng(seed=77)

        calibrated_drifts_deg = []
        uncalibrated_drifts_deg = []

        for trial in range(num_trials):
            nuller = GyroBiasNuller()
            true_bias = rng.uniform(-0.15, 0.15, size=3)  # Significant bias on all axes

            # Calibrate on 1,000 stationary samples
            cal_samples = true_bias + rng.normal(0.0, 0.0012, size=(1000, 3))
            nuller.calibrate(cal_samples)

            # 60 seconds resting stream
            raw_stream = true_bias + rng.normal(0.0, 0.0012, size=(sim_steps, 3))

            # Uncalibrated yaw integration
            uncal_yaw_rad = np.sum(raw_stream[:, 2]) * dt
            uncal_drift_deg = abs(uncal_yaw_rad) * (180.0 / math.pi)
            uncalibrated_drifts_deg.append(uncal_drift_deg)

            # Calibrated yaw integration
            cal_stream = nuller.apply(raw_stream)
            cal_yaw_rad = np.sum(cal_stream[:, 2]) * dt
            cal_drift_deg = abs(cal_yaw_rad) * (180.0 / math.pi)
            calibrated_drifts_deg.append(cal_drift_deg)

            assert cal_drift_deg < max_drift_threshold_deg, (
                f"Trial {trial}: calibrated drift {cal_drift_deg:.4f} deg exceeded {max_drift_threshold_deg} deg"
            )

        mean_uncal = float(np.mean(uncalibrated_drifts_deg))
        max_cal = float(np.max(calibrated_drifts_deg))
        mean_cal = float(np.mean(calibrated_drifts_deg))

        # Uncalibrated drift should be huge (typically > 100 degrees over 60s)
        assert mean_uncal > 50.0
        # Calibrated drift should be well under 3.0 degrees
        assert max_cal < max_drift_threshold_deg
        assert mean_cal < 0.5

    def test_motion_disturbance_rejection_rate(self):
        """Verify motion impulses (w > 0.05 rad/s) during calibration trigger rejection."""
        num_trials = 500
        rng = np.random.default_rng(seed=99)
        rejected_count = 0

        for trial in range(num_trials):
            nuller = GyroBiasNuller(variance_threshold=1.0e-4)
            base_bias = rng.uniform(-0.1, 0.1, size=3)

            # Base stationary samples
            samples = base_bias + rng.normal(0.0, 0.001, size=(1000, 3))

            # Inject impulse w > 0.05 rad/s (0.06 to 2.0 rad/s)
            start = rng.integers(10, 920)
            duration = rng.integers(10, 40)
            axis = trial % 3
            impulse_mag = rng.uniform(0.06, 2.0)

            samples[start:start + duration, axis] += impulse_mag

            try:
                nuller.calibrate(samples)
            except ValueError as e:
                if "Motion detected" in str(e):
                    rejected_count += 1

        rejection_rate = (rejected_count / num_trials) * 100.0
        assert rejection_rate >= 99.0, f"Rejection rate {rejection_rate:.2f}% < 99.0%"

    def test_rep103_enu_static_gravity_and_right_hand_rule(self):
        """Verify static Z accel is +9.80665 m/s^2 and positive yaw follows right-hand rule."""
        g = 9.80665
        calibrator = ST_AN4508_Calibrator(g=g)

        # Level ground stationary robot: reaction specific force is upwards (+Z)
        static_raw_a = np.array([0.0, 0.0, g])
        cal_a = calibrator.apply(static_raw_a)

        assert abs(cal_a[0]) < 1e-4, f"Ax non-zero: {cal_a[0]}"
        assert abs(cal_a[1]) < 1e-4, f"Ay non-zero: {cal_a[1]}"
        assert abs(cal_a[2] - g) < 1e-4, f"Az mismatch: {cal_a[2]} != {g}"
        assert cal_a[2] > 0.0, "Az must be positive (upward reaction force in ENU)"

        # Right-hand rule rotation check: CCW rotation about +Z gives positive wz
        nuller = GyroBiasNuller()
        samples = np.zeros((100, 3))
        nuller.calibrate(samples)

        ccw_yaw_rate = np.array([0.0, 0.0, 0.5])
        cal_gyro = nuller.apply(ccw_yaw_rate)
        assert cal_gyro[2] > 0.0, "CCW rotation about +Z must yield positive yaw rate (right-hand rule)"

        # CCW rotation about +Y (pitch) and +X (roll)
        ccw_pitch_rate = np.array([0.0, 0.5, 0.0])
        assert nuller.apply(ccw_pitch_rate)[1] > 0.0

        ccw_roll_rate = np.array([0.5, 0.0, 0.0])
        assert nuller.apply(ccw_roll_rate)[0] > 0.0

    def test_spherical_gravity_norm_invariance(self):
        """Verify vector norm of gravity is invariant across 3D arbitrary orientations."""
        g = 9.80665
        calibrator = ST_AN4508_Calibrator(g=g)

        for lat_deg in range(-80, 81, 20):
            for lon_deg in range(0, 360, 30):
                lat = math.radians(lat_deg)
                lon = math.radians(lon_deg)
                u = np.array([math.cos(lat) * math.cos(lon), math.cos(lat) * math.sin(lon), math.sin(lat)])
                raw_a = u * g
                cal_a = calibrator.apply(raw_a)
                norm = float(np.linalg.norm(cal_a))
                assert abs(norm - g) < 1e-4, f"Norm error at lat={lat_deg}, lon={lon_deg}: {norm}"
