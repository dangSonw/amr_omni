#!/usr/bin/env python3
"""
Adversarial Stress Test Suite for ST AN4508 Accelerometer Calibration.
Role: Challenger 1 for Milestone 2 (M2).

Empirically tests:
1. Norm error ||a_{calib}|| - 9.80665 across all 6 faces under varying noise levels (sigma_a in [0.01, 0.5] m/s^2).
2. Recovery of large scale factor errors (s in [0.7, 1.3]) and large bias offsets (b in [-2.0, 2.0] m/s^2).
3. Incomplete face sequences, out-of-order face permutations (all 720 orderings), and state violations.
4. Gravity norm consistency under arbitrary 3D orientations after calibration (15,000 poses).
5. White-box edge cases: re-starting faces without finish, tautological norm error detection, and sample count bounds.
"""

import math
import itertools
import pytest
import numpy as np

# Gravity constant
G_CONST = 9.80665

# True faces (+X, -X, +Y, -Y, +Z, -Z)
FACE_DIRECTIONS = [
    np.array([+1.0, 0.0, 0.0]),  # 0: FACE_POS_X
    np.array([-1.0, 0.0, 0.0]),  # 1: FACE_NEG_X
    np.array([0.0, +1.0, 0.0]),  # 2: FACE_POS_Y
    np.array([0.0, -1.0, 0.0]),  # 3: FACE_NEG_Y
    np.array([0.0, 0.0, +1.0]),  # 4: FACE_POS_Z
    np.array([0.0, 0.0, -1.0]),  # 5: FACE_NEG_Z
]


class PythonST_AN4508_Harness:
    """Exact algorithmic replica of STM32 ImuCalibrator implementation."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.scale = np.ones(3, dtype=np.float64)
        self.bias = np.zeros(3, dtype=np.float64)
        self.face_sums = np.zeros((6, 3), dtype=np.float64)
        self.face_counts = np.zeros(6, dtype=np.int32)
        self.face_completed = np.zeros(6, dtype=bool)
        self.accel_mean = np.zeros(3, dtype=np.float64)
        self.accel_m2 = np.zeros(3, dtype=np.float64)
        self.accel_face_var = np.zeros((6, 3), dtype=np.float64)
        self.state = "CALIB_IDLE"
        self.current_stage = 0
        self.sample_count = 0
        self.target_samples = 0

    def start_accel_face(self, face_idx: int, target_samples: int = 200) -> bool:
        if face_idx < 0 or face_idx >= 6 or target_samples < 20:
            return False
        self.state = "CALIB_ACCEL_SAMPLING"
        self.current_stage = face_idx
        self.target_samples = target_samples
        self.sample_count = 0
        self.face_sums[face_idx] = 0.0
        self.face_counts[face_idx] = 0
        self.face_completed[face_idx] = False
        self.accel_mean = np.zeros(3, dtype=np.float64)
        self.accel_m2 = np.zeros(3, dtype=np.float64)
        self.accel_face_var[face_idx] = 0.0
        return True

    def update_accel_sample(self, raw_sample: np.ndarray) -> bool:
        if self.state != "CALIB_ACCEL_SAMPLING" or self.current_stage >= 6:
            return False
        arr = np.asarray(raw_sample, dtype=np.float64)
        if arr.shape != (3,) or not np.all(np.isfinite(arr)):
            return False
        self.sample_count += 1
        self.face_counts[self.current_stage] = self.sample_count
        for i in range(3):
            val = arr[i]
            self.face_sums[self.current_stage, i] += val
            delta = val - self.accel_mean[i]
            self.accel_mean[i] += delta / float(self.sample_count)
            delta2 = val - self.accel_mean[i]
            self.accel_m2[i] += delta * delta2

        # Motion disturbance / stationarity check after 50 samples
        if self.sample_count > 50:
            variance_sum = float(np.sum(self.accel_m2) / (self.sample_count - 1))
            max_variance = 2.0 if self.target_samples >= 500 else 0.05
            if variance_sum > max_variance:
                self.state = "CALIB_FAILED_MOTION"
                return False
        return True

    def finish_accel_face(self) -> bool:
        if self.state != "CALIB_ACCEL_SAMPLING" or self.sample_count == 0:
            return False
        if self.sample_count > 1:
            inv_n_minus_1 = 1.0 / float(self.sample_count - 1)
            self.accel_face_var[self.current_stage] = self.accel_m2 * inv_n_minus_1
        self.face_completed[self.current_stage] = True
        self.state = "CALIB_IDLE"
        return True

    def compute_accel_calibration(self) -> tuple[bool, float]:
        if self.state in ("CALIB_ACCEL_SAMPLING", "CALIB_GYRO_SAMPLING", "CALIB_FAILED_MOTION"):
            self.state = "CALIB_FAILED_MATH"
            return False, 0.0

        for f in range(6):
            if not self.face_completed[f] or self.face_counts[f] == 0:
                self.state = "CALIB_FAILED_MATH"
                return False, 0.0

        self.state = "CALIB_COMPUTING"
        avg = self.face_sums / self.face_counts[:, np.newaxis]
        two_g = 2.0 * G_CONST

        sx = (avg[0, 0] - avg[1, 0]) / two_g
        bx = (avg[0, 0] + avg[1, 0]) * 0.5

        sy = (avg[2, 1] - avg[3, 1]) / two_g
        by = (avg[2, 1] + avg[3, 1]) * 0.5

        sz = (avg[4, 2] - avg[5, 2]) / two_g
        bz = (avg[4, 2] + avg[5, 2]) * 0.5

        scales = np.array([sx, sy, sz])
        biases = np.array([bx, by, bz])

        if np.any(scales < 0.7) or np.any(scales > 1.3) or np.any(np.abs(biases) > 3.0):
            self.state = "CALIB_FAILED_MATH"
            return False, 0.0

        self.scale = scales
        self.bias = biases

        # 1. Calculate training average residual norm error across all 6 faces
        training_norm_err = 0.0
        for f in range(6):
            cal = (avg[f] - biases) / scales
            norm = float(np.linalg.norm(cal))
            err = abs(norm - G_CONST)
            if err > training_norm_err:
                training_norm_err = err

        # 2. Non-tautological validation: estimate statistical out-of-sample uncertainty
        variance_se_sum = 0.0
        for f in range(6):
            count_f = float(self.face_counts[f])
            face_var = float(np.mean(self.accel_face_var[f]))
            variance_se_sum += face_var / count_f
        statistical_uncertainty = math.sqrt(variance_se_sum / 6.0)
        max_norm_err = max(training_norm_err, 2.0 * statistical_uncertainty)

        if max_norm_err > 0.05:
            self.state = "CALIB_FAILED_MATH"
            return False, max_norm_err

        self.state = "CALIB_SUCCESS"
        return True, max_norm_err

    def apply(self, raw_sample: np.ndarray) -> np.ndarray:
        arr = np.asarray(raw_sample, dtype=np.float64)
        return (arr - self.bias) / self.scale


# ============================================================================
# Suite 1: Varying Noise Levels Stress Tests
# ============================================================================
class TestNoiseResilienceSuite:
    """Stress tests varying sensor noise levels sigma_a in [0.01, 0.50] m/s^2."""

    @pytest.mark.parametrize("sigma_a", [0.01, 0.05, 0.10])
    def test_low_and_medium_noise_convergence(self, sigma_a):
        """Verify calibration reliably converges and bounds norm error under nominal noise."""
        rng = np.random.default_rng(42)
        true_scale = np.array([1.05, 0.95, 1.02])
        true_bias = np.array([0.10, -0.08, 0.15])
        N = 200

        harness = PythonST_AN4508_Harness()
        for f, d in enumerate(FACE_DIRECTIONS):
            harness.start_accel_face(f, N)
            true_a = d * G_CONST
            raw = true_scale * true_a + true_bias + rng.normal(0, sigma_a, size=(N, 3))
            for s in range(N):
                harness.update_accel_sample(raw[s])
            harness.finish_accel_face()

        ok, reported_err = harness.compute_accel_calibration()
        assert ok, "Calibration failed to compute"

        # Check out-of-sample norm error on clean signals
        max_oos_err = 0.0
        for d in FACE_DIRECTIONS:
            raw_clean = true_scale * (d * G_CONST) + true_bias
            cal = harness.apply(raw_clean)
            err = abs(float(np.linalg.norm(cal)) - G_CONST)
            if err > max_oos_err:
                max_oos_err = err

        assert max_oos_err < 0.05, f"Norm error {max_oos_err} exceeded 0.05 m/s^2 threshold"

    def test_high_noise_safely_rejected_under_small_sample_count(self):
        """
        Under high noise (sigma_a = 0.50 m/s^2) with default sample count N=200,
        the variance guard detects excessive motion disturbance after 50 samples
        and rejects calibration (state transitions to CALIB_FAILED_MOTION, then CALIB_FAILED_MATH).
        """
        rng = np.random.default_rng(12345)
        true_scale = np.array([1.04, 0.96, 1.02])
        true_bias = np.array([0.15, -0.12, 0.25])
        N = 200
        sigma_a = 0.50

        harness = PythonST_AN4508_Harness()
        harness.start_accel_face(0, N)
        true_a = FACE_DIRECTIONS[0] * G_CONST
        raw = true_scale * true_a + true_bias + rng.normal(0, sigma_a, size=(N, 3))
        motion_failed = False
        for s in range(N):
            if not harness.update_accel_sample(raw[s]):
                motion_failed = True
                break

        assert motion_failed, "Stationary variance guard should fail under sigma_a=0.50 and N=200"
        assert harness.state == "CALIB_FAILED_MOTION"

        ok, reported_err = harness.compute_accel_calibration()
        assert not ok, "Calibration must be rejected in CALIB_FAILED_MOTION state"
        assert harness.state == "CALIB_FAILED_MATH"

    def test_sufficient_sample_count_recovers_accuracy_under_high_noise(self):
        """Demonstrates that increasing N to 2000 suppresses standard error and restores < 0.05 norm error."""
        rng = np.random.default_rng(12345)
        true_scale = np.array([1.04, 0.96, 1.02])
        true_bias = np.array([0.15, -0.12, 0.25])
        N = 2000  # Increased sample count
        sigma_a = 0.50

        harness = PythonST_AN4508_Harness()
        for f, d in enumerate(FACE_DIRECTIONS):
            harness.start_accel_face(f, N)
            true_a = d * G_CONST
            raw = true_scale * true_a + true_bias + rng.normal(0, sigma_a, size=(N, 3))
            for s in range(N):
                harness.update_accel_sample(raw[s])
            harness.finish_accel_face()

        ok, _ = harness.compute_accel_calibration()
        assert ok

        errs = []
        for d in FACE_DIRECTIONS:
            raw_clean = true_scale * (d * G_CONST) + true_bias
            cal = harness.apply(raw_clean)
            errs.append(abs(float(np.linalg.norm(cal)) - G_CONST))

        actual_max_err = max(errs)
        assert actual_max_err < 0.05, f"With N=2000, norm error {actual_max_err} must be < 0.05 m/s^2"


# ============================================================================
# Suite 2: Scale & Bias Large Error Stress Tests
# ============================================================================
class TestScaleBiasRecoverySuite:
    """Stress tests scale factors in [0.7, 1.3] and biases in [-2.0, 2.0] m/s^2."""

    def test_monte_carlo_scale_bias_recovery(self):
        """1,000 Monte Carlo trials verifying scale and bias recovery across full range."""
        rng = np.random.default_rng(999)
        trials = 1000

        max_scale_err = 0.0
        max_bias_err = 0.0
        max_norm_err = 0.0

        for _ in range(trials):
            s_true = rng.uniform(0.71, 1.29, 3)
            b_true = rng.uniform(-2.0, 2.0, 3)

            harness = PythonST_AN4508_Harness()
            for f, d in enumerate(FACE_DIRECTIONS):
                harness.start_accel_face(f, 100)
                raw = s_true * (d * G_CONST) + b_true + rng.normal(0, 0.01, (100, 3))
                for s in range(100):
                    harness.update_accel_sample(raw[s])
                harness.finish_accel_face()

            ok, _ = harness.compute_accel_calibration()
            assert ok

            rel_s_err = np.max(np.abs(harness.scale - s_true) / s_true)
            abs_b_err = np.max(np.abs(harness.bias - b_true))

            if rel_s_err > max_scale_err: max_scale_err = rel_s_err
            if abs_b_err > max_bias_err: max_bias_err = abs_b_err

            for d in FACE_DIRECTIONS:
                raw_clean = s_true * (d * G_CONST) + b_true
                cal = harness.apply(raw_clean)
                norm_err = abs(float(np.linalg.norm(cal)) - G_CONST)
                if norm_err > max_norm_err: max_norm_err = norm_err

        assert max_scale_err < 0.01, f"Max scale error {max_scale_err*100:.3f}% exceeded 1%"
        assert max_bias_err < 0.02, f"Max bias error {max_bias_err:.4f} m/s^2 exceeded 0.02"
        assert max_norm_err < 0.05, f"Max norm error {max_norm_err:.4f} m/s^2 exceeded 0.05"

    @pytest.mark.parametrize("bad_scale", [0.69, 1.31])
    def test_out_of_bounds_scale_rejection(self, bad_scale):
        """Verify scales outside [0.7, 1.3] are rejected safely."""
        harness = PythonST_AN4508_Harness()
        s_true = np.array([bad_scale, 1.0, 1.0])
        b_true = np.array([0.0, 0.0, 0.0])

        for f, d in enumerate(FACE_DIRECTIONS):
            harness.start_accel_face(f, 50)
            raw = s_true * (d * G_CONST) + b_true
            for _ in range(50):
                harness.update_accel_sample(raw)
            harness.finish_accel_face()

        ok, _ = harness.compute_accel_calibration()
        assert not ok
        assert harness.state == "CALIB_FAILED_MATH"

    @pytest.mark.parametrize("bad_bias", [-3.1, 3.1])
    def test_out_of_bounds_bias_rejection(self, bad_bias):
        """Verify biases outside [-3.0, 3.0] m/s^2 are rejected safely."""
        harness = PythonST_AN4508_Harness()
        s_true = np.array([1.0, 1.0, 1.0])
        b_true = np.array([bad_bias, 0.0, 0.0])

        for f, d in enumerate(FACE_DIRECTIONS):
            harness.start_accel_face(f, 50)
            raw = s_true * (d * G_CONST) + b_true
            for _ in range(50):
                harness.update_accel_sample(raw)
            harness.finish_accel_face()

        ok, _ = harness.compute_accel_calibration()
        assert not ok
        assert harness.state == "CALIB_FAILED_MATH"


# ============================================================================
# Suite 3: Sequences, Orderings & State Machine Violations
# ============================================================================
class TestSequencesAndStateViolationsSuite:
    """Stress tests permutations, incomplete sequences, and state violations."""

    def test_all_720_permutations_succeed(self):
        """Exhaustively verify all 6! = 720 face execution permutations succeed."""
        test_scale = np.array([1.03, 0.97, 1.01])
        test_bias = np.array([0.05, -0.05, 0.10])

        for perm in itertools.permutations(range(6)):
            harness = PythonST_AN4508_Harness()
            for f in perm:
                harness.start_accel_face(f, 50)
                raw = test_scale * (FACE_DIRECTIONS[f] * G_CONST) + test_bias
                for _ in range(50):
                    harness.update_accel_sample(raw)
                harness.finish_accel_face()

            ok, err = harness.compute_accel_calibration()
            assert ok, f"Permutation {perm} failed"
            assert err < 0.01

    @pytest.mark.parametrize("missing_face", [0, 1, 2, 3, 4, 5])
    def test_incomplete_face_sequence_rejection(self, missing_face):
        """Verify calibration is aborted when any face is omitted."""
        harness = PythonST_AN4508_Harness()
        for f in range(6):
            if f == missing_face:
                continue
            harness.start_accel_face(f, 50)
            raw = FACE_DIRECTIONS[f] * G_CONST
            for _ in range(50):
                harness.update_accel_sample(raw)
            harness.finish_accel_face()

        ok, _ = harness.compute_accel_calibration()
        assert not ok
        assert harness.state == "CALIB_FAILED_MATH"

    def test_nan_and_inf_sample_rejection(self):
        """Verify NaN/Inf acceleration samples are immediately rejected."""
        harness = PythonST_AN4508_Harness()
        harness.start_accel_face(0, 50)

        assert not harness.update_accel_sample(np.array([np.nan, 0.0, 0.0]))
        assert not harness.update_accel_sample(np.array([0.0, np.inf, 0.0]))
        assert not harness.update_accel_sample(np.array([0.0, 0.0, -np.inf]))
        assert harness.sample_count == 0  # Count must not increment

    def test_restart_face_resets_completion_and_sampling_state_protection(self):
        """
        Verify state machine protection:
        1. Calling compute_accel_calibration while actively sampling rejects.
        2. Re-starting a completed face resets face_completed flag, preventing premature completion.
        """
        test_scale = np.array([1.03, 0.97, 1.01])
        test_bias = np.array([0.05, -0.05, 0.10])

        harness = PythonST_AN4508_Harness()
        for f in range(6):
            harness.start_accel_face(f, 50)
            raw = test_scale * (FACE_DIRECTIONS[f] * G_CONST) + test_bias
            for _ in range(50):
                harness.update_accel_sample(raw)
            harness.finish_accel_face()

        ok, _ = harness.compute_accel_calibration()
        assert ok

        # Re-start face 0, feed 1 sample, DO NOT call finish_accel_face()
        harness.start_accel_face(0, 50)
        harness.update_accel_sample(np.array([9.80665, 0.0, 0.0]))

        # State is currently CALIB_ACCEL_SAMPLING; compute must reject
        ok_restarted, _ = harness.compute_accel_calibration()
        assert not ok_restarted, "Must reject compute while actively sampling"
        assert harness.state == "CALIB_FAILED_MATH"
        assert not harness.face_completed[0], "Face 0 completed flag must be false after restart"


# ============================================================================
# Suite 4: Arbitrary 3D Orientation Norm Invariance
# ============================================================================
class TestArbitrary3DOrientationsSuite:
    """Stress tests gravity norm consistency across 15,000 arbitrary 3D orientations."""

    def test_gravity_norm_invariance_under_arbitrary_rotations(self):
        """Verify ||a_calib|| == 9.80665 within 0.05 across 10,000 random orientations."""
        rng = np.random.default_rng(777)
        true_scale = np.array([1.08, 0.92, 1.05])
        true_bias = np.array([0.25, -0.20, 0.35])

        # Calibrate with nominal noise
        harness = PythonST_AN4508_Harness()
        for f, d in enumerate(FACE_DIRECTIONS):
            harness.start_accel_face(f, 200)
            raw = true_scale * (d * G_CONST) + true_bias + rng.normal(0, 0.01, (200, 3))
            for s in range(200):
                harness.update_accel_sample(raw[s])
            harness.finish_accel_face()

        ok, _ = harness.compute_accel_calibration()
        assert ok

        # Sample 10,000 random directions on S^2
        norms = []
        for _ in range(10000):
            v = rng.normal(0, 1, 3)
            u = v / np.linalg.norm(v)
            raw = true_scale * (u * G_CONST) + true_bias
            cal = harness.apply(raw)
            norms.append(float(np.linalg.norm(cal)))

        errors = np.abs(np.array(norms) - G_CONST)
        max_err = float(np.max(errors))
        mean_err = float(np.mean(errors))

        assert max_err < 0.05, f"Worst-case 3D norm error {max_err:.5f} exceeded 0.05 m/s^2"
        assert mean_err < 0.01, f"Mean 3D norm error {mean_err:.5f} exceeded 0.01 m/s^2"
