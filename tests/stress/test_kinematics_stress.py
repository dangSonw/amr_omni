#!/usr/bin/env python3
"""Adversarial stress test and fuzzing suite for AMR Omni Mecanum Kinematics.

Validates:
1. Monte Carlo random sampling across 50,000 velocity vectors ([-5.0, 5.0] m/s)
   and perturbed wheel radius correction matrices Kr in [0.8, 1.2].
2. Strict Moore-Penrose pseudo-inverse round-trip consistency: ||FK(IK(v)) - v|| < 1e-5.
3. Actuator saturation direction and magnitude scaling invariants.
4. Adversarial fuzzing with subnormals, zeros, negative numbers, NaNs, infinities,
   and malformed shapes to guarantee graceful exception handling without process crashes.
"""

import math
import sys
import time
import unittest
import numpy as np

from omni_control.kinematics import (
    forward_kinematics,
    inverse_kinematics,
    compute_wheel_speeds,
    compute_body_twist,
    validate_twist,
    WHEEL_ORDER,
)


class KinematicsMonteCarloStressTest(unittest.TestCase):
    """50,000-sample Monte Carlo stress test for kinematics consistency."""

    def test_50k_monte_carlo_round_trip(self):
        """Verify ||FK(IK(v)) - v|| < 1e-5 across 50,000 random samples."""
        num_samples = 50000
        rng = np.random.default_rng(seed=42)

        # Velocities within and beyond operational envelope [-5.0, 5.0] m/s and rad/s
        vx_samples = rng.uniform(-5.0, 5.0, num_samples)
        vy_samples = rng.uniform(-5.0, 5.0, num_samples)
        wz_samples = rng.uniform(-5.0, 5.0, num_samples)

        # Perturbed Kr multipliers in [0.8, 1.2]
        kr_samples = rng.uniform(0.8, 1.2, (num_samples, 4))

        # Diverse robot geometries
        nominal_r = 0.03
        wheelbase = 0.1312
        track_width = 0.1312
        # Unconstrained wheel speed limit to test pure Moore-Penrose invariant
        max_speed = 1e9

        max_err = 0.0
        sum_err = 0.0
        violations = []
        errors = np.empty(num_samples, dtype=np.float64)

        t_start = time.perf_counter()

        for i in range(num_samples):
            vx = float(vx_samples[i])
            vy = float(vy_samples[i])
            wz = float(wz_samples[i])
            kr = kr_samples[i]

            # Alternate representation: 1D list, 1D numpy array, 4x4 diagonal matrix, wheel_radii
            rep_type = i % 4
            if rep_type == 0:
                kr_arg = kr.tolist()
                wheels = inverse_kinematics(vx, vy, wz, nominal_r, wheelbase, track_width,
                                            max_speed, wheel_radius_correction=kr_arg)
                rec = forward_kinematics(wheels, nominal_r, wheelbase, track_width,
                                         wheel_radius_correction=kr_arg)
            elif rep_type == 1:
                kr_arg = kr
                wheels = inverse_kinematics(vx, vy, wz, nominal_r, wheelbase, track_width,
                                            max_speed, kr=kr_arg)
                rec = forward_kinematics(wheels, nominal_r, wheelbase, track_width,
                                         kr=kr_arg)
            elif rep_type == 2:
                kr_arg = np.diag(kr)
                wheels = inverse_kinematics(vx, vy, wz, nominal_r, wheelbase, track_width,
                                            max_speed, wheel_radius_correction=kr_arg)
                rec = forward_kinematics(wheels, nominal_r, wheelbase, track_width,
                                         wheel_radius_correction=kr_arg)
            else:
                effective_radii = nominal_r * kr
                wheels = inverse_kinematics(vx, vy, wz, nominal_r, wheelbase, track_width,
                                            max_speed, wheel_radii=effective_radii)
                rec = forward_kinematics(wheels, nominal_r, wheelbase, track_width,
                                         wheel_radii=effective_radii)

            err = math.sqrt((rec[0] - vx) ** 2 + (rec[1] - vy) ** 2 + (rec[2] - wz) ** 2)
            errors[i] = err
            if err > max_err:
                max_err = err
            sum_err += err

            if err >= 1e-5:
                violations.append((i, (vx, vy, wz), err))

        elapsed = time.perf_counter() - t_start
        mean_err = sum_err / num_samples
        median_err = float(np.median(errors))
        p99_err = float(np.percentile(errors, 99.0))
        p99_99_err = float(np.percentile(errors, 99.99))

        print("\n" + "=" * 70)
        print("=== 50,000 MONTE CARLO KINEMATICS STRESS TEST RESULTS ===")
        print("=" * 70)
        print(f"Total samples tested:       {num_samples:,}")
        print(f"Sampling space:             vx, vy, wz in [-5.0, 5.0], Kr in [0.8, 1.2]")
        print(f"Execution time:             {elapsed:.3f} s ({num_samples / elapsed:,.0f} samples/s)")
        print(f"Violations (err >= 1e-5):   {len(violations)} (0.0000%)")
        print(f"Compliance rate:            {(num_samples - len(violations)) / num_samples * 100:.4f}%")
        print(f"Maximum round-trip error:   {max_err:.3e} (threshold: 1.000e-05)")
        print(f"Mean round-trip error:      {mean_err:.3e}")
        print(f"Median round-trip error:    {median_err:.3e}")
        print(f"99.0th percentile error:    {p99_err:.3e}")
        print(f"99.99th percentile error:   {p99_99_err:.3e}")
        print("=" * 70)

        self.assertEqual(len(violations), 0, f"Encountered {len(violations)} consistency violations >= 1e-5")
        self.assertLess(max_err, 1e-5, f"Maximum error {max_err} exceeds 1e-5 threshold")

    def test_actuator_saturation_invariance(self):
        """Verify saturation uniformly scales twist while strictly preserving direction."""
        num_samples = 5000
        rng = np.random.default_rng(seed=123)
        nominal_r = 0.03
        wheelbase = 0.1312
        track_width = 0.1312
        max_speed = 50.0  # rad/s, deliberately low to trigger saturation

        # High velocity vectors guaranteed to saturate
        vx_samples = rng.uniform(-4.0, 4.0, num_samples)
        vy_samples = rng.uniform(-4.0, 4.0, num_samples)
        wz_samples = rng.uniform(-6.0, 6.0, num_samples)
        kr_samples = rng.uniform(0.85, 1.15, (num_samples, 4))

        max_scaled_err = 0.0
        max_angular_err = 0.0
        saturation_count = 0

        for i in range(num_samples):
            vx = float(vx_samples[i])
            vy = float(vy_samples[i])
            wz = float(wz_samples[i])
            kr = kr_samples[i]

            # Compute unscaled wheel speeds to determine expected scale
            unscaled_wheels = inverse_kinematics(vx, vy, wz, nominal_r, wheelbase, track_width,
                                                 1e9, wheel_radius_correction=kr)
            expected_scale = max(1.0, max(abs(w) for w in unscaled_wheels) / max_speed)
            if expected_scale > 1.0:
                saturation_count += 1

            # Compute saturated wheel speeds
            sat_wheels = inverse_kinematics(vx, vy, wz, nominal_r, wheelbase, track_width,
                                            max_speed, wheel_radius_correction=kr)
            self.assertLessEqual(max(abs(w) for w in sat_wheels), max_speed + 1e-9)

            # Reconstruct twist from saturated wheels
            rec_vx, rec_vy, rec_wz = forward_kinematics(sat_wheels, nominal_r, wheelbase, track_width,
                                                        wheel_radius_correction=kr)

            # Expected reconstructed twist is original scaled down by expected_scale
            target_vx = vx / expected_scale
            target_vy = vy / expected_scale
            target_wz = wz / expected_scale

            err = math.sqrt((rec_vx - target_vx)**2 + (rec_vy - target_vy)**2 + (rec_wz - target_wz)**2)
            if err > max_scaled_err:
                max_scaled_err = err

            # Check directional collinearity in linear (vx, vy) subspace if nonzero
            orig_lin_norm = math.hypot(vx, vy)
            rec_lin_norm = math.hypot(rec_vx, rec_vy)
            if orig_lin_norm > 1e-4 and rec_lin_norm > 1e-4:
                dot = (vx * rec_vx + vy * rec_vy) / (orig_lin_norm * rec_lin_norm)
                # Clamp dot to [-1.0, 1.0] for acos
                dot = max(-1.0, min(1.0, dot))
                ang_err = math.acos(dot)
                if ang_err > max_angular_err:
                    max_angular_err = ang_err

        print("\n" + "=" * 70)
        print("=== ACTUATOR SATURATION INVARIANCE TEST ===")
        print("=" * 70)
        print(f"Total samples tested:       {num_samples:,}")
        print(f"Samples saturated:          {saturation_count:,} ({saturation_count / num_samples * 100:.1f}%)")
        print(f"Max scaled twist error:     {max_scaled_err:.3e} (threshold: 1.000e-05)")
        print(f"Max linear direction error: {max_angular_err:.3e} rad")
        print("=" * 70)

        self.assertGreaterEqual(saturation_count, int(0.85 * num_samples))
        self.assertLess(max_scaled_err, 1e-5)
        self.assertLess(max_angular_err, 1e-7)


class KinematicsAdversarialFuzzTest(unittest.TestCase):
    """Adversarial fuzz testing across extreme and pathological inputs."""

    def setUp(self):
        self.nominal_r = 0.03
        self.wheelbase = 0.1312
        self.track_width = 0.1312
        self.max_speed = 100.0

    def test_fuzz_nan_rejection_in_all_parameters(self):
        """Verify that NaN in any parameter raises ValueError and does not crash."""
        nan = float('nan')

        # Twist parameters
        with self.assertRaises(ValueError):
            inverse_kinematics(nan, 0.0, 0.0, self.nominal_r, self.wheelbase, self.track_width, self.max_speed)
        with self.assertRaises(ValueError):
            inverse_kinematics(0.0, nan, 0.0, self.nominal_r, self.wheelbase, self.track_width, self.max_speed)
        with self.assertRaises(ValueError):
            inverse_kinematics(0.0, 0.0, nan, self.nominal_r, self.wheelbase, self.track_width, self.max_speed)

        # Geometry parameters
        with self.assertRaises(ValueError):
            inverse_kinematics(0.1, 0.1, 0.1, nan, self.wheelbase, self.track_width, self.max_speed)
        with self.assertRaises(ValueError):
            inverse_kinematics(0.1, 0.1, 0.1, self.nominal_r, nan, self.track_width, self.max_speed)
        with self.assertRaises(ValueError):
            inverse_kinematics(0.1, 0.1, 0.1, self.nominal_r, self.wheelbase, nan, self.max_speed)
        with self.assertRaises(ValueError):
            inverse_kinematics(0.1, 0.1, 0.1, self.nominal_r, self.wheelbase, self.track_width, nan)

        # Correction parameter
        with self.assertRaises(ValueError):
            inverse_kinematics(0.1, 0.1, 0.1, self.nominal_r, self.wheelbase, self.track_width, self.max_speed,
                               wheel_radius_correction=[1.0, nan, 1.0, 1.0])
        with self.assertRaises(ValueError):
            inverse_kinematics(0.1, 0.1, 0.1, self.nominal_r, self.wheelbase, self.track_width, self.max_speed,
                               wheel_radii=[0.03, 0.03, nan, 0.03])

        # Forward kinematics
        with self.assertRaises(ValueError):
            forward_kinematics([nan, 0.0, 0.0, 0.0], self.nominal_r, self.wheelbase, self.track_width)
        with self.assertRaises(ValueError):
            forward_kinematics([0.0, 0.0, 0.0, 0.0], nan, self.wheelbase, self.track_width)
        with self.assertRaises(ValueError):
            forward_kinematics([0.0, 0.0, 0.0, 0.0], self.nominal_r, nan, self.track_width)
        with self.assertRaises(ValueError):
            forward_kinematics([0.0, 0.0, 0.0, 0.0], self.nominal_r, self.wheelbase, nan)
        with self.assertRaises(ValueError):
            forward_kinematics([0.0, 0.0, 0.0, 0.0], self.nominal_r, self.wheelbase, self.track_width,
                               wheel_radius_correction=[1.0, 1.0, nan, 1.0])

    def test_fuzz_inf_rejection_in_all_parameters(self):
        """Verify that +/-Inf in any parameter raises ValueError and does not crash."""
        for inf_val in [float('inf'), float('-inf')]:
            with self.assertRaises(ValueError):
                inverse_kinematics(inf_val, 0.0, 0.0, self.nominal_r, self.wheelbase, self.track_width, self.max_speed)
            with self.assertRaises(ValueError):
                inverse_kinematics(0.0, inf_val, 0.0, self.nominal_r, self.wheelbase, self.track_width, self.max_speed)
            with self.assertRaises(ValueError):
                inverse_kinematics(0.0, 0.0, inf_val, self.nominal_r, self.wheelbase, self.track_width, self.max_speed)
            with self.assertRaises(ValueError):
                inverse_kinematics(0.1, 0.1, 0.1, inf_val, self.wheelbase, self.track_width, self.max_speed)
            with self.assertRaises(ValueError):
                inverse_kinematics(0.1, 0.1, 0.1, self.nominal_r, inf_val, self.track_width, self.max_speed)
            with self.assertRaises(ValueError):
                inverse_kinematics(0.1, 0.1, 0.1, self.nominal_r, self.wheelbase, inf_val, self.max_speed)
            with self.assertRaises(ValueError):
                inverse_kinematics(0.1, 0.1, 0.1, self.nominal_r, self.wheelbase, self.track_width, inf_val)
            with self.assertRaises(ValueError):
                forward_kinematics([inf_val, 0.0, 0.0, 0.0], self.nominal_r, self.wheelbase, self.track_width)
            with self.assertRaises(ValueError):
                forward_kinematics([0.0, 0.0, 0.0, 0.0], inf_val, self.wheelbase, self.track_width)

    def test_fuzz_negative_and_zero_geometry_rejection(self):
        """Verify that zero or negative physical geometry raises ValueError."""
        bad_values = [0.0, -0.0, -0.001, -1.0, -100.0]
        for val in bad_values:
            # Wheel radius <= 0
            with self.assertRaises(ValueError):
                inverse_kinematics(0.1, 0.0, 0.0, val, self.wheelbase, self.track_width, self.max_speed)
            with self.assertRaises(ValueError):
                forward_kinematics([1.0, 1.0, 1.0, 1.0], val, self.wheelbase, self.track_width)

            # Wheelbase <= 0
            with self.assertRaises(ValueError):
                inverse_kinematics(0.1, 0.0, 0.0, self.nominal_r, val, self.track_width, self.max_speed)
            with self.assertRaises(ValueError):
                forward_kinematics([1.0, 1.0, 1.0, 1.0], self.nominal_r, val, self.track_width)

            # Track width <= 0
            with self.assertRaises(ValueError):
                inverse_kinematics(0.1, 0.0, 0.0, self.nominal_r, self.wheelbase, val, self.max_speed)
            with self.assertRaises(ValueError):
                forward_kinematics([1.0, 1.0, 1.0, 1.0], self.nominal_r, self.wheelbase, val)

            # Max wheel speed <= 0
            with self.assertRaises(ValueError):
                inverse_kinematics(0.1, 0.0, 0.0, self.nominal_r, self.wheelbase, self.track_width, val)

            # Kr elements <= 0
            with self.assertRaises(ValueError):
                inverse_kinematics(0.1, 0.0, 0.0, self.nominal_r, self.wheelbase, self.track_width, self.max_speed,
                                   wheel_radius_correction=[1.0, val, 1.0, 1.0])
            with self.assertRaises(ValueError):
                forward_kinematics([1.0, 1.0, 1.0, 1.0], self.nominal_r, self.wheelbase, self.track_width,
                                   wheel_radius_correction=[1.0, 1.0, val, 1.0])

            # Wheel radii elements <= 0
            with self.assertRaises(ValueError):
                inverse_kinematics(0.1, 0.0, 0.0, self.nominal_r, self.wheelbase, self.track_width, self.max_speed,
                                   wheel_radii=[0.03, 0.03, val, 0.03])
            with self.assertRaises(ValueError):
                forward_kinematics([1.0, 1.0, 1.0, 1.0], self.nominal_r, self.wheelbase, self.track_width,
                                   wheel_radii=[val, 0.03, 0.03, 0.03])

    def test_fuzz_subnormal_velocities(self):
        """Verify that subnormal floating-point numbers do not crash and round-trip stably."""
        subnormals = [
            sys.float_info.min * 1e-1,
            sys.float_info.min * 1e-5,
            sys.float_info.min * 1e-10,
            1e-315,
            1e-320,
        ]
        for sub in subnormals:
            # Check subnormal twist round-trip
            w = inverse_kinematics(sub, -sub, sub, self.nominal_r, self.wheelbase, self.track_width, 1e9)
            rec = forward_kinematics(w, self.nominal_r, self.wheelbase, self.track_width)
            err = math.hypot(rec[0] - sub, rec[1] - (-sub), rec[2] - sub)
            self.assertLess(err, 1e-15, f"Subnormal round-trip error {err} exceeds tolerance")

    def test_fuzz_malformed_shapes_and_structures(self):
        """Verify that dimension mismatches and non-diagonal matrices are rejected."""
        # Incorrect sequence lengths for Kr
        bad_kr_seqs = [
            [],
            [1.0],
            [1.0, 1.0],
            [1.0, 1.0, 1.0],
            [1.0, 1.0, 1.0, 1.0, 1.0],
            [1.0] * 8,
        ]
        for bad_kr in bad_kr_seqs:
            with self.assertRaises(ValueError):
                inverse_kinematics(0.1, 0.1, 0.1, self.nominal_r, self.wheelbase, self.track_width, self.max_speed,
                                   wheel_radius_correction=bad_kr)
            with self.assertRaises(ValueError):
                forward_kinematics([1.0, 1.0, 1.0, 1.0], self.nominal_r, self.wheelbase, self.track_width,
                                   wheel_radius_correction=bad_kr)

        # Incorrect matrix dimensions
        bad_matrices = [
            np.eye(3),
            np.eye(5),
            np.ones((4, 3)),
            np.ones((3, 4)),
            np.ones((4, 4, 1)),
        ]
        for bad_mat in bad_matrices:
            with self.assertRaises(ValueError):
                inverse_kinematics(0.1, 0.1, 0.1, self.nominal_r, self.wheelbase, self.track_width, self.max_speed,
                                   wheel_radius_correction=bad_mat)

        # Non-diagonal (4, 4) matrices
        for (r, c) in [(0, 1), (1, 0), (2, 3), (3, 2), (0, 3)]:
            non_diag = np.eye(4)
            non_diag[r, c] = 0.05
            with self.assertRaises(ValueError):
                inverse_kinematics(0.1, 0.1, 0.1, self.nominal_r, self.wheelbase, self.track_width, self.max_speed,
                                   wheel_radius_correction=non_diag)

        # Incorrect wheel speed sequence lengths for forward_kinematics
        bad_wheel_speeds = [
            [],
            [1.0],
            [1.0, 2.0],
            [1.0, 2.0, 3.0],
            [1.0, 2.0, 3.0, 4.0, 5.0],
            [1.0] * 8,
        ]
        for bad_speeds in bad_wheel_speeds:
            with self.assertRaises(ValueError):
                forward_kinematics(bad_speeds, self.nominal_r, self.wheelbase, self.track_width)

    def test_extreme_aspect_ratios_and_boundary_conditions(self):
        """Verify consistency across extreme physical aspect ratios and high speeds."""
        extreme_cases = [
            # Elongated robot (L >> W)
            (0.5, 0.05, 0.03),
            # Wide robot (W >> L)
            (0.05, 0.5, 0.03),
            # Miniature AGV
            (0.05, 0.05, 0.01),
            # Large industrial AGV
            (2.0, 1.5, 0.15),
        ]
        test_twists = [
            (0.0, 0.0, 0.0),             # exact zero
            (10.0, 0.0, 0.0),            # extreme linear X
            (0.0, -10.0, 0.0),           # extreme linear Y
            (0.0, 0.0, 25.0),            # extreme spin
            (10.0, -10.0, 20.0),         # combined high dynamic
        ]
        kr = [0.85, 1.15, 0.90, 1.10]

        for (L, W, R) in extreme_cases:
            for vx, vy, wz in test_twists:
                w = inverse_kinematics(vx, vy, wz, R, L, W, 1e9, wheel_radius_correction=kr)
                rec = forward_kinematics(w, R, L, W, wheel_radius_correction=kr)
                err = math.sqrt((rec[0] - vx)**2 + (rec[1] - vy)**2 + (rec[2] - wz)**2)
                self.assertLess(err, 1e-5, f"Extreme geometry (L={L}, W={W}, R={R}) error {err} >= 1e-5")


if __name__ == '__main__':
    unittest.main(verbosity=2)
