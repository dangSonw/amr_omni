import math
import unittest
import numpy as np

from omni_control.kinematics import (
    forward_kinematics,
    inverse_kinematics,
    compute_wheel_speeds,
    compute_body_twist,
    validate_twist,
)


class KinematicsTest(unittest.TestCase):
    def test_zero(self):
        self.assertEqual(
            inverse_kinematics(0, 0, 0, 0.075, 0.42, 0.36, 18),
            (0, 0, 0, 0),
        )

    def test_forward_inverse_consistency(self):
        twist = (0.4, -0.2, 0.3)
        wheels = inverse_kinematics(*twist, 0.03, 0.1312, 0.1312, 100)
        result = forward_kinematics(wheels, 0.03, 0.1312, 0.1312)
        for expected, actual in zip(twist, result):
            self.assertTrue(math.isclose(expected, actual, rel_tol=1e-9))

    def test_saturation(self):
        wheels = inverse_kinematics(10, 10, 10, 0.075, 0.42, 0.36, 18)
        self.assertLessEqual(max(abs(value) for value in wheels), 18)

    def test_round_trip_consistency_nominal(self):
        test_twists = [
            (0.01, 0.0, 0.0),       # low speed
            (0.0, -0.02, 0.0),
            (0.0, 0.0, 0.05),
            (0.5, 0.0, 0.0),        # moderate speed
            (0.0, 0.4, 0.0),
            (0.0, 0.0, 1.5),
            (1.5, -1.0, 2.0),       # high speed (within 100 rad/s limit)
            (-0.8, 0.6, -1.2),
        ]
        for vx, vy, wz in test_twists:
            wheels = inverse_kinematics(vx, vy, wz, 0.03, 0.1312, 0.1312, 200.0)
            rec_vx, rec_vy, rec_wz = forward_kinematics(wheels, 0.03, 0.1312, 0.1312)
            err = math.sqrt((rec_vx - vx)**2 + (rec_vy - vy)**2 + (rec_wz - wz)**2)
            self.assertLess(err, 1e-5, f"Round trip error {err} exceeds 1e-5 for twist {(vx, vy, wz)}")

    def test_round_trip_consistency_with_kr_1d(self):
        # 1D array of calibration multipliers
        kr_1d = [1.03, 0.97, 1.01, 0.99]
        test_twists = [
            (0.02, 0.01, -0.01),
            (0.4, -0.2, 0.5),
            (-1.2, 0.8, 1.8),
        ]
        for vx, vy, wz in test_twists:
            wheels = inverse_kinematics(
                vx, vy, wz, 0.03, 0.1312, 0.1312, 200.0,
                wheel_radius_correction=kr_1d,
            )
            rec_vx, rec_vy, rec_wz = forward_kinematics(
                wheels, 0.03, 0.1312, 0.1312,
                wheel_radius_correction=kr_1d,
            )
            err = math.sqrt((rec_vx - vx)**2 + (rec_vy - vy)**2 + (rec_wz - wz)**2)
            self.assertLess(err, 1e-5, f"Round trip error with Kr 1D {err} exceeds 1e-5")

    def test_round_trip_consistency_with_kr_matrix(self):
        # 4x4 diagonal matrix Kr
        kr_matrix = np.diag([1.02, 0.98, 1.04, 0.96])
        twist = (0.5, -0.3, 0.8)
        wheels = inverse_kinematics(
            *twist, 0.03, 0.1312, 0.1312, 200.0,
            wheel_radius_correction=kr_matrix,
        )
        rec = forward_kinematics(
            wheels, 0.03, 0.1312, 0.1312,
            wheel_radius_correction=kr_matrix,
        )
        for expected, actual in zip(twist, rec):
            self.assertTrue(math.isclose(expected, actual, abs_tol=1e-5))

    def test_round_trip_with_wheel_radii_parameter(self):
        # Direct wheel radii in meters
        radii = (0.0305, 0.0295, 0.0300, 0.0298)
        twist = (0.3, 0.2, 0.5)
        wheels = inverse_kinematics(
            *twist, 0.03, 0.1312, 0.1312, 200.0,
            wheel_radii=radii,
        )
        rec = forward_kinematics(
            wheels, 0.03, 0.1312, 0.1312,
            wheel_radii=radii,
        )
        for expected, actual in zip(twist, rec):
            self.assertTrue(math.isclose(expected, actual, abs_tol=1e-5))

    def test_aliases(self):
        twist = (0.2, -0.1, 0.3)
        w1 = inverse_kinematics(*twist, 0.03, 0.1312, 0.1312, 50.0)
        w2 = compute_wheel_speeds(*twist, 0.03, 0.1312, 0.1312, 50.0)
        self.assertEqual(w1, w2)
        t1 = forward_kinematics(w1, 0.03, 0.1312, 0.1312)
        t2 = compute_body_twist(w1, 0.03, 0.1312, 0.1312)
        self.assertEqual(t1, t2)

    def test_nan_inf_twist_rejection(self):
        with self.assertRaises(ValueError):
            inverse_kinematics(float('nan'), 0.0, 0.0, 0.03, 0.1312, 0.1312, 50.0)
        with self.assertRaises(ValueError):
            inverse_kinematics(0.0, float('inf'), 0.0, 0.03, 0.1312, 0.1312, 50.0)
        with self.assertRaises(ValueError):
            inverse_kinematics(0.0, 0.0, float('-inf'), 0.03, 0.1312, 0.1312, 50.0)

    def test_nan_inf_wheel_speeds_rejection(self):
        with self.assertRaises(ValueError):
            forward_kinematics([float('nan'), 1.0, 2.0, 3.0], 0.03, 0.1312, 0.1312)
        with self.assertRaises(ValueError):
            forward_kinematics([1.0, float('inf'), 2.0, 3.0], 0.03, 0.1312, 0.1312)

    def test_invalid_wheel_radius_correction_values(self):
        # Non-positive multiplier
        with self.assertRaises(ValueError):
            inverse_kinematics(0.1, 0.0, 0.0, 0.03, 0.1312, 0.1312, 50.0,
                               wheel_radius_correction=[1.0, 0.0, 1.0, 1.0])
        with self.assertRaises(ValueError):
            inverse_kinematics(0.1, 0.0, 0.0, 0.03, 0.1312, 0.1312, 50.0,
                               wheel_radius_correction=[1.0, -0.5, 1.0, 1.0])
        # NaN / Inf in correction
        with self.assertRaises(ValueError):
            inverse_kinematics(0.1, 0.0, 0.0, 0.03, 0.1312, 0.1312, 50.0,
                               wheel_radius_correction=[1.0, float('nan'), 1.0, 1.0])
        with self.assertRaises(ValueError):
            forward_kinematics([1.0, 1.0, 1.0, 1.0], 0.03, 0.1312, 0.1312,
                               wheel_radius_correction=[1.0, float('inf'), 1.0, 1.0])

    def test_invalid_wheel_radius_correction_shapes(self):
        # Length 3 instead of 4
        with self.assertRaises(ValueError):
            inverse_kinematics(0.1, 0.0, 0.0, 0.03, 0.1312, 0.1312, 50.0,
                               wheel_radius_correction=[1.0, 1.0, 1.0])
        # Length 5
        with self.assertRaises(ValueError):
            inverse_kinematics(0.1, 0.0, 0.0, 0.03, 0.1312, 0.1312, 50.0,
                               wheel_radius_correction=[1.0, 1.0, 1.0, 1.0, 1.0])
        # Non-diagonal 4x4 matrix
        non_diag = np.eye(4)
        non_diag[0, 1] = 0.5
        with self.assertRaises(ValueError):
            inverse_kinematics(0.1, 0.0, 0.0, 0.03, 0.1312, 0.1312, 50.0,
                               wheel_radius_correction=non_diag)

    def test_invalid_geometry(self):
        with self.assertRaises(ValueError):
            inverse_kinematics(0.1, 0.0, 0.0, 0.0, 0.1312, 0.1312, 50.0)
        with self.assertRaises(ValueError):
            inverse_kinematics(0.1, 0.0, 0.0, -0.03, 0.1312, 0.1312, 50.0)
        with self.assertRaises(ValueError):
            inverse_kinematics(0.1, 0.0, 0.0, 0.03, 0.0, 0.1312, 50.0)
        with self.assertRaises(ValueError):
            inverse_kinematics(0.1, 0.0, 0.0, 0.03, 0.1312, -0.1, 50.0)
        with self.assertRaises(ValueError):
            inverse_kinematics(0.1, 0.0, 0.0, 0.03, 0.1312, 0.1312, 0.0)

    def test_standard_radius_pure_rotation(self):
        # With wz = 1.0 rad/s, vx = 0, vy = 0:
        # Standard radius = (0.1312 + 0.1312) / 2 = 0.1312 m
        # Linear wheel rim speed = radius * wz = 0.1312 m/s
        # Angular wheel speed = rim_speed / wheel_radius = 0.1312 / 0.03 = 4.37333 rad/s
        wheels = inverse_kinematics(0.0, 0.0, 1.0, 0.03, 0.1312, 0.1312, 50.0)
        expected_speed = 0.1312 / 0.03
        for w in wheels:
            self.assertAlmostEqual(w, expected_speed, places=5)

    def test_absolute_radius_guard(self):
        # Passing absolute radius [0.03, 0.03, 0.03, 0.03] should not double-multiply to 0.0009
        wheels_with_abs = inverse_kinematics(
            0.3, 0.0, 0.0, 0.03, 0.1312, 0.1312, 50.0,
            wheel_radius_correction=[0.03, 0.03, 0.03, 0.03]
        )
        wheels_nominal = inverse_kinematics(
            0.3, 0.0, 0.0, 0.03, 0.1312, 0.1312, 50.0
        )
        for w_abs, w_nom in zip(wheels_with_abs, wheels_nominal):
            self.assertAlmostEqual(w_abs, w_nom, places=5)


if __name__ == '__main__':
    unittest.main()