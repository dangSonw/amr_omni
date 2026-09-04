import math
import unittest

from omni_control.kinematics import forward_kinematics, inverse_kinematics


class KinematicsTest(unittest.TestCase):
    def test_zero(self):
        self.assertEqual(inverse_kinematics(0, 0, 0, .075, .42, .36, 18),
                         (0, 0, 0, 0))

    def test_forward_inverse_consistency(self):
        twist = (.4, -.2, .3)
        wheels = inverse_kinematics(*twist, .03, .1312, .1312, 100)
        result = forward_kinematics(wheels, .03, .1312, .1312)
        for expected, actual in zip(twist, result):
            self.assertTrue(math.isclose(expected, actual, rel_tol=1e-9))

    def test_saturation(self):
        wheels = inverse_kinematics(10, 10, 10, .075, .42, .36, 18)
        self.assertLessEqual(max(abs(value) for value in wheels), 18)


if __name__ == '__main__':
    unittest.main()