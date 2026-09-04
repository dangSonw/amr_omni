import math
import unittest

from omni_control.pid import ScalarKalman, WheelSpeedPid


class PidTest(unittest.TestCase):
    def test_kalman_rejects_non_finite_measurements(self):
        filter_value = ScalarKalman(0.1, 0.1)
        with self.assertRaises(ValueError):
            filter_value.update(math.nan, 0.01)

    def test_kalman_moves_toward_measurement(self):
        filter_value = ScalarKalman(0.1, 0.1)
        filter_value.reset(0.0)
        estimate = filter_value.update(1.0, 0.01)
        self.assertGreater(estimate, 0.0)
        self.assertLess(estimate, 1.0)

    def test_pid_is_bounded_and_resets(self):
        controller = WheelSpeedPid(1.0, 0.5, 0.0, 2.0)
        self.assertEqual(controller.update(10.0, 0.0, 0.01), 2.0)
        controller.reset()
        self.assertEqual(controller.integral, 0.0)
        self.assertFalse(controller.initialized)


if __name__ == '__main__':
    unittest.main()