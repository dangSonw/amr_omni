import unittest

from omni_safety.safety_zone import evaluate_safety_zone


class SafetyZoneTest(unittest.TestCase):
    def test_clear_path(self):
        ranges = [2.0, 3.5, 4.0, 1.8]
        stop, factor, min_d = evaluate_safety_zone(
            ranges, angle_min=0.0, angle_increment=0.1,
            slow_zone_m=1.0, stop_zone_m=0.5
        )
        self.assertFalse(stop)
        self.assertEqual(factor, 1.0)
        self.assertAlmostEqual(min_d, 1.8)

    def test_slow_down_zone(self):
        ranges = [2.5, 0.75, 3.0]
        stop, factor, min_d = evaluate_safety_zone(
            ranges, angle_min=0.0, angle_increment=0.1,
            slow_zone_m=1.0, stop_zone_m=0.5
        )
        self.assertFalse(stop)
        self.assertEqual(factor, 0.5)
        self.assertAlmostEqual(min_d, 0.75)

    def test_stop_zone(self):
        ranges = [2.0, 0.35, 1.5]
        stop, factor, min_d = evaluate_safety_zone(
            ranges, angle_min=0.0, angle_increment=0.1,
            slow_zone_m=1.0, stop_zone_m=0.5
        )
        self.assertTrue(stop)
        self.assertEqual(factor, 0.0)
        self.assertAlmostEqual(min_d, 0.35)

    def test_ignores_nan_and_inf_and_too_close(self):
        ranges = [float('nan'), float('inf'), 0.02, 1.2]
        stop, factor, min_d = evaluate_safety_zone(
            ranges, angle_min=0.0, angle_increment=0.1,
            slow_zone_m=1.0, stop_zone_m=0.5, min_valid_range_m=0.08
        )
        self.assertFalse(stop)
        self.assertEqual(factor, 1.0)
        self.assertAlmostEqual(min_d, 1.2)

    def test_invalid_parameters_raise_error(self):
        with self.assertRaises(ValueError):
            evaluate_safety_zone([1.0], 0.0, 0.1, slow_zone_m=0.5, stop_zone_m=0.8)


if __name__ == '__main__':
    unittest.main()

