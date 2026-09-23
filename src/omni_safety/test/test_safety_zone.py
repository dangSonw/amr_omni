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

    def test_directional_strafe_ignores_behind_obstacle(self):
        # 4 beams: Front (0 rad), Left (pi/2 rad), Back (pi rad), Right (-pi/2 rad)
        # Obstacle at Back (index 2) is 0.35m (inside stop zone 0.5m)
        # All other directions are clear (2.5m)
        # Robot is strafing to the Left: vx = 0.0, vy = 0.35 m/s
        ranges = [2.5, 2.5, 0.35, 2.5]
        # angle_min = 0, angle_inc = pi/2
        import math
        stop, factor, min_d = evaluate_safety_zone(
            ranges, angle_min=0.0, angle_increment=math.pi / 2.0,
            vx=0.0, vy=0.35, slow_zone_m=1.0, stop_zone_m=0.5
        )
        # Should NOT stop because obstacle is at Back (opposite to Left strafe) and > immediate bubble (0.2m)
        self.assertFalse(stop)
        self.assertEqual(factor, 1.0)

    def test_directional_forward_detects_front_obstacle(self):
        # Obstacle at Front (index 0) is 0.35m
        # Robot is moving forward: vx = 0.35 m/s, vy = 0.0
        ranges = [0.35, 2.5, 2.5, 2.5]
        import math
        stop, factor, min_d = evaluate_safety_zone(
            ranges, angle_min=0.0, angle_increment=math.pi / 2.0,
            vx=0.35, vy=0.0, slow_zone_m=1.0, stop_zone_m=0.5
        )
        # Must trigger safety stop
        self.assertTrue(stop)
        self.assertEqual(factor, 0.0)


if __name__ == '__main__':
    unittest.main()

