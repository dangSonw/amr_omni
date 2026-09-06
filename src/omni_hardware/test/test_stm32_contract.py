import math
import unittest

from omni_hardware.stm32_bridge import Stm32Bridge
from omni_hardware.stm32_contract import TOPICS, validate_twist


class Stm32ContractTest(unittest.TestCase):
    def test_topics_match_micro_ros_contract(self):
        self.assertEqual(TOPICS['input_cmd_vel'], 'cmd_vel')
        self.assertEqual(TOPICS['stm32_cmd_vel'], 'stm32_cmd_vel')
        self.assertNotIn('auto_goal', TOPICS)
        self.assertNotIn('mode', TOPICS)
        self.assertNotIn('parameters', TOPICS)
        self.assertEqual(TOPICS['wheel_odom'], 'wheel/odom')
        self.assertEqual(TOPICS['imu'], 'imu/data')
        self.assertEqual(TOPICS['status'], 'status')
        self.assertNotIn('diagnostics', TOPICS)
        self.assertNotIn('encoder_counts', TOPICS)
        self.assertNotIn('wheel_state', TOPICS)

    def test_validates_finite_bounded_twist(self):
        self.assertTrue(validate_twist((0.2, -0.1, 1.0), 0.54, 3.0))
        self.assertFalse(validate_twist((math.nan, 0.0, 0.0), 0.54, 3.0))
        self.assertFalse(validate_twist((0.55, 0.0, 0.0), 0.54, 3.0))
        self.assertFalse(validate_twist((0.0, 0.0, 3.1), 0.54, 3.0))
        self.assertFalse(validate_twist((0.0, 0.0), 0.54, 3.0))


if __name__ == '__main__':
    unittest.main()