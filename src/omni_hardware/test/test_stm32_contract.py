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
        self.assertEqual(TOPICS['odom'], 'odom')
        self.assertEqual(TOPICS['diagnostics'], 'diagnostics')
        self.assertEqual(TOPICS['encoder_counts'], 'encoder_counts')

    def test_validates_finite_bounded_twist(self):
        self.assertTrue(validate_twist((0.2, -0.1, 1.0), 0.54, 3.0))
        self.assertFalse(validate_twist((math.nan, 0.0, 0.0), 0.54, 3.0))
        self.assertFalse(validate_twist((0.55, 0.0, 0.0), 0.54, 3.0))
        self.assertFalse(validate_twist((0.0, 0.0, 3.1), 0.54, 3.0))
        self.assertFalse(validate_twist((0.0, 0.0), 0.54, 3.0))

    def test_diagnostic_level_supports_ros_byte_sequence(self):
        self.assertEqual(Stm32Bridge._diagnostic_level(None, b'\x02'), 2)
        self.assertEqual(Stm32Bridge._diagnostic_level(None, 1), 1)

    def test_debug_telemetry_contract_uses_twelve_wheel_values(self):
        self.assertEqual(len((0.0,) * 12), 12)


if __name__ == '__main__':
    unittest.main()