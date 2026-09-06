import math
import unittest
from unittest.mock import MagicMock

from geometry_msgs.msg import Twist
import rclpy
from rclpy.time import Duration
from std_msgs.msg import Bool

from omni_safety.command_watchdog import CommandWatchdog


class WatchdogPolicyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = CommandWatchdog()

    def tearDown(self):
        self.node.destroy_node()

    def test_initial_state_defaults(self):
        self.assertFalse(self.node.estop_active)
        self.assertEqual(self.node.last_command.linear.x, 0.0)
        self.assertLessEqual(self.node.timeout_sec, 0.5)

    def test_on_command_updates_valid_twist(self):
        cmd = Twist()
        cmd.linear.x = 0.35
        cmd.linear.y = -0.15
        cmd.angular.z = 1.2
        self.node._on_command(cmd)
        self.assertEqual(self.node.last_command.linear.x, 0.35)
        self.assertEqual(self.node.last_command.linear.y, -0.15)
        self.assertEqual(self.node.last_command.angular.z, 1.2)

    def test_on_command_rejects_nan_values(self):
        cmd = Twist()
        cmd.linear.x = 0.35
        self.node._on_command(cmd)

        invalid_cmd = Twist()
        invalid_cmd.linear.x = float('nan')
        self.node._on_command(invalid_cmd)
        # Should retain previous valid command
        self.assertEqual(self.node.last_command.linear.x, 0.35)

    def test_estop_flag_handling(self):
        estop_msg = Bool()
        estop_msg.data = True
        self.node._on_estop(estop_msg)
        self.assertTrue(self.node.estop_active)

        estop_msg.data = False
        self.node._on_estop(estop_msg)
        self.assertFalse(self.node.estop_active)

    def test_publish_stopped_when_estop_or_timed_out(self):
        self.node.publisher.publish = MagicMock()
        self.node.state_publisher.publish = MagicMock()

        # 1. Fresh command with no estop -> publishes command
        cmd = Twist()
        cmd.linear.x = 0.5
        self.node._on_command(cmd)
        self.node._publish()

        published_twist = self.node.publisher.publish.call_args[0][0]
        published_state = self.node.state_publisher.publish.call_args[0][0]
        self.assertEqual(published_twist.linear.x, 0.5)
        self.assertFalse(published_state.data)

        # 2. Timeout simulation (artificially age timestamp)
        self.node.last_command_time = self.node.get_clock().now() - Duration(seconds=1.0)
        self.node._publish()

        timeout_twist = self.node.publisher.publish.call_args[0][0]
        timeout_state = self.node.state_publisher.publish.call_args[0][0]
        self.assertEqual(timeout_twist.linear.x, 0.0)
        self.assertTrue(timeout_state.data)

        # 3. Active estop even with fresh command -> publishes zero twist
        self.node._on_command(cmd)
        self.node.estop_active = True
        self.node._publish()

        estop_twist = self.node.publisher.publish.call_args[0][0]
        estop_state = self.node.state_publisher.publish.call_args[0][0]
        self.assertEqual(estop_twist.linear.x, 0.0)
        self.assertTrue(estop_state.data)


if __name__ == '__main__':
    unittest.main()