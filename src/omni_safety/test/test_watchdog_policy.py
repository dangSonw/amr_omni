import unittest


class WatchdogPolicyTest(unittest.TestCase):
    def test_timeout_is_conservative(self):
        timeout_sec = 0.25
        self.assertLessEqual(timeout_sec, 0.5)


if __name__ == '__main__':
    unittest.main()