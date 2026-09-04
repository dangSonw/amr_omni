import os
import unittest


class BringupFilesTest(unittest.TestCase):
    def test_launch_file_exists(self):
        root = os.path.dirname(os.path.dirname(__file__))
        path = os.path.join(root, 'launch', 'simulation_bringup.launch.py')
        self.assertTrue(os.path.isfile(path))


if __name__ == '__main__':
    unittest.main()