import os
import unittest


class BringupFilesTest(unittest.TestCase):
    def test_launch_file_exists(self):
        root = os.path.dirname(os.path.dirname(__file__))
        sim_path = os.path.join(root, 'launch', 'simulation_bringup.launch.py')
        real_path = os.path.join(root, 'launch', 'real_robot_bringup.launch.py')
        self.assertTrue(os.path.isfile(sim_path))
        self.assertTrue(os.path.isfile(real_path))


if __name__ == '__main__':
    unittest.main()