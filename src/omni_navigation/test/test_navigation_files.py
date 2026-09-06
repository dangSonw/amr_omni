import os
import unittest
import xml.etree.ElementTree as ET
import yaml


class NavigationFilesTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    def test_package_xml(self):
        pkg_xml = os.path.join(self.pkg_dir, 'package.xml')
        self.assertTrue(os.path.isfile(pkg_xml))
        tree = ET.parse(pkg_xml)
        root = tree.getroot()
        self.assertEqual(root.find('name').text, 'omni_navigation')

    def test_nav2_params_valid_yaml(self):
        config_path = os.path.join(self.pkg_dir, 'config', 'nav2_params.yaml')
        self.assertTrue(os.path.isfile(config_path))
        with open(config_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
            self.assertIsInstance(data, dict)
            self.assertIn('controller_server', data)
            self.assertIn('planner_server', data)
            self.assertIn('local_costmap', data)
            self.assertIn('global_costmap', data)

    def test_behavior_tree_valid_xml(self):
        bt_path = os.path.join(
            self.pkg_dir, 'behavior_trees',
            'navigate_w_replanning_and_recovery.xml'
        )
        self.assertTrue(os.path.isfile(bt_path))
        tree = ET.parse(bt_path)
        root = tree.getroot()
        self.assertEqual(root.tag, 'root')

    def test_launch_file_exists(self):
        launch_path = os.path.join(self.pkg_dir, 'launch', 'navigation.launch.py')
        self.assertTrue(os.path.isfile(launch_path))


if __name__ == '__main__':
    unittest.main()

