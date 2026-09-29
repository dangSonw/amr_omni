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

            ctrl_params = data['controller_server']['ros__parameters']['FollowPath']
            cost_critic = ctrl_params['CostCritic']
            self.assertEqual(cost_critic['cost_weight'], 2.5)
            self.assertEqual(cost_critic['near_collision_cost'], 220)
            self.assertEqual(cost_critic['critical_cost'], 280.0)
            self.assertTrue(cost_critic['consider_footprint'])

            local_inflation = data['local_costmap']['local_costmap']['ros__parameters']['inflation_layer']
            self.assertEqual(local_inflation['inflation_radius'], 0.14)
            self.assertEqual(local_inflation['cost_scaling_factor'], 3.0)

            global_inflation = data['global_costmap']['global_costmap']['ros__parameters']['inflation_layer']
            self.assertEqual(global_inflation['inflation_radius'], 0.15)
            self.assertEqual(global_inflation['cost_scaling_factor'], 3.0)

    def test_behavior_tree_valid_xml(self):
        bt_path = os.path.join(
            self.pkg_dir, 'behavior_trees',
            'navigate_w_replanning_and_recovery.xml'
        )
        self.assertTrue(os.path.isfile(bt_path))
        tree = ET.parse(bt_path)
        root = tree.getroot()
        self.assertEqual(root.tag, 'root')

        backup_node = tree.find('.//BackUp')
        self.assertIsNotNone(backup_node)
        self.assertEqual(backup_node.get('backup_dist'), '0.10')
        self.assertEqual(backup_node.get('backup_speed'), '0.08')

    def test_launch_file_exists(self):
        launch_path = os.path.join(self.pkg_dir, 'launch', 'navigation.launch.py')
        self.assertTrue(os.path.isfile(launch_path))


if __name__ == '__main__':
    unittest.main()

