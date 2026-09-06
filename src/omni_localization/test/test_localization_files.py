import os
import unittest
import xml.etree.ElementTree as ET
import yaml


class LocalizationFilesTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    def test_package_xml(self):
        pkg_xml = os.path.join(self.pkg_dir, 'package.xml')
        self.assertTrue(os.path.isfile(pkg_xml))
        tree = ET.parse(pkg_xml)
        root = tree.getroot()
        self.assertEqual(root.find('name').text, 'omni_localization')

    def test_configs_valid_yaml(self):
        configs = ['ekf.yaml', 'slam_params.yaml', 'amcl_params.yaml']
        for config_name in configs:
            config_path = os.path.join(self.pkg_dir, 'config', config_name)
            self.assertTrue(os.path.isfile(config_path), f'Missing {config_name}')
            with open(config_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                self.assertIsInstance(data, dict, f'{config_name} should be valid YAML dict')

    def test_launch_files_exist(self):
        launches = ['ekf.launch.py', 'slam.launch.py', 'localization.launch.py']
        for launch_name in launches:
            launch_path = os.path.join(self.pkg_dir, 'launch', launch_name)
            self.assertTrue(os.path.isfile(launch_path), f'Missing {launch_name}')


if __name__ == '__main__':
    unittest.main()

