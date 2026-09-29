import os
import unittest
import xml.etree.ElementTree as ET
import yaml


class PerceptionFilesTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    def test_package_xml(self):
        pkg_xml = os.path.join(self.pkg_dir, 'package.xml')
        self.assertTrue(os.path.isfile(pkg_xml))
        tree = ET.parse(pkg_xml)
        root = tree.getroot()
        self.assertEqual(root.find('name').text, 'omni_perception')

    def test_configs_valid_yaml(self):
        configs = ['laser_filter.yaml', 'depth_to_laser.yaml', 'camera_detection.yaml']
        for config_name in configs:
            config_path = os.path.join(self.pkg_dir, 'config', config_name)
            self.assertTrue(os.path.isfile(config_path), f'Missing {config_name}')
            with open(config_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                self.assertIsInstance(data, dict, f'{config_name} should be valid YAML dict')

    def test_camera_detection_config_keys(self):
        config_path = os.path.join(self.pkg_dir, 'config', 'camera_detection.yaml')
        with open(config_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        params = data.get('camera_detector_node', {}).get('ros__parameters', {})
        self.assertIn('model', params)
        self.assertIn('confidence_threshold', params)
        self.assertIn('device', params)
        self.assertIn('publish_debug_image', params)
        self.assertIn('max_fps', params)

    def test_package_xml_dependencies(self):
        pkg_xml = os.path.join(self.pkg_dir, 'package.xml')
        tree = ET.parse(pkg_xml)
        root = tree.getroot()
        exec_depends = [elem.text for elem in root.findall('exec_depend')]
        self.assertIn('cv_bridge', exec_depends)
        self.assertIn('vision_msgs', exec_depends)

    def test_launch_files_exist(self):
        launches = ['perception.launch.py']
        for launch_name in launches:
            launch_path = os.path.join(self.pkg_dir, 'launch', launch_name)
            self.assertTrue(os.path.isfile(launch_path), f'Missing {launch_name}')


if __name__ == '__main__':
    unittest.main()

