import os
import unittest
import xml.etree.ElementTree as element_tree


class DescriptionFilesTest(unittest.TestCase):
    def test_xacro_files_are_well_formed_xml(self):
        root = os.path.dirname(os.path.dirname(__file__))
        for filename in ('omni.urdf.xacro', 'chassis.xacro', 'wheels.xacro',
                         'sensors.xacro'):
            element_tree.parse(os.path.join(root, 'urdf', filename))

    def test_wheel_description_has_four_driven_omni_wheels(self):
        root = os.path.dirname(os.path.dirname(__file__))
        with open(os.path.join(root, 'urdf', 'wheels.xacro')) as stream:
            text = stream.read()
        self.assertEqual(text.count('type="continuous"'), 2)
        self.assertIn('roller_link_${suffix}', text)
        self.assertIn('roller_joint_${suffix}', text)
        self.assertEqual(text.count('<xacro:roller '), 6)
        self.assertIn('xyz="0.0656 0.0656 0.0145"', text)
        self.assertIn('package://omni_description/meshes/4w/omni_frame.stl',
                      text)
        self.assertEqual(text.count('<xacro:wheel '), 4)
        for wheel_index in ('1', '2', '3', '4'):
            self.assertIn('suffix="%s"' % wheel_index, text)


if __name__ == '__main__':
    unittest.main()