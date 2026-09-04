from setuptools import setup

package_name = 'omni_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', ['urdf/omni.urdf.xacro',
                                             'urdf/chassis.xacro',
                                             'urdf/wheels.xacro',
                                             'urdf/sensors.xacro']),
        ('share/' + package_name + '/meshes/4w', [
            'meshes/4w/base_link.dae',
            'meshes/4w/base_link.stl',
            'meshes/4w/camera.stl',
            'meshes/4w/omni_frame.stl',
            'meshes/4w/roller.stl',
            'meshes/4w/UPSTREAM_LICENSE.txt',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': []},
)