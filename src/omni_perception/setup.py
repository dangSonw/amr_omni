from setuptools import find_packages, setup

package_name = 'omni_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/laser_filter.yaml',
            'config/depth_to_laser.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/perception.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AMR Omni Maintainers',
    maintainer_email='maintainer@example.com',
    description='Perception package for AMR Omni including laser scan filtering and depth processing.',
    license='Apache-2.0',
)

