from setuptools import find_packages, setup

# ROS 2 libexec placement is configured in setup.cfg.

package_name = 'omni_simulation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/simulation.launch.py']),
        ('share/' + package_name + '/config', [
            'config/gz_bridge.yaml',
            'config/simulation.yaml',
        ]),
        ('share/' + package_name + '/worlds', ['worlds/amr_lab.sdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'stm32_simulator = omni_simulation.stm32_simulator:main',
    ]},
)