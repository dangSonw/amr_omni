from setuptools import find_packages, setup

package_name = 'omni_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/hardware.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'stm32_bridge = omni_hardware.stm32_bridge:main',
    ]},
)