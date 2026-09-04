from setuptools import find_packages, setup

package_name = 'omni_safety'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/safety.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'command_watchdog = omni_safety.command_watchdog:main',
    ]},
)