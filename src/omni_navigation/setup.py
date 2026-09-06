from setuptools import find_packages, setup

package_name = 'omni_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/nav2_params.yaml',
        ]),
        ('share/' + package_name + '/behavior_trees', [
            'behavior_trees/navigate_w_replanning_and_recovery.xml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/navigation.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
)

