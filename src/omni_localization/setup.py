from setuptools import find_packages, setup

package_name = 'omni_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/ekf.yaml',
            'config/slam_params.yaml',
            'config/amcl_params.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/ekf.launch.py',
            'launch/slam.launch.py',
            'launch/localization.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
)

