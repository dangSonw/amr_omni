import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('omni_localization')
    default_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Full path to the EKF configuration yaml file',
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odometry/filtered')],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_config_file,
        ekf_node,
    ])

