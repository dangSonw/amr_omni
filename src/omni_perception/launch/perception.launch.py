import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('omni_perception')
    default_laser_config = os.path.join(pkg_share, 'config', 'laser_filter.yaml')
    default_depth_config = os.path.join(pkg_share, 'config', 'depth_to_laser.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_depth = LaunchConfiguration('use_depth')
    laser_config = LaunchConfiguration('laser_config')
    depth_config = LaunchConfiguration('depth_config')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )
    declare_use_depth = DeclareLaunchArgument(
        'use_depth',
        default_value='false',
        description='Enable depthimage_to_laserscan conversion',
    )
    declare_laser_config = DeclareLaunchArgument(
        'laser_config',
        default_value=default_laser_config,
        description='Path to laser filter YAML configuration file',
    )
    declare_depth_config = DeclareLaunchArgument(
        'depth_config',
        default_value=default_depth_config,
        description='Path to depth to laser YAML configuration file',
    )

    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        parameters=[laser_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('scan', '/scan'),
            ('scan_filtered', '/scan_filtered'),
        ],
        output='screen',
    )

    depth_to_laser_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        parameters=[depth_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('image', '/camera/depth/image_raw'),
            ('camera_info', '/camera/depth/camera_info'),
            ('scan', '/depth_scan'),
        ],
        output='screen',
        condition=IfCondition(use_depth),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_depth,
        declare_laser_config,
        declare_depth_config,
        laser_filter_node,
        depth_to_laser_node,
    ])

