import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('omni_localization')
    default_amcl_params = os.path.join(pkg_share, 'config', 'amcl_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    amcl_params_file = LaunchConfiguration('amcl_params_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )
    declare_map_yaml = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load',
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )
    declare_amcl_params = DeclareLaunchArgument(
        'amcl_params_file',
        default_value=default_amcl_params,
        description='Full path to the AMCL configuration yaml file',
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'ekf.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, {'yaml_filename': map_yaml_file}],
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params_file, {'use_sim_time': use_sim_time}],
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': ['map_server', 'amcl']},
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map_yaml,
        declare_autostart,
        declare_amcl_params,
        ekf_launch,
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
    ])

