import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory('omni_description')
    simulation_share = get_package_share_directory('omni_simulation')
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    use_joint_encoder_input = LaunchConfiguration('use_joint_encoder_input')
    use_imu_input = LaunchConfiguration('use_imu_input')
    simulation_mode = LaunchConfiguration('simulation_mode')
    xacro_file = os.path.join(description_share, 'urdf', 'omni.urdf.xacro')
    world_file = os.path.join(simulation_share, 'worlds', 'amr_lab.sdf')
    simulation_config = os.path.join(
        simulation_share, 'config', 'simulation.yaml')
    robot_description = Command([
        'xacro ', xacro_file, ' headless:=', headless,
    ])
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.dirname(description_share), ':',
               os.path.join(description_share, 'meshes')],
    )

    gazebo_launch = PythonLaunchDescriptionSource(PathJoinSubstitution([
        get_package_share_directory('ros_gz_sim'), 'launch',
        'gz_sim.launch.py'
    ]))
    gazebo = IncludeLaunchDescription(
        gazebo_launch,
        launch_arguments={'gz_args': [world, ' -r']}.items(),
        condition=UnlessCondition(headless),
    )
    gazebo_headless = IncludeLaunchDescription(
        gazebo_launch,
        launch_arguments={
            'gz_args': [world, ' -r -s --headless-rendering'],
        }.items(),
        condition=IfCondition(headless),
    )
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                robot_description, value_type=str),
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        # Match the upstream 4w launch: the wheel contact geometry starts at
        # z=0, so a 0.1 m spawn height prevents initial ground penetration.
        arguments=['-topic', 'robot_description', '-name', 'amr_omni',
                   '-z', '0.1'],
        output='screen',
    )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': os.path.join(
            simulation_share, 'config', 'gz_bridge.yaml')}],
        output='screen',
    )
    stm32_simulator = Node(
        package='omni_simulation',
        executable='stm32_simulator',
        parameters=[simulation_config, {
            'use_sim_time': use_sim_time,
            'use_joint_encoder_input': use_joint_encoder_input,
            'use_imu_input': use_imu_input,
            'simulation_mode': simulation_mode,
        }],
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value=world_file,
            description='Absolute path to a Gazebo Harmonic SDF world'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run the Gazebo server with off-screen rendering'),
        DeclareLaunchArgument('use_joint_encoder_input', default_value='true'),
        DeclareLaunchArgument('use_imu_input', default_value='true'),
        DeclareLaunchArgument('simulation_mode', default_value='true'),
        gazebo_resource_path,
        gazebo,
        gazebo_headless,
        state_publisher,
        spawn,
        bridge,
        stm32_simulator,
    ])
