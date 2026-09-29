import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

WORLD_PRESETS = {
    'amr_lab': 'worlds/amr_lab.sdf',
    'outdoor': 'worlds/outdoor.sdf',
    'maze': 'worlds/maze.sdf',
}


def resolve_world_path(world_arg, simulation_share):
    """Resolve preset name, relative path, or absolute path to Gazebo SDF world."""
    if world_arg in WORLD_PRESETS:
        return os.path.join(simulation_share, WORLD_PRESETS[world_arg]), world_arg
    if os.path.isabs(world_arg):
        if not os.path.isfile(world_arg):
            raise FileNotFoundError(f"Gazebo world not found: {world_arg}")
        return world_arg, os.path.splitext(os.path.basename(world_arg))[0]
    if world_arg.startswith(('https://', 'http://')):
        return world_arg, 'remote'

    candidate = os.path.join(
        simulation_share, 'worlds',
        world_arg if world_arg.endswith('.sdf') else f"{world_arg}.sdf"
    )
    if os.path.isfile(candidate):
        return candidate, os.path.splitext(os.path.basename(candidate))[0]

    if os.path.isfile(world_arg):
        return os.path.abspath(world_arg), os.path.splitext(os.path.basename(world_arg))[0]

    raise FileNotFoundError(
        f"World '{world_arg}' not found. Available presets: {list(WORLD_PRESETS.keys())}"
    )


def launch_setup(context, *args, **kwargs):
    description_share = get_package_share_directory('omni_description')
    simulation_share = get_package_share_directory('omni_simulation')
    world_raw = LaunchConfiguration('world').perform(context).strip()
    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    use_joint_encoder_input = LaunchConfiguration('use_joint_encoder_input')
    use_imu_input = LaunchConfiguration('use_imu_input')
    simulation_mode = LaunchConfiguration('simulation_mode')
    user_spawn_x = LaunchConfiguration('spawn_x').perform(context).strip()
    user_spawn_y = LaunchConfiguration('spawn_y').perform(context).strip()
    user_spawn_z = LaunchConfiguration('spawn_z').perform(context).strip()

    world_file, preset_name = resolve_world_path(world_raw, simulation_share)

    # Determine default spawn poses per world preset
    if user_spawn_x != '':
        spawn_x = user_spawn_x
    elif preset_name == 'maze':
        spawn_x = '1.5'
    elif preset_name == 'outdoor':
        spawn_x = '25.0'
    else:
        spawn_x = '0.0'

    if user_spawn_y != '':
        spawn_y = user_spawn_y
    elif preset_name == 'maze':
        spawn_y = '1.5'
    elif preset_name == 'outdoor':
        spawn_y = '25.0'
    else:
        spawn_y = '0.0'

    spawn_z = user_spawn_z if user_spawn_z != '' else '0.018'

    xacro_file = os.path.join(description_share, 'urdf', 'omni.urdf.xacro')
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
        launch_arguments={'gz_args': [world_file, ' -r']}.items(),
        condition=UnlessCondition(headless),
    )
    gazebo_headless = IncludeLaunchDescription(
        gazebo_launch,
        launch_arguments={
            'gz_args': [world_file, ' -r -s --headless-rendering'],
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
        # Spawn at 0.018m: wheel bottom is at -0.0155m, gently placing robot 2.5mm above ground without impact bounce
        arguments=['-topic', 'robot_description', '-name', 'amr_omni',
                   '-x', spawn_x, '-y', spawn_y, '-z', '0.018'],
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
    return [
        gazebo_resource_path,
        gazebo,
        gazebo_headless,
        state_publisher,
        spawn,
        bridge,
        stm32_simulator,
    ]


def generate_launch_description():
    simulation_share = get_package_share_directory('omni_simulation')
    default_world = os.path.join(simulation_share, 'worlds', 'amr_lab.sdf')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value=default_world,
            description='World preset (amr_lab, outdoor, maze) or path to Gazebo Harmonic SDF world'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run the Gazebo server with off-screen rendering'),
        DeclareLaunchArgument('use_joint_encoder_input', default_value='true'),
        DeclareLaunchArgument('use_imu_input', default_value='true'),
        DeclareLaunchArgument('simulation_mode', default_value='true'),
        DeclareLaunchArgument('spawn_x', default_value='',
                              description='Optional initial X coordinate'),
        DeclareLaunchArgument('spawn_y', default_value='',
                              description='Optional initial Y coordinate'),
        DeclareLaunchArgument('spawn_z', default_value='',
                              description='Optional initial Z coordinate'),
        OpaqueFunction(function=launch_setup),
    ])
