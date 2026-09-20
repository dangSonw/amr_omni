import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory('omni_description')
    hardware_share = get_package_share_directory('omni_hardware')
    safety_share = get_package_share_directory('omni_safety')
    localization_share = get_package_share_directory('omni_localization')
    navigation_share = get_package_share_directory('omni_navigation')
    perception_share = get_package_share_directory('omni_perception')

    xacro_file = os.path.join(description_share, 'urdf', 'omni.urdf.xacro')
    hardware_config = os.path.join(hardware_share, 'config', 'hardware.yaml')
    safety_config = os.path.join(safety_share, 'config', 'safety.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    debug_telemetry = LaunchConfiguration('debug_telemetry')
    debug_telemetry_frequency_hz = LaunchConfiguration('debug_telemetry_frequency_hz')

    web_enabled = LaunchConfiguration('web')
    web_port = LaunchConfiguration('web_port')
    localization_enabled = LaunchConfiguration('localization')
    slam_enabled = LaunchConfiguration('slam')
    nav_enabled = LaunchConfiguration('nav')
    perception_enabled = LaunchConfiguration('perception')

    robot_description = Command(['xacro ', xacro_file, ' headless:=true'])

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    stm32_bridge = Node(
        package='omni_hardware',
        executable='stm32_bridge',
        parameters=[hardware_config, {
            'enabled': True,
            'debug_telemetry': debug_telemetry,
            'debug_telemetry_frequency_hz': debug_telemetry_frequency_hz,
        }],
        output='screen',
    )

    safety_watchdog = Node(
        package='omni_safety',
        executable='command_watchdog',
        parameters=[safety_config],
        output='screen',
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(localization_share, 'launch', 'ekf.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(localization_enabled),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(localization_share, 'launch', 'slam.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(slam_enabled),
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation_share, 'launch', 'navigation.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(nav_enabled),
    )

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(perception_share, 'launch', 'perception.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(perception_enabled),
    )

    ws_root = os.environ.get('AMR_WORKSPACE') or os.getcwd()
    default_web_script = os.path.join(ws_root, 'web', 'backend', 'run_backend.py')
    default_venv_python = os.path.join(ws_root, 'web', 'backend', '.venv', 'bin', 'python3')
    default_python_bin = default_venv_python if os.path.isfile(default_venv_python) else 'python3'

    python_bin = LaunchConfiguration('python_bin')
    web_script = LaunchConfiguration('web_script')

    web_process = ExecuteProcess(
        cmd=[python_bin, web_script, '--port', web_port, '--mode', 'ros2'],
        output='screen',
        condition=IfCondition(web_enabled),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('debug_telemetry', default_value='false'),
        DeclareLaunchArgument('debug_telemetry_frequency_hz', default_value='2.0'),
        DeclareLaunchArgument('web', default_value='false'),
        DeclareLaunchArgument('web_port', default_value='8000'),
        DeclareLaunchArgument('web_script', default_value=default_web_script),
        DeclareLaunchArgument('python_bin', default_value=default_python_bin),
        DeclareLaunchArgument('localization', default_value='true'),
        DeclareLaunchArgument('slam', default_value='false'),
        DeclareLaunchArgument('nav', default_value='false'),
        DeclareLaunchArgument('perception', default_value='false'),
        state_publisher,
        safety_watchdog,
        stm32_bridge,
        perception_launch,
        ekf_launch,
        slam_launch,
        nav_launch,
        web_process,
    ])
