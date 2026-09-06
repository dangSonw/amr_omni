from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    simulation = os.path.join(
        get_package_share_directory('omni_simulation'), 'launch',
        'simulation.launch.py')
    simulation_share = get_package_share_directory('omni_simulation')
    world_file = os.path.join(simulation_share, 'worlds', 'amr_lab.sdf')
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    use_joint_encoder_input = LaunchConfiguration('use_joint_encoder_input')
    use_imu_input = LaunchConfiguration('use_imu_input')
    simulation_mode = LaunchConfiguration('simulation_mode')
    debug_telemetry = LaunchConfiguration('debug_telemetry')
    debug_telemetry_frequency_hz = LaunchConfiguration(
        'debug_telemetry_frequency_hz')
    safety_config = os.path.join(
        get_package_share_directory('omni_safety'), 'config', 'safety.yaml')
    hardware_config = os.path.join(
        get_package_share_directory('omni_hardware'), 'config',
        'hardware.yaml')

    web_enabled = LaunchConfiguration('web')
    web_port = LaunchConfiguration('web_port')
    localization_enabled = LaunchConfiguration('localization')
    slam_enabled = LaunchConfiguration('slam')
    nav_enabled = LaunchConfiguration('nav')
    perception_enabled = LaunchConfiguration('perception')

    localization_share = get_package_share_directory('omni_localization')
    navigation_share = get_package_share_directory('omni_navigation')
    perception_share = get_package_share_directory('omni_perception')

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_share, 'launch', 'ekf.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(localization_enabled),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_share, 'launch', 'slam.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(slam_enabled),
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_share, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(nav_enabled),
    )

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_share, 'launch', 'perception.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(perception_enabled),
    )

    # Định vị thư mục workspace và script web backend
    ws_root = os.environ.get('AMR_WORKSPACE') or os.getcwd()
    if not os.path.isfile(os.path.join(ws_root, 'web', 'backend', 'run_backend.py')):
        ws_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))

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
        DeclareLaunchArgument('world', default_value=world_file),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('use_joint_encoder_input', default_value='true'),
        DeclareLaunchArgument('use_imu_input', default_value='true'),
        DeclareLaunchArgument('simulation_mode', default_value='true'),
        DeclareLaunchArgument('debug_telemetry', default_value='false'),
        DeclareLaunchArgument(
            'debug_telemetry_frequency_hz', default_value='2.0'),
        DeclareLaunchArgument('web', default_value='false',
                              description='Launch FastAPI web monitoring & control interface'),
        DeclareLaunchArgument('web_port', default_value='8000',
                              description='Port for FastAPI web server'),
        DeclareLaunchArgument('web_script', default_value=default_web_script,
                              description='Path to web runner script'),
        DeclareLaunchArgument('python_bin', default_value=default_python_bin,
                              description='Path to Python interpreter for web'),
        DeclareLaunchArgument('localization', default_value='false',
                              description='Launch EKF odometry fusion'),
        DeclareLaunchArgument('slam', default_value='false',
                              description='Launch SLAM Toolbox for 2D mapping'),
        DeclareLaunchArgument('nav', default_value='false',
                              description='Launch Nav2 autonomous navigation stack'),
        DeclareLaunchArgument('perception', default_value='false',
                              description='Launch laser filtering and depth perception pipeline'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulation),
            launch_arguments={
                'world': world,
                'use_sim_time': use_sim_time,
                'headless': headless,
                'use_joint_encoder_input': use_joint_encoder_input,
                'use_imu_input': use_imu_input,
                'simulation_mode': simulation_mode,
            }.items(),
        ),
        Node(package='omni_safety', executable='command_watchdog',
             parameters=[safety_config], output='screen'),
        Node(package='omni_hardware', executable='stm32_bridge',
             parameters=[hardware_config, {
                 'debug_telemetry': debug_telemetry,
                 'debug_telemetry_frequency_hz': debug_telemetry_frequency_hz,
             }], output='screen'),
        perception_launch,
        ekf_launch,
        slam_launch,
        nav_launch,
        web_process,
    ])