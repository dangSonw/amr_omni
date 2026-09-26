from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration('port')
    baudrate = LaunchConfiguration('baudrate')
    transport = LaunchConfiguration('transport')

    declare_port = DeclareLaunchArgument(
        'port',
        default_value='/dev/stm32',
        description='Serial port device path for micro-ROS agent (e.g. /dev/stm32 or /dev/ttyACM0)',
    )
    declare_baudrate = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baud rate for serial communication with STM32',
    )
    declare_transport = DeclareLaunchArgument(
        'transport',
        default_value='serial',
        description='micro-ROS transport type (serial, udp4, tcp4)',
    )

    agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=[transport, '--dev', port, '-b', baudrate],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription([
        declare_port,
        declare_baudrate,
        declare_transport,
        agent_node,
    ])
