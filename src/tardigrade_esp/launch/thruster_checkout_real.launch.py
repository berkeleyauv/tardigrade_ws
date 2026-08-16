"""Individual-thruster checkout; deliberately contains no mixer or PID."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('max_abs_command', default_value='0.10'),
        DeclareLaunchArgument('max_duration_sec', default_value='2.0'),
        Node(
            package='tardigrade_esp',
            executable='thruster_test',
            name='thruster_test',
            output='screen',
            parameters=[{
                'max_abs_command': ParameterValue(
                    LaunchConfiguration('max_abs_command'), value_type=float
                ),
                'max_duration_sec': ParameterValue(
                    LaunchConfiguration('max_duration_sec'), value_type=float
                ),
            }],
        ),
        Node(
            package='tardigrade_esp',
            executable='esp_bridge',
            name='esp_bridge',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud': ParameterValue(
                    LaunchConfiguration('baud'), value_type=int
                ),
                'cmd_timeout_sec': 0.5,
            }],
        ),
    ])
