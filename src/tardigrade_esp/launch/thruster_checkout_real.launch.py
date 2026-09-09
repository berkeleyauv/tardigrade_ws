"""Individual-thruster checkout; deliberately contains no mixer or PID."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('tardigrade_esp'),
        'config', 'esp_thruster_map.json')
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('config_file', default_value=default_config),
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
                'config_file': LaunchConfiguration('config_file'),
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
                'config_file': LaunchConfiguration('config_file'),
            }],
        ),
    ])
