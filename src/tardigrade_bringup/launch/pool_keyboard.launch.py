"""Direct keyboard backend: mixer -> ESP bridge.

Run keyboard_cmd_vel separately in an interactive terminal. ROS launch does
not reliably pass terminal input through to a child node.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


_ESP_PORT = (
    '/dev/serial/by-id/'
    'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
)


def generate_launch_description():
    default_map = os.path.join(
        get_package_share_directory('tardigrade_esp'),
        'config',
        'esp_thruster_map.json',
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value=_ESP_PORT),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('config_file', default_value=default_map),
        Node(
            package='tardigrade_esp',
            executable='thruster_mixer',
            name='thruster_mixer',
            output='screen',
            parameters=[{
                'config_file': LaunchConfiguration('config_file'),
                'cmd_timeout_sec': 0.5,
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
                    LaunchConfiguration('baud'), value_type=int),
                'cmd_timeout_sec': 0.5,
            }],
        ),
    ])
