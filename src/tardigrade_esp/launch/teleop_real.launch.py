# Copyright 2026 Berkeley AUV
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch the real-robot open-loop teleop backend.

The interactive keyboard node must run in its own terminal:
  ros2 run tardigrade_teleop keyboard_cmd_vel
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('tardigrade_esp'),
        'config',
        'esp_thruster_map.json',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='ESP device; prefer its /dev/serial/by-id path.',
        ),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('publish_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('cmd_timeout_sec', default_value='0.5'),
        Node(
            package='tardigrade_esp',
            executable='thruster_mixer',
            name='thruster_mixer',
            output='screen',
            parameters=[{
                'wrench_topic': '/tardigrade/cmd_vel',
                'output_topic': '/tardigrade/thrusters/cmd',
                'config_file': LaunchConfiguration('config_file'),
                'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
                'cmd_timeout_sec': LaunchConfiguration('cmd_timeout_sec'),
            }],
        ),
        Node(
            package='tardigrade_esp',
            executable='esp_bridge',
            name='esp_bridge',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud': LaunchConfiguration('baud'),
                'poll_rate_hz': LaunchConfiguration('publish_rate_hz'),
                'cmd_timeout_sec': LaunchConfiguration('cmd_timeout_sec'),
            }],
        ),
    ])
