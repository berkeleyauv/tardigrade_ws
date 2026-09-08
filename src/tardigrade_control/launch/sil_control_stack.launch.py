"""Compatibility wrapper for the shared control stack in simulation time."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    control_launch = os.path.join(
        get_package_share_directory('tardigrade_control'),
        'launch', 'control_stack.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),
    ])
