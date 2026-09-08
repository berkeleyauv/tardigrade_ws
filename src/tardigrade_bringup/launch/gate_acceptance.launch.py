"""End-to-end camera/perception/estimation/control gate acceptance run."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    sil_launch = os.path.join(
        get_package_share_directory('tardigrade_bringup'),
        'launch', 'unity_sil.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(sil_launch)),
        Node(
            package='tardigrade_sim',
            executable='acceptance_monitor',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='tardigrade_mission',
            executable='gate_mission',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'pass_gate_sec': 18.0,
                'timeout_sec': 60.0,
            }],
        ),
    ])
