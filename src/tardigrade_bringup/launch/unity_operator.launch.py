"""Start the interactive Unity SIL stack and its Foxglove connection."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory('tardigrade_bringup')
    sil_launch = os.path.join(
        bringup_share, 'launch', 'unity_sil.launch.py')
    foxglove_launch = os.path.join(
        bringup_share, 'launch', 'foxglove_rosbridge.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'active_source', default_value='manual',
            description='Selected control source for interactive testing.'),
        DeclareLaunchArgument(
            'rosbridge_port', default_value='9090',
            description='Host-visible Rosbridge port for Foxglove.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sil_launch),
            launch_arguments={
                'active_source': LaunchConfiguration('active_source'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(foxglove_launch),
            launch_arguments={
                'port': LaunchConfiguration('rosbridge_port'),
            }.items(),
        ),
    ])
