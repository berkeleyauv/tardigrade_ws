import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def package_launch_file(name):
    return os.path.join(
        get_package_share_directory('tardigrade_bringup'),
        'launch',
        name,
    )


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(package_launch_file('mock.launch.py')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                package_launch_file('foxglove_rosbridge.launch.py')
            ),
        ),
    ])

