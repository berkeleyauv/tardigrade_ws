from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tardigrade_sim',
            executable='fake_unity_backend',
            name='fake_unity_backend',
            output='screen',
        ),
    ])
