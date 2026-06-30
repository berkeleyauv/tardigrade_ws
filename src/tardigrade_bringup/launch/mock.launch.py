from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tardigrade_px4',
            executable='mock_px4_status',
            name='mock_px4_status',
            output='screen'
        ),
        Node(
            package='tardigrade_px4',
            executable='pixhawk_interface',
            name='pixhawk_interface',
            output='screen'
        )
    ])