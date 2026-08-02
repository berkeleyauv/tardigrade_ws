from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tardigrade_mock',
            executable='mock_cameras',
            name='mock_cameras',
            output='screen',
        ),
        Node(
            package='tardigrade_mock',
            executable='mock_controller',
            name='mock_controller',
            output='screen',
        ),
        Node(
            package='tardigrade_mock',
            executable='mock_robot_state',
            name='mock_robot_state',
            output='screen',
        ),
        Node(
            package='tardigrade_mock',
            executable='mock_esp_bridge',
            name='mock_esp_bridge',
            output='screen',
        ),
    ])
