from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tardigrade',
            executable='gate_approach',
            name='gate_approach',
            output='screen',
            remappings=[
                ('input', '/camera/image_raw'),  # update this if needed
                ('output', '/cmd_vel')           # where the output velocity goes
            ]
        )
    ])
