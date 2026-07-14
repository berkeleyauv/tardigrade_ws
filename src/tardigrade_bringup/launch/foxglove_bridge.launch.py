from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    address = LaunchConfiguration('address')
    port = LaunchConfiguration('port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'address',
            default_value='0.0.0.0',
            description='Address for the Foxglove WebSocket server to bind.',
        ),
        DeclareLaunchArgument(
            'port',
            default_value='8765',
            description='Port for the Foxglove WebSocket server.',
        ),
        ExecuteProcess(
            cmd=[
                'ros2',
                'launch',
                'foxglove_bridge',
                'foxglove_bridge_launch.xml',
                ['address:=', address],
                ['port:=', port],
            ],
            output='screen',
        ),
    ])

