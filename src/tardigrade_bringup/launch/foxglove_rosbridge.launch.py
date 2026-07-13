from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    port = LaunchConfiguration('port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='9090',
            description='Port for the rosbridge WebSocket server.',
        ),
        ExecuteProcess(
            cmd=[
                'ros2',
                'launch',
                'rosbridge_server',
                'rosbridge_websocket_launch.xml',
                ['port:=', port],
            ],
            output='screen',
        ),
    ])

