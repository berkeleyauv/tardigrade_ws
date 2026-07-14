from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    config_file = LaunchConfiguration('config_file')
    depth_kp = LaunchConfiguration('depth_kp')
    depth_ki = LaunchConfiguration('depth_ki')
    depth_kd = LaunchConfiguration('depth_kd')
    max_heave_command = LaunchConfiguration('max_heave_command')
    roll_kp = LaunchConfiguration('roll_kp')
    roll_kd = LaunchConfiguration('roll_kd')
    pitch_kp = LaunchConfiguration('pitch_kp')
    pitch_kd = LaunchConfiguration('pitch_kd')
    max_attitude_command = LaunchConfiguration('max_attitude_command')
    enable_depth_hold = LaunchConfiguration('enable_depth_hold')
    enable_attitude_hold = LaunchConfiguration('enable_attitude_hold')
    capture_initial_attitude_target = LaunchConfiguration(
        'capture_initial_attitude_target'
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument(
            'config_file',
            default_value='/ws/config/esp_thruster_map.json',
        ),
        DeclareLaunchArgument('depth_kp', default_value='0.8'),
        DeclareLaunchArgument('depth_ki', default_value='0.0'),
        DeclareLaunchArgument('depth_kd', default_value='0.25'),
        DeclareLaunchArgument('max_heave_command', default_value='0.35'),
        DeclareLaunchArgument('roll_kp', default_value='0.8'),
        DeclareLaunchArgument('roll_kd', default_value='0.15'),
        DeclareLaunchArgument('pitch_kp', default_value='0.8'),
        DeclareLaunchArgument('pitch_kd', default_value='0.15'),
        DeclareLaunchArgument('max_attitude_command', default_value='0.25'),
        DeclareLaunchArgument('enable_depth_hold', default_value='true'),
        DeclareLaunchArgument('enable_attitude_hold', default_value='true'),
        DeclareLaunchArgument(
            'capture_initial_attitude_target',
            default_value='true',
        ),
        Node(
            package='tardigrade_esp',
            executable='depth_attitude_controller',
            name='depth_attitude_controller',
            output='screen',
            parameters=[{
                'depth_kp': depth_kp,
                'depth_ki': depth_ki,
                'depth_kd': depth_kd,
                'max_heave_command': max_heave_command,
                'roll_kp': roll_kp,
                'roll_kd': roll_kd,
                'pitch_kp': pitch_kp,
                'pitch_kd': pitch_kd,
                'max_attitude_command': max_attitude_command,
                'enable_depth_hold': enable_depth_hold,
                'enable_attitude_hold': enable_attitude_hold,
                'capture_initial_attitude_target': capture_initial_attitude_target,
            }],
        ),
        Node(
            package='tardigrade_esp',
            executable='esp_thruster_bridge',
            name='esp_thruster_bridge',
            output='screen',
            parameters=[{
                'serial_port': serial_port,
                'config_file': config_file,
            }],
        ),
    ])
