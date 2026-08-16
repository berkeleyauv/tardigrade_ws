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
    yaw_kp = LaunchConfiguration('yaw_kp')
    yaw_ki = LaunchConfiguration('yaw_ki')
    yaw_kd = LaunchConfiguration('yaw_kd')
    max_roll_command = LaunchConfiguration('max_roll_command')
    max_pitch_command = LaunchConfiguration('max_pitch_command')
    max_yaw_command = LaunchConfiguration('max_yaw_command')
    enable_roll = LaunchConfiguration('enable_roll')
    enable_pitch = LaunchConfiguration('enable_pitch')
    enable_yaw = LaunchConfiguration('enable_yaw')
    enable_depth = LaunchConfiguration('enable_depth')
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
        DeclareLaunchArgument('yaw_kp', default_value='0.7'),
        DeclareLaunchArgument('yaw_ki', default_value='0.03'),
        DeclareLaunchArgument('yaw_kd', default_value='0.12'),
        DeclareLaunchArgument('max_roll_command', default_value='0.20'),
        DeclareLaunchArgument('max_pitch_command', default_value='0.20'),
        DeclareLaunchArgument('max_yaw_command', default_value='0.20'),
        DeclareLaunchArgument('enable_roll', default_value='false'),
        DeclareLaunchArgument('enable_pitch', default_value='false'),
        DeclareLaunchArgument('enable_yaw', default_value='false'),
        DeclareLaunchArgument('enable_depth', default_value='false'),
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
                'yaw_kp': yaw_kp,
                'yaw_ki': yaw_ki,
                'yaw_kd': yaw_kd,
                'max_roll_command': max_roll_command,
                'max_pitch_command': max_pitch_command,
                'max_yaw_command': max_yaw_command,
                'enable_roll': enable_roll,
                'enable_pitch': enable_pitch,
                'enable_yaw': enable_yaw,
                'enable_depth': enable_depth,
                'capture_initial_attitude_target': (
                    capture_initial_attitude_target
                ),
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
