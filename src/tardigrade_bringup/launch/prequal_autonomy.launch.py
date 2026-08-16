from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    vectornav_port = LaunchConfiguration('vectornav_port')
    vectornav_baud = LaunchConfiguration('vectornav_baud')
    esp_port = LaunchConfiguration('esp_port')
    thruster_config = LaunchConfiguration('thruster_config')
    dry_run = LaunchConfiguration('dry_run')
    startup_delay_sec = LaunchConfiguration('startup_delay_sec')
    target_depth_m = LaunchConfiguration('target_depth_m')
    forward_distance_m = LaunchConfiguration('forward_distance_m')
    forward_command = LaunchConfiguration('forward_command')
    descent_command = LaunchConfiguration('descent_command')
    descent_duration_sec = LaunchConfiguration('descent_duration_sec')
    outbound_duration_sec = LaunchConfiguration('outbound_duration_sec')
    return_duration_sec = LaunchConfiguration('return_duration_sec')
    yaw_kp = LaunchConfiguration('yaw_kp')
    yaw_kd = LaunchConfiguration('yaw_kd')
    max_yaw_command = LaunchConfiguration('max_yaw_command')
    depth_kp = LaunchConfiguration('depth_kp')
    depth_ki = LaunchConfiguration('depth_ki')
    depth_kd = LaunchConfiguration('depth_kd')
    roll_kp = LaunchConfiguration('roll_kp')
    roll_kd = LaunchConfiguration('roll_kd')
    pitch_kp = LaunchConfiguration('pitch_kp')
    pitch_kd = LaunchConfiguration('pitch_kd')

    return LaunchDescription([
        DeclareLaunchArgument(
            'vectornav_port',
            description='Stable /dev/serial/by-id path for VectorNav',
        ),
        DeclareLaunchArgument('vectornav_baud', default_value='115200'),
        DeclareLaunchArgument(
            'esp_port',
            description='Stable /dev/serial/by-id path for ESP32',
        ),
        DeclareLaunchArgument(
            'thruster_config',
            default_value='/ws/config/esp_thruster_map.json',
        ),
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('startup_delay_sec', default_value='60.0'),
        DeclareLaunchArgument('target_depth_m', default_value='1.5'),
        DeclareLaunchArgument('forward_distance_m', default_value='20.0'),
        DeclareLaunchArgument('forward_command', default_value='0.20'),
        DeclareLaunchArgument('descent_command', default_value='0.20'),
        DeclareLaunchArgument('descent_duration_sec', default_value='8.0'),
        DeclareLaunchArgument('outbound_duration_sec', default_value='40.0'),
        DeclareLaunchArgument('return_duration_sec', default_value='40.0'),
        DeclareLaunchArgument('yaw_kp', default_value='0.4'),
        DeclareLaunchArgument('yaw_kd', default_value='0.1'),
        DeclareLaunchArgument('max_yaw_command', default_value='0.20'),
        DeclareLaunchArgument('depth_kp', default_value='0.8'),
        DeclareLaunchArgument('depth_ki', default_value='0.0'),
        DeclareLaunchArgument('depth_kd', default_value='0.25'),
        DeclareLaunchArgument('roll_kp', default_value='0.8'),
        DeclareLaunchArgument('roll_kd', default_value='0.15'),
        DeclareLaunchArgument('pitch_kp', default_value='0.8'),
        DeclareLaunchArgument('pitch_kd', default_value='0.15'),

        Node(
            package='vectornav',
            executable='vectornav',
            name='vectornav',
            output='screen',
            parameters=[{'port': vectornav_port, 'baud': vectornav_baud}],
        ),
        Node(
            package='vectornav',
            executable='vn_sensor_msgs',
            name='vn_sensor_msgs',
            output='screen',
        ),
        Node(
            package='tardigrade_state_estimation',
            executable='vectornav_odometry',
            name='vectornav_odometry',
            output='screen',
            parameters=[{
                'imu_topic': '/vectornav/imu',
                'odom_topic': '/tardigrade/state/odometry',
            }],
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
                'roll_kp': roll_kp,
                'roll_kd': roll_kd,
                'pitch_kp': pitch_kp,
                'pitch_kd': pitch_kd,
                # The mission's angular.z is a desired heading rate; this
                # inner PID closes yaw directly from VectorNav orientation.
                'yaw_kp': 0.7,
                'yaw_ki': 0.03,
                'yaw_kd': 0.12,
                'max_yaw_command': max_yaw_command,
                # Intentionally gated off until the mission publishes the
                # controller enable heartbeat required by the pool stack.
                'enable_roll': False,
                'enable_pitch': False,
                'enable_yaw': False,
                'enable_depth': False,
            }],
        ),
        Node(
            package='tardigrade_esp',
            executable='esp_thruster_bridge',
            name='esp_thruster_bridge',
            output='screen',
            parameters=[{
                'serial_port': esp_port,
                'config_file': thruster_config,
                'startup_neutral_sec': 2.0,
            }],
        ),
        Node(
            package='tardigrade_mission',
            executable='prequal_test',
            name='prequal_test',
            output='screen',
            parameters=[{
                'dry_run': dry_run,
                'navigation_mode': 'imu_timed',
                'startup_delay_sec': startup_delay_sec,
                'target_depth_m': target_depth_m,
                'forward_distance_m': forward_distance_m,
                'forward_command': forward_command,
                'descent_command': descent_command,
                'descent_duration_sec': descent_duration_sec,
                'outbound_duration_sec': outbound_duration_sec,
                'return_duration_sec': return_duration_sec,
                'yaw_kp': yaw_kp,
                'yaw_kd': yaw_kd,
                'max_yaw_command': max_yaw_command,
            }],
            # Shut down the controller and bridge when the one-shot mission
            # completes or aborts. Their shutdown paths command neutral.
            on_exit=[Shutdown(reason='prequal mission exited')],
        ),
    ])
