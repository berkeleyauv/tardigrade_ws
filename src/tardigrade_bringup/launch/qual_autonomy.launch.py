import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    vectornav_port = LaunchConfiguration('vectornav_port')
    vectornav_baud = LaunchConfiguration('vectornav_baud')
    esp_port = LaunchConfiguration('esp_port')
    thruster_config = LaunchConfiguration('thruster_config')
    filtered_odom_topic = LaunchConfiguration('filtered_odom_topic')
    dry_run = LaunchConfiguration('dry_run')
    startup_delay_sec = LaunchConfiguration('startup_delay_sec')
    target_depth_m = LaunchConfiguration('target_depth_m')
    forward_distance_m = LaunchConfiguration('forward_distance_m')
    forward_command = LaunchConfiguration('forward_command')
    descent_command = LaunchConfiguration('descent_command')
    yaw_kp = LaunchConfiguration('yaw_kp')
    yaw_kd = LaunchConfiguration('yaw_kd')
    max_yaw_command = LaunchConfiguration('max_yaw_command')

    zed_launch = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'launch',
        'zed_camera.launch.py',
    )
    ekf_launch = os.path.join(
        get_package_share_directory('tardigrade_bringup'),
        'launch',
        'zed_vectornav_ekf.launch.py',
    )
    control_launch = os.path.join(
        get_package_share_directory('tardigrade_control'),
        'launch', 'control_stack.launch.py')

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
        DeclareLaunchArgument(
            'filtered_odom_topic',
            default_value='/tardigrade/state/odometry/filtered',
        ),
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('startup_delay_sec', default_value='15.0'),
        DeclareLaunchArgument('target_depth_m', default_value='1.5'),
        DeclareLaunchArgument('forward_distance_m', default_value='20.0'),
        DeclareLaunchArgument('forward_command', default_value='0.20'),
        DeclareLaunchArgument('descent_command', default_value='0.20'),
        DeclareLaunchArgument('yaw_kp', default_value='0.4'),
        DeclareLaunchArgument('yaw_kd', default_value='0.1'),
        DeclareLaunchArgument('max_yaw_command', default_value='0.20'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_launch),
            launch_arguments={
                'camera_model': 'zed',
                'publish_tf': 'false',
            }.items(),
        ),
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
            parameters=[{'use_enu': False}],
        ),
        Node(
            package='tardigrade_state_estimation',
            executable='vectornav_imu_transform',
            name='vectornav_imu_transform',
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ekf_launch),
            launch_arguments={
                'imu_topic': '/tardigrade/sensors/imu',
                'filtered_odom_topic': filtered_odom_topic,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch),
            launch_arguments={
                'active_source': 'mission',
                'odometry_topic': filtered_odom_topic,
                'use_sim_time': 'false',
            }.items(),
        ),
        Node(
            package='tardigrade_esp',
            executable='esp_bridge',
            name='esp_bridge',
            output='screen',
            parameters=[{
                'serial_port': esp_port,
                'config_file': thruster_config,
                'cmd_timeout_sec': 0.5,
            }],
        ),
        Node(
            package='tardigrade_mission',
            executable='qual_test',
            name='qual_test',
            output='screen',
            parameters=[{
                'odometry_topic': filtered_odom_topic,
                'dry_run': dry_run,
                'startup_delay_sec': startup_delay_sec,
                'target_depth_m': target_depth_m,
                'forward_distance_m': forward_distance_m,
                'forward_command': forward_command,
                'descent_command': descent_command,
                'yaw_kp': yaw_kp,
                'yaw_kd': yaw_kd,
                'max_yaw_command': max_yaw_command,
            }],
            # Shut down the controller and bridge when the one-shot mission
            # completes or aborts. Their shutdown paths command neutral.
            on_exit=[Shutdown(reason='qual mission exited')],
        ),
    ])
