from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB1',
            description='Serial port for the VectorNav device',
        ),
        DeclareLaunchArgument(
            'baud',
            default_value='115200',
            description='Baud rate for the VectorNav device',
        ),

        Node(
            package='vectornav',
            executable='vectornav',
            name='vectornav',
            output='screen',
            parameters=[{
                'port': port,
                'baud': baud,
            }],
        ),

        Node(
            package='tardigrade_state_estimation',
            executable='vectornav_imu_transform',
            name='vectornav_imu_transform',
            output='screen',
            parameters=[{
                'input_topic': '/vectornav/imu',
                'output_topic': '/tardigrade/sensors/imu',
                'output_frame': 'base_link',
                # Installed mounting: sensor +X backward, +Y left, +Z down.
                'base_from_sensor_qx': 0.0,
                'base_from_sensor_qy': 1.0,
                'base_from_sensor_qz': 0.0,
                'base_from_sensor_qw': 0.0,
            }],
        ),

        Node(
            package='vectornav',
            executable='vn_sensor_msgs',
            name='vn_sensor_msgs',
            output='screen',
            parameters=[{
                # The dedicated transform node performs the one conversion.
                'use_enu': False,
            }],
        ),

        Node(
            package='tardigrade_state_estimation',
            executable='vectornav_odometry',
            name='vectornav_odometry',
            output='screen',
            parameters=[{
                'imu_topic': '/tardigrade/sensors/imu',
                'odom_topic': '/tardigrade/state/odometry',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
            }],
        ),
    ])
