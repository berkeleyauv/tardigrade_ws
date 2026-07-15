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
            package='vectornav',
            executable='vn_sensor_msgs',
            name='vn_sensor_msgs',
            output='screen',
            parameters=[{
                # Keep the driver output in its native NED/FRD convention.
                # vectornav_odometry performs the single conversion to the
                # ROS ENU/FLU convention used by base_link.
                'use_enu': False,
            }],
        ),

        Node(
            package='tardigrade_state_estimation',
            executable='vectornav_odometry',
            name='vectornav_odometry',
            output='screen',
            parameters=[{
                'imu_topic': '/vectornav/imu',
                'odom_topic': '/tardigrade/state/odometry',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
            }],
        ),
    ])
