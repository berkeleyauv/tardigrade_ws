"""Gate mission bringup (RoboSub 2026 Task 1).

Launches: VectorNav driver + odometry, gate perception, gate mission,
pixhawk interface, and odometry->PX4 bridge.

NOT launched here (start separately):
  - ZED wrapper (see docs/jetson_zed_px4_startup.md)
  - micro-ros / uXRCE-DDS agent for the Pixhawk

Usage:
  ros2 launch tardigrade_bringup gate.launch.py port:=/dev/ttyUSB1
Then:
  ros2 service call /tardigrade/set_armed tardigrade_interfaces/srv/SetArmed "{armed: true}"
  ros2 service call /tardigrade/set_external_control tardigrade_interfaces/srv/SetExternalControl "{enabled: true}"
  ros2 service call /tardigrade/mission/start std_srvs/srv/Trigger
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')
    image_topic = LaunchConfiguration('image_topic')

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
        DeclareLaunchArgument(
            'image_topic',
            default_value='/zed/zed_node/rgb/image_rect_color',
            description='Camera topic consumed by gate perception',
        ),

        # --- State estimation (VectorNav IMU) ---
        Node(
            package='vectornav',
            executable='vectornav',
            name='vectornav',
            output='screen',
            parameters=[{'port': port, 'baud': baud}],
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
                'odom_frame': 'odom',
                'base_frame': 'base_link',
            }],
        ),

        # --- PX4 bridge ---
        Node(
            package='tardigrade_px4',
            executable='pixhawk_interface',
            name='pixhawk_interface',
            output='screen',
            parameters=[{
                'cmd_vel_topic': '/tardigrade/cmd_vel',
                'odom_topic': '/tardigrade/state/odometry',
                'cmd_vel_timeout': 0.5,
            }],
        ),
        Node(
            package='tardigrade_px4',
            executable='odometry_to_px4',
            name='odometry_to_px4',
            output='screen',
            parameters=[{
                'odom_topic': '/tardigrade/state/odometry',
                'px4_odom_topic': '/fmu/in/vehicle_visual_odometry',
            }],
        ),

        # --- Perception ---
        Node(
            package='tardigrade_perception',
            executable='gate_detector',
            name='gate_detector',
            output='screen',
            parameters=[{
                'image_topic': image_topic,
                'detection_topic': '/tardigrade/perception/gate',
                'publish_debug_image': True,
                'max_rate_hz': 10.0,
                'resize_width': 640,
            }],
        ),

        # --- Mission ---
        Node(
            package='tardigrade_mission',
            executable='gate_mission',
            name='gate_mission',
            output='screen',
            parameters=[{
                'detection_topic': '/tardigrade/perception/gate',
                'odom_topic': '/tardigrade/state/odometry',
                'cmd_vel_topic': '/tardigrade/cmd_vel',
            }],
        ),
    ])
