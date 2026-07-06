from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # This launch assumes the ZED wrapper is already publishing pose. It starts
    # VectorNav plus the node that combines ZED position with VectorNav attitude.
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')
    zed_pose_topic = LaunchConfiguration('zed_pose_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    position_variance = LaunchConfiguration('position_variance')
    orientation_variance = LaunchConfiguration('orientation_variance')
    angular_velocity_variance = LaunchConfiguration('angular_velocity_variance')
    imu_timeout_sec = LaunchConfiguration('imu_timeout_sec')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/serial/by-id/usb-FTDI_USB-RS232-WE_AV0LN035-if00-port0',
            description='Serial port for the VectorNav device',
        ),
        DeclareLaunchArgument(
            'baud',
            default_value='115200',
            description='Baud rate for the VectorNav device',
        ),
        DeclareLaunchArgument(
            'zed_pose_topic',
            default_value='/zed/zed_node/pose',
            description='PoseStamped topic published by the ZED ROS 2 wrapper',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/vectornav/imu',
            description='IMU topic published by the VectorNav driver',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/tardigrade/state/odometry',
            description='Odometry topic consumed by the PX4 MAVLink interface',
        ),
        DeclareLaunchArgument(
            'odom_frame',
            default_value='odom',
            description='Frame ID for the published odometry',
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Child frame ID for the published odometry',
        ),
        DeclareLaunchArgument(
            'position_variance',
            default_value='0.05',
            description='Diagonal covariance value for ZED position',
        ),
        DeclareLaunchArgument(
            'orientation_variance',
            default_value='0.02',
            description='Diagonal covariance value for VectorNav orientation',
        ),
        DeclareLaunchArgument(
            'angular_velocity_variance',
            default_value='0.02',
            description='Diagonal covariance value for VectorNav angular velocity',
        ),
        DeclareLaunchArgument(
            'imu_timeout_sec',
            default_value='0.25',
            description='Maximum VectorNav IMU age before falling back to ZED orientation',
        ),

        # Raw VectorNav driver: talks to the serial device.
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

        # Adapter from VectorNav-specific messages into standard sensor_msgs/Imu.
        Node(
            package='vectornav',
            executable='vn_sensor_msgs',
            name='vn_sensor_msgs',
            output='screen',
        ),

        # Fused odometry publisher consumed by mavlink_pixhawk_interface.
        Node(
            package='tardigrade_state_estimation',
            executable='zed_vectornav_odometry',
            name='zed_vectornav_odometry',
            output='screen',
            parameters=[{
                'zed_pose_topic': zed_pose_topic,
                'imu_topic': imu_topic,
                'odom_topic': odom_topic,
                'odom_frame': odom_frame,
                'base_frame': base_frame,
                'position_variance': position_variance,
                'orientation_variance': orientation_variance,
                'angular_velocity_variance': angular_velocity_variance,
                'imu_timeout_sec': imu_timeout_sec,
            }],
        ),
    ])
