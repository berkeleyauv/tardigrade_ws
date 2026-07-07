from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pose_topic = LaunchConfiguration('pose_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    position_variance = LaunchConfiguration('position_variance')
    orientation_variance = LaunchConfiguration('orientation_variance')

    return LaunchDescription([
        DeclareLaunchArgument(
            'pose_topic',
            default_value='/zed/zed_node/pose',
            description='PoseStamped topic published by the ZED ROS 2 wrapper',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/tardigrade/state/odometry',
            description='Odometry topic consumed by the PX4 bridge',
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
            default_value='0.05',
            description='Diagonal covariance value for ZED orientation',
        ),

        Node(
            package='tardigrade_state_estimation',
            executable='zed_odometry',
            name='zed_odometry',
            output='screen',
            parameters=[{
                'pose_topic': pose_topic,
                'odom_topic': odom_topic,
                'odom_frame': odom_frame,
                'base_frame': base_frame,
                'position_variance': position_variance,
                'orientation_variance': orientation_variance,
            }],
        ),
    ])
