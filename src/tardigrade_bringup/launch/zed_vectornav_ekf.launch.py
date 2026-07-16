import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    zed_odom_topic = LaunchConfiguration('zed_odom_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    filtered_odom_topic = LaunchConfiguration('filtered_odom_topic')
    map_frame = LaunchConfiguration('map_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    imu_frame = LaunchConfiguration('imu_frame')
    vectornav_x = LaunchConfiguration('vectornav_x')
    vectornav_y = LaunchConfiguration('vectornav_y')
    vectornav_z = LaunchConfiguration('vectornav_z')
    vectornav_qx = LaunchConfiguration('vectornav_qx')
    vectornav_qy = LaunchConfiguration('vectornav_qy')
    vectornav_qz = LaunchConfiguration('vectornav_qz')
    vectornav_qw = LaunchConfiguration('vectornav_qw')
    publish_tf = LaunchConfiguration('publish_tf')
    publish_vectornav_static_tf = LaunchConfiguration(
        'publish_vectornav_static_tf'
    )

    default_config = os.path.join(
        get_package_share_directory('tardigrade_bringup'),
        'config',
        'zed_vectornav_ekf.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='robot_localization EKF parameter file',
        ),
        DeclareLaunchArgument(
            'zed_odom_topic',
            default_value='/zed/zed_node/odom',
            description='ZED nav_msgs/Odometry input topic',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/vectornav/imu',
            description='VectorNav sensor_msgs/Imu input topic',
        ),
        DeclareLaunchArgument(
            'filtered_odom_topic',
            default_value='/tardigrade/state/odometry/filtered',
            description='Filtered odometry output topic',
        ),
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='Global frame name',
        ),
        DeclareLaunchArgument(
            'odom_frame',
            default_value='odom',
            description='Continuous local odometry frame name',
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Robot body frame name',
        ),
        DeclareLaunchArgument(
            'imu_frame',
            default_value='vectornav',
            description='Frame ID used by the VectorNav IMU messages',
        ),
        DeclareLaunchArgument(
            'vectornav_x',
            default_value='0.0',
            description='VectorNav x offset from base_link in meters',
        ),
        DeclareLaunchArgument(
            'vectornav_y',
            default_value='0.0',
            description='VectorNav y offset from base_link in meters',
        ),
        DeclareLaunchArgument(
            'vectornav_z',
            default_value='0.0',
            description='VectorNav z offset from base_link in meters',
        ),
        DeclareLaunchArgument(
            'vectornav_qx',
            default_value='0.70710678',
            description='base_link -> VectorNav quaternion x',
        ),
        DeclareLaunchArgument(
            'vectornav_qy',
            default_value='-0.70710678',
            description='base_link -> VectorNav quaternion y',
        ),
        DeclareLaunchArgument(
            'vectornav_qz',
            default_value='0.0',
            description='base_link -> VectorNav quaternion z',
        ),
        DeclareLaunchArgument(
            'vectornav_qw',
            default_value='0.0',
            description='base_link -> VectorNav quaternion w',
        ),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description='Publish the EKF odom -> base_link transform',
        ),
        DeclareLaunchArgument(
            'publish_vectornav_static_tf',
            default_value='true',
            description='Publish the base_link -> VectorNav static transform',
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_vectornav_tf',
            output='screen',
            condition=IfCondition(publish_vectornav_static_tf),
            arguments=[
                vectornav_x,
                vectornav_y,
                vectornav_z,
                vectornav_qx,
                vectornav_qy,
                vectornav_qz,
                vectornav_qw,
                base_frame,
                imu_frame,
            ],
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'odom0': zed_odom_topic,
                    'imu0': imu_topic,
                    'map_frame': map_frame,
                    'odom_frame': odom_frame,
                    'base_link_frame': base_frame,
                    'world_frame': odom_frame,
                    'publish_tf': publish_tf,
                },
            ],
            remappings=[
                ('odometry/filtered', filtered_odom_topic),
            ],
        ),
    ])
