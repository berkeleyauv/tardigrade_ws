"""Start ROS-TCP, estimation, perception, and the modern SIL control chain."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    control_launch = os.path.join(
        get_package_share_directory('tardigrade_control'),
        'launch', 'control_stack.launch.py')
    endpoint_launch = os.path.join(
        get_package_share_directory('ros_tcp_endpoint'),
        'launch', 'endpoint.py')
    estimator_launch = os.path.join(
        get_package_share_directory('tardigrade_bringup'),
        'launch', 'zed_vectornav_ekf.launch.py')
    description_file = os.path.join(
        get_package_share_directory('tardigrade_description'),
        'urdf', 'tardigrade.urdf')
    with open(description_file, 'r', encoding='utf-8') as stream:
        robot_description = stream.read()
    return LaunchDescription([
        DeclareLaunchArgument(
            'active_source', default_value='mission',
            description='Selected control source: manual, mission, or pose.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(endpoint_launch)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch),
            launch_arguments={
                'active_source': LaunchConfiguration('active_source'),
                'use_sim_time': 'true',
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(estimator_launch),
            launch_arguments={
                'zed_odom_topic': '/tardigrade/sensors/visual_odometry',
                'imu_topic': '/tardigrade/sensors/imu/data',
                'publish_vectornav_static_tf': 'false',
                'publish_zed_static_tf': 'false',
                'use_sim_time': 'true',
            }.items(),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
        ),
        Node(
            package='tardigrade_perception',
            executable='gate_detector',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
