"""Launch the backend-agnostic physical-unit control stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_parameters = os.path.join(
        get_package_share_directory('tardigrade_control'),
        'config', 'control.yaml')
    parameters = LaunchConfiguration('parameters')
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)
    active_source = LaunchConfiguration('active_source')
    odometry_topic = LaunchConfiguration('odometry_topic')
    common = {'use_sim_time': use_sim_time}
    feedback = {
        'use_sim_time': use_sim_time,
        'odometry_topic': odometry_topic,
    }
    return LaunchDescription([
        DeclareLaunchArgument(
            'parameters', default_value=default_parameters,
            description='Shared physical-unit control parameters.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'active_source', default_value='mission',
            description='Selected velocity source: manual, mission, or pose.'),
        DeclareLaunchArgument(
            'odometry_topic',
            default_value='/tardigrade/state/odometry/filtered'),
        Node(
            package='tardigrade_control',
            executable='velocity_setpoint_mux',
            output='screen',
            parameters=[parameters, common, {
                'active_source': active_source,
            }]),
        Node(
            package='tardigrade_control',
            executable='pose_velocity_controller',
            output='screen',
            parameters=[parameters, feedback]),
        Node(
            package='tardigrade_control',
            executable='velocity_wrench_controller',
            output='screen',
            parameters=[parameters, feedback]),
        Node(
            package='tardigrade_control',
            executable='thruster_allocator',
            output='screen',
            parameters=[parameters, common]),
        Node(
            package='tardigrade_control',
            executable='thruster_actuator_mapper',
            output='screen',
            parameters=[parameters, common]),
    ])
