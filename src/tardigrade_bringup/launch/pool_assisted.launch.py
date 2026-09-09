"""Pool assisted teleop: Xbox setpoints -> controller/mixer -> ESP."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


_ESP_PORT = (
    '/dev/serial/by-id/'
    'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
)


def generate_launch_description():
    esp_share = get_package_share_directory('tardigrade_esp')
    control_launch = os.path.join(
        get_package_share_directory('tardigrade_control'),
        'launch', 'control_stack.launch.py')
    default_map = os.path.join(
        esp_share, 'config', 'esp_thruster_map.json')
    device_id = ParameterValue(
        LaunchConfiguration('device_id'), value_type=int)

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value=_ESP_PORT),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('config_file', default_value=default_map),
        DeclareLaunchArgument(
            'start_joy_node',
            default_value='false',
            description=(
                'Start a Jetson Linux joy driver. Leave false when Foxglove '
                'on the operator laptop publishes /joy.'
            ),
        ),
        DeclareLaunchArgument('device_id', default_value='0'),
        DeclareLaunchArgument('joy_topic', default_value='/joy'),
        DeclareLaunchArgument('deadzone', default_value='0.12'),
        DeclareLaunchArgument('deadman_button', default_value='4'),
        # Defaults match foxglove-joystick's browser Xbox layout.
        DeclareLaunchArgument('surge_axis', default_value='1'),
        DeclareLaunchArgument('sway_axis', default_value='0'),
        DeclareLaunchArgument('heave_axis', default_value='3'),
        DeclareLaunchArgument('yaw_axis', default_value='2'),
        DeclareLaunchArgument('max_surge', default_value='0.25'),
        DeclareLaunchArgument('max_sway', default_value='0.25'),
        DeclareLaunchArgument('max_heave', default_value='0.20'),
        DeclareLaunchArgument('max_yaw', default_value='0.20'),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_joy_node')),
            parameters=[{
                'device_id': device_id,
                'deadzone': 0.0,
                'autorepeat_rate': 20.0,
            }],
        ),
        Node(
            package='tardigrade_teleop',
            executable='xbox_cmd_vel',
            name='xbox_cmd_vel',
            output='screen',
            parameters=[{
                'joy_topic': LaunchConfiguration('joy_topic'),
                'cmd_vel_topic': (
                    '/tardigrade/control/velocity_setpoint/manual'),
                'deadzone': ParameterValue(
                    LaunchConfiguration('deadzone'), value_type=float),
                'deadman_button': ParameterValue(
                    LaunchConfiguration('deadman_button'), value_type=int),
                'surge_axis': ParameterValue(
                    LaunchConfiguration('surge_axis'), value_type=int),
                'sway_axis': ParameterValue(
                    LaunchConfiguration('sway_axis'), value_type=int),
                'heave_axis': ParameterValue(
                    LaunchConfiguration('heave_axis'), value_type=int),
                'yaw_axis': ParameterValue(
                    LaunchConfiguration('yaw_axis'), value_type=int),
                'max_surge': ParameterValue(
                    LaunchConfiguration('max_surge'), value_type=float),
                'max_sway': ParameterValue(
                    LaunchConfiguration('max_sway'), value_type=float),
                'max_heave': ParameterValue(
                    LaunchConfiguration('max_heave'), value_type=float),
                'max_yaw': ParameterValue(
                    LaunchConfiguration('max_yaw'), value_type=float),
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch),
            launch_arguments={
                'active_source': 'manual',
                'use_sim_time': 'false',
            }.items(),
        ),
        Node(
            package='tardigrade_esp',
            executable='esp_bridge',
            name='esp_bridge',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud': ParameterValue(
                    LaunchConfiguration('baud'), value_type=int),
                'cmd_timeout_sec': 0.5,
                'config_file': LaunchConfiguration('config_file'),
            }],
        ),
    ])
