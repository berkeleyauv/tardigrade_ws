"""Launch the Jetson-side control chain: controller + mixer.

  ros2 launch tardigrade_esp control_stack.launch.py

Brings up:
  depth_attitude_controller  (EKF odometry -> Twist wrench, gains from YAML)
  thruster_mixer             (wrench -> /tardigrade/thrusters/cmd)

Deliberately does NOT start a thruster backend — that's the sim/real seam.
Separately run ONE consumer of /tardigrade/thrusters/cmd:
  real robot:  esp_bridge (once its thruster-command subscription lands — F2)
  simulator:   the sim backend

Gains live in config/controller_gains.yaml — the versioned source of truth
shared by sim and real. Override at launch if needed:
  ros2 launch tardigrade_esp control_stack.launch.py gains_file:=/path/to.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_gains = os.path.join(
        get_package_share_directory('tardigrade_esp'),
        'config',
        'controller_gains.yaml',
    )

    gains_file = LaunchConfiguration('gains_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'gains_file',
            default_value=default_gains,
            description='Controller/mixer parameter YAML (the tuned gains).',
        ),
        Node(
            package='tardigrade_esp',
            executable='depth_attitude_controller',
            name='depth_attitude_controller',
            output='screen',
            parameters=[gains_file],
        ),
        Node(
            package='tardigrade_esp',
            executable='thruster_mixer',
            name='thruster_mixer',
            output='screen',
            parameters=[gains_file],
        ),
    ])
