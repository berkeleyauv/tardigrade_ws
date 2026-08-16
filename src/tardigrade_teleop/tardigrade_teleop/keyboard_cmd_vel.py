# Copyright 2026 Berkeley AUV
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


HELP = """
Keyboard teleop publishes short, automatically stopping command pulses.

w/s: pulse forward/back
j/l: pulse strafe left/right
r/f: pulse up/down
a/d: pulse yaw left/right
space: zero command
Ctrl-C: quit

Tap repeatedly for continued motion. Every pulse automatically returns to
zero; this is a checkout fallback, not the assisted/PID deadman interface.
"""


class KeyboardCmdVel(Node):
    def __init__(self):
        super().__init__('keyboard_cmd_vel')

        self.declare_parameter('cmd_vel_topic', '/tardigrade/cmd_vel')
        self.declare_parameter('linear_step', 0.1)
        self.declare_parameter('vertical_step', 0.05)
        self.declare_parameter('yaw_step', 0.2)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('command_hold_sec', 0.25)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.linear_step = float(self.get_parameter('linear_step').value)
        self.vertical_step = float(self.get_parameter('vertical_step').value)
        self.yaw_step = float(self.get_parameter('yaw_step').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.command_hold_sec = max(
            0.0, float(self.get_parameter('command_hold_sec').value))

        self.cmd = Twist()
        self.command_deadline_ns = None
        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.timer = self.create_timer(1.0 / max(publish_rate_hz, 1.0), self.tick)

        if not sys.stdin.isatty():
            raise RuntimeError('keyboard_cmd_vel requires an interactive terminal')

        # Single-key mode lets the command update without pressing Enter.
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        self.get_logger().info(f'Publishing keyboard cmd_vel: {self.cmd_vel_topic}')
        print(HELP)

    def destroy_node(self):
        if hasattr(self, 'settings'):
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        super().destroy_node()

    def tick(self):
        key = self.read_key()
        if key is not None:
            self.handle_key(key)

        now_ns = self.get_clock().now().nanoseconds
        if (self.command_deadline_ns is not None
                and now_ns >= self.command_deadline_ns):
            self.stop()

        self.pub.publish(self.cmd)

    def read_key(self):
        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not ready:
            return None
        return sys.stdin.read(1)

    def handle_key(self, key):
        # Body-frame ROS FLU convention: x forward, y left, z up, yaw positive left.
        command = Twist()
        motion_key = True
        if key == 'w':
            command.linear.x = self.linear_step
        elif key == 's':
            command.linear.x = -self.linear_step
        elif key == 'j':
            command.linear.y = self.linear_step
        elif key == 'l':
            command.linear.y = -self.linear_step
        elif key == 'r':
            command.linear.z = self.vertical_step
        elif key == 'f':
            command.linear.z = -self.vertical_step
        elif key == 'a':
            command.angular.z = self.yaw_step
        elif key == 'd':
            command.angular.z = -self.yaw_step
        elif key == ' ':
            self.stop()
            return
        else:
            motion_key = False

        if motion_key:
            self.cmd = command
            now_ns = self.get_clock().now().nanoseconds
            self.command_deadline_ns = (
                now_ns + int(self.command_hold_sec * 1e9))

    def stop(self):
        self.cmd = Twist()
        self.command_deadline_ns = None

    def publish_stop(self):
        self.stop()
        self.pub.publish(self.cmd)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardCmdVel()

    try:
        rclpy.spin(node)
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()
