import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


HELP = """
Keyboard teleop publishes /tardigrade/cmd_vel.

w/s: forward/back
j/l: strafe left/right
r/f: up/down
a/d: yaw left/right
space: zero command
Ctrl-C: quit
"""


class KeyboardCmdVel(Node):
    def __init__(self):
        super().__init__('keyboard_cmd_vel')

        # These are requested speeds. mavlink_pixhawk_interface applies the
        # final safety clamps, so this node is not the only safety boundary.
        self.declare_parameter('cmd_vel_topic', '/tardigrade/cmd_vel')
        self.declare_parameter('linear_step', 0.1)
        self.declare_parameter('vertical_step', 0.05)
        self.declare_parameter('yaw_step', 0.2)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.linear_step = float(self.get_parameter('linear_step').value)
        self.vertical_step = float(self.get_parameter('vertical_step').value)
        self.yaw_step = float(self.get_parameter('yaw_step').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.cmd = Twist()
        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.timer = self.create_timer(1.0 / max(publish_rate_hz, 1.0), self.tick)

        if not sys.stdin.isatty():
            raise RuntimeError('keyboard_cmd_vel requires an interactive terminal')

        # Put the terminal in single-key mode so keys are read immediately.
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        self.get_logger().info(f'Publishing keyboard cmd_vel: {self.cmd_vel_topic}')
        print(HELP)

    def destroy_node(self):
        if hasattr(self, 'settings'):
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        super().destroy_node()

    def tick(self):
        # Keep publishing the last command so the receiver sees a live stream.
        # Space resets the latched command to zero.
        key = self.read_key()
        if key is not None:
            self.handle_key(key)

        self.pub.publish(self.cmd)

    def read_key(self):
        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not ready:
            return None
        return sys.stdin.read(1)

    def handle_key(self, key):
        # Body-frame ROS FLU convention: x forward, y left, z up, yaw positive left.
        if key == 'w':
            self.cmd.linear.x = self.linear_step
        elif key == 's':
            self.cmd.linear.x = -self.linear_step
        elif key == 'j':
            self.cmd.linear.y = self.linear_step
        elif key == 'l':
            self.cmd.linear.y = -self.linear_step
        elif key == 'r':
            self.cmd.linear.z = self.vertical_step
        elif key == 'f':
            self.cmd.linear.z = -self.vertical_step
        elif key == 'a':
            self.cmd.angular.z = self.yaw_step
        elif key == 'd':
            self.cmd.angular.z = -self.yaw_step
        elif key == ' ':
            self.cmd = Twist()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardCmdVel()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
