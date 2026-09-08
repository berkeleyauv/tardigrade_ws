"""One-semester compatibility bridge for the legacy control interfaces."""

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from tardigrade_interfaces.msg import ThrusterCommands


class LegacyInterfaceBridge(Node):
    def __init__(self):
        super().__init__('legacy_interface_bridge')
        self.velocity_publisher = self.create_publisher(
            TwistStamped,
            '/tardigrade/control/velocity_setpoint/mission',
            10)
        self.command_publisher = self.create_publisher(
            Float32MultiArray, '/tardigrade/thrusters/cmd', 10)
        self.velocity_subscriber = self.create_subscription(
            Twist, '/tardigrade/cmd_vel', self.on_legacy_velocity, 10)
        self.command_subscriber = self.create_subscription(
            ThrusterCommands,
            '/tardigrade/actuators/thruster_commands',
            self.on_commands,
            10,
        )
        self.warned_velocity = False
        self.warned_commands = False

    def on_legacy_velocity(self, message):
        if not self.warned_velocity:
            self.get_logger().warn(
                '/tardigrade/cmd_vel is deprecated; publish a '
                'a stamped manual/mission/pose velocity source instead')
            self.warned_velocity = True
        output = TwistStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = 'base_link'
        output.twist = message
        self.velocity_publisher.publish(output)

    def on_commands(self, message):
        if not self.warned_commands:
            self.get_logger().warn(
                '/tardigrade/thrusters/cmd compatibility mirror is active')
            self.warned_commands = True
        output = Float32MultiArray()
        output.data = list(message.setpoints)
        self.command_publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = LegacyInterfaceBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
