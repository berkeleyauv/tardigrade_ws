"""Deadman-gated Xbox controller teleoperation for Tardigrade."""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


def shape_axis(value, deadzone):
    """Remove the deadzone and rescale the remaining travel to zero-to-one."""
    magnitude = abs(value)
    if magnitude <= deadzone:
        return 0.0
    scaled = (magnitude - deadzone) / max(1.0 - deadzone, 1e-6)
    return math.copysign(min(scaled, 1.0), value)


def joy_is_valid(axes, buttons, config):
    """Return false when required Xbox fields are absent or non-finite."""
    axis_indices = (
        config['surge_axis'], config['sway_axis'],
        config['heave_axis'], config['yaw_axis'],
    )
    if any(index < 0 or index >= len(axes) for index in axis_indices):
        return False
    button = config['deadman_button']
    if button < 0 or button >= len(buttons):
        return False
    return all(math.isfinite(float(axes[index])) for index in axis_indices)


def command_from_joy(axes, buttons, config):
    """Convert one Joy payload to a command and deadman state."""
    command = Twist()
    if not joy_is_valid(axes, buttons, config):
        return command, False
    if not bool(buttons[config['deadman_button']]):
        return command, False

    deadzone = config['deadzone']
    command.linear.x = config['max_surge'] * shape_axis(
        axes[config['surge_axis']], deadzone)
    command.linear.y = config['max_sway'] * shape_axis(
        axes[config['sway_axis']], deadzone)
    command.linear.z = config['max_heave'] * shape_axis(
        axes[config['heave_axis']], deadzone)
    command.angular.z = config['max_yaw'] * shape_axis(
        axes[config['yaw_axis']], deadzone)
    return command, True


def timestamp_is_fresh(last_ns, now_ns, timeout_sec):
    """Test a receive timestamp without depending on a ROS clock."""
    if last_ns is None or now_ns < last_ns:
        return False
    return (now_ns - last_ns) / 1e9 <= timeout_sec


class XboxCmdVel(Node):
    """Publish conservative commands only while Xbox LB is held."""

    def __init__(self, **node_kwargs):
        super().__init__('xbox_cmd_vel', **node_kwargs)
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('cmd_vel_topic', '/tardigrade/cmd_vel')
        self.declare_parameter(
            'enabled_topic', '/tardigrade/teleop/enabled')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('joy_timeout_sec', 0.25)
        self.declare_parameter('deadzone', 0.12)
        self.declare_parameter('deadman_button', 4)
        self.declare_parameter('surge_axis', 1)
        self.declare_parameter('sway_axis', 0)
        self.declare_parameter('heave_axis', 4)
        self.declare_parameter('yaw_axis', 3)
        self.declare_parameter('max_surge', 0.25)
        self.declare_parameter('max_sway', 0.25)
        self.declare_parameter('max_heave', 0.20)
        self.declare_parameter('max_yaw', 0.20)

        names = (
            'deadzone', 'deadman_button', 'surge_axis', 'sway_axis',
            'heave_axis', 'yaw_axis', 'max_surge', 'max_sway',
            'max_heave', 'max_yaw',
        )
        self._config = {
            name: self.get_parameter(name).value for name in names
        }
        self._timeout = max(
            0.0, float(self.get_parameter('joy_timeout_sec').value))
        rate = max(1.0, float(
            self.get_parameter('publish_rate_hz').value))
        joy_topic = self.get_parameter('joy_topic').value
        cmd_topic = self.get_parameter('cmd_vel_topic').value
        enabled_topic = self.get_parameter('enabled_topic').value

        self._command = Twist()
        self._last_joy_ns = None
        self._deadman = False
        self._warned_stale = False
        self._warned_malformed = False

        self._cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self._enabled_pub = self.create_publisher(Bool, enabled_topic, 10)
        self._joy_sub = self.create_subscription(
            Joy, joy_topic, self._on_joy, 10)
        self._timer = self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            f'Xbox teleop: {joy_topic} -> {cmd_topic}; hold LB to enable; '
            'left stick=surge/sway, right stick=heave/yaw')

    def _on_joy(self, message):
        valid = joy_is_valid(
            message.axes, message.buttons, self._config)
        self._last_joy_ns = self.get_clock().now().nanoseconds
        self._command, self._deadman = command_from_joy(
            message.axes, message.buttons, self._config)
        self._warned_stale = False
        if not valid and not self._warned_malformed:
            self.get_logger().warn(
                'malformed joystick input; publishing zero command')
            self._warned_malformed = True
        elif valid:
            self._warned_malformed = False

    def _fresh(self, now_ns):
        return timestamp_is_fresh(
            self._last_joy_ns, now_ns, self._timeout)

    def _publish(self):
        now_ns = self.get_clock().now().nanoseconds
        enabled = self._fresh(now_ns) and self._deadman
        self._cmd_pub.publish(self._command if enabled else Twist())
        enabled_message = Bool()
        enabled_message.data = enabled
        self._enabled_pub.publish(enabled_message)

        if (self._last_joy_ns is not None and not self._fresh(now_ns)
                and not self._warned_stale):
            self.get_logger().warn(
                'joystick data stale; publishing zero and disabling teleop')
            self._warned_stale = True

    def publish_stop(self):
        """Publish an explicit final disabled/zero sample."""
        self._cmd_pub.publish(Twist())
        enabled_message = Bool()
        enabled_message.data = False
        self._enabled_pub.publish(enabled_message)


def main(args=None):
    rclpy.init(args=args)
    node = XboxCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
