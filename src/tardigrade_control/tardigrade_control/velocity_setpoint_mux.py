"""Explicitly select one stamped velocity-command source."""

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Bool


SOURCES = ('manual', 'mission', 'pose')


def twist_values(message):
    """Return the six numeric components of a stamped twist."""
    return (
        message.twist.linear.x,
        message.twist.linear.y,
        message.twist.linear.z,
        message.twist.angular.x,
        message.twist.angular.y,
        message.twist.angular.z,
    )


class VelocitySetpointMux(Node):
    """Publish only the explicitly selected fresh command source."""

    def __init__(self, **node_kwargs):
        super().__init__('velocity_setpoint_mux', **node_kwargs)
        self.declare_parameter('active_source', 'mission')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter(
            'manual_enabled_topic', '/tardigrade/teleop/enabled')
        self.declare_parameter(
            'pose_enabled_topic',
            '/tardigrade/control/pose_setpoint_enabled')
        self.declare_parameter(
            'enabled_topic', '/tardigrade/control/velocity_setpoint_enabled')
        self.declare_parameter(
            'output_topic', '/tardigrade/control/velocity_setpoint')
        for source in SOURCES:
            self.declare_parameter(
                f'{source}_topic',
                f'/tardigrade/control/velocity_setpoint/{source}')

        self.active_source = str(
            self.get_parameter('active_source').value)
        if self.active_source not in SOURCES:
            raise ValueError(f'active_source must be one of {SOURCES}')
        self.timeout = max(
            0.0, float(
                self.get_parameter('command_timeout_sec').value))
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.latest = {source: None for source in SOURCES}
        self.received_ns = {source: None for source in SOURCES}
        self.manual_enabled = False
        self.manual_enabled_ns = None
        self.pose_enabled = False
        self.pose_enabled_ns = None
        self._source_subscriptions = [
            self.create_subscription(
                TwistStamped,
                str(self.get_parameter(f'{source}_topic').value),
                lambda message, selected=source:
                    self.on_command(selected, message),
                10,
            )
            for source in SOURCES
        ]
        self.publisher = self.create_publisher(
            TwistStamped,
            str(self.get_parameter('output_topic').value),
            10,
        )
        self.enabled_publisher = self.create_publisher(
            Bool,
            str(self.get_parameter('enabled_topic').value),
            10,
        )
        self.manual_enabled_sub = self.create_subscription(
            Bool,
            str(self.get_parameter('manual_enabled_topic').value),
            self.on_manual_enabled,
            10,
        )
        self.pose_enabled_sub = self.create_subscription(
            Bool,
            str(self.get_parameter('pose_enabled_topic').value),
            self.on_pose_enabled,
            10,
        )
        rate = max(1.0, float(
            self.get_parameter('publish_rate_hz').value))
        self.timer = self.create_timer(1.0 / rate, self.publish)
        self.add_on_set_parameters_callback(self.on_parameters)

    def on_parameters(self, parameters):
        candidate = self.active_source
        for parameter in parameters:
            if parameter.name != 'active_source':
                continue
            if str(parameter.value) not in SOURCES:
                return SetParametersResult(
                    successful=False,
                    reason=f'active_source must be one of {SOURCES}',
                )
            candidate = str(parameter.value)
        self.active_source = candidate
        return SetParametersResult(successful=True)

    def on_command(self, source, message):
        if message.header.frame_id not in ('', self.frame_id):
            self.get_logger().error(
                f'Rejected {source} command in '
                f'{message.header.frame_id}; expected {self.frame_id}')
            return
        if not all(math.isfinite(value) for value in twist_values(message)):
            self.get_logger().error(
                f'Rejected non-finite {source} command')
            return
        self.latest[source] = message
        self.received_ns[source] = self.get_clock().now().nanoseconds

    def on_manual_enabled(self, message):
        self.manual_enabled = bool(message.data)
        self.manual_enabled_ns = self.get_clock().now().nanoseconds

    def on_pose_enabled(self, message):
        self.pose_enabled = bool(message.data)
        self.pose_enabled_ns = self.get_clock().now().nanoseconds

    def enable_signal_is_fresh(self, received_ns, now_ns):
        return (
            received_ns is not None and now_ns >= received_ns and
            (now_ns - received_ns) / 1e9 <= self.timeout
        )

    def publish(self):
        now = self.get_clock().now()
        received_ns = self.received_ns[self.active_source]
        fresh = (
            received_ns is not None and now.nanoseconds >= received_ns and
            (now.nanoseconds - received_ns) / 1e9 <= self.timeout
        )
        if self.active_source == 'manual':
            manual_fresh = self.enable_signal_is_fresh(
                self.manual_enabled_ns, now.nanoseconds)
            fresh = fresh and manual_fresh and self.manual_enabled
        elif self.active_source == 'pose':
            pose_fresh = self.enable_signal_is_fresh(
                self.pose_enabled_ns, now.nanoseconds)
            fresh = fresh and pose_fresh and self.pose_enabled
        output = TwistStamped()
        output.header.stamp = now.to_msg()
        output.header.frame_id = self.frame_id
        if fresh:
            output.twist = self.latest[self.active_source].twist
        self.publisher.publish(output)
        enabled = Bool()
        enabled.data = fresh
        self.enabled_publisher.publish(enabled)


def main(args=None):
    rclpy.init(args=args)
    node = VelocitySetpointMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
