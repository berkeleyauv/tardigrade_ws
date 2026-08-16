#!/usr/bin/env python3
"""Bounded, one-at-a-time thruster checkout surface."""

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from tardigrade_interfaces.srv import TestThruster

NUM_THRUSTERS = 8


def validate_request(slot, command, duration_sec, max_command, max_duration):
    """Validate one bounded checkout request and return an error or ``None``."""
    if slot < 1 or slot > NUM_THRUSTERS:
        return f'slot must be 1..{NUM_THRUSTERS}'
    if not math.isfinite(command) or not math.isfinite(duration_sec):
        return 'command and duration_sec must be finite'
    if abs(command) > max_command:
        return f'abs(command) must be <= {max_command:.3f}'
    if duration_sec <= 0.0 or duration_sec > max_duration:
        return f'duration_sec must be > 0 and <= {max_duration:.2f}'
    return None


class ThrusterTest(Node):
    """Publish one bounded motor command and automatically return to neutral."""

    def __init__(self):
        super().__init__('thruster_test')
        self.declare_parameter('output_topic', '/tardigrade/thrusters/cmd')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('max_abs_command', 0.10)
        self.declare_parameter('max_duration_sec', 2.0)

        output_topic = self.get_parameter('output_topic').value
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.max_command = min(
            0.10,
            max(0.0, float(self.get_parameter('max_abs_command').value)),
        )
        self.max_duration = min(
            2.0,
            max(0.0, float(self.get_parameter('max_duration_sec').value)),
        )
        self.command = [0.0] * NUM_THRUSTERS
        self.stop_at = None

        self.publisher = self.create_publisher(
            Float32MultiArray, output_topic, 10
        )
        self.service = self.create_service(
            TestThruster,
            '/tardigrade/test/run_thruster',
            self.run_thruster,
        )
        self.timer = self.create_timer(1.0 / max(1.0, rate), self.publish)
        self.get_logger().info(
            'Individual checkout ready: /tardigrade/test/run_thruster; '
            f'max command={self.max_command:.2f}, '
            f'max duration={self.max_duration:.1f}s'
        )

    def neutralize(self):
        """Select eight zero commands."""
        self.command = [0.0] * NUM_THRUSTERS
        self.stop_at = None

    def publish(self):
        """Publish the active command or neutral after its deadline."""
        if self.stop_at is not None and time.monotonic() >= self.stop_at:
            self.neutralize()
            self.get_logger().info('Thruster test complete; publishing neutral')
        msg = Float32MultiArray()
        msg.data = list(self.command)
        self.publisher.publish(msg)

    def run_thruster(self, request, response):
        """Handle one bounded, 1-indexed thruster test request."""
        error = validate_request(
            int(request.slot),
            float(request.command),
            float(request.duration_sec),
            self.max_command,
            self.max_duration,
        )
        if error is not None:
            self.neutralize()
            self.publish()
            response.success = False
            response.message = error
            self.get_logger().warn(f'Rejected thruster test: {error}')
            return response

        self.neutralize()
        self.command[int(request.slot) - 1] = float(request.command)
        self.stop_at = time.monotonic() + float(request.duration_sec)
        self.publish()
        response.success = True
        response.message = (
            f'slot {request.slot} at {request.command:.3f} for '
            f'{request.duration_sec:.2f}s'
        )
        self.get_logger().warn(f'THRUSTER TEST: {response.message}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.neutralize()
        node.publish()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
