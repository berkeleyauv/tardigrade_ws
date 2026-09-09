#!/usr/bin/env python3
"""Bounded, one-at-a-time thruster checkout surface."""

import json
import math
import os
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node

from tardigrade_interfaces.msg import ThrusterCommands
from tardigrade_interfaces.srv import TestThruster as ThrusterTestService

NUM_THRUSTERS = 8


def validate_request(
        slot, command, duration_sec, max_command, max_duration,
        test_active=False):
    """Validate one bounded checkout request and return an error or ``None``."""
    if test_active:
        return 'another thruster test was active; command neutralized'
    if slot < 1 or slot > NUM_THRUSTERS:
        return f'slot must be 1..{NUM_THRUSTERS}'
    if not math.isfinite(command) or not math.isfinite(duration_sec):
        return 'command and duration_sec must be finite'
    if abs(command) > max_command:
        return f'abs(command) must be <= {max_command:.3f}'
    if duration_sec <= 0.0 or duration_sec > max_duration:
        return f'duration_sec must be > 0 and <= {max_duration:.2f}'
    return None


def load_thruster_names(path):
    """Load the unique physical slot order used by the ESP bridge."""
    with open(path, encoding='utf-8') as stream:
        configuration = json.load(stream)
    thrusters = sorted(
        configuration.get('thrusters', []),
        key=lambda item: int(item['slot']),
    )
    slots = [int(item['slot']) for item in thrusters]
    names = [str(item['name']) for item in thrusters]
    if slots != list(range(1, NUM_THRUSTERS + 1)):
        raise ValueError('ESP map must contain slots 1 through 8 exactly once')
    if len(set(names)) != NUM_THRUSTERS:
        raise ValueError('ESP map must contain eight unique thruster names')
    return names


def command_values(slot, command):
    """Return one eight-element command with only ``slot`` selected."""
    values = [0.0] * NUM_THRUSTERS
    if slot is not None:
        values[int(slot) - 1] = float(command)
    return values


def command_message(names, values):
    """Build the named actuator message shared by checkout and ESP bridge."""
    message = ThrusterCommands()
    message.header.frame_id = 'base_link'
    message.names = list(names)
    message.setpoints = list(values)
    return message


class ThrusterTest(Node):
    """Publish one bounded motor command and automatically return to neutral."""

    def __init__(self):
        super().__init__('thruster_test')
        self.declare_parameter(
            'output_topic', '/tardigrade/actuators/thruster_commands')
        self.declare_parameter(
            'config_file',
            os.path.join(
                get_package_share_directory('tardigrade_esp'),
                'config', 'esp_thruster_map.json'))
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('max_abs_command', 0.10)
        self.declare_parameter('max_duration_sec', 2.0)

        output_topic = self.get_parameter('output_topic').value
        self.names = load_thruster_names(
            str(self.get_parameter('config_file').value))
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.max_command = min(
            0.10,
            max(0.0, float(self.get_parameter('max_abs_command').value)),
        )
        self.max_duration = min(
            2.0,
            max(0.0, float(self.get_parameter('max_duration_sec').value)),
        )
        self.command = command_values(None, 0.0)
        self.stop_at = None

        self.publisher = self.create_publisher(
            ThrusterCommands, output_topic, 10
        )
        self.service = self.create_service(
            ThrusterTestService,
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
        self.command = command_values(None, 0.0)
        self.stop_at = None

    def active(self, now=None):
        """Return whether an individual-thruster command is still active."""
        if self.stop_at is None:
            return False
        return (time.monotonic() if now is None else now) < self.stop_at

    def publish(self):
        """Publish the active command or neutral after its deadline."""
        if self.stop_at is not None and time.monotonic() >= self.stop_at:
            self.neutralize()
            self.get_logger().info('Thruster test complete; publishing neutral')
        msg = command_message(self.names, self.command)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)

    def run_thruster(self, request, response):
        """Handle one bounded, 1-indexed thruster test request."""
        error = validate_request(
            int(request.slot),
            float(request.command),
            float(request.duration_sec),
            self.max_command,
            self.max_duration,
            test_active=self.active(),
        )
        if error is not None:
            self.neutralize()
            self.publish()
            response.success = False
            response.message = error
            self.get_logger().warn(f'Rejected thruster test: {error}')
            return response

        self.neutralize()
        self.publish()
        self.command = command_values(request.slot, request.command)
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
