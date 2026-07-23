#!/usr/bin/env python3
"""fake_esp_state — synthetic EspState publisher for hardware-free testing.

Publishes plausible /tardigrade/esp/state without any ESP or serial port, so
the Foxglove layout and the ROS pipeline can be verified before dealing with
USB passthrough. Same idea as the webapp's demo mode. Do NOT run alongside the
real esp_bridge — they'd publish the same topic.

Run:
  ros2 run tardigrade_esp fake_esp_state
"""

import math
import time

import rclpy
from rclpy.node import Node

from tardigrade_interfaces.msg import EspState


class FakeEspState(Node):
    def __init__(self):
        super().__init__('fake_esp_state')
        self.declare_parameter('rate_hz', 20.0)
        rate = float(self.get_parameter('rate_hz').value)
        self._pub = self.create_publisher(EspState, '/tardigrade/esp/state', 10)
        self._t0 = time.monotonic()
        self._timer = self.create_timer(1.0 / max(1.0, rate), self._tick)
        self.get_logger().info(
            f'fake_esp_state: synthetic telemetry -> /tardigrade/esp/state '
            f'at {rate:.0f} Hz (no hardware)')

    def _tick(self):
        t = time.monotonic() - self._t0
        msg = EspState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.roll = 0.20 * math.sin(t * 0.9)          # rad
        msg.pitch = 0.12 * math.sin(t * 0.6 + 1.0)
        msg.yaw = math.atan2(math.sin(t * 0.3), math.cos(t * 0.3))  # wraps
        msg.depth = -1.0 + 0.4 * math.sin(t * 0.4)   # ENU z, oscillating depth
        msg.vertical_velocity = 0.4 * 0.4 * math.cos(t * 0.4)
        msg.armed = (int(t) % 20) >= 10              # toggles every 10 s
        msg.state_valid = True
        msg.altitude_valid = True
        msg.link_ok = True
        msg.pose_ok = True
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeEspState()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
