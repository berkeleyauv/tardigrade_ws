#!/usr/bin/env python3
"""esp_bridge — ROS 2 telemetry bridge for the Tardigrade ESP32 flight controller.

F1 scope (read-only): owns the ESP serial port, polls it for State telemetry,
and republishes each frame as /tardigrade/esp/state (tardigrade_interfaces/EspState)
so Foxglove and rosbag can see and record ESP-side state. This is the ROS-native
counterpart to the firmware repo's gcs_server.py, minus the WebSocket/webapp
relay and (for now) pose injection, tuning, and control — those are F2.
See tardigrade_firmware/docs/foxglove_integration.md.

IMPORTANT: only ONE process may own the ESP serial port at a time. Run this OR
gcs_server.py / pose_bridge.py, not both.

Run:
  ros2 run tardigrade_esp esp_bridge --ros-args -p serial_port:=/dev/ttyUSB0
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node

from tardigrade_interfaces.msg import EspState

from . import tardigrade_protocol as tp

try:
    import serial  # pyserial
except ImportError:
    serial = None

_DEG2RAD = math.pi / 180.0


class EspBridge(Node):
    def __init__(self):
        super().__init__('esp_bridge')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('poll_rate_hz', 20.0)

        port = self.get_parameter('serial_port').value
        baud = int(self.get_parameter('baud').value)
        rate = float(self.get_parameter('poll_rate_hz').value)
        self._poll_period = 1.0 / max(1.0, rate)

        if serial is None:
            raise RuntimeError('pyserial required: pip install pyserial '
                               '(or apt install python3-serial)')

        self._ser = serial.Serial(port, baud, timeout=0.02, write_timeout=0.2)
        self._parser = tp.Parser()
        self._pub = self.create_publisher(EspState, '/tardigrade/esp/state', 10)

        self.get_logger().info(
            f'esp_bridge: {port} @ {baud} -> /tardigrade/esp/state '
            f'(polling GetState at {rate:.0f} Hz)')

        # One thread owns the serial port for both the GetState poll (write) and
        # reading/decoding (read), so no lock is needed. rclpy publish is safe
        # to call from this thread.
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._serial_loop, daemon=True)
        self._thread.start()

    def _serial_loop(self):
        last_poll = 0.0
        while not self._stop.is_set() and rclpy.ok():
            now = time.monotonic()
            if now - last_poll >= self._poll_period:
                last_poll = now
                # GetState is a read request — it solicits telemetry, it does
                # not command anything. Safe for the read-only F1 scope.
                try:
                    self._ser.write(tp.encode(tp.GET_STATE))
                except Exception as exc:  # noqa: BLE001 - report, keep running
                    self.get_logger().warn(f'serial write failed: {exc}')

            try:
                data = self._ser.read(256)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'serial read failed: {exc}')
                continue
            if not data:
                continue
            for mtype, payload in self._parser.feed(data):
                if mtype == tp.STATE:
                    self._publish_state(payload)

    def _publish_state(self, payload):
        s = tp.decode_state(payload)
        if not s:
            return
        msg = EspState()
        msg.stamp = self.get_clock().now().to_msg()
        # decode_state returns attitude in DEGREES; EspState is radians (REP-103).
        msg.roll = float(s['roll']) * _DEG2RAD
        msg.pitch = float(s['pitch']) * _DEG2RAD
        msg.yaw = float(s['yaw']) * _DEG2RAD
        msg.depth = float(s['alt'])
        msg.vertical_velocity = float(s['vz'])
        msg.armed = bool(s['armed'])
        msg.state_valid = bool(s['state_ok'])
        msg.altitude_valid = bool(s['alt_ok'])
        msg.link_ok = bool(s['link_ok'])
        msg.pose_ok = bool(s['pose_ok'])
        self._pub.publish(msg)

    def destroy_node(self):
        self._stop.set()
        try:
            if self._ser.is_open:
                self._ser.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EspBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
