#!/usr/bin/env python3
"""esp_bridge — ROS 2 telemetry + control bridge for the Tardigrade ESP32 flight
controller.

F1 (read-only telemetry): owns the ESP serial port, polls it for State
telemetry, and republishes each frame as /tardigrade/esp/state
(tardigrade_interfaces/EspState) so Foxglove and rosbag can see and record
ESP-side state.

F2 subset (motor test + arm/disarm), added on top of F1:
  - subscribes /tardigrade/thrusters/cmd (std_msgs/Float32MultiArray, 8
    elements, index = thruster, value -1..+1) and forwards each as a SetMotor
    frame — the same raw per-thruster surface described in
    foxglove_integration.md, minus the ROS-parameter tuning half of F2.
  - /tardigrade/set_armed (tardigrade_interfaces/SetArmed) sends Arm/Disarm and
    reports back what the ESP actually Ack'd, not just "frame was sent".
  - sends a continuous Heartbeat to the ESP for as long as the last decoded
    State reports armed=true, regardless of whether any client is connected —
    this is what keeps Safety::update()'s 300 ms link-timeout satisfied once
    arming stops being a continuous webapp heartbeat. See
    foxglove_integration.md's "Safety model" section.

Not built: ROS-parameter gain sync/save/reset, diagnostics topic. Those are
the rest of F2/D4 and unnecessary for bench motor testing.

TEMPORARY BENCH-TEST HOOK — synthetic pose feed:
  Nothing normally sends Pose frames through this bridge (that's
  gcs_server.py --ros / pose_bridge.py's job, and only one process may own
  the serial port at a time), which means ExternalEstimator can never go
  healthy and the sensor/pose-timeout failsafe can never be exercised on a
  bench sub with no real EKF running. /tardigrade/test/synthetic_pose
  (std_msgs/Bool) toggles a fake, valid, identity-orientation Pose frame at
  the poll rate — enough to make the ESP report healthy, so arming +
  disabling it can confirm the ESP auto-disarms on SensorTimeout. Defaults
  OFF; touching nothing unless explicitly enabled. This is scaffolding for
  the bench failsafe check, not part of the real pose path — remove once
  real pose forwarding (or Track C) makes it moot.

This is the ROS-native counterpart to the firmware repo's gcs_server.py.

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
from tardigrade_interfaces.srv import SetArmed
from std_msgs.msg import Bool, Float32MultiArray

from . import tardigrade_protocol as tp

try:
    import serial  # pyserial
except ImportError:
    serial = None

_DEG2RAD = math.pi / 180.0
_NUM_THRUSTERS = 8
_ACK_TIMEOUT_SEC = 0.5


def validated_motor_values(data):
    """Return one safe eight-motor command or raise ValueError."""
    if len(data) != _NUM_THRUSTERS:
        raise ValueError(
            f'expected {_NUM_THRUSTERS} values, got {len(data)}'
        )

    values = [float(value) for value in data]
    if not all(math.isfinite(value) for value in values):
        raise ValueError('all motor values must be finite')
    return [max(-1.0, min(1.0, value)) for value in values]


class EspBridge(Node):
    def __init__(self):
        super().__init__('esp_bridge')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('poll_rate_hz', 20.0)
        self.declare_parameter('cmd_timeout_sec', 0.5)

        port = self.get_parameter('serial_port').value
        baud = int(self.get_parameter('baud').value)
        rate = float(self.get_parameter('poll_rate_hz').value)
        self._cmd_timeout_sec = max(
            0.0,
            float(self.get_parameter('cmd_timeout_sec').value),
        )
        self._poll_period = 1.0 / max(1.0, rate)

        if serial is None:
            raise RuntimeError('pyserial required: pip install pyserial '
                               '(or apt install python3-serial)')

        self._ser = serial.Serial(port, baud, timeout=0.02, write_timeout=0.2)
        self._parser = tp.Parser()
        self._pub = self.create_publisher(EspState, '/tardigrade/esp/state', 10)

        # Serial writes now come from three places: this bridge's own GetState
        # poll (background thread), and the motor-cmd subscription + set_armed
        # service callbacks (rclpy executor thread). A lock keeps two frames
        # from interleaving on the wire; reading stays solely on the
        # background thread, so it needs no lock.
        self._write_lock = threading.Lock()
        self._ack_event = threading.Event()
        self._last_ack = None  # (echoed_type, accepted, reason)
        self._reported_armed = False
        self._last_motor_cmd_monotonic = None
        self._watchdog_neutral_active = False

        self._motor_sub = self.create_subscription(
            Float32MultiArray, '/tardigrade/thrusters/cmd',
            self._on_thruster_cmd, 10)
        self._arm_srv = self.create_service(
            SetArmed, '/tardigrade/set_armed', self._on_set_armed)
        # Independent of any client staying connected — see module docstring
        # and foxglove_integration.md's "Safety model" section.
        self._heartbeat_timer = self.create_timer(
            0.1, self._send_heartbeat_if_armed)

        # Bench-test-only synthetic pose feed — see module docstring. Off by
        # default; a fake, always-identity, always-fresh Pose frame sent at
        # the poll rate is enough to make ExternalEstimator report healthy.
        self._synthetic_pose_enabled = False
        self._pose_seq = 0
        self._synthetic_pose_sub = self.create_subscription(
            Bool, '/tardigrade/test/synthetic_pose',
            self._on_synthetic_pose_toggle, 10)
        self._synthetic_pose_timer = self.create_timer(
            self._poll_period, self._send_synthetic_pose_if_enabled)

        self.get_logger().info(
            f'esp_bridge: {port} @ {baud} -> /tardigrade/esp/state '
            f'(polling GetState at {rate:.0f} Hz); '
            f'/tardigrade/thrusters/cmd -> SetMotor; '
            f'/tardigrade/set_armed -> Arm/Disarm; '
            f'command timeout={self._cmd_timeout_sec:.2f}s')

        # One thread owns the serial port for both the GetState poll (write) and
        # reading/decoding (read). Other writers (above) take _write_lock.
        # rclpy publish is safe to call from this thread.
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
                # not command anything.
                try:
                    with self._write_lock:
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
                elif mtype == tp.ACK:
                    ack = tp.decode_ack(payload)
                    if ack is not None:
                        self._last_ack = ack
                        self._ack_event.set()

    def _publish_state(self, payload):
        s = tp.decode_state(payload)
        if not s:
            return
        self._reported_armed = bool(s['armed'])
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

    def _on_thruster_cmd(self, msg):
        try:
            values = validated_motor_values(msg.data)
        except (TypeError, ValueError) as exc:
            self.get_logger().warn(
                f'/tardigrade/thrusters/cmd: {exc} — ignoring')
            return
        try:
            with self._write_lock:
                self._write_motor_values(values)
                self._last_motor_cmd_monotonic = time.monotonic()
                self._watchdog_neutral_active = False
        except Exception as exc:  # noqa: BLE001 - report, keep running
            self.get_logger().warn(f'serial write failed: {exc}')

    def _write_motor_values(self, values):
        """Write one complete motor command while the caller holds the lock."""
        for index, value in enumerate(values):
            self._ser.write(tp.encode_set_motor(index, value))

    def _motor_command_is_stale(self, now=None):
        if self._last_motor_cmd_monotonic is None:
            return True
        if now is None:
            now = time.monotonic()
        return now - self._last_motor_cmd_monotonic > self._cmd_timeout_sec

    def _send_heartbeat_if_armed(self):
        if not self._reported_armed:
            return
        try:
            with self._write_lock:
                if self._motor_command_is_stale():
                    self._write_motor_values([0.0] * _NUM_THRUSTERS)
                    if not self._watchdog_neutral_active:
                        self.get_logger().warn(
                            'motor command stale; forcing all thrusters neutral'
                        )
                    self._watchdog_neutral_active = True
                self._ser.write(tp.encode(tp.HEARTBEAT))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'safety heartbeat write failed: {exc}')

    def _on_synthetic_pose_toggle(self, msg):
        self._synthetic_pose_enabled = bool(msg.data)
        self.get_logger().info(
            f'synthetic pose feed: {"ON" if msg.data else "OFF"} '
            f'(bench sensor-timeout test hook)')

    def _send_synthetic_pose_if_enabled(self):
        if not self._synthetic_pose_enabled:
            return
        self._pose_seq += 1
        frame = tp.encode_pose(
            self._pose_seq,
            (0.0, 0.0, 0.0),      # position: origin
            (1.0, 0.0, 0.0, 0.0),  # orientation: identity (level)
            (0.0, 0.0, 0.0),      # linear velocity: zero
            (0.0, 0.0, 0.0),      # angular velocity: zero
        )
        try:
            with self._write_lock:
                self._ser.write(frame)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'synthetic pose write failed: {exc}')

    def _on_set_armed(self, request, response):
        frame = tp.encode(tp.ARM if request.armed else tp.DISARM)
        with self._write_lock:
            self._ack_event.clear()
            self._last_ack = None
            self._ser.write(frame)

        got = self._ack_event.wait(_ACK_TIMEOUT_SEC)
        verb = 'arm' if request.armed else 'disarm'
        if not got or self._last_ack is None:
            response.success = False
            response.message = f'no Ack from ESP within {_ACK_TIMEOUT_SEC}s'
            self.get_logger().warn(f'set_armed({verb}): {response.message}')
            return response

        _echoed, accepted, reason = self._last_ack
        reason_name = tp.ACK_REASONS.get(reason, f'Unknown({reason})')
        response.success = accepted
        response.message = reason_name
        self.get_logger().info(
            f'set_armed({verb}): accepted={accepted} reason={reason_name}')
        return response

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
