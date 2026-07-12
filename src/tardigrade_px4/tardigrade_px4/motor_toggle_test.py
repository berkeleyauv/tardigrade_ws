import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from pymavlink import mavutil

from tardigrade_px4.mavlink_common import connect_mavlink


MAV_RESULT_NAMES = {
    0: 'ACCEPTED',
    1: 'TEMPORARILY_REJECTED',
    2: 'DENIED',
    3: 'UNSUPPORTED',
    4: 'FAILED',
    5: 'IN_PROGRESS',
    6: 'CANCELLED',
}

MAV_CMD_ACTUATOR_TEST = getattr(mavutil.mavlink, 'MAV_CMD_ACTUATOR_TEST', 310)
ACTUATOR_FUNCTION_MOTOR1 = 101


class RawTerminal:
    def __enter__(self):
        if not sys.stdin.isatty():
            raise RuntimeError('motor_toggle_test requires an interactive terminal')
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def read_key(self, timeout_sec):
        readable, _, _ = select.select([sys.stdin], [], [], timeout_sec)
        if not readable:
            return None
        return sys.stdin.read(1)


class MotorToggleTest(Node):
    def __init__(self):
        super().__init__('motor_toggle_test')

        self.declare_parameter('device', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 921600)
        self.declare_parameter('source_system', 44)
        self.declare_parameter('source_component', 191)
        self.declare_parameter('motor', 3)
        self.declare_parameter('actuator_function', 0)
        self.declare_parameter('throttle_percent', 10.0)
        self.declare_parameter('command_timeout_sec', 0.75)
        self.declare_parameter('resend_rate_hz', 4.0)

        self.device = self.get_parameter('device').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        source_system = int(self.get_parameter('source_system').value)
        source_component = int(self.get_parameter('source_component').value)
        self.motor = int(self.get_parameter('motor').value)
        self.actuator_function_override = int(self.get_parameter('actuator_function').value)
        self.throttle_percent = float(self.get_parameter('throttle_percent').value)
        self.command_timeout_sec = float(self.get_parameter('command_timeout_sec').value)
        self.resend_period_sec = 1.0 / max(float(self.get_parameter('resend_rate_hz').value), 0.1)

        if self.motor < 1:
            raise ValueError('motor must be 1 or greater')
        if self.throttle_percent < 0.0 or self.throttle_percent > 100.0:
            raise ValueError('throttle_percent must be in [0, 100]')

        self.mav = connect_mavlink(
            self.device,
            self.baudrate,
            source_system=source_system,
            source_component=source_component,
        )
        self.target_system = 1
        self.target_component = 1
        self.running = False
        self.last_ack = None

    @property
    def actuator_output_function(self):
        if self.actuator_function_override > 0:
            return float(self.actuator_function_override)
        # PX4 actuator functions start at 101 for Motor 1, so motor 3 is 103.
        return float(ACTUATOR_FUNCTION_MOTOR1 + self.motor - 1)

    def wait_for_pixhawk(self):
        self.get_logger().info(f'Opening MAVLink serial: {self.device} @ {self.baudrate}')
        heartbeat = self.mav.wait_heartbeat(timeout=10)
        if heartbeat is None:
            raise RuntimeError('Timed out waiting for Pixhawk MAVLink heartbeat')
        self.target_system = heartbeat.get_srcSystem()
        self.target_component = heartbeat.get_srcComponent()
        self.get_logger().info(
            f'Pixhawk heartbeat from system={self.target_system}, component={self.target_component}'
        )

    def send_actuator_test(self, value, timeout_sec):
        self.mav.mav.command_long_send(
            self.target_system,
            self.target_component,
            MAV_CMD_ACTUATOR_TEST,
            0,
            float(value),
            float(timeout_sec),
            0.0,
            0.0,
            self.actuator_output_function,
            0.0,
            0.0,
        )

    def stop_motor(self):
        self.running = False
        for _ in range(3):
            self.send_actuator_test(float('nan'), 0.0)
            self.drain_command_acks()
            time.sleep(0.05)
        self.get_logger().info(f'Motor {self.motor} OFF')

    def toggle_motor(self):
        if self.running:
            self.stop_motor()
            return

        self.running = True
        self.send_actuator_test(self.throttle_percent / 100.0, self.command_timeout_sec)
        self.get_logger().warn(
            f'Motor {self.motor} ON at {self.throttle_percent:.1f}% '
            f'(space toggles off, q quits)'
        )

    def drain_command_acks(self):
        while True:
            msg = self.mav.recv_match(type='COMMAND_ACK', blocking=False)
            if msg is None:
                return
            if msg.command != MAV_CMD_ACTUATOR_TEST:
                continue
            if self.last_ack == (msg.command, msg.result):
                continue
            self.last_ack = (msg.command, msg.result)
            result = MAV_RESULT_NAMES.get(msg.result, str(msg.result))
            self.get_logger().info(f'Actuator test ACK: {result} ({msg.result})')

    def run(self):
        self.wait_for_pixhawk()
        self.get_logger().warn('Bench test only: secure the vehicle and keep clear of thrusters.')
        self.get_logger().info(
            f'Press space to toggle motor {self.motor} '
            f'(actuator_function={int(self.actuator_output_function)}); '
            f'throttle={self.throttle_percent:.1f}%; '
            'press q to quit.'
        )

        next_send_time = time.monotonic()
        with RawTerminal() as terminal:
            try:
                while rclpy.ok():
                    key = terminal.read_key(0.05)
                    if key == ' ':
                        self.toggle_motor()
                    elif key in ('q', 'Q', '\x03'):
                        break

                    now = time.monotonic()
                    if self.running and now >= next_send_time:
                        self.send_actuator_test(self.throttle_percent / 100.0, self.command_timeout_sec)
                        next_send_time = now + self.resend_period_sec

                    self.drain_command_acks()
            finally:
                self.stop_motor()


def main(args=None):
    rclpy.init(args=args)
    node = MotorToggleTest()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
