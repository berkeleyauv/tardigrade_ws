import json
import os
import threading
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

try:
    import serial
except ImportError:  # pragma: no cover - exercised only on missing dependency
    serial = None


@dataclass
class Thruster:
    name: str
    esp_pin: int
    surge: float
    sway: float
    heave: float
    roll: float
    pitch: float
    yaw: float


def clamp(value, low, high):
    return max(low, min(high, value))


def load_thruster_config(path):
    with open(path, 'r', encoding='utf-8') as config_file:
        data = json.load(config_file)

    protocol = data.get('protocol', {})
    thrusters = []
    for entry in data.get('thrusters', []):
        mix = entry.get('mix', {})
        thrusters.append(
            Thruster(
                name=entry['name'],
                esp_pin=int(entry['esp_pin']),
                surge=float(mix.get('surge', 0.0)),
                sway=float(mix.get('sway', 0.0)),
                heave=float(mix.get('heave', 0.0)),
                roll=float(mix.get('roll', 0.0)),
                pitch=float(mix.get('pitch', 0.0)),
                yaw=float(mix.get('yaw', 0.0)),
            )
        )

    if not thrusters:
        raise ValueError(f'No thrusters configured in {path}')

    return protocol, thrusters


class EspThrusterBridge(Node):
    def __init__(self):
        super().__init__('esp_thruster_bridge')

        default_config = os.path.join(
            os.getcwd(),
            'config',
            'esp_thruster_map.json',
        )

        self.declare_parameter('cmd_vel_topic', '/tardigrade/cmd_vel')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('config_file', default_config)
        self.declare_parameter('send_rate_hz', 20.0)
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.declare_parameter('startup_neutral_sec', 2.0)
        self.declare_parameter('log_every_n', 20)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.config_file = self.get_parameter('config_file').value
        send_rate_hz = float(self.get_parameter('send_rate_hz').value)
        self.cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)
        self.startup_neutral_sec = float(
            self.get_parameter('startup_neutral_sec').value
        )
        self.log_every_n = int(self.get_parameter('log_every_n').value)

        protocol, self.thrusters = load_thruster_config(self.config_file)
        self.neutral_us = int(protocol.get('neutral_us', 1500))
        self.min_us = int(protocol.get('min_us', 1100))
        self.max_us = int(protocol.get('max_us', 1900))
        self.max_delta_us = int(protocol.get('max_delta_us', 200))

        self.latest_cmd = Twist()
        self.latest_cmd_time_ns = None
        self.send_count = 0
        self.lock = threading.Lock()

        if serial is None:
            raise RuntimeError('pyserial is required for esp_thruster_bridge')

        self.serial = serial.Serial(
            self.serial_port,
            self.baudrate,
            timeout=0.02,
            write_timeout=0.1,
        )

        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_callback,
            10,
        )

        period = 1.0 / max(send_rate_hz, 1.0)
        self.timer = self.create_timer(period, self.send_thruster_command)

        self.get_logger().info(f'Loaded thruster config: {self.config_file}')
        self.get_logger().info(
            f'Connected ESP serial: {self.serial_port} @ {self.baudrate}'
        )
        self.get_logger().info(f'Subscribing: {self.cmd_vel_topic}')
        self.get_logger().info(
            'PWM limits: '
            f'neutral={self.neutral_us}, min={self.min_us}, '
            f'max={self.max_us}, max_delta={self.max_delta_us}'
        )
        self.get_logger().info(
            'Thruster order: '
            + ', '.join(f'{t.name}(pin {t.esp_pin})' for t in self.thrusters)
        )
        self.send_neutral_for_startup()

    def destroy_node(self):
        try:
            if hasattr(self, 'serial') and self.serial.is_open:
                self.write_pwm([self.neutral_us] * len(self.thrusters))
                self.serial.close()
        finally:
            super().destroy_node()

    def send_neutral_for_startup(self):
        end_time_ns = (
            self.get_clock().now().nanoseconds
            + int(self.startup_neutral_sec * 1_000_000_000)
        )
        while self.get_clock().now().nanoseconds < end_time_ns:
            self.write_pwm([self.neutral_us] * len(self.thrusters))
            rclpy.spin_once(self, timeout_sec=0.05)

    def cmd_callback(self, msg):
        with self.lock:
            self.latest_cmd = msg
            self.latest_cmd_time_ns = self.get_clock().now().nanoseconds

    def command_is_stale(self):
        if self.latest_cmd_time_ns is None:
            return True
        age_sec = (
            self.get_clock().now().nanoseconds - self.latest_cmd_time_ns
        ) / 1_000_000_000.0
        return age_sec > self.cmd_timeout_sec

    def send_thruster_command(self):
        with self.lock:
            cmd = self.latest_cmd
            stale = self.command_is_stale()

        if stale:
            pwm_values = [self.neutral_us] * len(self.thrusters)
        else:
            pwm_values = self.mix_cmd_vel(cmd)

        self.write_pwm(pwm_values)
        self.send_count += 1

        if self.log_every_n > 0 and self.send_count % self.log_every_n == 0:
            self.get_logger().info(
                'ESP PWM '
                + ('stale->neutral ' if stale else '')
                + ' '.join(str(value) for value in pwm_values)
            )

    def mix_cmd_vel(self, cmd):
        pwm_values = []
        for thruster in self.thrusters:
            normalized = (
                thruster.surge * cmd.linear.x
                + thruster.sway * cmd.linear.y
                + thruster.heave * cmd.linear.z
                + thruster.roll * cmd.angular.x
                + thruster.pitch * cmd.angular.y
                + thruster.yaw * cmd.angular.z
            )
            normalized = clamp(normalized, -1.0, 1.0)
            delta_us = int(round(normalized * self.max_delta_us))
            pwm_values.append(
                clamp(
                    self.neutral_us + delta_us,
                    self.min_us,
                    self.max_us,
                )
            )
        return pwm_values

    def write_pwm(self, pwm_values):
        message = 'PWM ' + ' '.join(str(value) for value in pwm_values) + '\n'
        self.serial.write(message.encode('ascii'))


def main(args=None):
    rclpy.init(args=args)
    node = EspThrusterBridge()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
