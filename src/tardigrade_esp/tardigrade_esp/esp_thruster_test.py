import time

import rclpy
from rclpy.node import Node

try:
    import serial
except ImportError:  # pragma: no cover - exercised only on missing dependency
    serial = None


class EspThrusterTest(Node):
    def __init__(self):
        super().__init__('esp_thruster_test')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('thruster_count', 8)
        self.declare_parameter('neutral_us', 1500)
        self.declare_parameter('test_us', 1600)
        self.declare_parameter('hold_sec', 1.0)

        if serial is None:
            raise RuntimeError('pyserial is required for esp_thruster_test')

        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.thruster_count = int(self.get_parameter('thruster_count').value)
        self.neutral_us = int(self.get_parameter('neutral_us').value)
        self.test_us = int(self.get_parameter('test_us').value)
        self.hold_sec = float(self.get_parameter('hold_sec').value)
        self.serial = serial.Serial(self.serial_port, self.baudrate, timeout=0.1)

        self.get_logger().info(
            f'Connected ESP serial: {self.serial_port} @ {self.baudrate}'
        )

    def send_pwm(self, values):
        line = 'PWM ' + ' '.join(str(value) for value in values) + '\n'
        self.serial.write(line.encode('ascii'))
        self.get_logger().info(line.strip())

    def run_test(self):
        neutral = [self.neutral_us] * self.thruster_count
        self.send_pwm(neutral)
        time.sleep(self.hold_sec)

        for index in range(self.thruster_count):
            values = list(neutral)
            values[index] = self.test_us
            self.get_logger().warn(f'Testing thruster {index + 1}')
            self.send_pwm(values)
            time.sleep(self.hold_sec)
            self.send_pwm(neutral)
            time.sleep(0.5)

        self.send_pwm(neutral)
        self.serial.close()


def main(args=None):
    rclpy.init(args=args)
    node = EspThrusterTest()
    try:
        node.run_test()
    finally:
        node.destroy_node()
        rclpy.shutdown()
