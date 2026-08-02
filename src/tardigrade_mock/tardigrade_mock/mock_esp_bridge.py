import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Int32MultiArray

from tardigrade_interfaces.msg import EspState
from tardigrade_interfaces.srv import SetArmed


def clamp(value, low, high):
    return max(low, min(high, value))


def euler_from_quaternion(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    pitch = math.asin(clamp(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return roll, pitch, yaw


class MockEspBridge(Node):
    def __init__(self):
        super().__init__('mock_esp_bridge')

        self.declare_parameter('pose_timeout_sec', 0.5)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('test_override_sec', 1.0)
        self.declare_parameter('neutral_pwm', 1500)
        self.declare_parameter('pwm_span', 400)
        self.pose_timeout_sec = float(
            self.get_parameter('pose_timeout_sec').value
        )
        self.command_timeout_sec = float(
            self.get_parameter('command_timeout_sec').value
        )
        self.test_override_sec = float(
            self.get_parameter('test_override_sec').value
        )

        self.armed = False
        self.latest_odometry = None
        self.latest_odometry_time = None
        self.latest_command = [0.0] * 8
        self.latest_command_time = None
        self.test_command = [0.0] * 8
        self.test_command_time = None
        self.command_valid = True
        self.send_count = 0
        self.start_time = self.get_clock().now()

        self.state_pub = self.create_publisher(
            EspState, '/tardigrade/esp/state', 10
        )
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, '/tardigrade/esp/diagnostics', 10
        )
        self.pwm_pub = self.create_publisher(
            Int32MultiArray, '/tardigrade/thrusters/pwm', 10
        )
        self.link_quality_pub = self.create_publisher(
            Float32, '/tardigrade/esp/link_quality', 10
        )
        self.round_trip_pub = self.create_publisher(
            Float32, '/tardigrade/esp/round_trip_ms', 10
        )

        self.create_subscription(
            Odometry,
            '/tardigrade/state/odometry/filtered',
            self.odometry_callback,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            '/tardigrade/thrusters/command',
            self.thruster_command_callback,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            '/tardigrade/thrusters/test_cmd',
            self.motor_test_callback,
            10,
        )
        self.create_service(
            SetArmed,
            '/tardigrade/set_armed',
            self.handle_set_armed,
        )

        self.state_timer = self.create_timer(0.05, self.publish_state)
        self.diagnostics_timer = self.create_timer(0.5, self.publish_diagnostics)
        self.output_timer = self.create_timer(0.05, self.publish_thruster_pwm)
        self.get_logger().info('Publishing mock ESP bridge telemetry')

    def odometry_callback(self, msg):
        self.latest_odometry = msg
        self.latest_odometry_time = self.get_clock().now()

    def thruster_command_callback(self, msg):
        if len(msg.data) != 8:
            self.command_valid = False
            return
        self.command_valid = True
        self.latest_command = [clamp(value, -1.0, 1.0) for value in msg.data]
        self.latest_command_time = self.get_clock().now()

    def motor_test_callback(self, msg):
        if len(msg.data) != 8:
            self.command_valid = False
            return
        self.command_valid = True
        self.test_command = [clamp(value, -1.0, 1.0) for value in msg.data]
        self.test_command_time = self.get_clock().now()

    def handle_set_armed(self, request, response):
        self.armed = bool(request.armed)
        response.success = True
        response.message = 'Mock ESP armed' if self.armed else 'Mock ESP disarmed'
        return response

    def age_sec(self, timestamp):
        if timestamp is None:
            return math.inf
        age = self.get_clock().now() - timestamp
        return age.nanoseconds / 1_000_000_000.0

    def pose_is_fresh(self):
        return self.age_sec(self.latest_odometry_time) <= self.pose_timeout_sec

    def active_command(self):
        if self.age_sec(self.test_command_time) <= self.test_override_sec:
            return self.test_command, 'motor_test'
        if self.age_sec(self.latest_command_time) <= self.command_timeout_sec:
            return self.latest_command, 'controller'
        return [0.0] * 8, 'stale_neutral'

    def publish_state(self):
        pose_ok = self.pose_is_fresh()
        msg = EspState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.armed = self.armed
        msg.state_valid = pose_ok
        msg.altitude_valid = pose_ok
        msg.link_ok = True
        msg.pose_ok = pose_ok
        if self.latest_odometry is not None:
            odometry = self.latest_odometry
            msg.roll, msg.pitch, msg.yaw = euler_from_quaternion(
                odometry.pose.pose.orientation
            )
            msg.depth = odometry.pose.pose.position.z
            msg.vertical_velocity = odometry.twist.twist.linear.z
        self.state_pub.publish(msg)

    def publish_thruster_pwm(self):
        command, source = self.active_command()
        del source
        if not self.armed:
            command = [0.0] * 8
        neutral = int(self.get_parameter('neutral_pwm').value)
        span = int(self.get_parameter('pwm_span').value)
        msg = Int32MultiArray()
        msg.data = [int(round(neutral + value * span)) for value in command]
        self.pwm_pub.publish(msg)
        self.send_count += 1

    def publish_diagnostics(self):
        elapsed = self.age_sec(self.start_time)
        pose_ok = self.pose_is_fresh()
        command, source = self.active_command()
        del command
        healthy = pose_ok and self.command_valid

        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK if healthy else DiagnosticStatus.WARN
        status.name = 'tardigrade/esp_bridge'
        status.hardware_id = 'mock-esp32'
        status.message = 'Mock ESP link healthy' if healthy else 'Mock warning'
        status.values = [
            KeyValue(key='serial_connected', value='true'),
            KeyValue(key='crc_errors', value='0'),
            KeyValue(key='pose_ok', value=str(pose_ok).lower()),
            KeyValue(key='armed', value=str(self.armed).lower()),
            KeyValue(key='command_valid', value=str(self.command_valid).lower()),
            KeyValue(key='command_source', value=source),
            KeyValue(key='send_count', value=str(self.send_count)),
            KeyValue(key='heartbeat_age_ms', value='50'),
            KeyValue(key='uptime_sec', value=f'{elapsed:.1f}'),
            KeyValue(key='last_disarm_reason', value='none'),
        ]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self.diagnostics_pub.publish(msg)

        link_quality = Float32()
        link_quality.data = 98.0 + 1.5 * math.sin(elapsed * 0.2)
        self.link_quality_pub.publish(link_quality)
        round_trip = Float32()
        round_trip.data = 4.5 + 1.0 * math.sin(elapsed * 0.7)
        self.round_trip_pub.publish(round_trip)


def main(args=None):
    rclpy.init(args=args)
    node = MockEspBridge()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
