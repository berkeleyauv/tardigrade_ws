"""Runtime evaluator for the gate SIL acceptance scenario."""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from tardigrade_interfaces.msg import GateDetection, ThrusterCommands


def quaternion_angle(a, b):
    dot = abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w)
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class AcceptanceMonitor(Node):
    def __init__(self):
        super().__init__('unity_gate_acceptance_monitor')
        self.declare_parameter('gate_plane_x_m', 4.0)
        self.declare_parameter('aperture_half_width_m', 1.2)
        self.declare_parameter('timeout_sec', 60.0)
        self.gate_x = float(self.get_parameter('gate_plane_x_m').value)
        self.half_width = float(
            self.get_parameter('aperture_half_width_m').value)
        self.timeout = float(self.get_parameter('timeout_sec').value)
        self.started_ns = self.get_clock().now().nanoseconds
        self.truth = None
        self.filtered = None
        self.initial_depth = None
        self.max_position_error = 0.0
        self.max_attitude_error = 0.0
        self.max_depth_error = 0.0
        self.detected_gate = False
        self.crossed = False
        self.commands_neutral = False
        self.create_subscription(
            Odometry, '/tardigrade/sim/ground_truth/odometry',
            self.on_truth, 10)
        self.create_subscription(
            Odometry, '/tardigrade/state/odometry/filtered',
            self.on_filtered, 10)
        self.create_subscription(
            GateDetection, '/tardigrade/perception/gate',
            self.on_gate, 10)
        self.create_subscription(
            ThrusterCommands, '/tardigrade/actuators/thruster_commands',
            self.on_commands, 10)
        self.timer = self.create_timer(0.1, self.evaluate)

    def on_truth(self, message):
        self.truth = message
        if self.initial_depth is None:
            self.initial_depth = message.pose.pose.position.z
        self.max_depth_error = max(
            self.max_depth_error,
            abs(message.pose.pose.position.z - self.initial_depth),
        )
        if (message.pose.pose.position.x >= self.gate_x and
                abs(message.pose.pose.position.y) <= self.half_width):
            self.crossed = True
        self.update_estimator_error()

    def on_filtered(self, message):
        self.filtered = message
        self.update_estimator_error()

    def on_gate(self, message):
        self.detected_gate |= bool(
            message.visible and message.confidence >= 0.6)

    def on_commands(self, message):
        self.commands_neutral = bool(message.setpoints) and all(
            abs(value) <= 1e-3 for value in message.setpoints)

    def update_estimator_error(self):
        if self.truth is None or self.filtered is None:
            return
        truth = self.truth.pose.pose
        estimate = self.filtered.pose.pose
        position_error = math.sqrt(
            (truth.position.x - estimate.position.x) ** 2 +
            (truth.position.y - estimate.position.y) ** 2 +
            (truth.position.z - estimate.position.z) ** 2)
        self.max_position_error = max(
            self.max_position_error, position_error)
        self.max_attitude_error = max(
            self.max_attitude_error,
            quaternion_angle(truth.orientation, estimate.orientation))

    def evaluate(self):
        elapsed = (self.get_clock().now().nanoseconds - self.started_ns) / 1e9
        if self.crossed:
            failures = []
            if not self.detected_gate:
                failures.append('gate was never detected from camera images')
            if self.max_position_error > 0.25:
                failures.append(
                    f'position error {self.max_position_error:.3f} m > 0.25 m')
            if self.max_attitude_error > math.radians(5.0):
                failures.append(
                    'attitude error '
                    f'{math.degrees(self.max_attitude_error):.2f} deg '
                    '> 5 deg')
            if self.max_depth_error > 0.10:
                failures.append(
                    f'depth error {self.max_depth_error:.3f} m > 0.10 m')
            if failures:
                self.get_logger().error(
                    'ACCEPTANCE FAILED: ' + '; '.join(failures))
            else:
                self.get_logger().info(
                    'ACCEPTANCE PASSED: camera gate detection, aperture '
                    'crossing, '
                    f'max estimator error={self.max_position_error:.3f} m / '
                    f'{math.degrees(self.max_attitude_error):.2f} deg, '
                    f'max depth error={self.max_depth_error:.3f} m')
            self.timer.cancel()
        elif elapsed > self.timeout:
            self.get_logger().error(
                'ACCEPTANCE FAILED: gate plane was not crossed before timeout')
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = AcceptanceMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
