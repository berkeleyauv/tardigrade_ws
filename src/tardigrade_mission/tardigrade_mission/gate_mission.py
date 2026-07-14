import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from tardigrade_interfaces.msg import GateDetection, RobotStatus
from tardigrade_interfaces.srv import SetArmed, SetExternalControl


def clamp(value, low, high):
    return max(low, min(high, value))


class GateMission(Node):
    def __init__(self):
        super().__init__('gate_mission')

        self.declare_parameter('cmd_vel_topic', '/tardigrade/cmd_vel')
        self.declare_parameter('status_topic', '/tardigrade/status')
        self.declare_parameter('gate_topic', '/tardigrade/perception/gate')
        self.declare_parameter('dry_run', False)
        self.declare_parameter('search_yaw_rate_radps', 0.25)
        self.declare_parameter('align_kp', 0.9)
        self.declare_parameter('align_max_yaw_rate_radps', 0.35)
        self.declare_parameter('alignment_tolerance_rad', 0.08)
        self.declare_parameter('gate_confidence_threshold', 0.6)
        self.declare_parameter('forward_speed_mps', 0.25)
        self.declare_parameter('pass_gate_sec', 6.0)
        self.declare_parameter('timeout_sec', 45.0)
        self.declare_parameter('disarm_when_done', True)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.gate_topic = self.get_parameter('gate_topic').value
        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.search_yaw_rate_radps = float(
            self.get_parameter('search_yaw_rate_radps').value
        )
        self.align_kp = float(self.get_parameter('align_kp').value)
        self.align_max_yaw_rate_radps = float(
            self.get_parameter('align_max_yaw_rate_radps').value
        )
        self.alignment_tolerance_rad = float(
            self.get_parameter('alignment_tolerance_rad').value
        )
        self.gate_confidence_threshold = float(
            self.get_parameter('gate_confidence_threshold').value
        )
        self.forward_speed_mps = float(self.get_parameter('forward_speed_mps').value)
        self.pass_gate_sec = float(self.get_parameter('pass_gate_sec').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.disarm_when_done = bool(self.get_parameter('disarm_when_done').value)

        self.latest_status = None
        self.latest_gate = None

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.status_sub = self.create_subscription(
            RobotStatus,
            self.status_topic,
            self.status_callback,
            10,
        )
        self.gate_sub = self.create_subscription(
            GateDetection,
            self.gate_topic,
            self.gate_callback,
            10,
        )
        self.arm_client = self.create_client(SetArmed, '/tardigrade/set_armed')
        self.external_client = self.create_client(
            SetExternalControl,
            '/tardigrade/set_external_control',
        )

    def status_callback(self, msg):
        self.latest_status = msg

    def gate_callback(self, msg):
        self.latest_gate = msg

    def run(self):
        self.wait_for_status()
        if self.dry_run:
            self.get_logger().warn('dry_run=true: not enabling, arming, or moving')
            self.wait_for_gate()
            self.get_logger().info('Dry run complete: status and gate topics work.')
            return

        self.set_external_control(True)
        self.set_armed(True)

        try:
            self.search_for_gate()
            self.align_to_gate()
            self.pass_through_gate()
            self.get_logger().info('Gate mission complete.')
        finally:
            self.publish_zero()
            if self.disarm_when_done:
                self.set_armed(False)
                self.set_external_control(False)

    def wait_for_status(self):
        self.get_logger().info(f'Waiting for {self.status_topic}')
        self.wait_until(
            lambda: self.latest_status is not None and self.latest_status.px4_connected,
            self.timeout_sec,
            'Timed out waiting for connected robot status',
        )

    def wait_for_gate(self):
        self.get_logger().info(f'Waiting for {self.gate_topic}')
        self.wait_until(
            lambda: self.gate_is_visible(),
            self.timeout_sec,
            'Timed out waiting for visible gate',
        )

    def search_for_gate(self):
        self.get_logger().info('Searching for gate...')
        start = time.monotonic()
        while rclpy.ok() and not self.gate_is_visible():
            if time.monotonic() - start > self.timeout_sec:
                raise RuntimeError('Timed out searching for gate')
            cmd = Twist()
            cmd.angular.z = self.search_yaw_rate_radps
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.1)
        self.publish_zero()
        self.get_logger().info('Gate visible.')

    def align_to_gate(self):
        self.get_logger().info('Aligning to gate...')
        start = time.monotonic()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            if not self.gate_is_visible():
                self.search_for_gate()
                start = time.monotonic()
                continue

            yaw_error = self.latest_gate.yaw_error_rad
            if abs(yaw_error) <= self.alignment_tolerance_rad:
                self.publish_zero()
                self.get_logger().info('Gate aligned.')
                return

            if time.monotonic() - start > self.timeout_sec:
                raise RuntimeError('Timed out aligning to gate')

            cmd = Twist()
            cmd.angular.z = clamp(
                self.align_kp * yaw_error,
                -self.align_max_yaw_rate_radps,
                self.align_max_yaw_rate_radps,
            )
            self.cmd_pub.publish(cmd)
            time.sleep(0.1)

    def pass_through_gate(self):
        self.get_logger().info(
            f'Passing through gate for {self.pass_gate_sec:.1f}s '
            f'at {self.forward_speed_mps:.2f} m/s'
        )
        end = time.monotonic() + self.pass_gate_sec
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.0)
            cmd = Twist()
            cmd.linear.x = self.forward_speed_mps
            if self.gate_is_visible():
                cmd.angular.z = clamp(
                    self.align_kp * self.latest_gate.yaw_error_rad,
                    -self.align_max_yaw_rate_radps,
                    self.align_max_yaw_rate_radps,
                )
            self.cmd_pub.publish(cmd)
            time.sleep(0.1)
        self.publish_zero()

    def gate_is_visible(self):
        return (
            self.latest_gate is not None
            and self.latest_gate.visible
            and self.latest_gate.confidence >= self.gate_confidence_threshold
        )

    def publish_zero(self):
        self.cmd_pub.publish(Twist())

    def set_external_control(self, enabled):
        if not self.external_client.wait_for_service(timeout_sec=self.timeout_sec):
            raise RuntimeError('Service /tardigrade/set_external_control unavailable')
        request = SetExternalControl.Request()
        request.enabled = enabled
        response = self.call_service(self.external_client, request)
        if not response.success:
            raise RuntimeError(response.message)
        self.get_logger().info(response.message)

    def set_armed(self, armed):
        if not self.arm_client.wait_for_service(timeout_sec=self.timeout_sec):
            raise RuntimeError('Service /tardigrade/set_armed unavailable')
        request = SetArmed.Request()
        request.armed = armed
        response = self.call_service(self.arm_client, request)
        if not response.success:
            raise RuntimeError(response.message)
        self.get_logger().info(response.message)

    def call_service(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout_sec)
        if not future.done():
            raise RuntimeError('Timed out waiting for service response')
        return future.result()

    def wait_until(self, predicate, timeout_sec, error_message):
        start = time.monotonic()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return
            if time.monotonic() - start > timeout_sec:
                raise RuntimeError(error_message)


def main(args=None):
    rclpy.init(args=args)
    node = GateMission()
    try:
        node.run()
    except Exception as exc:
        node.get_logger().error(str(exc))
        node.publish_zero()
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()
