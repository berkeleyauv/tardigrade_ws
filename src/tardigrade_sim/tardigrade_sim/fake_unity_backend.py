import math

import rclpy
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from tardigrade_interfaces.msg import (
    GateDetection,
    RobotStatus,
    SlalomMarkerDetection,
)
from tardigrade_interfaces.srv import SetArmed, SetExternalControl


def clamp(value, low, high):
    return max(low, min(high, value))


def angle_wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def quaternion_from_yaw(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class FakeUnityBackend(Node):
    def __init__(self):
        super().__init__('fake_unity_backend')

        self.declare_parameter('cmd_vel_topic', '/tardigrade/cmd_vel')
        self.declare_parameter('status_topic', '/tardigrade/status')
        self.declare_parameter('odometry_topic', '/tardigrade/state/odometry')
        self.declare_parameter('gate_topic', '/tardigrade/perception/gate')
        self.declare_parameter('slalom_topic', '/tardigrade/perception/slalom')
        self.declare_parameter('update_rate_hz', 30.0)
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.declare_parameter('gate_x_m', 4.0)
        self.declare_parameter('gate_y_m', 0.0)
        self.declare_parameter('gate_visible_distance_m', 8.0)
        self.declare_parameter('gate_fov_rad', 1.2)
        self.declare_parameter('slalom_start_x_m', 8.0)
        self.declare_parameter('slalom_spacing_m', 2.0)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.odometry_topic = self.get_parameter('odometry_topic').value
        self.gate_topic = self.get_parameter('gate_topic').value
        self.slalom_topic = self.get_parameter('slalom_topic').value
        update_rate_hz = float(self.get_parameter('update_rate_hz').value)
        self.cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)
        self.gate_x_m = float(self.get_parameter('gate_x_m').value)
        self.gate_y_m = float(self.get_parameter('gate_y_m').value)
        self.gate_visible_distance_m = float(
            self.get_parameter('gate_visible_distance_m').value
        )
        self.gate_fov_rad = float(self.get_parameter('gate_fov_rad').value)
        self.slalom_start_x_m = float(self.get_parameter('slalom_start_x_m').value)
        self.slalom_spacing_m = float(self.get_parameter('slalom_spacing_m').value)

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.armed = False
        self.external_control_enabled = False
        self.latest_cmd = Twist()
        self.latest_cmd_time = None
        self.last_update_time = self.get_clock().now()

        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_callback,
            10,
        )
        self.status_pub = self.create_publisher(RobotStatus, self.status_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odometry_topic, 10)
        self.gate_pub = self.create_publisher(GateDetection, self.gate_topic, 10)
        self.slalom_pub = self.create_publisher(
            SlalomMarkerDetection,
            self.slalom_topic,
            10,
        )
        self.create_service(
            SetArmed,
            '/tardigrade/set_armed',
            self.handle_set_armed,
        )
        self.create_service(
            SetExternalControl,
            '/tardigrade/set_external_control',
            self.handle_set_external_control,
        )

        period = 1.0 / max(update_rate_hz, 1.0)
        self.timer = self.create_timer(period, self.tick)

        self.get_logger().info('Fake Unity backend started.')
        self.get_logger().info(f'Subscribing: {self.cmd_vel_topic}')
        self.get_logger().info(
            'Publishing: '
            f'{self.status_topic}, {self.odometry_topic}, '
            f'{self.gate_topic}, {self.slalom_topic}'
        )

    def cmd_callback(self, msg):
        self.latest_cmd = msg
        self.latest_cmd_time = self.get_clock().now()

    def handle_set_armed(self, request, response):
        self.armed = bool(request.armed)
        response.success = True
        response.message = 'Fake backend armed' if self.armed else 'Fake backend disarmed'
        return response

    def handle_set_external_control(self, request, response):
        self.external_control_enabled = bool(request.enabled)
        response.success = True
        response.message = (
            'Fake backend external control enabled'
            if self.external_control_enabled
            else 'Fake backend external control disabled'
        )
        return response

    def tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1_000_000_000.0
        self.last_update_time = now

        self.integrate_motion(dt)
        self.publish_status()
        self.publish_odometry()
        self.publish_gate_detection()
        self.publish_slalom_detection()

    def integrate_motion(self, dt):
        if not self.armed or not self.external_control_enabled or self.command_is_stale():
            return

        cmd = self.latest_cmd
        forward = clamp(cmd.linear.x, -1.0, 1.0)
        left = clamp(cmd.linear.y, -1.0, 1.0)
        up = clamp(cmd.linear.z, -1.0, 1.0)
        yaw_rate = clamp(cmd.angular.z, -1.0, 1.0)

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)

        self.x += (forward * cos_yaw - left * sin_yaw) * dt
        self.y += (forward * sin_yaw + left * cos_yaw) * dt
        self.z += up * dt
        self.yaw = angle_wrap(self.yaw + yaw_rate * dt)

    def command_is_stale(self):
        if self.latest_cmd_time is None:
            return True
        age_sec = (
            self.get_clock().now() - self.latest_cmd_time
        ).nanoseconds / 1_000_000_000.0
        return age_sec > self.cmd_timeout_sec

    def publish_status(self):
        msg = RobotStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.control_connected = True
        msg.armed = self.armed
        msg.external_control_enabled = self.external_control_enabled
        msg.nav_state = 0
        msg.arming_state = 1 if self.armed else 0
        msg.detail = (
            'fake_unity_backend; '
            f'x={self.x:.2f}; y={self.y:.2f}; yaw={self.yaw:.2f}'
        )
        self.status_pub.publish(msg)

    def publish_odometry(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = self.z
        msg.pose.pose.orientation = quaternion_from_yaw(self.yaw)
        self.odom_pub.publish(msg)

    def publish_gate_detection(self):
        dx = self.gate_x_m - self.x
        dy = self.gate_y_m - self.y
        distance = math.hypot(dx, dy)
        target_bearing = math.atan2(dy, dx)
        yaw_error = angle_wrap(target_bearing - self.yaw)
        visible = (
            distance <= self.gate_visible_distance_m
            and abs(yaw_error) <= self.gate_fov_rad * 0.5
        )

        msg = GateDetection()
        msg.stamp = self.get_clock().now().to_msg()
        msg.visible = visible
        msg.confidence = 0.95 if visible else 0.0
        msg.center_x = clamp(0.5 + yaw_error / self.gate_fov_rad, 0.0, 1.0)
        msg.center_y = 0.5
        msg.yaw_error_rad = yaw_error
        msg.pitch_error_rad = 0.0
        msg.distance_m = distance
        msg.survey_and_repair_visible = visible
        msg.search_and_rescue_visible = visible
        msg.recommended_side = 'survey_and_repair' if visible else ''
        self.gate_pub.publish(msg)

    def publish_slalom_detection(self):
        marker_index = 1
        marker_x = self.slalom_start_x_m
        for index in range(1, 4):
            candidate_x = self.slalom_start_x_m + (index - 1) * self.slalom_spacing_m
            if self.x < candidate_x + 0.5:
                marker_index = index
                marker_x = candidate_x
                break

        dx = marker_x - self.x
        dy = -0.5 - self.y
        distance = math.hypot(dx, dy)
        yaw_error = angle_wrap(math.atan2(dy, dx) - self.yaw)
        visible = distance < 6.0 and abs(yaw_error) < 0.9

        msg = SlalomMarkerDetection()
        msg.stamp = self.get_clock().now().to_msg()
        msg.marker_index = marker_index
        msg.visible = visible
        msg.confidence = 0.8 if visible else 0.0
        msg.center_x = clamp(0.5 + yaw_error / 1.8, 0.0, 1.0)
        msg.center_y = 0.5
        msg.yaw_error_rad = yaw_error
        msg.distance_m = distance
        msg.left_side_yaw_error_rad = yaw_error
        msg.left_side_lateral_offset_m = 0.75
        self.slalom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeUnityBackend()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
