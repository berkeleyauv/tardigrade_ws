"""Outer-loop pose controller producing body velocity setpoints."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool


def normalize_quaternion(values):
    """Return a normalized w, x, y, z quaternion."""
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-9 or not math.isfinite(norm):
        raise ValueError('quaternion must be finite and non-zero')
    return tuple(value / norm for value in values)


def quaternion_multiply(left, right):
    """Multiply w, x, y, z quaternions."""
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


def quaternion_conjugate(value):
    """Return the conjugate of a w, x, y, z quaternion."""
    return value[0], -value[1], -value[2], -value[3]


def rotate_world_to_body(vector, body_to_world):
    """Rotate one ENU world vector into the current FLU body frame."""
    inverse = quaternion_conjugate(body_to_world)
    rotated = quaternion_multiply(
        quaternion_multiply(inverse, (0.0,) + tuple(vector)),
        body_to_world,
    )
    return rotated[1:]


def orientation_error_body(current, target):
    """Return the shortest body-frame rotation vector to target."""
    error = quaternion_multiply(quaternion_conjugate(current), target)
    if error[0] < 0.0:
        error = tuple(-value for value in error)
    vector_norm = math.sqrt(sum(value * value for value in error[1:]))
    if vector_norm < 1e-9:
        return 0.0, 0.0, 0.0
    angle = 2.0 * math.atan2(vector_norm, max(error[0], 0.0))
    return tuple(angle * value / vector_norm for value in error[1:])


def clamp_axes(values, limits):
    """Clamp each value to its corresponding symmetric limit."""
    return tuple(
        max(-abs(limit), min(abs(limit), value))
        for value, limit in zip(values, limits)
    )


class PoseVelocityController(Node):
    """Convert an odom-frame pose target into body velocity and rates."""

    def __init__(self, **node_kwargs):
        super().__init__('pose_velocity_controller', **node_kwargs)
        self.declare_parameter(
            'setpoint_topic', '/tardigrade/control/pose_setpoint')
        self.declare_parameter(
            'odometry_topic', '/tardigrade/state/odometry/filtered')
        self.declare_parameter(
            'output_topic',
            '/tardigrade/control/velocity_setpoint/pose')
        self.declare_parameter(
            'enabled_topic', '/tardigrade/control/pose_setpoint_enabled')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('setpoint_timeout_sec', 1.0)
        self.declare_parameter('odometry_timeout_sec', 0.25)
        self.declare_parameter('position.kp', [0.6, 0.6, 0.7])
        self.declare_parameter(
            'position.max_velocity', [0.5, 0.4, 0.3])
        self.declare_parameter('attitude.kp', [0.8, 0.8, 1.0])
        self.declare_parameter('attitude.max_rate', [0.4, 0.4, 0.5])

        self.position_kp = self._vector_parameter('position.kp')
        self.max_velocity = self._vector_parameter(
            'position.max_velocity')
        self.attitude_kp = self._vector_parameter('attitude.kp')
        self.max_rate = self._vector_parameter('attitude.max_rate')
        self.setpoint_timeout = max(0.0, float(
            self.get_parameter('setpoint_timeout_sec').value))
        self.odometry_timeout = max(0.0, float(
            self.get_parameter('odometry_timeout_sec').value))
        self.setpoint = None
        self.setpoint_ns = None
        self.odometry = None
        self.odometry_ns = None

        self.setpoint_sub = self.create_subscription(
            PoseStamped,
            str(self.get_parameter('setpoint_topic').value),
            self.on_setpoint,
            10,
        )
        self.odometry_sub = self.create_subscription(
            Odometry,
            str(self.get_parameter('odometry_topic').value),
            self.on_odometry,
            10,
        )
        self.publisher = self.create_publisher(
            TwistStamped,
            str(self.get_parameter('output_topic').value),
            10,
        )
        self.enabled_publisher = self.create_publisher(
            Bool,
            str(self.get_parameter('enabled_topic').value),
            10,
        )
        rate = max(1.0, float(
            self.get_parameter('control_rate_hz').value))
        self.timer = self.create_timer(1.0 / rate, self.control)

    def _vector_parameter(self, name):
        values = [float(value) for value in self.get_parameter(name).value]
        if len(values) != 3 or not all(
                math.isfinite(value) for value in values):
            raise ValueError(f'{name} must contain three finite values')
        return values

    def on_setpoint(self, message):
        if message.header.frame_id not in ('', 'odom'):
            self.get_logger().error('Pose setpoint must use the odom frame')
            return
        self.setpoint = message.pose
        self.setpoint_ns = self.get_clock().now().nanoseconds

    def on_odometry(self, message):
        self.odometry = message.pose.pose
        self.odometry_ns = self.get_clock().now().nanoseconds

    @staticmethod
    def _fresh(received_ns, timeout, now_ns):
        return received_ns is not None and now_ns >= received_ns and \
            (now_ns - received_ns) / 1e9 <= timeout

    @staticmethod
    def _quaternion(message):
        return normalize_quaternion(
            (message.w, message.x, message.y, message.z))

    def control(self):
        now = self.get_clock().now()
        output = TwistStamped()
        output.header.stamp = now.to_msg()
        output.header.frame_id = 'base_link'
        active = (
            self.setpoint is not None and self.odometry is not None and
            self._fresh(self.setpoint_ns, self.setpoint_timeout,
                        now.nanoseconds) and
            self._fresh(self.odometry_ns, self.odometry_timeout,
                        now.nanoseconds)
        )
        if active:
            current_q = self._quaternion(self.odometry.orientation)
            target_q = self._quaternion(self.setpoint.orientation)
            world_error = (
                self.setpoint.position.x - self.odometry.position.x,
                self.setpoint.position.y - self.odometry.position.y,
                self.setpoint.position.z - self.odometry.position.z,
            )
            body_error = rotate_world_to_body(world_error, current_q)
            linear = clamp_axes(
                [gain * error for gain, error in zip(
                    self.position_kp, body_error)],
                self.max_velocity,
            )
            attitude_error = orientation_error_body(current_q, target_q)
            angular = clamp_axes(
                [gain * error for gain, error in zip(
                    self.attitude_kp, attitude_error)],
                self.max_rate,
            )
            output.twist.linear.x, output.twist.linear.y, \
                output.twist.linear.z = linear
            output.twist.angular.x, output.twist.angular.y, \
                output.twist.angular.z = angular
        self.publisher.publish(output)
        enabled = Bool()
        enabled.data = active
        self.enabled_publisher.publish(enabled)


def main(args=None):
    rclpy.init(args=args)
    node = PoseVelocityController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
