import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String

from tardigrade_interfaces.msg import PidDebug


AXIS_DEFAULTS = {
    'depth': {'kp': 1.2, 'ki': 0.15, 'kd': 0.25, 'setpoint': -1.0},
    'roll': {'kp': 1.0, 'ki': 0.08, 'kd': 0.18, 'setpoint': 0.0},
    'pitch': {'kp': 1.1, 'ki': 0.08, 'kd': 0.20, 'setpoint': 0.0},
    'yaw': {'kp': 0.9, 'ki': 0.05, 'kd': 0.16, 'setpoint': 0.0},
}


def clamp(value, low, high):
    return max(low, min(high, value))


def angle_error(target, measurement):
    return math.atan2(
        math.sin(target - measurement),
        math.cos(target - measurement),
    )


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


class MockController(Node):
    def __init__(self):
        super().__init__('mock_controller')

        self.declare_parameter('enabled', True)
        self.declare_parameter('integral_limit', 0.5)
        for axis, defaults in AXIS_DEFAULTS.items():
            self.declare_parameter(f'{axis}.kp', defaults['kp'])
            self.declare_parameter(f'{axis}.ki', defaults['ki'])
            self.declare_parameter(f'{axis}.kd', defaults['kd'])
            self.declare_parameter(f'{axis}.setpoint', defaults['setpoint'])
            self.declare_parameter(f'{axis}.output_limit', 0.7)

        self.latest_odometry = None
        self.latest_manual_command = Twist()
        self.integrals = {axis: 0.0 for axis in AXIS_DEFAULTS}
        self.last_tick = self.get_clock().now()

        self.create_subscription(
            Odometry,
            '/tardigrade/state/odometry/filtered',
            self.odometry_callback,
            10,
        )
        self.create_subscription(
            Twist,
            '/tardigrade/cmd_vel/manual',
            self.manual_command_callback,
            10,
        )

        self.pid_publishers = {
            axis: self.create_publisher(
                PidDebug,
                f'/tardigrade/control/{axis}/debug',
                10,
            )
            for axis in AXIS_DEFAULTS
        }
        self.command_pub = self.create_publisher(
            Twist,
            '/tardigrade/cmd_vel',
            10,
        )
        self.thruster_pub = self.create_publisher(
            Float32MultiArray,
            '/tardigrade/thrusters/command',
            10,
        )
        self.enabled_pub = self.create_publisher(
            Bool,
            '/tardigrade/control/enabled',
            10,
        )
        self.mode_pub = self.create_publisher(
            String,
            '/tardigrade/control/mode',
            10,
        )

        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info('Publishing mock Jetson PID telemetry')

    def odometry_callback(self, msg):
        self.latest_odometry = msg

    def manual_command_callback(self, msg):
        self.latest_manual_command = msg

    def axis_parameters(self, axis):
        return (
            float(self.get_parameter(f'{axis}.kp').value),
            float(self.get_parameter(f'{axis}.ki').value),
            float(self.get_parameter(f'{axis}.kd').value),
            float(self.get_parameter(f'{axis}.setpoint').value),
            float(self.get_parameter(f'{axis}.output_limit').value),
        )

    def compute_axis(self, axis, measurement, measurement_rate, dt):
        kp, ki, kd, setpoint, output_limit = self.axis_parameters(axis)
        error = (
            angle_error(setpoint, measurement)
            if axis in ('roll', 'pitch', 'yaw')
            else setpoint - measurement
        )
        integral_limit = float(self.get_parameter('integral_limit').value)
        self.integrals[axis] = clamp(
            self.integrals[axis] + error * dt,
            -integral_limit,
            integral_limit,
        )

        p_term = kp * error
        i_term = ki * self.integrals[axis]
        d_term = -kd * measurement_rate
        unclamped = p_term + i_term + d_term
        output = clamp(unclamped, -output_limit, output_limit)

        msg = PidDebug()
        msg.stamp = self.get_clock().now().to_msg()
        msg.axis = axis
        msg.setpoint = setpoint
        msg.measurement = measurement
        msg.error = error
        msg.kp = kp
        msg.ki = ki
        msg.kd = kd
        msg.p_term = p_term
        msg.i_term = i_term
        msg.d_term = d_term
        msg.output = output
        msg.output_limit = output_limit
        msg.saturated = abs(unclamped) > output_limit
        self.pid_publishers[axis].publish(msg)
        return output

    def tick(self):
        if self.latest_odometry is None:
            return

        now = self.get_clock().now()
        dt = clamp(
            (now - self.last_tick).nanoseconds / 1_000_000_000.0,
            0.001,
            0.2,
        )
        self.last_tick = now
        odometry = self.latest_odometry
        roll, pitch, yaw = euler_from_quaternion(
            odometry.pose.pose.orientation
        )
        twist = odometry.twist.twist

        enabled = bool(self.get_parameter('enabled').value)
        outputs = {
            'depth': self.compute_axis(
                'depth',
                odometry.pose.pose.position.z,
                twist.linear.z,
                dt,
            ),
            'roll': self.compute_axis('roll', roll, twist.angular.x, dt),
            'pitch': self.compute_axis('pitch', pitch, twist.angular.y, dt),
            'yaw': self.compute_axis('yaw', yaw, twist.angular.z, dt),
        }
        if not enabled:
            outputs = {axis: 0.0 for axis in outputs}

        command = Twist()
        command.linear.x = self.latest_manual_command.linear.x
        command.linear.y = self.latest_manual_command.linear.y
        command.linear.z = outputs['depth']
        command.angular.x = outputs['roll']
        command.angular.y = outputs['pitch']
        command.angular.z = outputs['yaw']
        self.command_pub.publish(command)
        self.publish_thruster_command(command)

        enabled_msg = Bool()
        enabled_msg.data = enabled
        self.enabled_pub.publish(enabled_msg)
        mode_msg = String()
        mode_msg.data = 'depth_attitude_hold' if enabled else 'manual'
        self.mode_pub.publish(mode_msg)

    def publish_thruster_command(self, command):
        surge = command.linear.x
        sway = command.linear.y
        heave = command.linear.z
        roll = command.angular.x
        pitch = command.angular.y
        yaw = command.angular.z
        efforts = [
            heave + roll - pitch,
            heave - roll - pitch,
            heave + roll + pitch,
            heave - roll + pitch,
            surge - sway - yaw,
            surge + sway + yaw,
            surge + sway - yaw,
            surge - sway + yaw,
        ]

        msg = Float32MultiArray()
        msg.data = [clamp(value, -1.0, 1.0) for value in efforts]
        self.thruster_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockController()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
