import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64


def clamp(value, limit):
    return max(-limit, min(limit, value))


def euler_from_quaternion(q):
    roll = math.atan2(
        2.0 * (q.w * q.x + q.y * q.z),
        1.0 - 2.0 * (q.x * q.x + q.y * q.y),
    )
    pitch_term = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, pitch_term)))
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    return roll, pitch, yaw


def roll_pitch_from_quaternion(q):
    return euler_from_quaternion(q)[:2]


def angle_error(target, current):
    """Return the shortest signed angular error in radians."""
    return math.atan2(math.sin(target - current), math.cos(target - current))


class DepthAttitudeController(Node):
    def __init__(self):
        super().__init__('depth_attitude_controller')

        self.declare_parameter('input_cmd_topic', '/tardigrade/cmd_vel/manual')
        self.declare_parameter('output_cmd_topic', '/tardigrade/cmd_vel')
        self.declare_parameter('odometry_topic', '/tardigrade/state/odometry')
        self.declare_parameter('depth_target_topic', '/tardigrade/depth_target')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.declare_parameter('odometry_timeout_sec', 0.3)
        self.declare_parameter('depth_kp', 0.8)
        self.declare_parameter('depth_ki', 0.0)
        self.declare_parameter('depth_kd', 0.25)
        self.declare_parameter('depth_integral_limit', 0.3)
        self.declare_parameter('depth_deadband_m', 0.02)
        self.declare_parameter('max_heave_command', 0.35)
        self.declare_parameter('roll_kp', 0.8)
        self.declare_parameter('roll_ki', 0.05)
        self.declare_parameter('roll_kd', 0.15)
        self.declare_parameter('pitch_kp', 0.8)
        self.declare_parameter('pitch_ki', 0.05)
        self.declare_parameter('pitch_kd', 0.15)
        self.declare_parameter('yaw_kp', 0.7)
        self.declare_parameter('yaw_ki', 0.03)
        self.declare_parameter('yaw_kd', 0.12)
        self.declare_parameter('attitude_integral_limit', 0.25)
        self.declare_parameter('target_roll_rad', 0.0)
        self.declare_parameter('target_pitch_rad', 0.0)
        self.declare_parameter('capture_initial_attitude_target', True)
        self.declare_parameter('max_attitude_command', 0.25)
        self.declare_parameter('max_yaw_command', 0.25)
        self.declare_parameter('enable_depth_hold', True)
        self.declare_parameter('enable_attitude_hold', True)
        self.declare_parameter('log_rate_hz', 2.0)

        self.input_cmd_topic = self.get_parameter('input_cmd_topic').value
        self.output_cmd_topic = self.get_parameter('output_cmd_topic').value
        self.odometry_topic = self.get_parameter('odometry_topic').value
        self.depth_target_topic = self.get_parameter('depth_target_topic').value
        control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)
        self.odometry_timeout_sec = float(
            self.get_parameter('odometry_timeout_sec').value
        )
        self.depth_kp = float(self.get_parameter('depth_kp').value)
        self.depth_ki = float(self.get_parameter('depth_ki').value)
        self.depth_kd = float(self.get_parameter('depth_kd').value)
        self.depth_integral_limit = abs(float(
            self.get_parameter('depth_integral_limit').value
        ))
        self.depth_deadband_m = abs(float(
            self.get_parameter('depth_deadband_m').value
        ))
        self.max_heave_command = abs(float(
            self.get_parameter('max_heave_command').value
        ))
        self.roll_kp = float(self.get_parameter('roll_kp').value)
        self.roll_ki = float(self.get_parameter('roll_ki').value)
        self.roll_kd = float(self.get_parameter('roll_kd').value)
        self.pitch_kp = float(self.get_parameter('pitch_kp').value)
        self.pitch_ki = float(self.get_parameter('pitch_ki').value)
        self.pitch_kd = float(self.get_parameter('pitch_kd').value)
        self.yaw_kp = float(self.get_parameter('yaw_kp').value)
        self.yaw_ki = float(self.get_parameter('yaw_ki').value)
        self.yaw_kd = float(self.get_parameter('yaw_kd').value)
        self.attitude_integral_limit = abs(float(
            self.get_parameter('attitude_integral_limit').value
        ))
        self.target_roll = float(self.get_parameter('target_roll_rad').value)
        self.target_pitch = float(self.get_parameter('target_pitch_rad').value)
        self.capture_initial_attitude_target = bool(
            self.get_parameter('capture_initial_attitude_target').value
        )
        self.max_attitude_command = abs(float(
            self.get_parameter('max_attitude_command').value
        ))
        self.max_yaw_command = abs(float(
            self.get_parameter('max_yaw_command').value
        ))
        self.enable_depth_hold = bool(self.get_parameter('enable_depth_hold').value)
        self.enable_attitude_hold = bool(
            self.get_parameter('enable_attitude_hold').value
        )
        log_rate_hz = float(self.get_parameter('log_rate_hz').value)

        self.latest_cmd = Twist()
        self.latest_cmd_ns = None
        self.latest_odom = None
        self.latest_odom_ns = None
        self.target_z = None
        self.external_target_z = None
        self.depth_integral = 0.0
        self.attitude_target_captured = False
        self.target_yaw = 0.0
        self.roll_integral = 0.0
        self.pitch_integral = 0.0
        self.yaw_integral = 0.0
        self.last_control_ns = None
        self.last_log_ns = None

        self.cmd_sub = self.create_subscription(
            Twist,
            self.input_cmd_topic,
            self.cmd_callback,
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odometry_topic,
            self.odom_callback,
            10,
        )
        self.depth_target_sub = self.create_subscription(
            Float64,
            self.depth_target_topic,
            self.depth_target_callback,
            10,
        )
        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_topic, 10)
        self.timer = self.create_timer(
            1.0 / max(control_rate_hz, 1.0),
            self.control,
        )
        self.log_period_ns = int(1_000_000_000 / max(log_rate_hz, 0.1))

        self.get_logger().info(
            f'Command path: {self.input_cmd_topic} -> {self.output_cmd_topic}'
        )
        self.get_logger().info(f'Odometry input: {self.odometry_topic}')
        self.get_logger().info(
            'Depth and attitude targets will engage after fresh odometry and '
            'a fresh operator command are both available.'
        )

    def cmd_callback(self, msg):
        self.latest_cmd = msg
        self.latest_cmd_ns = self.get_clock().now().nanoseconds

    def odom_callback(self, msg):
        self.latest_odom = msg
        self.latest_odom_ns = self.get_clock().now().nanoseconds

    def depth_target_callback(self, msg):
        self.external_target_z = float(msg.data)
        self.target_z = self.external_target_z

    def is_fresh(self, received_ns, timeout_sec, now_ns):
        if received_ns is None:
            return False
        return (now_ns - received_ns) / 1_000_000_000.0 <= timeout_sec

    def control(self):
        now_ns = self.get_clock().now().nanoseconds
        cmd_fresh = self.is_fresh(
            self.latest_cmd_ns,
            self.cmd_timeout_sec,
            now_ns,
        )
        odom_fresh = self.is_fresh(
            self.latest_odom_ns,
            self.odometry_timeout_sec,
            now_ns,
        )

        if not cmd_fresh or not odom_fresh:
            self.target_z = None
            self.depth_integral = 0.0
            self.attitude_target_captured = False
            self.roll_integral = 0.0
            self.pitch_integral = 0.0
            self.yaw_integral = 0.0
            self.last_control_ns = now_ns
            self.cmd_pub.publish(Twist())
            return

        if self.external_target_z is not None:
            self.target_z = self.external_target_z
        elif self.target_z is None:
            self.target_z = self.latest_odom.pose.pose.position.z
            self.get_logger().info(f'Depth target captured: z={self.target_z:.3f} m')

        dt = 0.0
        if self.last_control_ns is not None:
            dt = max(0.0, min(0.2, (now_ns - self.last_control_ns) / 1e9))
        self.last_control_ns = now_ns

        output = Twist()
        output.linear.x = self.latest_cmd.linear.x
        output.linear.y = self.latest_cmd.linear.y

        # Manual z is a target-depth rate unless an autonomous absolute target
        # has been supplied. Releasing r/f holds the new depth.
        if self.external_target_z is None:
            self.target_z += self.latest_cmd.linear.z * dt

        position = self.latest_odom.pose.pose.position
        twist = self.latest_odom.twist.twist
        depth_error = self.target_z - position.z
        if abs(depth_error) < self.depth_deadband_m:
            depth_error = 0.0

        if self.enable_depth_hold:
            self.depth_integral = clamp(
                self.depth_integral + depth_error * dt,
                self.depth_integral_limit,
            )
            heave = (
                self.depth_kp * depth_error
                + self.depth_ki * self.depth_integral
                - self.depth_kd * twist.linear.z
            )
            output.linear.z = clamp(heave, self.max_heave_command)
        else:
            output.linear.z = self.latest_cmd.linear.z

        roll, pitch, yaw = euler_from_quaternion(
            self.latest_odom.pose.pose.orientation
        )
        if self.capture_initial_attitude_target and not self.attitude_target_captured:
            self.target_roll = roll
            self.target_pitch = pitch
            self.target_yaw = yaw
            self.attitude_target_captured = True
            self.get_logger().info(
                'Attitude target captured: '
                f'roll={math.degrees(roll):.1f} deg, '
                f'pitch={math.degrees(pitch):.1f} deg, '
                f'yaw={math.degrees(yaw):.1f} deg'
            )
        if self.enable_attitude_hold:
            # angular.z is a requested heading rate. Integrating it here means
            # releasing the stick or mission command holds the new VectorNav
            # heading instead of reverting to open-loop yaw thrust.
            self.target_yaw = angle_error(
                self.target_yaw + self.latest_cmd.angular.z * dt,
                0.0,
            )
            roll_error = angle_error(self.target_roll, roll)
            pitch_error = angle_error(self.target_pitch, pitch)
            yaw_error = angle_error(self.target_yaw, yaw)
            self.roll_integral = clamp(
                self.roll_integral + roll_error * dt,
                self.attitude_integral_limit,
            )
            self.pitch_integral = clamp(
                self.pitch_integral + pitch_error * dt,
                self.attitude_integral_limit,
            )
            self.yaw_integral = clamp(
                self.yaw_integral + yaw_error * dt,
                self.attitude_integral_limit,
            )
            roll_command = (
                self.roll_kp * roll_error
                + self.roll_ki * self.roll_integral
                - self.roll_kd * twist.angular.x
            )
            pitch_command = (
                self.pitch_kp * pitch_error
                + self.pitch_ki * self.pitch_integral
                - self.pitch_kd * twist.angular.y
            )
            yaw_command = (
                self.yaw_kp * yaw_error
                + self.yaw_ki * self.yaw_integral
                - self.yaw_kd * twist.angular.z
            )
            output.angular.x = clamp(roll_command, self.max_attitude_command)
            output.angular.y = clamp(pitch_command, self.max_attitude_command)
            output.angular.z = clamp(yaw_command, self.max_yaw_command)
        else:
            output.angular.z = self.latest_cmd.angular.z

        self.cmd_pub.publish(output)
        if self.last_log_ns is None or now_ns - self.last_log_ns >= self.log_period_ns:
            self.last_log_ns = now_ns
            self.get_logger().info(
                f'z={position.z:.3f} target={self.target_z:.3f} '
                f'error={depth_error:.3f} heave={output.linear.z:.3f}; '
                f'rpy=({math.degrees(roll):.1f}, {math.degrees(pitch):.1f}, '
                f'{math.degrees(yaw):.1f}) deg; attitude_cmd=('
                f'{output.angular.x:.3f}, {output.angular.y:.3f}, '
                f'{output.angular.z:.3f})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = DepthAttitudeController()
    try:
        rclpy.spin(node)
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
