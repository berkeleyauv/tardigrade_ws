"""Jetson-side depth and attitude controller for Tardigrade."""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float64

from tardigrade_interfaces.msg import PidDebug
from tardigrade_interfaces.srv import SetControlAxes, SetPidGains


AXES = ('roll', 'pitch', 'yaw', 'depth')
MAX_GAIN = 100.0
MAX_INTEGRAL_LIMIT = 10.0


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


def valid_gain_request(axis, kp, ki, kd, output_limit):
    """Validate a complete live-tuning request without changing state."""
    if axis not in AXES:
        return False, f'axis must be one of {", ".join(AXES)}'
    values = (kp, ki, kd, output_limit)
    if not all(math.isfinite(float(value)) for value in values):
        return False, 'gain values must be finite'
    if any(value < 0.0 or value > MAX_GAIN for value in (kp, ki, kd)):
        return False, f'gains must be in [0, {MAX_GAIN:g}]'
    if output_limit <= 0.0 or output_limit > 1.0:
        return False, 'output_limit must be in (0, 1]'
    return True, 'ok'


class DepthAttitudeController(Node):
    """Convert manual/mission intent and odometry into a body-frame wrench."""

    def __init__(self, **node_kwargs):
        super().__init__('depth_attitude_controller', **node_kwargs)

        self.declare_parameter('input_cmd_topic', '/tardigrade/cmd_vel/manual')
        self.declare_parameter('output_cmd_topic', '/tardigrade/cmd_vel')
        self.declare_parameter(
            'enable_topic', '/tardigrade/teleop/enabled')
        self.declare_parameter(
            'odometry_topic', '/tardigrade/state/odometry/filtered')
        self.declare_parameter(
            'depth_target_topic', '/tardigrade/depth_target')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.declare_parameter('enable_timeout_sec', 0.25)
        self.declare_parameter('odometry_timeout_sec', 0.3)
        self.declare_parameter('depth_kp', 0.8)
        self.declare_parameter('depth_ki', 0.0)
        self.declare_parameter('depth_kd', 0.25)
        self.declare_parameter('depth_integral_limit', 0.3)
        self.declare_parameter('depth_deadband_m', 0.02)
        self.declare_parameter('max_heave_command', 0.20)
        self.declare_parameter('roll_kp', 0.8)
        self.declare_parameter('roll_ki', 0.0)
        self.declare_parameter('roll_kd', 0.15)
        self.declare_parameter('pitch_kp', 0.8)
        self.declare_parameter('pitch_ki', 0.0)
        self.declare_parameter('pitch_kd', 0.15)
        self.declare_parameter('yaw_kp', 0.7)
        self.declare_parameter('yaw_ki', 0.0)
        self.declare_parameter('yaw_kd', 0.12)
        self.declare_parameter('attitude_integral_limit', 0.25)
        self.declare_parameter('target_roll_rad', 0.0)
        self.declare_parameter('target_pitch_rad', 0.0)
        self.declare_parameter('capture_initial_attitude_target', True)
        self.declare_parameter('max_roll_command', 0.20)
        self.declare_parameter('max_pitch_command', 0.20)
        self.declare_parameter('max_yaw_command', 0.20)
        self.declare_parameter('enable_roll', False)
        self.declare_parameter('enable_pitch', False)
        self.declare_parameter('enable_yaw', False)
        self.declare_parameter('enable_depth', False)

        self.input_cmd_topic = self.get_parameter('input_cmd_topic').value
        self.output_cmd_topic = self.get_parameter('output_cmd_topic').value
        self.enable_topic = self.get_parameter('enable_topic').value
        self.odometry_topic = self.get_parameter('odometry_topic').value
        self.depth_target_topic = self.get_parameter(
            'depth_target_topic').value
        control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.cmd_timeout_sec = float(
            self.get_parameter('cmd_timeout_sec').value)
        self.enable_timeout_sec = float(
            self.get_parameter('enable_timeout_sec').value)
        self.odometry_timeout_sec = float(
            self.get_parameter('odometry_timeout_sec').value)

        self._load_control_parameters()

        self.latest_cmd = Twist()
        self.latest_cmd_ns = None
        self.latest_enable = False
        self.latest_enable_ns = None
        self.latest_odom = None
        self.latest_odom_ns = None
        self.external_target_z = None
        self.target_z = None
        self.target_yaw = 0.0
        self.attitude_target_captured = False
        self.last_control_ns = None
        self.integrals = {axis: 0.0 for axis in AXES}

        self.cmd_sub = self.create_subscription(
            Twist, self.input_cmd_topic, self.cmd_callback, 10)
        self.enable_sub = self.create_subscription(
            Bool, self.enable_topic, self.enable_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.odometry_topic, self.odom_callback, 10)
        self.depth_target_sub = self.create_subscription(
            Float64, self.depth_target_topic, self.depth_target_callback, 10)
        self.cmd_pub = self.create_publisher(
            Twist, self.output_cmd_topic, 10)
        self.enabled_pub = self.create_publisher(
            Bool, '/tardigrade/control/enabled', 10)
        self.odom_fresh_pub = self.create_publisher(
            Bool, '/tardigrade/control/odometry_fresh', 10)
        self.command_fresh_pub = self.create_publisher(
            Bool, '/tardigrade/control/command_fresh', 10)
        self.axis_enabled_pubs = {
            axis: self.create_publisher(
                Bool, f'/tardigrade/control/{axis}/enabled', 10)
            for axis in AXES
        }
        self.debug_pubs = {
            axis: self.create_publisher(
                PidDebug, f'/tardigrade/control/{axis}/debug', 10)
            for axis in AXES
        }
        self.gains_service = self.create_service(
            SetPidGains,
            '/tardigrade/control/set_pid_gains',
            self.set_pid_gains_callback,
        )
        self.axes_service = self.create_service(
            SetControlAxes,
            '/tardigrade/control/set_axes_enabled',
            self.set_control_axes_callback,
        )
        self.add_on_set_parameters_callback(self.parameters_callback)
        self.timer = self.create_timer(
            1.0 / max(control_rate_hz, 1.0), self.control)

        self.get_logger().info(
            f'Assisted command path: {self.input_cmd_topic} -> '
            f'{self.output_cmd_topic}; enable={self.enable_topic}')
        self.get_logger().info(f'Odometry input: {self.odometry_topic}')
        self.get_logger().info(
            'All PID axes start from the gains YAML enable settings; '
            'enable one axis at a time for pool tuning.')

    def _load_control_parameters(self):
        for axis in AXES:
            setattr(self, f'{axis}_kp', float(
                self.get_parameter(f'{axis}_kp').value))
            setattr(self, f'{axis}_ki', float(
                self.get_parameter(f'{axis}_ki').value))
            setattr(self, f'{axis}_kd', float(
                self.get_parameter(f'{axis}_kd').value))
            setattr(self, f'enable_{axis}', bool(
                self.get_parameter(f'enable_{axis}').value))
        self.depth_integral_limit = abs(float(
            self.get_parameter('depth_integral_limit').value))
        self.attitude_integral_limit = abs(float(
            self.get_parameter('attitude_integral_limit').value))
        self.depth_deadband_m = abs(float(
            self.get_parameter('depth_deadband_m').value))
        self.target_roll = float(
            self.get_parameter('target_roll_rad').value)
        self.target_pitch = float(
            self.get_parameter('target_pitch_rad').value)
        self.capture_initial_attitude_target = bool(
            self.get_parameter('capture_initial_attitude_target').value)
        self.max_heave_command = abs(float(
            self.get_parameter('max_heave_command').value))
        self.max_roll_command = abs(float(
            self.get_parameter('max_roll_command').value))
        self.max_pitch_command = abs(float(
            self.get_parameter('max_pitch_command').value))
        self.max_yaw_command = abs(float(
            self.get_parameter('max_yaw_command').value))

    def cmd_callback(self, message):
        self.latest_cmd = message
        self.latest_cmd_ns = self.get_clock().now().nanoseconds

    def enable_callback(self, message):
        self.latest_enable = bool(message.data)
        self.latest_enable_ns = self.get_clock().now().nanoseconds

    def odom_callback(self, message):
        self.latest_odom = message
        self.latest_odom_ns = self.get_clock().now().nanoseconds

    def depth_target_callback(self, message):
        if math.isfinite(float(message.data)):
            self.external_target_z = float(message.data)
            self.target_z = self.external_target_z

    @staticmethod
    def is_fresh(received_ns, timeout_sec, now_ns):
        if received_ns is None or now_ns < received_ns:
            return False
        return (now_ns - received_ns) / 1e9 <= timeout_sec

    def _reset_control_state(self):
        self.target_z = None
        self.attitude_target_captured = False
        for axis in AXES:
            self.integrals[axis] = 0.0

    def _gate_is_open(self, now_ns):
        return (
            self.latest_enable
            and self.is_fresh(
                self.latest_enable_ns, self.enable_timeout_sec, now_ns)
            and self.is_fresh(
                self.latest_cmd_ns, self.cmd_timeout_sec, now_ns)
            and self.is_fresh(
                self.latest_odom_ns, self.odometry_timeout_sec, now_ns)
        )

    def _publish_enabled_status(self, controller_enabled):
        message = Bool()
        message.data = controller_enabled
        self.enabled_pub.publish(message)
        for axis in AXES:
            axis_message = Bool()
            axis_message.data = (
                controller_enabled and getattr(self, f'enable_{axis}'))
            self.axis_enabled_pubs[axis].publish(axis_message)

    def _publish_freshness(self, now_ns):
        odom_message = Bool()
        odom_message.data = self.is_fresh(
            self.latest_odom_ns, self.odometry_timeout_sec, now_ns)
        self.odom_fresh_pub.publish(odom_message)
        command_message = Bool()
        command_message.data = (
            self.is_fresh(
                self.latest_cmd_ns, self.cmd_timeout_sec, now_ns)
            and self.is_fresh(
                self.latest_enable_ns, self.enable_timeout_sec, now_ns)
        )
        self.command_fresh_pub.publish(command_message)

    def _publish_debug(self, axis, setpoint, measurement, error,
                       p_term, i_term, d_term, raw_output, limit):
        message = PidDebug()
        message.stamp = self.get_clock().now().to_msg()
        message.axis = axis
        message.setpoint = float(setpoint)
        message.measurement = float(measurement)
        message.error = float(error)
        message.kp = float(getattr(self, f'{axis}_kp'))
        message.ki = float(getattr(self, f'{axis}_ki'))
        message.kd = float(getattr(self, f'{axis}_kd'))
        message.p_term = float(p_term)
        message.i_term = float(i_term)
        message.d_term = float(d_term)
        message.output = float(clamp(raw_output, limit))
        message.output_limit = float(limit)
        message.saturated = abs(raw_output) > limit
        self.debug_pubs[axis].publish(message)

    def _pid_terms(self, axis, error, measured_rate, dt, integral_limit):
        self.integrals[axis] = clamp(
            self.integrals[axis] + error * dt, integral_limit)
        p_term = getattr(self, f'{axis}_kp') * error
        i_term = getattr(self, f'{axis}_ki') * self.integrals[axis]
        d_term = -getattr(self, f'{axis}_kd') * measured_rate
        return p_term, i_term, d_term

    def control(self):
        now_ns = self.get_clock().now().nanoseconds
        self._publish_freshness(now_ns)
        if not self._gate_is_open(now_ns):
            self._reset_control_state()
            self.last_control_ns = now_ns
            self.cmd_pub.publish(Twist())
            self._publish_enabled_status(False)
            return

        dt = 0.0
        if self.last_control_ns is not None:
            dt = max(0.0, min(0.2, (now_ns - self.last_control_ns) / 1e9))
        self.last_control_ns = now_ns

        position = self.latest_odom.pose.pose.position
        twist = self.latest_odom.twist.twist
        roll, pitch, yaw = euler_from_quaternion(
            self.latest_odom.pose.pose.orientation)

        if self.external_target_z is not None:
            self.target_z = self.external_target_z
        elif self.target_z is None:
            self.target_z = position.z

        if (self.capture_initial_attitude_target
                and not self.attitude_target_captured):
            self.target_roll = roll
            self.target_pitch = pitch
            self.target_yaw = yaw
            self.attitude_target_captured = True

        output = Twist()
        output.linear.x = self.latest_cmd.linear.x
        output.linear.y = self.latest_cmd.linear.y

        if self.enable_depth:
            if self.external_target_z is None:
                self.target_z += self.latest_cmd.linear.z * dt
            error = self.target_z - position.z
            if abs(error) < self.depth_deadband_m:
                error = 0.0
            terms = self._pid_terms(
                'depth', error, twist.linear.z, dt,
                self.depth_integral_limit)
            raw = sum(terms)
            output.linear.z = clamp(raw, self.max_heave_command)
            self._publish_debug(
                'depth', self.target_z, position.z, error,
                *terms, raw, self.max_heave_command)
        else:
            self.integrals['depth'] = 0.0
            output.linear.z = self.latest_cmd.linear.z
            error = self.target_z - position.z
            self._publish_debug(
                'depth', self.target_z, position.z, error,
                0.0, 0.0, 0.0, 0.0, self.max_heave_command)

        targets = {
            'roll': self.target_roll,
            'pitch': self.target_pitch,
            'yaw': self.target_yaw,
        }
        measurements = {'roll': roll, 'pitch': pitch, 'yaw': yaw}
        rates = {
            'roll': twist.angular.x,
            'pitch': twist.angular.y,
            'yaw': twist.angular.z,
        }
        limits = {
            'roll': self.max_roll_command,
            'pitch': self.max_pitch_command,
            'yaw': self.max_yaw_command,
        }
        if self.enable_yaw:
            self.target_yaw = angle_error(
                self.target_yaw + self.latest_cmd.angular.z * dt, 0.0)
            targets['yaw'] = self.target_yaw

        for axis in ('roll', 'pitch', 'yaw'):
            if not getattr(self, f'enable_{axis}'):
                self.integrals[axis] = 0.0
                error = angle_error(targets[axis], measurements[axis])
                self._publish_debug(
                    axis, targets[axis], measurements[axis], error,
                    0.0, 0.0, 0.0, 0.0, limits[axis])
                continue
            error = angle_error(targets[axis], measurements[axis])
            terms = self._pid_terms(
                axis, error, rates[axis], dt,
                self.attitude_integral_limit)
            raw = sum(terms)
            value = clamp(raw, limits[axis])
            component = {'roll': 'x', 'pitch': 'y', 'yaw': 'z'}[axis]
            setattr(output.angular, component, value)
            self._publish_debug(
                axis, targets[axis], measurements[axis], error,
                *terms, raw, limits[axis])

        if not self.enable_yaw:
            output.angular.z = self.latest_cmd.angular.z

        self.cmd_pub.publish(output)
        self._publish_enabled_status(True)

    def parameters_callback(self, parameters):
        candidate = {
            parameter.name: parameter.value for parameter in parameters
        }
        gain_names = [
            f'{axis}_{term}' for axis in AXES for term in ('kp', 'ki', 'kd')
        ]
        for name in gain_names:
            if name not in candidate:
                continue
            value = candidate[name]
            if (not isinstance(value, (float, int))
                    or not math.isfinite(float(value))
                    or value < 0.0 or value > MAX_GAIN):
                return SetParametersResult(
                    successful=False,
                    reason=f'{name} must be finite and in [0, {MAX_GAIN:g}]',
                )
        output_names = (
            'max_heave_command', 'max_roll_command',
            'max_pitch_command', 'max_yaw_command',
        )
        for name in output_names:
            if name in candidate and (
                    not isinstance(candidate[name], (float, int))
                    or not math.isfinite(float(candidate[name]))
                    or candidate[name] <= 0.0 or candidate[name] > 1.0):
                return SetParametersResult(
                    successful=False,
                    reason=f'{name} must be finite and in (0, 1]',
                )
        for name in ('depth_integral_limit', 'attitude_integral_limit'):
            if name in candidate and (
                    not isinstance(candidate[name], (float, int))
                    or not math.isfinite(float(candidate[name]))
                    or candidate[name] < 0.0
                    or candidate[name] > MAX_INTEGRAL_LIMIT):
                return SetParametersResult(
                    successful=False,
                    reason=(f'{name} must be finite and in '
                            f'[0, {MAX_INTEGRAL_LIMIT:g}]'),
                )

        for parameter in parameters:
            if hasattr(self, parameter.name):
                setattr(self, parameter.name, parameter.value)
        if any(name.startswith('enable_') for name in candidate):
            self._reset_control_state()
        return SetParametersResult(successful=True)

    def set_pid_gains_callback(self, request, response):
        axis = request.axis.strip().lower()
        valid, reason = valid_gain_request(
            axis, request.kp, request.ki, request.kd,
            request.output_limit)
        if not valid:
            response.success = False
            response.message = reason
            return response

        output_name = {
            'depth': 'max_heave_command',
            'roll': 'max_roll_command',
            'pitch': 'max_pitch_command',
            'yaw': 'max_yaw_command',
        }[axis]
        result = self.set_parameters_atomically([
            Parameter(f'{axis}_kp', value=request.kp),
            Parameter(f'{axis}_ki', value=request.ki),
            Parameter(f'{axis}_kd', value=request.kd),
            Parameter(output_name, value=request.output_limit),
        ])
        response.success = result.successful
        response.message = (
            result.reason if not result.successful else 'updated')
        return response

    def set_control_axes_callback(self, request, response):
        result = self.set_parameters_atomically([
            Parameter('enable_roll', value=request.roll),
            Parameter('enable_pitch', value=request.pitch),
            Parameter('enable_yaw', value=request.yaw),
            Parameter('enable_depth', value=request.depth),
        ])
        response.success = result.successful
        response.message = (
            result.reason if not result.successful else 'updated')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DepthAttitudeController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
