"""Body-velocity feedback controller with physical wrench output."""

import math

import rclpy
from geometry_msgs.msg import TwistStamped, WrenchStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from tardigrade_description.vehicle_model import load_vehicle_model
from tardigrade_interfaces.msg import AllocationStatus, PidDebug
from tardigrade_interfaces.srv import SetVelocityPidGains


AXES = ('surge', 'sway', 'heave', 'roll', 'pitch', 'yaw')
GAIN_FIELDS = ('kp', 'ki', 'kd', 'integral_limit', 'output_limit')
MAX_GAIN = 1000.0
MAX_INTEGRAL_LIMIT = 1000.0
MAX_OUTPUT_LIMIT = 1000.0


def clamp(value, limit):
    return max(-limit, min(limit, value))


def antiwindup_integral_rate(error, ki, gain, requested, achieved):
    """Return integral error rate with achieved-wrench back-calculation."""
    if ki <= 1e-9 or requested is None or achieved is None:
        return error
    return error + gain * (achieved - requested) / ki


def valid_velocity_gain_request(axis, kp, ki, kd, integral_limit,
                                output_limit):
    """Validate one complete physical-unit velocity PID configuration."""
    if axis not in AXES:
        return False, f'axis must be one of {", ".join(AXES)}'
    values = (kp, ki, kd, integral_limit, output_limit)
    if not all(math.isfinite(float(value)) for value in values):
        return False, 'gain values must be finite'
    if any(value < 0.0 or value > MAX_GAIN for value in (kp, ki, kd)):
        return False, f'gains must be in [0, {MAX_GAIN:g}]'
    if integral_limit < 0.0 or integral_limit > MAX_INTEGRAL_LIMIT:
        return False, (
            f'integral_limit must be in [0, {MAX_INTEGRAL_LIMIT:g}]')
    if output_limit <= 0.0 or output_limit > MAX_OUTPUT_LIMIT:
        return False, (
            f'output_limit must be in (0, {MAX_OUTPUT_LIMIT:g}]')
    return True, 'ok'


class VelocityWrenchController(Node):
    """PID velocity loop using filtered odometry as its only state input."""

    def __init__(self, **node_kwargs):
        super().__init__('velocity_wrench_controller', **node_kwargs)
        self.declare_parameter(
            'setpoint_topic', '/tardigrade/control/velocity_setpoint')
        self.declare_parameter(
            'odometry_topic', '/tardigrade/state/odometry/filtered')
        self.declare_parameter(
            'output_topic', '/tardigrade/control/wrench_command')
        self.declare_parameter(
            'allocation_status_topic',
            '/tardigrade/control/allocation_status')
        self.declare_parameter(
            'enabled_topic',
            '/tardigrade/control/velocity_setpoint_enabled')
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('setpoint_timeout_sec', 0.5)
        self.declare_parameter('odometry_timeout_sec', 0.25)
        self.declare_parameter('allocation_status_timeout_sec', 0.25)
        self.declare_parameter('hold_depth_when_heave_zero', True)
        self.declare_parameter('depth_position_kp_n_per_m', 80.0)
        self.declare_parameter('antiwindup_gain', 0.5)
        self.declare_parameter('derivative_filter_alpha', 0.2)
        self.declare_parameter('enable_drag_feedforward', False)
        self.declare_parameter('vehicle_config', '')
        defaults = {
            'surge': (45.0, 3.0, 3.0, 80.0),
            'sway': (55.0, 3.0, 4.0, 80.0),
            'heave': (65.0, 5.0, 5.0, 120.0),
            'roll': (10.0, 0.5, 1.5, 20.0),
            'pitch': (12.0, 0.5, 1.8, 20.0),
            'yaw': (14.0, 0.8, 2.0, 25.0),
        }
        for axis, (kp, ki, kd, limit) in defaults.items():
            self.declare_parameter(f'{axis}.kp', kp)
            self.declare_parameter(f'{axis}.ki', ki)
            self.declare_parameter(f'{axis}.kd', kd)
            self.declare_parameter(
                f'{axis}.integral_limit', limit / max(ki, 1.0))
            self.declare_parameter(f'{axis}.output_limit', limit)

        self.gains = {
            axis: (
                float(self.get_parameter(f'{axis}.kp').value),
                float(self.get_parameter(f'{axis}.ki').value),
                float(self.get_parameter(f'{axis}.kd').value),
                abs(float(self.get_parameter(
                    f'{axis}.integral_limit').value)),
                abs(float(self.get_parameter(f'{axis}.output_limit').value)),
            ) for axis in AXES
        }
        self.setpoint_timeout = float(
            self.get_parameter('setpoint_timeout_sec').value)
        self.odom_timeout = float(
            self.get_parameter('odometry_timeout_sec').value)
        self.allocation_timeout = float(
            self.get_parameter('allocation_status_timeout_sec').value)
        self.hold_depth = bool(
            self.get_parameter('hold_depth_when_heave_zero').value)
        self.depth_position_kp = float(
            self.get_parameter('depth_position_kp_n_per_m').value)
        self.antiwindup_gain = max(
            0.0, float(self.get_parameter('antiwindup_gain').value))
        self.derivative_alpha = max(
            0.0, min(
                1.0,
                float(self.get_parameter('derivative_filter_alpha').value)))
        self.enable_drag_feedforward = bool(
            self.get_parameter('enable_drag_feedforward').value)
        vehicle_path = str(self.get_parameter('vehicle_config').value) or None
        hydrodynamics = load_vehicle_model(vehicle_path).data[
            'hydrodynamics']
        self.linear_damping = [
            float(value) for value in hydrodynamics['linear_damping']]
        self.quadratic_damping = [
            float(value) for value in hydrodynamics['quadratic_damping']]
        self.setpoint = None
        self.setpoint_ns = None
        self.odometry = None
        self.odom_ns = None
        self.last_ns = None
        self.integrals = [0.0] * 6
        self.previous_measurement = None
        self.filtered_derivative = [0.0] * 6
        self.allocation_requested = None
        self.allocation_achieved = None
        self.allocation_ns = None
        self.enabled = False
        self.enabled_ns = None
        self.target_z = None
        self.current_z = None

        self.setpoint_sub = self.create_subscription(
            TwistStamped,
            str(self.get_parameter('setpoint_topic').value),
            self.on_setpoint,
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            str(self.get_parameter('odometry_topic').value),
            self.on_odometry,
            10,
        )
        self.allocation_sub = self.create_subscription(
            AllocationStatus,
            str(self.get_parameter('allocation_status_topic').value),
            self.on_allocation_status,
            10,
        )
        self.enabled_sub = self.create_subscription(
            Bool,
            str(self.get_parameter('enabled_topic').value),
            self.on_enabled,
            10,
        )
        self.publisher = self.create_publisher(
            WrenchStamped,
            str(self.get_parameter('output_topic').value),
            10,
        )
        self.controller_enabled_pub = self.create_publisher(
            Bool, '/tardigrade/control/enabled', 10)
        self.odom_fresh_pub = self.create_publisher(
            Bool, '/tardigrade/control/odometry_fresh', 10)
        self.command_fresh_pub = self.create_publisher(
            Bool, '/tardigrade/control/command_fresh', 10)
        self.debug_pubs = {
            axis: self.create_publisher(
                PidDebug, f'/tardigrade/control/{axis}/debug', 10)
            for axis in AXES
        }
        self.gains_service = self.create_service(
            SetVelocityPidGains,
            '/tardigrade/control/set_velocity_pid_gains',
            self.set_velocity_pid_gains,
        )
        self.reset_service = self.create_service(
            Trigger,
            '/tardigrade/control/reset_pid',
            self.reset_pid,
        )
        self.add_on_set_parameters_callback(self.parameters_callback)
        rate = float(self.get_parameter('control_rate_hz').value)
        self.timer = self.create_timer(1.0 / max(rate, 1.0), self.control)

    def parameters_callback(self, parameters):
        """Apply gain parameter changes atomically to the running loop."""
        prospective = {
            axis: list(values) for axis, values in self.gains.items()
        }
        changed_axes = set()
        for parameter in parameters:
            parts = parameter.name.split('.')
            if len(parts) != 2 or parts[0] not in AXES or \
                    parts[1] not in GAIN_FIELDS:
                continue
            try:
                value = float(parameter.value)
            except (TypeError, ValueError):
                return SetParametersResult(
                    successful=False,
                    reason=f'{parameter.name} must be numeric',
                )
            field_index = GAIN_FIELDS.index(parts[1])
            prospective[parts[0]][field_index] = value
            changed_axes.add(parts[0])

        for axis in changed_axes:
            valid, reason = valid_velocity_gain_request(
                axis, *prospective[axis])
            if not valid:
                return SetParametersResult(
                    successful=False,
                    reason=f'{axis}: {reason}',
                )

        if changed_axes:
            self.gains = {
                axis: tuple(values)
                for axis, values in prospective.items()
            }
            self._reset_control_state()
            self.get_logger().info(
                'Applied live PID gains for ' + ', '.join(
                    sorted(changed_axes)))
        return SetParametersResult(successful=True)

    def set_velocity_pid_gains(self, request, response):
        """Foxglove-friendly service wrapper around live ROS parameters."""
        valid, reason = valid_velocity_gain_request(
            request.axis, request.kp, request.ki, request.kd,
            request.integral_limit, request.output_limit)
        if not valid:
            response.success = False
            response.message = reason
            return response
        values = (
            request.kp, request.ki, request.kd,
            request.integral_limit, request.output_limit,
        )
        result = self.set_parameters_atomically([
            Parameter(f'{request.axis}.{field}', value=float(value))
            for field, value in zip(GAIN_FIELDS, values)
        ])
        response.success = bool(result.successful)
        response.message = (
            f'updated {request.axis}; PID state reset'
            if result.successful else result.reason)
        return response

    def reset_pid(self, request, response):
        """Clear integrators, derivatives, and the captured depth target."""
        del request
        self._reset_control_state()
        response.success = True
        response.message = 'velocity PID state reset'
        return response

    def _reset_control_state(self):
        self.integrals = [0.0] * 6
        self.previous_measurement = None
        self.filtered_derivative = [0.0] * 6
        self.target_z = None

    @staticmethod
    def _publish_bool(publisher, value):
        message = Bool()
        message.data = bool(value)
        publisher.publish(message)

    def _publish_debug(self, now, active, setpoints, measurements, errors,
                       p_terms, i_terms, d_terms, outputs, raw_outputs):
        for index, axis in enumerate(AXES):
            message = PidDebug()
            message.stamp = now.to_msg()
            message.axis = axis
            message.setpoint = float(setpoints[index])
            message.measurement = float(measurements[index])
            message.error = float(errors[index])
            message.kp = float(self.gains[axis][0])
            message.ki = float(self.gains[axis][1])
            message.kd = float(self.gains[axis][2])
            message.p_term = float(p_terms[index])
            message.i_term = float(i_terms[index])
            message.d_term = float(d_terms[index])
            message.output = float(outputs[index])
            message.integral_limit = float(self.gains[axis][3])
            message.output_limit = float(self.gains[axis][4])
            message.saturated = bool(
                active and abs(raw_outputs[index]) > self.gains[axis][4])
            self.debug_pubs[axis].publish(message)

    def on_setpoint(self, message):
        values = self._twist_values(message.twist)
        if message.header.frame_id not in ('', 'base_link') or not all(
                math.isfinite(value) for value in values):
            self.get_logger().error('Rejected invalid velocity setpoint')
            return
        self.setpoint = values
        self.setpoint_ns = self.get_clock().now().nanoseconds

    def on_odometry(self, message):
        values = self._twist_values(message.twist.twist)
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error('Rejected non-finite filtered odometry')
            return
        self.odometry = values
        self.current_z = float(message.pose.pose.position.z)
        self.odom_ns = self.get_clock().now().nanoseconds

    def on_allocation_status(self, message):
        requested = [float(value) for value in message.requested_wrench]
        achieved = [float(value) for value in message.achieved_wrench]
        if not all(math.isfinite(value)
                   for value in requested + achieved):
            self.get_logger().error('Rejected non-finite allocation status')
            return
        self.allocation_requested = requested
        self.allocation_achieved = achieved
        self.allocation_ns = self.get_clock().now().nanoseconds

    def on_enabled(self, message):
        self.enabled = bool(message.data)
        self.enabled_ns = self.get_clock().now().nanoseconds

    @staticmethod
    def _twist_values(twist):
        return (
            twist.linear.x, twist.linear.y, twist.linear.z,
            twist.angular.x, twist.angular.y, twist.angular.z,
        )

    @staticmethod
    def _fresh(received_ns, timeout, now_ns):
        return received_ns is not None and now_ns >= received_ns and \
            (now_ns - received_ns) / 1e9 <= timeout

    def control(self):
        now = self.get_clock().now()
        dt = 0.0 if self.last_ns is None else min(
            max((now.nanoseconds - self.last_ns) / 1e9, 0.0), 0.1)
        self.last_ns = now.nanoseconds
        setpoint_fresh = self._fresh(
            self.setpoint_ns, self.setpoint_timeout, now.nanoseconds)
        enable_fresh = self._fresh(
            self.enabled_ns, self.setpoint_timeout, now.nanoseconds)
        odom_fresh = self._fresh(
            self.odom_ns, self.odom_timeout, now.nanoseconds)
        command_fresh = setpoint_fresh and enable_fresh
        active = (
            self.enabled and self.setpoint is not None and
            self.odometry is not None and
            command_fresh and odom_fresh
        )
        outputs = [0.0] * 6
        raw_outputs = [0.0] * 6
        p_terms = [0.0] * 6
        i_terms = [0.0] * 6
        d_terms = [0.0] * 6
        setpoints = (
            list(self.setpoint) if self.setpoint is not None else [0.0] * 6)
        measurements = (
            list(self.odometry) if self.odometry is not None else [0.0] * 6)
        errors = [
            desired - measured
            for desired, measured in zip(setpoints, measurements)
        ]
        if active:
            if self.target_z is None:
                self.target_z = self.current_z
            self.target_z += self.setpoint[2] * dt
            if self.previous_measurement is None or dt <= 0.0:
                measured_derivative = [0.0] * 6
            else:
                measured_derivative = [
                    (value - previous) / dt
                    for value, previous in zip(
                        self.odometry, self.previous_measurement)
                ]
            self.filtered_derivative = [
                self.derivative_alpha * derivative +
                (1.0 - self.derivative_alpha) * filtered
                for derivative, filtered in zip(
                    measured_derivative, self.filtered_derivative)
            ]
            allocation_fresh = (
                self.allocation_requested is not None and
                self.allocation_achieved is not None and
                self._fresh(self.allocation_ns, self.allocation_timeout,
                            now.nanoseconds)
            )
            for index, axis in enumerate(AXES):
                kp, ki, kd, integral_limit, output_limit = self.gains[axis]
                error = self.setpoint[index] - self.odometry[index]
                requested = None
                achieved = None
                if allocation_fresh:
                    requested = self.allocation_requested[index]
                    achieved = self.allocation_achieved[index]
                integral_rate = antiwindup_integral_rate(
                    error, ki, self.antiwindup_gain,
                    requested, achieved)
                candidate = clamp(
                    self.integrals[index] + integral_rate * dt,
                    integral_limit)
                feedforward = 0.0
                if self.enable_drag_feedforward:
                    desired = self.setpoint[index]
                    feedforward = (
                        self.linear_damping[index] * desired +
                        self.quadratic_damping[index] *
                        abs(desired) * desired)
                raw = (
                    kp * error + ki * candidate -
                    kd * self.filtered_derivative[index] + feedforward)
                p_terms[index] = kp * error
                i_terms[index] = ki * candidate
                d_terms[index] = -kd * self.filtered_derivative[index]
                raw_outputs[index] = raw
                outputs[index] = clamp(raw, output_limit)
                unwinding = (
                    raw > output_limit and integral_rate < 0.0 or
                    raw < -output_limit and integral_rate > 0.0)
                if abs(raw) <= output_limit or unwinding:
                    self.integrals[index] = candidate
            if self.hold_depth and self.current_z is not None:
                depth_correction = self.depth_position_kp * (
                    self.target_z - self.current_z)
                raw_outputs[2] += depth_correction
                outputs[2] = clamp(
                    outputs[2] + depth_correction,
                    self.gains['heave'][4],
                )
            self.previous_measurement = list(self.odometry)
        else:
            self._reset_control_state()

        self._publish_bool(self.controller_enabled_pub, active)
        self._publish_bool(self.odom_fresh_pub, odom_fresh)
        self._publish_bool(self.command_fresh_pub, command_fresh)
        self._publish_debug(
            now, active, setpoints, measurements, errors,
            p_terms, i_terms, d_terms, outputs, raw_outputs)

        message = WrenchStamped()
        message.header.stamp = now.to_msg()
        message.header.frame_id = 'base_link'
        message.wrench.force.x, message.wrench.force.y, \
            message.wrench.force.z = outputs[:3]
        message.wrench.torque.x, message.wrench.torque.y, \
            message.wrench.torque.z = outputs[3:]
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityWrenchController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
