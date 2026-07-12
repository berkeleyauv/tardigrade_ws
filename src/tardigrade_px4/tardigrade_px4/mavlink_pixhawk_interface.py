import threading

import rclpy
from rclpy.node import Node
from pymavlink import mavutil

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tardigrade_interfaces.msg import RobotStatus
from tardigrade_interfaces.srv import SetArmed, SetExternalControl
from tardigrade_px4.mavlink_common import (
    connect_mavlink,
    enu_to_ned_point,
    flu_to_frd_vector,
    now_us,
    quaternion_enu_flu_to_ned_frd,
)


PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
MAV_FRAME_BODY_NED = getattr(mavutil.mavlink, 'MAV_FRAME_BODY_NED', 8)
SETPOINT_TYPEMASK_VELOCITY_ONLY = (
    1      # ignore x position
    | 2    # ignore y position
    | 4    # ignore z position
    | 64   # ignore x acceleration
    | 128  # ignore y acceleration
    | 256  # ignore z acceleration
    | 1024 # ignore yaw angle; use yaw rate instead
)


def covariance_upper_triangle(position_variance, orientation_variance):
    return [
        position_variance,
        0.0, position_variance,
        0.0, 0.0, position_variance,
        0.0, 0.0, 0.0, orientation_variance,
        0.0, 0.0, 0.0, 0.0, orientation_variance,
        0.0, 0.0, 0.0, 0.0, 0.0, orientation_variance,
    ]


class MavlinkPixhawkInterface(Node):
    def __init__(self):
        super().__init__('mavlink_pixhawk_interface')

        self.declare_parameter('device', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 921600)
        self.declare_parameter('source_system', 43)
        self.declare_parameter('source_component', 191)
        self.declare_parameter('offboard_thrust', 0.0)
        self.declare_parameter('offboard_setpoint_mode', 'attitude')
        self.declare_parameter('cmd_vel_topic', '/tardigrade/cmd_vel')
        self.declare_parameter('cmd_vel_timeout_sec', 0.5)
        self.declare_parameter('max_forward_speed', 0.0)
        self.declare_parameter('max_lateral_speed', 0.0)
        self.declare_parameter('max_vertical_speed', 0.0)
        self.declare_parameter('max_yaw_rate', 0.0)
        self.declare_parameter('configure_px4_params', False)
        self.declare_parameter('visual_odometry_topic', '/tardigrade/state/odometry')
        self.declare_parameter('send_visual_odometry', True)
        self.declare_parameter('visual_odometry_rate_hz', 30.0)
        self.declare_parameter('visual_odometry_timeout_sec', 0.5)
        self.declare_parameter('offboard_warmup_sec', 1.0)
        self.declare_parameter('offboard_command_retry_sec', 1.0)
        self.declare_parameter('offboard_command_retries', 3)
        self.declare_parameter('visual_position_variance', 0.05)
        self.declare_parameter('visual_orientation_variance', 0.05)
        self.declare_parameter('visual_velocity_variance', 999.0)
        self.declare_parameter('visual_odometry_quality', 100)

        self.device = self.get_parameter('device').value
        self.baudrate = self.get_parameter('baudrate').value
        source_system = self.get_parameter('source_system').value
        source_component = self.get_parameter('source_component').value
        self.offboard_thrust = self.get_parameter('offboard_thrust').value
        self.offboard_setpoint_mode = self.get_parameter('offboard_setpoint_mode').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.cmd_vel_timeout_sec = float(self.get_parameter('cmd_vel_timeout_sec').value)
        self.max_forward_speed = float(self.get_parameter('max_forward_speed').value)
        self.max_lateral_speed = float(self.get_parameter('max_lateral_speed').value)
        self.max_vertical_speed = float(self.get_parameter('max_vertical_speed').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.configure_px4_params = bool(self.get_parameter('configure_px4_params').value)
        self.visual_odometry_topic = self.get_parameter('visual_odometry_topic').value
        self.send_visual_odometry = self.get_parameter('send_visual_odometry').value
        self.visual_odometry_rate_hz = float(self.get_parameter('visual_odometry_rate_hz').value)
        self.visual_odometry_timeout_sec = float(self.get_parameter('visual_odometry_timeout_sec').value)
        self.offboard_warmup_sec = float(self.get_parameter('offboard_warmup_sec').value)
        self.offboard_command_retry_sec = float(self.get_parameter('offboard_command_retry_sec').value)
        self.offboard_command_retries = int(self.get_parameter('offboard_command_retries').value)
        visual_position_variance = float(self.get_parameter('visual_position_variance').value)
        visual_orientation_variance = float(self.get_parameter('visual_orientation_variance').value)
        visual_velocity_variance = float(self.get_parameter('visual_velocity_variance').value)
        self.visual_odometry_quality = int(self.get_parameter('visual_odometry_quality').value)

        self.mav = connect_mavlink(
            self.device,
            self.baudrate,
            source_system=source_system,
            source_component=source_component,
        )

        self.target_system = 1
        self.target_component = 1
        self.last_heartbeat = None
        self.last_command_ack = None
        self.last_status_text = None
        self.latest_visual_odometry = None
        self.latest_visual_odometry_received_ns = None
        self.visual_odometry_sent_count = 0
        self.odometry_send_supports_quality = None
        self.px4_params_sent = False
        self.diagnostic_streams_requested = False
        self.px4_param_values = {}
        self.last_local_position = None
        self.last_local_position_ns = None
        self.last_estimator_status = None
        self.last_estimator_status_ns = None
        self.latest_cmd_vel = Twist()
        self.latest_cmd_vel_received_ns = None
        self.cmd_vel_received_count = 0
        self.external_control_enabled = False
        self.external_control_started_ns = None
        self.offboard_mode_command_count = 0
        self.last_offboard_mode_command_ns = None
        self.boot_time_ns = self.get_clock().now().nanoseconds
        self.lock = threading.Lock()
        self.visual_pose_covariance = covariance_upper_triangle(
            visual_position_variance,
            visual_orientation_variance,
        )
        self.visual_velocity_covariance = covariance_upper_triangle(
            visual_velocity_variance,
            visual_orientation_variance,
        )

        self.robot_status_pub = self.create_publisher(
            RobotStatus,
            '/tardigrade/status',
            10,
        )

        self.set_armed_srv = self.create_service(
            SetArmed,
            '/tardigrade/set_armed',
            self.handle_set_armed,
        )

        self.set_external_control_srv = self.create_service(
            SetExternalControl,
            '/tardigrade/set_external_control',
            self.handle_set_external_control,
        )

        self.visual_odometry_sub = self.create_subscription(
            Odometry,
            self.visual_odometry_topic,
            self.visual_odometry_callback,
            10,
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10,
        )

        self.read_timer = self.create_timer(0.02, self.read_mavlink)
        self.status_timer = self.create_timer(0.2, self.publish_robot_status)
        self.offboard_timer = self.create_timer(0.1, self.publish_offboard_setpoint)
        self.px4_param_timer = self.create_timer(1.0, self.configure_required_px4_params)
        self.diagnostic_timer = self.create_timer(1.0, self.publish_debug_diagnostics)
        visual_period = 1.0 / max(self.visual_odometry_rate_hz, 1.0)
        self.visual_odometry_timer = self.create_timer(visual_period, self.publish_visual_odometry)

        self.get_logger().info(f'Opening MAVLink serial: {self.device} @ {self.baudrate}')
        self.get_logger().info('Services: /tardigrade/set_armed, /tardigrade/set_external_control')
        self.get_logger().info('Publishing: /tardigrade/status')
        self.get_logger().info(
            f'Offboard setpoint mode: {self.offboard_setpoint_mode}; '
            f'subscribing cmd_vel: {self.cmd_vel_topic}'
        )
        if self.send_visual_odometry:
            self.get_logger().info(f'Sending visual odometry from: {self.visual_odometry_topic}')
        self.get_logger().info(
            'Offboard enable behavior: '
            f'warmup={self.offboard_warmup_sec:.1f}s, '
            f'retries={self.offboard_command_retries}, '
            f'retry_period={self.offboard_command_retry_sec:.1f}s'
        )
        if self.configure_px4_params:
            self.get_logger().warn('PX4 runtime parameter configuration is enabled')

    def read_mavlink(self):
        while True:
            msg = self.mav.recv_match(blocking=False)
            if msg is None:
                return

            msg_type = msg.get_type()
            with self.lock:
                if msg_type == 'HEARTBEAT':
                    self.last_heartbeat = msg
                    self.target_system = msg.get_srcSystem()
                    self.target_component = msg.get_srcComponent()
                elif msg_type == 'COMMAND_ACK':
                    self.last_command_ack = msg
                elif msg_type == 'STATUSTEXT':
                    self.last_status_text = msg.text
                    self.get_logger().warn(f'PX4 status text: {msg.text}')
                elif msg_type == 'PARAM_VALUE':
                    param_id = msg.param_id
                    if isinstance(param_id, bytes):
                        param_id = param_id.decode('ascii', errors='ignore')
                    param_id = param_id.rstrip('\x00')
                    self.px4_param_values[param_id] = msg.param_value
                elif msg_type == 'LOCAL_POSITION_NED':
                    self.last_local_position = msg
                    self.last_local_position_ns = self.get_clock().now().nanoseconds
                elif msg_type == 'ESTIMATOR_STATUS':
                    self.last_estimator_status = msg
                    self.last_estimator_status_ns = self.get_clock().now().nanoseconds

    def publish_offboard_setpoint(self):
        if not self.external_control_enabled:
            return

        if self.offboard_setpoint_mode == 'velocity':
            self.publish_velocity_setpoint()
            self.maybe_send_offboard_mode_command()
            return

        self.mav.mav.set_attitude_target_send(
            self.time_boot_ms(),
            self.target_system,
            self.target_component,
            0,
            [1.0, 0.0, 0.0, 0.0],
            0.0,
            0.0,
            0.0,
            self.offboard_thrust,
        )
        self.maybe_send_offboard_mode_command()

    def publish_velocity_setpoint(self):
        cmd = self.current_clamped_cmd_vel()
        self.mav.mav.set_position_target_local_ned_send(
            self.time_boot_ms(),
            self.target_system,
            self.target_component,
            MAV_FRAME_BODY_NED,
            SETPOINT_TYPEMASK_VELOCITY_ONLY,
            0.0,
            0.0,
            0.0,
            cmd.linear.x,
            -cmd.linear.y,
            -cmd.linear.z,
            0.0,
            0.0,
            0.0,
            0.0,
            -cmd.angular.z,
        )

    def current_clamped_cmd_vel(self):
        now_ns = self.get_clock().now().nanoseconds
        with self.lock:
            cmd = self.latest_cmd_vel
            received_ns = self.latest_cmd_vel_received_ns

        if received_ns is None:
            return Twist()

        age_sec = (now_ns - received_ns) / 1_000_000_000.0
        if age_sec > self.cmd_vel_timeout_sec:
            return Twist()

        clamped = Twist()
        clamped.linear.x = self.clamp(
            cmd.linear.x,
            -self.max_forward_speed,
            self.max_forward_speed,
        )
        clamped.linear.y = self.clamp(
            cmd.linear.y,
            -self.max_lateral_speed,
            self.max_lateral_speed,
        )
        clamped.linear.z = self.clamp(
            cmd.linear.z,
            -self.max_vertical_speed,
            self.max_vertical_speed,
        )
        clamped.angular.z = self.clamp(cmd.angular.z, -self.max_yaw_rate, self.max_yaw_rate)
        return clamped

    @staticmethod
    def clamp(value, min_value, max_value):
        return min(max(float(value), min_value), max_value)

    def maybe_send_offboard_mode_command(self):
        now_ns = self.get_clock().now().nanoseconds
        if self.external_control_started_ns is None:
            return

        warmup_ns = int(self.offboard_warmup_sec * 1_000_000_000)
        if now_ns - self.external_control_started_ns < warmup_ns:
            return

        if self.offboard_mode_command_count >= self.offboard_command_retries:
            return

        retry_ns = int(self.offboard_command_retry_sec * 1_000_000_000)
        if (
            self.last_offboard_mode_command_ns is not None
            and now_ns - self.last_offboard_mode_command_ns < retry_ns
        ):
            return

        self.send_offboard_mode_command()
        self.offboard_mode_command_count += 1
        self.last_offboard_mode_command_ns = now_ns

    def send_offboard_mode_command(self):
        self.mav.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
            float(PX4_CUSTOM_MAIN_MODE_OFFBOARD),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        self.get_logger().warn(
            'Sent MAVLink OFFBOARD mode command '
            f'({self.offboard_mode_command_count + 1}/'
            f'{self.offboard_command_retries})'
        )

    def request_message_interval(self, message_id, rate_hz):
        self.mav.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            float(message_id),
            float(1_000_000 / rate_hz),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

    def request_diagnostic_streams(self):
        if self.diagnostic_streams_requested:
            return

        with self.lock:
            heartbeat = self.last_heartbeat

        if heartbeat is None:
            return

        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 5.0)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ESTIMATOR_STATUS, 2.0)
        self.diagnostic_streams_requested = True

    def configure_required_px4_params(self):
        if not self.configure_px4_params or self.px4_params_sent:
            return

        with self.lock:
            heartbeat = self.last_heartbeat

        if heartbeat is None:
            return

        params = {
            'COM_RC_IN_MODE': (4.0, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
            'COM_ARM_WO_GPS': (1.0, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
            'COM_ARM_MIS_REQ': (0.0, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
            'EKF2_EV_CTRL': (3.0, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
            'EKF2_HGT_REF': (3.0, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
            'EKF2_EV_QMIN': (0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
        }

        for name, (value, param_type) in params.items():
            self.mav.mav.param_set_send(
                self.target_system,
                self.target_component,
                name.encode('ascii'),
                value,
                param_type,
            )
            self.mav.mav.param_request_read_send(
                self.target_system,
                self.target_component,
                name.encode('ascii'),
                -1,
            )

        self.px4_params_sent = True
        self.get_logger().warn('Sent required PX4 params in RAM; Pixhawk param save is still not fixed')

    def visual_odometry_callback(self, msg):
        with self.lock:
            self.latest_visual_odometry = msg
            self.latest_visual_odometry_received_ns = self.get_clock().now().nanoseconds

    def cmd_vel_callback(self, msg):
        with self.lock:
            self.latest_cmd_vel = msg
            self.latest_cmd_vel_received_ns = self.get_clock().now().nanoseconds
            self.cmd_vel_received_count += 1

    def publish_visual_odometry(self):
        if not self.send_visual_odometry:
            return

        now_ns = self.get_clock().now().nanoseconds
        with self.lock:
            msg = self.latest_visual_odometry
            received_ns = self.latest_visual_odometry_received_ns

        if msg is None or received_ns is None:
            return

        age_sec = (now_ns - received_ns) / 1_000_000_000.0
        if age_sec > self.visual_odometry_timeout_sec:
            return

        position = enu_to_ned_point(msg.pose.pose.position)
        q = quaternion_enu_flu_to_ned_frd(msg.pose.pose.orientation)
        velocity = enu_to_ned_point(msg.twist.twist.linear)
        angular_velocity = flu_to_frd_vector(msg.twist.twist.angular)

        odometry_args = (
            now_us(),
            mavutil.mavlink.MAV_FRAME_LOCAL_FRD,
            mavutil.mavlink.MAV_FRAME_BODY_FRD,
            position[0],
            position[1],
            position[2],
            q,
            velocity[0],
            velocity[1],
            velocity[2],
            angular_velocity[0],
            angular_velocity[1],
            angular_velocity[2],
            self.visual_pose_covariance,
            self.visual_velocity_covariance,
            0,
            mavutil.mavlink.MAV_ESTIMATOR_TYPE_VISION,
        )

        if self.odometry_send_supports_quality is not False:
            try:
                self.mav.mav.odometry_send(
                    *odometry_args,
                    max(0, min(self.visual_odometry_quality, 100)),
                )
                self.odometry_send_supports_quality = True
            except TypeError:
                if self.odometry_send_supports_quality is True:
                    raise
                self.odometry_send_supports_quality = False
                self.mav.mav.odometry_send(*odometry_args)
        else:
            self.mav.mav.odometry_send(*odometry_args)

        with self.lock:
            self.visual_odometry_sent_count += 1

    def publish_debug_diagnostics(self):
        self.request_diagnostic_streams()

        with self.lock:
            command_ack = self.last_command_ack
            px4_param_values = dict(self.px4_param_values)
            local_position = self.last_local_position
            local_position_ns = self.last_local_position_ns
            estimator_status = self.last_estimator_status
            estimator_status_ns = self.last_estimator_status_ns
            visual_received_ns = self.latest_visual_odometry_received_ns
            visual_sent_count = self.visual_odometry_sent_count
            px4_params_sent = self.px4_params_sent
            cmd_vel_received_ns = self.latest_cmd_vel_received_ns
            cmd_vel_received_count = self.cmd_vel_received_count

        now_ns = self.get_clock().now().nanoseconds
        parts = []

        if command_ack is not None:
            parts.append(f'ack={command_ack.command}/{command_ack.result}')

        if visual_received_ns is not None:
            visual_age_ms = (now_ns - visual_received_ns) / 1_000_000.0
            parts.append(f'visual_odom_age_ms={visual_age_ms:.0f}')
            parts.append(f'visual_odom_sent={visual_sent_count}')

        if self.offboard_setpoint_mode == 'velocity':
            if cmd_vel_received_ns is not None:
                cmd_age_ms = (now_ns - cmd_vel_received_ns) / 1_000_000.0
                parts.append(f'cmd_vel_age_ms={cmd_age_ms:.0f}')
                parts.append(f'cmd_vel_received={cmd_vel_received_count}')
            else:
                parts.append('cmd_vel_received=0')

        if self.configure_px4_params:
            parts.append(f'px4_params_sent={px4_params_sent}')
            tracked_params = [
                'COM_RC_IN_MODE',
                'COM_ARM_WO_GPS',
                'COM_ARM_MIS_REQ',
                'EKF2_EV_CTRL',
                'EKF2_HGT_REF',
                'EKF2_EV_QMIN',
            ]
            param_summary = ','.join(
                f'{name}={px4_param_values[name]:.3g}'
                for name in tracked_params
                if name in px4_param_values
            )
            if param_summary:
                parts.append(f'px4_param_values={param_summary}')

        if local_position is not None and local_position_ns is not None:
            local_age_ms = (now_ns - local_position_ns) / 1_000_000.0
            parts.append(
                'local_position='
                f'age_ms:{local_age_ms:.0f},'
                f'x:{local_position.x:.2f},'
                f'y:{local_position.y:.2f},'
                f'z:{local_position.z:.2f}'
            )
        else:
            parts.append('local_position=none')

        if estimator_status is not None and estimator_status_ns is not None:
            estimator_age_ms = (now_ns - estimator_status_ns) / 1_000_000.0
            parts.append(
                'estimator='
                f'age_ms:{estimator_age_ms:.0f},'
                f'flags:{estimator_status.flags},'
                f'pos_h:{estimator_status.pos_horiz_accuracy:.2f},'
                f'pos_v:{estimator_status.pos_vert_accuracy:.2f}'
            )

        if parts:
            self.get_logger().info('PX4 debug: ' + '; '.join(parts))

    def time_boot_ms(self):
        elapsed_ns = self.get_clock().now().nanoseconds - self.boot_time_ns
        return (elapsed_ns // 1_000_000) & 0xFFFFFFFF

    def handle_set_armed(self, request, response):
        arm_value = 1.0 if request.armed else 0.0

        self.mav.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            arm_value,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

        response.success = True
        response.message = 'MAVLink arm command sent' if request.armed else 'MAVLink disarm command sent'
        return response

    def handle_set_external_control(self, request, response):
        try:
            self.external_control_enabled = request.enabled

            if request.enabled:
                self.external_control_started_ns = self.get_clock().now().nanoseconds
                self.offboard_mode_command_count = 0
                self.last_offboard_mode_command_ns = None
                self.publish_offboard_setpoint()
                response.message = (
                    'External control setpoint stream enabled; '
                    'OFFBOARD mode command will be sent after warmup'
                )
            else:
                self.external_control_started_ns = None
                self.offboard_mode_command_count = 0
                self.last_offboard_mode_command_ns = None
                response.message = 'External control setpoint stream disabled'

            response.success = True
        except Exception as exc:
            self.external_control_enabled = False
            response.success = False
            response.message = f'Failed to set external control: {exc}'
            self.get_logger().error(response.message)

        return response

    def publish_robot_status(self):
        msg = RobotStatus()
        msg.stamp = self.get_clock().now().to_msg()

        with self.lock:
            heartbeat = self.last_heartbeat
            command_ack = self.last_command_ack
            status_text = self.last_status_text
            visual_received_ns = self.latest_visual_odometry_received_ns
            visual_sent_count = self.visual_odometry_sent_count
            px4_params_sent = self.px4_params_sent
            cmd_vel_received_ns = self.latest_cmd_vel_received_ns
            cmd_vel_received_count = self.cmd_vel_received_count

        msg.px4_connected = heartbeat is not None
        msg.external_control_enabled = self.external_control_enabled

        if heartbeat is not None:
            msg.armed = bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            msg.arming_state = 1 if msg.armed else 0
            msg.nav_state = 0
            msg.detail = f'MAVLink heartbeat received; base_mode={heartbeat.base_mode}; custom_mode={heartbeat.custom_mode}'
            if command_ack is not None:
                msg.detail += f'; last_ack_command={command_ack.command}; last_ack_result={command_ack.result}'
            if status_text is not None:
                msg.detail += f'; last_status_text="{status_text}"'
            if visual_received_ns is not None:
                age_ms = (self.get_clock().now().nanoseconds - visual_received_ns) / 1_000_000.0
                msg.detail += f'; visual_odom_age_ms={age_ms:.0f}; visual_odom_sent={visual_sent_count}'
            if self.external_control_enabled:
                msg.detail += (
                    f'; offboard_mode_commands={self.offboard_mode_command_count}'
                    f'/{self.offboard_command_retries}'
                )
            if self.offboard_setpoint_mode == 'velocity':
                if cmd_vel_received_ns is not None:
                    age_ms = (
                        self.get_clock().now().nanoseconds - cmd_vel_received_ns
                    ) / 1_000_000.0
                    msg.detail += (
                        f'; cmd_vel_age_ms={age_ms:.0f}; '
                        f'cmd_vel_received={cmd_vel_received_count}'
                    )
                else:
                    msg.detail += '; cmd_vel_received=0'
            if self.configure_px4_params:
                msg.detail += f'; px4_params_sent={px4_params_sent}'
        else:
            msg.armed = False
            msg.arming_state = 0
            msg.nav_state = 0
            msg.detail = 'No MAVLink heartbeat received'

        self.robot_status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkPixhawkInterface()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
