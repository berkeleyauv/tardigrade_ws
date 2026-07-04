import threading

import rclpy
from rclpy.node import Node
from pymavlink import mavutil

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
        self.declare_parameter('visual_odometry_topic', '/tardigrade/state/odometry')
        self.declare_parameter('send_visual_odometry', True)
        self.declare_parameter('visual_position_variance', 0.05)
        self.declare_parameter('visual_orientation_variance', 0.05)

        self.device = self.get_parameter('device').value
        self.baudrate = self.get_parameter('baudrate').value
        source_system = self.get_parameter('source_system').value
        source_component = self.get_parameter('source_component').value
        self.offboard_thrust = self.get_parameter('offboard_thrust').value
        self.visual_odometry_topic = self.get_parameter('visual_odometry_topic').value
        self.send_visual_odometry = self.get_parameter('send_visual_odometry').value
        visual_position_variance = float(self.get_parameter('visual_position_variance').value)
        visual_orientation_variance = float(self.get_parameter('visual_orientation_variance').value)

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
        self.external_control_enabled = False
        self.boot_time_ns = self.get_clock().now().nanoseconds
        self.lock = threading.Lock()
        self.visual_covariance = covariance_upper_triangle(
            visual_position_variance,
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

        self.read_timer = self.create_timer(0.02, self.read_mavlink)
        self.status_timer = self.create_timer(0.2, self.publish_robot_status)
        self.offboard_timer = self.create_timer(0.1, self.publish_offboard_setpoint)

        self.get_logger().info(f'Opening MAVLink serial: {self.device} @ {self.baudrate}')
        self.get_logger().info('Services: /tardigrade/set_armed, /tardigrade/set_external_control')
        self.get_logger().info('Publishing: /tardigrade/status')
        if self.send_visual_odometry:
            self.get_logger().info(f'Sending visual odometry from: {self.visual_odometry_topic}')

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

    def publish_offboard_setpoint(self):
        if not self.external_control_enabled:
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

    def visual_odometry_callback(self, msg):
        if not self.send_visual_odometry:
            return

        position = enu_to_ned_point(msg.pose.pose.position)
        q = quaternion_enu_flu_to_ned_frd(msg.pose.pose.orientation)
        velocity = enu_to_ned_point(msg.twist.twist.linear)
        angular_velocity = flu_to_frd_vector(msg.twist.twist.angular)

        self.mav.mav.odometry_send(
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
            self.visual_covariance,
            self.visual_covariance,
            0,
            mavutil.mavlink.MAV_ESTIMATOR_TYPE_VISION,
        )

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
                self.publish_offboard_setpoint()
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
                response.message = 'MAVLink OFFBOARD mode command sent'
            else:
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
