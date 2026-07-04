import math
import threading

import rclpy
from rclpy.node import Node
from pymavlink import mavutil

from tardigrade_interfaces.msg import RobotStatus
from tardigrade_interfaces.srv import SetArmed, SetExternalControl
from tardigrade_px4.mavlink_common import connect_mavlink


PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6


class MavlinkPixhawkInterface(Node):
    def __init__(self):
        super().__init__('mavlink_pixhawk_interface')

        self.declare_parameter('device', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 921600)
        self.declare_parameter('source_system', 43)
        self.declare_parameter('source_component', 191)

        self.device = self.get_parameter('device').value
        self.baudrate = self.get_parameter('baudrate').value
        source_system = self.get_parameter('source_system').value
        source_component = self.get_parameter('source_component').value

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
        self.external_control_enabled = False
        self.lock = threading.Lock()

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

        self.read_timer = self.create_timer(0.02, self.read_mavlink)
        self.status_timer = self.create_timer(0.2, self.publish_robot_status)
        self.offboard_timer = self.create_timer(0.1, self.publish_offboard_setpoint)

        self.get_logger().info(f'Opening MAVLink serial: {self.device} @ {self.baudrate}')
        self.get_logger().info('Services: /tardigrade/set_armed, /tardigrade/set_external_control')
        self.get_logger().info('Publishing: /tardigrade/status')

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

    def publish_offboard_setpoint(self):
        if not self.external_control_enabled:
            return

        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        )

        self.mav.mav.set_position_target_local_ned_send(
            0,
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            math.nan,
            0.0,
        )

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
        self.external_control_enabled = request.enabled

        if request.enabled:
            self.publish_offboard_setpoint()
            self.mav.mav.set_mode_send(
                self.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                PX4_CUSTOM_MAIN_MODE_OFFBOARD,
            )
            response.message = 'MAVLink OFFBOARD mode command sent'
        else:
            response.message = 'External control setpoint stream disabled'

        response.success = True
        return response

    def publish_robot_status(self):
        msg = RobotStatus()
        msg.stamp = self.get_clock().now().to_msg()

        with self.lock:
            heartbeat = self.last_heartbeat
            command_ack = self.last_command_ack

        msg.px4_connected = heartbeat is not None
        msg.external_control_enabled = self.external_control_enabled

        if heartbeat is not None:
            msg.armed = bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            msg.arming_state = 1 if msg.armed else 0
            msg.nav_state = 0
            msg.detail = f'MAVLink heartbeat received; base_mode={heartbeat.base_mode}; custom_mode={heartbeat.custom_mode}'
            if command_ack is not None:
                msg.detail += f'; last_ack_command={command_ack.command}; last_ack_result={command_ack.result}'
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
