import rclpy
from rclpy.node import Node

from px4_msgs.msg import (
    VehicleStatus,
    VehicleCommand,
    VehicleCommandAck,
    OffboardControlMode,
    TrajectorySetpoint
)

from tardigrade_interfaces.srv import SetArmed, SetExternalControl
from tardigrade_interfaces.msg import RobotStatus

class PixhawkInterface(Node):
    def __init__(self):
        super().__init__('pixhawk_interface')

        self.last_vehicle_status = None
        self.last_command_ack = None
        self.external_control_enabled = False

        # Subscribers
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            'fmu/out/vehicle_status',
            self.vehicle_status_callback,
            10,
        )

        self.vehicle_command_ack_sub = self.create_subscription(
            VehicleCommandAck,
            '/fmu/out/vehicle_command_ack',
            self.vehicle_command_ack_callback,
            10
        )

        # Publishers
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            'fmu/in/vehicle_command',
            10
        )

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            'fmu/in/offboard_control_mode',
            10,
        )

        self.robot_status_pub = self.create_publisher(
            RobotStatus,
            "/tardigrade/status",
            10,
        )

        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            'fmu/in/trajectory_setpoint',
            10
        )

        # Services
        self.set_armed_srv = self.create_service(
            SetArmed,
            "/tardigrade/set_armed",
            self.handle_set_armed,
        )

        self.set_external_control_srv = self.create_service(
            SetExternalControl,
            "/tardigrade/set_external_control",
            self.handle_set_external_control,
        )

        # Timers
        self.heartbeat_timer = self.create_timer(
            0.1,
            self.publish_offboard_heartbeat,
        )

        self.get_logger().info("Pixhawk interface started.")
        self.get_logger().info("Listening: /fmu/out/vehicle_status")
        self.get_logger().info('Listening: /fmu/out/vehicle_command_ack')
        self.get_logger().info("Publishing: /fmu/in/offboard_control_mode")
        self.get_logger().info("Publishing: /fmu/in/vehicle_command")
        self.get_logger().info("Publishing: /fmu/in/trajectory_setpoint")
        self.get_logger().info('Publishing: /tardigrade/status')

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.last_vehicle_status = msg

        self.publish_robot_status()
        
        self.get_logger().info(
            f"PX4 status | arming_state={msg.arming_state}, nav_state={msg.nav_state}",
            throttle_duration_sec=1.0,
        )
    
    def vehicle_command_ack_callback(self, msg: VehicleCommandAck):
        self.last_command_ack = msg

        self.get_logger().info(
            f'PX4 command ack | command={msg.command}, result={msg.result}',
            throttle_duration_sec=1.0
        )

        self.publish_robot_status()

    def publish_offboard_heartbeat(self):
        control_mode = OffboardControlMode()
        control_mode.timestamp = self.get_clock().now().nanoseconds // 1000

        control_mode.position = False
        control_mode.velocity = True
        control_mode.acceleration = False
        control_mode.attitude = False
        control_mode.body_rate = False
        control_mode.actuator = False

        self.offboard_control_mode_pub.publish(control_mode)

        setpoint = TrajectorySetpoint()
        setpoint.timestamp = control_mode.timestamp

        setpoint.position = [float('nan'), float('nan'), float('nan')]
        setpoint.velocity = [0.0, 0.0, 0.0]
        setpoint.acceleration = [float('nan'), float('nan'), float('nan')]
        setpoint.jerk = [float('nan'), float('nan'), float('nan')]
        setpoint.yaw = float('nan')
        setpoint.yawspeed = 0.0

        self.trajectory_setpoint_pub.publish(setpoint)

    def publish_vehicle_command(self, command: int, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000

        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.vehicle_command_pub.publish(msg)

    def handle_set_armed(self, request, response):
        if request.armed:
            self.arm()
            response.message = 'Arm command sent'
        else:
            self.disarm()
            response.message = 'Disarm command sent'
        
        response.success = True
        return response

    def handle_set_external_control(self, request, response):
        self.external_control_enabled = request.enabled

        if request.enabled:
            self.enter_offboard_mode()
            response.message = 'External control enable command sent'
        else:
            response.message = 'External control disabled locally'
        
        response.success = True
        return response
    
    def arm(self):
        self.get_logger().warn("Sending ARM command")
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0
        )

    def disarm(self):
        self.get_logger().warn("Sending DISARM command")
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0
        )

    def enter_offboard_mode(self):
        self.get_logger().warn("Sending OFFBOARD mode command")
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )
    
    def publish_robot_status(self):
        msg = RobotStatus()
        msg.stamp = self.get_clock().now().to_msg()

        msg.px4_connected = self.last_vehicle_status is not None
        msg.external_control_enabled = self.external_control_enabled

        if self.last_vehicle_status is not None:
            msg.arming_state = self.last_vehicle_status.arming_state
            msg.nav_state = self.last_vehicle_status.nav_state
            msg.armed = self.last_vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED
            detail = 'PX4 status received'

            if self.last_command_ack is not None:
                detail += (
                    f'; last_ack_command={self.last_command_ack.command}'
                    f'; last_ack_result={self.last_command_ack.result}'
                )
            
            msg.detail = detail
        else:
            msg.arming_state = 0
            msg.nav_state = 0
            msg.armed = False
            msg.detail = 'No PX4 status received'
        
        self.robot_status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PixhawkInterface()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
