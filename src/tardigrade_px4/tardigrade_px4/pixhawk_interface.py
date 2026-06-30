import rclpy
from rclpy.node import Node

from px4_msgs.msg import VehicleStatus, VehicleCommand, OffboardControlMode

from tardigrade_interfaces.srv import SetArmed, SetExternalControl
from tardigrade_interfaces.msg import RobotStatus

class PixhawkInterface(Node):
    def __init__(self):
        super().__init__('pixhawk_interface')

        self.last_vehicle_status = None
        self.external_control_enabled = False

        # Subscribers
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            'fmu/out/vehicle_status',
            self.vehicle_status_callback,
            10,
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
        self.get_logger().info("Publishing: /fmu/in/offboard_control_mode")
        self.get_logger().info("Publishing: /fmu/in/vehicle_command")

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.last_vehicle_status = msg

        self.publish_robot_status()
        
        self.get_logger().info(
            f"PX4 status | arming_state={msg.arming_state}, nav_state={msg.nav_state}",
            throttle_duration_sec=1.0,
        )

    def publish_offboard_heartbeat(self):
        msg = OffboardControlMode()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000

        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_control_mode_pub.publish(msg)

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
            msg.detail = 'PX4 status received'
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
