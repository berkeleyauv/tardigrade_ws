import rclpy
from rclpy.node import Node

from px4_msgs.msg import VehicleCommand, VehicleCommandAck, VehicleStatus


class MockPx4Status(Node):
    def __init__(self):
        super().__init__("mock_px4_status")

        self.arming_state = VehicleStatus.ARMING_STATE_STANDBY
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MANUAL

        self.status_pub = self.create_publisher(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            10,
        )

        self.command_ack_pub = self.create_publisher(
            VehicleCommandAck,
            "/fmu/out/vehicle_command_ack",
            10,
        )

        self.vehicle_command_sub = self.create_subscription(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            self.vehicle_command_callback,
            10,
        )

        self.timer = self.create_timer(0.5, self.publish_status)
        self.get_logger().info("Mock PX4 status publisher started.")

    def publish_status(self):
        msg = VehicleStatus()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000

        msg.arming_state = self.arming_state
        msg.nav_state = self.nav_state

        self.status_pub.publish(msg)

    def vehicle_command_callback(self, msg: VehicleCommand):
        if msg.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM:
            self.handle_arm_disarm(msg)
        elif msg.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE:
            self.handle_set_mode(msg)
        else:
            self.get_logger().info(f"Mock received command={msg.command}")

        self.publish_command_ack(msg)
        self.publish_status()

    def handle_arm_disarm(self, msg: VehicleCommand):
        if msg.param1 == 1.0:
            self.arming_state = VehicleStatus.ARMING_STATE_ARMED
            self.get_logger().info("Mock PX4 armed.")
        else:
            self.arming_state = VehicleStatus.ARMING_STATE_STANDBY
            self.get_logger().info("Mock PX4 disarmed.")

    def handle_set_mode(self, msg: VehicleCommand):
        if msg.param1 == 1.0 and msg.param2 == 6.0:
            self.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
            self.get_logger().info("Mock PX4 entered offboard mode.")
        else:
            self.get_logger().info(
                f"Mock received set mode param1={msg.param1}, param2={msg.param2}"
            )

    def publish_command_ack(self, command_msg: VehicleCommand):
        ack = VehicleCommandAck()
        ack.timestamp = self.get_clock().now().nanoseconds // 1000
        ack.command = command_msg.command
        ack.result = VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED
        ack.target_system = command_msg.source_system
        ack.target_component = command_msg.source_component
        ack.from_external = False

        self.command_ack_pub.publish(ack)


def main(args=None):
    rclpy.init(args=args)
    node = MockPx4Status()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
