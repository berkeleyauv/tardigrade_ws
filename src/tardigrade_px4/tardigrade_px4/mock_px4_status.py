import rclpy
from rclpy.node import Node

from px4_msgs.msg import VehicleStatus


class MockPx4Status(Node):
    def __init__(self):
        super().__init__("mock_px4_status")

        self.publisher = self.create_publisher(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            10,
        )

        self.timer = self.create_timer(0.5, self.publish_status)
        self.get_logger().info("Mock PX4 status publisher started.")

    def publish_status(self):
        msg = VehicleStatus()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000

        msg.arming_state = VehicleStatus.ARMING_STATE_STANDBY
        msg.nav_state = VehicleStatus.NAVIGATION_STATE_MANUAL

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockPx4Status()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        