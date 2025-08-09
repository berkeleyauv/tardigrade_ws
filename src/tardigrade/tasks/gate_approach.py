#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import cv2 as cv

from perception.perception.tasks.gate.GateCenterAlgo import GateCenterAlgo


class GateApproach(Node):
    def __init__(self, **kwargs):
        super().__init__('gate_approach', **kwargs)
        self.bridge = CvBridge()
        self.gate_algo = GateCenterAlgo()

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.yaw_gain = 0.002
        self.strafe_gain = 0.002
        self.forward_speed = 0.2

    def camera_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            print("ros image to CV conversion successful")
        except CvBridgeError as e:
            print(e)

        h, w, _ = frame.shape

        center_x, center_y = self.gate_algo.analyze(
            frame, debug=False, slider_vals={'optical_flow_c': 10}
        )

        err_x = center_x - (w // 2)
        err_y = center_y - (h // 2)

        twist = Twist()
        twist.linear.x = self.forward_speed
        twist.linear.y = -err_x * self.strafe_gain
        twist.angular.z = -err_y * self.yaw_gain
        self.cmd_pub.publish(twist)

        self.get_logger().info(f"Gate center: ({center_x}, {center_y}) | Error: ({err_x}, {err_y})")


def main(args=None):
    rclpy.init(args=args)
    node = GateApproach()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()