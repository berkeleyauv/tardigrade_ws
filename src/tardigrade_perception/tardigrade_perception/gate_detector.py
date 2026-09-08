"""Classical gate detector operating only on camera images."""

import math

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from tardigrade_interfaces.msg import GateDetection


def find_gate_posts(bgr_image, minimum_area_fraction=0.0005):
    """Return the two strongest tall orange/red post rectangles or None."""
    hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    orange = cv2.inRange(hsv, np.array([0, 75, 55]), np.array([35, 255, 255]))
    red = cv2.inRange(hsv, np.array([165, 75, 55]), np.array([179, 255, 255]))
    mask = cv2.bitwise_or(orange, red)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 9))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]
    minimum_area = bgr_image.shape[0] * bgr_image.shape[1] * \
        minimum_area_fraction
    candidates = []
    for contour in contours:
        x, y, width, height = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        if area >= minimum_area and height >= width * 1.6:
            candidates.append((area * height / max(width, 1),
                               (x, y, width, height)))
    candidates.sort(reverse=True)
    for first_index, (_, first) in enumerate(candidates):
        for _, second in candidates[first_index + 1:]:
            first_center = first[0] + first[2] * 0.5
            second_center = second[0] + second[2] * 0.5
            separation = abs(first_center - second_center)
            if separation >= 0.08 * bgr_image.shape[1]:
                return tuple(sorted((first, second), key=lambda box: box[0]))
    return None


class GateDetector(Node):
    def __init__(self):
        super().__init__('gate_detector')
        self.declare_parameter(
            'image_topic',
            '/tardigrade/sensors/camera/front/left/image_raw')
        self.declare_parameter(
            'camera_info_topic',
            '/tardigrade/sensors/camera/front/left/camera_info')
        self.declare_parameter(
            'output_topic', '/tardigrade/perception/gate')
        self.declare_parameter('gate_post_height_m', 1.6)
        self.declare_parameter('minimum_area_fraction', 0.0005)
        self.bridge = CvBridge()
        self.camera_info = None
        self.post_height = float(
            self.get_parameter('gate_post_height_m').value)
        self.minimum_area = float(
            self.get_parameter('minimum_area_fraction').value)
        self.info_subscriber = self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self.on_camera_info,
            10,
        )
        self.image_subscriber = self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self.on_image,
            10,
        )
        self.publisher = self.create_publisher(
            GateDetection,
            str(self.get_parameter('output_topic').value),
            10,
        )

    def on_camera_info(self, message):
        self.camera_info = message

    def on_image(self, message):
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        boxes = find_gate_posts(image, self.minimum_area)
        output = GateDetection()
        output.stamp = message.header.stamp
        if boxes is None or self.camera_info is None:
            self.publisher.publish(output)
            return
        left, right = boxes
        left_center = left[0] + left[2] * 0.5
        right_center = right[0] + right[2] * 0.5
        center_x_pixels = (left_center + right_center) * 0.5
        center_y_pixels = (
            left[1] + left[3] * 0.5 + right[1] + right[3] * 0.5) * 0.5
        width = float(image.shape[1])
        height = float(image.shape[0])
        fx = float(self.camera_info.k[0])
        fy = float(self.camera_info.k[4])
        cx = float(self.camera_info.k[2])
        cy = float(self.camera_info.k[5])
        pixel_height = 0.5 * (left[3] + right[3])
        output.visible = True
        output.confidence = float(min(1.0, 0.6 + pixel_height / height))
        output.center_x = float(center_x_pixels / width)
        output.center_y = float(center_y_pixels / height)
        output.yaw_error_rad = float(-math.atan2(center_x_pixels - cx, fx))
        output.pitch_error_rad = float(-math.atan2(center_y_pixels - cy, fy))
        output.distance_m = float(
            self.post_height * fy / max(pixel_height, 1.0))
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = GateDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
