import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class MockCameras(Node):
    def __init__(self):
        super().__init__('mock_cameras')

        self.declare_parameter('width', 320)
        self.declare_parameter('height', 180)
        self.declare_parameter('rate_hz', 5.0)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        rate_hz = float(self.get_parameter('rate_hz').value)
        self.frame_index = 0

        self.left_pub = self.create_publisher(
            Image,
            '/zed/zed_node/left/image_rect_color',
            2,
        )
        self.right_pub = self.create_publisher(
            Image,
            '/zed/zed_node/right/image_rect_color',
            2,
        )
        self.depth_pub = self.create_publisher(
            Image,
            '/zed/zed_node/depth/depth_registered',
            2,
        )
        self.debug_pub = self.create_publisher(
            Image,
            '/tardigrade/perception/debug_image',
            2,
        )
        self.downward_pub = self.create_publisher(
            Image,
            '/tardigrade/camera/downward/image_raw',
            2,
        )
        self.left_info_pub = self.create_publisher(
            CameraInfo,
            '/zed/zed_node/left/camera_info',
            2,
        )
        self.right_info_pub = self.create_publisher(
            CameraInfo,
            '/zed/zed_node/right/camera_info',
            2,
        )

        self.timer = self.create_timer(1.0 / max(rate_hz, 1.0), self.tick)
        self.get_logger().info(
            f'Publishing mock camera images at {self.width}x{self.height}'
        )

    def make_rgb_image(self, frame_id, stereo_offset=0, debug=False):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.height = self.height
        msg.width = self.width
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = self.width * 3

        phase = self.frame_index * 3 + stereo_offset
        gate_center = int(self.width * 0.5 + 35 * math.sin(phase * 0.03))
        gate_left = gate_center - 42
        gate_right = gate_center + 42
        gate_top = int(self.height * 0.25)
        gate_bottom = int(self.height * 0.78)

        pixels = bytearray()
        for y in range(self.height):
            for x in range(self.width):
                water = int(55 + 25 * math.sin((x + phase) * 0.025))
                red = 18
                green = clamp_byte(water + y // 7)
                blue = clamp_byte(120 + x // 8)
                on_gate = (
                    (abs(x - gate_left) <= 3 or abs(x - gate_right) <= 3)
                    and gate_top <= y <= gate_bottom
                ) or (
                    (abs(y - gate_top) <= 3 or abs(y - gate_bottom) <= 3)
                    and gate_left <= x <= gate_right
                )
                if on_gate:
                    red, green, blue = (255, 180, 20)
                if debug and (
                    abs(x - gate_center) <= 1 or abs(y - self.height // 2) <= 1
                ):
                    red, green, blue = (40, 255, 90)
                pixels.extend((red, green, blue))
        msg.data = bytes(pixels)
        return msg

    def make_downward_image(self):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'downward_camera_link'
        msg.height = self.height
        msg.width = self.width
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = self.width * 3

        pixels = bytearray()
        phase = self.frame_index * 2
        for y in range(self.height):
            for x in range(self.width):
                checker = ((x + phase) // 30 + y // 30) % 2
                base = 55 if checker else 75
                pixels.extend((base, base + 20, base + 35))
        msg.data = bytes(pixels)
        return msg

    def make_depth_image(self):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'zed_camera_link'
        msg.height = self.height
        msg.width = self.width
        msg.encoding = '16UC1'
        msg.is_bigendian = False
        msg.step = self.width * 2

        pixels = bytearray()
        phase = self.frame_index * 0.08
        for y in range(self.height):
            for x in range(self.width):
                distance_mm = int(
                    1800
                    + 700 * x / max(self.width - 1, 1)
                    + 180 * math.sin(y * 0.05 + phase)
                )
                pixels.extend(distance_mm.to_bytes(2, 'little'))
        msg.data = bytes(pixels)
        return msg

    def make_camera_info(self, frame_id):
        msg = CameraInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.height = self.height
        msg.width = self.width
        focal = float(self.width) * 0.75
        center_x = float(self.width) * 0.5
        center_y = float(self.height) * 0.5
        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0] * 5
        msg.k = [focal, 0.0, center_x, 0.0, focal, center_y, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [
            focal, 0.0, center_x, 0.0,
            0.0, focal, center_y, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return msg

    def tick(self):
        self.left_pub.publish(self.make_rgb_image('zed_camera_link'))
        self.right_pub.publish(
            self.make_rgb_image('zed_camera_link', stereo_offset=8)
        )
        self.depth_pub.publish(self.make_depth_image())
        self.debug_pub.publish(
            self.make_rgb_image('zed_camera_link', debug=True)
        )
        self.downward_pub.publish(self.make_downward_image())
        self.left_info_pub.publish(self.make_camera_info('zed_camera_link'))
        self.right_info_pub.publish(self.make_camera_info('zed_camera_link'))
        self.frame_index += 1


def clamp_byte(value):
    return max(0, min(255, int(value)))


def main(args=None):
    rclpy.init(args=args)
    node = MockCameras()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
