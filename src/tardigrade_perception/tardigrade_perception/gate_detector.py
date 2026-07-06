import cv2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from tardigrade_interfaces.msg import GateDetection

from tardigrade_perception import gate_pipeline


class GateDetector(Node):
    """Subscribes to camera frames, publishes GateDetection.

    Runs the classical (LAB color) gate pipeline from UR-B-Perception.
    Throttles processing to `max_rate_hz` so the Jetson isn't saturated.
    """

    def __init__(self):
        super().__init__('gate_detector')

        self.declare_parameter('image_topic', '/zed/zed_node/rgb/image_rect_color')
        self.declare_parameter('detection_topic', '/tardigrade/perception/gate')
        self.declare_parameter('debug_image_topic', '/tardigrade/perception/gate_debug')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('enhance', True)
        self.declare_parameter('black_percentile', gate_pipeline.BLACK_PERCENTILE)
        self.declare_parameter('a_threshold', gate_pipeline.A_THRESHOLD)
        self.declare_parameter('max_rate_hz', 10.0)
        self.declare_parameter('resize_width', 640)

        image_topic = self.get_parameter('image_topic').value
        detection_topic = self.get_parameter('detection_topic').value
        debug_topic = self.get_parameter('debug_image_topic').value
        self.publish_debug = bool(self.get_parameter('publish_debug_image').value)
        self.enhance = bool(self.get_parameter('enhance').value)
        self.percentile = float(self.get_parameter('black_percentile').value)
        self.a_threshold = int(self.get_parameter('a_threshold').value)
        self.min_period = 1.0 / max(float(self.get_parameter('max_rate_hz').value), 0.1)
        self.resize_width = int(self.get_parameter('resize_width').value)

        self.bridge = CvBridge()
        self.last_proc_time = 0.0

        self.detection_pub = self.create_publisher(GateDetection, detection_topic, 10)
        self.debug_pub = self.create_publisher(Image, debug_topic, 1)
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 1)

        self.get_logger().info('Subscribing: {}'.format(image_topic))
        self.get_logger().info('Publishing: {}'.format(detection_topic))

    def image_callback(self, msg):
        # Throttle: drop frames arriving faster than max_rate_hz
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_proc_time < self.min_period:
            return
        self.last_proc_time = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error('cv_bridge failed: {}'.format(e))
            return

        # Downscale for speed; all outputs are normalized so scale is safe
        if self.resize_width > 0 and frame.shape[1] > self.resize_width:
            scale = self.resize_width / frame.shape[1]
            frame = cv2.resize(frame, None, fx=scale, fy=scale)

        result = gate_pipeline.analyze_frame(
            frame, enhance=self.enhance,
            percentile=self.percentile, a_threshold=self.a_threshold)

        out = GateDetection()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id

        if result is None:
            out.detected = False
            out.both_posts = False
            out.yaw_signal = 0.0
            out.lateral = 0.0
            out.gate_width_frac = 0.0
        else:
            out.detected = True
            out.both_posts = result['both_posts']
            out.yaw_signal = float(result['yaw_signal']) if result['yaw_signal'] is not None else 0.0
            out.lateral = float(result['lateral_norm'])
            out.gate_width_frac = float(result['gate_width_frac'])
            gx, gy, gw, gh = result['roi']
            out.box_x, out.box_y, out.box_w, out.box_h = int(gx), int(gy), int(gw), int(gh)

        self.detection_pub.publish(out)

        if self.publish_debug and self.debug_pub.get_subscription_count() > 0:
            debug = gate_pipeline.draw_debug(result, frame)
            debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GateDetector()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
