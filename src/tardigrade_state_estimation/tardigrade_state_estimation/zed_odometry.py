import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


def covariance_6d(position_variance, orientation_variance):
    return [
        position_variance, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, position_variance, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, position_variance, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, orientation_variance, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, orientation_variance, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, orientation_variance,
    ]


def normalize_quaternion(q):
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm <= 0.0:
        q.w = 1.0
        q.x = 0.0
        q.y = 0.0
        q.z = 0.0
        return q

    q.x /= norm
    q.y /= norm
    q.z /= norm
    q.w /= norm
    return q


class ZedOdometry(Node):
    def __init__(self):
        super().__init__('zed_odometry')

        self.declare_parameter('pose_topic', '/zed/zed_node/pose')
        self.declare_parameter('odom_topic', '/tardigrade/state/odometry')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('position_variance', 0.05)
        self.declare_parameter('orientation_variance', 0.05)
        self.declare_parameter('use_zed_frame_id', False)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.position_variance = float(self.get_parameter('position_variance').value)
        self.orientation_variance = float(self.get_parameter('orientation_variance').value)
        self.use_zed_frame_id = bool(self.get_parameter('use_zed_frame_id').value)

        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            10,
        )
        self.odom_pub = self.create_publisher(
            Odometry,
            self.odom_topic,
            10,
        )

        self.pose_covariance = covariance_6d(
            self.position_variance,
            self.orientation_variance,
        )
        self.twist_covariance = covariance_6d(
            self.position_variance,
            self.orientation_variance,
        )

        self.get_logger().info(f'Subscribing: {self.pose_topic}')
        self.get_logger().info(f'Publishing: {self.odom_topic}')

    def pose_callback(self, msg):
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = msg.header.frame_id if self.use_zed_frame_id else self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position = msg.pose.position
        odom.pose.pose.orientation = normalize_quaternion(msg.pose.orientation)
        odom.pose.covariance = self.pose_covariance
        odom.twist.covariance = self.twist_covariance

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = ZedOdometry()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
