# Copyright 2026 Berkeley AUV

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PoseStamped, Quaternion
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
    out = Quaternion()
    if norm <= 0.0:
        out.w = 1.0
        return out

    out.x = q.x / norm
    out.y = q.y / norm
    out.z = q.z / norm
    out.w = q.w / norm
    return out


def copy_point(p):
    out = Point()
    out.x = p.x
    out.y = p.y
    out.z = p.z
    return out


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
        self.declare_parameter('log_pose_every_n', 0)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.position_variance = float(self.get_parameter('position_variance').value)
        self.orientation_variance = float(self.get_parameter('orientation_variance').value)
        self.use_zed_frame_id = bool(self.get_parameter('use_zed_frame_id').value)
        self.log_pose_every_n = int(self.get_parameter('log_pose_every_n').value)
        self.pose_count = 0

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

        odom.pose.pose.position = copy_point(msg.pose.position)
        odom.pose.pose.orientation = normalize_quaternion(msg.pose.orientation)
        odom.pose.covariance = self.pose_covariance
        odom.twist.covariance = self.twist_covariance

        self.odom_pub.publish(odom)
        self.pose_count += 1

        if self.log_pose_every_n > 0 and self.pose_count % self.log_pose_every_n == 0:
            p = odom.pose.pose.position
            q = odom.pose.pose.orientation
            self.get_logger().info(
                'ZED odometry pose '
                f'x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}, '
                f'qx={q.x:.3f}, qy={q.y:.3f}, qz={q.z:.3f}, qw={q.w:.3f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ZedOdometry()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
