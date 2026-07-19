import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry


def quat_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b

    return [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]


def enu_to_ned_point(p):
    return [p.y, p.x, -p.z]


def flu_to_frd_vector(v):
    return [v.x, -v.y, -v.z]


def quaternion_enu_flu_to_ned_frd(q):
    ros_q = [q.w, q.x, q.y, q.z]

    q_ned_from_enu = [
        0.0,
        math.sqrt(0.5),
        math.sqrt(0.5),
        0.0,
    ]

    q_flu_from_frd = [
        0.0,
        1.0,
        0.0,
        0.0,
    ]

    return quat_multiply(
        quat_multiply(q_ned_from_enu, ros_q),
        q_flu_from_frd,
    )


class OdometryToPx4(Node):
    def __init__(self):
        super().__init__('odometry_to_px4')

        self.declare_parameter('odom_topic', '/tardigrade/state/odometry')
        self.declare_parameter('px4_odom_topic', '/fmu/in/vehicle_visual_odometry')

        odom_topic = self.get_parameter('odom_topic').value
        px4_odom_topic = self.get_parameter('px4_odom_topic').value

        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10,
        )

        self.px4_odom_pub = self.create_publisher(
            VehicleOdometry,
            px4_odom_topic,
            10,
        )

        self.get_logger().info(f'Subscribing: {odom_topic}')
        self.get_logger().info(f'Publishing: {px4_odom_topic}')

    def odom_callback(self, msg):
        px4_msg = VehicleOdometry()

        now_us = self.get_clock().now().nanoseconds // 1000
        px4_msg.timestamp = now_us
        px4_msg.timestamp_sample = now_us

        px4_msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        px4_msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED

        px4_msg.position = enu_to_ned_point(msg.pose.pose.position)
        px4_msg.q = quaternion_enu_flu_to_ned_frd(msg.pose.pose.orientation)

        px4_msg.velocity = enu_to_ned_point(msg.twist.twist.linear)
        px4_msg.angular_velocity = flu_to_frd_vector(msg.twist.twist.angular)

        px4_msg.position_variance = [999.0, 999.0, 999.0]
        px4_msg.orientation_variance = [0.1, 0.1, 0.1]
        px4_msg.velocity_variance = [999.0, 999.0, 999.0]

        px4_msg.reset_counter = 0
        px4_msg.quality = 100

        self.px4_odom_pub.publish(px4_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryToPx4()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()