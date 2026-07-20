# Copyright 2026 Berkeley AUV

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Quaternion, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def covariance_6d(position_variance, orientation_variance):
    # nav_msgs/Odometry covariances are 6x6 matrices flattened row-major:
    # x, y, z, roll, pitch, yaw. Tune these variances to tell downstream
    # consumers how much confidence to place in ZED position and VectorNav
    # attitude.
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


def quat_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b

    return [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]


def vector_frd_to_flu(v):
    # VectorNav body vectors are FRD. ROS body vectors are FLU.
    # Forward stays forward; right/down flip sign into left/up.
    out = Vector3()
    out.x = v.x
    out.y = -v.y
    out.z = -v.z
    return out


def quaternion_ned_frd_to_enu_flu(q):
    # VectorNav orientation is treated as NED world + FRD body. ROS odometry
    # should be ENU world + FLU body, so we wrap the sensor quaternion with
    # fixed frame-conversion rotations.
    vn_q = [q.w, q.x, q.y, q.z]

    q_enu_from_ned = [
        0.0,
        math.sqrt(0.5),
        math.sqrt(0.5),
        0.0,
    ]

    q_frd_from_flu = [
        0.0,
        1.0,
        0.0,
        0.0,
    ]

    ros_q = quat_multiply(
        quat_multiply(q_enu_from_ned, vn_q),
        q_frd_from_flu,
    )

    out = Quaternion()
    out.w = ros_q[0]
    out.x = ros_q[1]
    out.y = ros_q[2]
    out.z = ros_q[3]
    return normalize_quaternion(out)


class ZedVectornavOdometry(Node):
    def __init__(self):
        super().__init__('zed_vectornav_odometry')

        # Launch-time knobs. Change topic/frame names here only if upstream
        # drivers or downstream bridge topics change.
        self.declare_parameter('zed_pose_topic', '/zed/zed_node/pose')
        self.declare_parameter('imu_topic', '/vectornav/imu')
        self.declare_parameter('odom_topic', '/tardigrade/state/odometry')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('position_variance', 0.05)
        self.declare_parameter('orientation_variance', 0.02)
        self.declare_parameter('angular_velocity_variance', 0.02)
        self.declare_parameter('linear_velocity_variance', 0.10)
        self.declare_parameter('velocity_filter_alpha', 0.25)
        self.declare_parameter('imu_timeout_sec', 0.25)
        self.declare_parameter('use_zed_orientation_if_imu_stale', True)
        self.declare_parameter('use_zed_frame_id', False)
        self.declare_parameter('zero_initial_position', True)

        self.zed_pose_topic = self.get_parameter('zed_pose_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.position_variance = float(self.get_parameter('position_variance').value)
        self.orientation_variance = float(self.get_parameter('orientation_variance').value)
        angular_velocity_variance = float(self.get_parameter('angular_velocity_variance').value)
        linear_velocity_variance = float(
            self.get_parameter('linear_velocity_variance').value
        )
        self.velocity_filter_alpha = max(0.0, min(
            1.0,
            float(self.get_parameter('velocity_filter_alpha').value),
        ))
        self.imu_timeout_sec = float(self.get_parameter('imu_timeout_sec').value)
        self.use_zed_orientation_if_imu_stale = bool(
            self.get_parameter('use_zed_orientation_if_imu_stale').value
        )
        self.use_zed_frame_id = bool(self.get_parameter('use_zed_frame_id').value)
        self.zero_initial_position = bool(self.get_parameter('zero_initial_position').value)

        self.latest_imu = None
        self.latest_imu_received_ns = None
        self.initial_position = None
        self.previous_position = None
        self.previous_pose_received_ns = None
        self.filtered_velocity = [0.0, 0.0, 0.0]

        # ZED provides position and filtered finite-difference velocity.
        # VectorNav provides orientation and angular rates.
        self.pose_covariance = covariance_6d(
            self.position_variance,
            self.orientation_variance,
        )
        self.twist_covariance = covariance_6d(
            linear_velocity_variance,
            angular_velocity_variance,
        )

        # ZED pose callbacks drive publication. The IMU callback only caches the
        # latest VectorNav sample so each ZED pose can be paired with fresh IMU.
        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.zed_pose_topic,
            self.pose_callback,
            10,
        )
        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            10,
        )
        self.odom_pub = self.create_publisher(
            Odometry,
            self.odom_topic,
            10,
        )

        self.get_logger().info(f'Subscribing ZED pose: {self.zed_pose_topic}')
        self.get_logger().info(f'Subscribing VectorNav IMU: {self.imu_topic}')
        self.get_logger().info(f'Publishing fused odometry: {self.odom_topic}')
        if self.zero_initial_position:
            self.get_logger().info('Zeroing odometry position to first ZED pose')

    def imu_callback(self, msg):
        self.latest_imu = msg
        self.latest_imu_received_ns = self.get_clock().now().nanoseconds

    def latest_imu_is_fresh(self):
        # If VectorNav stops publishing, fall back instead of mixing stale
        # attitude with current ZED position.
        if self.latest_imu is None or self.latest_imu_received_ns is None:
            return False

        age_sec = (
            self.get_clock().now().nanoseconds - self.latest_imu_received_ns
        ) / 1_000_000_000.0
        return age_sec <= self.imu_timeout_sec

    def pose_callback(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = msg.header.frame_id if self.use_zed_frame_id else self.odom_frame
        odom.child_frame_id = self.base_frame

        if self.zero_initial_position:
            if self.initial_position is None:
                self.initial_position = (
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z,
                )
            odom.pose.pose.position.x = msg.pose.position.x - self.initial_position[0]
            odom.pose.pose.position.y = msg.pose.position.y - self.initial_position[1]
            odom.pose.pose.position.z = msg.pose.position.z - self.initial_position[2]
        else:
            odom.pose.pose.position = msg.pose.position

        position = odom.pose.pose.position
        if self.previous_position is not None and self.previous_pose_received_ns is not None:
            dt = (now_ns - self.previous_pose_received_ns) / 1_000_000_000.0
            if 0.001 <= dt <= 0.5:
                raw_velocity = [
                    (position.x - self.previous_position[0]) / dt,
                    (position.y - self.previous_position[1]) / dt,
                    (position.z - self.previous_position[2]) / dt,
                ]
                alpha = self.velocity_filter_alpha
                self.filtered_velocity = [
                    alpha * raw + (1.0 - alpha) * old
                    for raw, old in zip(raw_velocity, self.filtered_velocity)
                ]

        odom.twist.twist.linear.x = self.filtered_velocity[0]
        odom.twist.twist.linear.y = self.filtered_velocity[1]
        odom.twist.twist.linear.z = self.filtered_velocity[2]
        self.previous_position = (position.x, position.y, position.z)
        self.previous_pose_received_ns = now_ns
        odom.twist.covariance = self.twist_covariance

        # Preferred path: ZED position + VectorNav orientation/angular velocity.
        # Fallback path keeps odometry alive using ZED orientation if allowed.
        if self.latest_imu_is_fresh():
            odom.pose.pose.orientation = quaternion_ned_frd_to_enu_flu(
                self.latest_imu.orientation,
            )
            odom.twist.twist.angular = vector_frd_to_flu(
                self.latest_imu.angular_velocity,
            )
        elif self.use_zed_orientation_if_imu_stale:
            odom.pose.pose.orientation = normalize_quaternion(msg.pose.orientation)
        else:
            return

        odom.pose.covariance = self.pose_covariance
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = ZedVectornavOdometry()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
