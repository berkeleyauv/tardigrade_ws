"""Publish a single ROS ENU/FLU IMU stream from native VectorNav data."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from .frame_conversions import native_orientation_to_enu_base
from .frame_conversions import native_vector_to_base
from .frame_conversions import normalize_quaternion


class VectornavImuTransform(Node):
    """Apply navigation-convention and physical-mount rotations exactly once."""

    def __init__(self):
        super().__init__('vectornav_imu_transform')
        self.declare_parameter('input_topic', '/vectornav/imu')
        self.declare_parameter('output_topic', '/tardigrade/sensors/imu')
        self.declare_parameter('output_frame', 'base_link')
        self.declare_parameter('base_from_sensor_qx', 0.0)
        self.declare_parameter('base_from_sensor_qy', 1.0)
        self.declare_parameter('base_from_sensor_qz', 0.0)
        self.declare_parameter('base_from_sensor_qw', 0.0)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.output_frame = self.get_parameter('output_frame').value
        self.q_base_from_sensor = normalize_quaternion((
            float(self.get_parameter('base_from_sensor_qw').value),
            float(self.get_parameter('base_from_sensor_qx').value),
            float(self.get_parameter('base_from_sensor_qy').value),
            float(self.get_parameter('base_from_sensor_qz').value),
        ))

        self.publisher = self.create_publisher(Imu, self.output_topic, 10)
        self.subscription = self.create_subscription(
            Imu, self.input_topic, self.imu_callback, 10
        )
        self.get_logger().info(
            f'Transforming native VectorNav {self.input_topic} into '
            f'{self.output_topic} ({self.output_frame})'
        )

    @staticmethod
    def _set_vector(target, values):
        target.x, target.y, target.z = values

    def imu_callback(self, source):
        msg = Imu()
        msg.header.stamp = source.header.stamp
        msg.header.frame_id = self.output_frame

        q_ned_from_sensor = (
            source.orientation.w,
            source.orientation.x,
            source.orientation.y,
            source.orientation.z,
        )
        try:
            q_enu_from_base = native_orientation_to_enu_base(
                q_ned_from_sensor, self.q_base_from_sensor
            )
        except ValueError:
            self.get_logger().error('Dropping IMU message with invalid orientation')
            return

        msg.orientation.w, msg.orientation.x, msg.orientation.y, \
            msg.orientation.z = q_enu_from_base
        self._set_vector(
            msg.angular_velocity,
            native_vector_to_base(
                (source.angular_velocity.x,
                 source.angular_velocity.y,
                 source.angular_velocity.z),
                self.q_base_from_sensor,
            ),
        )
        self._set_vector(
            msg.linear_acceleration,
            native_vector_to_base(
                (source.linear_acceleration.x,
                 source.linear_acceleration.y,
                 source.linear_acceleration.z),
                self.q_base_from_sensor,
            ),
        )
        msg.orientation_covariance = source.orientation_covariance
        msg.angular_velocity_covariance = source.angular_velocity_covariance
        msg.linear_acceleration_covariance = source.linear_acceleration_covariance

        values = (
            *q_enu_from_base,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        )
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error('Dropping IMU message with non-finite data')
            return
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VectornavImuTransform()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
