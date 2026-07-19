import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from pymavlink import mavutil

from tardigrade_px4.mavlink_common import (
    connect_mavlink,
    enu_to_ned_point,
    flu_to_frd_vector,
    now_us,
    quaternion_enu_flu_to_ned_frd,
)


UNKNOWN_COVARIANCE = [math.nan] + [0.0] * 20


class MavlinkOdometryToPx4(Node):
    def __init__(self):
        super().__init__('mavlink_odometry_to_px4')

        self.declare_parameter('device', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 921600)
        self.declare_parameter('odom_topic', '/tardigrade/state/odometry')
        self.declare_parameter('source_system', 42)
        self.declare_parameter('source_component', 191)

        self.device = self.get_parameter('device').value
        self.baudrate = self.get_parameter('baudrate').value
        self.odom_topic = self.get_parameter('odom_topic').value
        source_system = self.get_parameter('source_system').value
        source_component = self.get_parameter('source_component').value

        self.mav = connect_mavlink(
            self.device,
            self.baudrate,
            source_system=source_system,
            source_component=source_component,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )

        self.get_logger().info(f'Opening MAVLink serial: {self.device} @ {self.baudrate}')
        self.get_logger().info(f'Subscribing: {self.odom_topic}')
        self.get_logger().info('Sending MAVLink ODOMETRY to Pixhawk')

    def odom_callback(self, msg):
        position = enu_to_ned_point(msg.pose.pose.position)
        q = quaternion_enu_flu_to_ned_frd(msg.pose.pose.orientation)
        velocity = enu_to_ned_point(msg.twist.twist.linear)
        angular_velocity = flu_to_frd_vector(msg.twist.twist.angular)

        self.mav.mav.odometry_send(
            now_us(),
            mavutil.mavlink.MAV_FRAME_LOCAL_FRD,
            mavutil.mavlink.MAV_FRAME_BODY_FRD,
            position[0],
            position[1],
            position[2],
            q,
            velocity[0],
            velocity[1],
            velocity[2],
            angular_velocity[0],
            angular_velocity[1],
            angular_velocity[2],
            UNKNOWN_COVARIANCE,
            UNKNOWN_COVARIANCE,
            0,
            mavutil.mavlink.MAV_ESTIMATOR_TYPE_VISION,
        )


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkOdometryToPx4()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
