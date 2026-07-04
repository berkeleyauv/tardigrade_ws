import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Vector3


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
    out = Vector3()
    out.x = v.x
    out.y = -v.y
    out.z = -v.z
    return out


def quaternion_ned_frd_to_enu_flu(q):
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
    return out

class VectornavOdometry(Node):
    def __init__(self):
        super().__init__('vectornav_odometry')

        self.declare_parameter('imu_topic', '/vectornav/imu')
        self.declare_parameter('odom_topic', '/tardigrade/state/odometry')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.imu_topic = self.get_parameter('imu_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.odom_pub = self.create_publisher(
            Odometry,
            self.odom_topic,
            10,
        )

        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            10,
        )

        self.get_logger().info(f'Subscribing: {self.imu_topic}')
        self.get_logger().info(f'Publishing: {self.odom_topic}')

    def imu_callback(self, msg: Imu):
        odom = Odometry()

        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.orientation = quaternion_ned_frd_to_enu_flu(msg.orientation)
        odom.pose.covariance = [
            999.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 999.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, msg.orientation_covariance[0], msg.orientation_covariance[1], msg.orientation_covariance[2],
            0.0, 0.0, 0.0, msg.orientation_covariance[3], msg.orientation_covariance[4], msg.orientation_covariance[5],
            0.0, 0.0, 0.0, msg.orientation_covariance[6], msg.orientation_covariance[7], msg.orientation_covariance[8],
        ]

        odom.twist.twist.angular = vector_frd_to_flu(msg.angular_velocity)
        odom.twist.covariance = [
            999.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 999.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, msg.angular_velocity_covariance[0], msg.angular_velocity_covariance[1], msg.angular_velocity_covariance[2],
            0.0, 0.0, 0.0, msg.angular_velocity_covariance[3], msg.angular_velocity_covariance[4], msg.angular_velocity_covariance[5],
            0.0, 0.0, 0.0, msg.angular_velocity_covariance[6], msg.angular_velocity_covariance[7], msg.angular_velocity_covariance[8],
        ]

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = VectornavOdometry()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()