import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from tardigrade_interfaces.msg import RobotStatus
from tardigrade_interfaces.srv import SetArmed, SetExternalControl


def clamp(value, low, high):
    return max(low, min(high, value))


def angle_wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def yaw_from_quaternion(q):
    # Standard yaw (rotation about z) from a quaternion.
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PathFollower(Node):
    """Drives the robot through a list of waypoints by publishing cmd_vel.

    This is a backend-agnostic autonomy node: it subscribes to
    /tardigrade/state/odometry and publishes /tardigrade/cmd_vel, exactly like
    gate_mission. Run it against fake_unity_backend today, or against the Unity
    RosRobotBridge later, with no changes.

    cmd_vel here follows the same convention as fake_unity_backend: body-frame
    normalized commands in [-1, 1] (linear.x forward, linear.z up, angular.z yaw
    rate), NOT physical m/s. Tune the *_cmd parameters if you change that.
    """

    def __init__(self):
        super().__init__('path_follower')

        self.declare_parameter('cmd_vel_topic', '/tardigrade/cmd_vel')
        self.declare_parameter('odometry_topic', '/tardigrade/state/odometry')
        self.declare_parameter('status_topic', '/tardigrade/status')

        # Flat list of waypoints [x0, y0, x1, y1, ...] in the odom frame.
        # Default: a 3 m square starting ahead of the robot.
        self.declare_parameter(
            'waypoints',
            [3.0, 0.0, 3.0, 3.0, 0.0, 3.0, 0.0, 0.0],
        )
        # Optional depth hold; leave 0.0 to ignore the z axis.
        self.declare_parameter('target_depth_m', 0.0)

        self.declare_parameter('position_tolerance_m', 0.25)
        self.declare_parameter('depth_tolerance_m', 0.15)
        # Face the waypoint before driving if heading error exceeds this.
        self.declare_parameter('heading_align_threshold_rad', 0.4)

        self.declare_parameter('forward_cmd', 0.6)
        self.declare_parameter('yaw_kp', 1.2)
        self.declare_parameter('max_yaw_cmd', 0.8)
        self.declare_parameter('depth_kp', 1.0)
        self.declare_parameter('max_depth_cmd', 0.6)

        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('timeout_per_waypoint_sec', 30.0)
        self.declare_parameter('disarm_when_done', True)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odometry_topic = self.get_parameter('odometry_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.target_depth_m = float(self.get_parameter('target_depth_m').value)
        self.position_tolerance_m = float(
            self.get_parameter('position_tolerance_m').value
        )
        self.depth_tolerance_m = float(self.get_parameter('depth_tolerance_m').value)
        self.heading_align_threshold_rad = float(
            self.get_parameter('heading_align_threshold_rad').value
        )
        self.forward_cmd = float(self.get_parameter('forward_cmd').value)
        self.yaw_kp = float(self.get_parameter('yaw_kp').value)
        self.max_yaw_cmd = float(self.get_parameter('max_yaw_cmd').value)
        self.depth_kp = float(self.get_parameter('depth_kp').value)
        self.max_depth_cmd = float(self.get_parameter('max_depth_cmd').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.timeout_per_waypoint_sec = float(
            self.get_parameter('timeout_per_waypoint_sec').value
        )
        self.disarm_when_done = bool(self.get_parameter('disarm_when_done').value)

        self.waypoints = self.parse_waypoints(self.get_parameter('waypoints').value)

        self.latest_odom = None
        self.latest_status = None

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.odometry_topic, self.odom_callback, 10
        )
        self.status_sub = self.create_subscription(
            RobotStatus, self.status_topic, self.status_callback, 10
        )
        self.arm_client = self.create_client(SetArmed, '/tardigrade/set_armed')
        self.external_client = self.create_client(
            SetExternalControl, '/tardigrade/set_external_control'
        )

    def parse_waypoints(self, flat):
        if len(flat) % 2 != 0:
            raise ValueError('waypoints must be a flat list of x,y pairs')
        return [(float(flat[i]), float(flat[i + 1])) for i in range(0, len(flat), 2)]

    def odom_callback(self, msg):
        self.latest_odom = msg

    def status_callback(self, msg):
        self.latest_status = msg

    # ------------------------------------------------------------------ run

    def run(self):
        self.wait_for_odometry()
        self.set_external_control(True)
        self.set_armed(True)
        try:
            for index, (wx, wy) in enumerate(self.waypoints):
                self.get_logger().info(
                    f'Waypoint {index + 1}/{len(self.waypoints)}: '
                    f'({wx:.2f}, {wy:.2f})'
                )
                self.go_to_waypoint(wx, wy)
            self.get_logger().info('Path complete.')
        finally:
            self.publish_zero()
            if self.disarm_when_done:
                self.set_armed(False)
                self.set_external_control(False)

    def go_to_waypoint(self, wx, wy):
        period = 1.0 / max(self.control_rate_hz, 1.0)
        start = time.monotonic()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            x, y, z, yaw = self.current_pose()

            dx = wx - x
            dy = wy - y
            distance = math.hypot(dx, dy)
            depth_error = self.target_depth_m - z
            depth_ok = (
                self.target_depth_m == 0.0
                or abs(depth_error) <= self.depth_tolerance_m
            )

            if distance <= self.position_tolerance_m and depth_ok:
                self.publish_zero()
                self.get_logger().info('  reached.')
                return

            if time.monotonic() - start > self.timeout_per_waypoint_sec:
                raise RuntimeError('Timed out driving to waypoint')

            bearing = math.atan2(dy, dx)
            yaw_error = angle_wrap(bearing - yaw)

            cmd = Twist()
            cmd.angular.z = clamp(
                self.yaw_kp * yaw_error, -self.max_yaw_cmd, self.max_yaw_cmd
            )
            # Only drive forward once roughly pointed at the target, so we turn
            # in place instead of arcing wide.
            if abs(yaw_error) < self.heading_align_threshold_rad:
                cmd.linear.x = self.forward_cmd
            if self.target_depth_m != 0.0:
                cmd.linear.z = clamp(
                    self.depth_kp * depth_error,
                    -self.max_depth_cmd,
                    self.max_depth_cmd,
                )
            self.cmd_pub.publish(cmd)
            time.sleep(period)

    def current_pose(self):
        p = self.latest_odom.pose.pose.position
        yaw = yaw_from_quaternion(self.latest_odom.pose.pose.orientation)
        return p.x, p.y, p.z, yaw

    # -------------------------------------------------------------- helpers

    def wait_for_odometry(self):
        self.get_logger().info(f'Waiting for {self.odometry_topic}')
        start = time.monotonic()
        while rclpy.ok() and self.latest_odom is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - start > self.timeout_per_waypoint_sec:
                raise RuntimeError('Timed out waiting for odometry')

    def publish_zero(self):
        self.cmd_pub.publish(Twist())

    def set_external_control(self, enabled):
        if not self.external_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('Service /tardigrade/set_external_control unavailable')
        request = SetExternalControl.Request()
        request.enabled = enabled
        response = self.call_service(self.external_client, request)
        if not response.success:
            raise RuntimeError(response.message)
        self.get_logger().info(response.message)

    def set_armed(self, armed):
        if not self.arm_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('Service /tardigrade/set_armed unavailable')
        request = SetArmed.Request()
        request.armed = armed
        response = self.call_service(self.arm_client, request)
        if not response.success:
            raise RuntimeError(response.message)
        self.get_logger().info(response.message)

    def call_service(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done():
            raise RuntimeError('Timed out waiting for service response')
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        node.run()
    except Exception as exc:
        node.get_logger().error(str(exc))
        node.publish_zero()
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()
