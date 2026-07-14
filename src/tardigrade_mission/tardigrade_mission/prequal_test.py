import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64, String


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_delta(current, previous):
    return math.atan2(math.sin(current - previous), math.cos(current - previous))


def heading_error(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))


class PrequalTest(Node):
    """Run the prequal path through the active ESP depth-control command path."""

    def __init__(self):
        super().__init__('prequal_test')
        self.declare_parameter('cmd_vel_topic', '/tardigrade/cmd_vel/manual')
        self.declare_parameter('odometry_topic', '/tardigrade/state/odometry')
        self.declare_parameter('esp_status_topic', '/tardigrade/esp/status')
        self.declare_parameter('depth_target_topic', '/tardigrade/depth_target')
        self.declare_parameter('dry_run', True)
        self.declare_parameter('startup_delay_sec', 15.0)
        self.declare_parameter('readiness_timeout_sec', 90.0)
        self.declare_parameter('odometry_timeout_sec', 0.5)
        self.declare_parameter('esp_status_timeout_sec', 1.0)
        self.declare_parameter('phase_timeout_sec', 240.0)
        self.declare_parameter('forward_distance_m', 20.0)
        self.declare_parameter('forward_command', 0.20)
        self.declare_parameter('target_depth_m', 1.5)
        self.declare_parameter('depth_tolerance_m', 0.05)
        self.declare_parameter('yaw_turn_rad', math.pi)
        self.declare_parameter('yaw_kp', 0.4)
        self.declare_parameter('yaw_kd', 0.1)
        self.declare_parameter('max_yaw_command', 0.20)
        self.declare_parameter('turn_yaw_command', 0.20)
        self.declare_parameter('heading_tolerance_rad', math.radians(4.0))
        self.declare_parameter('return_position_tolerance_m', 0.75)
        self.declare_parameter('max_cross_track_error_m', 3.0)
        self.declare_parameter('max_depth_error_m', 0.40)
        self.declare_parameter('max_tilt_rad', math.radians(30.0))
        self.declare_parameter('settle_sec', 1.0)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odometry_topic = self.get_parameter('odometry_topic').value
        self.esp_status_topic = self.get_parameter('esp_status_topic').value
        self.depth_target_topic = self.get_parameter('depth_target_topic').value
        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.startup_delay_sec = float(self.get_parameter('startup_delay_sec').value)
        self.readiness_timeout_sec = float(
            self.get_parameter('readiness_timeout_sec').value
        )
        self.odometry_timeout_sec = float(
            self.get_parameter('odometry_timeout_sec').value
        )
        self.esp_status_timeout_sec = float(
            self.get_parameter('esp_status_timeout_sec').value
        )
        self.phase_timeout_sec = float(self.get_parameter('phase_timeout_sec').value)
        self.forward_distance_m = float(
            self.get_parameter('forward_distance_m').value
        )
        self.forward_command = abs(float(self.get_parameter('forward_command').value))
        self.target_depth_m = abs(float(self.get_parameter('target_depth_m').value))
        self.depth_tolerance_m = abs(float(
            self.get_parameter('depth_tolerance_m').value
        ))
        self.yaw_turn_rad = abs(float(self.get_parameter('yaw_turn_rad').value))
        self.yaw_kp = float(self.get_parameter('yaw_kp').value)
        self.yaw_kd = float(self.get_parameter('yaw_kd').value)
        self.max_yaw_command = abs(float(
            self.get_parameter('max_yaw_command').value
        ))
        self.turn_yaw_command = abs(float(
            self.get_parameter('turn_yaw_command').value
        ))
        self.heading_tolerance_rad = abs(float(
            self.get_parameter('heading_tolerance_rad').value
        ))
        self.return_position_tolerance_m = abs(float(
            self.get_parameter('return_position_tolerance_m').value
        ))
        self.max_cross_track_error_m = abs(float(
            self.get_parameter('max_cross_track_error_m').value
        ))
        self.max_depth_error_m = abs(float(
            self.get_parameter('max_depth_error_m').value
        ))
        self.max_tilt_rad = abs(float(self.get_parameter('max_tilt_rad').value))
        self.settle_sec = float(self.get_parameter('settle_sec').value)

        self.latest_odom = None
        self.latest_odom_ns = None
        self.latest_esp_status = None
        self.latest_esp_status_ns = None
        self.depth_start_z = None
        self.target_z = None
        self.start_x = None
        self.start_y = None
        self.outbound_heading = None

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.depth_target_pub = self.create_publisher(
            Float64,
            self.depth_target_topic,
            10,
        )
        self.create_subscription(
            Odometry, self.odometry_topic, self.odom_callback, 10
        )
        self.create_subscription(
            String, self.esp_status_topic, self.esp_status_callback, 10
        )

    def odom_callback(self, msg):
        self.latest_odom = msg
        self.latest_odom_ns = self.get_clock().now().nanoseconds

    def esp_status_callback(self, msg):
        try:
            self.latest_esp_status = json.loads(msg.data)
        except (TypeError, ValueError):
            self.latest_esp_status = None
        self.latest_esp_status_ns = self.get_clock().now().nanoseconds

    def age_sec(self, timestamp_ns):
        if timestamp_ns is None:
            return math.inf
        return (self.get_clock().now().nanoseconds - timestamp_ns) / 1e9

    def inputs_ready(self):
        status = self.latest_esp_status or {}
        return (
            self.age_sec(self.latest_odom_ns) <= self.odometry_timeout_sec
            and self.age_sec(self.latest_esp_status_ns) <= self.esp_status_timeout_sec
            and bool(status.get('serial_connected'))
            and bool(status.get('last_write_ok'))
        )

    def require_ready(self, check_depth=True):
        if not self.inputs_ready():
            raise RuntimeError('Odometry or ESP status became stale/unhealthy')
        roll, pitch = self.current_roll_pitch()
        if max(abs(roll), abs(pitch)) > self.max_tilt_rad:
            raise RuntimeError(
                f'Excessive tilt: roll={math.degrees(roll):.1f} deg, '
                f'pitch={math.degrees(pitch):.1f} deg'
            )
        if (
            check_depth
            and self.target_z is not None
            and abs(self.current_z() - self.target_z) > self.max_depth_error_m
        ):
            raise RuntimeError('Depth error exceeded safety limit')

    def run(self):
        self.get_logger().info('Waiting for fresh odometry and a healthy ESP bridge')
        self.wait_until(self.inputs_ready, self.readiness_timeout_sec)

        if self.dry_run:
            self.get_logger().warn('Dry run passed; no movement command was published')
            return

        self.get_logger().warn(
            f'PREQUAL WILL MOVE in {self.startup_delay_sec:.1f} seconds; '
            'use the physical kill switch to abort'
        )
        end = time.monotonic() + max(0.0, self.startup_delay_sec)
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            self.require_ready()
            # Publish nothing during the hold. This keeps the controller's
            # manual input stale, so both controller and bridge stay neutral.

        self.depth_start_z = self.current_z()
        self.target_z = self.depth_start_z - self.target_depth_m
        self.publish_depth_target()
        position = self.latest_odom.pose.pose.position
        self.start_x = position.x
        self.start_y = position.y
        self.outbound_heading = self.current_yaw()
        try:
            self.move_to_depth()
            self.stop_for(self.settle_sec, self.outbound_heading)
            self.drive_outbound()
            self.stop_for(self.settle_sec, self.outbound_heading)
            self.turn()
            return_heading = heading_error(
                self.outbound_heading + self.yaw_turn_rad,
                0.0,
            )
            self.stop_for(self.settle_sec, return_heading)
            self.drive_home()
            self.stop_for(self.settle_sec)
            self.get_logger().info('Prequal path complete')
        finally:
            self.publish_zero()

    def wait_until(self, predicate, timeout_sec):
        start = time.monotonic()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return
            if time.monotonic() - start > timeout_sec:
                raise RuntimeError('Timed out waiting for startup readiness')

    def run_phase(self, label, finished, command_factory, check_depth=True):
        self.get_logger().warn(f'Starting phase: {label}')
        start = time.monotonic()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            self.require_ready(check_depth=check_depth)
            if finished():
                self.publish_zero()
                return
            if time.monotonic() - start > self.phase_timeout_sec:
                raise RuntimeError(f'Phase timed out: {label}')
            self.cmd_pub.publish(command_factory())
            self.publish_depth_target()
            time.sleep(0.1)

    def move_to_depth(self):
        if self.target_depth_m <= self.depth_tolerance_m:
            return

        def command():
            msg = Twist()
            msg.angular.z = self.heading_command(self.outbound_heading)
            return msg

        self.run_phase(
            'descend',
            lambda: self.current_z() <= self.target_z + self.depth_tolerance_m,
            command,
            check_depth=False,
        )

    def drive_outbound(self):
        heading = self.outbound_heading

        def finished():
            along, cross = self.track_errors(heading)
            if abs(cross) > self.max_cross_track_error_m:
                raise RuntimeError(
                    f'Outbound cross-track error too large: {cross:.2f} m'
                )
            return along >= self.forward_distance_m

        def command():
            msg = Twist()
            msg.linear.x = self.forward_command
            msg.angular.z = self.heading_command(heading)
            return msg

        self.run_phase('drive outbound', finished, command)

    def turn(self):
        target = heading_error(self.outbound_heading + self.yaw_turn_rad, 0.0)

        def finished():
            return abs(heading_error(target, self.current_yaw())) <= (
                self.heading_tolerance_rad
            )

        def command():
            msg = Twist()
            error = heading_error(target, self.current_yaw())
            command_limit = self.turn_yaw_command
            command_value = self.heading_command(target)
            msg.angular.z = max(-command_limit, min(command_limit, command_value))
            return msg

        self.run_phase('turnaround', finished, command)

    def drive_home(self):
        def distance_home():
            position = self.latest_odom.pose.pose.position
            return math.hypot(self.start_x - position.x, self.start_y - position.y)

        def finished():
            return distance_home() <= self.return_position_tolerance_m

        def command():
            position = self.latest_odom.pose.pose.position
            desired_heading = math.atan2(
                self.start_y - position.y,
                self.start_x - position.x,
            )
            msg = Twist()
            msg.linear.x = self.forward_command
            msg.angular.z = self.heading_command(desired_heading)
            return msg

        self.run_phase('drive home', finished, command)

    def track_errors(self, heading):
        position = self.latest_odom.pose.pose.position
        dx = position.x - self.start_x
        dy = position.y - self.start_y
        along = dx * math.cos(heading) + dy * math.sin(heading)
        cross = -dx * math.sin(heading) + dy * math.cos(heading)
        return along, cross

    def heading_command(self, target_heading):
        error = heading_error(target_heading, self.current_yaw())
        yaw_rate = self.latest_odom.twist.twist.angular.z
        command = self.yaw_kp * error - self.yaw_kd * yaw_rate
        return max(-self.max_yaw_command, min(self.max_yaw_command, command))

    def stop_for(self, duration_sec, heading=None):
        end = time.monotonic() + max(0.0, duration_sec)
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.0)
            self.require_ready()
            msg = Twist()
            if heading is not None:
                msg.angular.z = self.heading_command(heading)
            self.cmd_pub.publish(msg)
            self.publish_depth_target()
            time.sleep(0.1)

    def current_z(self):
        return self.latest_odom.pose.pose.position.z

    def current_yaw(self):
        return yaw_from_quaternion(self.latest_odom.pose.pose.orientation)

    def current_roll_pitch(self):
        q = self.latest_odom.pose.pose.orientation
        roll = math.atan2(
            2.0 * (q.w * q.x + q.y * q.z),
            1.0 - 2.0 * (q.x * q.x + q.y * q.y),
        )
        pitch_term = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(max(-1.0, min(1.0, pitch_term)))
        return roll, pitch

    def publish_zero(self):
        self.cmd_pub.publish(Twist())

    def publish_depth_target(self):
        if self.target_z is None:
            return
        msg = Float64()
        msg.data = self.target_z
        self.depth_target_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PrequalTest()
    try:
        node.run()
    except Exception as exc:
        node.get_logger().error(str(exc))
        node.publish_zero()
        raise
    finally:
        node.publish_zero()
        node.destroy_node()
        rclpy.shutdown()
