import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tardigrade_interfaces.msg import RobotStatus
from tardigrade_interfaces.srv import SetArmed, SetExternalControl


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_delta(current, previous):
    return math.atan2(math.sin(current - previous), math.cos(current - previous))


class PrequalMission(Node):
    def __init__(self):
        super().__init__('prequal_mission')

        self.declare_parameter('cmd_vel_topic', '/tardigrade/cmd_vel')
        self.declare_parameter('status_topic', '/tardigrade/status')
        self.declare_parameter('odometry_topic', '/tardigrade/state/odometry')
        self.declare_parameter('forward_distance_m', 1.0)
        self.declare_parameter('forward_speed_mps', 0.15)
        self.declare_parameter('target_depth_m', 0.0)
        self.declare_parameter('vertical_speed_mps', 0.05)
        self.declare_parameter('depth_tolerance_m', 0.05)
        self.declare_parameter('depth_kp', 0.8)
        self.declare_parameter('yaw_turn_rad', math.pi)
        self.declare_parameter('yaw_rate_radps', 0.25)
        self.declare_parameter('settle_sec', 1.0)
        self.declare_parameter('status_timeout_sec', 20.0)
        self.declare_parameter('odometry_timeout_sec', 10.0)
        self.declare_parameter('arm_timeout_sec', 15.0)
        self.declare_parameter('phase_timeout_sec', 60.0)
        self.declare_parameter('dry_run', True)
        self.declare_parameter('disarm_when_done', True)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.odometry_topic = self.get_parameter('odometry_topic').value
        self.forward_distance_m = float(self.get_parameter('forward_distance_m').value)
        self.forward_speed_mps = float(self.get_parameter('forward_speed_mps').value)
        self.target_depth_m = float(self.get_parameter('target_depth_m').value)
        self.vertical_speed_mps = abs(float(self.get_parameter('vertical_speed_mps').value))
        self.depth_tolerance_m = float(self.get_parameter('depth_tolerance_m').value)
        self.depth_kp = float(self.get_parameter('depth_kp').value)
        self.yaw_turn_rad = abs(float(self.get_parameter('yaw_turn_rad').value))
        self.yaw_rate_radps = float(self.get_parameter('yaw_rate_radps').value)
        self.settle_sec = float(self.get_parameter('settle_sec').value)
        self.status_timeout_sec = float(self.get_parameter('status_timeout_sec').value)
        self.odometry_timeout_sec = float(self.get_parameter('odometry_timeout_sec').value)
        self.arm_timeout_sec = float(self.get_parameter('arm_timeout_sec').value)
        self.phase_timeout_sec = float(self.get_parameter('phase_timeout_sec').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.disarm_when_done = bool(self.get_parameter('disarm_when_done').value)

        self.latest_status = None
        self.latest_odom = None
        self.depth_reference_z = None
        self.target_z = None
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.status_sub = self.create_subscription(
            RobotStatus,
            self.status_topic,
            self.status_callback,
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odometry_topic,
            self.odometry_callback,
            10,
        )
        self.arm_client = self.create_client(SetArmed, '/tardigrade/set_armed')
        self.external_client = self.create_client(
            SetExternalControl,
            '/tardigrade/set_external_control',
        )

    def status_callback(self, msg):
        self.latest_status = msg

    def odometry_callback(self, msg):
        self.latest_odom = msg

    def run(self):
        if self.dry_run:
            self.get_logger().warn(
                'dry_run is true: this will validate topics/services but will '
                'not arm or publish motion. '
                'Set -p dry_run:=false for the real pre-qualification run.'
            )

        self.wait_for_startup_inputs()
        if self.dry_run:
            self.get_logger().info('Dry run passed: status and odometry are available.')
            return

        self.enable_external_control()
        self.arm()

        try:
            self.configure_depth_target()
            self.move_to_target_depth()
            self.stop_for(self.settle_sec, 'settling after depth target')
            self.drive_distance(
                self.forward_distance_m,
                self.forward_speed_mps,
                'outbound forward',
            )
            self.stop_for(self.settle_sec, 'settling before turn')
            self.turn_yaw(self.yaw_turn_rad, self.yaw_rate_radps, 'turnaround')
            self.stop_for(self.settle_sec, 'settling after turn')
            self.drive_distance(
                self.forward_distance_m,
                self.forward_speed_mps,
                'return forward',
            )
            self.stop_for(self.settle_sec, 'mission complete stop')
            self.get_logger().info('Pre-qualification path complete.')
        finally:
            self.publish_zero()
            if self.disarm_when_done:
                self.disarm()

    def wait_for_startup_inputs(self):
        self.get_logger().info('Waiting for Pixhawk status...')
        self.wait_until(
            lambda: self.latest_status is not None and self.latest_status.px4_connected,
            self.status_timeout_sec,
            'Timed out waiting for /tardigrade/status with px4_connected=true',
        )

        self.get_logger().info('Waiting for odometry...')
        self.wait_until(
            lambda: self.latest_odom is not None,
            self.odometry_timeout_sec,
            f'Timed out waiting for {self.odometry_topic}',
        )

    def enable_external_control(self):
        self.call_external_control(True)
        self.wait_until(
            lambda: self.latest_status is not None and self.latest_status.external_control_enabled,
            self.status_timeout_sec,
            'Timed out waiting for external_control_enabled=true',
        )

    def arm(self):
        self.call_armed(True)
        self.wait_until(
            lambda: self.latest_status is not None and self.latest_status.armed,
            self.arm_timeout_sec,
            'Timed out waiting for armed=true',
        )
        self.get_logger().warn('Pixhawk is armed. Starting motion sequence.')

    def disarm(self):
        self.call_armed(False)
        self.get_logger().info('Disarm command sent.')

    def configure_depth_target(self):
        self.depth_reference_z = self.current_z()
        self.target_z = self.depth_reference_z - self.target_depth_m
        self.get_logger().info(
            f'Depth target: {self.target_depth_m:.2f} m below start '
            f'(start_z={self.depth_reference_z:.2f}, target_z={self.target_z:.2f})'
        )

    def move_to_target_depth(self):
        if self.target_z is None or self.target_depth_m <= self.depth_tolerance_m:
            return

        self.get_logger().info(
            f'Descending to target depth at up to {self.vertical_speed_mps:.2f} m/s'
        )
        self.run_phase(
            'descend to target depth',
            lambda: abs(self.depth_error()) <= self.depth_tolerance_m,
            Twist(),
        )

    def drive_distance(self, distance_m, speed_mps, label):
        start = self.current_position_xy()
        cmd = Twist()
        cmd.linear.x = abs(speed_mps)

        self.get_logger().info(f'{label}: driving {distance_m:.2f} m at {cmd.linear.x:.2f} m/s')
        self.run_phase(
            label,
            lambda: self.distance_from(start) >= distance_m,
            cmd,
        )

    def turn_yaw(self, target_rad, yaw_rate_radps, label):
        previous_yaw = self.current_yaw()
        progress = 0.0
        direction = 1.0 if yaw_rate_radps >= 0.0 else -1.0
        cmd = Twist()
        cmd.angular.z = direction * abs(yaw_rate_radps)

        self.get_logger().info(
            f'{label}: turning {target_rad:.2f} rad at '
            f'{cmd.angular.z:.2f} rad/s'
        )

        def finished():
            nonlocal previous_yaw, progress
            yaw = self.current_yaw()
            progress += angle_delta(yaw, previous_yaw)
            previous_yaw = yaw
            return abs(progress) >= target_rad

        self.run_phase(label, finished, cmd)

    def run_phase(self, label, finished, cmd):
        start = time.monotonic()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            if finished():
                self.publish_zero()
                self.get_logger().info(f'{label}: done')
                return

            if time.monotonic() - start > self.phase_timeout_sec:
                self.publish_zero()
                raise RuntimeError(f'{label}: timed out')

            self.cmd_pub.publish(self.with_depth_hold(cmd))
            time.sleep(0.1)

    def stop_for(self, duration_sec, label):
        self.get_logger().info(label)
        end = time.monotonic() + max(duration_sec, 0.0)
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.0)
            self.cmd_pub.publish(self.with_depth_hold(Twist()))
            time.sleep(0.1)

    def publish_zero(self):
        self.cmd_pub.publish(Twist())

    def with_depth_hold(self, base_cmd):
        cmd = Twist()
        cmd.linear.x = base_cmd.linear.x
        cmd.linear.y = base_cmd.linear.y
        cmd.linear.z = base_cmd.linear.z
        cmd.angular.x = base_cmd.angular.x
        cmd.angular.y = base_cmd.angular.y
        cmd.angular.z = base_cmd.angular.z

        if self.target_z is not None:
            cmd.linear.z = self.depth_hold_velocity()

        return cmd

    def depth_hold_velocity(self):
        correction = self.depth_kp * self.depth_error()
        return max(-self.vertical_speed_mps, min(self.vertical_speed_mps, correction))

    def depth_error(self):
        if self.target_z is None:
            return 0.0
        return self.target_z - self.current_z()

    def current_position_xy(self):
        pose = self.latest_odom.pose.pose.position
        return pose.x, pose.y

    def current_z(self):
        return self.latest_odom.pose.pose.position.z

    def distance_from(self, start_xy):
        x, y = self.current_position_xy()
        return math.hypot(x - start_xy[0], y - start_xy[1])

    def current_yaw(self):
        return yaw_from_quaternion(self.latest_odom.pose.pose.orientation)

    def call_external_control(self, enabled):
        if not self.external_client.wait_for_service(timeout_sec=self.status_timeout_sec):
            raise RuntimeError('Service /tardigrade/set_external_control is not available')
        request = SetExternalControl.Request()
        request.enabled = enabled
        response = self.call_service(self.external_client, request)
        if not response.success:
            raise RuntimeError(response.message)
        self.get_logger().info(response.message)

    def call_armed(self, armed):
        if not self.arm_client.wait_for_service(timeout_sec=self.status_timeout_sec):
            raise RuntimeError('Service /tardigrade/set_armed is not available')
        request = SetArmed.Request()
        request.armed = armed
        response = self.call_service(self.arm_client, request)
        if not response.success:
            raise RuntimeError(response.message)
        self.get_logger().info(response.message)

    def call_service(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.status_timeout_sec)
        if not future.done():
            raise RuntimeError('Timed out waiting for service response')
        return future.result()

    def wait_until(self, predicate, timeout_sec, error_message):
        start = time.monotonic()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return
            if time.monotonic() - start > timeout_sec:
                raise RuntimeError(error_message)


def main(args=None):
    rclpy.init(args=args)
    node = PrequalMission()
    try:
        node.run()
    except Exception as exc:
        node.get_logger().error(str(exc))
        node.publish_zero()
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()
