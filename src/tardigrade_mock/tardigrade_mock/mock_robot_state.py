import math

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, FluidPressure, Imu, Temperature
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from tardigrade_interfaces.msg import (
    EspState,
    GateDetection,
    RobotStatus,
    SlalomMarkerDetection,
)


def clamp(value, low, high):
    return max(low, min(high, value))


def angle_wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def quaternion_from_euler(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    msg = Quaternion()
    msg.w = cr * cp * cy + sr * sp * sy
    msg.x = sr * cp * cy - cr * sp * sy
    msg.y = cr * sp * cy + sr * cp * sy
    msg.z = cr * cp * sy - sr * sp * cy
    return msg


class MockRobotState(Node):
    def __init__(self):
        super().__init__('mock_robot_state')

        self.armed = False
        self.start_time = self.get_clock().now()
        self.last_tick = self.start_time
        self.path_poses = []
        self.latest_command = Twist()
        self.z = -0.4
        self.roll = 0.18
        self.pitch = -0.12
        self.yaw = 0.4
        self.vertical_velocity = 0.0
        self.roll_rate = 0.0
        self.pitch_rate = 0.0
        self.yaw_rate = 0.0

        self.status_pub = self.create_publisher(
            RobotStatus, '/tardigrade/status', 10
        )
        self.odom_pub = self.create_publisher(
            Odometry, '/tardigrade/state/odometry', 10
        )
        self.filtered_odom_pub = self.create_publisher(
            Odometry, '/tardigrade/state/odometry/filtered', 10
        )
        self.zed_odom_pub = self.create_publisher(
            Odometry, '/zed/zed_node/odom', 10
        )
        self.zed_pose_pub = self.create_publisher(
            PoseStamped, '/zed/zed_node/pose', 10
        )
        self.imu_pub = self.create_publisher(
            Imu, '/vectornav/imu', 10
        )
        self.path_pub = self.create_publisher(
            Path, '/tardigrade/state/path', 10
        )
        self.battery_pub = self.create_publisher(
            BatteryState, '/tardigrade/power/battery', 10
        )
        self.pressure_pub = self.create_publisher(
            FluidPressure, '/tardigrade/sensors/pressure', 10
        )
        self.temperature_pub = self.create_publisher(
            Temperature, '/tardigrade/sensors/temperature', 10
        )
        self.gate_pub = self.create_publisher(
            GateDetection, '/tardigrade/perception/gate', 10
        )
        self.slalom_pub = self.create_publisher(
            SlalomMarkerDetection, '/tardigrade/perception/slalom', 10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, '/tardigrade/visualization/task_markers', 10
        )
        self.autonomy_state_pub = self.create_publisher(
            String, '/tardigrade/autonomy/state', 10
        )
        self.autonomy_event_pub = self.create_publisher(
            String, '/tardigrade/autonomy/events', 10
        )
        self.create_subscription(
            EspState,
            '/tardigrade/esp/state',
            self.esp_state_callback,
            10,
        )
        self.create_subscription(
            Twist,
            '/tardigrade/cmd_vel',
            self.command_callback,
            10,
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transforms()
        self.timer = self.create_timer(1.0 / 30.0, self.tick)
        self.slow_timer = self.create_timer(0.5, self.publish_slow_topics)

        self.get_logger().info('Publishing broad mock robot telemetry')

    def esp_state_callback(self, msg):
        self.armed = msg.armed

    def command_callback(self, msg):
        self.latest_command = msg

    def elapsed_sec(self):
        elapsed = self.get_clock().now() - self.start_time
        return elapsed.nanoseconds / 1_000_000_000.0

    def trajectory(self, elapsed):
        x = 2.0 + 1.5 * math.sin(elapsed * 0.12)
        y = 0.6 * math.sin(elapsed * 0.20)
        vx = 0.18 * math.cos(elapsed * 0.12)
        vy = 0.12 * math.cos(elapsed * 0.20)
        return (
            x, y, self.z, self.roll, self.pitch, self.yaw,
            vx, vy, self.vertical_velocity,
            self.roll_rate, self.pitch_rate, self.yaw_rate,
        )

    def tick(self):
        now = self.get_clock().now()
        dt = clamp(
            (now - self.last_tick).nanoseconds / 1_000_000_000.0,
            0.001,
            0.1,
        )
        self.last_tick = now
        elapsed = self.elapsed_sec()
        self.update_dynamics(dt, elapsed)
        state = self.trajectory(elapsed)
        filtered = self.make_odometry(state, raw=False)
        raw = self.make_odometry(state, raw=True)

        self.filtered_odom_pub.publish(filtered)
        self.odom_pub.publish(raw)
        self.zed_odom_pub.publish(raw)
        self.publish_zed_pose(raw)
        self.publish_imu(state)
        self.publish_tf(filtered)
        self.publish_path(filtered)
        self.publish_status(state)
        self.publish_gate_detection(state)
        self.publish_slalom_detection(state)

    def update_dynamics(self, dt, elapsed):
        command = self.latest_command
        self.vertical_velocity = (
            0.85 * command.linear.z
            + 0.025 * math.sin(elapsed * 0.7)
        )
        self.roll_rate = (
            1.8 * command.angular.x
            - 0.7 * self.roll
            + 0.025 * math.sin(elapsed * 0.9)
        )
        self.pitch_rate = (
            1.7 * command.angular.y
            - 0.7 * self.pitch
            + 0.020 * math.sin(elapsed * 0.8)
        )
        self.yaw_rate = (
            1.3 * command.angular.z
            - 0.15 * self.yaw
            + 0.018 * math.sin(elapsed * 0.5)
        )
        self.z += self.vertical_velocity * dt
        self.roll = angle_wrap(self.roll + self.roll_rate * dt)
        self.pitch = angle_wrap(self.pitch + self.pitch_rate * dt)
        self.yaw = angle_wrap(self.yaw + self.yaw_rate * dt)

    def make_odometry(self, state, raw):
        (
            x, y, z, roll, pitch, yaw,
            vx, vy, vz, roll_rate, pitch_rate, yaw_rate,
        ) = state
        if raw:
            elapsed = self.elapsed_sec()
            x += 0.025 * math.sin(elapsed * 2.1)
            y += 0.020 * math.sin(elapsed * 1.7)
            z += 0.030 * math.sin(elapsed * 2.4)
            yaw += 0.015 * math.sin(elapsed * 1.9)

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation = quaternion_from_euler(roll, pitch, yaw)
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.linear.z = vz
        msg.twist.twist.angular.x = roll_rate
        msg.twist.twist.angular.y = pitch_rate
        msg.twist.twist.angular.z = yaw_rate
        return msg

    def publish_zed_pose(self, odometry):
        msg = PoseStamped()
        msg.header = odometry.header
        msg.pose = odometry.pose.pose
        self.zed_pose_pub.publish(msg)

    def publish_imu(self, state):
        (
            x, y, z, roll, pitch, yaw,
            vx, vy, vz, roll_rate, pitch_rate, yaw_rate,
        ) = state
        del x, y, z, vx, vy, vz
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'vectornav_link'
        msg.orientation = quaternion_from_euler(roll, pitch, yaw)
        msg.angular_velocity.x = roll_rate
        msg.angular_velocity.y = pitch_rate
        msg.angular_velocity.z = yaw_rate
        msg.linear_acceleration.z = 9.80665
        msg.orientation_covariance[0] = 0.0025
        msg.orientation_covariance[4] = 0.0025
        msg.orientation_covariance[8] = 0.004
        self.imu_pub.publish(msg)

    def publish_tf(self, odometry):
        transform = TransformStamped()
        transform.header.stamp = odometry.header.stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = odometry.pose.pose.position.x
        transform.transform.translation.y = odometry.pose.pose.position.y
        transform.transform.translation.z = odometry.pose.pose.position.z
        transform.transform.rotation = odometry.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def publish_static_transforms(self):
        stamp = self.get_clock().now().to_msg()
        transforms = []
        for child_frame, x, y, z in (
            ('zed_camera_link', 0.20, 0.0, 0.05),
            ('vectornav_link', 0.0, 0.0, 0.0),
            ('downward_camera_link', 0.0, 0.0, -0.12),
        ):
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = 'base_link'
            transform.child_frame_id = child_frame
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = z
            transform.transform.rotation.w = 1.0
            transforms.append(transform)
        self.static_tf_broadcaster.sendTransform(transforms)

    def publish_path(self, odometry):
        pose = PoseStamped()
        pose.header = odometry.header
        pose.pose = odometry.pose.pose
        self.path_poses.append(pose)
        self.path_poses = self.path_poses[-500:]

        msg = Path()
        msg.header = odometry.header
        msg.poses = self.path_poses
        self.path_pub.publish(msg)

    def publish_status(self, state):
        x, y, z, roll, pitch, yaw = state[:6]
        del roll, pitch
        msg = RobotStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.control_connected = True
        msg.armed = self.armed
        msg.external_control_enabled = True
        msg.nav_state = 1
        msg.arming_state = 1 if self.armed else 0
        msg.detail = (
            'mock_robot_state; '
            f'x={x:.2f}; y={y:.2f}; z={z:.2f}; yaw={yaw:.2f}'
        )
        self.status_pub.publish(msg)

    def publish_gate_detection(self, state):
        x, y, z, roll, pitch, yaw = state[:6]
        del roll
        dx = 4.0 - x
        dy = -y
        distance = math.sqrt(dx * dx + dy * dy + z * z)
        yaw_error = angle_wrap(math.atan2(dy, dx) - yaw)
        pitch_error = math.atan2(-z, max(math.hypot(dx, dy), 0.01)) - pitch
        visible = distance <= 8.0 and abs(yaw_error) <= 0.6

        msg = GateDetection()
        msg.stamp = self.get_clock().now().to_msg()
        msg.visible = visible
        msg.confidence = 0.95 if visible else 0.0
        msg.center_x = clamp(0.5 + yaw_error / 1.2, 0.0, 1.0)
        msg.center_y = clamp(0.5 + pitch_error / 0.9, 0.0, 1.0)
        msg.yaw_error_rad = yaw_error
        msg.pitch_error_rad = pitch_error
        msg.distance_m = distance
        msg.survey_and_repair_visible = visible
        msg.search_and_rescue_visible = visible
        msg.recommended_side = 'survey_and_repair' if visible else ''
        self.gate_pub.publish(msg)

    def publish_slalom_detection(self, state):
        x, y, z, roll, pitch, yaw = state[:6]
        del z, roll, pitch
        dx = 8.0 - x
        dy = -0.5 - y
        distance = math.hypot(dx, dy)
        yaw_error = angle_wrap(math.atan2(dy, dx) - yaw)
        visible = distance <= 8.0 and abs(yaw_error) <= 0.9

        msg = SlalomMarkerDetection()
        msg.stamp = self.get_clock().now().to_msg()
        msg.marker_index = 1
        msg.visible = visible
        msg.confidence = 0.85 if visible else 0.0
        msg.center_x = clamp(0.5 + yaw_error / 1.8, 0.0, 1.0)
        msg.center_y = 0.5
        msg.yaw_error_rad = yaw_error
        msg.distance_m = distance
        msg.left_side_yaw_error_rad = yaw_error
        msg.left_side_lateral_offset_m = 0.75
        self.slalom_pub.publish(msg)

    def publish_slow_topics(self):
        elapsed = self.elapsed_sec()
        state = self.trajectory(elapsed)
        z = state[2]

        battery = BatteryState()
        battery.header.stamp = self.get_clock().now().to_msg()
        battery.voltage = 15.8 - 0.25 * (1.0 + math.sin(elapsed * 0.03))
        battery.current = 4.0 + 1.5 * math.sin(elapsed * 0.4)
        battery.percentage = clamp((battery.voltage - 13.2) / 3.6, 0.0, 1.0)
        battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        battery.present = True
        self.battery_pub.publish(battery)

        pressure = FluidPressure()
        pressure.header.stamp = self.get_clock().now().to_msg()
        pressure.header.frame_id = 'base_link'
        pressure.fluid_pressure = 101325.0 + max(-z, 0.0) * 10050.0
        pressure.variance = 16.0
        self.pressure_pub.publish(pressure)

        temperature = Temperature()
        temperature.header = pressure.header
        temperature.temperature = 21.0 + 0.6 * math.sin(elapsed * 0.05)
        temperature.variance = 0.04
        self.temperature_pub.publish(temperature)

        self.publish_task_markers()
        autonomy_state = String()
        autonomy_state.data = 'GATE_APPROACH' if elapsed % 30.0 < 18.0 else 'SLALOM'
        self.autonomy_state_pub.publish(autonomy_state)
        autonomy_event = String()
        autonomy_event.data = f'mock heartbeat t={elapsed:.1f}s'
        self.autonomy_event_pub.publish(autonomy_event)

    def publish_task_markers(self):
        markers = MarkerArray()
        markers.markers = [
            self.make_vehicle_marker(),
            self.make_marker(0, 'gate_left', Marker.CUBE, 4.0, -0.8, -1.0),
            self.make_marker(1, 'gate_right', Marker.CUBE, 4.0, 0.8, -1.0),
            self.make_marker(2, 'slalom_1', Marker.CYLINDER, 8.0, -0.5, -1.0),
            self.make_marker(3, 'slalom_2', Marker.CYLINDER, 10.0, 0.5, -1.0),
        ]
        self.marker_pub.publish(markers)

    def make_vehicle_marker(self):
        msg = Marker()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.ns = 'mock_vehicle'
        msg.id = 0
        msg.type = Marker.CUBE
        msg.action = Marker.ADD
        msg.pose.orientation.w = 1.0
        msg.scale.x = 0.85
        msg.scale.y = 0.55
        msg.scale.z = 0.30
        msg.color.r = 0.10
        msg.color.g = 0.75
        msg.color.b = 0.88
        msg.color.a = 0.9
        return msg

    def make_marker(self, marker_id, name, marker_type, x, y, z):
        msg = Marker()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.ns = 'mock_tasks'
        msg.id = marker_id
        msg.type = marker_type
        msg.action = Marker.ADD
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        msg.scale.x = 0.16 if marker_type == Marker.CUBE else 0.25
        msg.scale.y = 0.16 if marker_type == Marker.CUBE else 0.25
        msg.scale.z = 1.5
        msg.color.r = 1.0 if marker_type == Marker.CUBE else 0.2
        msg.color.g = 0.55 if marker_type == Marker.CUBE else 0.9
        msg.color.b = 0.05 if marker_type == Marker.CUBE else 1.0
        msg.color.a = 0.9
        msg.text = name
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = MockRobotState()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
