"""Gate mission state machine (RoboSub 2026 Task 1).

Consumes:
  /tardigrade/perception/gate   (tardigrade_interfaces/GateDetection)
  /tardigrade/state/odometry    (nav_msgs/Odometry, ENU/FLU -- IMU heading)

Produces:
  /tardigrade/cmd_vel           (geometry_msgs/Twist, BODY frame, FLU:
                                 +x forward, +y left, +z up,
                                 angular.z = yaw rate, +CCW)

States:
  IDLE      wait for /tardigrade/mission/start (std_srvs/Trigger)
  SUBMERGE  timed open-loop descent (no depth sensor!)
  SEARCH    yaw in place until the gate is seen consistently
  ALIGN     visual servo: center gate (sway) + square up (yaw), creep forward
  THROUGH   heading-hold dead reckon forward through the gate
  DONE      stop (zero velocity)

Operator flow:
  1. ros2 service call /tardigrade/set_armed tardigrade_interfaces/srv/SetArmed "{armed: true}"
  2. ros2 service call /tardigrade/set_external_control tardigrade_interfaces/srv/SetExternalControl "{enabled: true}"
  3. ros2 service call /tardigrade/mission/start std_srvs/srv/Trigger
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

from tardigrade_interfaces.msg import GateDetection


def yaw_from_quaternion(q):
    """Yaw (rad) from ENU/FLU quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class GateMission(Node):

    IDLE = 'IDLE'
    SUBMERGE = 'SUBMERGE'
    SEARCH = 'SEARCH'
    ALIGN = 'ALIGN'
    THROUGH = 'THROUGH'
    DONE = 'DONE'

    def __init__(self):
        super().__init__('gate_mission')

        # -- Topics --
        self.declare_parameter('detection_topic', '/tardigrade/perception/gate')
        self.declare_parameter('odom_topic', '/tardigrade/state/odometry')
        self.declare_parameter('cmd_vel_topic', '/tardigrade/cmd_vel')

        # -- Speeds (m/s, rad/s). Start SLOW in the pool. --
        self.declare_parameter('submerge_speed', 0.2)      # downward
        self.declare_parameter('submerge_duration', 4.0)   # s (open loop, no depth sensor)
        self.declare_parameter('search_yaw_rate', 0.25)    # rad/s
        self.declare_parameter('search_direction', 1.0)    # +1 = CCW/left, -1 = CW/right
        self.declare_parameter('approach_speed', 0.25)     # forward while aligning
        self.declare_parameter('through_speed', 0.4)       # forward through the gate
        self.declare_parameter('through_duration', 6.0)    # s of dead reckoning

        # -- Controller gains --
        self.declare_parameter('kp_yaw', 0.8)      # yawspeed per unit yaw_signal
        self.declare_parameter('kp_lateral', 0.5)  # sway m/s per unit lateral
        self.declare_parameter('kp_heading', 0.7)  # yawspeed per rad heading error
        self.declare_parameter('max_yaw_rate', 0.4)
        self.declare_parameter('max_sway', 0.25)

        # -- Thresholds --
        self.declare_parameter('detect_confirm_frames', 5)   # frames to confirm gate in SEARCH
        self.declare_parameter('lost_timeout', 2.0)           # s without detection in ALIGN -> SEARCH
        self.declare_parameter('close_width_frac', 0.55)      # gate fills this much of frame -> THROUGH
        self.declare_parameter('aligned_yaw_tol', 0.08)       # |yaw_signal| below this = squared up
        self.declare_parameter('aligned_lateral_tol', 0.10)   # |lateral| below this = centered
        self.declare_parameter('mission_timeout', 180.0)      # s, hard stop

        self.detection_topic = self.get_parameter('detection_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.state = self.IDLE
        self.state_entered = None       # rclpy Time
        self.mission_started = None
        self.last_detection = None      # GateDetection
        self.last_detection_time = None
        self.consecutive_detections = 0
        self.current_yaw = None         # from odometry
        self.hold_yaw = None            # heading target for THROUGH

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.detection_sub = self.create_subscription(
            GateDetection, self.detection_topic, self.detection_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10)

        self.start_srv = self.create_service(
            Trigger, '/tardigrade/mission/start', self.handle_start)
        self.abort_srv = self.create_service(
            Trigger, '/tardigrade/mission/abort', self.handle_abort)

        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        self.get_logger().info('Gate mission ready. Call /tardigrade/mission/start')

    # ------------------------------------------------------------------ io

    def detection_callback(self, msg):
        if msg.detected:
            self.last_detection = msg
            self.last_detection_time = self.get_clock().now()
            self.consecutive_detections += 1
        else:
            self.consecutive_detections = 0

    def odom_callback(self, msg):
        self.current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def handle_start(self, request, response):
        if self.state != self.IDLE:
            response.success = False
            response.message = 'Mission already running (state={})'.format(self.state)
            return response
        self.mission_started = self.get_clock().now()
        self.transition(self.SUBMERGE)
        response.success = True
        response.message = 'Mission started'
        return response

    def handle_abort(self, request, response):
        self.transition(self.DONE)
        response.success = True
        response.message = 'Mission aborted, commanding zero velocity'
        return response

    # ------------------------------------------------------------- helpers

    def transition(self, new_state):
        self.get_logger().warn('STATE: {} -> {}'.format(self.state, new_state))
        self.state = new_state
        self.state_entered = self.get_clock().now()
        if new_state == self.THROUGH:
            self.hold_yaw = self.current_yaw

    def seconds_in_state(self):
        if self.state_entered is None:
            return 0.0
        return (self.get_clock().now() - self.state_entered).nanoseconds * 1e-9

    def seconds_since_detection(self):
        if self.last_detection_time is None:
            return float('inf')
        return (self.get_clock().now() - self.last_detection_time).nanoseconds * 1e-9

    def p(self, name):
        return float(self.get_parameter(name).value)

    def clamp(self, v, lim):
        return max(-lim, min(lim, v))

    # -------------------------------------------------------- control loop

    def control_loop(self):
        cmd = Twist()  # zeros

        # Hard mission timeout
        if self.mission_started is not None and self.state not in (self.IDLE, self.DONE):
            elapsed = (self.get_clock().now() - self.mission_started).nanoseconds * 1e-9
            if elapsed > self.p('mission_timeout'):
                self.get_logger().error('Mission timeout, stopping')
                self.transition(self.DONE)

        if self.state == self.SUBMERGE:
            cmd = self.run_submerge()
        elif self.state == self.SEARCH:
            cmd = self.run_search()
        elif self.state == self.ALIGN:
            cmd = self.run_align()
        elif self.state == self.THROUGH:
            cmd = self.run_through()
        # IDLE / DONE publish zeros -> pixhawk_interface holds zero velocity

        self.cmd_pub.publish(cmd)

    def run_submerge(self):
        cmd = Twist()
        cmd.linear.z = -self.p('submerge_speed')  # FLU: -z = down
        if self.seconds_in_state() >= self.p('submerge_duration'):
            self.transition(self.SEARCH)
        return cmd

    def run_search(self):
        cmd = Twist()
        cmd.angular.z = self.p('search_direction') * self.p('search_yaw_rate')
        if self.consecutive_detections >= int(self.p('detect_confirm_frames')):
            self.transition(self.ALIGN)
        return cmd

    def run_align(self):
        cmd = Twist()

        # Lost the gate? Coast briefly, then go back to searching.
        if self.seconds_since_detection() > self.p('lost_timeout'):
            self.consecutive_detections = 0
            self.transition(self.SEARCH)
            return cmd

        det = self.last_detection
        if det is None:
            return cmd

        # Sway to center the gate. lateral > 0 = gate right of center.
        # FLU: +y = left, so strafe right = negative y.
        cmd.linear.y = self.clamp(-self.p('kp_lateral') * det.lateral,
                                  self.p('max_sway'))

        # Yaw to square up. yaw_signal > 0 -> rotate left (+CCW = +angular.z).
        if det.both_posts:
            cmd.angular.z = self.clamp(self.p('kp_yaw') * det.yaw_signal,
                                       self.p('max_yaw_rate'))

        # Creep forward once roughly centered; full approach speed when aligned.
        centered = abs(det.lateral) < self.p('aligned_lateral_tol')
        squared = (not det.both_posts) or abs(det.yaw_signal) < self.p('aligned_yaw_tol')
        if centered and squared:
            cmd.linear.x = self.p('approach_speed')
        elif centered:
            cmd.linear.x = 0.5 * self.p('approach_speed')

        # Close enough that the gate fills the frame -> commit and drive through.
        if det.gate_width_frac >= self.p('close_width_frac'):
            self.transition(self.THROUGH)

        return cmd

    def run_through(self):
        cmd = Twist()
        cmd.linear.x = self.p('through_speed')

        # Heading hold using IMU yaw (gate likely out of view this close)
        if self.hold_yaw is not None and self.current_yaw is not None:
            err = wrap_angle(self.hold_yaw - self.current_yaw)
            cmd.angular.z = self.clamp(self.p('kp_heading') * err,
                                       self.p('max_yaw_rate'))

        if self.seconds_in_state() >= self.p('through_duration'):
            self.get_logger().warn('Gate mission complete!')
            self.transition(self.DONE)
        return cmd


def main(args=None):
    rclpy.init(args=args)
    node = GateMission()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
