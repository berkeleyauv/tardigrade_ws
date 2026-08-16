"""ROS integration check for the pool-test command chain."""

import math
import time
import unittest

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

from tardigrade_esp.depth_attitude_controller import DepthAttitudeController
from tardigrade_esp.thruster_mixer import ThrusterMixer
from tardigrade_teleop.xbox_cmd_vel import XboxCmdVel


class PoolControlChainTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.executor = SingleThreadedExecutor()
        self.test_node = Node('pool_control_chain_test')
        self.teleop = XboxCmdVel(parameter_overrides=[
            Parameter(
                'cmd_vel_topic', value='/tardigrade/cmd_vel/manual'),
        ])
        self.controller = DepthAttitudeController()
        self.mixer = ThrusterMixer()
        self.joy_pub = self.test_node.create_publisher(Joy, '/joy', 10)
        self.odom_pub = self.test_node.create_publisher(
            Odometry, '/tardigrade/state/odometry/filtered', 10)
        self.latest_thrusters = None
        self.thruster_sub = self.test_node.create_subscription(
            Float32MultiArray,
            '/tardigrade/thrusters/cmd',
            self._on_thrusters,
            10,
        )
        for node in (
                self.test_node, self.teleop, self.controller, self.mixer):
            self.executor.add_node(node)

    def tearDown(self):
        for node in (
                self.test_node, self.teleop, self.controller, self.mixer):
            self.executor.remove_node(node)
            node.destroy_node()

    def _on_thrusters(self, message):
        self.latest_thrusters = list(message.data)

    def _spin_with_inputs(self, joy, duration=0.35, odometry=None):
        if odometry is None:
            odometry = Odometry()
            odometry.pose.pose.orientation.w = 1.0
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.joy_pub.publish(joy)
            self.odom_pub.publish(odometry)
            self.executor.spin_once(timeout_sec=0.02)

    @staticmethod
    def _joy(deadman, surge):
        message = Joy()
        message.axes = [0.0, surge, 0.0, 0.0, 0.0]
        message.buttons = [0] * 11
        message.buttons[4] = int(deadman)
        return message

    def test_deadman_release_reaches_eight_zero_commands(self):
        self._spin_with_inputs(self._joy(True, 1.0))
        self.assertIsNotNone(self.latest_thrusters)
        self.assertEqual(len(self.latest_thrusters), 8)
        self.assertTrue(any(abs(value) > 0.0
                            for value in self.latest_thrusters))
        self.assertTrue(all(abs(value) <= 1.0
                            for value in self.latest_thrusters))

        self._spin_with_inputs(self._joy(False, 1.0), duration=0.2)
        self.assertEqual(self.latest_thrusters, [0.0] * 8)

    def test_roll_loop_does_not_drive_horizontal_thrusters(self):
        result = self.controller.set_parameters_atomically([
            Parameter('capture_initial_attitude_target', value=False),
            Parameter('enable_roll', value=True),
            Parameter('enable_pitch', value=False),
            Parameter('enable_yaw', value=False),
            Parameter('enable_depth', value=False),
        ])
        self.assertTrue(result.successful)
        odometry = Odometry()
        odometry.pose.pose.orientation.x = math.sin(0.1)
        odometry.pose.pose.orientation.w = math.cos(0.1)
        self._spin_with_inputs(
            self._joy(True, 0.0), odometry=odometry)

        vertical = [self.latest_thrusters[index] for index in (2, 3, 4, 5)]
        horizontal = [
            self.latest_thrusters[index] for index in (0, 1, 6, 7)
        ]
        self.assertTrue(any(abs(value) > 0.0 for value in vertical))
        self.assertEqual(horizontal, [0.0] * 4)

    def test_invalid_live_gain_is_rejected_without_change(self):
        original = self.controller.roll_kp
        result = self.controller.set_parameters_atomically([
            Parameter('roll_kp', value=-1.0),
        ])
        self.assertFalse(result.successful)
        self.assertEqual(self.controller.roll_kp, original)


if __name__ == '__main__':
    unittest.main()
