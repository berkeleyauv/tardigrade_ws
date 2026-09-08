"""Integration check for the modern pool control and actuator seam."""

import time
import unittest

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Joy

from tardigrade_control.thruster_actuator_mapper import (
    ThrusterActuatorMapper)
from tardigrade_control.thruster_allocator import ThrusterAllocator
from tardigrade_control.velocity_setpoint_mux import VelocitySetpointMux
from tardigrade_control.velocity_wrench_controller import (
    VelocityWrenchController)
from tardigrade_interfaces.msg import AllocationStatus, ThrusterCommands
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
        self.teleop = XboxCmdVel()
        self.mux = VelocitySetpointMux(parameter_overrides=[
            Parameter('active_source', value='manual'),
        ])
        self.controller = VelocityWrenchController()
        self.allocator = ThrusterAllocator()
        self.mapper = ThrusterActuatorMapper()
        self.joy_pub = self.test_node.create_publisher(Joy, '/joy', 10)
        self.odom_pub = self.test_node.create_publisher(
            Odometry, '/tardigrade/state/odometry/filtered', 10)
        self.latest_commands = None
        self.latest_allocation = None
        self.command_sub = self.test_node.create_subscription(
            ThrusterCommands,
            '/tardigrade/actuators/thruster_commands',
            self._on_commands,
            10,
        )
        self.allocation_sub = self.test_node.create_subscription(
            AllocationStatus,
            '/tardigrade/control/allocation_status',
            self._on_allocation,
            10,
        )
        self.nodes = (
            self.test_node, self.teleop, self.mux, self.controller,
            self.allocator, self.mapper,
        )
        for node in self.nodes:
            self.executor.add_node(node)

    def tearDown(self):
        for node in self.nodes:
            self.executor.remove_node(node)
            node.destroy_node()

    def _on_commands(self, message):
        self.latest_commands = message

    def _on_allocation(self, message):
        self.latest_allocation = message

    def _spin_with_inputs(self, joy, duration=0.5):
        odometry = Odometry()
        odometry.pose.pose.orientation.w = 1.0
        odometry.child_frame_id = 'base_link'
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.joy_pub.publish(joy)
            self.odom_pub.publish(odometry)
            self.executor.spin_once(timeout_sec=0.01)

    @staticmethod
    def _joy(deadman, surge=0.0, yaw=0.0):
        message = Joy()
        message.axes = [0.0, surge, 0.0, yaw, 0.0]
        message.buttons = [0] * 11
        message.buttons[4] = int(deadman)
        return message

    def test_deadman_command_reaches_named_actuator_interface(self):
        self._spin_with_inputs(self._joy(True, surge=1.0))
        self.assertIsNotNone(self.latest_commands)
        self.assertEqual(len(self.latest_commands.names), 8)
        self.assertEqual(len(self.latest_commands.setpoints), 8)
        self.assertTrue(any(
            abs(value) > 0.0 for value in self.latest_commands.setpoints))
        self.assertTrue(all(
            abs(value) <= 1.0 for value in self.latest_commands.setpoints))
        self.assertIsNotNone(self.latest_allocation)

    def test_deadman_release_neutralizes_all_thrusters(self):
        self._spin_with_inputs(self._joy(True, surge=1.0))
        self._spin_with_inputs(self._joy(False, surge=1.0), duration=0.3)
        self.assertEqual(list(self.latest_commands.setpoints), [0.0] * 8)

    def test_yaw_request_produces_opposed_horizontal_commands(self):
        self._spin_with_inputs(self._joy(True, yaw=1.0))
        by_name = dict(zip(
            self.latest_commands.names,
            self.latest_commands.setpoints,
        ))
        horizontal = [
            value for name, value in by_name.items()
            if name.endswith('horizontal')
        ]
        self.assertTrue(any(value > 0.0 for value in horizontal))
        self.assertTrue(any(value < 0.0 for value in horizontal))


if __name__ == '__main__':
    unittest.main()
