"""Geometry-based six-DOF wrench allocator."""

import math

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node

from tardigrade_description.vehicle_model import load_vehicle_model
from tardigrade_interfaces.msg import AllocationStatus, ThrusterForces

from .actuator_math import allocate_wrench


class ThrusterAllocator(Node):
    def __init__(self, **node_kwargs):
        super().__init__('thruster_allocator', **node_kwargs)
        self.declare_parameter(
            'input_topic', '/tardigrade/control/wrench_command')
        self.declare_parameter(
            'output_topic', '/tardigrade/actuators/thruster_forces')
        self.declare_parameter(
            'status_topic', '/tardigrade/control/allocation_status')
        self.declare_parameter('vehicle_config', '')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('command_timeout_sec', 0.25)
        self.declare_parameter('feasible_residual_tolerance', 0.25)

        path = str(self.get_parameter('vehicle_config').value) or None
        self.model = load_vehicle_model(path)
        self.names = self.model.thruster_names
        self.matrix = self.model.allocation_matrix
        self.lower_limits = [
            -float(item['max_reverse_n']) for item in self.model.thrusters]
        self.upper_limits = [
            float(item['max_forward_n']) for item in self.model.thrusters]
        self.timeout = float(
            self.get_parameter('command_timeout_sec').value)
        self.feasible_tolerance = abs(float(
            self.get_parameter('feasible_residual_tolerance').value))
        self.latest = None
        self.latest_ns = None

        self.subscriber = self.create_subscription(
            WrenchStamped,
            str(self.get_parameter('input_topic').value),
            self.on_wrench,
            10,
        )
        self.publisher = self.create_publisher(
            ThrusterForces,
            str(self.get_parameter('output_topic').value),
            10,
        )
        self.status_publisher = self.create_publisher(
            AllocationStatus,
            str(self.get_parameter('status_topic').value),
            10,
        )
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / max(rate, 1.0), self.publish)

    def on_wrench(self, message):
        values = (
            message.wrench.force.x, message.wrench.force.y,
            message.wrench.force.z, message.wrench.torque.x,
            message.wrench.torque.y, message.wrench.torque.z,
        )
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error('Rejected non-finite wrench command')
            return
        if message.header.frame_id not in ('', 'base_link'):
            self.get_logger().error(
                'Rejected wrench outside base_link: ' +
                message.header.frame_id)
            return
        self.latest = values
        self.latest_ns = self.get_clock().now().nanoseconds

    def publish(self):
        now = self.get_clock().now()
        stale = self.latest_ns is None or (
            now.nanoseconds - self.latest_ns) / 1e9 > self.timeout
        wrench = [0.0] * 6 if stale else self.latest
        forces = allocate_wrench(
            self.matrix, wrench, self.lower_limits, self.upper_limits)
        achieved = [
            sum(row[index] * forces[index]
                for index in range(len(forces)))
            for row in self.matrix
        ]
        residual = [
            requested - actual
            for requested, actual in zip(wrench, achieved)
        ]
        epsilon = 1e-5
        saturated = [
            name for name, force, lower, upper in zip(
                self.names, forces, self.lower_limits, self.upper_limits)
            if abs(force - lower) <= epsilon or
            abs(force - upper) <= epsilon
        ]
        message = ThrusterForces()
        message.header.stamp = now.to_msg()
        message.header.frame_id = 'base_link'
        message.names = self.names
        message.forces = [float(force) for force in forces]
        self.publisher.publish(message)

        status = AllocationStatus()
        status.header = message.header
        status.requested_wrench = [float(value) for value in wrench]
        status.achieved_wrench = [float(value) for value in achieved]
        status.residual = [float(value) for value in residual]
        status.saturated_thrusters = saturated
        status.feasible = all(
            abs(value) <= self.feasible_tolerance for value in residual)
        self.status_publisher.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterAllocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
