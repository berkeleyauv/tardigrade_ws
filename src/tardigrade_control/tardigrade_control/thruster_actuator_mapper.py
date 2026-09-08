"""Convert named forces in newtons to normalized actuator setpoints."""

import rclpy
from rclpy.node import Node

from tardigrade_description.vehicle_model import load_vehicle_model
from tardigrade_interfaces.msg import ThrusterCommands, ThrusterForces

from .actuator_math import force_to_command, validate_named_values


class ThrusterActuatorMapper(Node):
    def __init__(self, **node_kwargs):
        super().__init__('thruster_actuator_mapper', **node_kwargs)
        self.declare_parameter(
            'input_topic', '/tardigrade/actuators/thruster_forces')
        self.declare_parameter(
            'output_topic', '/tardigrade/actuators/thruster_commands')
        self.declare_parameter('vehicle_config', '')
        self.declare_parameter('voltage_scale', 1.0)
        self.declare_parameter('command_timeout_sec', 0.25)

        path = str(self.get_parameter('vehicle_config').value) or None
        self.model = load_vehicle_model(path)
        self.names = self.model.thruster_names
        self.voltage_scale = float(
            self.get_parameter('voltage_scale').value)
        self.timeout = float(
            self.get_parameter('command_timeout_sec').value)
        self.latest = [0.0] * len(self.names)
        self.latest_ns = None

        self.subscriber = self.create_subscription(
            ThrusterForces,
            str(self.get_parameter('input_topic').value),
            self.on_forces,
            10,
        )
        self.publisher = self.create_publisher(
            ThrusterCommands,
            str(self.get_parameter('output_topic').value),
            10,
        )
        self.timer = self.create_timer(0.02, self.publish)

    def on_forces(self, message):
        try:
            self.latest = validate_named_values(
                message.names, message.forces, self.names)
        except ValueError as error:
            self.get_logger().error(f'Rejected thruster forces: {error}')
            return
        self.latest_ns = self.get_clock().now().nanoseconds

    def publish(self):
        now = self.get_clock().now()
        stale = self.latest_ns is None or (
            now.nanoseconds - self.latest_ns) / 1e9 > self.timeout
        forces = [0.0] * len(self.names) if stale else self.latest
        message = ThrusterCommands()
        message.header.stamp = now.to_msg()
        message.header.frame_id = 'base_link'
        message.names = self.names
        message.setpoints = [
            force_to_command(force, thruster, self.voltage_scale)
            for force, thruster in zip(forces, self.model.thrusters)
        ]
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterActuatorMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
