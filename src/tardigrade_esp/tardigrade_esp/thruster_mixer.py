#!/usr/bin/env python3
"""thruster_mixer — body-frame wrench -> 8 normalized thruster commands.

The mixing half of the old esp_thruster_bridge, split out so the OUTPUT is a
plain ROS topic instead of serial bytes. That split is the sim/real seam from
docs/jetson_control_architecture.md:

  depth_attitude_controller -> /tardigrade/cmd_vel (Twist wrench)
        thruster_mixer      -> /tardigrade/thrusters/cmd (Float32MultiArray[8])
              esp_bridge (real)  OR  sim backend (sim)   consumes it

Geometry comes from the same esp_thruster_map.json the old bridge used — that
file stays the single source of truth for which thruster contributes to which
axis. Output is normalized -1..+1 per thruster; converting to PWM (and capping
authority) is the consumer's job. On the real robot the ESP's Safety layer
independently clamps magnitude, so a mixer bug cannot command full thrust.

Publishes neutral (all zeros) when the wrench goes stale, so a dead controller
upstream coasts the thrusters instead of freezing the last command.
"""

import json
import os

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from ament_index_python.packages import get_package_share_directory


def clamp(value, low, high):
    return max(low, min(high, value))


class ThrusterMixer(Node):
    def __init__(self, **node_kwargs):
        super().__init__('thruster_mixer', **node_kwargs)

        default_config = os.path.join(
            get_package_share_directory('tardigrade_esp'),
            'config',
            'esp_thruster_map.json',
        )
        self.declare_parameter('wrench_topic', '/tardigrade/cmd_vel')
        self.declare_parameter('output_topic', '/tardigrade/thrusters/cmd')
        self.declare_parameter('config_file', default_config)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('cmd_timeout_sec', 0.5)

        wrench_topic = self.get_parameter('wrench_topic').value
        output_topic = self.get_parameter('output_topic').value
        config_file = self.get_parameter('config_file').value
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.cmd_timeout_sec = float(
            self.get_parameter('cmd_timeout_sec').value)

        with open(config_file, 'r', encoding='utf-8') as f:
            data = json.load(f)
        self.mix = []
        for entry in data.get('thrusters', []):
            m = entry.get('mix', {})
            self.mix.append((
                float(m.get('surge', 0.0)), float(m.get('sway', 0.0)),
                float(m.get('heave', 0.0)), float(m.get('roll', 0.0)),
                float(m.get('pitch', 0.0)), float(m.get('yaw', 0.0)),
            ))
        if not self.mix:
            raise ValueError(f'No thrusters configured in {config_file}')

        self.latest = Twist()
        self.latest_ns = None

        self.sub = self.create_subscription(
            Twist, wrench_topic, self.on_wrench, 10)
        self.pub = self.create_publisher(Float32MultiArray, output_topic, 10)
        self.timer = self.create_timer(1.0 / max(rate, 1.0), self.publish_mix)

        self.get_logger().info(
            f'{wrench_topic} -> {output_topic} '
            f'({len(self.mix)} thrusters from '
            f'{os.path.basename(config_file)})')

    def on_wrench(self, msg):
        self.latest = msg
        self.latest_ns = self.get_clock().now().nanoseconds

    def stale(self):
        if self.latest_ns is None:
            return True
        age = (self.get_clock().now().nanoseconds - self.latest_ns) / 1e9
        return age > self.cmd_timeout_sec

    def publish_mix(self):
        out = Float32MultiArray()
        if self.stale():
            out.data = [0.0] * len(self.mix)
        else:
            w = self.latest
            out.data = [
                clamp(
                    surge * w.linear.x + sway * w.linear.y + heave * w.linear.z
                    + roll * w.angular.x + pitch * w.angular.y
                    + yaw * w.angular.z,
                    -1.0, 1.0,
                )
                for surge, sway, heave, roll, pitch, yaw in self.mix
            ]
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterMixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
