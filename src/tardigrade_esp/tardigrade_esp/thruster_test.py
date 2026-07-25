#!/usr/bin/env python3
"""thruster_test — per-thruster manual command surface for bench testing.

Exposes eight ROS parameters, thruster_0 .. thruster_7, each a normalized
command in [-1, +1]. Foxglove's Parameters panel renders them as eight labeled,
individually editable fields (a slider/box per thruster). Every edit — and a
steady timer — publishes the current eight values as a std_msgs/Float32MultiArray
on /tardigrade/thrusters/cmd, the same topic thruster_mixer outputs and the ESP
bridge will eventually consume (F2).

No mixer, no PID, no odometry, no hardware. This is the "spin thruster N and
watch it" surface: on the real robot the values drive ESCs (still capped by the
ESP's independent Safety clamp); with no ESP attached you watch the published
array move so the control surface itself can be verified first.

Run:
  ros2 run tardigrade_esp thruster_test
Then open Foxglove's Parameters panel on the thruster_test node.
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32MultiArray

NUM_THRUSTERS = 8


def clamp(value, low=-1.0, high=1.0):
    return max(low, min(high, value))


class ThrusterTest(Node):
    def __init__(self):
        super().__init__('thruster_test')

        self.declare_parameter('output_topic', '/tardigrade/thrusters/cmd')
        self.declare_parameter('publish_rate_hz', 20.0)
        for i in range(NUM_THRUSTERS):
            self.declare_parameter(f'thruster_{i}', 0.0)

        output_topic = self.get_parameter('output_topic').value
        rate = float(self.get_parameter('publish_rate_hz').value)

        self._pub = self.create_publisher(Float32MultiArray, output_topic, 10)
        self._timer = self.create_timer(1.0 / max(1.0, rate), self._publish)

        # Publish immediately on any thruster_* edit so the response feels live
        # in Foxglove, not just at the timer rate. Values are clamped but the
        # stored parameter is left as-typed (rejecting it would fight the panel).
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info(
            f'thruster_test: {NUM_THRUSTERS} per-thruster params '
            f'-> {output_topic} at {rate:.0f} Hz (no hardware). '
            f'Edit thruster_0..{NUM_THRUSTERS - 1} in Foxglove Parameters panel.')

    def _current(self):
        return [
            clamp(float(self.get_parameter(f'thruster_{i}').value))
            for i in range(NUM_THRUSTERS)
        ]

    def _publish(self):
        msg = Float32MultiArray()
        msg.data = self._current()
        self._pub.publish(msg)

    def _on_set_parameters(self, params):
        # Accept the write first, then publish the post-write state. Reading the
        # incoming params here (rather than get_parameter, which still holds the
        # old value) makes the edit take effect on this same publish.
        pending = {p.name: p for p in params}
        data = []
        for i in range(NUM_THRUSTERS):
            name = f'thruster_{i}'
            if name in pending:
                data.append(clamp(float(pending[name].value)))
            else:
                data.append(clamp(float(self.get_parameter(name).value)))
        msg = Float32MultiArray()
        msg.data = data
        self._pub.publish(msg)
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
