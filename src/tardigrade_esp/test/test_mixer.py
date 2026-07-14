from pathlib import Path
import unittest

from geometry_msgs.msg import Twist

from tardigrade_esp.esp_thruster_bridge import EspThrusterBridge
from tardigrade_esp.esp_thruster_bridge import load_thruster_config


def configured_bridge():
    bridge = EspThrusterBridge.__new__(EspThrusterBridge)
    _, bridge.thrusters = load_thruster_config(
        str(Path(__file__).parents[1] / 'config' / 'esp_thruster_map.json')
    )
    bridge.neutral_us = 1500
    bridge.min_us = 1100
    bridge.max_us = 1900
    bridge.max_delta_us = 50
    return bridge


class MixerTest(unittest.TestCase):
    def test_positive_heave_drives_all_vertical_thrusters_together(self):
        cmd = Twist()
        cmd.linear.z = 0.2

        self.assertEqual(configured_bridge().mix_cmd_vel(cmd), [
            1500, 1510, 1500, 1510, 1500, 1510, 1510, 1500,
        ])

    def test_positive_roll_drives_left_and_right_oppositely(self):
        cmd = Twist()
        cmd.angular.x = 0.2

        self.assertEqual(configured_bridge().mix_cmd_vel(cmd), [
            1500, 1510, 1500, 1490, 1500, 1510, 1490, 1500,
        ])

    def test_positive_pitch_drives_front_and_back_oppositely(self):
        cmd = Twist()
        cmd.angular.y = 0.2

        self.assertEqual(configured_bridge().mix_cmd_vel(cmd), [
            1500, 1490, 1500, 1490, 1500, 1510, 1510, 1500,
        ])
