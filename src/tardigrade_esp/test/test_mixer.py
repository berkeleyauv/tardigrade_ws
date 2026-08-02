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
    def test_root_and_packaged_maps_are_identical(self):
        package_map = Path(__file__).parents[1] / 'config' / 'esp_thruster_map.json'
        root_map = Path(__file__).parents[3] / 'config' / 'esp_thruster_map.json'

        self.assertEqual(package_map.read_bytes(), root_map.read_bytes())

    def test_positive_heave_drives_all_vertical_thrusters_together(self):
        cmd = Twist()
        cmd.linear.z = 0.2

        self.assertEqual(configured_bridge().mix_cmd_vel(cmd), [
            1510, 1490, 1500, 1500, 1500, 1490, 1500, 1510,
        ])

    def test_positive_roll_drives_left_and_right_oppositely(self):
        cmd = Twist()
        cmd.angular.x = 0.2

        self.assertEqual(configured_bridge().mix_cmd_vel(cmd), [
            1510, 1510, 1500, 1500, 1500, 1490, 1500, 1490,
        ])

    def test_positive_pitch_drives_front_and_back_oppositely(self):
        cmd = Twist()
        cmd.angular.y = 0.2

        self.assertEqual(configured_bridge().mix_cmd_vel(cmd), [
            1490, 1510, 1500, 1500, 1500, 1490, 1500, 1510,
        ])

    def test_positive_surge_drives_all_vectored_thrusters_together(self):
        cmd = Twist()
        cmd.linear.x = 0.2

        self.assertEqual(configured_bridge().mix_cmd_vel(cmd), [
            1500, 1500, 1510, 1510, 1510, 1500, 1490, 1500,
        ])

    def test_positive_sway_uses_outward_x_pattern(self):
        cmd = Twist()
        cmd.linear.y = 0.2

        self.assertEqual(configured_bridge().mix_cmd_vel(cmd), [
            1500, 1500, 1510, 1510, 1490, 1500, 1510, 1500,
        ])

    def test_positive_yaw_uses_opposing_corner_pairs(self):
        cmd = Twist()
        cmd.angular.z = 0.2

        self.assertEqual(configured_bridge().mix_cmd_vel(cmd), [
            1500, 1500, 1490, 1510, 1490, 1500, 1490, 1500,
        ])
