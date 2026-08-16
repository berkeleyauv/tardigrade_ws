import math
import unittest

from tardigrade_teleop.xbox_cmd_vel import command_from_joy
from tardigrade_teleop.xbox_cmd_vel import joy_is_valid
from tardigrade_teleop.xbox_cmd_vel import shape_axis
from tardigrade_teleop.xbox_cmd_vel import timestamp_is_fresh


CONFIG = {
    'deadzone': 0.1,
    'deadman_button': 4,
    'surge_axis': 1,
    'sway_axis': 0,
    'heave_axis': 4,
    'yaw_axis': 3,
    'max_surge': 0.25,
    'max_sway': 0.25,
    'max_heave': 0.2,
    'max_yaw': 0.2,
}

FOXGLOVE_CONFIG = {
    **CONFIG,
    # Browser Gamepad API order after foxglove-joystick's axis inversion:
    # left X/Y, right X/Y.
    'heave_axis': 3,
    'yaw_axis': 2,
}


class XboxMappingTest(unittest.TestCase):
    def test_deadzone_is_rescaled(self):
        self.assertEqual(shape_axis(0.1, 0.1), 0.0)
        self.assertTrue(math.isclose(shape_axis(0.55, 0.1), 0.5))

    def test_released_deadman_forces_zero(self):
        command, active = command_from_joy(
            [1.0, 1.0, 0.0, 1.0, 1.0], [0] * 11, CONFIG)
        self.assertFalse(active)
        self.assertEqual(command.linear.x, 0.0)
        self.assertEqual(command.angular.z, 0.0)

    def test_standard_xbox_axes_and_signs(self):
        buttons = [0] * 11
        buttons[4] = 1
        command, active = command_from_joy(
            [-1.0, 1.0, 0.0, -0.5, 0.5], buttons, CONFIG)
        self.assertTrue(active)
        self.assertEqual(command.linear.x, 0.25)
        self.assertEqual(command.linear.y, -0.25)
        self.assertGreater(command.linear.z, 0.0)
        self.assertLess(command.angular.z, 0.0)

    def test_foxglove_browser_xbox_axes_and_signs(self):
        buttons = [0] * 17
        buttons[4] = 1
        command, active = command_from_joy(
            [1.0, 1.0, -0.5, 0.5], buttons, FOXGLOVE_CONFIG)
        self.assertTrue(active)
        self.assertEqual(command.linear.x, 0.25)
        self.assertEqual(command.linear.y, 0.25)
        self.assertGreater(command.linear.z, 0.0)
        self.assertLess(command.angular.z, 0.0)

    def test_missing_axes_fail_closed(self):
        buttons = [0] * 5
        buttons[4] = 1
        command, active = command_from_joy([], buttons, CONFIG)
        self.assertFalse(active)
        self.assertEqual(command.linear.x, 0.0)

    def test_nan_axis_fails_closed(self):
        axes = [0.0, math.nan, 0.0, 0.0, 0.0]
        self.assertFalse(joy_is_valid(axes, [0] * 11, CONFIG))

    def test_stale_timestamp_fails_closed(self):
        self.assertTrue(timestamp_is_fresh(1_000, 101_000, 0.001))
        self.assertFalse(
            timestamp_is_fresh(1_000, 300_001_000, 0.25))
        self.assertFalse(timestamp_is_fresh(None, 1_000, 0.25))


if __name__ == '__main__':
    unittest.main()
