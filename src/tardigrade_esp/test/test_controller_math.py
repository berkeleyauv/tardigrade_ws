import math
import unittest

from geometry_msgs.msg import Quaternion

from tardigrade_esp.depth_attitude_controller import clamp
from tardigrade_esp.depth_attitude_controller import roll_pitch_from_quaternion


class ControllerMathTest(unittest.TestCase):
    def test_clamp_is_symmetric(self):
        self.assertEqual(clamp(0.5, 0.25), 0.25)
        self.assertEqual(clamp(-0.5, 0.25), -0.25)

    def test_roll_pitch_from_identity_quaternion(self):
        q = Quaternion()
        q.w = 1.0

        roll, pitch = roll_pitch_from_quaternion(q)

        self.assertTrue(math.isclose(roll, 0.0))
        self.assertTrue(math.isclose(pitch, 0.0))
