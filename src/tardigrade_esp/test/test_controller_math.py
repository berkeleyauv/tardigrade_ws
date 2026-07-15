import math
import unittest

from geometry_msgs.msg import Quaternion

from tardigrade_esp.depth_attitude_controller import clamp
from tardigrade_esp.depth_attitude_controller import angle_error
from tardigrade_esp.depth_attitude_controller import euler_from_quaternion
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

    def test_euler_yaw(self):
        q = Quaternion()
        q.w = math.cos(math.pi / 4.0)
        q.z = math.sin(math.pi / 4.0)

        roll, pitch, yaw = euler_from_quaternion(q)

        self.assertTrue(math.isclose(roll, 0.0, abs_tol=1e-9))
        self.assertTrue(math.isclose(pitch, 0.0, abs_tol=1e-9))
        self.assertTrue(math.isclose(yaw, math.pi / 2.0, abs_tol=1e-9))

    def test_angle_error_wraps_at_pi(self):
        error = angle_error(math.radians(-179.0), math.radians(179.0))
        self.assertTrue(math.isclose(error, math.radians(2.0), abs_tol=1e-9))
