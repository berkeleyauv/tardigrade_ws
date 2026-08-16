import math
import unittest

from geometry_msgs.msg import Quaternion

from tardigrade_esp.depth_attitude_controller import DepthAttitudeController
from tardigrade_esp.depth_attitude_controller import clamp
from tardigrade_esp.depth_attitude_controller import angle_error
from tardigrade_esp.depth_attitude_controller import euler_from_quaternion
from tardigrade_esp.depth_attitude_controller import roll_pitch_from_quaternion
from tardigrade_esp.depth_attitude_controller import valid_gain_request


class ControllerMathTest(unittest.TestCase):
    def test_odometry_subscription_callback_exists(self):
        self.assertTrue(callable(DepthAttitudeController.odom_callback))

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

    def test_live_gain_validation_is_atomic_friendly(self):
        self.assertEqual(
            valid_gain_request('roll', 0.8, 0.0, 0.15, 0.2),
            (True, 'ok'),
        )
        self.assertFalse(
            valid_gain_request('roll', -0.1, 0.0, 0.0, 0.2)[0])
        self.assertFalse(
            valid_gain_request('bad-axis', 1.0, 0.0, 0.0, 0.2)[0])
        self.assertFalse(
            valid_gain_request('yaw', 1.0, 0.0, 0.0, 1.1)[0])
