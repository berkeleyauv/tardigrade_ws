import unittest

from tardigrade_control.velocity_wrench_controller import (
    antiwindup_integral_rate)
from tardigrade_control.velocity_wrench_controller import clamp
from tardigrade_control.velocity_wrench_controller import (
    valid_velocity_gain_request)
from tardigrade_control.velocity_wrench_controller import (
    VelocityWrenchController)


class ControllerMathTest(unittest.TestCase):
    def test_allocator_shortfall_back_calculates_integrator(self):
        rate = antiwindup_integral_rate(
            error=1.0,
            ki=2.0,
            gain=1.0,
            requested=10.0,
            achieved=4.0,
        )
        self.assertEqual(rate, -2.0)

    def test_velocity_gain_validation_uses_physical_output_units(self):
        self.assertEqual(
            valid_velocity_gain_request(
                'heave', 65.0, 5.0, 5.0, 20.0, 120.0),
            (True, 'ok'),
        )
        self.assertFalse(
            valid_velocity_gain_request(
                'bad-axis', 1.0, 0.0, 0.0, 1.0, 10.0)[0])
        self.assertFalse(
            valid_velocity_gain_request(
                'yaw', -1.0, 0.0, 0.0, 1.0, 10.0)[0])
        self.assertFalse(
            valid_velocity_gain_request(
                'surge', 1.0, 0.0, 0.0, 1.0, 0.0)[0])

    def test_odometry_subscription_callback_exists(self):
        self.assertTrue(callable(VelocityWrenchController.on_odometry))

    def test_clamp_is_symmetric(self):
        self.assertEqual(clamp(0.5, 0.25), 0.25)
        self.assertEqual(clamp(-0.5, 0.25), -0.25)
