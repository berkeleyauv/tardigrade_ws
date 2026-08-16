import math
import unittest

from tardigrade_esp.thruster_test import validate_request


class ThrusterRequestTest(unittest.TestCase):
    def test_accepts_bounded_request(self):
        self.assertIsNone(validate_request(1, 0.05, 1.0, 0.10, 2.0))
        self.assertIsNone(validate_request(8, -0.10, 2.0, 0.10, 2.0))

    def test_rejects_invalid_slot(self):
        self.assertIsNotNone(validate_request(0, 0.05, 1.0, 0.10, 2.0))
        self.assertIsNotNone(validate_request(9, 0.05, 1.0, 0.10, 2.0))

    def test_rejects_excess_authority_or_duration(self):
        self.assertIsNotNone(validate_request(1, 0.11, 1.0, 0.10, 2.0))
        self.assertIsNotNone(validate_request(1, 0.05, 2.1, 0.10, 2.0))
        self.assertIsNotNone(validate_request(1, 0.05, 0.0, 0.10, 2.0))

    def test_rejects_nonfinite_values(self):
        self.assertIsNotNone(
            validate_request(1, math.nan, 1.0, 0.10, 2.0)
        )
        self.assertIsNotNone(
            validate_request(1, 0.05, math.inf, 0.10, 2.0)
        )
