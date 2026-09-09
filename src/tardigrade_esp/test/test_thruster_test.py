import math
from types import SimpleNamespace
import unittest

from tardigrade_esp.thruster_test import ThrusterTest
from tardigrade_esp.thruster_test import command_message
from tardigrade_esp.thruster_test import command_values
from tardigrade_esp.thruster_test import validate_request


class FakeLogger:
    def warn(self, message):
        del message


class CheckoutHarness:
    max_command = 0.10
    max_duration = 2.0

    def __init__(self, active=False):
        self.command = command_values(None, 0.0)
        self.stop_at = None
        self._active = active
        self.published = []

    def active(self):
        return self._active

    def neutralize(self):
        self.command = command_values(None, 0.0)
        self.stop_at = None

    def publish(self):
        self.published.append(list(self.command))

    def get_logger(self):
        return FakeLogger()


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

    def test_repeated_request_is_rejected_and_marked_for_neutral(self):
        self.assertEqual(
            validate_request(1, 0.05, 1.0, 0.10, 2.0, test_active=True),
            'another thruster test was active; command neutralized',
        )

    def test_command_selects_exactly_one_physical_slot(self):
        values = command_values(5, -0.08)
        self.assertEqual(len(values), 8)
        self.assertEqual(values[4], -0.08)
        self.assertEqual(sum(value != 0.0 for value in values), 1)

    def test_named_command_contract_is_preserved(self):
        names = [f'thruster_{index}' for index in range(1, 9)]
        message = command_message(names, command_values(2, 0.05))
        self.assertEqual(message.header.frame_id, 'base_link')
        self.assertEqual(list(message.names), names)
        self.assertEqual(len(message.setpoints), 8)
        self.assertAlmostEqual(message.setpoints[1], 0.05)
        self.assertEqual(sum(value != 0.0 for value in message.setpoints), 1)

    def test_accepted_request_publishes_neutral_before_one_slot(self):
        node = CheckoutHarness()
        request = SimpleNamespace(slot=4, command=0.05, duration_sec=1.0)
        response = SimpleNamespace(success=False, message='')

        result = ThrusterTest.run_thruster(node, request, response)

        self.assertTrue(result.success)
        self.assertEqual(node.published[0], [0.0] * 8)
        self.assertEqual(node.published[1], command_values(4, 0.05))

    def test_overlapping_request_neutralizes_without_new_motion(self):
        node = CheckoutHarness(active=True)
        request = SimpleNamespace(slot=4, command=0.05, duration_sec=1.0)
        response = SimpleNamespace(success=True, message='')

        result = ThrusterTest.run_thruster(node, request, response)

        self.assertFalse(result.success)
        self.assertEqual(node.published, [[0.0] * 8])
