# Copyright 2026 Berkeley AUV
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import threading
import time
import unittest
from unittest.mock import patch

from tardigrade_esp import tardigrade_protocol as tp
from tardigrade_esp.esp_bridge import EspBridge
from tardigrade_esp.esp_bridge import validated_motor_values


class FakeLogger:
    def __init__(self):
        self.warnings = []

    def warn(self, message):
        self.warnings.append(message)


class FakeSerial:
    def __init__(self):
        self.writes = []

    def write(self, data):
        self.writes.append(data)


def bridge_for_watchdog(last_command_time):
    bridge = EspBridge.__new__(EspBridge)
    bridge._ser = FakeSerial()
    bridge._write_lock = threading.Lock()
    bridge._reported_armed = True
    bridge._cmd_timeout_sec = 0.5
    bridge._last_motor_cmd_monotonic = last_command_time
    bridge._watchdog_neutral_active = False
    return bridge


class BridgeSafetyTest(unittest.TestCase):
    def test_motor_command_requires_exactly_eight_values(self):
        with self.assertRaises(ValueError):
            validated_motor_values([0.0] * 7)
        with self.assertRaises(ValueError):
            validated_motor_values([0.0] * 9)

    def test_motor_command_rejects_non_finite_values(self):
        for value in (float('nan'), float('inf'), float('-inf')):
            with self.assertRaises(ValueError):
                validated_motor_values([value] + [0.0] * 7)

    def test_motor_command_is_clamped(self):
        self.assertEqual(
            validated_motor_values([-2.0, -1.0, -0.2, 0.0, 0.2, 1.0, 2.0, 0.5]),
            [-1.0, -1.0, -0.2, 0.0, 0.2, 1.0, 1.0, 0.5],
        )

    def test_stale_command_sends_eight_neutrals_then_heartbeat(self):
        bridge = bridge_for_watchdog(time.monotonic() - 1.0)
        logger = FakeLogger()

        with patch.object(
            EspBridge,
            'get_logger',
            return_value=logger,
            create=True,
        ):
            bridge._send_heartbeat_if_armed()

        expected = [tp.encode_set_motor(index, 0.0) for index in range(8)]
        expected.append(tp.encode(tp.HEARTBEAT))
        self.assertEqual(bridge._ser.writes, expected)
        self.assertTrue(bridge._watchdog_neutral_active)
        self.assertEqual(len(logger.warnings), 1)

    def test_fresh_command_sends_only_heartbeat(self):
        bridge = bridge_for_watchdog(time.monotonic())

        bridge._send_heartbeat_if_armed()

        self.assertEqual(bridge._ser.writes, [tp.encode(tp.HEARTBEAT)])

    def test_disarmed_bridge_sends_nothing(self):
        bridge = bridge_for_watchdog(None)
        bridge._reported_armed = False

        bridge._send_heartbeat_if_armed()

        self.assertEqual(bridge._ser.writes, [])
