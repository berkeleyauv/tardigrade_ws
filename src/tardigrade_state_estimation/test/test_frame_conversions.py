import math

import pytest

from tardigrade_state_estimation.frame_conversions import Q_ENU_FROM_NED
from tardigrade_state_estimation.frame_conversions import native_orientation_to_enu_base
from tardigrade_state_estimation.frame_conversions import native_vector_to_base
from tardigrade_state_estimation.frame_conversions import quaternion_inverse
from tardigrade_state_estimation.frame_conversions import quaternion_multiply


BACKWARD_DOWN_MOUNT = (0.0, 0.0, 1.0, 0.0)


@pytest.mark.parametrize(
    'sensor_vector, expected_base_vector',
    [
        ((1.0, 0.0, 0.0), (-1.0, 0.0, 0.0)),
        ((0.0, 1.0, 0.0), (0.0, 1.0, 0.0)),
        ((0.0, 0.0, 1.0), (0.0, 0.0, -1.0)),
    ],
)
def test_backward_down_mount_axes(sensor_vector, expected_base_vector):
    actual = native_vector_to_base(sensor_vector, BACKWARD_DOWN_MOUNT)
    assert actual == pytest.approx(expected_base_vector, abs=1e-12)


def test_level_robot_converts_to_identity_enu_orientation():
    # Construct the native sensor attitude for a level, ENU-aligned robot.
    q_ned_from_sensor = quaternion_multiply(
        quaternion_inverse(Q_ENU_FROM_NED),
        BACKWARD_DOWN_MOUNT,
    )
    actual = native_orientation_to_enu_base(
        q_ned_from_sensor, BACKWARD_DOWN_MOUNT
    )
    assert actual == pytest.approx((1.0, 0.0, 0.0, 0.0), abs=1e-12)


def test_nonfinite_or_zero_mount_is_rejected():
    with pytest.raises(ValueError):
        native_vector_to_base((1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        native_vector_to_base(
            (1.0, 0.0, 0.0), (math.nan, 0.0, 0.0, 1.0)
        )
