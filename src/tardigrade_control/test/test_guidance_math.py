import math

import pytest

from tardigrade_control.pose_velocity_controller import clamp_axes
from tardigrade_control.pose_velocity_controller import (
    normalize_quaternion)
from tardigrade_control.pose_velocity_controller import (
    orientation_error_body)
from tardigrade_control.pose_velocity_controller import rotate_world_to_body


def test_world_error_is_rotated_into_body_flu():
    yaw_90 = normalize_quaternion((
        math.cos(math.pi / 4.0),
        0.0,
        0.0,
        math.sin(math.pi / 4.0),
    ))
    body = rotate_world_to_body((1.0, 0.0, 0.0), yaw_90)
    assert body == pytest.approx((0.0, -1.0, 0.0), abs=1e-9)


def test_orientation_error_uses_shortest_body_rotation():
    identity = (1.0, 0.0, 0.0, 0.0)
    target = normalize_quaternion((
        math.cos(math.pi / 4.0),
        0.0,
        0.0,
        math.sin(math.pi / 4.0),
    ))
    error = orientation_error_body(identity, target)
    assert error == pytest.approx((0.0, 0.0, math.pi / 2.0))
    same_rotation = tuple(-value for value in target)
    assert orientation_error_body(identity, same_rotation) == \
        pytest.approx(error)


def test_axis_limits_are_independent_and_symmetric():
    assert clamp_axes((2.0, -3.0, 0.2), (1.0, 2.0, 0.5)) == \
        (1.0, -2.0, 0.2)


def test_zero_quaternion_is_rejected():
    with pytest.raises(ValueError):
        normalize_quaternion((0.0, 0.0, 0.0, 0.0))
