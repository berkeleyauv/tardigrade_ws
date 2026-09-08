import numpy as np
import pytest

from tardigrade_description.vehicle_model import load_vehicle_model
from tardigrade_control.actuator_math import (
    allocate_wrench,
    command_to_force,
    force_to_command,
    validate_named_values,
)


def test_thrust_curve_round_trip_and_asymmetry():
    thruster = load_vehicle_model().thrusters[0]
    for command in (-1.0, -0.4, 0.0, 0.4, 1.0):
        force = command_to_force(command, thruster)
        recovered = force_to_command(force, thruster)
        assert recovered == pytest.approx(command, abs=1e-8)
    assert command_to_force(1.0, thruster) == 50.0
    assert command_to_force(-1.0, thruster) == -40.0


def test_allocator_reconstructs_feasible_wrench():
    model = load_vehicle_model()
    matrix = np.asarray(model.allocation_matrix)
    requested = np.asarray([20.0, -5.0, 30.0, 1.0, -2.0, 3.0])
    forces = allocate_wrench(
        matrix,
        requested,
        [-item['max_reverse_n'] for item in model.thrusters],
        [item['max_forward_n'] for item in model.thrusters],
    )
    assert matrix @ forces == pytest.approx(requested, abs=1e-8)


def test_allocator_obeys_limits_for_infeasible_wrench():
    model = load_vehicle_model()
    forces = allocate_wrench(
        model.allocation_matrix,
        [10000.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [-item['max_reverse_n'] for item in model.thrusters],
        [item['max_forward_n'] for item in model.thrusters],
    )
    assert all(
        -thruster['max_reverse_n'] <= force <= thruster['max_forward_n']
        for force, thruster in zip(forces, model.thrusters)
    )


def test_named_array_contract_rejects_malformed_inputs():
    expected = ['a', 'b']
    assert validate_named_values(
        ['b', 'a'], [2.0, 1.0], expected) == [1.0, 2.0]
    with pytest.raises(ValueError, match='equal'):
        validate_named_values(['a'], [1.0, 2.0], expected)
    with pytest.raises(ValueError, match='duplicate'):
        validate_named_values(['a', 'a'], [1.0, 2.0], expected)
    with pytest.raises(ValueError, match='unknown'):
        validate_named_values(['a', 'c'], [1.0, 2.0], expected)
    with pytest.raises(ValueError, match='non-finite'):
        validate_named_values(['a', 'b'], [1.0, float('nan')], expected)
    with pytest.raises(ValueError, match='above'):
        validate_named_values(['a', 'b'], [1.0, 1.1], expected, -1.0, 1.0)
