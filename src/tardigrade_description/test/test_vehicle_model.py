import json

import numpy as np
import pytest

from tardigrade_description.vehicle_model import (
    load_vehicle_model,
    validate_vehicle_data,
)


def test_vehicle_has_full_rank_six_dof_allocation():
    model = load_vehicle_model()
    matrix = np.asarray(model.allocation_matrix)

    assert matrix.shape == (6, 8)
    assert np.linalg.matrix_rank(matrix) == 6
    assert model.thruster_names == [
        thruster['name'] for thruster in sorted(
            model.thrusters, key=lambda item: item['slot'])
    ]


def test_invalid_duplicate_thruster_name_is_rejected():
    model = load_vehicle_model()
    data = json.loads(json.dumps(model.data))
    data['thrusters'][1]['name'] = data['thrusters'][0]['name']

    with pytest.raises(ValueError, match='unique'):
        validate_vehicle_data(data)


def test_horizontal_axes_generate_expected_yaw_signs():
    model = load_vehicle_model()
    yaw_row = model.allocation_matrix[5]
    by_name = dict(zip(model.thruster_names, yaw_row))

    assert by_name['front_left_horizontal'] < 0.0
    assert by_name['rear_right_horizontal'] > 0.0
    assert by_name['front_right_horizontal'] > 0.0
    assert by_name['rear_left_horizontal'] < 0.0
