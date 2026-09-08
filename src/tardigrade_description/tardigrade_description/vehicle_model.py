"""Validated access to the canonical Tardigrade vehicle description."""

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Dict, List, Optional


def _finite_vector(value: Any, length: int, field: str) -> List[float]:
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f'{field} must contain {length} values')
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f'{field} contains a non-finite value')
    return result


def _cross(a: List[float], b: List[float]) -> List[float]:
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


@dataclass(frozen=True)
class VehicleModel:
    """Canonical config plus derived actuator geometry."""

    data: Dict[str, Any]
    source_path: Path

    @property
    def thrusters(self) -> List[Dict[str, Any]]:
        return self.data['thrusters']

    @property
    def thruster_names(self) -> List[str]:
        return [thruster['name'] for thruster in self.thrusters]

    @property
    def allocation_matrix(self) -> List[List[float]]:
        """Return B for wrench = B @ per-thruster force."""
        columns = []
        for thruster in self.thrusters:
            axis = [float(value) for value in thruster['axis']]
            position = [float(value) for value in thruster['position_m']]
            columns.append(axis + _cross(position, axis))
        return [[column[row] for column in columns] for row in range(6)]

    def thruster(self, name: str) -> Dict[str, Any]:
        for thruster in self.thrusters:
            if thruster['name'] == name:
                return thruster
        raise KeyError(name)


def default_vehicle_path() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('tardigrade_description')) / \
            'config' / 'vehicle.json'
    except (ImportError, LookupError):
        return Path(__file__).resolve().parents[1] / 'config' / 'vehicle.json'


def validate_vehicle_data(data: Dict[str, Any]) -> None:
    if data.get('schema_version') != 1:
        raise ValueError('unsupported vehicle schema_version')
    body = data.get('rigid_body', {})
    if float(body.get('mass_kg', 0.0)) <= 0.0:
        raise ValueError('rigid_body.mass_kg must be positive')
    if float(body.get('displaced_volume_m3', 0.0)) <= 0.0:
        raise ValueError('rigid_body.displaced_volume_m3 must be positive')
    for key in ('center_of_mass_m', 'center_of_buoyancy_m',
                'inertia_kg_m2', 'added_mass_kg',
                'added_inertia_kg_m2', 'collision_size_m'):
        values = _finite_vector(body.get(key), 3, f'rigid_body.{key}')
        if key.endswith(('inertia_kg_m2', 'mass_kg', 'size_m')) and \
                any(value <= 0.0 for value in values):
            raise ValueError(f'rigid_body.{key} values must be positive')

    hydro = data.get('hydrodynamics', {})
    for key in ('linear_damping', 'quadratic_damping'):
        values = _finite_vector(hydro.get(key), 6, f'hydrodynamics.{key}')
        if any(value < 0.0 for value in values):
            raise ValueError(f'hydrodynamics.{key} cannot be negative')

    thrusters = data.get('thrusters')
    if not isinstance(thrusters, list) or not thrusters:
        raise ValueError('thrusters must be a non-empty array')
    names = [str(item.get('name', '')) for item in thrusters]
    slots = [int(item.get('slot', -1)) for item in thrusters]
    if any(not name for name in names) or len(set(names)) != len(names):
        raise ValueError('thruster names must be non-empty and unique')
    if len(set(slots)) != len(slots) or sorted(slots) != \
            list(range(1, len(slots) + 1)):
        raise ValueError('thruster slots must be unique and contiguous from 1')
    for index, thruster in enumerate(thrusters):
        prefix = f'thrusters[{index}]'
        _finite_vector(thruster.get('position_m'), 3, prefix + '.position_m')
        axis = _finite_vector(thruster.get('axis'), 3, prefix + '.axis')
        norm = math.sqrt(sum(value * value for value in axis))
        if abs(norm - 1.0) > 1e-5:
            raise ValueError(prefix + '.axis must be a unit vector')
        for key in ('max_forward_n', 'max_reverse_n', 'time_constant_s'):
            if float(thruster.get(key, 0.0)) <= 0.0:
                raise ValueError(prefix + f'.{key} must be positive')
        deadband = float(thruster.get('deadband', -1.0))
        if deadband < 0.0 or deadband >= 1.0:
            raise ValueError(prefix + '.deadband must be in [0, 1)')
        if float(thruster.get('command_delay_s', -1.0)) < 0.0:
            raise ValueError(prefix + '.command_delay_s cannot be negative')

    elements = data.get('buoyancy_elements', [])
    volume_sum = 0.0
    for index, element in enumerate(elements):
        _finite_vector(element.get('position_m'), 3,
                       f'buoyancy_elements[{index}].position_m')
        volume = float(element.get('volume_m3', 0.0))
        height = float(element.get('height_m', 0.0))
        if volume <= 0.0 or height <= 0.0:
            raise ValueError('buoyancy element volume and height must be positive')
        volume_sum += volume
    if not math.isclose(volume_sum, float(body['displaced_volume_m3']),
                        rel_tol=1e-5):
        raise ValueError('buoyancy element volumes do not equal displacement')


def load_vehicle_model(path: Optional[str] = None) -> VehicleModel:
    source = Path(path) if path else default_vehicle_path()
    with source.open(encoding='utf-8') as stream:
        data = json.load(stream)
    validate_vehicle_data(data)
    return VehicleModel(data=data, source_path=source)
