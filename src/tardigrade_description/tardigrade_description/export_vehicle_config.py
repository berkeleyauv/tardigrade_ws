"""Export the canonical description into consumer-specific JSON files."""

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any, Dict, Optional, Sequence

from .vehicle_model import default_vehicle_path, load_vehicle_model


def source_hash(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def generated_documents(source: Path) -> Dict[str, Dict[str, Any]]:
    model = load_vehicle_model(str(source))
    digest = source_hash(source)
    generated = {
        'source': 'tardigrade_description/config/vehicle.json',
        'sha256': digest,
    }
    # Unity receives only runtime fields. Provenance and uncertainty remain in
    # the canonical source and the hash ties this projection back to them.
    unity = {
        'schema_version': model.data['schema_version'],
        'vehicle_id': model.data['vehicle_id'],
        'generated': generated,
        'rigid_body': {
            key: model.data['rigid_body'][key] for key in (
                'mass_kg', 'center_of_mass_m', 'center_of_buoyancy_m',
                'inertia_kg_m2', 'added_mass_kg', 'added_inertia_kg_m2',
                'displaced_volume_m3', 'collision_size_m')
        },
        'water': model.data['water'],
        'hydrodynamics': {
            key: model.data['hydrodynamics'][key]
            for key in ('linear_damping', 'quadratic_damping')
        },
        'buoyancy_elements': model.data['buoyancy_elements'],
        'thrusters': model.thrusters,
        'sensors': model.data['sensors'],
    }
    allocator = {
        'schema_version': 1,
        'generated': generated,
        'thruster_names': model.thruster_names,
        'allocation_matrix': model.allocation_matrix,
        'limits': [{
            'name': item['name'],
            'max_forward_n': item['max_forward_n'],
            'max_reverse_n': item['max_reverse_n'],
            'deadband': item['deadband'],
        } for item in model.thrusters],
    }
    return {'unity': unity, 'allocator': allocator}


def _serialized(document: Dict[str, Any]) -> str:
    return json.dumps(document, indent=2, sort_keys=True) + '\n'


def _write_or_check(path: Path, document: Dict[str, Any], check: bool) -> bool:
    expected = _serialized(document)
    if check:
        if not path.is_file():
            return False
        try:
            return json.loads(path.read_text(encoding='utf-8')) == document
        except json.JSONDecodeError:
            return False
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(expected, encoding='utf-8')
    return True


def run(source: Path, unity_output: Path, allocator_output: Path,
        check: bool = False) -> bool:
    documents = generated_documents(source)
    results = [
        _write_or_check(unity_output, documents['unity'], check),
        _write_or_check(allocator_output, documents['allocator'], check),
    ]
    return all(results)


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser()
    result.add_argument('--source', type=Path, default=default_vehicle_path())
    result.add_argument('--unity-output', type=Path, required=True)
    result.add_argument('--allocator-output', type=Path, required=True)
    result.add_argument('--check', action='store_true')
    return result


def main(argv: Optional[Sequence[str]] = None) -> None:
    args = parser().parse_args(argv)
    if not run(args.source, args.unity_output, args.allocator_output,
               args.check):
        raise SystemExit('generated vehicle configuration is stale')


def validate_main(argv: Optional[Sequence[str]] = None) -> None:
    args = argparse.ArgumentParser()
    args.add_argument('--source', type=Path, default=default_vehicle_path())
    parsed = args.parse_args(argv)
    model = load_vehicle_model(str(parsed.source))
    print(f'valid: {model.data["vehicle_id"]}; '
          f'{len(model.thrusters)} thrusters')


if __name__ == '__main__':
    main()
