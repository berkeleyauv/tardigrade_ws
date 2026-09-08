from pathlib import Path

import json

from tardigrade_description.export_vehicle_config import generated_documents
from tardigrade_description.vehicle_model import default_vehicle_path


def test_generated_configs_are_current():
    package_root = Path(__file__).resolve().parents[1]
    ros_workspace = package_root.parents[1]
    unity_output = ros_workspace.parent / 'tardigrade_unity_world' / \
        'Assets' / 'StreamingAssets' / 'tardigrade_vehicle.json'
    allocator_output = package_root / 'config' / 'allocator.json'
    documents = generated_documents(default_vehicle_path())
    assert json.loads(allocator_output.read_text()) == documents['allocator']
    # The two repositories are checked out side-by-side in development. ROS CI
    # may intentionally check out only this repository.
    if unity_output.is_file():
        assert json.loads(unity_output.read_text()) == documents['unity']
