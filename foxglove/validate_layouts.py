#!/usr/bin/env python3
"""Static checks for canonical Foxglove layouts and extension packaging."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parent
CANONICAL = (
    "sim_operator.json",
    "pool_operator.json",
    "pid_tuning.json",
    "sim_pid_tuning.json",
    "sim_sensors.json",
    "pool_sensors.json",
    "pool_checkout.json",
)
SHARED_PANELS = {
    "tardigrade-tools.Vehicle Status!status",
    "tardigrade-tools.Attitude!attitude",
}


def leaves(node: Any) -> list[str]:
    if isinstance(node, str):
        return [node]
    if not isinstance(node, dict) or "first" not in node or "second" not in node:
        raise AssertionError(f"Invalid layout node: {node!r}")
    if node.get("direction") not in {"row", "column"}:
        raise AssertionError(f"Invalid split direction: {node.get('direction')!r}")
    percentage = float(node.get("splitPercentage", 0))
    if not 0 < percentage < 100:
        raise AssertionError(f"Invalid split percentage: {percentage}")
    return leaves(node["first"]) + leaves(node["second"])


def load(name: str) -> dict[str, Any]:
    return json.loads((ROOT / "layouts" / name).read_text(encoding="utf-8"))


def main() -> None:
    loaded = {name: load(name) for name in CANONICAL}
    for name, data in loaded.items():
        panel_ids = leaves(data["layout"])
        configs = data["configById"]
        assert len(panel_ids) == len(set(panel_ids)), f"{name}: duplicate panel ID"
        assert set(panel_ids) == set(configs), f"{name}: layout/config panel mismatch"
        for panel_id, config in configs.items():
            if panel_id.startswith("Plot!"):
                assert len(config.get("paths", [])) <= 3, f"{name}: crowded plot {panel_id}"
                assert config.get("legendDisplay") != "floating", f"{name}: floating legend"

    for mode in ("operator", "sensors"):
        sim = loaded[f"sim_{mode}.json"]
        pool = loaded[f"pool_{mode}.json"]
        assert SHARED_PANELS <= set(sim["configById"]), f"sim_{mode}: missing shared panels"
        assert SHARED_PANELS <= set(pool["configById"]), f"pool_{mode}: missing shared panels"

    assert loaded["pid_tuning.json"] == loaded["sim_pid_tuning.json"], "PID alias is stale"
    assert "tardigrade-tools.PID Tuner!pid" in loaded["pid_tuning.json"]["configById"]

    package = json.loads(
        (ROOT / "extensions" / "tardigrade-tools" / "package.json").read_text(encoding="utf-8")
    )
    assert package["name"] == "tardigrade-tools"
    assert package["license"] == "Apache-2.0"
    print(f"Validated {len(loaded)} canonical Foxglove layouts and Tardigrade Tools metadata")


if __name__ == "__main__":
    main()
