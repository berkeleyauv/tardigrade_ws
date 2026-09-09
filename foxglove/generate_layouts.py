#!/usr/bin/env python3
"""Generate the small canonical Foxglove layout set.

Keep sim and pool layouts structurally aligned; only backend-specific camera and
service panels should differ. Run this file whenever a shared layout changes.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parent / "layouts"
STATUS = "tardigrade-tools.Vehicle Status!status"
ATTITUDE = "tardigrade-tools.Attitude!attitude"
PID = "tardigrade-tools.PID Tuner!pid"


def split(direction: str, first: Any, second: Any, percentage: float = 50) -> dict[str, Any]:
    return {
        "direction": direction,
        "first": first,
        "second": second,
        "splitPercentage": percentage,
    }


def document(config: dict[str, Any], layout: Any) -> dict[str, Any]:
    return {
        "configById": config,
        "globalVariables": {},
        "userNodes": {},
        "playbackConfig": {"speed": 1},
        "layout": layout,
    }


def image(topic: str, camera_info: str) -> dict[str, Any]:
    return {
        "cameraState": {},
        "followMode": "follow-pose",
        "scene": {},
        "transforms": {},
        "topics": {},
        "layers": {},
        "publish": {"type": "point"},
        "synchronize": False,
        "imageMode": {"imageTopic": topic, "calibrationTopic": camera_info},
    }


def vehicle_3d() -> dict[str, Any]:
    return {
        "cameraState": {
            "perspective": True,
            "distance": 6,
            "phi": 63,
            "thetaOffset": 120,
            "target": [0, 0, 0],
            "targetOffset": [0, 0, 0],
            "targetOrientation": [0, 0, 0, 1],
            "fovy": 45,
            "near": 0.1,
            "far": 5000,
        },
        "followMode": "follow-pose",
        "scene": {},
        "transforms": {
            "frame:base_link": {"visible": True},
            "frame:imu_link": {"visible": True},
            "frame:pressure_link": {"visible": True},
            "frame:zed_camera_link": {"visible": True},
        },
        "topics": {"/tardigrade/state/odometry/filtered": {"visible": True}},
        "layers": {
            "tardigrade-grid": {
                "visible": True,
                "frameLocked": True,
                "label": "Pool grid",
                "instanceId": "tardigrade-grid",
                "layerId": "foxglove.Grid",
                "size": 20,
                "divisions": 20,
                "lineWidth": 1,
                "color": "#248eff",
                "position": [0, 0, 0],
                "rotation": [0, 0, 0],
                "order": 1,
            }
        },
        "synchronize": False,
        "imageMode": {},
        "followTf": "base_link",
        "fixedFrame": "odom",
    }


def plot(title: str, series: list[tuple[str, str]], *, legend: bool = True) -> dict[str, Any]:
    return {
        "title": title,
        "paths": [
            {
                "value": path,
                "label": label,
                "enabled": True,
                "timestampMethod": "receiveTime",
            }
            for path, label in series
        ],
        "showLegend": legend,
        "legendDisplay": "top",
        "xAxisVal": "timestamp",
    }


def call(service: str, payload: str) -> dict[str, Any]:
    return {
        "requestPayload": payload,
        "layout": "vertical",
        "timeoutSeconds": 5,
        "serviceName": service,
    }


def operator(sim: bool) -> dict[str, Any]:
    prefix = "/tardigrade/sensors/camera/front" if sim else "/zed/zed_node"
    left_image = f"{prefix}/left/image_raw" if sim else f"{prefix}/left/image_rect_color"
    right_image = f"{prefix}/right/image_raw" if sim else f"{prefix}/right/image_rect_color"
    left_info = f"{prefix}/left/camera_info"
    right_info = f"{prefix}/right/camera_info"
    config: dict[str, Any] = {
        STATUS: {},
        ATTITUDE: {},
        "3D!vehicle": vehicle_3d(),
        "Image!left": image(left_image, left_info),
        "Image!right": image(right_image, right_info),
    }
    lower_right: Any = "Image!right"
    if sim:
        config["CallService!reset"] = call(
            "/tardigrade/sim/reset",
            '{"scenario_id":"clean","seed":42,"initial_pose":{"position":{"x":0.0,"y":0.0,"z":0.0},"orientation":{"w":1.0}}}',
        )
        lower_right = split("column", "Image!right", "CallService!reset", 78)
    layout = split(
        "row",
        STATUS,
        split(
            "column",
            split("row", "3D!vehicle", ATTITUDE, 58),
            split("row", "Image!left", lower_right, 50),
            58,
        ),
        28,
    )
    return document(config, layout)


def pid_tuning() -> dict[str, Any]:
    config = {PID: {"axis": "surge"}, STATUS: {}, ATTITUDE: {}}
    layout = split("row", PID, split("column", STATUS, ATTITUDE, 55), 72)
    return document(config, layout)


def sensors(sim: bool) -> dict[str, Any]:
    prefix = "/tardigrade/sensors/camera/front" if sim else "/zed/zed_node"
    left_image = f"{prefix}/left/image_raw" if sim else f"{prefix}/left/image_rect_color"
    right_image = f"{prefix}/right/image_raw" if sim else f"{prefix}/right/image_rect_color"
    config: dict[str, Any] = {
        STATUS: {},
        ATTITUDE: {},
        "3D!vehicle": vehicle_3d(),
        "Image!left": image(left_image, f"{prefix}/left/camera_info"),
        "Image!right": image(right_image, f"{prefix}/right/camera_info"),
    }
    if sim:
        config["Plot!depth"] = plot(
            "ENU vertical position: truth / VIO / EKF (m)",
            [
                ("/tardigrade/sim/ground_truth/odometry.pose.pose.position.z", "truth z"),
                ("/tardigrade/sensors/visual_odometry.pose.pose.position.z", "VIO z"),
                ("/tardigrade/state/odometry/filtered.pose.pose.position.z", "EKF z"),
            ],
        )
        config["Plot!pressure"] = plot(
            "Absolute pressure (Pa)",
            [("/tardigrade/sensors/pressure.fluid_pressure", "pressure")],
            legend=False,
        )
    else:
        config["Plot!depth"] = plot(
            "Vertical estimate comparison (m)",
            [
                ("/zed/zed_node/odom.pose.pose.position.z", "ZED z"),
                ("/tardigrade/state/odometry/filtered.pose.pose.position.z", "EKF z"),
            ],
        )
        config["Plot!pressure"] = plot(
            "ESP depth (m)", [("/tardigrade/esp/state.depth", "depth")], legend=False
        )
    layout = split(
        "column",
        split("row", "Image!left", "Image!right", 50),
        split(
            "row",
            split("column", "3D!vehicle", ATTITUDE, 52),
            split("column", STATUS, split("row", "Plot!depth", "Plot!pressure", 58), 54),
            52,
        ),
        44,
    )
    return document(config, layout)


def pool_checkout() -> dict[str, Any]:
    config = {
        STATUS: {},
        ATTITUDE: {},
        "3D!vehicle": vehicle_3d(),
        "Image!left": image(
            "/zed/zed_node/left/image_rect_color", "/zed/zed_node/left/camera_info"
        ),
        "CallService!thruster": call(
            "/tardigrade/test/run_thruster", '{"slot":1,"command":0.05,"duration_sec":1.0}'
        ),
    }
    layout = split(
        "row",
        split("column", STATUS, "CallService!thruster", 76),
        split(
            "column",
            split("row", "3D!vehicle", ATTITUDE, 56),
            "Image!left",
            58,
        ),
        31,
    )
    return document(config, layout)


def write(name: str, data: dict[str, Any]) -> None:
    (ROOT / name).write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


def main() -> None:
    shared_pid = pid_tuning()
    write("sim_operator.json", operator(sim=True))
    write("pool_operator.json", operator(sim=False))
    write("pid_tuning.json", shared_pid)
    write("sim_pid_tuning.json", shared_pid)  # compatibility filename
    write("sim_sensors.json", sensors(sim=True))
    write("pool_sensors.json", sensors(sim=False))
    write("pool_checkout.json", pool_checkout())


if __name__ == "__main__":
    main()
