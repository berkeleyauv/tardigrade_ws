# Current Development Plan

The physical robot is not in a stable state: PDB behavior, thruster behavior,
and wiring are not trusted. Local development is the active path. Preserve the
Jetson/Pixhawk bringup knowledge, but do not make new work depend on the ZED,
VectorNav, Pixhawk, or thrusters being available.

## Active Goal

Make the software observable, easy to build, and testable without the robot:

1. Keep Docker startup simple on a laptop and on the Jetson.
2. Keep the ROS workspace building without hardware.
3. Make mock/fake bringup useful for interface-level development.
4. Build a Foxglove pool-test UI instead of a custom webapp.
5. Add fake perception/status/odometry so autonomy can be tested locally.
6. Keep the Jetson/ZED/Pixhawk runbook accurate for the next hardware session.
7. Keep real-thrust work blocked behind documented safety and mapping checks.

## Local Startup

From the repo root:

```bash
./docker-build.sh
```

Inside the container:

```bash
cd /ws
./build.sh
source install/setup.bash
ros2 launch tardigrade_bringup mock.launch.py
```

Use `docker/compose.yaml` alone for MacBook/local work. Use
`docker/compose.jetson.yaml` only on the Jetson hardware bench. Prefer
`./docker-build.sh --jetson` over spelling out the Compose command directly.

The ZED wrapper source is present in local clones, but the SDK-backed packages
`zed_components`, `zed_wrapper`, and `zed_ros2` should be skipped on machines
without the Stereolabs ZED SDK.

## Docker / Script Workflow

Human-facing scripts live at the repo root:

- `docker-build.sh`: starts the container. Use `--build`, `--rebuild`,
  `--jetson`, or `--detached` as needed.
- `build.sh`: builds the ROS workspace. Use `--clean`, `--hardware`,
  `--pkg PACKAGE`, or `--debug` as needed.

Docker internals live under `docker/`:

- `docker/compose.yaml`: base local-development container.
- `docker/compose.jetson.yaml`: Jetson hardware override.
- `docker/Dockerfile`: image definition.
- `docker/ros_entrypoint.sh`: runs when the container starts.
- `docker/ros_bashrc.sh`: runs for interactive shells and defines aliases.
- `docker/run_jetson_hardware.sh`: raw `docker run` fallback when Compose is
  unavailable on the Jetson.

Do not restore `docker/run_jetson_zed.sh`; it was renamed/replaced by
`docker/run_jetson_hardware.sh`.

## Known Hardware State

Recent hardware sessions verified:

- ZED pose can publish on `/zed/zed_node/pose`.
- VectorNav connects at `115200` baud and publishes `/vectornav/imu`.
- `zed_vectornav_odometry` publishes `/tardigrade/state/odometry`.
- `mavlink_pixhawk_interface` connects to PX4 over `/dev/ttyACM0`.
- MAVLink visual odometry reaches PX4.
- PX4 reports local position and estimator diagnostics.
- The Pixhawk can arm when it receives acceptable ZED visual odometry.

Not yet proven repeatable:

- PX4 remaining in Offboard mode.
- Real-thrust teleop.
- Thruster map validation against the physical vehicle.
- End-to-end pool task execution.

## Roadmap

Current priority order:

1. Foxglove observability and pool-test UI.
2. Fake ROS inputs for mission logic: status, odometry, gate/slalom detections,
   and a fake controller.
3. BehaviorTree.CPP or behavior-tree-shaped autonomy for gate first.
4. Lightweight simulation/fake world before full Gazebo/PX4 SITL.
5. Sensor-frame cleanup, calibration notes, and eventual fusion with ZED, IMU,
   and depth.

Foxglove is the intended UI path. Do not start a custom webapp unless the team
explicitly decides Foxglove cannot satisfy a concrete requirement. The right
work is to publish good ROS topics, debug images, metrics, and layouts.

## Next Useful Local Work

- Add a `foxglove/` or `config/foxglove/` directory with a committed layout.
- Add/run `foxglove_bridge` in the container once the Foxy dependency path is
  confirmed.
- Standardize perception debug topics for gate/slalom detections and overlay
  images.
- Add fake ROS nodes for status, odometry, and perception.
- Add tests around MAVLink command construction and frame conversions when
  working in `tardigrade_px4`.
- Keep `/tardigrade/status.detail` readable enough to debug without QGC.
- Keep `README.md` short and keep the detailed hardware sequence in
  `docs/jetson_zed_px4_startup.md`.
- Keep `.legacy_inspect` removed; it was stale repository metadata.

## Guardrails

- Do not move Jetson-only mounts into `docker/compose.yaml`.
- Do not add a top-level `src/zed-ros2-interfaces`; it already exists as a
  nested submodule inside `src/zed-ros2-wrapper`.
- Do not re-center Micro XRCE-DDS unless the team explicitly revives that path.
- Keep PX4-specific message/protocol details inside `tardigrade_px4`.
- Keep robot-level APIs centered on `/tardigrade/*`.
- Never hide arming or external-control enable inside launch files.
- Do not build a custom dashboard before proving Foxglove is insufficient.
