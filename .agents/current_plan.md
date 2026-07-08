# Current Development Plan

Electrical hardware access is currently blocked, so local development is the
active path. Preserve the Jetson/Pixhawk bringup knowledge, but do not make new
work depend on the ZED, VectorNav, Pixhawk, or thrusters being available.

## Active Goal

Make the software easy to clone, build, test, and extend locally:

1. Keep Docker startup simple on a laptop.
2. Keep the ROS workspace building without hardware.
3. Keep mock bringup useful for interface-level development.
4. Improve PX4/MAVLink code through tests and isolated local checks.
5. Keep the Jetson/ZED/Pixhawk runbook accurate for the next hardware session.
6. Keep real-thrust work blocked behind documented safety and mapping checks.

## Local Startup

From the repo root:

```bash
docker compose -f docker/compose.yaml run --rm tardigrade
```

Inside the container:

```bash
cd /ws
colcon build --symlink-install --packages-skip zed_components zed_wrapper zed_ros2
source install/setup.bash
ros2 launch tardigrade_bringup mock.launch.py
```

Use `docker/compose.yaml` alone for MacBook/local work. Use
`docker/compose.jetson.yaml` only on the Jetson hardware bench.

The ZED wrapper source is present in local clones, but the SDK-backed packages
`zed_components`, `zed_wrapper`, and `zed_ros2` should be skipped on machines
without the Stereolabs ZED SDK.

## Known Hardware State

The latest hardware session verified:

- ZED pose can publish on `/zed/zed_node/pose`.
- VectorNav connects at `115200` baud and publishes `/vectornav/imu`.
- `zed_vectornav_odometry` publishes `/tardigrade/state/odometry`.
- `mavlink_pixhawk_interface` connects to PX4 over `/dev/ttyACM0`.
- MAVLink visual odometry reaches PX4.
- PX4 reports local position and estimator diagnostics.

Not yet proven repeatable:

- PX4 remaining in Offboard mode.
- Arming from ROS on the current hardware setup.
- Real-thrust teleop.
- Thruster map validation against the physical vehicle.

## Next Useful Local Work

- Add tests around MAVLink command construction and frame conversions.
- Make Offboard enable delay explicit: start setpoint streaming, wait, then send
  the mode command.
- Keep `/tardigrade/status.detail` readable enough to debug without
  QGroundControl.
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
