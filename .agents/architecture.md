# Tardigrade v2 Architecture

## Purpose

Tardigrade v2 is a clean ROS 2 Foxy rebuild of the Berkeley AUV software
stack. The goal is to let future team members write robot behavior in robot
concepts without needing to understand PX4 internals.

The core design rule is:

> Robot code talks in robot concepts. Only driver/adapter code talks in
> hardware concepts.

The ESP32 serial PWM path is the current actuator backend. PX4/Pixhawk code is
legacy and should not define the whole codebase. The architecture should allow
different controllers without rewriting teleop, autonomy, perception, or
state-estimation code.

## Current Status Snapshot

As of July 13, 2026:

- The Pixhawk/PX4 path is retired for now and preserved under `src/legacy/`.
- `src/legacy/COLCON_IGNORE` keeps `px4_msgs` and `tardigrade_px4` out of
  normal builds.
- Local development is currently the safest path because the physical robot has
  PDB, wiring, and thruster reliability issues.
- The Jetson/ZED/VectorNav/ESP bringup path is the active hardware direction.
- Verified pieces from the hardware session:
  - ZED publishes pose,
  - VectorNav connects at 115200 baud and publishes IMU data,
  - `zed_vectornav_odometry` publishes `/tardigrade/state/odometry`,
  - `zed_vectornav_ekf.launch.py` provides an experimental EKF output,
  - `tardigrade_esp` owns the ESP serial PWM bridge,
  - `tardigrade_teleop` owns keyboard velocity commands.
- ZED/ESP bringup support is documented in `docs/esp_thruster_bringup.md`.
- ZED wrapper source is now tracked as a top-level submodule pinned to the
  `humble-v4.0.8` tag commit:
  - `src/zed-ros2-wrapper`
  The wrapper's own nested submodule provides:
  - `src/zed-ros2-wrapper/zed-ros2-interfaces`
- Docker startup is split into a local-safe base Compose file and a Jetson
  hardware override:
  - `docker/compose.yaml`
  - `docker/compose.jetson.yaml`
- Root helper scripts now wrap the common workflows:
  - `docker-build.sh`: build/start local or Jetson containers,
  - `build.sh`: build the ROS workspace locally or on hardware.
- The ZED camera is the local position source.
- The VectorNav IMU is the attitude and angular-velocity source.
- `tardigrade_state_estimation` publishes `/tardigrade/state/odometry`.
- The experimental EKF publishes `/tardigrade/state/odometry/filtered`.
- `keyboard_cmd_vel` provides a first keyboard teleop path through
  `/tardigrade/cmd_vel`.
- `config/thruster_map.yaml` documents actual thruster wiring and geometry, but
  it is not consumed by code yet.

## Package Boundaries

### `tardigrade_teleop`

Owns operator input tools that publish robot-level commands.

Current executables:

- `keyboard_cmd_vel`: publishes `geometry_msgs/Twist` on `/tardigrade/cmd_vel`
  by default.

### Legacy `px4_msgs`

Vendored PX4 ROS 2 messages. This package exists for the retired/direct PX4 ROS
2 topic path and is skipped by `./build.sh` by default.

### `vectornav` and `vectornav_msgs`

External VectorNav driver packages. The current hardware launch uses the
VectorNav driver directly and expects the IMU on:

```text
/vectornav/imu
```

The VectorNav is not a local-position source by itself. It provides attitude,
angular velocity, acceleration, and related IMU data.

Physical mounting convention: VectorNav FRD.

```text
VectorNav +X  robot forward
VectorNav +Y  robot right
VectorNav +Z  robot down
```

ROS `base_link` stays FLU. The EKF launch publishes the default
`base_link -> vectornav` static transform as a 180 degree rotation about X
using quaternion `(x=1, y=0, z=0, w=0)`.

### ZED wrapper packages

The Stereolabs ZED ROS 2 wrapper is used as an external dependency on the
Jetson. The current known working direction is documented in
`docs/esp_thruster_bringup.md`.

The wrapper is a Git submodule, not a manually cloned source directory:

```text
src/zed-ros2-wrapper
```

It is pinned by exact submodule commit corresponding to the `humble-v4.0.8` tag.
The wrapper contains its own nested `zed-ros2-interfaces` submodule, so a
recursive clone or `git submodule update --init --recursive` should fetch:

```text
src/zed-ros2-wrapper/zed-ros2-interfaces
```

Do not add or keep a duplicate top-level `src/zed-ros2-interfaces`; it causes
Colcon duplicate-package errors.

The main pose topic is:

```text
/zed/zed_node/pose
```

This is the current local position source.

### `tardigrade_interfaces`

Owns robot-level messages and services. This is the public API between
subsystems.

Current important interfaces:

- `/tardigrade/set_armed`
- `/tardigrade/set_external_control`
- `/tardigrade/status`

This package must not depend on `px4_msgs`.

### `tardigrade_state_estimation`

Consumes sensor topics and publishes robot-level odometry.

Current nodes:

- `vectornav_odometry`: VectorNav-only odometry bridge. Useful for IMU testing,
  but not enough for valid local position.
- `zed_odometry`: ZED-only pose-to-odometry bridge.
- `zed_vectornav_odometry`: current preferred hardware odometry path. Uses ZED
  position and VectorNav orientation/angular velocity, then publishes:

```text
/tardigrade/state/odometry
```

Experimental EKF work lives in `tardigrade_bringup` for now:

- `config/zed_vectornav_ekf.yaml`: `robot_localization` EKF config.
- `launch/zed_vectornav_ekf.launch.py`: reads `/zed/zed_node/odom` and
  `/vectornav/imu`, publishes `/tardigrade/state/odometry/filtered`, and can
  publish `odom -> base_link` TF.

This package should remain sensor/estimation focused. It should not publish
direct actuator commands.

### Legacy `tardigrade_px4`

Retired PX4-aware package. It is preserved under `src/legacy/tardigrade_px4`
and ignored by colcon through `src/legacy/COLCON_IGNORE`.

Historical responsibilities:

- connect to Pixhawk over USB MAVLink,
- keep the older PX4 ROS 2 `/fmu/*` path available for mock/uXRCE work,
- send MAVLink arm/disarm commands,
- enter external-control/Offboard mode,
- publish `/tardigrade/status`,
- send `/tardigrade/state/odometry` to PX4 as MAVLink visual odometry,
- optionally send required PX4 params into RAM at startup,
- clamp teleop speed limits to zero by default.

Important executables:

- `mavlink_pixhawk_interface`: old hardware Pixhawk interface.
- `mavlink_odometry_to_px4`: narrower MAVLink odometry bridge.
- `odometry_to_px4`: PX4 ROS 2 topic odometry bridge.
- `pixhawk_interface`: older/mock PX4 ROS 2 topic interface.
- `keyboard_cmd_vel`: old copy of the keyboard teleop tool. Active teleop now
  lives in `tardigrade_teleop`.

### `tardigrade_bringup`

Owns launch files.

Current important launch files:

- `mock.launch.py`: ROS-only fake backend for local development.
- `vectornav_state.launch.py`: VectorNav-only odometry path.
- `zed_state.launch.py`: ZED-only odometry path.
- `zed_vectornav_state.launch.py`: current preferred ZED + VectorNav odometry
  path.

Bringup launch files should not hide safety-critical actions. Arming and
external-control enabling stay explicit service calls.

### `config`

Holds operator-tuned configuration.

Current important files:

- `zed_low_load.yaml`: lower-load ZED wrapper settings.
- `thruster_map.yaml`: physical thruster map. This documents real wiring,
  output assignment, and force directions. The current ROS/PX4 path does not
  read this file yet.

### `docker`

Owns container startup for both local development and Jetson hardware bringup.

Current important files:

- `Dockerfile`: builds the ROS Foxy environment.
- `compose.yaml`: base local-development service. This is safe to run on a
  MacBook or other non-Jetson machine because it only mounts the workspace. It
  exposes rosbridge port `9090` for the Foxglove MVP and Foxglove WebSocket
  port `8765` for a future `foxglove_bridge` path.
- `compose.jetson.yaml`: Jetson/ZED/ESP override. This adds host networking,
  privileged device access, `/dev`, USB, ZED SDK, CUDA, and Tegra library
  mounts.
- `ros_entrypoint.sh`: sources ROS and the workspace when the container starts.
- `ros_bashrc.sh`: interactive shell setup and aliases.
- `run_jetson_hardware.sh`: raw `docker run` fallback. Use it only when Docker
  Compose is unavailable on the Jetson.

Use `compose.yaml` alone for laptop/local work. Use both Compose files together
for Jetson hardware work. Prefer the root helper script:

```bash
./docker-build.sh
./docker-build.sh --build
./docker-build.sh --jetson
```

Build the workspace inside the container with:

```bash
./build.sh
./build.sh --hardware
./build.sh --clean
```

Do not restore `docker/run_jetson_zed.sh`; it was replaced by
`docker/run_jetson_hardware.sh`.

### Git Metadata Notes

`.legacy_inspect` was a stale gitlink without a matching `.gitmodules` entry.
It was not an active package and should not be restored.

### Future packages

These remain planned or future-facing:

- `tardigrade_teleop`: should eventually own keyboard/manual control.
- `tardigrade_perception`: camera perception and detector outputs.
- `tardigrade_gate_controller`: convert detections into robot-level motion.
- `tardigrade_behavior_tree`: future BehaviorTree.CPP orchestration.
- `tardigrade_sim` or similar: fake ROS inputs and lightweight world simulation.

Future packages should communicate through robot-level topics/services, not
direct PX4 topics.

### Foxglove / Observability

Foxglove is the intended pool-test UI path. Avoid building a custom webapp
unless Foxglove fails a specific, documented requirement.

For ROS 2 Foxy, the first connection path is rosbridge:

```bash
ros2 launch tardigrade_bringup foxglove_rosbridge.launch.py
```

Connect Foxglove using the Rosbridge connection type at:

```text
ws://localhost:9090
```

The preferred `foxglove_bridge` can be revisited later. The apt package
`ros-foxy-foxglove-bridge` does not exist in the standard ROS package index,
and source-building current upstream has required extra dependencies such as
`rosx_introspection` and `resource_retriever`.

The repo should grow toward:

- committed Foxglove layout files,
- a standard rosbridge launch path for Foxy,
- camera and debug-image topics,
- gate/slalom detection topics,
- `/tf`, odometry, IMU, command, and status panels,
- mission/autonomy state topics once autonomy exists.

The UI work should focus on publishing good ROS data and layouts, not on
custom frontend infrastructure.

## System Data Flow

### Current Hardware Bringup Path

```text
ZED camera
  |
  | /zed/zed_node/pose
  v
zed_wrapper

VectorNav IMU
  |
  | /vectornav/imu
  v
vectornav driver

/zed/zed_node/pose + /vectornav/imu
  |
  v
tardigrade_state_estimation.zed_vectornav_odometry
  |
  | /tardigrade/state/odometry
  v
tardigrade_px4.mavlink_pixhawk_interface
  |
  | MAVLink ODOMETRY
  | MAVLink SET_MODE / COMMAND_LONG
  | MAVLink Offboard setpoints
  v
Pixhawk / PX4 over USB
```

This path is partially verified, but not yet proven repeatable for Offboard and
arming. Use `docs/jetson_zed_px4_startup.md` for the latest debug procedure.

### Teleop Path

```text
keyboard
  |
  v
tardigrade_px4.keyboard_cmd_vel
  |
  | /tardigrade/cmd_vel
  v
tardigrade_px4.mavlink_pixhawk_interface
  |
  | MAVLink body-frame velocity setpoints
  v
PX4 actuator allocation / mixer
  |
  v
thrusters
```

`/tardigrade/cmd_vel` uses ROS body-frame FLU:

```text
linear.x   forward
linear.y   left
linear.z   up
angular.z  yaw left
```

The Pixhawk interface converts that into PX4/MAVLink body-frame conventions.

### Mock PX4 Path

```text
ros2 service call /tardigrade/set_armed
  |
  v
tardigrade_px4.pixhawk_interface
  |
  | /fmu/in/vehicle_command
  v
tardigrade_px4.mock_px4_status
  |
  | /fmu/out/vehicle_command_ack
  | /fmu/out/vehicle_status
  v
tardigrade_px4.pixhawk_interface
  |
  | /tardigrade/status
  v
operator / tests
```

This path remains useful for local software testing, but it is not the current
Pixhawk hardware path.

## Runtime Interfaces To Know

```text
/zed/zed_node/pose
```

ZED local pose. This is the current local position source.

```text
/vectornav/imu
```

VectorNav IMU. Used for attitude and angular velocity.

```text
/tardigrade/state/odometry
```

Robot-level odometry consumed by the Pixhawk interface.

```text
/tardigrade/status
```

Robot/Pixhawk status. Use this to check `px4_connected`, `armed`,
`external_control_enabled`, visual odometry age, and command ACKs.

```text
/tardigrade/cmd_vel
```

Robot-level teleop velocity command.

```text
/tardigrade/set_external_control
```

Service that asks PX4 to enter external-control/Offboard mode.

```text
/tardigrade/set_armed
```

Service that sends the arm/disarm command.

## Safety Defaults

- Never arm automatically on node startup.
- Never enter external control automatically on node startup.
- Keep arming and external-control enable as explicit service calls.
- Speed clamps default to zero in velocity teleop mode.
- Ignore or neutralize stale teleop/autonomy commands.
- Keep direct actuator/thruster tests separate from normal teleop.
- Fill and verify `config/thruster_map.yaml` before commanding real thrust.
- Treat `configure_px4_params:=true` as a bench workaround until Pixhawk
  parameter storage is fixed.

## Development Philosophy

- Build one thin vertical slice at a time.
- Keep packages small and responsibility-driven.
- Keep PX4-specific details inside `src/legacy/tardigrade_px4`.
- Prefer robot-level topics and services at package boundaries.
- Mock hardware-facing inputs when hardware is unreliable.
- Keep these `.agents` docs updated when decisions change.

## Near-Term Roadmap

The active task list lives in:

```text
.agents/current_plan.md
```

High-level direction:

1. Keep local Docker development smooth while the robot is unreliable.
2. Build Foxglove observability for pool and bench testing.
3. Add fake ROS inputs so autonomy can be tested without hardware.
4. Keep PX4/MAVLink behavior debuggable through status topics and isolated
   tests.
5. Preserve the Jetson/Pixhawk runbook for the next hardware session.
6. Keep real-thrust work blocked until arming, odometry, and physical thruster
   mapping are repeatable.
