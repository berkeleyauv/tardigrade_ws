# Tardigrade v2 Architecture

## Purpose

Tardigrade v2 is a clean ROS 2 Foxy rebuild of the Berkeley AUV software
stack. The goal is to let future team members write robot behavior in robot
concepts without needing to understand PX4 internals.

The core design rule is:

> Robot code talks in robot concepts. Only driver/adapter code talks in
> hardware concepts.

PX4 is the current actuator and safety backend. It should not define the whole
codebase. The long-term architecture should allow a different controller, such
as an ESP-based thruster controller, without rewriting teleop, autonomy,
perception, or state-estimation code.

## Current Status Snapshot

As of July 6, 2026:

- The working bench Pixhawk path is USB MAVLink, not Micro XRCE-DDS.
- `mavlink_pixhawk_interface` can connect to the Pixhawk over `/dev/ttyACM0`.
- The robot has successfully armed through ROS using:
  - ZED visual position,
  - VectorNav attitude/angular velocity,
  - MAVLink visual odometry,
  - runtime PX4 parameter configuration.
- The Pixhawk parameter save path is broken on the current board:
  `param save` fails to export to `/fs/mtd_params`.
- Because PX4 params do not persist, `configure_px4_params:=true` is required
  after Pixhawk reboot for bench arming.
- ZED wrapper support is documented in `docs/jetson_zed_px4_startup.md`.
- ZED wrapper source is now tracked as a top-level submodule pinned to the
  `humble-v4.0.8` tag commit:
  - `src/zed-ros2-wrapper`
  The wrapper's own nested submodule provides:
  - `src/zed-ros2-wrapper/zed-ros2-interfaces`
- Docker startup is split into a local-safe base Compose file and a Jetson
  hardware override:
  - `docker/compose.yaml`
  - `docker/compose.jetson.yaml`
- The ZED camera is the local position source.
- The VectorNav IMU is the attitude and angular-velocity source.
- `tardigrade_state_estimation` publishes `/tardigrade/state/odometry`.
- `tardigrade_px4` forwards that odometry to PX4 as MAVLink `ODOMETRY`.
- `keyboard_cmd_vel` provides a first keyboard teleop path through
  `/tardigrade/cmd_vel`.
- `config/thruster_map.yaml` documents actual thruster wiring and geometry, but
  it is not consumed by code yet.

## Package Boundaries

### `px4_msgs`

Vendored PX4 ROS 2 messages. This package exists for the older/direct PX4 ROS 2
topic path and mock testing.

Only PX4 adapter code should import `px4_msgs`. Application packages should not
depend on PX4 message definitions.

### `vectornav` and `vectornav_msgs`

External VectorNav driver packages. The current hardware launch uses the
VectorNav driver directly and expects the IMU on:

```text
/vectornav/imu
```

The VectorNav is not a local-position source by itself. It provides attitude,
angular velocity, acceleration, and related IMU data.

### ZED wrapper packages

The Stereolabs ZED ROS 2 wrapper is used as an external dependency on the
Jetson. The current known working direction is documented in
`docs/jetson_zed_px4_startup.md`.

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

This is the current local position source for PX4 arming.

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
- `zed_vectornav_odometry`: current preferred arming path. Uses ZED position
  and VectorNav orientation/angular velocity, then publishes:

```text
/tardigrade/state/odometry
```

This package should remain sensor/estimation focused. It should not arm PX4,
enter Offboard mode, or publish direct actuator commands.

### `tardigrade_px4`

The only PX4-aware package.

Current responsibilities:

- connect to Pixhawk over USB MAVLink for the working hardware path,
- keep the older PX4 ROS 2 `/fmu/*` path available for mock/uXRCE work,
- send MAVLink arm/disarm commands,
- enter external-control/Offboard mode,
- publish `/tardigrade/status`,
- send `/tardigrade/state/odometry` to PX4 as MAVLink visual odometry,
- optionally send required PX4 params into RAM at startup,
- subscribe to `/tardigrade/cmd_vel` for teleop velocity commands,
- clamp teleop speed limits to zero by default.

Important executables:

- `mavlink_pixhawk_interface`: current hardware Pixhawk interface.
- `mavlink_odometry_to_px4`: narrower MAVLink odometry bridge.
- `odometry_to_px4`: PX4 ROS 2 topic odometry bridge.
- `pixhawk_interface`: older/mock PX4 ROS 2 topic interface.
- `keyboard_cmd_vel`: first keyboard teleop tool.

Long term, `keyboard_cmd_vel` may move to a dedicated teleop package. For now it
lives in `tardigrade_px4` because it was added as part of the bench control
slice.

### `tardigrade_bringup`

Owns launch files.

Current important launch files:

- `mock.launch.py`: mock PX4 path for local development.
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
  MacBook or other non-Jetson machine because it only mounts the workspace.
- `compose.jetson.yaml`: Jetson/ZED/Pixhawk override. This adds host networking,
  privileged device access, `/dev`, USB, ZED SDK, CUDA, and Tegra library
  mounts.
- `run_jetson_zed.sh`: legacy convenience script. It still starts the same style
  of hardware container and auto-detects the NVIDIA Docker runtime when present.

Use `compose.yaml` alone for laptop/local work. Use both Compose files together
for Jetson hardware work.

### Git Metadata Notes

`.legacy_inspect` is currently tracked as a gitlink without a matching
`.gitmodules` entry. It appears to be stale repository metadata, not an active
package. Do not confuse it with the active submodules under `src/`.

### Future packages

These remain planned or future-facing:

- `tardigrade_teleop`: should eventually own keyboard/manual control.
- `tardigrade_perception`: camera perception and detector outputs.
- `tardigrade_gate_controller`: convert detections into robot-level motion.
- `tardigrade_behavior_tree`: future BehaviorTree.CPP orchestration.

Future packages should communicate through robot-level topics/services, not
direct PX4 topics.

## System Data Flow

### Current Bench Arming Path

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
working Pixhawk hardware path.

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
- Keep PX4-specific details inside `tardigrade_px4`.
- Prefer robot-level topics and services at package boundaries.
- Mock hardware-facing inputs when hardware is unreliable.
- Keep these `.agents` docs updated when decisions change.

## Near-Term Roadmap

1. Make the Pixhawk parameter-storage problem explicit and decide whether to
   replace the Pixhawk, fix MTD storage, or keep runtime params as a bench-only
   workaround.
2. Verify `config/thruster_map.yaml` against real wiring and QGroundControl
   actuator tests.
3. Test teleop with zero clamps, then tiny nonzero clamps one axis at a time.
4. Move keyboard teleop into a dedicated `tardigrade_teleop` package when the
   control path stabilizes.
5. Add real thruster allocation in ROS only if the team decides to bypass PX4
   actuator allocation later.
6. Start perception/autonomy only after arming, odometry, teleop, and safe
   actuator mapping are repeatable.
