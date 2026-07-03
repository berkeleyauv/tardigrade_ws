# Tardigrade v2 Architecture

## Purpose

Tardigrade v2 is a clean rebuild of the Berkeley AUV software stack. The goal is a maintainable ROS 2 Foxy workspace where future team members can write robot behavior without needing to understand PX4 internals.

The core design rule is simple:

> Robot code talks in robot concepts. Only driver code talks in hardware concepts.

PX4 is the week-one actuator/control backend, but it should not define the whole codebase. The long-term architecture should allow the Pixhawk to be replaced by an ESP-based controller without rewriting teleop, autonomy, perception, or mission logic.

## Package Boundaries

### Current Status Snapshot

As of July 2, 2026:

- `px4_msgs` is present as a submodule and builds.
- `vectornav` is present as a submodule from the `ros2` branch; `vectornav` and `vectornav_msgs` build.
- `tardigrade_interfaces` exists and builds.
- `tardigrade_px4` exists and provides the current PX4 adapter.
- `tardigrade_bringup` exists with `mock.launch.py`.
- Mock bringup works: mock PX4 status, fake command acknowledgements, PX4 driver services, neutral Offboard heartbeat, and `/tardigrade/status`.
- `tardigrade_state_estimation`, `tardigrade_teleop`, `tardigrade_perception`, and `tardigrade_behavior_tree` are planned but not implemented yet.

### `px4_msgs`

Vendored as a submodule and pinned to the PX4 firmware version running on the Pixhawk. This package is a dependency of `tardigrade_px4` only.

No application package should import `px4_msgs`.

### `vectornav` and `vectornav_msgs`

External dependency from `berkeleyauv/vectornav`, branch `ros2`.

The VectorNav driver publishes:

- `/vectornav/raw/common`
- `/vectornav/imu`
- `/vectornav/pose`
- `/vectornav/velocity`
- related GNSS, magnetic, pressure, and temperature topics

Treat these as sensor inputs, not robot state by themselves.

### `tardigrade_interfaces`

Owns robot-level messages and services. This package is the public API between subsystems.

Initial interfaces should cover:

- arm/disarm requests
- external-control enable/disable requests
- robot status
- robot-level teleop motion commands
- robot odometry if standard `nav_msgs/Odometry` is not sufficient

This package must not depend on `px4_msgs`.

### `tardigrade_px4`

The only PX4-aware package.

Responsibilities:

- subscribe to `/fmu/out/*`
- publish `/fmu/in/*`
- send Offboard heartbeat
- publish neutral Offboard `TrajectorySetpoint` while active
- send arm/disarm commands
- enter/exit Offboard/external-control mode
- subscribe to PX4 `VehicleCommandAck`
- convert robot-level teleop commands into PX4 Offboard setpoints
- convert robot odometry into PX4-compatible odometry input
- publish robot-level status

This package may import `px4_msgs`. Other Tardigrade packages may not.

Current implemented PX4 topics:

- subscribes `/fmu/out/vehicle_status`
- subscribes `/fmu/out/vehicle_command_ack`
- publishes `/fmu/in/vehicle_command`
- publishes `/fmu/in/offboard_control_mode`
- publishes `/fmu/in/trajectory_setpoint`
- publishes `/tardigrade/status`
- serves `/tardigrade/set_armed`
- serves `/tardigrade/set_external_control`

### `tardigrade_state_estimation`

Consumes VectorNav topics and produces robot-level state.

Week-one scope should be a thin, useful bridge:

- validate timestamps and frames
- consume `/vectornav/imu`, `/vectornav/pose`, and `/vectornav/velocity` as available
- publish robot odometry, preferably `nav_msgs/Odometry`
- provide PX4-ready odometry to `tardigrade_px4`

Do not build a full custom estimator until the basic robot bringup path is stable.

### `tardigrade_teleop`

Owns keyboard/manual control.

Responsibilities:

- read keyboard input
- publish robot-level motion commands
- call robot-level arm/disarm/external-control services

It must not publish PX4 messages or `/fmu/*` topics directly.

### `tardigrade_bringup`

Owns launch files and bringup modes.

Initial launch modes:

- `mock`: mock PX4 status/ack plus PX4 driver
- `hardware`: VectorNav, Micro XRCE-DDS Agent instructions, PX4 driver, state estimation, teleop-ready interfaces
- `bench`: later mode for explicitly gated thruster/actuator tests

### `tardigrade_perception`

Future package for camera pipelines, gate detection, target estimation, and perception health.

It should publish robot-level detections or target poses, not commands directly to PX4.

First gate-perception contract:

- subscribe `/front_camera/image_raw` or another configured `sensor_msgs/Image` topic
- run a YOLO gate detector
- publish `/tardigrade/perception/gate/detection`
- publish `/tardigrade/perception/gate/debug_image` when debug output is enabled

The first detection topic may use `geometry_msgs/PointStamped`:

- `point.x`: normalized gate center x in `[-1, 1]`
- `point.y`: normalized gate center y in `[-1, 1]`
- `point.z`: confidence in `[0, 1]`

Later, replace this with a custom `GateDetection.msg` if the task needs bounding boxes, class IDs, stale flags, or pose estimates.

Perception must not publish `/fmu/*` topics and should not directly arm, enter Offboard, or command PX4.

### `tardigrade_gate_controller`

Planned package or module for converting gate detections into robot-level motion commands.

Responsibilities:

- subscribe to `/tardigrade/perception/gate/detection`
- apply confidence thresholds and smoothing
- handle lost-gate behavior
- publish `/tardigrade/cmd_vel` as `geometry_msgs/TwistStamped`

This layer is deliberately separate from YOLO so the detector can change without rewriting control behavior.

### `tardigrade_behavior_tree`

Future C++ package for BehaviorTree.CPP.

Most robot code should remain Python. The C++ boundary exists because BehaviorTree.CPP is a C++ library. BT nodes should communicate through `tardigrade_interfaces`, not through PX4 topics.

## System Data Flow

### Week-One Hardware Path

```text
VectorNav
  |
  v
vectornav driver
  |
  | /vectornav/imu
  | /vectornav/pose
  | /vectornav/velocity
  v
tardigrade_state_estimation
  |
  | /tardigrade/state/odometry
  v
tardigrade_px4
  |
  | /fmu/in/vehicle_odometry or /fmu/in/vehicle_visual_odometry
  | /fmu/in/offboard_control_mode
  | /fmu/in/trajectory_setpoint
  | /fmu/in/vehicle_command
  v
PX4 / Pixhawk
```

### Current Mock PX4 Path

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
operator / future teleop / future autonomy
```

### Teleop Path

```text
keyboard
  |
  v
tardigrade_teleop
  |
  | robot-level command topic/services
  v
tardigrade_px4
  |
  | PX4 Offboard setpoints and commands
  v
PX4 / Pixhawk
```

### Autonomy Path

```text
tardigrade_behavior_tree
  |
  | robot-level services/actions/topics
  v
tardigrade_interfaces
  |
  v
tardigrade_px4 / teleop / perception / state estimation
```

### Gate Task Path

```text
front camera
  |
  | /front_camera/image_raw
  v
tardigrade_perception
  |
  | /tardigrade/perception/gate/detection
  v
tardigrade_gate_controller or behavior tree action node
  |
  | /tardigrade/cmd_vel
  v
tardigrade_px4
  |
  | PX4 Offboard setpoints
  v
PX4 / Pixhawk
```

## Interface Ownership

Use standard ROS messages where they are already a good fit:

- `nav_msgs/Odometry` for robot odometry
- `geometry_msgs/Twist` or `TwistStamped` for simple velocity commands
- `sensor_msgs/Imu` from VectorNav
- `sensor_msgs/Image` for camera frames
- `geometry_msgs/PointStamped` for the first simple gate-center detection output

Use `tardigrade_interfaces` when the concept is robot-specific:

- arm/disarm service
- external-control enable service
- robot status
- mission/autonomy status
- future task actions
- future custom gate detection if `PointStamped` becomes too limiting

Do not expose PX4 enum values, PX4 topic names, or PX4 message fields in public Tardigrade interfaces.

## Safety Defaults

- Never arm automatically on node startup.
- Never enter external control automatically on node startup.
- Publish Offboard heartbeat continuously only when the PX4 driver is active and configured to do so.
- Ignore teleop motion commands unless external control is explicitly enabled.
- Add a command timeout: stale teleop/autonomy commands produce neutral setpoints.
- Keep direct actuator/thruster tests separate from normal teleop.
- Prefer conservative defaults that make bench testing slower but harder to do accidentally.

## Development Philosophy

- Build one thin vertical slice at a time.
- Keep packages small and responsibility-driven.
- Avoid adding packages before they have a real job.
- Mock hardware-facing inputs when hardware is unreliable.
- Keep architecture documents updated as decisions change.

## Gate Autonomy Roadmap

The gate task is more than "detect gate and drive forward." It likely needs:

- camera calibration
- gate detection
- gate pose or centerline estimation
- search behavior when the gate is not visible
- alignment behavior
- approach behavior
- pass-through behavior
- completion detection
- timeout and recovery behavior
- manual abort path

BehaviorTree.CPP should orchestrate those behaviors, but the behavior nodes should call robot-level ROS APIs. PX4 should remain hidden behind `tardigrade_px4`.

Suggested first gate autonomy decomposition:

1. `DetectGate`: perception reports a confident gate detection.
2. `CenterOnGate`: controller reduces image-space x/y error.
3. `ApproachGate`: command slow forward motion while maintaining alignment.
4. `PassThroughGate`: continue through after close/centered criteria.
5. `ConfirmGateComplete`: use timeout, detection loss pattern, or downstream state to mark success.
6. `AbortOrHold`: neutral command and wait for operator if confidence/state is unsafe.
