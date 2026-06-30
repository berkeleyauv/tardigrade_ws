# Tardigrade v2 Architecture

## Purpose

Tardigrade v2 is a clean rebuild of the Berkeley AUV software stack. The goal is a maintainable ROS 2 Foxy workspace where future team members can write robot behavior without needing to understand PX4 internals.

The core design rule is simple:

> Robot code talks in robot concepts. Only driver code talks in hardware concepts.

PX4 is the week-one actuator/control backend, but it should not define the whole codebase. The long-term architecture should allow the Pixhawk to be replaced by an ESP-based controller without rewriting teleop, autonomy, perception, or mission logic.

## Package Boundaries

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
- send arm/disarm commands
- enter/exit Offboard/external-control mode
- convert robot-level teleop commands into PX4 Offboard setpoints
- convert robot odometry into PX4-compatible odometry input
- publish robot-level status

This package may import `px4_msgs`. Other Tardigrade packages may not.

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

- `mock`: mock PX4 status plus PX4 driver
- `hardware`: VectorNav, Micro XRCE-DDS Agent instructions, PX4 driver, state estimation, teleop-ready interfaces
- `bench`: later mode for explicitly gated thruster/actuator tests

### `tardigrade_perception`

Future package for camera pipelines, gate detection, target estimation, and perception health.

It should publish robot-level detections or target poses, not commands directly to PX4.

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

## Interface Ownership

Use standard ROS messages where they are already a good fit:

- `nav_msgs/Odometry` for robot odometry
- `geometry_msgs/Twist` or `TwistStamped` for simple velocity commands
- `sensor_msgs/Imu` from VectorNav

Use `tardigrade_interfaces` when the concept is robot-specific:

- arm/disarm service
- external-control enable service
- robot status
- mission/autonomy status
- future task actions

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
