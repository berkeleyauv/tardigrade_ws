# Tardigrade Architecture

## Purpose

Tardigrade is a ROS 2 Foxy workspace for the Berkeley AUV software stack. The
core design rule is:

> Robot code talks in robot concepts. Driver and adapter code talks to hardware.

The ESP32 serial PWM path is the active actuator backend. Autonomy, teleop,
state estimation, simulation, and visualization should communicate through
robot-level `/tardigrade/*` topics and services.

## Current Status Snapshot

As of July 2026:

- Local Docker development is the default workflow.
- Jetson hardware work uses `docker/compose.jetson.yaml` through
  `./docker-build.sh --jetson`.
- ZED pose is the local position source.
- VectorNav IMU is the attitude and angular-velocity source.
- `tardigrade_state_estimation` publishes `/tardigrade/state/odometry`.
- The experimental EKF publishes `/tardigrade/state/odometry/filtered`.
- `tardigrade_esp` owns the ESP serial PWM bridge.
- `tardigrade_teleop` owns keyboard velocity commands.
- `tardigrade_sim` provides a ROS-only fake backend for local mission testing.
- ZED/ESP bringup is documented in `docs/esp_thruster_bringup.md`.
- Thruster mapping is documented in `docs/thruster_mapping.md` and configured
  through `config/esp_thruster_map.json`.

## Package Boundaries

### `tardigrade_interfaces`

Owns robot-level messages and services:

- `/tardigrade/status`
- `/tardigrade/set_armed`
- `/tardigrade/set_external_control`
- gate and slalom detection messages

### `tardigrade_state_estimation`

Consumes sensor topics and publishes robot-level odometry.

Current nodes:

- `vectornav_odometry`
- `zed_odometry`
- `zed_vectornav_odometry`

This package should remain sensor/estimation focused and should not publish
direct actuator commands.

### `tardigrade_esp`

Owns the active ESP32 USB serial PWM path.

Important nodes:

- `esp_thruster_bridge`: converts `/tardigrade/cmd_vel` to ESP PWM serial
  commands using `config/esp_thruster_map.json`.
- `esp_thruster_test`: bench utility for one-output-at-a-time tests.
- `depth_attitude_controller`: stabilizes depth, roll, and pitch while passing
  manual planar commands through.

### `tardigrade_teleop`

Owns operator input tools. `keyboard_cmd_vel` publishes
`geometry_msgs/Twist` commands.

### `tardigrade_mission`

Owns mission scripts that use robot-level status, perception, and command
interfaces.

### `tardigrade_sim`

Owns local fake backends for testing missions without robot hardware.

### `tardigrade_bringup`

Owns launch files. Launch files should not hide safety-critical actions.
Arming and external-control enabling stay explicit service calls.

### External Driver Packages

- `src/vectornav`: VectorNav driver and messages.
- `src/zed-ros2-wrapper`: Stereolabs ZED wrapper, with its nested
  `zed-ros2-interfaces` submodule.
- `src/ROS-TCP-Endpoint`: Unity ROS-TCP endpoint.

## Current Data Flow

```text
ZED + VectorNav
  -> tardigrade_state_estimation
  -> /tardigrade/state/odometry

/tardigrade/cmd_vel/manual
  -> depth_attitude_controller
  -> /tardigrade/cmd_vel
  -> esp_thruster_bridge
  -> ESP32
  -> ESC PWM pins
```

For simple teleop, `keyboard_cmd_vel` can publish directly to
`/tardigrade/cmd_vel`.

## Runtime Interfaces To Know

```text
/zed/zed_node/pose
/zed/zed_node/odom
/vectornav/imu
/tardigrade/state/odometry
/tardigrade/state/odometry/filtered
/tardigrade/status
/tardigrade/cmd_vel
/tardigrade/cmd_vel/manual
/tardigrade/thrusters/pwm
/tardigrade/esp/status
/tardigrade/set_external_control
/tardigrade/set_armed
```

## Safety Defaults

- Never arm automatically on node startup.
- Never enter external control automatically on node startup.
- Keep arming and external-control enable as explicit service calls.
- Ignore or neutralize stale teleop/autonomy commands.
- Keep direct actuator/thruster tests separate from normal teleop.
- Verify `config/esp_thruster_map.json` and `docs/thruster_mapping.md` before
  commanding real thrust.
- Keep thruster power disconnected until sensor, command, and mapping checks
  look sane.

## Development Philosophy

- Build one thin vertical slice at a time.
- Keep packages small and responsibility-driven.
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
4. Keep ESP/control health visible through status topics and tests.
5. Keep real-thrust work blocked until odometry and physical thruster mapping
   are repeatable.
