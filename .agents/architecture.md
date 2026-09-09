# Tardigrade Workspace Architecture

## Package boundaries

- `tardigrade_interfaces`: stable robot-level messages and services.
- `tardigrade_description`: canonical vehicle, sensor, and thruster data plus
  generated allocator and Unity configuration.
- `tardigrade_control`: command selection, pose guidance, physical-unit
  feedback, geometry allocation, and actuator curves.
- `tardigrade_state_estimation`: ZED and VectorNav conversion and filtered
  robot state.
- `tardigrade_teleop`: stamped keyboard and Xbox velocity sources.
- `tardigrade_esp`: the real binary serial protocol, telemetry, arming,
  watchdog, physical slot mapping, and bounded individual-thruster checkout.
- `tardigrade_mission`: retained prequalification and qualification missions.
- `tardigrade_bringup`: complete Unity and hardware launch profiles only.

External drivers remain under `src/ROS-TCP-Endpoint`, `src/vectornav`, and
`src/zed-ros2-wrapper`.

## Canonical control flow

```text
manual / mission / pose velocity setpoint (TwistStamped)
  -> velocity_setpoint_mux
  -> velocity_wrench_controller + filtered odometry
  -> WrenchStamped (N, N m)
  -> thruster_allocator
  -> named ThrusterForces (N)
  -> thruster_actuator_mapper
  -> named ThrusterCommands [-1, 1]
  -> Unity plant OR esp_bridge
```

Legacy positional command topics and ROS-only mock backends have been removed.
Unity connects through ROS-TCP Endpoint and publishes its own simulated sensor,
clock, status, reset, and truth interfaces.

## Hardware invariants

- Only one process may own the ESP serial port.
- Firmware protocol encoding and physical slot mapping are hardware contracts.
- Arming is always an explicit `/tardigrade/set_armed` service call.
- Manual authority requires a fresh deadman signal.
- Every control stage and the ESP bridge neutralizes stale input.
- Direct motor identification uses `thruster_checkout_real.launch.py` and
  `/tardigrade/test/run_thruster`; it never bypasses the binary bridge.
- The synthetic-pose topic is a bench failsafe hook and must remain off during
  normal Jetson control.
- Sensor diagnostic launches stay available until pool evidence selects a
  canonical replacement.
