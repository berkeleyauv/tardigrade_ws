# Roll/Attitude Tuning Migration Note

The former normalized `depth_attitude_controller` procedure is retired. Do not
use its `SetControlAxes` or `SetPidGains` services for new testing.

The active stack controls body angular rate in physical torque units:

```text
pose guidance roll error
  -> desired roll rate (rad/s)
  -> velocity_wrench_controller roll loop
  -> requested roll torque (N m)
  -> bounded allocator
  -> named thruster forces and commands
```

First tune the inner roll-rate loop with a restrained, low-torque test. Set
`roll.ki` and drag feed-forward to zero, start with a small output limit, and
increase proportional gain gradually:

```bash
ros2 param set /velocity_wrench_controller roll.ki 0.0
ros2 param set /velocity_wrench_controller roll.output_limit 2.0
ros2 param set /velocity_wrench_controller roll.kp 1.0
```

Monitor and record:

```text
/tardigrade/control/velocity_setpoint
/tardigrade/state/odometry/filtered
/tardigrade/control/wrench_command
/tardigrade/control/allocation_status
/tardigrade/actuators/thruster_forces
/tardigrade/actuators/thruster_commands
```

Stop if the feedback sign is wrong, the allocator residual remains large, a
thruster stays saturated, or odometry becomes stale. After the inner rate loop
is stable, tune `pose_velocity_controller`'s `attitude.kp` separately. Record
accepted values in `src/tardigrade_control/config/control.yaml`.

See [Shared Control Architecture](jetson_control_architecture.md) and the
[pool runbook](pool_teleop.md) for the active command and safety path.
