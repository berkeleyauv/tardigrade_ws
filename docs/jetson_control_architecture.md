# Shared Control Architecture

## Decision

Simulation and hardware use one backend-agnostic physical-unit control path:

```text
manual / mission / pose guidance (TwistStamped)
  -> velocity_setpoint_mux
  -> /tardigrade/control/velocity_setpoint
  -> velocity_wrench_controller + filtered odometry
  -> /tardigrade/control/wrench_command (N, N m)
  -> thruster_allocator
  -> /tardigrade/actuators/thruster_forces (named N)
  -> thruster_actuator_mapper
  -> /tardigrade/actuators/thruster_commands (named [-1, 1])
  -> Unity OR esp_bridge
```

`tardigrade_control` owns guidance, feedback, allocation, and actuator curves.
`tardigrade_esp` owns only serial transport and physical slot mapping. Unity
and the ESP therefore receive the same actuator message.

## Command sources

The mux accepts three isolated sources:

| Source | Topic |
|---|---|
| Manual | `/tardigrade/control/velocity_setpoint/manual` |
| Mission | `/tardigrade/control/velocity_setpoint/mission` |
| Pose guidance | `/tardigrade/control/velocity_setpoint/pose` |

Select one with the `active_source` launch argument or the mux ROS parameter.
Manual authority additionally requires a fresh true
`/tardigrade/teleop/enabled`; releasing the deadman clears controller state and
commands zero. Every other stale source also produces zero and disables the
controller.

Pose guidance consumes an `odom`-frame `geometry_msgs/PoseStamped` on
`/tardigrade/control/pose_setpoint`, rotates position error into body FLU, and
produces bounded body velocity/angular-rate setpoints.

## Feedback and allocation

The 50 Hz inner controller uses only
`/tardigrade/state/odometry/filtered`. Each axis has versioned gains in
`src/tardigrade_control/config/control.yaml`. It implements PI feedback,
filtered derivative-on-measurement, optional fitted drag feed-forward, local
output limits, and tracking anti-windup.

The allocator solves the six-DOF wrench against measured thruster geometry and
asymmetric limits. It publishes `/tardigrade/control/allocation_status` with
requested wrench, achieved wrench, residual, feasibility, and saturated
thruster names. The controller uses this achieved wrench to prevent integral
windup when the vehicle cannot realize a request.

## ESP contract

`esp_bridge` validates every named command, rejects missing/duplicate/unknown
names and non-finite values, reorders by the physical slot map, then emits one
`SetMotor` frame per slot. Its command watchdog sends eight neutral commands
when updates stop. The firmware independently validates packets, enforces its
authority limit, handles link timeout, drives PWM, and remains below the
physical kill switch.

Individual checkout uses the same named
`/tardigrade/actuators/thruster_commands` interface as the controller. The
positional actuator interface has been removed.

## Safety layers

1. Manual control requires the continuously held deadman.
2. The source mux rejects stale, malformed, wrong-frame, or unselected input.
3. The feedback controller clears integrators when command or odometry expires.
4. The allocator and actuator mapper neutralize stale upstream commands.
5. `esp_bridge` neutralizes stale actuator commands within 0.5 seconds.
6. ESP link timeout, firmware watchdog, and authority limits remain active.
7. The physical kill switch removes motor power independently of software.

Do not forward Jetson pose to the transitional controller still present in the
ESP firmware. Run only `esp_bridge` as the serial owner and remove the old ESP
controller/mixer after this ROS path is proven in water.

## Launches

```bash
# Real closed-loop manual control (requires filtered odometry)
ros2 launch tardigrade_bringup pool_assisted.launch.py

# Unity with the same controller and allocator
ros2 launch tardigrade_bringup unity_sil.launch.py

# Individual hardware checkout; same named actuator interface, bounded service
ros2 launch tardigrade_esp thruster_checkout_real.launch.py
```
