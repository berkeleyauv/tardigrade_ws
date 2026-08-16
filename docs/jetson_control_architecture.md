# Jetson Control Architecture

## Decision

The Jetson owns robot state estimation, setpoints, PID control, and mixing. The
ESP32 is a safe eight-channel actuator.

```text
ZED + VectorNav
      -> robot_localization EKF
      -> /tardigrade/state/odometry/filtered
      -> depth_attitude_controller
      -> /tardigrade/cmd_vel
      -> thruster_mixer
      -> /tardigrade/thrusters/cmd
      -> esp_bridge
      -> ESP SetMotor
      -> ESC PWM
```

Direct teleop bypasses the PID node but uses the same mixer, bridge, and ESP
safety path. Assisted teleop and future autonomy use the same controller; only
the source of manual/setpoint intent changes.

## Ownership

| Responsibility | Owner |
|---|---|
| ZED and VectorNav drivers | Jetson ROS |
| Frame conversion and EKF | Jetson ROS |
| Xbox mapping and LB deadman | Jetson ROS |
| Roll, pitch, yaw, and optional depth PID | Jetson ROS |
| Eight-thruster mixing | Jetson ROS |
| Serial ownership and packet encoding | Jetson `esp_bridge` |
| Arming, heartbeat timeout, authority clamp | ESP firmware |
| PWM generation and neutral output | ESP firmware |
| Hardware watchdog | ESP firmware |
| Physical power removal | Kill switch/operator |

Exactly one backend consumes `/tardigrade/thrusters/cmd`: `esp_bridge` on the
robot or the simulator backend during simulation.

## ESP Contract

The normal host-to-ESP messages are:

```text
Heartbeat
Arm / Disarm
SetMotor(index, normalized value)
GetState
```

The ESP validates packet CRC, motor index, numeric range, armed state, and link
freshness. It clamps every direct motor request to its firmware authority limit
before writing PWM. In the current firmware that limit is `±0.30` per thruster.

The ROS checkout node has an additional, lower `±0.10` limit. Teleop body
commands have their own conservative limits before mixing. These stacked limits
are intentional.

Normalized command is ESC command range, not a linear fraction of electrical
power, RPM, or thrust.

## No Pose Packet In Normal Operation

The Jetson already owns fused pose and must not forward it to the ESP. Do not
run:

- firmware `pose_bridge.py`;
- firmware `gcs_server.py --ros`;
- ROS `/tardigrade/test/synthetic_pose`;
- any second process that owns the ESP serial port.

The transitional ESP firmware still contains `ExternalEstimator`,
`RobosubController`, and `RobosubMixer`. A fresh Pose packet can make that
legacy controller active and allow it to overwrite Jetson motor requests.
Keeping pose absent makes the path inactive.

Consequently these ESP telemetry fields are expected to be false:

```text
state_valid
pose_ok
```

They do not describe Jetson estimator health. Use
`/tardigrade/state/odometry/filtered` and the controller freshness topics for
that purpose.

After the Jetson path is proven, the firmware should remove the pose estimator,
onboard controller/mixer, parameter tuning path, and synthetic-pose scaffolding.
Until then, operational separation is mandatory.

## Safety Layers

1. LB must be continuously held for operator commands.
2. Joy older than 250 ms disables teleop and publishes zero.
3. Assisted control requires fresh Joy, enable heartbeat, command, and EKF
   odometry; failure clears integrators and publishes zero.
4. The mixer and bridge command watchdogs neutralize stale input within 500 ms.
5. Loss of Jetson serial traffic disarms the ESP in approximately 300 ms.
6. The ESP hardware watchdog resets a stalled firmware loop.
7. The physical kill switch removes motor authority independently of software.

No layer replaces the one below it. Every stop path must be rehearsed before a
wet or PID test.

## Runtime Modes

```bash
# Individual slot checkout
ros2 launch tardigrade_esp thruster_checkout_real.launch.py

# MacBook/Foxglove Xbox, open-loop body commands
ros2 launch tardigrade_bringup pool_direct.launch.py

# MacBook/Foxglove Xbox, Jetson assisted control
ros2 launch tardigrade_bringup pool_assisted.launch.py
```

These modes are mutually exclusive because each owns the real actuator path.
The complete operating procedure is [pool_teleop.md](pool_teleop.md).
