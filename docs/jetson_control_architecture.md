# Jetson-Side Control Architecture

## Decision

The robosub's PID control runs **on the Jetson as a ROS node**, not on the ESP.
The ESP becomes a **safe thruster actuator**: it receives per-thruster commands
over serial, applies them with independent failsafes, and reports telemetry. It
does no fusion, no control, no mixing.

## Why

- **Sim parity.** The controller is a ROS node, so the *same node with the same
  gains* drives the Unity/local simulator and the real robot — you tune in sim
  (no pool time) and the values transfer verbatim. With control on the ESP
  (C++), the sim would need a duplicate controller (drift) or hardware-in-the-
  loop (complex).
- **Gains live in one versioned place.** A YAML in the repo is the source of
  truth, loaded by both sim and real. This dissolves the ESP-flash-vs-git
  tension entirely.
- **Foxglove tuning is ROS-native.** Gains are ROS parameters. For the Foxy
  pool path, Foxglove calls the controller's gain/axis services through
  rosbridge; a future `foxglove_bridge` upgrade can expose the native
  Parameters panel. No serial `SetParameter` bridge is needed.
- **Latency is a non-issue for a slow AUV.** Depth/heading dynamics are
  seconds-scale; 30–50 Hz control over serial with some Linux jitter is more
  than adequate. The determinism argument for on-MCU control applies to twitchy
  multirotors, not a sub.
- **Most of it already exists.** `depth_attitude_controller.py`,
  `esp_thruster_bridge.py` (mixing), and `esp_thruster_map.json` are already in
  this workspace. This is refinement + reconnection, not green-field.

## Node graph

```
  ZED + VectorNav → robot_localization EKF
                          │  /tardigrade/state/odometry/filtered (nav_msgs/Odometry, 30 Hz)
                          ▼
                    controller node  ── PID: depth, heading, roll, pitch
                          │            gains = ROS params (from YAML)
                          │  /tardigrade/cmd_vel (normalized body wrench)
                          ▼
                     mixer node  ── esp_thruster_map.json (8×6 matrix)
                          │  /tardigrade/thrusters/cmd (Float32MultiArray[8], -1..+1)
              ┌───────────┴────────────┐
              ▼                        ▼
       esp_bridge (REAL)        sim backend (SIM)
       SetMotor frames →        apply to simulated
       ESP over serial          thrusters
              │
              ▼
       ESP32: apply PWM + SAFETY (deadman, watchdog, arm, authority cap)
```

`/tardigrade/thrusters/cmd` is the seam. Exactly one backend subscribes at a
time, chosen by which launch file you run — that's the whole sim-vs-real switch.

## Where each piece lives

| Piece | Location | Notes |
|---|---|---|
| PID controller | Jetson ROS node | refines `depth_attitude_controller.py` |
| Gains | `config/controller_gains.yaml` → ROS params | source of truth; sim + real load the same file |
| Setpoints (depth/heading) | ROS params or `/tardigrade/setpoint` topic | captured on arm, or commanded live |
| Mixer + mix matrix | mixer node + `esp_thruster_map.json` | already exists |
| Thruster→serial | `esp_bridge` (F2 extension) | SetMotor frames in the new binary protocol |
| Simulated thrusters | sim backend node | subscribes the same `/tardigrade/thrusters/cmd` |
| Safety (deadman/watchdog/arm/PWM/authority cap) | **ESP firmware** | unchanged, independent of the Jetson |

## The ESP's new (smaller) role

The ESP firmware keeps only what must be close to the hardware and independent
of the smart layer:

- **Keep:** ESC PWM generation, `Safety` (arm state + operator deadman),
  `HardwareWatchdog`, `MotorManager`, the wire protocol, `SetMotor` handling
  with the authority cap.
- **Remove (migrates to the Jetson):** `RobosubController`, `RobosubMixer`,
  `Pid`, the `IController`/`IMixer` seams, `ExternalEstimator`, `JetsonLink`,
  the `Pose` frame, and the on-ESP parameter/flash tuning path
  (`Parameters.h`, `SetParameter`/`GetParameters`/`SaveParameters`, NVS).

Note the ESP no longer needs the pose at all — the Jetson has it directly from
the EKF. The whole `Pose`-frame-to-ESP path drops out.

## Safety after the move

Nothing about moving control weakens the ESP's safety role — if anything it
sharpens it:

- **Deadman unchanged.** The Jetson (via `esp_bridge`) sends continuous
  Heartbeat while armed; if the Jetson, the control node, or the link dies, the
  ESP disarms in 300 ms. The controller crashing = safe stop.
- **Authority cap on the ESP** clamps every `SetMotor`, so a bug in the
  Jetson-side controller commanding full thrust is still limited by firmware the
  controller can't override.
- **Physical kill switch** remains the ultimate backstop (see
  foxglove_integration.md).

The ESP as an *independent* safety layer guarding against bugs in the smart
layer is a cleaner separation than "the ESP does everything."

## Sim vs. real, concretely

Same controller + mixer nodes, same gains YAML, in both:

```bash
# real robot:
ros2 launch tardigrade_bringup control_real.launch.py   # + esp_bridge backend
# simulator:
ros2 launch tardigrade_bringup control_sim.launch.py    # + sim backend
```

The only difference is which node consumes `/tardigrade/thrusters/cmd`. Tune in
sim, commit the YAML, run on the real robot with identical numbers.

## Roadmap impact

This simplifies more than it adds:

- **D4 / F2 gets simpler:** no serial `SetParameter` bridge and no ESP-side
  parameter sync — gains are ROS params plus a YAML. The Foxy pool layout uses
  explicit ROS services for reliable rosbridge tuning.
- **The ESP firmware shrinks** to the safe-actuator role (a large deletion, not
  new code).
- **Build order:** (1) strip the ESP to safe-actuator, (2) bring the controller
  + mixer nodes onto the new `/tardigrade/thrusters/cmd` seam and the new binary
  protocol via `esp_bridge`, (3) wire the gains YAML + ROS params, (4) validate
  in sim, then on the real robot.

## Transitional note

The ESP-side control code (`RobosubController`, `ExternalEstimator`, etc.) stays
in the firmware until the Jetson controller is proven, then is removed — same
"don't delete the working thing before its replacement exists" principle used
for the webapp→Foxglove transition. Manual per-thruster bench testing does not
depend on it, so bench bring-up is unaffected in the meantime.
