# Legacy Week-One Bringup Plan

This file is historical context from the Jetson/ZED/VectorNav/Pixhawk bench
push. It is no longer the active plan because electrical hardware access is
blocked and local development is now the primary path.

The active plan is:

```text
.agents/current_plan.md
```

The active hardware runbook is:

```text
docs/jetson_zed_px4_startup.md
```

## What This Plan Captured

- The workspace moved away from Micro XRCE-DDS as the primary hardware path.
- The intended Pixhawk path became USB MAVLink.
- The ZED wrapper was added as a submodule at `src/zed-ros2-wrapper`.
- The wrapper's nested `zed-ros2-interfaces` submodule replaced the duplicate
  top-level `src/zed-ros2-interfaces` checkout.
- Docker startup was split between:
  - `docker/compose.yaml` for local/laptop development,
  - `docker/compose.jetson.yaml` for Jetson/ZED/Pixhawk hardware access.
- ZED pose, VectorNav IMU, fused odometry, MAVLink visual odometry, and PX4
  local-position diagnostics were observed on the bench.
- Offboard mode and arming were still under investigation.

## Hardware Findings Preserved For Context

Known useful commands from the bench session:

```bash
ros2 launch tardigrade_bringup zed_vectornav_state.launch.py \
  port:=/dev/serial/by-id/usb-FTDI_USB-RS232-WE_AV0LN035-if00-port0 \
  baud:=115200
```

```bash
ros2 run tardigrade_px4 mavlink_pixhawk_interface --ros-args \
  -p device:=/dev/ttyACM0 \
  -p baudrate:=921600 \
  -p configure_px4_params:=true
```

PX4 command ACKs that mattered:

```text
176/0  Offboard mode command accepted
400/0  arm/disarm command accepted
400/1  arm temporarily rejected
400/2  arm denied
```

Important unresolved hardware issue:

```text
param save
ERROR [parameters] parameter export to /fs/mtd_params failed
```

Because of this, `configure_px4_params:=true` was used as a bench workaround to
send required PX4 parameters into RAM on startup.
