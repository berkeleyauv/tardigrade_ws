# Tardigrade Tools

Original Berkeley AUV Foxglove panels for simulation and pool operations:

- **PID Tuner** selects one velocity axis, separates response from actuator
  effort, shows P/I/D contributions, and updates all gains atomically through
  `/tardigrade/control/set_velocity_pid_gains`.
- **Vehicle Status** combines safety gates, sensor freshness, arming controls,
  allocator health, and a physical eight-thruster display.
- **Attitude** provides an artificial horizon, ROS yaw tape, depth, and compact
  body-velocity readouts from filtered odometry.

The interaction patterns were informed by Duke Robotics' public Foxglove
panels and Blue Robotics Cockpit instruments, but the implementation and ROS
interfaces are Berkeley-specific and original Apache-2.0 code.

## Build and install

Foxglove Desktop and a developer seat are required for local extensions.

```bash
cd foxglove/extensions/tardigrade-tools
pnpm install --frozen-lockfile
pnpm run package
```

In Foxglove Desktop, open **Settings → Extensions → Install local extension**
and select the generated `.foxe` file. Install the extension before importing
layouts that contain Tardigrade panels.

For local development, `pnpm run local-install` builds and copies the extension
into Foxglove Desktop's local extension directory. Reload Foxglove after each
installation.

## Safety

The status panel requires a two-click confirmation to arm and offers immediate
disarm. It does not publish direct thruster commands. Direct physical thruster
testing remains behind `/tardigrade/test/run_thruster`, including its ROS-side
limits and ESP watchdog.
