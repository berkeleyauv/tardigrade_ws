# Foxglove Setup

Foxglove is the operator display for sensors, state estimation, Xbox input,
thruster commands, ESP status, recording review, and PID tuning.

ROS 2 Foxy uses rosbridge on port `9090` for the current pool path.

## Connect To The Jetson

Start rosbridge on the Jetson:

```bash
source /ws/install/setup.bash
ros2 launch tardigrade_bringup foxglove_rosbridge.launch.py
```

Find the Jetson's shared-network address:

```bash
hostname -I
```

In Foxglove Desktop on the MacBook, create a **Rosbridge** connection:

```text
ws://JETSON_IP:9090
```

The MacBook and Jetson must be on the same routed network. A dedicated router
or phone hotspot is preferable to venue Wi-Fi with client isolation.

## Xbox Controller On The MacBook

Install Josh Newans' **Joystick Panel** from the Foxglove Extension
Marketplace, or download the latest `.foxe`:

```text
https://github.com/joshnewans/foxglove-joystick/releases/latest
```

Pair the Xbox controller in macOS, add the panel to the active layout, and set:

```text
Data Source:       Gamepad
Gamepad ID:        0
Publish Mode:      enabled
Pub Joy Topic:     /joy
Display Mode:      Custom Display
Layout:            Xbox
```

Expected browser mapping after the extension's axis conversion:

```text
axes[0]    left horizontal:  positive left
axes[1]    left vertical:    positive forward
axes[2]    right horizontal: positive left yaw
axes[3]    right vertical:   positive up
buttons[4] LB deadman
```

Verify on the Jetson before thruster power is connected:

```bash
ros2 topic hz /joy
ros2 topic echo /joy
```

The extension currently publishes its platform mapping directly. If the
indices differ, use the configurable `pool_direct.launch.py` or
`pool_assisted.launch.py` axis arguments. Never memorize reversed controls.

The extension does not explicitly publish a final zero on every disconnect.
The Jetson teleop node therefore owns the safety behavior: `/joy` older than
250 ms disables teleop and commands zero. Keep Foxglove open, keep the panel
present, and keep the MacBook awake. Confirm the real disconnect behavior dry.

## Saved Layouts

Use these committed layouts:

- `layouts/pool_checkout.json`: camera, estimator, ESP, commands, and bounded
  individual-thruster service.
- `layouts/state_estimation.json`: ZED, VectorNav, filtered odometry, and TF.
- `layouts/pid_tuning.json`: PID services, four debug streams, command chain,
  saturation, and freshness.
- `layouts/zed.json`: focused camera and ZED tracking view.

Import the JSON into Foxglove and save a personal copy if panel placement is
changed. The Joystick Panel is a third-party extension and may need to be added
to the imported layout manually.

## Current Topics

State and sensors:

```text
/zed/zed_node/left/image_rect_color
/zed/zed_node/odom
/vectornav/imu
/tardigrade/sensors/imu
/tardigrade/state/odometry
/tardigrade/state/odometry/filtered
/tf
/tf_static
```

Operator and actuator chain:

```text
/joy
/tardigrade/teleop/enabled
/tardigrade/cmd_vel/manual
/tardigrade/cmd_vel
/tardigrade/thrusters/cmd
/tardigrade/esp/state
```

Controller health and PID debug:

```text
/tardigrade/control/enabled
/tardigrade/control/odometry_fresh
/tardigrade/control/command_fresh
/tardigrade/control/roll/debug
/tardigrade/control/pitch/debug
/tardigrade/control/yaw/debug
/tardigrade/control/depth/debug
```

Services:

```text
/tardigrade/set_armed
/tardigrade/test/run_thruster
/tardigrade/control/set_pid_gains
/tardigrade/control/set_axes_enabled
```

`/tardigrade/esp/state.state_valid` and `pose_ok` are expected to remain false
because pose is intentionally not forwarded to the transitional ESP
controller. Monitor the Jetson odometry topics for robot pose.

## Start A Robot Mode

The Foxglove Xbox direct path is the default:

```bash
ros2 launch tardigrade_bringup pool_direct.launch.py
```

After all sensor and direct-mode gates pass:

```bash
ros2 launch tardigrade_bringup pool_assisted.launch.py
```

Both launches start an ESP bridge. Stop standalone ESP/thruster launches first.
Follow the complete [pool runbook](../docs/pool_teleop.md) before arming.

## Recording And Playback

Use `ros2 bag record` on the Jetson for every powered attempt. The canonical
topic command is in the pool runbook. Open a recorded bag in Foxglove or play it
back into ROS:

```bash
ros2 bag play BAG_DIRECTORY
```

## Troubleshooting

No Foxglove connection:

```bash
ros2 node list
hostname -I
```

Confirm the rosbridge launch is still running, the selected IP belongs to the
shared network, and the network allows device-to-device traffic.

No `/joy`:

- confirm the Xbox is connected in macOS;
- press a controller button after opening the panel;
- confirm Gamepad ID `0`, Publish Mode, and `/joy`;
- keep the Joystick Panel in the current layout;
- inspect Foxglove's connection status.

Commands stop when Foxglove loses focus or network quality falls:

This is the 250 ms stale-input safety working. Restore a reliable operator
link; do not increase the timeout for pool operation.
