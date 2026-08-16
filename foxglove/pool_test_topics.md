# Pool Test Topics

This is the first topic plan for the Foxglove pool-test layout. Treat it as a
target interface list; some topics do not exist yet.

## Robot Status

```text
/tardigrade/status
```

Robot-level status, odometry freshness, and debug detail. The ESP-first path
does not currently have a full hardware status topic.

## State Estimate

```text
/tardigrade/state/odometry
/tardigrade/state/odometry/filtered
/zed/zed_node/odom
/zed/zed_node/pose
/vectornav/imu
/tardigrade/sensors/imu
/tf
/tf_static
```

Show ZED pose, VectorNav IMU behavior, and the fused robot odometry topic.
`/tardigrade/state/odometry` uses ZED position plus fresh VectorNav orientation
when available.

`/tardigrade/state/odometry/filtered` is the experimental `robot_localization`
EKF output. Compare it against raw ZED odometry and the current simple fused
topic before using it for autonomy.

Current caveat: this odometry topic does not yet publish an `odom -> base_link`
TF transform. Use Raw Messages and Plot panels first; add it to the 3D panel as
an odometry topic if available. A future TF publisher should make `base_link`
visible in the frame tree.

Useful fields:

```text
/tardigrade/sensors/imu.orientation
/tardigrade/sensors/imu.angular_velocity
/zed/zed_node/odom.pose.pose.position
/tardigrade/state/odometry.pose.pose.position
/tardigrade/state/odometry.pose.pose.orientation
/tardigrade/state/odometry/filtered.pose.pose.position
/tardigrade/state/odometry/filtered.pose.pose.orientation
/tardigrade/state/odometry.twist.twist.angular
```

## Commands

```text
/tardigrade/cmd_vel
/tardigrade/cmd_vel/manual
/tardigrade/thrusters/cmd
/tardigrade/esp/state
/tardigrade/teleop/enabled
/tardigrade/control/enabled
/tardigrade/control/odometry_fresh
/tardigrade/control/command_fresh
```

Current robot-level velocity command. Useful for checking teleop/autonomy
intent before worrying about physical motion.

The thruster and ESP topics show what the bridge is sending to the firmware. They are
monitoring-only and should be plotted next to `/tardigrade/cmd_vel` during
bench and pool checks.

## PID tuning

```text
/tardigrade/control/roll/debug
/tardigrade/control/pitch/debug
/tardigrade/control/yaw/debug
/tardigrade/control/depth/debug
```

Each `tardigrade_interfaces/PidDebug` contains the active gains, setpoint,
measurement, error, individual terms, limited output, and saturation flag.
The Foxglove layout calls `/tardigrade/control/set_pid_gains` and
`/tardigrade/control/set_axes_enabled` through rosbridge.

## Cameras

Known or expected sources:

```text
/zed/zed_node/left/image_rect_color
/zed/zed_node/right/image_rect_color
/zed/zed_node/depth/depth_registered
```

Open questions:

- What are the exact ZED topics on the Jetson with the current wrapper config?
- What is the final downward Arducam image topic?

## Perception Debug

Target topics:

```text
/tardigrade/perception/gate/detections
/tardigrade/perception/slalom/detections
/tardigrade/perception/debug_image
```

Open question: use `vision_msgs` if it is practical on Foxy, otherwise define
small custom messages in `tardigrade_interfaces`.

## Autonomy

Target topics:

```text
/tardigrade/autonomy/state
/tardigrade/autonomy/events
```

These do not exist yet. Add them when the first mission runner exists.
