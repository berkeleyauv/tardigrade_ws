# Tardigrade Runtime and Checkout Modes

Run only one command-producing mode at a time. Sensor and Foxglove processes
may run alongside any mode, but `thruster_checkout_real`, `pool_direct`,
`pool_keyboard`, and `pool_assisted` are mutually exclusive because each owns
the load-bearing thruster command path.

## Common monitoring

Start rosbridge and connect Foxglove to `ws://JETSON_IP:9090`:

```bash
ros2 launch tardigrade_bringup foxglove_rosbridge.launch.py
```

Useful layouts:

- `foxglove/layouts/pool_checkout.json`: camera, estimator, ESP, and bounded
  individual-thruster service.
- `foxglove/layouts/zed.json`: detailed ZED camera and TF inspection.
- `foxglove/layouts/state_estimation.json`: ZED, VectorNav, EKF, and TF.
- `foxglove/layouts/pid_tuning.json`: controller services and PID plots.

## Mode 1: ZED camera only

```bash
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zed publish_tf:=false
```

Check:

```bash
ros2 topic hz /zed/zed_node/left/image_rect_color
ros2 topic hz /zed/zed_node/odom
```

Use `zed.json` or the image panel in `pool_checkout.json`. Confirm live video,
smooth odometry, increasing Z when the camera moves upward, and no tracking
reset during short hand motions.

## Mode 2: VectorNav only

```bash
ros2 launch tardigrade_bringup vectornav_state.launch.py \
  port:=/dev/serial/by-id/usb-FTDI_USB-RS232-WE_AV0LN035-if00-port0
```

Check `/tardigrade/sensors/imu` and `/tardigrade/state/odometry`. Perform the
unpowered sign tests in `docs/coordinate_frames.md`.

## Mode 3: complete fused state

Start the ZED as in Mode 1, then run these in separate terminals:

```bash
ros2 launch tardigrade_bringup zed_vectornav_state.launch.py \
  port:=/dev/serial/by-id/usb-FTDI_USB-RS232-WE_AV0LN035-if00-port0 \
  use_zed_orientation_if_imu_stale:=false
```

```bash
ros2 launch tardigrade_bringup zed_vectornav_ekf.launch.py
```

The comparison output is `/tardigrade/state/odometry`; the EKF output used by
assisted control is `/tardigrade/state/odometry/filtered`. Use
`state_estimation.json` and check that position follows ZED while orientation
and angular rates follow the converted VectorNav without TF warnings.

## Mode 4: one thruster at a time

Stop every mixer, teleop, PID, and other ESP bridge first. Start only:

```bash
ros2 launch tardigrade_esp thruster_checkout_real.launch.py \
  serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

In Foxglove, use `/tardigrade/set_armed` deliberately, then call
`/tardigrade/test/run_thruster`. The request is 1-indexed:

```json
{"slot": 1, "command": 0.10, "duration_sec": 1.0}
```

The node rejects slots outside 1–8, commands above 0.10, durations above two
seconds, and non-finite values. It commands the other seven slots to zero and
automatically returns all eight to zero. Use `{"armed": false}` immediately
after each observation. Keep the physical kill switch reachable.

Expected current locations are listed in `docs/thruster_mapping.md`.

## Mode 5: direct Xbox checkout

Stop the individual-thruster launch, then run:

```bash
ros2 launch tardigrade_bringup pool_direct.launch.py
```

LB is the continuous deadman. This checks joystick, command signs, mixer, and
ESP behavior without feedback control. Start with thruster power disconnected,
then perform only the low-authority wet test after every axis sign is accepted.

The default expects the controller on the operator MacBook and the Foxglove
Joystick Panel publishing `/joy`. For an Xbox connected directly to the
Jetson, use:

```bash
ros2 launch tardigrade_bringup pool_direct.launch.py \
  start_joy_node:=true heave_axis:=4 yaw_axis:=3 device_id:=0
```

See `foxglove/README.md` for the Mac panel settings and dry checks.

## Mode 6: direct keyboard fallback

Stop `pool_direct` and every other ESP bridge. Start the mixer and ESP backend:

```bash
ros2 launch tardigrade_bringup pool_keyboard.launch.py
```

In a second interactive Jetson/SSH terminal, run:

```bash
ros2 run tardigrade_teleop keyboard_cmd_vel --ros-args \
  -p linear_step:=0.15 -p vertical_step:=0.12 -p yaw_step:=0.15 \
  -p command_hold_sec:=0.25
```

Keys are `w/s` surge, `j/l` sway, `r/f` heave, `a/d` yaw, and Space for
immediate zero. Every motion key produces one 250 ms pulse and then
automatically returns to zero. Tap repeatedly for continued motion. Stopping
the keyboard node or losing SSH also lets the mixer timeout to zero.

This mode has no continuous deadman and is therefore only a restrained direct
checkout fallback. It must not be used for assisted/PID tuning.

## Mode 7: assisted Xbox and PID tuning

Stop direct mode, start the fused state, then:

```bash
ros2 launch tardigrade_bringup pool_assisted.launch.py
```

Use `pid_tuning.json`. Begin with every axis disabled, then roll only, pitch
only, and yaw only. Depth remains disabled until underwater ZED Z tracking is
accepted. LB release, stale Joy, stale odometry, or stale commands force zero.

For an Xbox connected directly to the Jetson, use:

```bash
ros2 launch tardigrade_bringup pool_assisted.launch.py \
  start_joy_node:=true heave_axis:=4 yaw_axis:=3 device_id:=0
```

## Recording

Record every powered attempt:

```bash
ros2 bag record -o pool_checkout_01 \
  /zed/zed_node/odom /vectornav/imu /tardigrade/sensors/imu \
  /tardigrade/state/odometry /tardigrade/state/odometry/filtered \
  /tardigrade/cmd_vel/manual /tardigrade/cmd_vel \
  /tardigrade/thrusters/cmd /tardigrade/esp/state /tf /tf_static
```
