# Tardigrade Runtime and Checkout Modes

Run only one command-producing mode at a time. Sensor and Foxglove processes
may run alongside any mode, but `thruster_checkout_real`, `xbox_direct_real`,
and `xbox_assisted_real` are mutually exclusive because each owns the
load-bearing thruster command path.

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
  port:=/dev/serial/by-id/YOUR_VECTORNAV_DEVICE
```

Check `/tardigrade/sensors/imu` and `/tardigrade/state/odometry`. Perform the
unpowered sign tests in `docs/coordinate_frames.md`.

## Mode 3: complete fused state

Start the ZED as in Mode 1, then run these in separate terminals:

```bash
ros2 launch tardigrade_bringup zed_vectornav_state.launch.py \
  port:=/dev/serial/by-id/YOUR_VECTORNAV_DEVICE \
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
  serial_port:=/dev/serial/by-id/YOUR_ESP_DEVICE
```

In Foxglove, use `/tardigrade/set_armed` deliberately, then call
`/tardigrade/test/run_thruster`. The request is 1-indexed:

```json
{"slot": 1, "command": 0.05, "duration_sec": 1.0}
```

The node rejects slots outside 1–8, commands above 0.10, durations above two
seconds, and non-finite values. It commands the other seven slots to zero and
automatically returns all eight to zero. Use `{"armed": false}` immediately
after each observation. Keep the physical kill switch reachable.

Expected current locations are listed in `docs/thruster_mapping.md`.

## Mode 5: direct Xbox checkout

Stop the individual-thruster launch, then run:

```bash
ros2 launch tardigrade_esp xbox_direct_real.launch.py \
  serial_port:=/dev/serial/by-id/YOUR_ESP_DEVICE
```

LB is the continuous deadman. This checks joystick, command signs, mixer, and
ESP behavior without feedback control. Start with thruster power disconnected,
then perform only the low-authority wet test after every axis sign is accepted.

## Mode 6: assisted Xbox and PID tuning

Stop direct mode, start the fused state, then:

```bash
ros2 launch tardigrade_esp xbox_assisted_real.launch.py \
  serial_port:=/dev/serial/by-id/YOUR_ESP_DEVICE
```

Use `pid_tuning.json`. Begin with every axis disabled, then roll only, pitch
only, and yaw only. Depth remains disabled until underwater ZED Z tracking is
accepted. LB release, stale Joy, stale odometry, or stale commands force zero.

## Recording

Record every powered attempt:

```bash
ros2 bag record -o pool_checkout_01 \
  /zed/zed_node/odom /vectornav/imu /tardigrade/sensors/imu \
  /tardigrade/state/odometry /tardigrade/state/odometry/filtered \
  /tardigrade/cmd_vel/manual /tardigrade/cmd_vel \
  /tardigrade/thrusters/cmd /tardigrade/esp/state /tf /tf_static
```
