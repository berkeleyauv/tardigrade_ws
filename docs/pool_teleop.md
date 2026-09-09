# End-to-End Pool Test Runbook

This is the canonical procedure for the current Tardigrade hardware stack.
Follow it in order. The real ESP firmware and binary protocol are unchanged.

## 1. Safety and ownership

The production command path is:

```text
ZED + VectorNav -> EKF -> velocity/rate controller -> wrench allocator
Xbox / mission / pose -> source mux ----------------^             |
                                                                   v
                         named force/command mapping -> ESP bridge
                         ESP: arm, authority limit, PWM, watchdog
```

The Jetson owns feedback and allocation. The ESP remains an independent safety
layer. During normal operation do not run `pose_bridge.py`, `gcs_server.py
--ros`, the `/tardigrade/test/synthetic_pose` bench hook, or a second
`esp_bridge`.

Only one load-bearing mode may run at once:

```text
thruster_checkout_real.launch.py
pool_assisted.launch.py
prequal_autonomy.launch.py
qual_autonomy.launch.py
```

Keep thruster power disconnected for software checks. During powered tests,
secure or tether the vehicle and assign one person to the physical kill switch.
Release LB, disarm, and remove thruster power before troubleshooting.

## 2. Hardware identity and clean build

Current stable serial paths are:

```text
VectorNav  /dev/serial/by-id/usb-FTDI_USB-RS232-WE_AV0LN035-if00-port0
ESP32      /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

Confirm them rather than relying on changeable `/dev/ttyUSB*` numbers:

```bash
ls -l /dev/serial/by-id/
```

After pulling changes on the Jetson, clean-build so removed launch files cannot
survive in the install tree:

```bash
cd /ws
./build.sh --clean --hardware
source install/setup.bash

colcon test --packages-select \
  tardigrade_interfaces tardigrade_description \
  tardigrade_state_estimation tardigrade_control \
  tardigrade_teleop tardigrade_esp tardigrade_bringup tardigrade_mission
colcon test-result --verbose

ros2 launch tardigrade_esp thruster_checkout_real.launch.py --show-args
ros2 launch tardigrade_bringup pool_assisted.launch.py --show-args
```

Any build or test failure blocks powered testing. Firmware verification remains
a separate procedure; this ROS cleanup does not require reflashing it.

## 3. Foxglove and Xbox

Start rosbridge on the Jetson:

```bash
ros2 launch tardigrade_bringup foxglove_rosbridge.launch.py
```

Connect Foxglove Desktop using **Rosbridge** at `ws://JETSON_IP:9090`. Import:

```text
foxglove/layouts/pool_checkout.json
foxglove/layouts/pool_operator.json
foxglove/layouts/pool_sensors.json
foxglove/layouts/pid_tuning.json
```

Install the Tardigrade Tools extension first as described in
`foxglove/README.md`; these layouts reuse the same status, attitude, and PID
panels as simulation.

When the Foxglove Joystick Panel publishes `/joy`, use this browser mapping:

```text
axes[0]  left stick horizontal: positive left/sway
axes[1]  left stick vertical:   positive forward/surge
axes[2]  right stick horizontal: positive left/yaw
axes[3]  right stick vertical:   positive up/heave
buttons[4] LB deadman
```

With thruster power disconnected, verify `/joy` is fresh, axes return to zero,
and LB changes only `buttons[4]`. Override launch axis arguments if the actual
mapping differs; never compensate by memory.

## 4. Sensors and state estimation

The ZED, VectorNav, combined comparison, and EKF launches are intentionally
retained for pool diagnosis. A typical sequence is:

```bash
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zed publish_tf:=false

ros2 launch tardigrade_bringup zed_vectornav_state.launch.py \
  port:=/dev/serial/by-id/usb-FTDI_USB-RS232-WE_AV0LN035-if00-port0 \
  baud:=115200 use_zed_orientation_if_imu_stale:=false

ros2 launch tardigrade_bringup zed_vectornav_ekf.launch.py
```

Check:

```bash
ros2 topic hz /vectornav/imu
ros2 topic hz /tardigrade/sensors/imu
ros2 topic hz /zed/zed_node/odom
ros2 topic hz /tardigrade/state/odometry/filtered
```

Require finite, fresh filtered odometry; correct frame IDs; plausible level
roll/pitch; positive yaw when the nose turns left; and a clean
`odom -> base_link` TF owner. Use `coordinate_frames.md` for the full unpowered
sign check. A wrong sign, stale sensor, or conflicting TF blocks powered tests.

## 5. Bounded individual-thruster checkout

Use this whenever slot identity or polarity is uncertain. Stop all other
controllers and ESP bridges, then start:

```bash
ros2 launch tardigrade_esp thruster_checkout_real.launch.py \
  serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

Confirm `/tardigrade/esp/state` is fresh and `link_ok` is true. Confirm the
service and named command topic exist before arming:

```bash
ros2 service type /tardigrade/test/run_thruster
ros2 topic type /tardigrade/actuators/thruster_commands

ros2 service call /tardigrade/set_armed \
  tardigrade_interfaces/srv/SetArmed "{armed: true}"
```

Request one 1-indexed physical slot at low authority:

```bash
ros2 service call /tardigrade/test/run_thruster \
  tardigrade_interfaces/srv/TestThruster \
  "{slot: 1, command: 0.05, duration_sec: 1.0}"
```

The node publishes eight named setpoints in physical slot order with exactly
one nonzero value. It caps authority at `0.10`, duration at two seconds,
rejects malformed or overlapping requests, and publishes neutral before/after
tests, on rejection, on timeout, and during shutdown. The bridge independently
neutralizes stale commands within 0.5 seconds.

For every slot, record the physical motor, positive force direction, and that
it remained neutral before and after the request. Do not raise software or
firmware authority to work around an ESC deadband. Disarm after each session:

```bash
ros2 service call /tardigrade/set_armed \
  tardigrade_interfaces/srv/SetArmed "{armed: false}"
```

Update `thruster_mapping.md` if observation differs from configuration.

## 6. Assisted-control dry check

Stop individual checkout and every standalone ESP bridge. Start:

```bash
ros2 launch tardigrade_bringup pool_assisted.launch.py
```

For an Xbox connected directly to the Jetson, append
`start_joy_node:=true heave_axis:=4 yaw_axis:=3 device_id:=0`.

With thruster power disconnected, verify exactly one publisher at each stage:

```text
/tardigrade/control/velocity_setpoint/manual
/tardigrade/control/velocity_setpoint
/tardigrade/control/wrench_command
/tardigrade/actuators/thruster_forces
/tardigrade/actuators/thruster_commands
```

LB released must disable manual authority and produce eight named zeros. With
LB held, move one axis at a time and inspect command signs. Stick release, LB
release, controller disconnect, network loss, stale odometry, and stopping an
upstream node must each neutralize the output within its documented watchdog
period. Do not arm until every stop path passes.

## 7. Restrained and wet tests

Secure the robot and keep the kill-switch operator ready. Arm explicitly and
test heave, yaw, surge, then sway with brief single-axis inputs. Release LB
between observations. Stop immediately for a wrong sign, unexpected motor,
stale state, persistent allocation residual, or failure to neutralize.

Record every powered attempt:

```bash
ros2 bag record -o pool_checkout_01 \
  /joy /zed/zed_node/odom /vectornav/imu \
  /tardigrade/sensors/imu \
  /tardigrade/state/odometry/filtered \
  /tardigrade/control/velocity_setpoint/manual \
  /tardigrade/control/velocity_setpoint \
  /tardigrade/control/wrench_command \
  /tardigrade/control/allocation_status \
  /tardigrade/actuators/thruster_forces \
  /tardigrade/actuators/thruster_commands \
  /tardigrade/esp/state /tf /tf_static
```

Tune one loop at a time with low output limits: heave, yaw, surge, sway, then
restrained roll/pitch rates. Start with integral and drag feed-forward at zero.
Use `/tardigrade/control/{axis}/debug`, allocation status, named forces, and
named commands. Accepted live gains must be copied into
`tardigrade_control/config/control.yaml` and verified after restart.

At the end of every run, release LB, call `SetArmed` with `armed: false`, stop
the launch, verify `armed: false`, and remove thruster power.
