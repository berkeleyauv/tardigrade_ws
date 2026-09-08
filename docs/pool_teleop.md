# End-to-End Pool Test Runbook

This is the canonical procedure for the current Tardigrade pool stack. Follow
it in order. Individual-slot and deprecated open-loop tests establish wiring
and signs; the normal operating path is closed-loop physical-unit control.

## 1. Architecture And Non-Negotiable Rules

The Jetson owns state estimation, command selection, feedback, and allocation:

```text
ZED + VectorNav -> EKF -> velocity/rate controller -> wrench allocator
Xbox / mission / pose -> source mux --------------^              |
                                                                    v
                             named force/command mapping -> ESP bridge
                             ESP: arm, clamp, PWM, watchdog
```

The ESP does not need pose. Its transitional onboard controller remains in the
firmware but must stay inactive. Never run any of these with the current path:

- `pose_bridge.py`
- `gcs_server.py --ros`
- `esp_thruster_bridge`
- `/tardigrade/test/synthetic_pose`
- a second `esp_bridge`

Normal ESP telemetry may show:

```text
state_valid: false
pose_ok: false
```

That is expected. Use Jetson odometry topics for pose. On the ESP, the fields
that matter are `armed` and `link_ok`.

Only one load-bearing mode may run at once:

```text
thruster_checkout_real.launch.py
pool_direct.launch.py
pool_assisted.launch.py
```

Each starts its own ESP bridge. Stop the previous mode before starting another.

The physical kill switch must be reachable by one assigned person during every
powered test. Keep the vehicle tethered. Maintain slight positive buoyancy for
closed-loop tests.

## 2. Fixed Hardware Identity

Current stable serial paths:

```text
VectorNav  /dev/serial/by-id/usb-FTDI_USB-RS232-WE_AV0LN035-if00-port0
ESP32      /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

Check what is present:

```bash
ls -l /dev/serial/by-id/
```

If identity is uncertain, unplug one device, rerun the command, reconnect it,
and confirm which entry returns. Do not use `/dev/ttyUSB0` or `/dev/ttyUSB1`;
those assignments can swap.

## 3. Sync And Build On The Jetson

After pushing the workstation changes and pulling them onto the Jetson, enter
the ROS container and clean-build the hardware workspace. A clean build is
important after launch files are renamed because old files can remain in the
install tree.

```bash
cd /ws
./build.sh --clean --hardware
source install/setup.bash
```

Run the affected tests and inspect launch arguments:

```bash
colcon test --packages-select \
  tardigrade_interfaces \
  tardigrade_state_estimation \
  tardigrade_teleop \
  tardigrade_esp \
  tardigrade_bringup
colcon test-result --verbose

ros2 launch tardigrade_bringup pool_direct.launch.py --show-args
ros2 launch tardigrade_bringup pool_assisted.launch.py --show-args
```

Any build or test failure blocks powered testing.

Firmware verification is separate; no firmware behavior change is required
for this runbook:

```bash
cd tardigrade_firmware/firmware
pio run
cd ..
python3 tools/tardigrade_protocol.py
```

## 4. Prepare The Pool Network

The MacBook and Jetson must be on the same routed network. A dedicated router
or phone hotspot is preferable to venue Wi-Fi, which may isolate clients.

On the Jetson:

```bash
hostname -I
```

Choose the address belonging to the shared Wi-Fi/network. From the MacBook,
verify it responds:

```bash
ping JETSON_IP
```

Prevent the MacBook from sleeping during operation. Keep Foxglove Desktop open
and visible; losing the panel or network must cause a safe stop, not a timeout
increase.

## 5. Start Rosbridge And Configure Foxglove

Jetson terminal A:

```bash
source /ws/install/setup.bash
ros2 launch tardigrade_bringup foxglove_rosbridge.launch.py
```

On the MacBook, connect Foxglove using the **Rosbridge** connection type:

```text
ws://JETSON_IP:9090
```

Import these layouts as needed:

```text
foxglove/layouts/pool_checkout.json
foxglove/layouts/state_estimation.json
foxglove/layouts/pid_tuning.json
foxglove/layouts/roll_pid_tuning.json
```

Install Josh Newans' **Joystick Panel** extension from the Foxglove Extension
Marketplace, or install the latest `.foxe` from:

```text
https://github.com/joshnewans/foxglove-joystick/releases/latest
```

Pair the Xbox controller with macOS and configure the panel:

```text
Data Source:       Gamepad
Gamepad ID:        0
Publish Mode:      enabled
Pub Joy Topic:     /joy
Display Mode:      Custom Display
Layout:            Xbox
```

Expected Foxglove/browser mapping:

```text
axes[0]  left stick horizontal: positive left/sway
axes[1]  left stick vertical:   positive forward
axes[2]  right stick horizontal: positive left yaw
axes[3]  right stick vertical:   positive up/heave
buttons[4] LB deadman
```

With thruster power disconnected, verify on the Jetson:

```bash
ros2 topic hz /joy
ros2 topic echo /joy
```

Move only one control at a time. All axes must return close to zero. LB alone
must change `buttons[4]` between `0` and `1`. If the indices differ, stop and
override the configurable launch arguments; do not compensate by memory.

## 6. Start And Accept The Sensors

Use separate terminals and leave each launch running.

Jetson terminal B, ZED wrapper:

```bash
source /ws/install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zed publish_tf:=false
```

Jetson terminal C, VectorNav and comparison odometry:

```bash
source /ws/install/setup.bash
ros2 launch tardigrade_bringup zed_vectornav_state.launch.py \
  port:=/dev/serial/by-id/usb-FTDI_USB-RS232-WE_AV0LN035-if00-port0 \
  baud:=115200 \
  use_zed_orientation_if_imu_stale:=false
```

Jetson terminal D, filtered EKF and TF:

```bash
source /ws/install/setup.bash
ros2 launch tardigrade_bringup zed_vectornav_ekf.launch.py
```

Check rates:

```bash
ros2 topic hz /vectornav/imu
ros2 topic hz /tardigrade/sensors/imu
ros2 topic hz /zed/zed_node/odom
ros2 topic hz /tardigrade/state/odometry/filtered
```

Acceptance requirements:

- `/tardigrade/sensors/imu.header.frame_id` is `base_link`.
- With the robot level, roll and pitch are plausible and stable.
- Nose left produces positive yaw; left side up produces positive roll.
- Nose up follows the documented ROS pitch sign.
- ZED video is live and ZED odometry moves smoothly without tracking resets.
- Filtered odometry is fresh and finite.
- TF contains `odom -> base_link`, `base_link -> vectornav`, and
  `base_link -> zed_camera_link` without repeated-frame warnings.

Use [coordinate_frames.md](coordinate_frames.md) for the complete hand-motion
test. A wrong axis, timestamp, frame, or estimator rate blocks every PID test.

## 7. Optional Individual-Thruster Identification

Do this only when physical slot identity or direction is still uncertain.
Stop `pool_direct`, `pool_assisted`, standalone mixers, and every existing ESP
bridge first.

```bash
ros2 launch tardigrade_esp thruster_checkout_real.launch.py \
  serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

Confirm the service exists, then arm deliberately:

```bash
ros2 service list | grep /tardigrade/test/run_thruster
ros2 service call /tardigrade/set_armed \
  tardigrade_interfaces/srv/SetArmed "{armed: true}"
```

Request one slot at a time. The current checkout node permits at most `0.10`
for at most two seconds and automatically returns all eight slots to zero:

```bash
ros2 service call /tardigrade/test/run_thruster \
  tardigrade_interfaces/srv/TestThruster \
  "{slot: 1, command: 0.10, duration_sec: 1.0}"
```

Record which physical thruster moves and its positive direction. A motor not
moving at `0.10` may be inside its ESC deadband; it is not permission to bypass
the software and firmware caps. Diagnose arming, command publication, wiring,
and ESC readiness before changing authority.

Disarm and stop the checkout launch:

```bash
ros2 service call /tardigrade/set_armed \
  tardigrade_interfaces/srv/SetArmed "{armed: false}"
```

Update [thruster_mapping.md](thruster_mapping.md) and the mixer JSON if the
physical observation differs.

## 8. Deprecated Open-Loop Teleop Dry Checkout

Stop the individual-thruster launch and every standalone ESP bridge. Leave
rosbridge and sensors running. Start the complete direct command path:

```bash
ros2 launch tardigrade_bringup pool_direct.launch.py
```

This compatibility-only launch defaults to Foxglove `/joy` and starts:

```text
xbox_cmd_vel -> thruster_mixer -> esp_bridge
```

For an Xbox physically connected to the Jetson instead:

```bash
ros2 launch tardigrade_bringup pool_direct.launch.py \
  start_joy_node:=true heave_axis:=4 yaw_axis:=3 device_id:=0
```

With thruster power still disconnected:

```bash
ros2 topic info /joy --verbose
ros2 topic info /tardigrade/cmd_vel --verbose
ros2 topic info /tardigrade/thrusters/cmd --verbose
ros2 topic echo /tardigrade/teleop/enabled
ros2 topic echo /tardigrade/thrusters/cmd
ros2 topic hz /tardigrade/esp/state
```

Required results:

1. Exactly one `/joy` publisher: Foxglove for the default Mac path.
2. Exactly one publisher at each load-bearing command stage.
3. LB released produces eight zeros and teleop disabled.
4. LB held plus one stick produces only the expected axis command.
5. Stick release produces zero for that axis.
6. Releasing LB produces eight zeros within 250 ms.
7. Disconnecting the controller or closing its Foxglove panel while LB is held
   produces eight zeros within 250 ms.
8. Network loss produces zero rather than retaining the last command.

Do not arm until all eight checks pass.

### Temporary keyboard fallback

If Foxglove `/joy` cannot be restored, stop `pool_direct` completely. In one
Jetson terminal start the keyboard backend:

```bash
ros2 launch tardigrade_bringup pool_keyboard.launch.py
```

In a second interactive SSH terminal start the keyboard publisher:

```bash
ros2 run tardigrade_teleop keyboard_cmd_vel --ros-args \
  -p linear_step:=0.15 -p vertical_step:=0.12 -p yaw_step:=0.15 \
  -p command_hold_sec:=0.25 \
  -p legacy_output_topic:=/tardigrade/cmd_vel
```

Use `w/s` for surge, `j/l` for sway, `r/f` for heave, `a/d` for yaw, and
Space for zero. Each press is a 250 ms pulse that automatically stops. This is
a restrained direct-checkout fallback only. It does not provide the continuous
deadman heartbeat required by assisted/PID tuning.

## 9. Restrained Direction And Stop-Layer Test

Remove propellers when practical or securely restrain the vehicle. Keep the
kill-switch operator ready. Arm:

```bash
ros2 service call /tardigrade/set_armed \
  tardigrade_interfaces/srv/SetArmed "{armed: true}"
```

Verify `armed: true` and `link_ok: true`:

```bash
ros2 topic echo /tardigrade/esp/state
```

Use brief taps, one body axis at a time. Expected robot-frame commands are:

```text
left stick up     surge forward
left stick left   sway left
right stick up    heave up
right stick left  positive yaw, nose left
```

Verify every independent stop layer:

1. Release LB: teleop zero within 250 ms.
2. Disconnect Xbox/Foxglove/network: teleop zero within 250 ms.
3. Stop the teleop node: mixer zero within 500 ms.
4. Stop mixer command input: ESP bridge sends neutral within 500 ms.
5. Stop `esp_bridge`: ESP disarms after approximately 300 ms.
6. Exercise the physical kill switch independently of ROS.

Restart direct mode as needed and disarm after the check:

```bash
ros2 service call /tardigrade/set_armed \
  tardigrade_interfaces/srv/SetArmed "{armed: false}"
```

Any wrong sign or failed stop blocks the wet test.

## 10. Low-Authority Direct Wet Test

Keep the robot tethered and start a bag before arming:

```bash
ros2 bag record -o pool_direct_01 \
  /joy \
  /tardigrade/teleop/enabled \
  /tardigrade/cmd_vel \
  /tardigrade/thrusters/cmd \
  /tardigrade/esp/state \
  /vectornav/imu \
  /tardigrade/sensors/imu \
  /zed/zed_node/odom \
  /tardigrade/state/odometry/filtered \
  /tf /tf_static
```

Arm, then test in this order with brief taps:

1. Heave
2. Yaw
3. Surge
4. Sway

Release LB between every observation. Do not combine axes until every
individual sign is accepted. The Xbox mapper uses conservative body-command
limits, and the ESP independently clamps every final thruster command to
`±0.30` of ESC command range. That number is not a linear percentage of thrust.

Direct teleop and recording are sufficient for a successful pool checkout.

## 11. Closed-Loop Velocity Tuning

The modern controller requires all sensor gates and direct checkout gates to
pass. Disarm and stop `pool_direct`, then start:

```bash
ros2 launch tardigrade_bringup pool_assisted.launch.py
```

The command mux selects `manual` in this launch. It requires fresh `/joy`, the
LB deadman, stamped manual velocity, and filtered odometry. Any stale input
disables the controller and clears its integrators. Start with low output
limits and tune one velocity/rate axis at a time. All parameters are on
`/velocity_wrench_controller`, for example:

```bash
ros2 param set /velocity_wrench_controller yaw.ki 0.0
ros2 param set /velocity_wrench_controller yaw.output_limit 5.0
ros2 param set /velocity_wrench_controller yaw.kp 2.0
```

Watch:

```text
/tardigrade/control/velocity_setpoint_enabled
/tardigrade/control/velocity_setpoint
/tardigrade/control/wrench_command
/tardigrade/control/allocation_status
/tardigrade/actuators/thruster_forces
/tardigrade/actuators/thruster_commands
```

Tune one axis at a time in this order:

1. Heave velocity
2. Yaw rate
3. Surge velocity
4. Sway velocity
5. Roll and pitch rates during restrained tests

For each axis, keep Ki and drag feed-forward at zero initially. Raise Kp
gradually, then add filtered Kd only if measured acceleration is clean enough.
Add Ki only for repeatable standing error. Stop immediately if feedback has the
wrong sign, the allocation residual stays large, a thruster saturates
continuously, or estimator/command freshness changes.

Record every powered PID attempt in a separate bag. Save the session values:

```bash
ros2 param dump /velocity_wrench_controller \
  > /tmp/pool_tuned_gains.yaml
```

Review that file and manually copy accepted values into
`src/tardigrade_control/config/control.yaml`. Do not blindly overwrite the
versioned source file. Tune pose-to-velocity gains separately after the inner
velocity/rate loops are stable.

## 12. Optional Depth Experiment

Depth hold is not required. First leave every PID loop disabled and record
filtered Z while stationary and during a short controlled vertical movement.
Accept ZED depth only if it is:

- correctly signed;
- smooth while stationary;
- responsive to short vertical motion;
- free of jumps, freezes, and tracking resets.

If any condition fails, keep depth disabled. A pressure sensor is the proper
long-term depth measurement. If the test passes, enable depth alone and retain
positive buoyancy.

## 13. Shutdown

At the end of every powered attempt:

If manual closed-loop mode is running, release the LB deadman and confirm
`/tardigrade/control/velocity_setpoint_enabled` becomes false.

In either direct or assisted mode, disarm:

```bash
ros2 service call /tardigrade/set_armed \
  tardigrade_interfaces/srv/SetArmed "{armed: false}"
```

Then:

1. Confirm eight zero thruster commands.
2. Use the physical kill switch/remove thruster power.
3. Stop the load-bearing launch.
4. Stop rosbag cleanly with Ctrl-C.
5. Preserve notes linking the bag name to gains, enabled axes, and observations.

## Go/No-Go Summary

Do not conduct a powered PID or wet test unless all applicable items are true:

- clean ROS build and tests pass;
- ESP protocol self-test and firmware build pass;
- stable serial identities are confirmed;
- Foxglove `/joy` mapping and LB are confirmed;
- every stale/disconnect path produces zero;
- ESP disarms on bridge loss;
- physical kill switch is rehearsed;
- thruster slots and signs are accepted;
- IMU mounting, axes, timestamps, and freshness are correct;
- ZED and EKF output are stable;
- exactly one command mode and one ESP serial owner are running.
