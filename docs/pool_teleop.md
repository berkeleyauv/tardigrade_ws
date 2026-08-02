# Pool Keyboard Teleop Runbook

This is the minimum open-loop path for a tethered pool test:

```text
keyboard_cmd_vel -> /tardigrade/cmd_vel -> thruster_mixer
  -> /tardigrade/thrusters/cmd -> esp_bridge -> ESP32 -> thrusters
```

PID, VectorNav, ZED, EKF, autonomy, and synthetic pose are not part of this
procedure. Keep the physical kill switch reachable and do not arm until every
dry check passes.

## 1. Update and build on the Jetson

Inside the ROS container:

```bash
cd /ws
git pull --ff-only
./build.sh --pkg tardigrade_interfaces
./build.sh --pkg tardigrade_esp
./build.sh --pkg tardigrade_teleop
source install/setup.bash
colcon test --packages-select \
  tardigrade_interfaces tardigrade_esp tardigrade_teleop
colcon test-result --verbose
ros2 launch tardigrade_esp teleop_real.launch.py --show-args
```

Any build or test failure blocks powered testing.

## 2. Identify the ESP reliably

```bash
ls -l /dev/serial/by-id/
```

Use the ESP's `/dev/serial/by-id/...` path. Do not assume `ttyUSB0` remains the
ESP after a reboot or power cycle.

## 3. Start the backend with thruster power disconnected

```bash
ros2 launch tardigrade_esp teleop_real.launch.py \
  serial_port:=/dev/serial/by-id/YOUR_ESP_DEVICE
```

Leave this terminal alone. Do not run `esp_thruster_bridge`, `gcs_server.py`,
`pose_bridge.py`, a second `esp_bridge`, or any PID/autonomy launch at the same
time.

In a second sourced terminal, verify the graph and telemetry:

```bash
ros2 node list
ros2 topic info /tardigrade/cmd_vel --verbose
ros2 topic info /tardigrade/thrusters/cmd --verbose
ros2 topic hz /tardigrade/esp/state
ros2 topic echo /tardigrade/esp/state --once
ros2 topic echo /tardigrade/thrusters/cmd --once
```

There must be exactly one mixer and one ESP bridge. Before keyboard teleop
starts, `/tardigrade/thrusters/cmd` must contain eight zeros.

## 4. Start conservative keyboard teleop

In a third sourced terminal:

```bash
ros2 run tardigrade_teleop keyboard_cmd_vel --ros-args \
  -p linear_step:=0.10 -p vertical_step:=0.05 -p yaw_step:=0.10
```

Keys latch their axis until another command changes it or Space zeros the
whole command:

```text
w/s  forward/back        j/l  left/right
r/f  up/down             a/d  yaw left/right
Space  zero all axes     Ctrl-C  stop teleop
```

With thruster power disconnected, press each direction and inspect
`/tardigrade/thrusters/cmd`. Press Space and confirm all eight values return
to zero. Stop keyboard teleop and confirm the mixer returns all eight values
to zero within 0.5 seconds.

## 5. Test physical directions before water

With propellers removed or the vehicle safely restrained, test one body axis
at a time. Confirm physical thrust is:

- `w`: forward
- `j`: left
- `r`: up
- `a`: counter-clockwise/left yaw when viewed from above

If any direction is wrong, disarm and correct the physical map or mix signs.
Do not compensate by memorizing reversed keyboard controls.

## 6. Verify all three stop layers

Perform these checks with propellers removed or the vehicle safely restrained:

1. Press Space while commanding motion; all commands must immediately become
   zero.
2. Stop keyboard teleop; the mixer must publish eight zeros within 0.5 seconds.
3. Command motion again, then stop only `thruster_mixer`; `esp_bridge` must log
   `motor command stale` and force eight neutral commands within 0.5 seconds.
4. Restart the backend, arm, then stop `esp_bridge`; the ESP must disarm through
   its link timeout in approximately 300 ms.
5. Verify the physical kill switch stops thrust independently of ROS.

Any failed stop check blocks powered pool testing.

## 7. Arm and perform the tethered pool test

Zero the keyboard command with Space, keep the kill switch in hand, then arm:

```bash
ros2 service call /tardigrade/set_armed \
  tardigrade_interfaces/srv/SetArmed "{armed: true}"
```

The service response must succeed and `/tardigrade/esp/state.armed` must become
true. Begin with a short `w`, immediately followed by Space. Keep the vehicle
tethered and validate one axis at a time before combining axes.

Disarm before recovery or troubleshooting:

```bash
ros2 service call /tardigrade/set_armed \
  tardigrade_interfaces/srv/SetArmed "{armed: false}"
```

## Important synthetic-pose warning

Do not enable `/tardigrade/test/synthetic_pose` during a teleop session. It
latches the transitional onboard controller until the ESP is reset and can
cause onboard control to override manual `SetMotor` commands.
