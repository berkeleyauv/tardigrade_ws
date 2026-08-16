# Pool Test Runbook: Xbox, Foxglove, and PID

This is the go/no-go procedure for the phased pool test. Direct Xbox teleop
and recording are the required outcome. Closed-loop control is optional and
must pass every earlier gate.

The physical kill switch must be reachable by one named person during every
powered test. Never cool T200 thrusters by running them dry for more than a
brief tap.

## Critical controller separation

For the Jetson-controller path, `esp_bridge` must be the only owner of the ESP
serial port. Do **not** run any of the following at the same time:

- `gcs_server.py`, especially `gcs_server.py --ros`
- `pose_bridge.py`
- `esp_thruster_bridge`
- a second `esp_bridge`
- `/tardigrade/test/synthetic_pose`

The firmware still contains its transitional onboard PID. Sending it pose can
make it healthy and able to compete with Jetson per-thruster commands. Normal
ZED, VectorNav, EKF, and Foxglove monitoring are safe because `esp_bridge`
does not forward their pose to the ESP.

## Gate 1: clean builds and tests

Inside the ROS container:

```bash
cd /ws
./build.sh --pkg tardigrade_interfaces
./build.sh --pkg tardigrade_teleop
./build.sh --pkg tardigrade_esp
source install/setup.bash
colcon test --packages-select \
  tardigrade_interfaces tardigrade_teleop tardigrade_esp
colcon test-result --verbose
ros2 launch tardigrade_esp xbox_direct_real.launch.py --show-args
ros2 launch tardigrade_esp xbox_assisted_real.launch.py --show-args
```

In the firmware repository:

```bash
cd tardigrade_firmware/firmware
pio run
cd ..
python3 tools/tardigrade_protocol.py
```

Any build or test failure blocks powered testing. This does not replace the
still-outstanding deliberate hardware-watchdog trip in the firmware bench
checklist.

## Gate 2: identify both serial devices and the Xbox

```bash
ls -l /dev/serial/by-id/
ls -l /dev/input/js*
ros2 pkg executables joy
```

Use stable `/dev/serial/by-id/...` paths for the ESP and VectorNav. Start only
the joystick driver and inspect its raw messages before connecting thruster
power:

```bash
ros2 run joy joy_node --ros-args -p device_id:=0
ros2 topic echo /joy
```

Confirm the expected standard Xbox mapping:

- axis 0: left stick horizontal, positive left
- axis 1: left stick vertical, positive forward/up
- axis 3: right stick horizontal, positive left
- axis 4: right stick vertical, positive forward/up
- button 4: LB deadman

If the hardware differs, pass the mapper's axis/button parameters rather than
memorizing reversed controls.

## Gate 3: Foxglove and sensors, unpowered

Start the ZED wrapper:

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed
```

Start VectorNav plus the comparison odometry path, using its stable serial
path:

```bash
ros2 launch tardigrade_bringup zed_vectornav_state.launch.py \
  port:=/dev/serial/by-id/YOUR_VECTORNAV_DEVICE
```

Start the filtered EKF output used by the controller:

```bash
ros2 launch tardigrade_bringup zed_vectornav_ekf.launch.py
```

Start rosbridge:

```bash
ros2 launch tardigrade_bringup foxglove_rosbridge.launch.py
```

Connect Foxglove's Rosbridge connection to `ws://JETSON_IP:9090` and import
`foxglove/layouts/pid_tuning.json`.

Before proceeding, verify:

- `/vectornav/imu` is continuous and roll, pitch, yaw-rate signs are correct.
- `/zed/zed_node/odom` changes smoothly and has no tracking resets.
- `/tardigrade/state/odometry/filtered` is at about 30 Hz.
- Stationary orientation is plausible and TF has no repeated errors.

## Gate 4: direct Xbox checkout, thruster power disconnected

Stop the standalone `joy_node` from Gate 2. Start the complete direct path:

```bash
ros2 launch tardigrade_esp xbox_direct_real.launch.py \
  serial_port:=/dev/serial/by-id/YOUR_ESP_DEVICE device_id:=0
```

The path is:

```text
Xbox -> /tardigrade/cmd_vel -> mixer
     -> /tardigrade/thrusters/cmd -> esp_bridge -> ESP
```

Check the graph and neutral state:

```bash
ros2 topic info /tardigrade/cmd_vel --verbose
ros2 topic info /tardigrade/thrusters/cmd --verbose
ros2 topic echo /tardigrade/thrusters/cmd --once
ros2 topic hz /tardigrade/esp/state
```

There must be exactly one publisher in the load-bearing command chain and the
first thruster command must contain eight zeros.

Hold LB and test one stick axis at a time. Release LB after each movement and
confirm eight zeros. Disconnect the Xbox while LB is held and confirm zeros
within 250 ms. Stop `xbox_cmd_vel` and confirm mixer zeros within 0.5 s. Stop
the mixer while armed only during the restrained safety check and confirm
`esp_bridge` forces neutral within 0.5 s.

## Gate 5: restrained direction and failsafe checks

With propellers removed or the vehicle securely restrained, arm through the
service and use brief, low-authority taps:

```bash
ros2 service call /tardigrade/set_armed \
  tardigrade_interfaces/srv/SetArmed "{armed: true}"
```

Expected body-frame directions are forward, left, up, and counter-clockwise
yaw viewed from above. Correct mapping or wiring errors; do not compensate in
the pilot's muscle memory.

Verify all stop layers:

1. Release LB: eight zeros immediately.
2. Disconnect Xbox: eight zeros within 250 ms.
3. Stop teleop: mixer zeros within 0.5 s.
4. Stop mixer: bridge zeros within 0.5 s.
5. Stop `esp_bridge`: ESP disarms in approximately 300 ms.
6. Exercise the physical kill switch independently of ROS.

Disarm before recovery or troubleshooting:

```bash
ros2 service call /tardigrade/set_armed \
  tardigrade_interfaces/srv/SetArmed "{armed: false}"
```

Any failed stop check blocks the wet test.

## Gate 6: low-authority direct wet test and recording

Keep the vehicle tethered. Begin in direct mode, validate heave and yaw with
brief taps, then surge and sway. Do not combine axes until each sign is known.

Record every powered attempt in a separate directory:

```bash
ros2 bag record -o pool_direct_01 \
  /joy /tardigrade/teleop/enabled \
  /tardigrade/cmd_vel /tardigrade/thrusters/cmd \
  /tardigrade/esp/state /vectornav/imu \
  /zed/zed_node/odom /tardigrade/state/odometry/filtered \
  /tf /tf_static
```

Playback is:

```bash
ros2 bag play pool_direct_01
```

## Gate 7: assisted tuning, one axis at a time

Disarm and stop the direct launch. Keep the sensor and rosbridge processes
running, then start the assisted path:

```bash
ros2 launch tardigrade_esp xbox_assisted_real.launch.py \
  serial_port:=/dev/serial/by-id/YOUR_ESP_DEVICE device_id:=0
```

All four PID axes start disabled. Direct surge, sway, heave, and yaw pass
through only while LB is held, but the controller also requires fresh EKF
odometry. In Foxglove, edit and call `set_axes_enabled`; never enable a new
axis before disabling the previous test configuration.

Command-line equivalents are:

```bash
# Roll only
ros2 service call /tardigrade/control/set_axes_enabled \
  tardigrade_interfaces/srv/SetControlAxes \
  "{roll: true, pitch: false, yaw: false, depth: false}"

# Change roll gains and normalized output limit
ros2 service call /tardigrade/control/set_pid_gains \
  tardigrade_interfaces/srv/SetPidGains \
  "{axis: roll, kp: 0.8, ki: 0.0, kd: 0.15, output_limit: 0.2}"

# Disable every PID loop
ros2 service call /tardigrade/control/set_axes_enabled \
  tardigrade_interfaces/srv/SetControlAxes \
  "{roll: false, pitch: false, yaw: false, depth: false}"
```

Tune in this order: roll, pitch, yaw. Start with Ki zero. Raise Kp slowly until
the response is useful, add Kd to damp oscillation, and add Ki only for a
repeatable standing error. Watch error, P/I/D terms, output, saturation,
odometry freshness, and all eight thruster commands in Foxglove.

Releasing LB disables the controller and clears its integrators. Stale command
or odometry does the same.

Record each axis separately and dump accepted live values:

```bash
ros2 param dump /depth_attitude_controller > /tmp/pool_tuned_gains.yaml
```

Review the dump and manually copy accepted values into
`src/tardigrade_esp/config/controller_gains.yaml`; do not overwrite the source
file blindly.

## Gate 8: optional ZED depth experiment

Depth is not a required pool-test result. With every PID loop disabled, record
the ZED and filtered z position while stationary and during a short controlled
vertical movement. It must be smooth, correctly signed, and free of tracking
resets. If it drifts, jumps, freezes, or loses tracking, leave depth disabled.

Only after that check may depth be enabled alone. Maintain slight positive
buoyancy so every failsafe surfaces the vehicle. A pressure sensor is the
proper long-term depth measurement.

## Autonomy boundary

Do not run mission nodes during manual tuning. Autonomy later publishes the
same `/tardigrade/cmd_vel/manual` intent and `/tardigrade/depth_target`
setpoint interface, but it must have its own explicit enable heartbeat before
becoming load-bearing. Exactly one manual or autonomous command source may run
at a time.
