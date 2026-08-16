# Roll PID Tuning

This is the focused procedure for tuning **roll only** with the Xbox controller
on the MacBook, the VectorNav providing attitude and angular rate, and the
Jetson controller commanding the vertical thrusters.

```text
Xbox/Foxglove /joy
  -> /tardigrade/cmd_vel/manual
  -> roll PID + filtered odometry
  -> /tardigrade/cmd_vel angular.x
  -> thruster mixer
  -> vertical thruster slots 3, 4, 5, 6
  -> ESP arm, clamp, PWM, watchdog
```

The ESP does not run this PID. It remains the actuator and independent safety
layer. Do not run `pose_bridge.py`, `gcs_server.py --ros`, or synthetic pose.

## Hard prerequisites

Do not start powered roll tuning until all of these are true:

- direct Xbox teleop has been tested successfully;
- `/joy` publishes continuously and LB is `buttons[4]`;
- all eight thruster slots and directions have been accepted;
- the physical kill switch has been rehearsed;
- the robot is tethered, restrained for the first response, and submerged;
- the VectorNav frame/sign hand test passes;
- only one ESP bridge and one load-bearing command mode are running.

Propellers should not be meaningfully operated dry. Keep the ESP disarmed and
thruster power disconnected during software and sensor checks.

## One-time build check

After pulling the roll-tuning changes onto the Jetson, rebuild because
`PidDebug.msg` is a generated ROS interface:

```bash
cd /ws
./build.sh --hardware --clean
source /ws/install/setup.bash
ros2 interface show tardigrade_interfaces/msg/PidDebug
```

The final command must print the fields of the message. An import error for
`PidDebug` means the interface package was not rebuilt or the shell was not
re-sourced.

## Start the system

Use a separate Jetson terminal for each section. Run `source
/ws/install/setup.bash` in every new terminal.

### Terminal A: ZED

```bash
source /ws/install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zed publish_tf:=false
```

### Terminal B: VectorNav conversion

```bash
source /ws/install/setup.bash
ros2 launch tardigrade_bringup zed_vectornav_state.launch.py \
  port:=/dev/serial/by-id/usb-FTDI_USB-RS232-WE_AV0LN035-if00-port0 \
  baud:=115200 \
  use_zed_orientation_if_imu_stale:=false
```

### Terminal C: filtered odometry and TF

```bash
source /ws/install/setup.bash
ros2 launch tardigrade_bringup zed_vectornav_ekf.launch.py
```

### Terminal D: Foxglove transport

```bash
source /ws/install/setup.bash
ros2 launch tardigrade_bringup foxglove_rosbridge.launch.py
```

Connect Foxglove Desktop on the MacBook using the Rosbridge connection:

```text
ws://JETSON_IP:9090
```

### Terminal E: assisted command path

Stop `pool_direct`, `thruster_checkout_real`, and every standalone ESP bridge
before starting this launch:

```bash
source /ws/install/setup.bash
ros2 launch tardigrade_bringup pool_assisted.launch.py
```

This starts the Xbox mapper, Jetson PID controller, thruster mixer, and the
only ESP bridge. All PID axes start disabled.

## Load the Foxglove roll station

Import this layout into Foxglove:

```text
foxglove/layouts/roll_pid_tuning.json
```

The left side contains four service panels, in order:

1. disable all PID axes;
2. enable roll and explicitly disable pitch, yaw, and depth;
3. edit and apply roll `kp`, `ki`, `kd`, and `output_limit`;
4. emergency software disarm.

The status area shows ESP armed/link state, LB, controller gate, roll PID,
odometry freshness, command freshness, and saturation. The plots show roll
target versus measurement, error and roll rate, P/I/D terms and output, and
the four vertical thruster commands. The raw roll-debug panel shows the gains
that the controller actually applied.

The third-party Joystick panel must be added separately and configured as:

```text
Data Source:   Gamepad
Gamepad ID:    0
Publish Mode:  enabled
Pub Joy Topic: /joy
Layout:        Xbox
```

## Prove the unpowered data path

With the ESP disarmed and thruster power disconnected:

```bash
ros2 topic hz /joy
ros2 topic hz /tardigrade/state/odometry/filtered
ros2 topic hz /tardigrade/esp/state
ros2 service list | grep /tardigrade/control
```

Required results:

- `/joy` is comfortably faster than 4 Hz;
- filtered odometry is continuous and finite;
- ESP telemetry is continuous with `link_ok: true`;
- both gain and axis-enable services exist.

In Foxglove, call **disable all axes**. Hold LB while the ESP remains disarmed.
The LB, controller, odometry, and command indicators should become green. Tilt
the robot by hand:

- left side up must increase measured roll;
- moving the left side upward must produce positive roll rate;
- setpoint and measurement must be smooth, with no jumps or NaNs.

Release LB. Controller and roll-active indicators must turn off and all eight
thruster commands must become zero. A wrong sign, stale indicator, or nonzero
command after LB release blocks powered tuning.

## Apply the first roll-only configuration

Leave LB released and call **disable all axes** before changing gains. In the
roll-gains service panel, begin with:

```json
{"axis":"roll","kp":0.2,"ki":0.0,"kd":0.0,"output_limit":0.1}
```

Call the service and verify that it returns `success: true`. Then call the
**enable roll only** service and verify its response. Pitch, yaw, and depth
must remain false.

These are deliberately gentle starting values, not validated final gains.
`output_limit` is a maximum normalized roll command, not a percentage of
thrust. A small Kp can also produce a command below the motors' usable
deadband; raise gains and authority gradually rather than jumping directly to
the ESP's `0.30` cap.

The equivalent terminal commands are:

```bash
ros2 service call /tardigrade/control/set_axes_enabled \
  tardigrade_interfaces/srv/SetControlAxes \
  "{roll: false, pitch: false, yaw: false, depth: false}"

ros2 service call /tardigrade/control/set_pid_gains \
  tardigrade_interfaces/srv/SetPidGains \
  "{axis: roll, kp: 0.2, ki: 0.0, kd: 0.0, output_limit: 0.1}"

ros2 service call /tardigrade/control/set_axes_enabled \
  tardigrade_interfaces/srv/SetControlAxes \
  "{roll: true, pitch: false, yaw: false, depth: false}"
```

## Record before arming

Start a new bag for every gain combination. Put the gains in the directory
name so recordings are not confused later:

```bash
cd /ws
ros2 bag record -o roll_kp020_ki000_kd000_lim010_01 \
  /joy \
  /tardigrade/teleop/enabled \
  /vectornav/imu \
  /tardigrade/sensors/imu \
  /tardigrade/state/odometry/filtered \
  /tardigrade/cmd_vel/manual \
  /tardigrade/cmd_vel \
  /tardigrade/thrusters/cmd \
  /tardigrade/esp/state \
  /tardigrade/control/enabled \
  /tardigrade/control/odometry_fresh \
  /tardigrade/control/command_fresh \
  /tardigrade/control/roll/enabled \
  /tardigrade/control/roll/debug \
  /tf /tf_static
```

## First powered sign test

The robot must be submerged, tethered, initially restrained, and approximately
level. One person owns the physical kill switch. Arm deliberately from a
Jetson terminal:

```bash
ros2 service call /tardigrade/set_armed \
  tardigrade_interfaces/srv/SetArmed "{armed: true}"
```

Confirm `armed: true` and `link_ok: true` in Foxglove. Hold LB briefly while
the robot is level. The controller captures the current roll as its target
when LB first opens the control gate.

Apply a small roll disturbance by hand and observe the first response:

- the PID output and thrusters must oppose the disturbance;
- measurement should move back toward setpoint;
- releasing LB must immediately command zero.

If the controller pushes farther into the disturbance, **release LB, disarm,
and stop**. That is a frame, sensor-sign, mixer-sign, or thruster-polarity
problem. It cannot be repaired by changing PID gains.

## Tune P, then D

Keep `ki` at zero for today's initial tuning.

1. With LB released, edit Kp in the gain service panel.
2. Call the service and check `success: true`.
3. Start a newly named bag.
4. Put the robot approximately level, then hold LB to capture the target.
5. Apply a repeatable small disturbance and release it.
6. Release LB after the observation; next activation captures a new target.

Increase Kp by roughly 25–50% per attempt. For example:

```text
0.20 -> 0.30 -> 0.45 -> 0.65
```

These are exploration points, not required values. Stop increasing when the
robot returns usefully but begins to overshoot or oscillate. Back Kp down by
roughly 20–30% from the oscillatory setting.

If the output is continuously saturated at `0.10` and the feedback direction
is proven correct, increase `output_limit` cautiously, for example to `0.12`
and then `0.15`. Do not exceed `0.20` during initial roll tuning. Saturation is
not by itself permission to increase authority: first check the sensor and
thruster signs.

Once P provides a clear restoring response, add D in small steps, commonly
`0.02` to `0.05` at a time. D should reduce overshoot and braking time. Too
much D makes the robot sluggish or noisy.

Leave Ki at zero unless repeated tests show a consistent steady roll error
that P and D cannot remove. A stable PD controller is a successful first-day
result.

## Reading the plots

A healthy response is:

```text
disturbance
  -> measurement leaves the setpoint
  -> error and output appear with the correcting sign
  -> measurement returns without growing oscillation
  -> output returns toward zero
```

Interpret the panels as follows:

- **target / measurement:** response speed, overshoot, and settling;
- **error / roll rate:** disturbance size and angular motion;
- **P/I/D/output:** which term is commanding the robot;
- **vertical thrusters:** whether left and right vertical groups oppose each
  other for roll;
- **saturation:** whether the requested output exceeded the configured limit.

Stop immediately for stale odometry, stale commands, repeated tracking jumps,
continuous saturation, growing oscillation, wrong correction direction, loss
of ESP link, or any thruster command that remains nonzero after LB release.

## Shutdown and save the result

Release LB, disable all loops, and disarm:

```bash
ros2 service call /tardigrade/control/set_axes_enabled \
  tardigrade_interfaces/srv/SetControlAxes \
  "{roll: false, pitch: false, yaw: false, depth: false}"

ros2 service call /tardigrade/set_armed \
  tardigrade_interfaces/srv/SetArmed "{armed: false}"
```

Confirm eight zero thruster commands, stop the bag, and use the physical kill
switch/remove thruster power.

Live service changes disappear when the controller restarts. Dump the final
running parameters for review:

```bash
ros2 param dump /depth_attitude_controller \
  > /tmp/roll_pid_tuned.yaml
```

Manually copy only the accepted `roll_kp`, `roll_ki`, `roll_kd`, and
`max_roll_command` values into
`src/tardigrade_esp/config/controller_gains.yaml`. Do not replace the source
YAML blindly with the complete parameter dump.
