# Jetson ZED PX4 Startup Runbook

This is the detailed hardware runbook for the Jetson + ZED + VectorNav +
Pixhawk bench path. It is intentionally more detailed than the README because
most failures here are hardware, PX4 parameter, serial-device, or timing
problems.

Current objective:

1. Run the Jetson hardware container.
2. Run the ZED wrapper.
3. Run the VectorNav driver.
4. Publish fused robot odometry on `/tardigrade/state/odometry`.
5. Send that odometry to PX4 over USB MAVLink.
6. Attempt Offboard/external-control mode.
7. Attempt arming.
8. Use terminal diagnostics to understand what PX4 is missing.

## Mental Model

Prompts:

```text
auv@jetson:~/Developer/tardigrade_ws$   Jetson host shell
root@jetson:/ws#                        Docker container shell
```

Paths:

```text
~/Developer/tardigrade_ws   repo on the Jetson host
/ws                         same repo mounted inside the container
```

Common devices:

```text
/dev/ttyACM0                                                       Pixhawk USB
/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00              Pixhawk stable symlink
/dev/ttyUSB0                                                       VectorNav USB serial adapter
/dev/serial/by-id/usb-FTDI_USB-RS232-WE_AV0LN035-if00-port0        VectorNav stable symlink
```

Do not run `docker compose` or `docker exec` from inside the container. Docker
commands are host commands.

## Repository State

Update the checkout on the Jetson host:

```bash
cd ~/Developer/tardigrade_ws
git pull
git submodule update --init --recursive
```

The ZED wrapper is tracked as:

```text
src/zed-ros2-wrapper
```

The matching ZED interfaces package is nested inside the wrapper:

```text
src/zed-ros2-wrapper/zed-ros2-interfaces
```

Do not keep a duplicate top-level `src/zed-ros2-interfaces`. It causes Colcon
to fail with duplicate package names. If an old checkout has it:

```bash
rm -rf src/zed-ros2-interfaces
git submodule update --init --recursive
```

If Colcon still remembers the removed package:

```bash
rm -rf build install log
```

## One-Time Jetson Checks

The Jetson must have Docker Compose and the NVIDIA runtime available.

```bash
docker compose version
sudo docker info | grep -i runtime
```

Expected runtime list includes `nvidia`. The Compose Jetson override explicitly
sets:

```yaml
runtime: nvidia
```

The Jetson should also have the ZED SDK installed:

```bash
sudo grep -n "PACKAGE_VERSION" /usr/local/zed/zed-config-version.cmake
```

Known target for this setup:

```text
ZED SDK 4.0.8
zed-ros2-wrapper humble-v4.0.8
```

## Start The Container

From the Jetson host:

```bash
cd ~/Developer/tardigrade_ws
sudo WORKSPACE=/home/auv/Developer/tardigrade_ws \
  docker compose -f docker/compose.yaml -f docker/compose.jetson.yaml run --rm tardigrade
```

Expected prompt:

```text
root@jetson:/ws#
```

Open extra container terminals from new Jetson SSH sessions:

```bash
sudo docker exec -it tardigrade-foxy bash
```

Inside every container shell:

```bash
cd /ws
source install/setup.bash
```

If Compose is not working, use the fallback script from the Jetson host:

```bash
cd ~/Developer/tardigrade_ws
sudo WORKSPACE=/home/auv/Developer/tardigrade_ws ./docker/run_jetson_zed.sh
```

## Build The Workspace

Inside the container:

```bash
cd /ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

After Python-only changes:

```bash
colcon build --symlink-install --packages-select tardigrade_state_estimation tardigrade_px4 tardigrade_bringup
source install/setup.bash
```

Warnings about old `/ws/install/...` paths after deleting `install/` usually
mean the shell sourced an old workspace before the clean. Open a fresh container
shell or continue if the build is otherwise progressing.

## Hardware Checks

On the Jetson host:

```bash
sudo nvpmodel -m 0
sudo jetson_clocks
lsusb
lsusb -t
```

The ZED should show USB3 speed:

```text
5000M
```

If it shows `480M`, fix cable/port/power before debugging ROS.

Inside the container:

```bash
ls -l /dev/serial/by-id/
ls -l /dev/ttyUSB* /dev/ttyACM*
```

Expected:

```text
PX4 FMU -> ../../ttyACM0
FTDI USB-RS232 -> ../../ttyUSB0
```

## ROS Interfaces

```text
/zed/zed_node/pose
```

ZED pose. This is the local position source.

```text
/vectornav/imu
```

VectorNav IMU. This provides orientation and angular velocity.

```text
/tardigrade/state/odometry
```

Robot odometry consumed by `mavlink_pixhawk_interface`.

```text
/tardigrade/status
```

Operator status from the Pixhawk interface. This is the main terminal-friendly
replacement for QGroundControl warning banners.

```text
/tardigrade/set_external_control
```

Service that enables the offboard setpoint stream and sends the PX4 Offboard
mode command.

```text
/tardigrade/set_armed
```

Service that sends the MAVLink arm/disarm command.

## Terminal Layout

Use at least four terminals.

### Terminal 1: ZED

Inside the container:

```bash
cd /ws
source install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed
```

Good checks from another terminal:

```bash
ros2 topic hz /zed/zed_node/pose
ros2 topic echo --once /zed/zed_node/pose
```

Stop here if the ZED reports:

```text
CAMERA NOT DETECTED
CAMERA_REBOOTING
```

For USB instability, check on the host:

```bash
sudo dmesg -T | tail -80
```

Repeated UVC `-71` errors usually mean cable, port, power, or hub trouble.

### Terminal 2: ZED + VectorNav Odometry

Inside the container:

```bash
cd /ws
source install/setup.bash
ros2 launch tardigrade_bringup zed_vectornav_state.launch.py \
  port:=/dev/serial/by-id/usb-FTDI_USB-RS232-WE_AV0LN035-if00-port0 \
  baud:=115200
```

Good VectorNav signs:

```text
Connected to ... @ 115200 baud
Model: VN-100T-CR
Publishing fused odometry: /tardigrade/state/odometry
```

Check:

```bash
ros2 topic hz /vectornav/imu
ros2 topic hz /tardigrade/state/odometry
```

`zed_vectornav_odometry` is a simple field-level fusion node:

```text
position          ZED
orientation       VectorNav when fresh, otherwise ZED fallback
angular velocity  VectorNav
linear velocity   not estimated
```

If the VectorNav is unavailable and you only need ZED pose:

```bash
ros2 launch tardigrade_bringup zed_state.launch.py
```

### Terminal 3: Pixhawk MAVLink Interface

Inside the container:

```bash
cd /ws
source install/setup.bash
ros2 run tardigrade_px4 mavlink_pixhawk_interface --ros-args \
  -p device:=/dev/ttyACM0 \
  -p baudrate:=921600 \
  -p configure_px4_params:=true
```

Keep this terminal visible. It prints PX4 `STATUSTEXT`, command ACKs, visual
odometry age, local position, estimator status, and parameter values.

Good signs:

```text
Opening MAVLink serial: /dev/ttyACM0 @ 921600
Publishing: /tardigrade/status
Sending visual odometry from: /tardigrade/state/odometry
PX4 runtime parameter configuration is enabled
PX4 debug: visual_odom_age_ms=...
PX4 debug: visual_odom_sent=...
PX4 debug: local_position=age_ms:...
PX4 debug: estimator=...
```

Do not continue to arming if:

```text
local_position=none
visual_odom_age_ms is stale or missing
```

### Terminal 4: Status, Offboard, Arm

Inside the container:

```bash
cd /ws
source install/setup.bash
ros2 topic echo /tardigrade/status
```

Confirm:

```text
px4_connected: true
```

Stop echo with `Ctrl+C`.

Enable external control:

```bash
ros2 service call /tardigrade/set_external_control tardigrade_interfaces/srv/SetExternalControl "{enabled: true}"
```

PX4 Offboard normally requires a setpoint stream before or while entering
Offboard. If the first attempt does not stick, wait two seconds and send the
external-control request again:

```bash
sleep 2
ros2 service call /tardigrade/set_external_control tardigrade_interfaces/srv/SetExternalControl "{enabled: true}"
```

Check status:

```bash
ros2 topic echo --once /tardigrade/status
```

Useful values:

```text
last_ack_command=176
last_ack_result=0
custom_mode=6
```

`176/0` means PX4 accepted the Offboard mode command packet. `custom_mode=6`
means PX4 is reporting Offboard mode in the heartbeat.

Arm:

```bash
ros2 service call /tardigrade/set_armed tardigrade_interfaces/srv/SetArmed "{armed: true}"
sleep 1
ros2 topic echo --once /tardigrade/status
```

Useful arm ACKs:

```text
last_ack_command=400
last_ack_result=0   accepted
last_ack_result=1   temporarily rejected
last_ack_result=2   denied
last_ack_result=4   failed
```

Disarm:

```bash
ros2 service call /tardigrade/set_armed tardigrade_interfaces/srv/SetArmed "{armed: false}"
```

Important: the ROS service response only confirms that the Jetson sent the
MAVLink command. PX4 acceptance is shown by command ACKs, heartbeat mode, and
`armed: true` in `/tardigrade/status`.

## PX4 Parameter Notes

The current bench Pixhawk has had parameter storage trouble:

```text
param save
ERROR [parameters] parameter export to /fs/mtd_params failed
```

So the startup command uses:

```text
-p configure_px4_params:=true
```

This sends required params into RAM each startup. They are lost when the Pixhawk
reboots.

Parameters sent by the node include:

```text
COM_RC_IN_MODE=4
COM_ARM_WO_GPS=1
COM_ARM_MIS_REQ=0
EKF2_EV_CTRL=3
EKF2_HGT_REF=3
EKF2_EV_QMIN=0
```

Verify in Pixhawk interface logs:

```text
px4_param_values=COM_RC_IN_MODE=4,...
```

## Offboard Requirements

PX4 Offboard is not just a mode command. PX4 expects:

- a live companion link,
- a continuous supported setpoint stream, normally above 2 Hz,
- the setpoint stream to exist before or while entering Offboard,
- estimator/local-position validity if the selected setpoint mode needs it,
- no blocking failsafe/prearm condition.

In this repo, `/tardigrade/set_external_control` enables the setpoint stream
and sends the Offboard command. If Offboard does not stick, call it once, wait
two seconds, then call it again. If that works, the code should be changed to
delay the mode command until the stream has been running.

## Terminal Debugging Without QGroundControl

Watch the Pixhawk interface terminal for:

```text
PX4 status text: ...
PX4 debug: ...
```

Watch `/tardigrade/status` for:

```text
px4_connected
armed
external_control_enabled
base_mode
custom_mode
last_ack_command
last_ack_result
last_status_text
visual_odom_age_ms
visual_odom_sent
```

MAVLink command ACK results:

```text
0 accepted
1 temporarily rejected
2 denied
3 unsupported
4 failed
```

Common command IDs:

```text
176 MAV_CMD_DO_SET_MODE
400 MAV_CMD_COMPONENT_ARM_DISARM
```

## Quick Failure Map

```text
docker compose: command not found
```

Install the Docker Compose plugin for the Jetson's Docker install, or use
`docker/run_jetson_zed.sh` as the fallback.

```text
open /home/auv/docker/compose.yaml: no such file or directory
```

You ran Compose from the wrong directory. Run:

```bash
cd ~/Developer/tardigrade_ws
```

```text
Duplicate package names: zed_interfaces
```

Remove the top-level duplicate:

```bash
rm -rf src/zed-ros2-interfaces
rm -rf build install log
git submodule update --init --recursive
```

```text
VectorNav timeout at 921600
```

Use `baud:=115200`. The observed VN-100T-CR bench unit connects at 115200.

```text
/tardigrade/state/odometry not publishing
```

Check `/zed/zed_node/pose` and `/vectornav/imu`. If VectorNav is intentionally
absent, run `zed_state.launch.py`.

```text
px4_connected: false
```

Check Pixhawk device path and make sure only one process has `/dev/ttyACM0`
open.

```text
local_position=none
```

PX4 is connected but not producing local position from the sent odometry. Do not
arm yet. Check visual odometry freshness, EKF2 vision params, and frame data.

```text
last_ack_command=176, last_ack_result=0, but no blue/offboard indication
```

PX4 accepted the mode command packet, but may not have stayed in Offboard.
Check `custom_mode=6`, setpoint stream timing, and call external control again
after the stream has been active for two seconds.

```text
last_ack_command=400, last_ack_result=1 or 2
```

PX4 rejected arming. Read `last_status_text` and the Pixhawk interface terminal.
The reason is usually a preflight check, safety, estimator, local position, or
mode/failsafe requirement.

## Teleop Dry Run

Teleop commands use:

```text
/tardigrade/cmd_vel
```

Convention:

```text
linear.x   forward/back
linear.y   left/right
linear.z   up/down
angular.z  yaw left/right
```

Start velocity mode with default zero clamps for a no-motion command-path test:

```bash
ros2 run tardigrade_px4 mavlink_pixhawk_interface --ros-args \
  -p device:=/dev/ttyACM0 \
  -p baudrate:=921600 \
  -p configure_px4_params:=true \
  -p offboard_setpoint_mode:=velocity
```

Run keyboard teleop:

```bash
ros2 run tardigrade_px4 keyboard_cmd_vel
```

Expected Pixhawk interface debug:

```text
cmd_vel_received=...
```

Do not command real thrust until `config/thruster_map.yaml` and
`docs/thruster_mapping.md` have been checked against the vehicle.
