# Jetson ZED PX4 Startup Runbook

This is the bench startup path for using the ZED as the local position source
for PX4 arming. The goal is:

1. Run the ZED wrapper.
2. Fuse ZED position with VectorNav attitude into `/tardigrade/state/odometry`.
3. Send that odometry to PX4 over USB MAVLink.
4. Arm the Pixhawk from ROS.

## Mental Model

- Host shell prompt: `auv@jetson:~/Developer/tardigrade_ws$`
- Container shell prompt: `root@jetson:/ws#`
- The repo lives on the Jetson host at `~/Developer/tardigrade_ws`.
- The container sees the same repo mounted at `/ws`.
- Pixhawk USB usually appears as `/dev/ttyACM0`.
- VectorNav USB usually appears as `/dev/serial/by-id/usb-FTDI_USB-RS232-WE_AV0LN035-if00-port0`.
- The ZED must be visible on USB3 at `5000M`, not `480M`.

Do not run `sudo docker exec ...` from inside the container. Docker commands are
host commands.

## One-Time Jetson Setup

Run these on the Jetson host.

```bash
cd ~/Developer/tardigrade_ws
git pull
git submodule update --init --recursive
sudo docker compose -f docker/compose.yaml -f docker/compose.jetson.yaml build
```

The ZED wrapper and interfaces are tracked as submodules:

```bash
src/zed-ros2-wrapper
src/zed-ros2-interfaces
```

They are pinned to the commits for Stereolabs `humble-v4.0.8`, so do not clone
the ZED repositories manually into `src/`. If an older Jetson checkout already
has manually cloned ZED directories, move them aside before running
`git submodule update --init --recursive`.

The Jetson host must have the ZED SDK installed at `/usr/local/zed`. Check it:

```bash
sudo grep -n "PACKAGE_VERSION" /usr/local/zed/zed-config-version.cmake
```

The known-good target for this setup is ZED SDK `4.0.8`.

## Open The Container

Start the main container from the Jetson host:

```bash
cd ~/Developer/tardigrade_ws
sudo WORKSPACE=/home/auv/Developer/tardigrade_ws \
  docker compose -f docker/compose.yaml -f docker/compose.jetson.yaml run --rm tardigrade
```

Expected prompt:

```bash
root@jetson:/ws#
```

Open another shell into the same running container from a second Jetson SSH
terminal:

```bash
sudo docker exec -it tardigrade-foxy bash
```

Inside every new container shell:

```bash
cd /ws
source install/setup.bash
```

Exit one container shell:

```bash
exit
```

Stop the whole container from the host:

```bash
sudo docker stop tardigrade-foxy
```

The helper script remains available if you want NVIDIA runtime auto-detection:

```bash
sudo WORKSPACE=/home/auv/Developer/tardigrade_ws ./docker/run_jetson_zed.sh
```

## Build The ROS Workspace

Inside the container:

```bash
cd /ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

After small Python-only changes, rebuilding only the relevant packages is
usually enough:

```bash
colcon build --symlink-install --packages-select tardigrade_state_estimation tardigrade_px4 tardigrade_bringup
source install/setup.bash
```

## PX4 Parameters In QGroundControl

Set these before trying to arm from ROS:

```text
COM_RC_IN_MODE = Disable manual control
COM_ARM_WO_GPS = Warning only / allow arming without GPS
COM_ARM_MIS_REQ = 0
```

For external vision/local position fusion, PX4 EKF2 must also be configured to
accept vision odometry. In QGroundControl, search the EKF2 parameters and set:

```text
EKF2_EV_CTRL = enable horizontal position fusion, vertical vision fusion, and velocity fusion as needed
EKF2_HGT_REF = Vision
EKF2_EV_POS_X/Y/Z = ZED position relative to the robot body frame, in meters
```

Reboot the Pixhawk after changing EKF2 parameters.

PX4's external position documentation says EKF2 consumes MAVLink `ODOMETRY`
with `MAV_FRAME_LOCAL_FRD` as `vehicle_visual_odometry`, and that external
vision needs to be streamed fast enough for EKF2 to fuse it:

```text
https://docs.px4.io/main/en/ros/external_position_estimation
```

## ZED Hardware Check

Run this on the Jetson host before launching ROS:

```bash
sudo nvpmodel -m 0
sudo jetson_clocks
lsusb
lsusb -t
```

Good ZED USB output includes:

```text
ID 2b03:f582 Technologies, Inc. ZED
5000M
```

Bad output:

```text
480M
```

If the ZED is at `480M`, change to a real USB3 port/cable before continuing.

If the ZED opens and then starts rebooting, check kernel logs on the host:

```bash
sudo dmesg -T | tail -80
```

Repeated `uvcvideo: Failed to set UVC probe control : -71` means the USB video
stream is unstable. Fix cable, port, power, or use a powered USB3 hub before
chasing ROS.

## ROS Interfaces

```text
/zed/zed_node/pose
```

ZED local pose. This is the position source.

```text
/vectornav/imu
```

VectorNav IMU. This is the attitude and angular-velocity source.

```text
/tardigrade/state/odometry
```

Fused odometry from ZED + VectorNav. The MAVLink interface forwards this to PX4
as visual odometry.

```text
/tardigrade/status
```

Status from the Pixhawk interface. Check this for `px4_connected`, `armed`,
`external_control_enabled`, visual odometry age, and MAVLink command ACKs.

```text
/tardigrade/cmd_vel
```

Teleop velocity command in ROS body-frame FLU:

```text
linear.x   forward/back
linear.y   left/right
linear.z   up/down
angular.z  yaw left/right
```

```text
/tardigrade/set_external_control
```

Service that asks PX4 to enter Offboard/external-control mode and starts the
setpoint stream.

```text
/tardigrade/set_armed
```

Service that sends the arm/disarm command to PX4.

## Terminal Layout For Arming Test

Use four terminals. Terminal 1 starts the container. Terminals 2-4 enter the
same running container with:

```bash
sudo docker exec -it tardigrade-foxy bash
cd /ws
source install/setup.bash
```

### Terminal 1: Launch ZED

Host:

```bash
cd ~/Developer/tardigrade_ws
sudo WORKSPACE=/home/auv/Developer/tardigrade_ws \
  docker compose -f docker/compose.yaml -f docker/compose.jetson.yaml run --rm tardigrade
```

Inside container:

```bash
cd /ws
source install/setup.bash
lsusb -t
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed
```

Good signs:

```text
Advertised on topic: /zed/zed_node/pose
Advertised on topic: /zed/zed_node/odom
```

Bad sign:

```text
Connection issue detected: CAMERA_REBOOTING
```

If that happens, stop here and fix ZED USB/power stability.

### Terminal 2: Start VectorNav And Publish Robot Odometry

Inside container:

```bash
cd /ws
source install/setup.bash
ros2 launch tardigrade_bringup zed_vectornav_state.launch.py
```

This converts:

```text
/zed/zed_node/pose + /vectornav/imu
```

into:

```text
/tardigrade/state/odometry
```

The fused odometry uses ZED for position and VectorNav for orientation/angular
velocity. If the VectorNav is not connected and you only need the ZED pose path,
use:

```bash
ros2 launch tardigrade_bringup zed_state.launch.py
```

### Terminal 3: Start Pixhawk MAVLink Interface

Inside container:

```bash
cd /ws
source install/setup.bash
ros2 run tardigrade_px4 mavlink_pixhawk_interface --ros-args \
  -p device:=/dev/ttyACM0 \
  -p baudrate:=921600 \
  -p configure_px4_params:=true
```

This node:

- opens the Pixhawk USB MAVLink connection,
- publishes `/tardigrade/status`,
- provides `/tardigrade/set_armed`,
- provides `/tardigrade/set_external_control`,
- sends `/tardigrade/state/odometry` to PX4 as MAVLink visual odometry,
- sends the required PX4 params into RAM when `configure_px4_params:=true`.

Important: `configure_px4_params:=true` is required on the current bench
Pixhawk because `param save` fails. The runtime params are lost when the Pixhawk
reboots.

Good signs in this terminal:

```text
Publishing: /tardigrade/status
Sending visual odometry from: /tardigrade/state/odometry
PX4 runtime parameter configuration is enabled
Sent required PX4 params in RAM; Pixhawk param save is still not fixed
PX4 debug: ...
```

### Terminal 4: Verify And Arm

Inside container:

```bash
cd /ws
source install/setup.bash
ros2 topic hz /tardigrade/state/odometry
```

Expected:

```text
average rate: ...
```

Stop the rate check with `Ctrl+C`, then:

```bash
ros2 topic echo /tardigrade/status
```

Confirm:

```text
px4_connected: true
```

Stop echo with `Ctrl+C`, then arm:

```bash
ros2 service call /tardigrade/set_external_control tardigrade_interfaces/srv/SetExternalControl "{enabled: true}"
ros2 service call /tardigrade/set_armed tardigrade_interfaces/srv/SetArmed "{armed: true}"
ros2 topic echo /tardigrade/status
```

Success:

```text
px4_connected: true
armed: true
external_control_enabled: true
last_ack_result=0
```

When testing is finished, disarm:

```bash
ros2 service call /tardigrade/set_armed tardigrade_interfaces/srv/SetArmed "{armed: false}"
ros2 topic echo /tardigrade/status
```

If it stays false and shows:

```text
last_ack_command=400
last_ack_result=1
```

PX4 received the arm command but rejected it. Check QGroundControl's current
arming blockers. Do not debug ROS until QGC shows only blockers related to local
position/vision quality.

## Teleop Dry Run

The command path uses standard ROS `geometry_msgs/Twist` on:

```text
/tardigrade/cmd_vel
```

The convention is body-frame ROS FLU:

```text
linear.x   forward/back
linear.y   left/right
linear.z   up/down
angular.z  yaw left/right
```

By default, `mavlink_pixhawk_interface` keeps the proven arming behavior and
uses attitude setpoints. To test the teleop path, restart the Pixhawk interface
in velocity mode:

```bash
ros2 run tardigrade_px4 mavlink_pixhawk_interface --ros-args \
  -p device:=/dev/ttyACM0 \
  -p baudrate:=921600 \
  -p configure_px4_params:=true \
  -p offboard_setpoint_mode:=velocity
```

With no speed parameters, velocity mode is a no-motion dry run: all speed clamps
default to zero. In another container terminal, run:

```bash
ros2 run tardigrade_px4 keyboard_cmd_vel
```

The keyboard node publishes `/tardigrade/cmd_vel`; the Pixhawk interface debug
line should show `cmd_vel_received=...`.

After the dry run passes, add small nonzero clamps one axis at a time. Make the
thrusters physically safe first:

```bash
ros2 run tardigrade_px4 mavlink_pixhawk_interface --ros-args \
  -p device:=/dev/ttyACM0 \
  -p baudrate:=921600 \
  -p configure_px4_params:=true \
  -p offboard_setpoint_mode:=velocity \
  -p max_forward_speed:=0.10 \
  -p max_lateral_speed:=0.10 \
  -p max_vertical_speed:=0.05 \
  -p max_yaw_rate:=0.20
```

Before commanding real thrust, fill out `config/thruster_map.yaml` and follow
`docs/thruster_mapping.md`. This lets the actual wiring differ from the old
diagram while still keeping PX4 actuator setup and ROS body-motion commands
consistent.

Keyboard controls:

```text
w/s     forward/back
j/l     strafe left/right
r/f     up/down
a/d     yaw left/right
space   zero command
Ctrl-C  quit
```

## VectorNav Check

The VectorNav is not enough to create valid local position by itself, but it is
still useful for attitude/IMU data.

Inside container:

```bash
cd /ws
source install/setup.bash
ros2 run vectornav vectornav --ros-args -p port:=/dev/serial/by-id/usb-FTDI_USB-RS232-WE_AV0LN035-if00-port0 -p baud:=115200
```

In another container shell:

```bash
source install/setup.bash
ros2 topic hz /vectornav/imu
```

Expected:

```text
average rate: 20...
```

## Quick Failure Map

```text
ZED not in lsusb
```

Camera is not enumerating. Fix cable, port, or power.

```text
ZED appears at 480M
```

It is running as USB2. Use a real USB3 cable/port.

```text
CAMERA_REBOOTING
```

The camera opened, then reset under stream load. Use max Jetson power, low-load
config, better cable, direct USB3, or powered USB3 hub.

```text
/zed/zed_node/pose not publishing
```

Do not proceed to PX4. Fix ZED first.

```text
/tardigrade/state/odometry not publishing
```

Run `zed_vectornav_state.launch.py` and confirm `/zed/zed_node/pose` and
`/vectornav/imu` are active. If the VectorNav is intentionally disconnected,
run `zed_state.launch.py` instead and confirm `/zed/zed_node/pose` is active.

```text
px4_connected: false
```

Check Pixhawk USB device path. It is usually `/dev/ttyACM0`.

```text
last_ack_result=1 and armed remains false
```

PX4 rejected arming due to preflight checks. Read QGroundControl arming errors.
