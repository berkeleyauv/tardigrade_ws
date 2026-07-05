# Jetson ZED PX4 Startup Runbook

This is the bench startup path for using the ZED as the local position source
for PX4 arming. The goal is:

1. Run the ZED wrapper.
2. Convert ZED pose into `/tardigrade/state/odometry`.
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
chmod +x docker/run_jetson_zed.sh
sudo docker build -t tardigrade-foxy -f docker/Dockerfile .
```

If the ZED wrapper source is missing, add the same wrapper/tag that worked with
the legacy workspace:

```bash
cd ~/Developer/tardigrade_ws/src
git clone --branch humble-v4.0.8 https://github.com/stereolabs/zed-ros2-wrapper.git
git clone --branch humble-v4.0.8 https://github.com/stereolabs/zed-ros2-interfaces.git
```

The Jetson host must have the ZED SDK installed at `/usr/local/zed`. Check it:

```bash
sudo grep -n "PACKAGE_VERSION" /usr/local/zed/zed-config-version.cmake
```

The known-good target for this setup is ZED SDK `4.0.8`.

## Open The Container

Start the main container from the Jetson host:

```bash
cd ~/Developer/tardigrade_ws
sudo ./docker/run_jetson_zed.sh
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

## Terminal Layout For Arming Test

Use five terminals. Terminal 1 starts the container. Terminals 2-5 enter the
same running container with:

```bash
sudo docker exec -it tardigrade-foxy bash
```

### Terminal 1: Launch ZED

Host:

```bash
cd ~/Developer/tardigrade_ws
sudo ./docker/run_jetson_zed.sh
```

Inside container:

```bash
cd /ws
source install/setup.bash
lsusb -t
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed ros_params_override_path:=/ws/config/zed_low_load.yaml
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

### Terminal 2: Verify ZED Pose

Inside container:

```bash
cd /ws
source install/setup.bash
ros2 topic hz /zed/zed_node/pose
```

Good output:

```text
average rate: ...
```

Leave it running long enough to confirm the ZED is stable.

### Terminal 3: Bridge ZED Pose To Robot Odometry

Inside container:

```bash
cd /ws
source install/setup.bash
ros2 launch tardigrade_bringup zed_state.launch.py
```

This converts:

```text
/zed/zed_node/pose
```

into:

```text
/tardigrade/state/odometry
```

### Terminal 4: Start Pixhawk MAVLink Interface

Inside container:

```bash
cd /ws
source install/setup.bash
ros2 run tardigrade_px4 mavlink_pixhawk_interface --ros-args -p device:=/dev/ttyACM0 -p baudrate:=921600
```

This node:

- opens the Pixhawk USB MAVLink connection,
- publishes `/tardigrade/status`,
- provides `/tardigrade/set_armed`,
- provides `/tardigrade/set_external_control`,
- sends `/tardigrade/state/odometry` to PX4 as MAVLink visual odometry.

### Terminal 5: Verify And Arm

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
armed: true
```

If it stays false and shows:

```text
last_ack_command=400
last_ack_result=1
```

PX4 received the arm command but rejected it. Check QGroundControl's current
arming blockers. Do not debug ROS until QGC shows only blockers related to local
position/vision quality.

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

Run `zed_state.launch.py` and confirm `/zed/zed_node/pose` is active.

```text
px4_connected: false
```

Check Pixhawk USB device path. It is usually `/dev/ttyACM0`.

```text
last_ack_result=1 and armed remains false
```

PX4 rejected arming due to preflight checks. Read QGroundControl arming errors.
