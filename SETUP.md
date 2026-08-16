# Set Up the Repository and Development Environment

This document walks through setup for local laptop development and Jetson robot
work. Detailed hardware procedures live in the runbooks under `docs/`.

## Steps

1. [Prerequisites](#prerequisites)
2. [Clone The Repository](#clone-the-repository)
3. [Set Up The Docker Container](#set-up-the-docker-container)
4. [Set Up The Robot Host](#set-up-the-robot-host)
5. [Set Up Foxglove](#set-up-foxglove)
6. [Common Checks](#common-checks)

## Prerequisites

Install:

- Docker
- Git
- Visual Studio Code, optional
- Foxglove Studio or Foxglove web app, optional

On macOS or ordinary laptop development, use the base Docker container only. Do
not use the Jetson hardware override on a laptop.

## Clone The Repository

```bash
git clone --recurse-submodules https://github.com/berkeleyauv/tardigrade_ws.git
cd tardigrade_ws
```

If the repo was cloned without submodules:

```bash
git submodule update --init --recursive
```

## Set Up The Docker Container

Build and start the development container:

```bash
./docker-build.sh --build
```

Start the container after the image already exists:

```bash
./docker-build.sh
```

Inside the container:

```bash
cd /ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
./build.sh
source install/setup.bash
```

The skipped ZED packages require the Stereolabs ZED SDK. Build them on the
Jetson or another machine with the ZED SDK installed.

If build state gets stale:

```bash
./build.sh --clean
source install/setup.bash
```

## Set Up The Robot Host

Robot-host setup files live in:

```text
robot/
```

Use [robot/README.md](robot/README.md) for:

- udev rules for stable `/dev/tardigrade_*` device names,
- autostart env files and systemd units,
- Jetson host notes.

From the Jetson host, start the hardware container with:

```bash
cd ~/Developer/tardigrade_ws
sudo env WORKSPACE=/home/auv/Developer/tardigrade_ws \
  ./docker-build.sh --jetson
```

The Jetson override enables host networking, device access, CUDA/Tegra mounts,
and the ZED SDK path. ESP, VectorNav, and other USB serial devices should appear
inside the container at the same paths as the host:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-id/
```

Prefer stable `/dev/serial/by-id/...` paths or verified udev symlinks in launch
arguments and `/etc/tardigrade/*.env`.

## Set Up Foxglove

For ROS 2 Foxy, use rosbridge first. The container exposes rosbridge's default
port, `9090`.

Start rosbridge inside the container:

```bash
ros2 launch tardigrade_bringup foxglove_rosbridge.launch.py
```

Connect Foxglove Studio or the Foxglove web app with:

```text
ws://localhost:9090
```

On the Jetson, use:

```text
ws://JETSON_IP:9090
```

See [foxglove/README.md](foxglove/README.md) for more details.

## Local Mock Bringup

After building and sourcing:

```bash
ros2 launch tardigrade_sim local_sim.launch.py
```

## Jetson ZED + VectorNav

Start the ZED wrapper first:

```bash
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zed publish_tf:=false
```

Start VectorNav plus the ZED/VectorNav odometry node separately:

```bash
ros2 launch tardigrade_bringup zed_vectornav_state.launch.py \
  port:=/dev/serial/by-id/usb-FTDI_USB-RS232-WE_AV0LN035-if00-port0 \
  baud:=115200
```

The output topic is:

```text
/tardigrade/state/odometry
```

## ZED + VectorNav EKF

Start the sensors first:

```bash
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zed publish_tf:=false
ros2 launch tardigrade_bringup vectornav_state.launch.py \
  port:=/dev/serial/by-id/usb-FTDI_USB-RS232-WE_AV0LN035-if00-port0 \
  baud:=115200
```

Then start the EKF:

```bash
ros2 launch tardigrade_bringup zed_vectornav_ekf.launch.py
```

Default inputs:

```text
/zed/zed_node/odom
/tardigrade/sensors/imu
```

Default output:

```text
/tardigrade/state/odometry/filtered
```

Assisted control consumes the filtered topic. Compare it with the simpler
`/tardigrade/state/odometry` topic in Foxglove during sensor acceptance.

## Common Checks

```bash
colcon test --packages-select \
  tardigrade_interfaces \
  tardigrade_state_estimation \
  tardigrade_esp \
  tardigrade_teleop \
  tardigrade_bringup \
  tardigrade_mission \
  tardigrade_sim
colcon test-result --verbose
```
