# Setup

This is the short setup path for the Tardigrade ROS 2 workspace. The detailed
Jetson/ESP/ZED runbook lives in `docs/esp_thruster_bringup.md`.

On the Jetson, the Docker override mounts `/dev` and `/dev/bus/usb`, so ESP,
VectorNav, and other USB serial devices should appear inside the container at
the same paths as the host:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-id/
```

## Clone

```bash
git clone --recurse-submodules https://github.com/berkeleyauv/tardigrade_ws.git
cd tardigrade_ws
```

If the repo was cloned without submodules:

```bash
git submodule update --init --recursive
```

## Docker Development

Build and start the dev container:

```bash
./docker-build.sh --build
```

Start the dev container after the image already exists:

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

## Jetson Hardware Container

From the Jetson host:

```bash
cd ~/Developer/tardigrade_ws
sudo env WORKSPACE=/home/auv/Developer/tardigrade_ws \
  ./docker-build.sh --jetson
```

The Jetson override enables host networking, device access, CUDA/Tegra mounts,
and the ZED SDK path. Do not use it for normal laptop development.

## Local Mock Bringup

After building and sourcing:

```bash
ros2 launch tardigrade_bringup mock.launch.py
```

## Foxglove

For ROS 2 Foxy, use rosbridge first. The container exposes rosbridge's default
port, `9090`.

Start rosbridge inside the container. Keep this separate from sensor/state
launch files so visualization can be restarted without touching the robot data
sources:

```bash
ros2 launch tardigrade_bringup foxglove_rosbridge.launch.py
```

Connect Foxglove Studio or the Foxglove web app with the Rosbridge connection
option:

```text
ws://localhost:9090
```

On the Jetson, use the Jetson's IP address instead of `localhost`.

For local Docker use, start the container with `./docker-build.sh`. It passes
Compose's `--service-ports` flag so the container's rosbridge port is reachable
from the Mac host.

For Jetson hardware Docker, the container uses host networking. In that mode,
`docker ps` will not show `9090->9090/tcp`; rosbridge listens directly on the
Jetson's network. Connect from your Mac with:

```text
ws://JETSON_IP:9090
```

Foxglove's preferred `foxglove_bridge` can be revisited later. ROS 2 Foxy does
not provide `ros-foxy-foxglove-bridge` in the standard package index, so that
path requires a source build.

If `foxglove_bridge` is built from source, the launch command is:

```bash
ros2 launch tardigrade_bringup foxglove_bridge.launch.py
```

## Jetson ZED + VectorNav

Start the ZED wrapper first:

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed
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

This is currently a simple combined estimate:

```text
position          ZED
orientation       VectorNav when fresh, otherwise ZED fallback
angular velocity  VectorNav
linear velocity   not estimated
```

## Experimental ZED + VectorNav EKF

The first `robot_localization` EKF path is separate from the current
`zed_vectornav_state.launch.py` path. Start the sensors first:

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed
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
/vectornav/imu
```

Default output:

```text
/tardigrade/state/odometry/filtered
```

The filtered topic is intentionally not the main `/tardigrade/state/odometry`
topic yet. Compare both in Foxglove first, then promote the EKF output once it
looks stable. The starter config fuses ZED position/linear velocity with
VectorNav orientation/angular velocity and does not fuse IMU linear
acceleration yet.

The VectorNav is mounted in FRD orientation by default:

```text
VectorNav +X  robot forward
VectorNav +Y  robot right
VectorNav +Z  robot down
```

ROS `base_link` remains FLU:

```text
base_link +X  robot forward
base_link +Y  robot left
base_link +Z  robot up
```

`zed_vectornav_ekf.launch.py` publishes a default `base_link -> vectornav`
static transform with quaternion `(x=1, y=0, z=0, w=0)`, which is a 180 degree
rotation about X. Override `vectornav_x`, `vectornav_y`, and `vectornav_z` once
the sensor's physical offset from the robot origin is measured.

## Common Checks

```bash
colcon test --packages-select tardigrade_interfaces tardigrade_state_estimation tardigrade_esp tardigrade_teleop tardigrade_bringup
```

If build state gets stale:

```bash
./build.sh --clean
source install/setup.bash
```
